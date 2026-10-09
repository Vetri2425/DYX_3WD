#include <fcntl.h>
#include <gtest/gtest.h>
#include <pty.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <future>
#include <mutex>
#include <random>
#include <thread>

#include "dyx3_gnss_rtk/control_socket.hpp"
#include "dyx3_gnss_rtk/gga_provider.hpp"
#include "dyx3_gnss_rtk/injection_authority.hpp"
#include "dyx3_gnss_rtk/lora_source.hpp"
#include "dyx3_gnss_rtk/rtcm_parser.hpp"
#include "dyx3_gnss_rtk/rtk_config.hpp"
#include "dyx3_gnss_rtk/serial_port.hpp"
#include "dyx3_gnss_rtk/transport_sink.hpp"

using namespace dyx3_gnss_rtk;
using namespace std::chrono_literals;

namespace {

std::vector<uint8_t> frame(uint8_t payload = 0x42) {
  std::vector<uint8_t> result{0xD3, 0x00, 0x01, payload};
  const uint32_t crc = crc24q(result.data(), result.size());
  result.push_back(static_cast<uint8_t>(crc >> 16));
  result.push_back(static_cast<uint8_t>(crc >> 8));
  result.push_back(static_cast<uint8_t>(crc));
  return result;
}

ValidatedFrame valid(uint8_t payload = 0x42) { return *ValidatedFrame::parse(frame(payload)); }

struct Pty {
  int master{-1};
  int slave{-1};
  std::string slave_name;
  Pty() {
    char name[128]{};
    EXPECT_EQ(::openpty(&master, &slave, name, nullptr, nullptr), 0);
    slave_name = name;
  }
  ~Pty() {
    if (master >= 0) ::close(master);
    if (slave >= 0) ::close(slave);
  }
};

struct SerialPath {
  std::string directory;
  std::string prefix;
  std::string path;
  explicit SerialPath(const std::string& slave) {
    char pattern[] = "/tmp/dyx3-rtk-XXXXXX";
    directory = ::mkdtemp(pattern);
    prefix = directory + "/";
    path = prefix + "radio-by-id";
    EXPECT_EQ(::symlink(slave.c_str(), path.c_str()), 0);
  }
  ~SerialPath() { std::filesystem::remove_all(directory); }
  void retarget(const std::string& slave) {
    ::unlink(path.c_str());
    ASSERT_EQ(::symlink(slave.c_str(), path.c_str()), 0);
  }
};

class FakeSink : public TransportSink {
public:
  bool opened{false};
  bool available{true};
  std::vector<uint8_t> received;
  bool open(double) override { return opened = available; }
  bool deliver(const ValidatedFrame& f, double) override {
    if (!opened) open(0);
    if (!opened || !available) return false;
    received.push_back(f.bytes()[3]);
    return true;
  }
  void close() override { opened = false; }
  bool active() const override { return opened; }
  SinkCounters counters() const override { return {}; }
};

}  // namespace

TEST(ValidatedFrame, RejectsEverythingExceptCompleteCrcValidRtcm) {
  EXPECT_FALSE(ValidatedFrame::parse(std::vector<uint8_t>{'C', 'O', 'N', 'F', 'I', 'G'}));
  auto bytes = frame();
  EXPECT_TRUE(ValidatedFrame::parse(bytes));
  bytes.back() ^= 1;
  EXPECT_FALSE(ValidatedFrame::parse(bytes));
  bytes = frame();
  bytes.pop_back();
  EXPECT_FALSE(ValidatedFrame::parse(bytes));
}

TEST(SerialPort, OnlyStableByIdPathsAndSupportedBauds) {
  EXPECT_FALSE(stable_serial_path("/dev/ttyACM0"));
  EXPECT_FALSE(stable_serial_path("/dev/serial/by-id/../ttyACM0"));
  EXPECT_FALSE(stable_serial_path("/dev/serial/by-id/x/y"));
  EXPECT_TRUE(stable_serial_path("/dev/serial/by-id/usb-Receiver_123"));
  // Serial-number-less CH340: physical-port identity under by-path (contains ':').
  EXPECT_TRUE(stable_serial_path("/dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0-port0"));
  EXPECT_FALSE(stable_serial_path("/dev/serial/by-path/../ttyUSB0"));
  EXPECT_FALSE(stable_serial_path("/dev/serial/by-path/a/b"));
  EXPECT_FALSE(stable_serial_path("/dev/serial/by-id/usb-0:2.1"));  // ':' only under by-path
  EXPECT_FALSE(stable_serial_path("/dev/serial/by-path/x", "/tmp/injected/"));
  EXPECT_TRUE(supported_serial_baud(115200));
  EXPECT_FALSE(supported_serial_baud(123456));
}

TEST(UsbSerialSink, WritesOnlyValidatedFramesByteExactly) {
  Pty pty;
  SerialPath path(pty.slave_name);
  UsbSerialSink sink({path.path, 115200, 0.2, 0.01}, path.prefix);
  EXPECT_TRUE(sink.open(10));
  EXPECT_TRUE(sink.deliver(valid(), 10));
  uint8_t actual[32]{};
  const ssize_t n = ::read(pty.master, actual, sizeof actual);
  const auto expected = frame();
  ASSERT_EQ(n, static_cast<ssize_t>(expected.size()));
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), actual));
  EXPECT_EQ(sink.counters().bytes_delivered, expected.size());
  sink.close();
  EXPECT_FALSE(sink.active());
}

TEST(UsbSerialSink, ReadsGgaWithoutSendingReceiverCommands) {
  Pty pty;
  SerialPath path(pty.slave_name);
  UsbSerialSink sink({path.path, 115200, 0.2, 0.01}, path.prefix);
  ASSERT_TRUE(sink.open(10));
  const std::string body =
      "GPGGA,123456.00,4800.0000,N,01100.0000,E,4,21,0.8,500.0,M,0.0,M,1.2,0001";
  const std::string line = "$" + body + "*" + nmea_checksum(body) + "\r\n";
  ASSERT_EQ(::write(pty.master, line.data(), line.size()), static_cast<ssize_t>(line.size()));
  sink.read_available(10.1);
  const auto info = sink.readback(10.2, 1.0);
  ASSERT_TRUE(info);
  EXPECT_EQ(info->fix_quality, 4);
  EXPECT_EQ(info->satellites, 21);
  ASSERT_TRUE(info->correction_age_s);
  EXPECT_NEAR(*info->correction_age_s, 1.2, 1e-6);
  EXPECT_EQ(sink.last_gga(10.2, 1.0), line);
  EXPECT_FALSE(sink.readback(12, 1.0));
  EXPECT_EQ(sink.counters().bytes_delivered, 0U);  // no command went to the receiver
  EXPECT_TRUE(sink.active());                      // an empty nonblocking read is not an unplug
}

TEST(UsbSerialSink, RetriesSelectedPortAfterUnplugWithoutReplayingOldFrame) {
  Pty first;
  SerialPath path(first.slave_name);
  UsbSerialSink sink({path.path, 115200, 0.2, 0.01}, path.prefix);
  ASSERT_TRUE(sink.deliver(valid(1), 10));
  uint8_t received[32]{};
  ASSERT_EQ(::read(first.master, received, sizeof received), static_cast<ssize_t>(frame(1).size()));
  ::close(first.master);
  first.master = -1;
  sink.read_available(10.1);
  EXPECT_FALSE(sink.active());
  EXPECT_FALSE(sink.deliver(valid(2), 10.105));
  Pty second;
  path.retarget(second.slave_name);
  EXPECT_TRUE(sink.deliver(valid(3), 10.2));
  const auto expected = frame(3);
  ASSERT_EQ(::read(second.master, received, sizeof received),
            static_cast<ssize_t>(expected.size()));
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), received));
  EXPECT_EQ(sink.counters().frames_delivered, 2U);
  EXPECT_EQ(sink.counters().reopens, 1U);
}

TEST(LoRaSource, ParsesFramesAndReopensAfterUnplug) {
  Pty first;
  SerialPath path(first.slave_name);
  std::atomic<int> received{0};
  LoraSource source(
      {path.path, 115200, 0.1, 0.01},
      [&](const std::vector<uint8_t>& bytes) {
        if (bytes == frame()) ++received;
      },
      path.prefix);
  source.start();
  for (int i = 0; i < 100 && !source.snapshot().port_open; ++i) std::this_thread::sleep_for(10ms);
  ASSERT_TRUE(source.snapshot().port_open);
  const auto valid_bytes = frame();
  ASSERT_EQ(::write(first.master, valid_bytes.data(), valid_bytes.size()),
            static_cast<ssize_t>(valid_bytes.size()));
  for (int i = 0; i < 100 && received.load() < 1; ++i) std::this_thread::sleep_for(10ms);
  EXPECT_EQ(received.load(), 1);
  ::close(first.master);
  first.master = -1;
  Pty second;
  path.retarget(second.slave_name);
  for (int i = 0; i < 100 && source.snapshot().reopens == 0; ++i) std::this_thread::sleep_for(10ms);
  EXPECT_GT(source.snapshot().reopens, 0U);
  ASSERT_EQ(::write(second.master, valid_bytes.data(), valid_bytes.size()),
            static_cast<ssize_t>(valid_bytes.size()));
  for (int i = 0; i < 100 && received.load() < 2; ++i) std::this_thread::sleep_for(10ms);
  EXPECT_EQ(received.load(), 2);
  source.stop();
  EXPECT_FALSE(source.snapshot().port_open);
}

TEST(InjectionAuthority, FourCombinationsAndNoStaleReplay) {
  FakeSink usb, dds;
  InjectionAuthority authority(usb, dds);
  for (auto source : {CorrectionSource::Ntrip, CorrectionSource::Lora}) {
    for (auto transport : {CorrectionTransport::UsbDirect, CorrectionTransport::Px4Dds}) {
      const uint64_t old = authority.generation();
      const uint64_t next = authority.switch_to(source, transport, 10);
      EXPECT_NE(old, next);
      EXPECT_FALSE(authority.deliver(old, valid(1), 10));
      EXPECT_TRUE(authority.deliver(next, valid(2), 10));
      EXPECT_FALSE(usb.opened && dds.opened);
    }
  }
  authority.stop();
  EXPECT_FALSE(usb.opened || dds.opened);
}

TEST(InjectionAuthority, FailedSelectedTransportNeverFallsBack) {
  FakeSink usb, dds;
  InjectionAuthority authority(usb, dds);
  usb.available = false;
  const uint64_t epoch =
      authority.switch_to(CorrectionSource::Lora, CorrectionTransport::UsbDirect, 10);
  EXPECT_FALSE(authority.deliver(epoch, valid(), 10));
  EXPECT_TRUE(dds.received.empty());
  usb.available = true;
  EXPECT_TRUE(authority.deliver(epoch, valid(), 11));
  EXPECT_TRUE(dds.received.empty());
}

TEST(InjectionAuthority, DdsRecoversAfterLinkRestartAndSubscriberRematchWithoutReplay) {
  FakeSink usb;
  bool status_fresh = false;
  bool session_alive = false;
  bool handshake_ok = false;
  bool subscriber_matched = false;
  std::vector<uint8_t> published;
  DdsSink dds(
      [&](const Chunk& chunk) {
        published.push_back(chunk.data[3]);
        return true;
      },
      [&] { return status_fresh && session_alive && handshake_ok && subscriber_matched; });
  InjectionAuthority authority(usb, dds);
  const auto generation =
      authority.switch_to(CorrectionSource::Ntrip, CorrectionTransport::Px4Dds, 10);

  // The previous px4_link exits, its status ages out, and its subscriber disappears.
  EXPECT_FALSE(authority.deliver(generation, valid(1), 10));
  status_fresh = true;
  session_alive = true;
  EXPECT_FALSE(authority.deliver(generation, valid(2), 11));  // handshake still pending
  handshake_ok = true;
  EXPECT_FALSE(authority.deliver(generation, valid(3), 12));  // discovery still pending
  subscriber_matched = true;
  EXPECT_TRUE(authority.deliver(generation, valid(4), 13));
  EXPECT_EQ(published, (std::vector<uint8_t>{4}));
  EXPECT_TRUE(usb.received.empty());

  // A second restart interrupts an already active sink. It reopens on a new frame only.
  status_fresh = false;
  subscriber_matched = false;
  EXPECT_FALSE(authority.deliver(generation, valid(5), 14));
  EXPECT_FALSE(authority.active());
  status_fresh = true;
  EXPECT_FALSE(authority.deliver(generation, valid(6), 15));
  subscriber_matched = true;
  EXPECT_TRUE(authority.deliver(generation, valid(7), 16));
  EXPECT_EQ(published, (std::vector<uint8_t>{4, 7}));
  EXPECT_EQ(dds.counters().frames_delivered, 2U);
  EXPECT_TRUE(usb.received.empty());

  authority.stop();
  EXPECT_FALSE(authority.deliver(generation, valid(8), 17));
  EXPECT_EQ(published, (std::vector<uint8_t>{4, 7}));
}

TEST(InjectionAuthority, SwitchWaitsForInFlightPartialWriteThenClosesOldSink) {
  class PausingSink final : public FakeSink {
  public:
    std::mutex gate;
    std::condition_variable cv;
    bool half_written{false};
    bool finish{false};
    bool deliver(const ValidatedFrame& f, double now) override {
      {
        std::unique_lock<std::mutex> lk(gate);
        half_written = true;
        cv.notify_all();
        cv.wait(lk, [this] { return finish; });
      }
      return FakeSink::deliver(f, now);
    }
  } usb;
  FakeSink dds;
  InjectionAuthority authority(usb, dds);
  const auto old = authority.switch_to(CorrectionSource::Ntrip, CorrectionTransport::UsbDirect, 10);
  auto write = std::async(std::launch::async, [&] { return authority.deliver(old, valid(1), 10); });
  {
    std::unique_lock<std::mutex> lk(usb.gate);
    ASSERT_TRUE(usb.cv.wait_for(lk, 1s, [&] { return usb.half_written; }));
  }
  auto change = std::async(std::launch::async, [&] {
    return authority.switch_to(CorrectionSource::Lora, CorrectionTransport::Px4Dds, 11);
  });
  EXPECT_EQ(change.wait_for(20ms), std::future_status::timeout);
  EXPECT_FALSE(dds.opened);
  {
    std::lock_guard<std::mutex> lk(usb.gate);
    usb.finish = true;
  }
  usb.cv.notify_all();
  EXPECT_TRUE(write.get());
  const auto next = change.get();
  EXPECT_FALSE(usb.opened);
  EXPECT_TRUE(dds.opened);
  EXPECT_FALSE(authority.deliver(old, valid(2), 12));
  EXPECT_TRUE(authority.deliver(next, valid(3), 12));
}

TEST(InjectionAuthority, RandomSwitchesNeverActivateBothOrDeliverOldFrames) {
  FakeSink usb, dds;
  InjectionAuthority authority(usb, dds);
  std::mt19937 rng(17);
  uint64_t epoch = 0;
  for (int i = 0; i < 1000; ++i) {
    if (rng() % 3 == 0) {
      const auto source = rng() % 2 ? CorrectionSource::Ntrip : CorrectionSource::Lora;
      const auto transport =
          rng() % 2 ? CorrectionTransport::UsbDirect : CorrectionTransport::Px4Dds;
      const uint64_t old = epoch;
      epoch = authority.switch_to(source, transport, i);
      EXPECT_FALSE(authority.deliver(old, valid(0x10), i));
    } else if (epoch) {
      EXPECT_TRUE(authority.deliver(epoch, valid(0x20), i));
    }
    EXPECT_FALSE(usb.opened && dds.opened);
  }
}

TEST(RtkConfigStore, FreshAndUpgradeDefaultsAndWriteOnlyPassword) {
  const auto fresh = RtkConfigStore::initial_from_environment();
  EXPECT_EQ(fresh.at("transport"), "USB_DIRECT");
  EXPECT_EQ(fresh.at("source"), "NTRIP");
  EXPECT_TRUE(fresh.at("ntrip").at("profiles").empty());
  ::setenv("DYX3_NTRIP_HOST", "caster.example", 1);
  ::setenv("DYX3_NTRIP_USER", "worker", 1);
  ::setenv("DYX3_NTRIP_PASSWORD", "do-not-return", 1);
  ::setenv("DYX3_NTRIP_MOUNTPOINT", "MOUNT", 1);
  ::setenv("DYX3_NTRIP_SECURITY", "TLS", 1);
  const auto upgrade = RtkConfigStore::initial_from_environment();
  EXPECT_EQ(upgrade.at("transport"), "PX4_DDS");
  EXPECT_EQ(RtkConfigStore::public_view(upgrade).dump().find("do-not-return"), std::string::npos);
  ::unsetenv("DYX3_NTRIP_HOST");
  ::unsetenv("DYX3_NTRIP_USER");
  ::unsetenv("DYX3_NTRIP_PASSWORD");
  ::unsetenv("DYX3_NTRIP_MOUNTPOINT");
  ::unsetenv("DYX3_NTRIP_SECURITY");
}

TEST(RtkConfigStore, FreshRoverUsesInstallerRecordedUsbReceiver) {
  const std::string dev = "/dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0-port0";
  ::setenv("DYX3_USB_RECEIVER_DEVICE", dev.c_str(), 1);
  ::setenv("DYX3_USB_RECEIVER_BAUD", "230400", 1);
  auto fresh = RtkConfigStore::initial_from_environment();
  EXPECT_EQ(fresh.at("transport"), "USB_DIRECT");
  EXPECT_EQ(fresh.at("usb").at("receiver_device"), dev);
  EXPECT_EQ(fresh.at("usb").at("baud"), 230400);
  EXPECT_DOUBLE_EQ(fresh.at("usb").at("write_timeout_s").get<double>(), 0.2);
  EXPECT_DOUBLE_EQ(fresh.at("usb").at("reopen_delay_s").get<double>(), 2.0);
  EXPECT_NO_THROW(RtkConfigStore::validate(fresh));
  // A seeded caster no longer forces DDS when the receiver USB identity is recorded.
  ::setenv("DYX3_NTRIP_HOST", "caster.example", 1);
  ::setenv("DYX3_NTRIP_USER", "worker", 1);
  ::setenv("DYX3_NTRIP_PASSWORD", "secret", 1);
  ::setenv("DYX3_NTRIP_MOUNTPOINT", "MOUNT", 1);
  ::setenv("DYX3_NTRIP_SECURITY", "PLAINTEXT", 1);
  EXPECT_EQ(RtkConfigStore::initial_from_environment().at("transport"), "USB_DIRECT");
  // Invalid recorded identities fail closed instead of selecting a guessed port.
  ::setenv("DYX3_USB_RECEIVER_DEVICE", "/dev/ttyUSB0", 1);
  EXPECT_THROW(RtkConfigStore::initial_from_environment(), ConfigError);
  ::setenv("DYX3_USB_RECEIVER_DEVICE", dev.c_str(), 1);
  ::setenv("DYX3_USB_RECEIVER_BAUD", "123456", 1);
  EXPECT_THROW(RtkConfigStore::initial_from_environment(), ConfigError);
  for (const char* k :
       {"DYX3_USB_RECEIVER_DEVICE", "DYX3_USB_RECEIVER_BAUD", "DYX3_NTRIP_HOST", "DYX3_NTRIP_USER",
        "DYX3_NTRIP_PASSWORD", "DYX3_NTRIP_MOUNTPOINT", "DYX3_NTRIP_SECURITY"})
    ::unsetenv(k);
}

TEST(RtkConfigStore, InterruptedSaveLeavesOldConfigIntact) {
  char pattern[] = "/tmp/dyx3-config-XXXXXX";
  const std::string directory = ::mkdtemp(pattern);
  RtkConfigStore store(directory);
  auto old = RtkConfigStore::initial_from_environment();
  store.save(old);
  auto next = old;
  next["revision"] = 2;
  EXPECT_THROW(store.save(next, [] { throw std::runtime_error("interrupt"); }), std::runtime_error);
  ASSERT_TRUE(store.load());
  EXPECT_EQ(store.load()->at("revision"), 1);
  EXPECT_EQ(std::distance(std::filesystem::directory_iterator(directory),
                          std::filesystem::directory_iterator{}),
            1);
  struct stat st{};
  ASSERT_EQ(::stat((directory + "/config.json").c_str(), &st), 0);
  EXPECT_EQ(st.st_mode & 0777, 0600);
  std::filesystem::remove_all(directory);
}

TEST(ControlSocket, VersionedNewlineJsonAndNoSecretInReadReply) {
  char pattern[] = "/tmp/dyx3-control-XXXXXX";
  const std::string directory = ::mkdtemp(pattern);
  const std::string path = directory + "/rtk.sock";
  ControlSocket server(path, [](const Json& request) -> Json {
    if (request.at("cmd") == "GET_CONFIG")
      return {{"v", 1}, {"ok", true}, {"data", {{"password_set", true}}}};
    return {{"v", 1}, {"ok", false}, {"code", "unknown_command"}};
  });
  server.start();
  const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
  ASSERT_GE(fd, 0);
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::strncpy(address.sun_path, path.c_str(), sizeof address.sun_path - 1);
  ASSERT_EQ(::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof address), 0);
  const std::string request = "{\"v\":1,\"cmd\":\"GET_CONFIG\"}\n";
  ASSERT_EQ(::write(fd, request.data(), request.size()), static_cast<ssize_t>(request.size()));
  char response[256]{};
  const ssize_t count = ::read(fd, response, sizeof response);
  ASSERT_GT(count, 0);
  const auto result = Json::parse(std::string(response, static_cast<size_t>(count)));
  EXPECT_EQ(result["data"]["password_set"], true);
  EXPECT_EQ(result.dump().find("secret"), std::string::npos);
  ::close(fd);
  server.stop();
  std::filesystem::remove_all(directory);
}
