// NTRIP client against a loopback fake caster (real sockets, real threads).
#include "dyx3_gnss_rtk/ntrip_client.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

using namespace dyx3_gnss_rtk;
using namespace std::chrono_literals;

namespace {

uint32_t crc_bitwise(const std::vector<uint8_t>& d, size_t n) {
  uint32_t crc = 0;
  for (size_t i = 0; i < n; ++i) {
    crc ^= static_cast<uint32_t>(d[i]) << 16;
    for (int k = 0; k < 8; ++k) {
      crc <<= 1;
      if (crc & 0x1000000U) crc ^= 0x1864CFBU;
    }
  }
  return crc & 0xFFFFFFU;
}
std::vector<uint8_t> make_frame(size_t payload, uint8_t seed) {
  std::vector<uint8_t> f = {0xD3, static_cast<uint8_t>((payload >> 8) & 3),
                            static_cast<uint8_t>(payload & 0xFF)};
  for (size_t i = 0; i < payload; ++i) f.push_back(static_cast<uint8_t>(seed * 13U + i));
  const uint32_t crc = crc_bitwise(f, f.size());
  f.push_back(static_cast<uint8_t>(crc >> 16));
  f.push_back(static_cast<uint8_t>(crc >> 8));
  f.push_back(static_cast<uint8_t>(crc));
  return f;
}

struct Script {
  std::string header = "ICY 200 OK\r\n";
  std::vector<uint8_t> payload;  // sent right after the header
  int repeat_payload = 0;        // extra copies sent 20 ms apart
  bool close_after = false;      // close right after the payload (stream ended)
  bool hold_open = true;         // otherwise stay silent until the client leaves
};

class FakeCaster {
public:
  FakeCaster() {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = 0;
    EXPECT_EQ(::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof a), 0);
    socklen_t len = sizeof a;
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&a), &len);
    port_ = ntohs(a.sin_port);
    ::listen(fd_, 4);
    thread_ = std::thread([this] { serve(); });
  }
  ~FakeCaster() {
    stop_ = true;
    ::shutdown(fd_, SHUT_RDWR);
    ::close(fd_);
    if (thread_.joinable()) thread_.join();
  }
  int port() const { return port_; }
  void push_script(Script s) {
    std::lock_guard<std::mutex> lk(m_);
    scripts_.push_back(std::move(s));
  }
  int connections() const { return connections_; }
  std::string last_request() const {
    std::lock_guard<std::mutex> lk(m_);
    return last_request_;
  }
  std::string gga_received() const {
    std::lock_guard<std::mutex> lk(m_);
    return gga_;
  }

private:
  void serve() {
    while (!stop_) {
      pollfd p{fd_, POLLIN, 0};
      if (::poll(&p, 1, 50) <= 0) continue;
      const int c = ::accept(fd_, nullptr, nullptr);
      if (c < 0) continue;
      ++connections_;
      Script s;
      {
        std::lock_guard<std::mutex> lk(m_);
        if (!scripts_.empty()) {
          s = scripts_.front();
          scripts_.erase(scripts_.begin());
        }
      }
      std::string req;
      char buf[1024];
      while (req.find("\r\n\r\n") == std::string::npos) {
        const ssize_t n = ::recv(c, buf, sizeof buf, 0);
        if (n <= 0) break;
        req.append(buf, static_cast<size_t>(n));
      }
      {
        std::lock_guard<std::mutex> lk(m_);
        last_request_ = req;
      }
      ::send(c, s.header.data(), s.header.size(), MSG_NOSIGNAL);
      if (!s.payload.empty()) ::send(c, s.payload.data(), s.payload.size(), MSG_NOSIGNAL);
      for (int k = 0; k < s.repeat_payload && !stop_; ++k) {
        std::this_thread::sleep_for(20ms);
        ::send(c, s.payload.data(), s.payload.size(), MSG_NOSIGNAL);
      }
      if (!s.close_after) {
        // stay open, collecting whatever the client sends (GGA), until it hangs up or we stop
        while (!stop_) {
          pollfd q{c, POLLIN, 0};
          if (::poll(&q, 1, 50) <= 0) continue;
          const ssize_t n = ::recv(c, buf, sizeof buf, 0);
          if (n <= 0) break;
          std::lock_guard<std::mutex> lk(m_);
          gga_.append(buf, static_cast<size_t>(n));
        }
      }
      ::close(c);
    }
  }
  int fd_{-1};
  int port_{0};
  std::atomic<bool> stop_{false};
  std::atomic<int> connections_{0};
  std::thread thread_;
  mutable std::mutex m_;
  std::vector<Script> scripts_;
  std::string last_request_;
  std::string gga_;
};

NtripConfig cfg_for(int port) {
  NtripConfig c;
  c.host = "127.0.0.1";
  c.port = port;
  c.mountpoint = "MOUNT";
  c.user = "rover";
  c.password = "s3cr3t-pw";
  c.connect_timeout_s = 2.0;
  c.stream_timeout_s = 0.5;
  c.gga_interval_s = 0.2;
  c.backoff_base_s = 0.05;
  c.backoff_max_s = 0.2;
  return c;
}

template <typename F>
bool wait_for(F pred, double seconds = 5.0) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < end) {
    if (pred()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return pred();
}

}  // namespace

TEST(NtripClient, StreamsFramesFromAnIcyCasterWithTheExpectedRequest) {
  FakeCaster caster;
  Script s;
  const auto f1 = make_frame(40, 1), f2 = make_frame(120, 2);
  s.payload = f1;
  s.payload.insert(s.payload.end(), f2.begin(), f2.end());
  caster.push_script(s);
  std::mutex m;
  std::vector<std::vector<uint8_t>> frames;
  NtripClient c(
      cfg_for(caster.port()),
      [&](const std::vector<uint8_t>& f) {
        std::lock_guard<std::mutex> lk(m);
        frames.push_back(f);
      },
      [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().frames >= 2; }));
  {
    std::lock_guard<std::mutex> lk(m);
    ASSERT_GE(frames.size(), 2U);
    EXPECT_EQ(frames[0], f1);
    EXPECT_EQ(frames[1], f2);
  }
  EXPECT_EQ(caster.last_request(), build_request("127.0.0.1", "MOUNT", "rover", "s3cr3t-pw"));
  const auto snap = c.snapshot();
  EXPECT_EQ(snap.state, NtripState::Streaming);
  EXPECT_TRUE(snap.connected);
  EXPECT_EQ(snap.bytes, s.payload.size());
  EXPECT_EQ(snap.frames, 2U);
  c.stop();
  EXPECT_EQ(c.snapshot().state, NtripState::Stopped);
}

TEST(NtripClient, HttpHeaderAndFramesInOneSegment) {
  FakeCaster caster;
  Script s;
  s.header = "HTTP/1.1 200 OK\r\nServer: x\r\nContent-Type: gnss/data\r\n\r\n";
  s.payload = make_frame(60, 3);
  caster.push_script(s);
  NtripClient c(
      cfg_for(caster.port()), [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().frames >= 1; }));
  c.stop();
}

TEST(NtripClient, RejectionIsReportedWithoutCredentialsAndRetried) {
  FakeCaster caster;
  Script s;
  s.header = "HTTP/1.1 401 Unauthorized\r\nWWW-Authenticate: Basic\r\n\r\n";
  s.close_after = true;
  caster.push_script(s);
  caster.push_script(s);
  caster.push_script(s);
  NtripClient c(
      cfg_for(caster.port()), [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().reconnects >= 2; }));
  const auto snap = c.snapshot();
  EXPECT_NE(snap.last_error.find("rejected"), std::string::npos);
  EXPECT_NE(snap.last_error.find("401"), std::string::npos);
  EXPECT_EQ(snap.last_error.find("s3cr3t-pw"),
            std::string::npos);  // the password never leaves the request
  EXPECT_EQ(snap.last_error.find("rover"), std::string::npos);
  EXPECT_FALSE(snap.connected);
  EXPECT_GE(caster.connections(), 2);
  c.stop();
}

TEST(NtripClient, SilentStreamTimesOutAndReconnects) {
  FakeCaster caster;
  Script s;  // header only, then silence
  caster.push_script(s);
  caster.push_script(s);
  NtripClient c(
      cfg_for(caster.port()), [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().reconnects >= 1; }, 5.0));
  EXPECT_EQ(c.snapshot().last_error.find("password"), std::string::npos);
  ASSERT_TRUE(wait_for([&] { return caster.connections() >= 2; }, 5.0));
  c.stop();
}

TEST(NtripClient, GgaBackFeedImmediatelyAndPeriodicallyOnlyWhenUsable) {
  FakeCaster caster;
  Script s;
  s.payload = make_frame(30, 4);
  s.repeat_payload = 40;  // keep the stream alive past several GGA intervals
  caster.push_script(s);
  std::atomic<int> asked{0};
  std::atomic<bool> usable{true};
  NtripClient c(
      cfg_for(caster.port()), [](const std::vector<uint8_t>&) {},
      [&]() -> std::optional<std::string> {
        ++asked;
        if (!usable) return std::nullopt;
        return std::string(
            "$GPGGA,000000.00,0000.0000,N,00000.0000,E,1,0,0.0,0.0,M,0.0,M,,*00\r\n");
      });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().gga_sent >= 2; }));
  // the caster reads the back-feed after its own send loop: allow it time
  EXPECT_TRUE(wait_for([&] { return caster.gga_received().find("$GPGGA") != std::string::npos; }));
  usable = false;  // an unusable position is never back-fed
  // A send may already be in flight (the provider answered before the flip, the send and the
  // counter come after): let one full interval (0.2 s) pass before sampling the baseline.
  std::this_thread::sleep_for(300ms);
  const auto before = c.snapshot().gga_sent;
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(c.snapshot().gga_sent, before);
  EXPECT_GE(asked.load(), 3);
  c.stop();
}

TEST(NtripClient, BadCrcFramesAreCountedAndNeverDelivered) {
  FakeCaster caster;
  Script s;
  auto bad = make_frame(30, 5);
  bad[8] ^= 0xFF;
  s.payload = bad;
  const auto good = make_frame(31, 6);
  s.payload.insert(s.payload.end(), good.begin(), good.end());
  caster.push_script(s);
  std::atomic<int> delivered{0};
  NtripClient c(
      cfg_for(caster.port()), [&](const std::vector<uint8_t>&) { ++delivered; },
      [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return delivered.load() >= 1; }));
  ASSERT_TRUE(wait_for([&] { return c.snapshot().frames >= 1; }));
  EXPECT_EQ(delivered.load(), 1);
  EXPECT_EQ(c.snapshot().bytes, s.payload.size());
  EXPECT_EQ(c.snapshot().frames, 1U);
  EXPECT_GE(c.snapshot().crc_failures, 1U);
  c.stop();
}

TEST(NtripClient, StopUnblocksQuicklyWhileWaitingForData) {
  FakeCaster caster;
  caster.push_script(Script{});
  NtripConfig cfg = cfg_for(caster.port());
  cfg.stream_timeout_s = 60.0;
  NtripClient c(cfg, [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().state == NtripState::Streaming; }));
  const auto t0 = std::chrono::steady_clock::now();
  c.stop();
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 1s);
}

TEST(NtripClient, UnreachableCasterBacksOffAndStopsPromptly) {
  NtripConfig cfg = cfg_for(1);  // nothing listens on port 1
  cfg.backoff_base_s = 30.0;     // a long backoff must still be interruptible
  cfg.backoff_max_s = 60.0;
  NtripClient c(cfg, [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().state == NtripState::Reconnecting; }, 5.0));
  EXPECT_NE(c.snapshot().last_error.find("connect"), std::string::npos);
  const auto t0 = std::chrono::steady_clock::now();
  c.stop();
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 1s);
}
