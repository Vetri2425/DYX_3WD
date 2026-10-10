// NTRIP client against a loopback fake caster (real sockets, real threads).
#include "dyx3_gnss_rtk/ntrip_client.hpp"

#include <arpa/inet.h>
#include <dirent.h>
#include <gtest/gtest.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <memory>
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
  int receive_buffer_bytes = 0;
  int pause_read_ms = 0;
};

class FakeCaster {
public:
  explicit FakeCaster(bool tls = false) : tls_server_(tls) {
    if (tls_server_) {
      server_ctx_ = SSL_CTX_new(TLS_server_method());
      const std::string base = std::string(DYX3_FIXTURES) + "/";
      EXPECT_EQ(SSL_CTX_use_certificate_file(server_ctx_, (base + "tls_test_cert.pem").c_str(),
                                             SSL_FILETYPE_PEM),
                1);
      EXPECT_EQ(SSL_CTX_use_PrivateKey_file(server_ctx_, (base + "tls_test_key.pem").c_str(),
                                            SSL_FILETYPE_PEM),
                1);
    }
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
    if (server_ctx_) SSL_CTX_free(server_ctx_);
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
      if (s.receive_buffer_bytes > 0) {
        ::setsockopt(c, SOL_SOCKET, SO_RCVBUF, &s.receive_buffer_bytes,
                     sizeof s.receive_buffer_bytes);
      }
      SSL* ssl = nullptr;
      if (tls_server_ && s.pause_read_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(s.pause_read_ms));
      }
      if (tls_server_) {
        ssl = SSL_new(server_ctx_);
        SSL_set_fd(ssl, c);
        if (SSL_accept(ssl) != 1) {
          SSL_free(ssl);
          ::close(c);
          continue;
        }
      }
      auto read_caster = [&](char* out, size_t count) {
        return ssl ? SSL_read(ssl, out, static_cast<int>(count)) : ::recv(c, out, count, 0);
      };
      auto write_caster = [&](const char* data, size_t count) {
        return ssl ? SSL_write(ssl, data, static_cast<int>(count))
                   : ::send(c, data, count, MSG_NOSIGNAL);
      };
      std::string req;
      char buf[1024];
      while (req.find("\r\n\r\n") == std::string::npos) {
        const ssize_t n = read_caster(buf, sizeof buf);
        if (n <= 0) break;
        req.append(buf, static_cast<size_t>(n));
      }
      {
        std::lock_guard<std::mutex> lk(m_);
        last_request_ = req;
      }
      write_caster(s.header.data(), s.header.size());
      if (!s.payload.empty())
        write_caster(reinterpret_cast<const char*>(s.payload.data()), s.payload.size());
      for (int k = 0; k < s.repeat_payload && !stop_; ++k) {
        std::this_thread::sleep_for(20ms);
        write_caster(reinterpret_cast<const char*>(s.payload.data()), s.payload.size());
      }
      if (!s.close_after) {
        // stay open, collecting whatever the client sends (GGA), until it hangs up or we stop
        if (s.pause_read_ms > 0)
          std::this_thread::sleep_for(std::chrono::milliseconds(s.pause_read_ms));
        while (!stop_) {
          pollfd q{c, POLLIN, 0};
          if ((!ssl || SSL_pending(ssl) == 0) && ::poll(&q, 1, 50) <= 0) continue;
          const ssize_t n = read_caster(buf, sizeof buf);
          if (n <= 0) break;
          std::lock_guard<std::mutex> lk(m_);
          gga_.append(buf, static_cast<size_t>(n));
        }
      }
      if (ssl) {
        SSL_shutdown(ssl);
        SSL_free(ssl);
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
  bool tls_server_{false};
  SSL_CTX* server_ctx_{nullptr};
};

NtripConfig cfg_for(int port) {
  NtripConfig c;
  c.host = "127.0.0.1";
  c.port = port;
  c.mountpoint = "MOUNT";
  c.user = "rover";
  c.password = "s3cr3t-pw";
  c.security = NtripSecurity::Plaintext;
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

int open_fd_count() {
  DIR* d = ::opendir("/proc/self/fd");
  if (!d) d = ::opendir("/dev/fd");
  if (!d) return -1;
  int count = 0;
  while (auto* e = ::readdir(d)) {
    if (e->d_name[0] != '.') ++count;
  }
  ::closedir(d);
  return count;
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

TEST(NtripClient, ArbitrarySourceBytesDoNotBecomeValidFrames) {
  FakeCaster caster;
  Script s;
  s.payload = {1, 2, 3, 4, 5, 6, 7};
  caster.push_script(s);
  NtripClient c(
      cfg_for(caster.port()),
      [](const std::vector<uint8_t>&) { FAIL() << "invalid RTCM delivered"; },
      [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().bytes == s.payload.size(); }));
  EXPECT_EQ(c.snapshot().frames, 0U);
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

TEST(NtripClient, LargeGgaIsByteExactAfterPartialNonblockingWrites) {
  FakeCaster caster;
  Script s;
  s.receive_buffer_bytes = 4096;
  caster.push_script(s);
  NtripConfig cfg = cfg_for(caster.port());
  cfg.stream_timeout_s = 20.0;
  cfg.gga_interval_s = 60.0;
  const std::string gga = "$GPGGA," + std::string(128 * 1024, 'A') + "*00\r\n";
  NtripClient c(cfg, [](const std::vector<uint8_t>&) {}, [gga] { return gga; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().gga_sent == 1; }, 20.0));
  ASSERT_TRUE(wait_for([&] { return caster.gga_received().size() == gga.size(); }, 20.0));
  EXPECT_EQ(caster.gga_received(), gga);
  c.stop();
}

TEST(NtripClient, StopCancelsBlockedGgaWithoutFurtherWrites) {
  FakeCaster caster;
  Script s;
  s.receive_buffer_bytes = 4096;
  s.pause_read_ms = 1500;
  caster.push_script(s);
  NtripConfig cfg = cfg_for(caster.port());
  cfg.stream_timeout_s = 10.0;
  cfg.gga_interval_s = 60.0;
  const std::string gga = "$GPGGA," + std::string(4 * 1024 * 1024, 'B') + "*00\r\n";
  NtripClient c(cfg, [](const std::vector<uint8_t>&) {}, [gga] { return gga; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().state == NtripState::Streaming; }));
  std::this_thread::sleep_for(100ms);
  const auto t0 = std::chrono::steady_clock::now();
  c.stop();
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 1s);
  EXPECT_EQ(c.snapshot().gga_sent, 0U);
}

TEST(NtripClient, FiftyStartStopCyclesDoNotLeakDescriptors) {
  FakeCaster caster;
  const int before = open_fd_count();
  ASSERT_GT(before, 0);
  for (int i = 0; i < 50; ++i) {
    NtripClient c(
        cfg_for(caster.port()), [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
    c.start();
    ASSERT_TRUE(wait_for([&] { return c.snapshot().state == NtripState::Streaming; }));
    c.stop();
  }
  // The FakeCaster closes its accepted socket on its own thread after the client hangs up, so the
  // count can lag by one for a moment (seen in CI with ctest -j2). A real leak never comes back.
  EXPECT_TRUE(wait_for([&] { return open_fd_count() == before; }, 2.0)) << open_fd_count();
}

TEST(NtripClient, RepeatedConnectFailureDoesNotLeakDescriptors) {
  const int before = open_fd_count();
  ASSERT_GT(before, 0);
  for (int i = 0; i < 50; ++i) {
    NtripClient c(cfg_for(1), [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
    c.start();
    ASSERT_TRUE(wait_for([&] { return c.snapshot().reconnects >= 1; }));
    c.stop();
  }
  EXPECT_EQ(open_fd_count(), before);
}

TEST(NtripClient, ExplicitPlaintextReportsCredentialWarning) {
  FakeCaster caster;
  caster.push_script(Script{});
  NtripClient c(
      cfg_for(caster.port()), [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().state == NtripState::Streaming; }));
  EXPECT_EQ(c.snapshot().security, NtripSecurity::Plaintext);
  EXPECT_TRUE(c.snapshot().plaintext_credentials_warning);
  EXPECT_FALSE(c.snapshot().tls_verified);
  c.stop();
}

TEST(NtripClient, MissingSecurityFailsClosed) {
  NtripConfig cfg = cfg_for(2101);
  cfg.security.reset();
  NtripClient c(cfg, [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().state == NtripState::Error; }));
  EXPECT_NE(c.snapshot().last_error.find("explicit"), std::string::npos);
  c.stop();
}

TEST(NtripClient, VerifiedTlsStreamsAndSendsGga) {
  FakeCaster caster(true);
  Script s;
  s.payload = make_frame(40, 7);
  caster.push_script(s);
  NtripConfig cfg = cfg_for(caster.port());
  cfg.security = NtripSecurity::Tls;
  cfg.ca_file = std::string(DYX3_FIXTURES) + "/tls_test_cert.pem";
  const std::string gga = "$GPGGA,123*00\r\n";
  NtripClient c(cfg, [](const std::vector<uint8_t>&) {}, [gga] { return gga; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().frames == 1; }));
  ASSERT_TRUE(wait_for([&] { return c.snapshot().gga_sent >= 1; }));
  EXPECT_TRUE(c.snapshot().tls_verified);
  EXPECT_FALSE(c.snapshot().plaintext_credentials_warning);
  ASSERT_TRUE(wait_for([&] { return caster.gga_received().find(gga) != std::string::npos; }));
  c.stop();
}

TEST(NtripClient, UntrustedTlsCertificateFailsWithoutDowngrade) {
  FakeCaster caster(true);
  caster.push_script(Script{});
  NtripConfig cfg = cfg_for(caster.port());
  cfg.security = NtripSecurity::Tls;
  NtripClient c(cfg, [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().tls_verification_failed; }));
  EXPECT_FALSE(c.snapshot().tls_verified);
  EXPECT_EQ(c.snapshot().security, NtripSecurity::Tls);
  EXPECT_TRUE(caster.last_request().empty());
  c.stop();
}

TEST(NtripClient, TlsHostnameMismatchFailsWithoutCredentialsInError) {
  FakeCaster caster(true);
  caster.push_script(Script{});
  NtripConfig cfg = cfg_for(caster.port());
  cfg.security = NtripSecurity::Tls;
  cfg.host = "localhost";  // trusted certificate contains only 127.0.0.1 IP SAN
  cfg.ca_file = std::string(DYX3_FIXTURES) + "/tls_test_cert.pem";
  NtripClient c(cfg, [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().tls_verification_failed; }));
  EXPECT_FALSE(c.snapshot().tls_verified);
  EXPECT_EQ(c.snapshot().last_error.find(cfg.password), std::string::npos);
  EXPECT_EQ(c.snapshot().last_error.find(cfg.user), std::string::npos);
  c.stop();
}

TEST(NtripClient, TlsNeverFallsBackToPlainCaster) {
  FakeCaster caster;
  caster.push_script(Script{});
  NtripConfig cfg = cfg_for(caster.port());
  cfg.security = NtripSecurity::Tls;
  cfg.connect_timeout_s = 0.4;
  NtripClient c(cfg, [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().reconnects >= 1; }));
  EXPECT_FALSE(c.snapshot().tls_verified);
  EXPECT_TRUE(caster.last_request().find("Authorization:") == std::string::npos);
  c.stop();
}

TEST(NtripClient, TlsGgaLargeWriteIsByteExact) {
  FakeCaster caster(true);
  Script s;
  s.receive_buffer_bytes = 4096;
  caster.push_script(s);
  NtripConfig cfg = cfg_for(caster.port());
  cfg.security = NtripSecurity::Tls;
  cfg.ca_file = std::string(DYX3_FIXTURES) + "/tls_test_cert.pem";
  cfg.stream_timeout_s = 20.0;
  cfg.gga_interval_s = 60.0;
  const std::string gga = "$GPGGA," + std::string(128 * 1024, 'T') + "*00\r\n";
  NtripClient c(cfg, [](const std::vector<uint8_t>&) {}, [gga] { return gga; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().gga_sent == 1; }, 20.0));
  ASSERT_TRUE(wait_for([&] { return caster.gga_received().size() == gga.size(); }, 20.0));
  EXPECT_EQ(caster.gga_received(), gga);
  c.stop();
}

TEST(NtripClient, StopInterruptsTlsHandshake) {
  FakeCaster caster(true);
  Script s;
  s.pause_read_ms = 1500;
  caster.push_script(s);
  NtripConfig cfg = cfg_for(caster.port());
  cfg.security = NtripSecurity::Tls;
  cfg.ca_file = std::string(DYX3_FIXTURES) + "/tls_test_cert.pem";
  NtripClient c(cfg, [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return caster.connections() == 1; }));
  const auto t0 = std::chrono::steady_clock::now();
  c.stop();
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 1s);
}

TEST(NtripClient, TlsReconnectsWithoutDowngrading) {
  FakeCaster caster(true);
  Script s;
  s.payload = make_frame(32, 8);
  s.close_after = true;
  caster.push_script(s);
  caster.push_script(s);
  NtripConfig cfg = cfg_for(caster.port());
  cfg.security = NtripSecurity::Tls;
  cfg.ca_file = std::string(DYX3_FIXTURES) + "/tls_test_cert.pem";
  NtripClient c(cfg, [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().frames >= 2; }));
  EXPECT_GE(caster.connections(), 2);
  EXPECT_EQ(c.snapshot().security, NtripSecurity::Tls);
  c.stop();
}

TEST(NtripClient, GgaSendDeadlineReportsTimeout) {
  FakeCaster caster;
  Script s;
  s.receive_buffer_bytes = 4096;
  s.pause_read_ms = 1500;
  caster.push_script(s);
  NtripConfig cfg = cfg_for(caster.port());
  cfg.stream_timeout_s = 0.2;
  cfg.gga_interval_s = 60.0;
  std::atomic<bool> timeout_seen{false};
  const std::string gga = "$GPGGA," + std::string(4 * 1024 * 1024, 'Z') + "*00\r\n";
  NtripClient c(cfg, [](const std::vector<uint8_t>&) {}, [gga] { return gga; });
  c.set_event_callback([&](NtripState state, const std::string& detail) {
    if (state == NtripState::Error && detail == "GGA send timeout") timeout_seen = true;
  });
  c.start();
  EXPECT_TRUE(wait_for([&] { return timeout_seen.load(); }, 5.0));
  EXPECT_EQ(c.snapshot().gga_sent, 0U);
  c.stop();
}

TEST(NtripClient, InterruptedGgaWriteAndPollResume) {
  FakeCaster caster;
  Script s;
  s.receive_buffer_bytes = 4096;
  s.pause_read_ms = 300;
  caster.push_script(s);
  NtripConfig cfg = cfg_for(caster.port());
  cfg.stream_timeout_s = 20.0;
  cfg.gga_interval_s = 60.0;
  const std::string gga = "$GPGGA," + std::string(256 * 1024, 'I') + "*00\r\n";
  std::atomic<bool> in_gga{false};
  pthread_t worker{};
  auto old_handler = ::signal(SIGUSR1, +[](int) {});
  NtripClient c(
      cfg, [](const std::vector<uint8_t>&) {},
      [&] {
        worker = pthread_self();
        in_gga = true;
        return std::optional<std::string>(gga);
      });
  c.start();
  if (!wait_for([&] { return in_gga.load(); })) {
    ADD_FAILURE() << "GGA callback did not start";
    c.stop();
    ::signal(SIGUSR1, old_handler);
    return;
  }
  for (int i = 0; i < 30 && c.snapshot().gga_sent == 0; ++i) {
    EXPECT_EQ(pthread_kill(worker, SIGUSR1), 0);
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(wait_for([&] { return c.snapshot().gga_sent == 1; }, 20.0));
  EXPECT_TRUE(wait_for([&] { return caster.gga_received().size() == gga.size(); }, 20.0));
  EXPECT_EQ(caster.gga_received(), gga);
  c.stop();
  ::signal(SIGUSR1, old_handler);
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

// ---- valid-frame liveness (a stream of bytes is not a stream of RTCM)
// -------------------------------------
TEST(NtripClient, KeepAliveBytesWithoutValidFramesTimeOutAndReconnect) {
  FakeCaster caster;
  Script s;
  s.payload = {'\r', '\n'};  // no 0xD3 preamble: bytes arrive, no RTCM frame ever completes
  s.repeat_payload = 150;    // ... every 20 ms for 3 s, far beyond stream_timeout_s (0.5 s)
  caster.push_script(s);
  std::atomic<int> delivered{0};
  NtripClient c(
      cfg_for(caster.port()), [&](const std::vector<uint8_t>&) { ++delivered; },
      [] { return std::nullopt; });
  const auto t0 = std::chrono::steady_clock::now();
  c.start();
  // Bytes keep flowing for 3 s; a byte-level timeout would not fire before they stop.
  ASSERT_TRUE(wait_for([&] { return c.snapshot().frame_timeouts >= 1; }, 2.5));
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 2500ms);
  ASSERT_TRUE(wait_for(
      [&] { return c.snapshot().last_error.find("no valid RTCM frame") != std::string::npos; },
      1.0));
  const auto snap = c.snapshot();
  EXPECT_GE(snap.reconnects, 1U);
  EXPECT_GT(snap.bytes, s.payload.size());  // the keep-alives were really being received
  EXPECT_EQ(snap.frames, 0U);
  EXPECT_EQ(delivered.load(), 0);
  c.stop();
}

TEST(NtripClient, ValidFramesKeepTheStreamAliveBeyondTheStreamTimeout) {
  FakeCaster caster;
  Script s;
  s.payload = make_frame(30, 9);
  s.repeat_payload = 60;  // 1.2 s of valid frames, 20 ms apart, stream_timeout_s is 0.5 s
  caster.push_script(s);
  NtripClient c(
      cfg_for(caster.port()), [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().frames >= 50; }, 5.0));
  const auto snap = c.snapshot();
  EXPECT_EQ(snap.frame_timeouts, 0U);
  EXPECT_EQ(snap.reconnects, 0U);
  EXPECT_EQ(snap.state, NtripState::Streaming);
  c.stop();
}

// ---- DNS with a deadline
// ------------------------------------------------------------------------------------
namespace {
// A resolver that blocks until released. Everything the helper thread touches is shared_ptr-owned
// so it stays valid after the test (and the call that started it) is gone.
struct HungResolver {
  std::shared_ptr<std::atomic<int>> calls = std::make_shared<std::atomic<int>>(0);
  std::shared_ptr<std::atomic<bool>> release = std::make_shared<std::atomic<bool>>(false);
  ResolverFn fn() const {
    return [calls = calls, release = release](const std::string&, const std::string&, addrinfo**) {
      ++*calls;
      const auto end = std::chrono::steady_clock::now() + 20s;  // safety net, never reached
      while (!*release && std::chrono::steady_clock::now() < end) std::this_thread::sleep_for(5ms);
      return EAI_AGAIN;
    };
  }
};
double seconds_since(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}
}  // namespace

TEST(NtripDns, UnresolvableNameFailsWithinTheConnectTimeout) {
  NtripConfig cfg = cfg_for(2101);
  cfg.host = "dyx3-no-such-host.invalid";  // RFC 6761: never resolves
  cfg.connect_timeout_s = 1.0;
  cfg.backoff_base_s = 30.0;
  cfg.backoff_max_s = 60.0;
  NtripClient c(cfg, [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
  const auto t0 = std::chrono::steady_clock::now();
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().state == NtripState::Reconnecting; },
                       cfg.connect_timeout_s + 2.0));
  EXPECT_LT(seconds_since(t0), cfg.connect_timeout_s + 1.5);
  // Either the resolver answered "no such name" or the deadline cut it off: both are the error
  // path.
  const std::string error = c.snapshot().last_error;
  EXPECT_TRUE(error == "cannot resolve caster host" || error == "caster host resolution timeout")
      << error;
  EXPECT_FALSE(c.snapshot().connected);
  EXPECT_GE(c.snapshot().reconnects, 1U);
  const auto t1 = std::chrono::steady_clock::now();
  c.stop();
  EXPECT_LT(seconds_since(t1), 1.0);
}

TEST(NtripDns, ResolveWithDeadlineStatuses) {
  std::shared_ptr<DnsLookup> inflight;
  addrinfo* res = nullptr;
  {  // numeric address: no resolver, no helper thread
    ASSERT_EQ(resolve_with_deadline("127.0.0.1", "2101", 1.0, -1, inflight, &res),
              ResolveStatus::Resolved);
    ASSERT_NE(res, nullptr);
    ::freeaddrinfo(res);
    EXPECT_EQ(inflight, nullptr);
  }
  {  // a resolver that answers
    const ResolverFn ok = [](const std::string&, const std::string& port, addrinfo** out) {
      addrinfo hints{};
      hints.ai_family = AF_INET;
      hints.ai_socktype = SOCK_STREAM;
      hints.ai_flags = AI_NUMERICHOST;
      return ::getaddrinfo("127.0.0.1", port.c_str(), &hints, out);
    };
    ASSERT_EQ(resolve_with_deadline("caster.example", "2101", 2.0, -1, inflight, &res, ok),
              ResolveStatus::Resolved);
    ASSERT_NE(res, nullptr);
    ::freeaddrinfo(res);
    EXPECT_EQ(inflight, nullptr);
  }
  {  // a resolver that fails
    const ResolverFn fail = [](const std::string&, const std::string&, addrinfo**) {
      return static_cast<int>(EAI_NONAME);
    };
    EXPECT_EQ(resolve_with_deadline("caster.example", "2101", 2.0, -1, inflight, &res, fail),
              ResolveStatus::Failed);
    EXPECT_EQ(res, nullptr);
    EXPECT_EQ(inflight, nullptr);
  }
}

TEST(NtripDns, HungResolverTimesOutAndIsRejoinedNotRestarted) {
  HungResolver hung;
  std::shared_ptr<DnsLookup> inflight;
  addrinfo* res = nullptr;
  auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(resolve_with_deadline("caster.example", "2101", 0.2, -1, inflight, &res, hung.fn()),
            ResolveStatus::Timeout);
  EXPECT_GE(seconds_since(t0), 0.15);
  EXPECT_LT(seconds_since(t0), 1.0);
  EXPECT_EQ(res, nullptr);
  ASSERT_NE(inflight, nullptr);
  // The next attempt waits on the same lookup: still one helper thread, one resolver call.
  t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(resolve_with_deadline("caster.example", "2101", 0.2, -1, inflight, &res, hung.fn()),
            ResolveStatus::Timeout);
  EXPECT_LT(seconds_since(t0), 1.0);
  EXPECT_EQ(hung.calls->load(), 1);
  // Once the resolver finally answers, the result is collected and the handle released.
  *hung.release = true;
  EXPECT_EQ(resolve_with_deadline("caster.example", "2101", 2.0, -1, inflight, &res, hung.fn()),
            ResolveStatus::Failed);  // EAI_AGAIN from the fake
  EXPECT_EQ(res, nullptr);
  EXPECT_EQ(inflight, nullptr);
  EXPECT_EQ(hung.calls->load(), 1);
}

TEST(NtripDns, StopRequestInterruptsALookupImmediately) {
  HungResolver hung;
  int wake[2];
  ASSERT_EQ(::pipe(wake), 0);
  std::shared_ptr<DnsLookup> inflight;
  addrinfo* res = nullptr;
  std::thread stopper([&] {
    std::this_thread::sleep_for(100ms);
    const char c = 1;
    [[maybe_unused]] const ssize_t w = ::write(wake[1], &c, 1);
  });
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(
      resolve_with_deadline("caster.example", "2101", 10.0, wake[0], inflight, &res, hung.fn()),
      ResolveStatus::Stopped);
  EXPECT_LT(seconds_since(t0), 1.0);
  stopper.join();
  ::close(wake[0]);
  ::close(wake[1]);
  *hung.release = true;  // lets the abandoned helper finish and free its own state
}

TEST(NtripDns, HungResolverNeitherStallsTheWorkerNorBlocksStop) {
  HungResolver hung;
  NtripConfig cfg = cfg_for(2101);
  cfg.host = "hung.example";
  cfg.connect_timeout_s = 0.3;
  cfg.backoff_base_s = 0.05;
  cfg.backoff_max_s = 0.1;
  NtripClient c(cfg, [](const std::vector<uint8_t>&) {}, [] { return std::nullopt; });
  c.set_resolver(hung.fn());
  c.start();
  ASSERT_TRUE(wait_for([&] { return c.snapshot().reconnects >= 2; }, 5.0));
  EXPECT_EQ(c.snapshot().last_error, "caster host resolution timeout");
  EXPECT_EQ(hung.calls->load(), 1);  // every attempt rejoined the one pending lookup
  const auto t0 = std::chrono::steady_clock::now();
  c.stop();  // joins the worker: must not wait for the resolver
  EXPECT_LT(seconds_since(t0), 1.0);
  *hung.release = true;
}
