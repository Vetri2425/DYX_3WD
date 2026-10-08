// ntrip_client — the NTRIP protocol (pure) and a threaded socket client. Contract: dyx3_gnss_rtk.md
// section 4. Behaviour carried from the prototype (ntrip_rtcm_node.py + ntrip_protocol.py); no
// credentials are ever logged or placed in a status string.
#pragma once

#include <openssl/ssl.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "dyx3_gnss_rtk/rtcm_parser.hpp"

namespace dyx3_gnss_rtk {

// ---- pure protocol
// -------------------------------------------------------------------------------------
std::string base64_encode(const std::string& in);

// "GET /<mountpoint> HTTP/1.0" request with HTTP Basic auth, exactly as the prototype sends it.
std::string build_request(const std::string& host, const std::string& mountpoint,
                          const std::string& user, const std::string& password);

// Accept only an ICY / HTTP/x.y status line whose status code is exactly 200.
bool response_is_success(const std::string& header);

enum class HeaderStatus { NeedMore, Complete, TooLong };
struct HeaderParse {
  HeaderStatus status{HeaderStatus::NeedMore};
  std::string header;             // the status line (ICY) or the whole header (HTTP)
  std::vector<uint8_t> leftover;  // bytes after the header: RTCM, kept
};
// Header ends at a blank line, or (ICY) after its status line; > 2048 bytes without either is
// TooLong.
HeaderParse parse_response_header(const std::vector<uint8_t>& received);

// min(base * 2^attempt, max)
double backoff_s(int attempt, double base_s, double max_s);

// ---- threaded client
// -----------------------------------------------------------------------------------
enum class NtripSecurity : uint8_t { Plaintext = 1, Tls = 2 };

struct NtripConfig {
  std::string host;
  int port{2101};
  std::string mountpoint;
  std::string user;
  std::string password;                   // never logged
  std::optional<NtripSecurity> security;  // required; never inferred from the port
  std::string ca_file;                    // optional PEM trust anchor; otherwise system trust store
  double connect_timeout_s{10.0};
  double stream_timeout_s{10.0};
  double gga_interval_s{10.0};
  double backoff_base_s{5.0};
  double backoff_max_s{60.0};
  size_t max_buffer_bytes{8192};
};

enum class NtripState : uint8_t {
  Starting = 0,
  Connecting,
  Streaming,
  Reconnecting,
  Error,
  Stopped
};

struct NtripSnapshot {
  NtripState state{NtripState::Starting};
  bool connected{false};
  std::string last_error;  // short reason, never credentials
  uint32_t reconnects{0};
  uint64_t frames{0};
  uint64_t bytes{0};
  uint64_t crc_failures{0};
  uint64_t resync_bytes{0};
  uint64_t gga_sent{0};
  std::optional<NtripSecurity> security;
  bool tls_verified{false};
  bool tls_verification_failed{false};
  bool plaintext_credentials_warning{false};
};

class NtripClient {
public:
  using FrameSink = std::function<void(const std::vector<uint8_t>& frame)>;
  using GgaSource =
      std::function<std::optional<std::string>()>;  // full sentence incl. CRLF, or nullopt
  using Event = std::function<void(NtripState, const std::string&)>;

  NtripClient(NtripConfig cfg, FrameSink sink, GgaSource gga);
  ~NtripClient();
  NtripClient(const NtripClient&) = delete;
  NtripClient& operator=(const NtripClient&) = delete;

  void set_event_callback(Event e) { event_ = std::move(e); }
  void start();
  void stop();  // idempotent; requests shutdown and joins; worker alone closes the socket
  NtripSnapshot snapshot() const;

private:
  void run();
  bool connect_and_handshake(int* fd, std::vector<uint8_t>* leftover, std::string* error);
  void set_state(NtripState s, const std::string& detail = std::string());
  void close_fd();
  bool own_fd(int fd);
  bool begin_tls(int fd, std::string* error);
  void close_tls();

  NtripConfig cfg_;
  FrameSink sink_;
  GgaSource gga_;
  Event event_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  // Guard both shutdown and close so a recycled fd number can never be targeted by stop().
  std::mutex fd_m_;
  int active_fd_{-1};          // worker owns close; stop only shutdowns while holding fd_m_
  SSL_CTX* tls_ctx_{nullptr};  // worker-owned; never accessed by stop()
  SSL* tls_{nullptr};
  mutable std::mutex m_;
  std::condition_variable cv_;
  NtripSnapshot snap_;
  int wake_[2]{-1, -1};
};

}  // namespace dyx3_gnss_rtk
