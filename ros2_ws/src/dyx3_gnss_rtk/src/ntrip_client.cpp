#include "dyx3_gnss_rtk/ntrip_client.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <limits>

namespace dyx3_gnss_rtk {

// ---------------------------------------------------------------------------------------------------------
std::string base64_encode(const std::string& in) {
  static const char* kTab = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  size_t i = 0;
  while (i + 2 < in.size()) {
    const unsigned v = (static_cast<unsigned char>(in[i]) << 16) |
                       (static_cast<unsigned char>(in[i + 1]) << 8) |
                       static_cast<unsigned char>(in[i + 2]);
    out += kTab[(v >> 18) & 63];
    out += kTab[(v >> 12) & 63];
    out += kTab[(v >> 6) & 63];
    out += kTab[v & 63];
    i += 3;
  }
  if (i + 1 == in.size()) {
    const unsigned v = static_cast<unsigned char>(in[i]) << 16;
    out += kTab[(v >> 18) & 63];
    out += kTab[(v >> 12) & 63];
    out += "==";
  } else if (i + 2 == in.size()) {
    const unsigned v =
        (static_cast<unsigned char>(in[i]) << 16) | (static_cast<unsigned char>(in[i + 1]) << 8);
    out += kTab[(v >> 18) & 63];
    out += kTab[(v >> 12) & 63];
    out += kTab[(v >> 6) & 63];
    out += '=';
  }
  return out;
}

std::string build_request(const std::string& host, const std::string& mountpoint,
                          const std::string& user, const std::string& password) {
  const std::string creds = base64_encode(user + ":" + password);
  return "GET /" + mountpoint + " HTTP/1.0\r\nHost: " + host +
         "\r\nNtrip-Version: Ntrip/2.0\r\nUser-Agent: NTRIP ROS2/1.0\r\nAuthorization: Basic " +
         creds + "\r\n\r\n";
}

bool response_is_success(const std::string& header) {
  // First line, trimmed; ^(ICY|HTTP/\d+(\.\d+)?)\s+200(\s|$), case-insensitive.
  const auto nl = header.find_first_of("\r\n");
  std::string line = header.substr(0, nl);
  const auto b = line.find_first_not_of(" \t");
  if (b == std::string::npos) return false;
  line = line.substr(b);
  while (!line.empty() && (line.back() == ' ' || line.back() == '\t')) line.pop_back();
  auto up = [](std::string s) {
    for (auto& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
  };
  const std::string u = up(line);
  size_t p = 0;
  if (u.rfind("ICY", 0) == 0) {
    p = 3;
  } else if (u.rfind("HTTP/", 0) == 0) {
    p = 5;
    const size_t d0 = p;
    while (p < u.size() && std::isdigit(static_cast<unsigned char>(u[p]))) ++p;
    if (p == d0) return false;
    if (p < u.size() && u[p] == '.') {
      ++p;
      const size_t d1 = p;
      while (p < u.size() && std::isdigit(static_cast<unsigned char>(u[p]))) ++p;
      if (p == d1) return false;
    }
  } else {
    return false;
  }
  const size_t ws = p;
  while (p < u.size() && std::isspace(static_cast<unsigned char>(u[p]))) ++p;
  if (p == ws) return false;  // \s+ needs at least one
  if (u.compare(p, 3, "200") != 0) return false;
  p += 3;
  return p == u.size() || std::isspace(static_cast<unsigned char>(u[p]));
}

HeaderParse parse_response_header(const std::vector<uint8_t>& rx) {
  HeaderParse out;
  const std::string s(rx.begin(), rx.end());
  const auto end = s.find("\r\n\r\n");
  if (end != std::string::npos) {
    out.status = HeaderStatus::Complete;
    out.header = s.substr(0, end);
    out.leftover.assign(rx.begin() + static_cast<std::ptrdiff_t>(end + 4), rx.end());
    return out;
  }
  if (s.rfind("ICY ", 0) == 0) {
    const auto nl = s.find("\r\n");
    if (nl != std::string::npos) {
      out.status = HeaderStatus::Complete;
      out.header = s.substr(0, nl);
      out.leftover.assign(rx.begin() + static_cast<std::ptrdiff_t>(nl + 2), rx.end());
      return out;
    }
  }
  out.status = rx.size() > 2048 ? HeaderStatus::TooLong : HeaderStatus::NeedMore;
  return out;
}

double backoff_s(int attempt, double base_s, double max_s) {
  const double v = base_s * std::pow(2.0, std::min(attempt, 30));
  return std::min(v, max_s);
}

// ---------------------------------------------------------------------------------------------------------
namespace {

double now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Waits for `events` on fd or the wake pipe. Returns 1 ready, 0 timeout, -1 woken (stop) / error.
int wait_fd(int fd, short events, int wake_fd, double timeout_s) {
  pollfd p[2];
  p[0] = {fd, events, 0};
  p[1] = {wake_fd, POLLIN, 0};
  const double deadline = now_s() + std::max(0.0, timeout_s);
  for (;;) {
    const int ms =
        static_cast<int>(std::min(std::max(0.0, deadline - now_s()), 3600.0) * 1000.0 + 0.5);
    const int r = ::poll(p, wake_fd >= 0 ? 2 : 1, ms);
    if (r < 0 && errno == EINTR) continue;
    if (r < 0) return -1;
    if (r == 0) return 0;
    if (wake_fd >= 0 && (p[1].revents & POLLIN) != 0) return -1;
    if ((p[0].revents & events) != 0) return 1;
    if ((p[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) return -1;
    return 0;
  }
}

enum class SendResult { Complete, Timeout, Stopped, Failed };

SendResult send_all(int fd, SSL* ssl, const char* data, size_t size, int wake_fd,
                    const std::atomic<bool>& stopped, double deadline) {
  size_t sent = 0;
  while (sent < size) {
    if (stopped) return SendResult::Stopped;
    if (now_s() >= deadline) return SendResult::Timeout;
    const ssize_t n =
        ssl ? SSL_write(ssl, data + sent,
                        static_cast<int>(std::min(
                            size - sent, static_cast<size_t>(std::numeric_limits<int>::max()))))
            : ::send(fd, data + sent, size - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += static_cast<size_t>(n);
      continue;
    }
    const int ssl_error = ssl ? SSL_get_error(ssl, static_cast<int>(n)) : 0;
    if (ssl && (ssl_error == SSL_ERROR_WANT_READ || ssl_error == SSL_ERROR_WANT_WRITE)) {
      const int w = wait_fd(fd, ssl_error == SSL_ERROR_WANT_READ ? POLLIN : POLLOUT, wake_fd,
                            deadline - now_s());
      if (w < 0) return stopped ? SendResult::Stopped : SendResult::Failed;
      if (w == 0) return SendResult::Timeout;
      continue;
    }
    if (ssl && ssl_error == SSL_ERROR_SYSCALL && errno == EINTR) continue;
    if (!ssl && n < 0 && errno == EINTR) continue;
    if (!ssl && n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      const int w = wait_fd(fd, POLLOUT, wake_fd, deadline - now_s());
      if (w < 0) return stopped ? SendResult::Stopped : SendResult::Failed;
      if (w == 0) return SendResult::Timeout;
      continue;
    }
    return SendResult::Failed;
  }
  return SendResult::Complete;
}

// >0 bytes, 0 EOF, -1 failure, -2 cancellation, -3 deadline.
ssize_t read_some(int fd, SSL* ssl, uint8_t* data, size_t size, int wake_fd,
                  const std::atomic<bool>& stopped, double deadline) {
  short events = POLLIN;
  for (;;) {
    if (stopped) return -2;
    const double left = deadline - now_s();
    if (left <= 0.0) return -3;
    if (!ssl || SSL_pending(ssl) == 0) {
      const int w = wait_fd(fd, events, wake_fd, left);
      if (w < 0) return stopped ? -2 : -1;
      if (w == 0) return -3;
    }
    const ssize_t n = ssl ? SSL_read(ssl, data, static_cast<int>(size)) : ::recv(fd, data, size, 0);
    if (n > 0 || n == 0) return n;
    if (ssl) {
      const int e = SSL_get_error(ssl, static_cast<int>(n));
      if (e == SSL_ERROR_ZERO_RETURN) return 0;
      if (e == SSL_ERROR_WANT_READ || e == SSL_ERROR_WANT_WRITE) {
        events = e == SSL_ERROR_WANT_READ ? POLLIN : POLLOUT;
        continue;
      }
      if (e == SSL_ERROR_SYSCALL && errno == EINTR) continue;
      return -1;
    }
    if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) return -1;
  }
}

std::string short_errno(const char* what) {
  return std::string(what) + ": " + std::strerror(errno);
}

}  // namespace

NtripClient::NtripClient(NtripConfig cfg, FrameSink sink, GgaSource gga)
    : cfg_(std::move(cfg)), sink_(std::move(sink)), gga_(std::move(gga)) {
  // OpenSSL writes to the socket without MSG_NOSIGNAL. RTK is a standalone service; a
  // peer close must become a reconnectable I/O error, never terminate the process.
  if (cfg_.security == NtripSecurity::Tls) {
    static std::once_flag ignore_sigpipe;
    std::call_once(ignore_sigpipe, [] { std::signal(SIGPIPE, SIG_IGN); });
  }
  snap_.security = cfg_.security;
  snap_.plaintext_credentials_warning =
      cfg_.security == NtripSecurity::Plaintext && (!cfg_.user.empty() || !cfg_.password.empty());
  if (::pipe(wake_) == 0) {
    ::fcntl(wake_[0], F_SETFL, O_NONBLOCK);
    ::fcntl(wake_[1], F_SETFL, O_NONBLOCK);
  }
}

NtripClient::~NtripClient() {
  stop();
  for (int& fd : wake_) {
    if (fd >= 0) ::close(fd);
    fd = -1;
  }
}

void NtripClient::set_state(NtripState s, const std::string& detail) {
  {
    std::lock_guard<std::mutex> lk(m_);
    snap_.state = s;
    snap_.connected = (s == NtripState::Streaming);
    if (s != NtripState::Streaming) snap_.tls_verified = false;
    if (s == NtripState::Error) snap_.last_error = detail;
    if (s == NtripState::Streaming) snap_.last_error.clear();
  }
  if (event_) event_(s, detail);
}

NtripSnapshot NtripClient::snapshot() const {
  std::lock_guard<std::mutex> lk(m_);
  return snap_;
}

void NtripClient::start() {
  if (thread_.joinable()) return;
  if (wake_[0] >= 0) {
    char buf[64];
    while (::read(wake_[0], buf, sizeof buf) > 0) {
    }
  }
  stop_ = false;
  thread_ = std::thread([this] { run(); });
}

void NtripClient::stop() {
  stop_ = true;
  if (wake_[1] >= 0) {
    const char c = 1;
    [[maybe_unused]] const ssize_t w = ::write(wake_[1], &c, 1);
  }
  cv_.notify_all();
  {
    std::lock_guard<std::mutex> lk(fd_m_);
    if (active_fd_ >= 0) ::shutdown(active_fd_, SHUT_RDWR);
  }
  if (thread_.joinable()) thread_.join();
  set_state(NtripState::Stopped);
}

void NtripClient::close_fd() {
  close_tls();
  std::lock_guard<std::mutex> lk(fd_m_);
  if (active_fd_ >= 0) {
    ::close(active_fd_);
    active_fd_ = -1;
  }
}

void NtripClient::close_tls() {
  if (tls_) {
    if (SSL_is_init_finished(tls_)) SSL_shutdown(tls_);  // one nonblocking close-notify attempt
    SSL_free(tls_);
    tls_ = nullptr;
  }
  if (tls_ctx_) {
    SSL_CTX_free(tls_ctx_);
    tls_ctx_ = nullptr;
  }
}

bool NtripClient::begin_tls(int fd, std::string* error) {
  tls_ctx_ = SSL_CTX_new(TLS_client_method());
  if (!tls_ctx_) {
    *error = "TLS context failed";
    return false;
  }
  SSL_CTX_set_verify(tls_ctx_, SSL_VERIFY_PEER, nullptr);
  const int trust_ok = cfg_.ca_file.empty()
                           ? SSL_CTX_set_default_verify_paths(tls_ctx_)
                           : SSL_CTX_load_verify_locations(tls_ctx_, cfg_.ca_file.c_str(), nullptr);
  if (trust_ok != 1) {
    *error = "TLS trust store failed";
    return false;
  }
  tls_ = SSL_new(tls_ctx_);
  if (!tls_ || SSL_set_fd(tls_, fd) != 1) {
    *error = "TLS setup failed";
    return false;
  }
  in_addr ip4{};
  in6_addr ip6{};
  if (::inet_pton(AF_INET, cfg_.host.c_str(), &ip4) == 1 ||
      ::inet_pton(AF_INET6, cfg_.host.c_str(), &ip6) == 1) {
    if (X509_VERIFY_PARAM_set1_ip_asc(SSL_get0_param(tls_), cfg_.host.c_str()) != 1) {
      *error = "TLS identity setup failed";
      return false;
    }
  } else if (SSL_set1_host(tls_, cfg_.host.c_str()) != 1 ||
             SSL_set_tlsext_host_name(tls_, cfg_.host.c_str()) != 1) {
    *error = "TLS hostname setup failed";
    return false;
  }
  const double deadline = now_s() + cfg_.connect_timeout_s;
  while (!stop_ && now_s() < deadline) {
    const int n = SSL_connect(tls_);
    if (n == 1) {
      std::lock_guard<std::mutex> lk(m_);
      snap_.tls_verified = true;
      snap_.tls_verification_failed = false;
      return true;
    }
    const int e = SSL_get_error(tls_, n);
    if (e != SSL_ERROR_WANT_READ && e != SSL_ERROR_WANT_WRITE) {
      const bool verify_failed = SSL_get_verify_result(tls_) != X509_V_OK;
      {
        std::lock_guard<std::mutex> lk(m_);
        snap_.tls_verification_failed = verify_failed;
      }
      *error =
          verify_failed ? "TLS certificate/hostname verification failed" : "TLS handshake failed";
      return false;
    }
    const int w =
        wait_fd(fd, e == SSL_ERROR_WANT_READ ? POLLIN : POLLOUT, wake_[0], deadline - now_s());
    if (w <= 0) {
      *error = stop_ ? "stopped" : w == 0 ? "TLS handshake timeout" : "TLS handshake failed";
      return false;
    }
  }
  *error = stop_ ? "stopped" : "TLS handshake timeout";
  return false;
}

bool NtripClient::own_fd(int fd) {
  std::lock_guard<std::mutex> lk(fd_m_);
  if (stop_) return false;
  active_fd_ = fd;
  return true;
}

bool NtripClient::connect_and_handshake(int* out_fd, std::vector<uint8_t>* leftover,
                                        std::string* error) {
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* res = nullptr;
  const std::string port = std::to_string(cfg_.port);
  if (::getaddrinfo(cfg_.host.c_str(), port.c_str(), &hints, &res) != 0 || res == nullptr) {
    *error = "cannot resolve caster host";
    return false;
  }
  int fd = -1;
  for (addrinfo* a = res; a != nullptr && fd < 0 && !stop_; a = a->ai_next) {
    const int s = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
    if (s < 0) continue;
    ::fcntl(s, F_SETFL, ::fcntl(s, F_GETFL, 0) | O_NONBLOCK);
    if (!own_fd(s)) {
      ::close(s);
      break;
    }
    int r = ::connect(s, a->ai_addr, a->ai_addrlen);
    if (r < 0 && errno == EINPROGRESS) {
      const int w = wait_fd(s, POLLOUT, wake_[0], cfg_.connect_timeout_s);
      if (w == 1) {
        int soerr = 0;
        socklen_t len = sizeof soerr;
        ::getsockopt(s, SOL_SOCKET, SO_ERROR, &soerr, &len);
        r = soerr == 0 ? 0 : -1;
        if (soerr != 0) errno = soerr;
      } else {
        errno = w == 0 ? ETIMEDOUT : ECANCELED;
        r = -1;
      }
    }
    if (r == 0) {
      fd = s;
    } else {
      close_fd();
    }
  }
  ::freeaddrinfo(res);
  if (fd < 0) {
    *error = stop_ ? "stopped" : short_errno("connect failed");
    return false;
  }
  int one = 1;
  ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);

  if (cfg_.security == NtripSecurity::Tls && !begin_tls(fd, error)) {
    close_fd();
    return false;
  }

  const std::string req = build_request(cfg_.host, cfg_.mountpoint, cfg_.user, cfg_.password);
  const double deadline = now_s() + cfg_.connect_timeout_s;
  const auto request_result = send_all(fd, tls_, req.data(), req.size(), wake_[0], stop_, deadline);
  if (request_result != SendResult::Complete) {
    *error = stop_                                   ? "stopped"
             : request_result == SendResult::Timeout ? "request send timeout"
                                                     : "request send failed";
    close_fd();
    return false;
  }

  std::vector<uint8_t> rx;
  for (;;) {
    uint8_t buf[512];
    const ssize_t n = read_some(fd, tls_, buf, sizeof buf, wake_[0], stop_, deadline);
    if (n < 0) {
      *error = stop_ ? "stopped" : n == -3 ? "handshake timeout" : "handshake read failed";
      close_fd();
      return false;
    }
    if (n == 0) {
      *error = "caster closed the connection during the handshake";
      close_fd();
      return false;
    }
    rx.insert(rx.end(), buf, buf + n);
    const auto hp = parse_response_header(rx);
    if (hp.status == HeaderStatus::NeedMore) continue;
    if (hp.status == HeaderStatus::TooLong) {
      *error = "bad response from the caster";
      close_fd();
      return false;
    }
    if (!response_is_success(hp.header)) {
      // A caster's response is untrusted: it may echo Authorization or a password.
      const auto first_space = hp.header.find(' ');
      const std::string status =
          first_space == std::string::npos ? std::string() : hp.header.substr(first_space + 1, 3);
      *error =
          status.size() == 3 && std::all_of(status.begin(), status.end(),
                                            [](unsigned char c) { return std::isdigit(c) != 0; })
              ? "caster rejected: status " + status
              : "caster rejected request";
      close_fd();
      return false;
    }
    *leftover = hp.leftover;
    *out_fd = fd;
    return true;
  }
}

void NtripClient::run() {
  if (!cfg_.security) {
    set_state(NtripState::Error, "NTRIP security must be explicit");
    return;
  }
  int attempt = 0;
  RtcmParser parser(cfg_.max_buffer_bytes);
  while (!stop_) {
    set_state(NtripState::Connecting);
    int fd = -1;
    std::vector<uint8_t> pending;
    std::string error;
    bool had_error = false;
    if (connect_and_handshake(&fd, &pending, &error)) {
      attempt = 0;
      parser.clear();
      set_state(NtripState::Streaming);
      double next_gga =
          now_s();  // VRS casters withhold RTCM until the first position: feed at once
      double last_rx = now_s();
      auto deliver = [&](const uint8_t* d, size_t n) {
        for (const auto& f : parser.feed(d, n)) {
          if (sink_) sink_(f);
        }
        std::lock_guard<std::mutex> lk(m_);
        snap_.frames = parser.frames();
        snap_.crc_failures = parser.crc_failures();
        snap_.resync_bytes = parser.resync_bytes();
      };
      if (!pending.empty()) {
        std::lock_guard<std::mutex> lk(m_);
        snap_.bytes += pending.size();
      }
      if (!pending.empty()) deliver(pending.data(), pending.size());
      while (!stop_) {
        const double t = now_s();
        if (t >= next_gga) {
          next_gga = t + cfg_.gga_interval_s;
          const auto gga = gga_ ? gga_() : std::nullopt;
          if (gga) {
            // DERIVED: bound a GGA write by the existing stream-liveness timeout.
            const auto result = send_all(fd, tls_, gga->data(), gga->size(), wake_[0], stop_,
                                         now_s() + cfg_.stream_timeout_s);
            if (result == SendResult::Stopped) break;
            if (result == SendResult::Complete) {
              std::lock_guard<std::mutex> lk(m_);
              ++snap_.gga_sent;
            } else {
              // Some bytes may already be on the wire. Reconnect rather than start a new
              // sentence behind an incomplete GGA on the same stream.
              error = result == SendResult::Timeout ? "GGA send timeout" : "GGA send failed";
              had_error = true;
              break;
            }
          }
        }
        const double until_gga = std::max(0.0, next_gga - now_s());
        const double until_timeout = std::max(0.0, cfg_.stream_timeout_s - (now_s() - last_rx));
        uint8_t buf[4096];
        const ssize_t n = read_some(fd, tls_, buf, sizeof buf, wake_[0], stop_,
                                    now_s() + std::min(until_gga, until_timeout) + 0.001);
        if (n == -2) break;  // stop requested
        if (n == -3) {
          if (now_s() - last_rx >= cfg_.stream_timeout_s) {
            error = "stream timeout";
            had_error = true;
            break;
          }
          continue;
        }
        if (n > 0) {
          last_rx = now_s();
          {
            std::lock_guard<std::mutex> lk(m_);
            snap_.bytes += static_cast<uint64_t>(n);
          }
          deliver(buf, static_cast<size_t>(n));
        } else if (n == 0) {
          error = "stream ended";
          had_error = true;
          break;
        } else {
          error = "stream read failed";
          had_error = true;
          break;
        }
      }
      close_fd();
    } else {
      had_error = !stop_;
    }
    if (stop_) break;
    if (had_error) {
      {
        std::lock_guard<std::mutex> lk(m_);
        ++snap_.reconnects;
      }
      set_state(NtripState::Error, error);
    }
    const double wait = backoff_s(attempt++, cfg_.backoff_base_s, cfg_.backoff_max_s);
    set_state(NtripState::Reconnecting,
              "reconnecting in " + std::to_string(static_cast<int>(wait)) + " s");
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait_for(lk, std::chrono::duration<double>(wait), [this] { return stop_.load(); });
  }
}

}  // namespace dyx3_gnss_rtk
