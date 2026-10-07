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
#include <cstring>

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
  const int ms =
      timeout_s <= 0.0 ? 0 : static_cast<int>(std::min(timeout_s, 3600.0) * 1000.0 + 0.5);
  for (;;) {
    const int r = ::poll(p, wake_fd >= 0 ? 2 : 1, ms);
    if (r < 0 && errno == EINTR) continue;
    if (r < 0) return -1;
    if (r == 0) return 0;
    if (wake_fd >= 0 && (p[1].revents & POLLIN) != 0) return -1;
    if ((p[0].revents & (events | POLLERR | POLLHUP)) != 0) return 1;
    return 0;
  }
}

std::string short_errno(const char* what) {
  return std::string(what) + ": " + std::strerror(errno);
}

}  // namespace

NtripClient::NtripClient(NtripConfig cfg, FrameSink sink, GgaSource gga)
    : cfg_(std::move(cfg)), sink_(std::move(sink)), gga_(std::move(gga)) {
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
  const int fd = active_fd_.exchange(-1);
  if (fd >= 0) ::shutdown(fd, SHUT_RDWR);
  if (thread_.joinable()) thread_.join();
  set_state(NtripState::Stopped);
}

void NtripClient::close_fd() {
  const int fd = active_fd_.exchange(-1);
  if (fd >= 0) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }
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
  for (addrinfo* a = res; a != nullptr && fd < 0; a = a->ai_next) {
    const int s = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
    if (s < 0) continue;
    ::fcntl(s, F_SETFL, ::fcntl(s, F_GETFL, 0) | O_NONBLOCK);
    active_fd_ = s;
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
      active_fd_ = -1;
      ::close(s);
    }
  }
  ::freeaddrinfo(res);
  if (fd < 0) {
    *error = stop_ ? "stopped" : short_errno("connect failed");
    return false;
  }
  int one = 1;
  ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);

  const std::string req = build_request(cfg_.host, cfg_.mountpoint, cfg_.user, cfg_.password);
  size_t sent = 0;
  const double deadline = now_s() + cfg_.connect_timeout_s;
  while (sent < req.size()) {
    const ssize_t n = ::send(fd, req.data() + sent, req.size() - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      if (wait_fd(fd, POLLOUT, wake_[0], deadline - now_s()) != 1) break;
      continue;
    }
    break;
  }
  if (sent < req.size()) {
    *error = stop_ ? "stopped" : "request send failed";
    close_fd();
    return false;
  }

  std::vector<uint8_t> rx;
  for (;;) {
    const double left = deadline - now_s();
    if (left <= 0.0) {
      *error = "handshake timeout";
      close_fd();
      return false;
    }
    const int w = wait_fd(fd, POLLIN, wake_[0], left);
    if (w <= 0) {
      *error = stop_ ? "stopped" : "handshake timeout";
      close_fd();
      return false;
    }
    uint8_t buf[512];
    const ssize_t n = ::recv(fd, buf, sizeof buf, 0);
    if (n == 0) {
      *error = "caster closed the connection during the handshake";
      close_fd();
      return false;
    }
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
      *error = short_errno("handshake read failed");
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
      const auto nl = hp.header.find_first_of("\r\n");
      std::string first = hp.header.substr(0, nl);
      if (first.size() > 60) first.resize(60);
      for (auto& c : first) {
        if (static_cast<unsigned char>(c) < 0x20 || static_cast<unsigned char>(c) > 0x7e) c = '?';
      }
      *error = "caster rejected: " + (first.empty() ? std::string("empty response") : first);
      close_fd();
      return false;
    }
    *leftover = hp.leftover;
    *out_fd = fd;
    return true;
  }
}

void NtripClient::run() {
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
      int gga_failures = 0;
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
            const ssize_t n = ::send(fd, gga->data(), gga->size(), MSG_NOSIGNAL);
            if (n == static_cast<ssize_t>(gga->size())) {
              gga_failures = 0;
              std::lock_guard<std::mutex> lk(m_);
              ++snap_.gga_sent;
            } else {
              ++gga_failures;
              if (gga_failures >= 3) {  // a failed back-feed usually means a half-open stream
                error = "GGA back-feed failing: stream looks half-open";
                had_error = true;
                break;
              }
            }
          }
        }
        const double until_gga = std::max(0.0, next_gga - now_s());
        const double until_timeout = std::max(0.0, cfg_.stream_timeout_s - (now_s() - last_rx));
        const int w = wait_fd(fd, POLLIN, wake_[0], std::min(until_gga, until_timeout) + 0.001);
        if (w < 0) break;  // stop requested
        if (w == 0) {
          if (now_s() - last_rx >= cfg_.stream_timeout_s) {
            error = "stream timeout";
            had_error = true;
            break;
          }
          continue;
        }
        uint8_t buf[4096];
        const ssize_t n = ::recv(fd, buf, sizeof buf, 0);
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
        } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          error = short_errno("stream read failed");
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
