#include "dyx3_gnss_rtk/lora_source.hpp"

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>

#include "dyx3_gnss_rtk/serial_port.hpp"

namespace dyx3_gnss_rtk {
namespace {
double steady_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

LoraSource::LoraSource(LoraConfig config, FrameSink sink, std::string by_id_prefix)
    : config_(std::move(config)), sink_(std::move(sink)), by_id_prefix_(std::move(by_id_prefix)) {
  if (::pipe(wake_) == 0) {
    ::fcntl(wake_[0], F_SETFL, O_NONBLOCK);
    ::fcntl(wake_[1], F_SETFL, O_NONBLOCK);
  }
}

LoraSource::~LoraSource() {
  stop();
  for (int& fd : wake_) {
    if (fd >= 0) ::close(fd);
    fd = -1;
  }
}

void LoraSource::start() {
  if (worker_.joinable()) return;
  stopping_ = false;
  worker_ = std::thread([this] { run(); });
}

void LoraSource::stop() {
  stopping_ = true;
  if (wake_[1] >= 0) {
    const char c = 1;
    [[maybe_unused]] const ssize_t n = ::write(wake_[1], &c, 1);
  }
  if (worker_.joinable()) worker_.join();
}

LoraSnapshot LoraSource::snapshot() const {
  std::lock_guard<std::mutex> lk(m_);
  return snapshot_;
}

void LoraSource::close_fd() {
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
  std::lock_guard<std::mutex> lk(m_);
  snapshot_.port_open = false;
}

void LoraSource::run() {
  RtcmParser parser;
  while (!stopping_) {
    fd_ = open_serial_exclusive(config_.serial_device, config_.baud, by_id_prefix_);
    if (fd_ >= 0) {
      parser.clear();
      {
        std::lock_guard<std::mutex> lk(m_);
        if (snapshot_.opens > 0) ++snapshot_.reopens;
        ++snapshot_.opens;
        snapshot_.port_open = true;
      }
      double last_rx = steady_s();
      while (!stopping_) {
        pollfd p[2]{{fd_, POLLIN, 0}, {wake_[0], POLLIN, 0}};
        const int n = ::poll(p, wake_[0] >= 0 ? 2 : 1, 100);
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 || (p[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
          std::lock_guard<std::mutex> lk(m_);
          ++snapshot_.read_errors;
          break;
        }
        if (wake_[0] >= 0 && (p[1].revents & POLLIN) != 0) break;
        if ((p[0].revents & POLLIN) != 0) {
          uint8_t bytes[4096];
          const ssize_t count = ::read(fd_, bytes, sizeof bytes);
          if (count > 0) {
            last_rx = steady_s();
            const auto frames = parser.feed(bytes, static_cast<size_t>(count));
            {
              std::lock_guard<std::mutex> lk(m_);
              snapshot_.bytes_received += static_cast<uint64_t>(count);
              snapshot_.valid_frames = parser.frames();
              snapshot_.crc_failures = parser.crc_failures();
              snapshot_.invalid_headers = parser.invalid_headers();
              snapshot_.resync_bytes = parser.resync_bytes();
              if (!frames.empty()) snapshot_.last_message_type = rtcm_message_type(frames.back());
            }
            for (const auto& frame : frames)
              if (sink_) sink_(frame);
          } else if (count == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) {
            std::lock_guard<std::mutex> lk(m_);
            ++snapshot_.read_errors;
            break;
          }
        }
        if (steady_s() - last_rx >= config_.read_timeout_s) {
          if (parser.buffered() > 0) {
            parser.clear();
            std::lock_guard<std::mutex> lk(m_);
            ++snapshot_.partial_timeouts;
          }
          last_rx = steady_s();
        }
      }
      close_fd();
    } else {
      std::lock_guard<std::mutex> lk(m_);
      ++snapshot_.read_errors;
    }
    if (stopping_) break;
    const double until = steady_s() + config_.reopen_delay_s;
    while (!stopping_ && steady_s() < until) {
      pollfd p{wake_[0], POLLIN, 0};
      const int ms = static_cast<int>(std::ceil((until - steady_s()) * 1000.0));
      if (::poll(&p, 1, ms > 100 ? 100 : ms) > 0) break;
    }
  }
  close_fd();
}

}  // namespace dyx3_gnss_rtk
