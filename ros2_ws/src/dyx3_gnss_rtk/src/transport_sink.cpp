#include "dyx3_gnss_rtk/transport_sink.hpp"

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <sstream>

#include "dyx3_gnss_rtk/gga_provider.hpp"
#include "dyx3_gnss_rtk/serial_port.hpp"

namespace dyx3_gnss_rtk {
namespace {
double steady_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool valid_gga(const std::string& line) {
  if (line.size() < 12 || line.size() > 120 || line.front() != '$' ||
      line.compare(3, 3, "GGA") != 0)
    return false;
  const size_t star = line.find('*');
  if (star == std::string::npos || star + 3 != line.size()) return false;
  return nmea_checksum(line.substr(1, star - 1)) == line.substr(star + 1, 2);
}

std::optional<ReceiverReadback> parse_gga(const std::string& line, double now_s) {
  if (!valid_gga(line)) return std::nullopt;
  std::vector<std::string> fields;
  std::istringstream input(line.substr(1, line.find('*') - 1));
  std::string part;
  while (std::getline(input, part, ',')) fields.push_back(part);
  if (fields.size() < 14) return std::nullopt;
  try {
    ReceiverReadback result;
    result.fix_quality = std::stoi(fields[6]);
    result.satellites = std::stoi(fields[7]);
    result.hdop = std::stod(fields[8]);
    if (result.fix_quality < 0 || result.fix_quality > 9 || result.satellites < 0 ||
        result.satellites > 99 || !std::isfinite(result.hdop) || result.hdop < 0)
      return std::nullopt;
    if (!fields[13].empty()) {
      const double age = std::stod(fields[13]);
      if (std::isfinite(age) && age >= 0) result.correction_age_s = age;
    }
    result.received_at_s = now_s;
    return result;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}
}  // namespace

UsbSerialSink::UsbSerialSink(UsbConfig config, std::string by_id_prefix)
    : config_(std::move(config)), by_id_prefix_(std::move(by_id_prefix)) {}

bool UsbSerialSink::open(double now_s) {
  std::lock_guard<std::mutex> lk(m_);
  if (fd_ >= 0) return true;
  if (config_.receiver_device.empty()) return false;
  if (now_s < next_open_s_) return false;
  fd_ = open_serial_exclusive(config_.receiver_device, config_.baud, by_id_prefix_);
  if (fd_ < 0) {
    ++counters_.failures;
    next_open_s_ = now_s + config_.reopen_delay_s;
    return false;
  }
  ++counters_.opens;
  if (ever_opened_) ++counters_.reopens;
  ever_opened_ = true;
  line_buffer_.clear();
  gga_.clear();
  gga_at_s_ = -1;
  readback_ = ReceiverReadback{};
  return true;
}

bool UsbSerialSink::deliver(const ValidatedFrame& frame, double now_s) {
  if (!open(now_s)) return false;
  std::lock_guard<std::mutex> lk(m_);
  const auto& bytes = frame.bytes();
  size_t offset = 0;
  const double deadline = steady_s() + config_.write_timeout_s;
  while (offset < bytes.size()) {
    const ssize_t n = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
    if (n > 0) {
      offset += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) continue;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      const double left = deadline - steady_s();
      if (left > 0) {
        pollfd p{fd_, POLLOUT, 0};
        const int r = ::poll(&p, 1, static_cast<int>(std::ceil(left * 1000.0)));
        if (r > 0 && (p.revents & POLLOUT) != 0) continue;
        if (r < 0 && errno == EINTR) continue;
      }
    }
    // A partial frame has already gone to the device. Never restart at byte zero.
    ++counters_.failures;
    close_locked();
    next_open_s_ = now_s + config_.reopen_delay_s;
    return false;
  }
  ++counters_.frames_delivered;
  counters_.bytes_delivered += bytes.size();
  counters_.last_delivery_s = now_s;
  return true;
}

void UsbSerialSink::close_locked() {
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
  line_buffer_.clear();
  gga_.clear();
  gga_at_s_ = -1;
  readback_ = ReceiverReadback{};
}

void UsbSerialSink::close() {
  std::lock_guard<std::mutex> lk(m_);
  close_locked();
}

bool UsbSerialSink::active() const {
  std::lock_guard<std::mutex> lk(m_);
  return fd_ >= 0;
}

SinkCounters UsbSerialSink::counters() const {
  std::lock_guard<std::mutex> lk(m_);
  return counters_;
}

void UsbSerialSink::read_available(double now_s) {
  std::lock_guard<std::mutex> lk(m_);
  if (fd_ < 0) return;
  pollfd p{fd_, POLLIN, 0};
  const int ready = ::poll(&p, 1, 0);
  if (ready == 0 || (ready < 0 && errno == EINTR)) return;
  if (ready < 0 || (p.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    ++counters_.failures;
    close_locked();
    next_open_s_ = now_s + config_.reopen_delay_s;
    return;
  }
  if ((p.revents & POLLIN) == 0) return;
  uint8_t buf[256];
  for (;;) {
    const ssize_t n = ::read(fd_, buf, sizeof buf);
    if (n > 0) {
      for (ssize_t i = 0; i < n; ++i) {
        const char c = static_cast<char>(buf[i]);
        if (c == '\n') {
          if (!line_buffer_.empty() && line_buffer_.back() == '\r') line_buffer_.pop_back();
          if (auto parsed = parse_gga(line_buffer_, now_s)) {
            gga_ = line_buffer_ + "\r\n";
            gga_at_s_ = now_s;
            readback_ = *parsed;
          }
          line_buffer_.clear();
        } else if (line_buffer_.size() < 120) {
          line_buffer_ += c;
        } else {
          line_buffer_.clear();
        }
      }
      continue;
    }
    if (n == 0) return;  // VMIN=0: no more bytes available is not a disconnect.
    if (n < 0 && errno == EINTR) continue;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
    ++counters_.failures;
    close_locked();
    next_open_s_ = now_s + config_.reopen_delay_s;
    return;
  }
}

std::string UsbSerialSink::last_gga(double now_s, double max_age_s) const {
  std::lock_guard<std::mutex> lk(m_);
  return readback_.fix_quality > 0 && gga_at_s_ >= 0 && now_s >= gga_at_s_ &&
                 now_s - gga_at_s_ <= max_age_s
             ? gga_
             : std::string();
}

std::optional<ReceiverReadback> UsbSerialSink::readback(double now_s, double max_age_s) const {
  std::lock_guard<std::mutex> lk(m_);
  if (readback_.received_at_s < 0 || now_s < readback_.received_at_s ||
      now_s - readback_.received_at_s > max_age_s)
    return std::nullopt;
  return readback_;
}

bool DdsSink::open(double) {
  if (active_) return true;
  try {
    if (!ready_ || !ready_()) return false;
  } catch (const std::exception&) {
    ++counters_.failures;
    return false;
  }
  active_ = true;
  ++counters_.opens;
  return true;
}

bool DdsSink::deliver(const ValidatedFrame& frame, double now_s) {
  bool ready = false;
  try {
    ready = open(now_s) && ready_();
  } catch (const std::exception&) {
    ready = false;
  }
  if (!ready) {
    active_ = false;
    ++counters_.failures;
    return false;
  }
  const auto chunks = chunker_.split(frame.bytes());
  if (chunks.empty()) {
    ++counters_.failures;
    return false;
  }
  for (const auto& chunk : chunks) {
    bool published = false;
    try {
      published = publish_ && publish_(chunk);
    } catch (const std::exception&) {
      published = false;
    }
    if (!published) {
      ++counters_.failures;
      return false;
    }
  }
  ++counters_.frames_delivered;
  counters_.bytes_delivered += frame.bytes().size();
  counters_.last_delivery_s = now_s;
  return true;
}

}  // namespace dyx3_gnss_rtk
