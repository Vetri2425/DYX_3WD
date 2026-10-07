#include "dyx3_gnss_rtk/rtcm_parser.hpp"

#include <array>

namespace dyx3_gnss_rtk {
namespace {

constexpr uint8_t kPreamble = 0xD3;
constexpr size_t kHeaderLen = 3;
constexpr size_t kCrcLen = 3;

const std::array<uint32_t, 256>& table() {
  static const std::array<uint32_t, 256> t = [] {
    std::array<uint32_t, 256> out{};
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t crc = i << 16;
      for (int k = 0; k < 8; ++k) {
        crc = (crc & 0x800000U) ? ((crc << 1) ^ 0x1864CFBU) : (crc << 1);
        crc &= 0xFFFFFFU;
      }
      out[i] = crc;
    }
    return out;
  }();
  return t;
}

}  // namespace

uint32_t crc24q(const uint8_t* data, size_t length) {
  const auto& t = table();
  uint32_t crc = 0;
  for (size_t i = 0; i < length; ++i)
    crc = ((crc << 8) ^ t[((crc >> 16) & 0xFFU) ^ data[i]]) & 0xFFFFFFU;
  return crc;
}

std::vector<std::vector<uint8_t>> RtcmParser::feed(const uint8_t* data, size_t length) {
  buf_.insert(buf_.end(), data, data + length);
  std::vector<std::vector<uint8_t>> frames;
  size_t i = 0;
  while (i < buf_.size()) {
    if (buf_[i] != kPreamble) {
      ++i;
      ++resync_bytes_;
      continue;
    }
    if (i + kHeaderLen > buf_.size()) break;  // partial header
    const size_t length_field = (static_cast<size_t>(buf_[i + 1]) << 8) | buf_[i + 2];
    const size_t msg_len =
        length_field & 0x03FFU;  // reserved bits (0xFC00) tolerated, as the prototype does
    const size_t total = kHeaderLen + msg_len + kCrcLen;
    if (i + total > buf_.size()) break;  // incomplete frame
    const size_t payload_len = kHeaderLen + msg_len;
    const uint32_t expected = (static_cast<uint32_t>(buf_[i + payload_len]) << 16) |
                              (static_cast<uint32_t>(buf_[i + payload_len + 1]) << 8) |
                              buf_[i + payload_len + 2];
    if (crc24q(&buf_[i], payload_len) != expected) {
      ++crc_failures_;
      ++i;  // skip this preamble and keep scanning
      ++resync_bytes_;
      continue;
    }
    frames.emplace_back(buf_.begin() + static_cast<std::ptrdiff_t>(i),
                        buf_.begin() + static_cast<std::ptrdiff_t>(i + total));
    ++frames_;
    i += total;
  }
  buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(i));
  if (buf_.size() > max_buffer_) {
    const size_t drop = buf_.size() - max_buffer_;
    resync_bytes_ += drop;
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(drop));
  }
  return frames;
}

}  // namespace dyx3_gnss_rtk
