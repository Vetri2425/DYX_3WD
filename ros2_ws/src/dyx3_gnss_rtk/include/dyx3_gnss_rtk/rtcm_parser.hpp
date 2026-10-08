// rtcm_parser — RTCM3 frame extraction. Contract: docs/contracts/dyx3_gnss_rtk.md section 3. Pure
// C++. Behaviour carried from the prototype (ntrip_rtcm_node.py::_parse_rtcm_frames): scan for
// 0xD3, 10-bit length (reserved bits tolerated), a frame is 3 + len + 3 bytes, a CRC-24Q mismatch
// discards the frame and resumes the scan ONE BYTE after that preamble, an incomplete frame waits
// for more data.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace dyx3_gnss_rtk {

// CRC-24Q (polynomial 0x1864CFB) over data[0, length).
uint32_t crc24q(const uint8_t* data, size_t length);
// 12-bit RTCM message number from a complete frame, or 0 for a payload shorter than 2 bytes.
uint16_t rtcm_message_type(const std::vector<uint8_t>& frame);

class RtcmParser {
public:
  // max_buffer_bytes: DERIVED hardening. A stream that never yields a valid frame must not grow
  // without bound; on overflow the oldest bytes are dropped and counted as resynchronised.
  explicit RtcmParser(size_t max_buffer_bytes = 8192) : max_buffer_(max_buffer_bytes) {}

  // Appends bytes and returns every complete CRC-valid frame now available (each includes preamble,
  // length and CRC).
  std::vector<std::vector<uint8_t>> feed(const uint8_t* data, size_t length);

  uint64_t frames() const { return frames_; }
  uint64_t crc_failures() const { return crc_failures_; }
  uint64_t invalid_headers() const { return invalid_headers_; }
  uint64_t resync_bytes() const { return resync_bytes_; }
  size_t buffered() const { return buf_.size(); }
  void clear() { buf_.clear(); }

private:
  size_t max_buffer_;
  std::vector<uint8_t> buf_;
  uint64_t frames_{0};
  uint64_t crc_failures_{0};
  uint64_t invalid_headers_{0};
  uint64_t resync_bytes_{0};
};

}  // namespace dyx3_gnss_rtk
