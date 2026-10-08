#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace dyx3_gnss_rtk {

// The only type a direct receiver sink accepts. Construction checks the complete RTCM3
// envelope and CRC-24Q, even when the source already used RtcmParser.
class ValidatedFrame {
public:
  static std::optional<ValidatedFrame> parse(const std::vector<uint8_t>& bytes);

  const std::vector<uint8_t>& bytes() const { return bytes_; }

private:
  explicit ValidatedFrame(std::vector<uint8_t> bytes) : bytes_(std::move(bytes)) {}
  std::vector<uint8_t> bytes_;
};

}  // namespace dyx3_gnss_rtk
