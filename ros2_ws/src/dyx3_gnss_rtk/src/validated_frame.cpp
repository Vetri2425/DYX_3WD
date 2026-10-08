#include "dyx3_gnss_rtk/validated_frame.hpp"

#include <utility>

#include "dyx3_gnss_rtk/rtcm_parser.hpp"

namespace dyx3_gnss_rtk {

std::optional<ValidatedFrame> ValidatedFrame::parse(const std::vector<uint8_t>& bytes) {
  if (bytes.size() < 6 || bytes.size() > 1029 || bytes[0] != 0xD3) return std::nullopt;
  // Preserve the existing parser's reserved-bit tolerance. Length and CRC still have to match.
  const size_t payload = (static_cast<size_t>(bytes[1] & 0x03) << 8) | bytes[2];
  if (bytes.size() != payload + 6) return std::nullopt;
  const size_t crc_at = bytes.size() - 3;
  const uint32_t wire = (static_cast<uint32_t>(bytes[crc_at]) << 16) |
                        (static_cast<uint32_t>(bytes[crc_at + 1]) << 8) | bytes[crc_at + 2];
  if (crc24q(bytes.data(), crc_at) != wire) return std::nullopt;
  return ValidatedFrame(bytes);
}

}  // namespace dyx3_gnss_rtk
