#include "dyx3_gnss_rtk/rtcm_transport.hpp"

#include <algorithm>

namespace dyx3_gnss_rtk {

std::vector<Chunk> Chunker::split(const std::vector<uint8_t>& frame) {
  std::vector<Chunk> out;
  if (frame.empty() || frame.size() > kMaxChunkBytes * kMaxFragments) {
    ++rejected_;
    return out;
  }
  const size_t n = (frame.size() + kMaxChunkBytes - 1) / kMaxChunkBytes;
  const uint8_t seq = seq_;
  seq_ = static_cast<uint8_t>((seq_ + 1) & 0x1F);
  for (size_t k = 0; k < n; ++k) {
    Chunk c;
    const size_t begin = k * kMaxChunkBytes;
    const size_t end = std::min(frame.size(), begin + kMaxChunkBytes);
    c.data.assign(frame.begin() + static_cast<std::ptrdiff_t>(begin),
                  frame.begin() + static_cast<std::ptrdiff_t>(end));
    c.flags = static_cast<uint8_t>((n > 1 ? 1U : 0U) | (static_cast<uint8_t>(k) << 1) |
                                   (static_cast<uint8_t>(seq) << 3));
    out.push_back(std::move(c));
  }
  return out;
}

}  // namespace dyx3_gnss_rtk
