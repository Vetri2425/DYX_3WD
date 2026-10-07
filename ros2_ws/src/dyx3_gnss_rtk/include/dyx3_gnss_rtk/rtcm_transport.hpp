// rtcm_transport — chunking of RTCM frames into GpsInjectData-sized pieces with the MAVLink
// GPS_RTCM_DATA flags the GPS driver's reassembly expects. Contract:
// docs/contracts/dyx3_gnss_rtk.md section 3. Pure C++.
//   flags: bit 0 = fragmented, bits 1-2 = fragment id, bits 3-7 = sequence id (increments per frame
//   mod 32).
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace dyx3_gnss_rtk {

struct Chunk {
  uint8_t flags{0};
  std::vector<uint8_t> data;
};

constexpr size_t kMaxChunkBytes = 300;  // GpsInjectData.data[300]
constexpr size_t kMaxFragments = 4;     // fragment id is 2 bits

class Chunker {
public:
  // A frame of at most 300 bytes is one chunk; a longer one is split into at most 4 chunks of at
  // most 300 bytes. A frame that would need more than 4 chunks (> 1200 bytes) or is empty is
  // REJECTED (empty result, counted) and never truncated: a truncated RTCM frame is garbage to the
  // receiver.
  std::vector<Chunk> split(const std::vector<uint8_t>& frame);

  uint8_t next_sequence() const { return seq_; }
  uint64_t rejected_frames() const { return rejected_; }

private:
  uint8_t seq_{0};
  uint64_t rejected_{0};
};

}  // namespace dyx3_gnss_rtk
