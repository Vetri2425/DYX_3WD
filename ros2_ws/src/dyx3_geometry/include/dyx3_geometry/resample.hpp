// resample — see docs/contracts/dyx3_geometry.md
#pragma once

#include <cstddef>
#include <vector>

#include "dyx3_geometry/point.hpp"

namespace dyx3_geometry {

/// Upper bound on the number of output samples (16 MB of points). A request that would need more
/// is refused (input returned unchanged), never truncated: 1e6 samples is a 10 km path at 1 cm.
constexpr std::size_t kResampleMaxSamples = 1000000;

struct ResampleResult {
  std::vector<Point> pts;
  std::vector<unsigned char> flags;  ///< same length as pts (all 0 when no flags were supplied)
};

/// Uniform arc-length resampling of a polyline (ancestor: `_resample_path`).
///
/// * Endpoints are kept EXACTLY; interior samples every `spacing` metres along the cumulative
///   length; sample count `max(2, ceil(total/spacing) + 1)`.
/// * Straight segments stay straight (linear interpolation).
/// * Interior flag = flag[seg] AND flag[seg+1]; first/last keep their own flag.
/// * n < 2, a non-finite or <= 0 `spacing`, a non-finite path length, or a sample count above
///   `kResampleMaxSamples` returns the input unchanged (the caller must compare sizes if it needs
///   to tell a refusal from a result); total < spacing returns the endpoints.
/// * `flags` may be null (all-false); a length mismatch is treated as all-false (ancestor).
///
/// Allocates: NOT for the control loop (mission install only).
ResampleResult resample(const std::vector<Point>& pts, double spacing,
                        const std::vector<unsigned char>* flags = nullptr);

}  // namespace dyx3_geometry
