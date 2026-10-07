// resample — see docs/contracts/dyx3_geometry.md
#pragma once

#include <vector>

#include "dyx3_geometry/point.hpp"

namespace dyx3_geometry {

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
/// * n < 2 or spacing <= 0 returns the input unchanged; total < spacing returns the endpoints.
/// * `flags` may be null (all-false); a length mismatch is treated as all-false (ancestor).
///
/// Allocates: NOT for the control loop (mission install only).
ResampleResult resample(const std::vector<Point>& pts, double spacing,
                        const std::vector<unsigned char>* flags = nullptr);

}  // namespace dyx3_geometry
