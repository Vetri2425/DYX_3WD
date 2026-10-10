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

/// `resample` with FIXED KNOTS: every index in `fixed` (plus both endpoints) is emitted verbatim
/// (exact coordinates, own flag) and the path is resampled independently inside each span between
/// consecutive knots, so a must-hit vertex survives resampling. This is the single implementation:
/// `resample(pts, s, f)` is `resample_fixed(pts, s, f, {})` and is bit-identical to the ancestor.
///
/// * Each span of cumulative length L gets `max(2, ceil(L/spacing) + 1)` samples, so the spacing
///   inside a span is L/(count-1): at most `spacing`, and equal to it only when L is a multiple.
/// * `fixed` may be unsorted or contain duplicates, 0, n-1 or out-of-range values (all ignored
///   or merged). Two knots closer than 1e-12 m (zero-length span) emit one sample.
/// * The refusals of `resample` apply to the whole path (the total sample count is bounded by
///   `kResampleMaxSamples`; a refused call returns the input unchanged, so every knot survives).
/// * Interior flag = flag[seg] AND flag[seg+1]; a knot keeps its own flag.
ResampleResult resample_fixed(const std::vector<Point>& pts, double spacing,
                              const std::vector<unsigned char>* flags,
                              const std::vector<std::size_t>& fixed);

}  // namespace dyx3_geometry
