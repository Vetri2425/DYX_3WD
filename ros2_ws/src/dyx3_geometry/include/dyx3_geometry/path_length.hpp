// path_length / cumulative_lengths — see docs/contracts/dyx3_geometry.md
#pragma once

#include <cstddef>
#include <vector>

#include "dyx3_geometry/point.hpp"

namespace dyx3_geometry {

/// Sum of segment lengths, accumulated left to right (ancestor: `_pts_length`). 0 for n < 2.
double path_length(PathView path);

/// Cumulative arc length at each vertex: out[0] = 0, out[i] = out[i-1] + |p[i]-p[i-1]|
/// (ancestor: `_pts_cumulative_lengths`). `out` must hold at least `path.n` doubles; nothing is
/// allocated. Returns the number of values written (`path.n`; 1 for an empty path writes out[0]
/// = 0 to match the ancestor, which always returns [0.0]).
std::size_t cumulative_lengths(PathView path, double* out, std::size_t out_capacity);

/// Allocating convenience for non-real-time callers (mission install).
std::vector<double> cumulative_lengths(PathView path);

}  // namespace dyx3_geometry
