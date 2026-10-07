// perpendicular_distance — see docs/contracts/dyx3_geometry.md
#pragma once

#include "dyx3_geometry/point.hpp"

namespace dyx3_geometry {

/// Perpendicular distance of p from the INFINITE line a->b (unsigned). When |ab| < 1e-9 the
/// distance to a (ancestor: `_perp_dist`).
double perpendicular_distance(Point p, Point a, Point b);

}  // namespace dyx3_geometry
