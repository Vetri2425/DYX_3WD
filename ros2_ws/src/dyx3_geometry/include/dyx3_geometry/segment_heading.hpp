// segment_heading — see docs/contracts/dyx3_geometry.md
#pragma once

#include "dyx3_geometry/point.hpp"

namespace dyx3_geometry {

/// NED heading of the directed segment a->b: atan2(east, north); 0 = North, clockwise-positive
/// (ancestor: `_segment_heading`). Degenerate a == b returns 0 (atan2(0, 0)).
double segment_heading(Point a, Point b);

}  // namespace dyx3_geometry
