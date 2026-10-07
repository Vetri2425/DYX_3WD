// segment_heading — see docs/contracts/dyx3_geometry.md
#include "dyx3_geometry/segment_heading.hpp"

#include <cmath>

namespace dyx3_geometry {

double segment_heading(Point a, Point b) { return std::atan2(b.e - a.e, b.n - a.n); }

}  // namespace dyx3_geometry
