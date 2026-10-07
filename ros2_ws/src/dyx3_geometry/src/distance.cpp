// distance — see docs/contracts/dyx3_geometry.md
#include "dyx3_geometry/distance.hpp"

#include <cmath>

namespace dyx3_geometry {

double distance(Point a, Point b) { return std::hypot(a.n - b.n, a.e - b.e); }

}  // namespace dyx3_geometry
