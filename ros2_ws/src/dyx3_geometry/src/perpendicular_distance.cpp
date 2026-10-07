// perpendicular_distance — see docs/contracts/dyx3_geometry.md
#include "dyx3_geometry/perpendicular_distance.hpp"

#include <cmath>

namespace dyx3_geometry {

double perpendicular_distance(Point p, Point a, Point b) {
  const double dx = b.n - a.n;
  const double dy = b.e - a.e;
  const double h = std::hypot(dx, dy);
  if (h < 1e-9) {
    return std::hypot(p.n - a.n, p.e - a.e);
  }
  return std::fabs(dx * (a.e - p.e) - dy * (a.n - p.n)) / h;
}

}  // namespace dyx3_geometry
