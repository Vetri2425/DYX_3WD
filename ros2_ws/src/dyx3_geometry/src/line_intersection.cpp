// line_intersection — see docs/contracts/dyx3_geometry.md
#include "dyx3_geometry/line_intersection.hpp"

#include <cmath>

namespace dyx3_geometry {

std::optional<Point> line_intersection(Point a, Point b, Point c, Point d) {
  const double d1x = b.n - a.n;
  const double d1y = b.e - a.e;
  const double d2x = d.n - c.n;
  const double d2y = d.e - c.e;
  const double denom = d1x * d2y - d1y * d2x;
  if (std::fabs(denom) < 1e-9) {
    return std::nullopt;
  }
  const double t = ((c.n - a.n) * d2y - (c.e - a.e) * d2x) / denom;
  return Point{a.n + t * d1x, a.e + t * d1y};
}

}  // namespace dyx3_geometry
