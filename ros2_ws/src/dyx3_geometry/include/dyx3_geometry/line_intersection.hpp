// line_intersection — see docs/contracts/dyx3_geometry.md
#pragma once

#include <optional>

#include "dyx3_geometry/point.hpp"

namespace dyx3_geometry {

/// Intersection of the infinite line a->b with the infinite line c->d, or nullopt when the
/// directions are near-parallel (|d1 x d2| < 1e-9) (ancestor: `_line_intersection`).
std::optional<Point> line_intersection(Point a, Point b, Point c, Point d);

}  // namespace dyx3_geometry
