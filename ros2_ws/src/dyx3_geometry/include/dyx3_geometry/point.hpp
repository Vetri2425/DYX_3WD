// Point — local NED metres. See docs/contracts/dyx3_geometry.md and docs/contracts/frames.md.
#pragma once

#include <cstddef>
#include <vector>

namespace dyx3_geometry {

/// A position in local NED, metres: `n` = North (+X), `e` = East (+Y).
/// Matches the prototype's `(pose.position.x, pose.position.y)` of a path pose.
struct Point {
  double n = 0.0;
  double e = 0.0;
};

/// Non-owning view of a polyline (C++17 has no std::span). Hot-path functions take this and never
/// allocate.
struct PathView {
  const Point* pts = nullptr;
  std::size_t n = 0;

  PathView() = default;
  PathView(const Point* p, std::size_t count) : pts(p), n(count) {}
  PathView(const std::vector<Point>& v)  // NOLINT(google-explicit-constructor): intentional
      : pts(v.data()), n(v.size()) {}
  const Point& operator[](std::size_t i) const { return pts[i]; }
};

}  // namespace dyx3_geometry
