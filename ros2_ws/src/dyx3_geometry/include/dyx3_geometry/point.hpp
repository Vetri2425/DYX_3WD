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
///
/// Lifetime: the view does not own or copy the points. The storage must outlive every use of the
/// view and must not be resized, reallocated or modified while a call that takes it is running
/// (a `std::vector` push_back/resize/swap invalidates `pts`). Do not build one from a temporary
/// that dies before the call returns, and do not keep a `PathView` across a path replacement: the
/// owner must re-create it (and reset any `ProjectionHint`) when the path changes.
///
/// Bounds: `pts` must point to at least `n` valid `Point`s (it may be null only when `n == 0`).
/// `operator[]` is unchecked, so `i < n` is the caller's responsibility. The library functions
/// clamp or reject their own indices (e.g. `curvature_at`, `project_onto_segment`) and treat
/// `n == 0` as an empty path, but they cannot detect a `n` that is larger than the real storage
/// or a dangling pointer.
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
