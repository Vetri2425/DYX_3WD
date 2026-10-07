// project_onto_segment — see docs/contracts/dyx3_geometry.md
#pragma once

#include "dyx3_geometry/point.hpp"

namespace dyx3_geometry {

/// Projection of a position onto one directed segment.
struct SegmentProjection {
  double t = 0.0;                  ///< clamped parameter in [0, 1]
  Point foot;                      ///< clamped foot (ON the segment)
  double signed_cross = 0.0;       ///< + = position is to the RIGHT of the directed segment (NED)
  double dist_to_end_along = 0.0;  ///< (1 - t) * |ab|
  bool valid = true;               ///< false only for an empty path
};

/// `signed_cross = cross_z / |ab|` — exact PERPENDICULAR distance even when t is clamped (the
/// along-track gap to a vertex never masquerades as cross-track; field fix 2026-07-30).
/// Degenerate segment (|ab|^2 < 1e-12): t = 0, foot = a, signed_cross = 0 and — a quirk carried
/// from the ancestor on purpose — dist_to_end_along = |pos - a|.
/// (ancestor: `_project_onto_segment`)
SegmentProjection project_onto_segment(Point pos, Point a, Point b);

/// Segment `seg_idx` of `path`, index clamped to [0, n-2]. n == 1: t = 0, foot = path[0],
/// signed_cross = 0, dist_to_end_along = distance to it. n == 0: `valid = false`.
SegmentProjection project_onto_segment(Point pos, PathView path, int seg_idx);

}  // namespace dyx3_geometry
