// project_onto_path — see docs/contracts/dyx3_geometry.md
#pragma once

#include "dyx3_geometry/point.hpp"

namespace dyx3_geometry {

/// Closest-segment search state (ancestor: `_closest_seg_hint`, `_hint_valid`). Owned by the
/// caller; reset with `invalid()` on a new path, after a position jump, or on an EKF reset.
struct ProjectionHint {
  int seg = 0;
  bool valid = false;
};

struct PathProjection {
  int seg_idx = 0;
  double t = 0.0;
  Point foot;
  /// + = position is to the RIGHT of the directed path (docs/contracts/frames.md).
  double signed_cross = 0.0;
  /// false for an empty path, or when every scanned segment is zero-length (no foot point exists;
  /// callers must not use the other fields then).
  bool valid = true;
};

/// Closest point on a polyline, as a segment projection.
///
/// Windowed: when `hint.valid`, only segments [hint-2, hint+4) are scanned (widened to a full
/// scan when the window is < 3 wide); a full O(n) scan otherwise. The winning segment is stored
/// back into `hint`. `signed_cross = copysign(distance, cross_z)`. A window containing only
/// zero-length segments (e.g. a path of identical points) returns `valid = false` and invalidates
/// the hint, so the next call full-scans.
///
/// n == 1 returns seg 0, t 0, foot = path[0], signed_cross = +distance (sign undefined).
/// KNOWN DEFECT carried from the ancestor (open C9): no monotonic-progress window, so on a
/// closed shape with coincident start/end a full scan can snap across the seam.
/// (ancestor: `_project_onto_path`)
PathProjection project_onto_path(Point pos, PathView path, ProjectionHint& hint);

}  // namespace dyx3_geometry
