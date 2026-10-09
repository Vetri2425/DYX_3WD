// project_onto_path — see docs/contracts/dyx3_geometry.md
#include "dyx3_geometry/project_onto_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "dyx3_geometry/distance.hpp"

namespace dyx3_geometry {

PathProjection project_onto_path(Point pos, PathView path, ProjectionHint& hint) {
  PathProjection out;
  if (path.n == 0) {
    out.valid = false;
    return out;
  }
  if (path.n == 1) {
    out.seg_idx = 0;
    out.t = 0.0;
    out.foot = path[0];
    out.signed_cross = distance(pos, path[0]);
    return out;
  }
  const int n = static_cast<int>(path.n);
  int lo = 0;
  int hi = n - 1;
  if (hint.valid) {
    lo = std::max(0, hint.seg - 2);
    hi = std::min(n - 1, hint.seg + 4);
    if (hi - lo < 3) {
      lo = 0;
      hi = n - 1;
    }
  }

  int best_i = lo;
  double best_t = 0.0;
  Point best_foot = path[static_cast<std::size_t>(lo)];
  double best_d = std::numeric_limits<double>::infinity();
  double best_signed = 0.0;
  bool scanned = false;  // at least one non-degenerate segment was evaluated

  for (int i = lo; i < hi; ++i) {
    const Point a = path[static_cast<std::size_t>(i)];
    const Point b = path[static_cast<std::size_t>(i) + 1];
    const double dx = b.n - a.n;
    const double dy = b.e - a.e;
    const double seg_sq = dx * dx + dy * dy;
    if (seg_sq < 1e-12) {
      continue;
    }
    scanned = true;
    const double t_raw = ((pos.n - a.n) * dx + (pos.e - a.e) * dy) / seg_sq;
    const double t = std::max(0.0, std::min(1.0, t_raw));
    const double foot_n = a.n + t * dx;
    const double foot_e = a.e + t * dy;
    const double d = distance(pos, Point{foot_n, foot_e});
    if (d < best_d) {
      const double cross_z = dx * (pos.e - foot_e) - dy * (pos.n - foot_n);
      const double seg_len = std::sqrt(seg_sq);
      best_i = i;
      best_t = t;
      best_foot = Point{foot_n, foot_e};
      best_d = d;
      best_signed = seg_len > 0.0 ? std::copysign(d, cross_z) : 0.0;
    }
  }

  if (!scanned) {
    // Every scanned segment was zero-length: there is no foot point, so the result is not a
    // projection. Report it as invalid (the other fields stay the neutral defaults).
    hint.valid = false;
    out.valid = false;
  } else {
    hint.seg = best_i;
    hint.valid = true;
  }
  out.seg_idx = best_i;
  out.t = best_t;
  out.foot = best_foot;
  out.signed_cross = best_signed;
  return out;
}

}  // namespace dyx3_geometry
