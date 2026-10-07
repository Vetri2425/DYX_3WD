// project_onto_segment — see docs/contracts/dyx3_geometry.md
#include "dyx3_geometry/project_onto_segment.hpp"

#include <algorithm>
#include <cmath>

#include "dyx3_geometry/distance.hpp"

namespace dyx3_geometry {

SegmentProjection project_onto_segment(Point pos, Point a, Point b) {
  const double dx = b.n - a.n;
  const double dy = b.e - a.e;
  const double seg_sq = dx * dx + dy * dy;
  SegmentProjection out;
  if (seg_sq < 1e-12) {
    out.t = 0.0;
    out.foot = a;
    out.signed_cross = 0.0;
    out.dist_to_end_along = distance(pos, a);
    return out;
  }
  const double t_raw = ((pos.n - a.n) * dx + (pos.e - a.e) * dy) / seg_sq;
  const double t = std::max(0.0, std::min(1.0, t_raw));  // ancestor `_clamp`
  const double foot_n = a.n + t * dx;
  const double foot_e = a.e + t * dy;
  const double seg_len = std::sqrt(seg_sq);
  const double cross_z = dx * (pos.e - foot_e) - dy * (pos.n - foot_n);
  out.t = t;
  out.foot = Point{foot_n, foot_e};
  out.signed_cross = cross_z / seg_len;
  out.dist_to_end_along = (1.0 - t) * seg_len;
  return out;
}

SegmentProjection project_onto_segment(Point pos, PathView path, int seg_idx) {
  SegmentProjection out;
  if (path.n == 0) {
    out.valid = false;
    return out;
  }
  if (path.n == 1) {
    out.foot = path[0];
    out.dist_to_end_along = distance(pos, path[0]);
    return out;
  }
  const int last = static_cast<int>(path.n) - 2;
  const int i = std::max(0, std::min(seg_idx, last));
  return project_onto_segment(pos, path[static_cast<std::size_t>(i)],
                              path[static_cast<std::size_t>(i) + 1]);
}

}  // namespace dyx3_geometry
