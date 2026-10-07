#include "dyx3_rpp/guidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "dyx3_geometry/heading_delta.hpp"
#include "dyx3_geometry/segment_heading.hpp"

namespace dyx3_rpp {
namespace {

double dist(double ax, double ay, double bx, double by) { return std::hypot(ax - bx, ay - by); }

// Real corner: never cross into the next leg. Pins to the vertex, or (tangent_extend) holds the aim
// `remaining` ahead along the INCOMING tangent b - pn (pn lies on the incoming line).
Point corner_clip(double pn, double pe, Point b, double remaining, double seg_rem,
                  bool tangent_extend) {
  if (!tangent_extend || seg_rem <= 1e-9) return b;
  const double f = (remaining - seg_rem) / seg_rem;
  return {b.n + (b.n - pn) * f, b.e + (b.e - pe) * f};
}

}  // namespace

LookaheadPoint smooth_lookahead_point(PathView path, int seg_idx, Point foot, double l_d) {
  LookaheadPoint out;
  const int n_pts = static_cast<int>(path.n);
  if (n_pts == 0) return out;
  seg_idx = std::clamp(seg_idx, 0, n_pts - 1);
  const Point end = (seg_idx + 1 < n_pts) ? path[static_cast<size_t>(seg_idx) + 1]
                                          : path[static_cast<size_t>(seg_idx)];
  double prev_n = foot.n, prev_e = foot.e;
  double next_n = end.n, next_e = end.e;
  double arc = 0.0;
  int i = seg_idx + 1;
  while (true) {
    const double seg_len = dist(prev_n, prev_e, next_n, next_e);
    if (arc + seg_len >= l_d) {
      const double remaining = l_d - arc;
      const double ratio = seg_len > 1e-9 ? remaining / seg_len : 1.0;
      out.p = {prev_n + ratio * (next_n - prev_n), prev_e + ratio * (next_e - prev_e)};
      out.hit_end = false;
      return out;
    }
    arc += seg_len;
    ++i;
    if (i >= n_pts) {
      out.p = path[static_cast<size_t>(n_pts) - 1];
      out.hit_end = true;
      return out;
    }
    prev_n = next_n;
    prev_e = next_e;
    next_n = path[static_cast<size_t>(i)].n;
    next_e = path[static_cast<size_t>(i)].e;
  }
}

double segment_angle_deg(PathView path, int idx) {
  const int n = static_cast<int>(path.n);
  if (idx < 0 || idx + 2 >= n) return std::numeric_limits<double>::quiet_NaN();
  const Point a = path[static_cast<size_t>(idx)], b = path[static_cast<size_t>(idx) + 1],
              c = path[static_cast<size_t>(idx) + 2];
  const double h0 = dyx3_geometry::segment_heading(a, b);
  const double h1 = dyx3_geometry::segment_heading(b, c);
  return dyx3_geometry::heading_delta(h0, h1) * (180.0 / M_PI);
}

Point segment_lookahead_point(PathView path, int seg_idx, Point foot, double l_d,
                              double max_junction_deg, bool extend_past_end,
                              bool extend_past_corner) {
  const int n_pts = static_cast<int>(path.n);
  if (n_pts == 0) return {};
  if (n_pts < 2) return path[0];

  double remaining = std::max(0.0, l_d);
  int cur = std::max(0, std::min(seg_idx, n_pts - 2));
  double pn = foot.n, pe = foot.e;

  while (cur + 1 < n_pts) {
    const Point b = path[static_cast<size_t>(cur) + 1];
    const double seg_rem = dist(pn, pe, b.n, b.e);
    if (seg_rem >= remaining) {
      if (seg_rem <= 1e-9) return b;
      const double f = remaining / seg_rem;
      return {pn + (b.n - pn) * f, pe + (b.e - pe) * f};
    }
    // The lookahead outruns this segment. The final vertex is not a corner: extend along the final
    // bearing.
    if (extend_past_end && cur + 2 >= n_pts) {
      double ux = b.n - pn, uy = b.e - pe;
      double norm = std::hypot(ux, uy);
      if (norm <= 1e-9) {
        const Point prev = path[static_cast<size_t>(std::max(0, n_pts - 2))];
        ux = b.n - prev.n;
        uy = b.e - prev.e;
        norm = std::hypot(ux, uy);
      }
      if (norm > 1e-9) {
        const double f = (remaining - seg_rem) / norm;
        return {b.n + ux * f, b.e + uy * f};
      }
      return b;
    }
    if (max_junction_deg <= 0.0) return b;
    const double angle = segment_angle_deg(path, cur);
    // NaN (last vertex) and any turn above the threshold terminate the walk at a corner.
    if (!(std::fabs(angle) <= max_junction_deg)) {
      return corner_clip(pn, pe, b, remaining, seg_rem, extend_past_corner && std::isfinite(angle));
    }
    remaining -= seg_rem;
    pn = b.n;
    pe = b.e;
    ++cur;
  }
  return path[static_cast<size_t>(n_pts) - 1];
}

LookaheadDistance lookahead_distance(const LookaheadParams& p, double last_speed_cmd,
                                     double signed_xtrack) {
  double v_for_ld = std::max(p.min_v, last_speed_cmd > 0.0 ? last_speed_cmd : p.max_v * 0.5);
  v_for_ld = 0.7 * v_for_ld + 0.3 * p.max_v;  // low-pass: prevents a one-step limit cycle
  LookaheadDistance out;
  out.raw = p.ld_gain * v_for_ld + p.xt_ld_gain * std::fabs(signed_xtrack);
  out.l_d = std::max(p.l_min, std::min(p.l_max, out.raw));
  return out;
}

double apply_arc_cap(double l_d, double kappa_path, const ArcCapParams& p) {
  if (kappa_path > 1e-6 && p.cut_target_m > 0.0) {
    const double l_cap = std::sqrt(8.0 * p.cut_target_m / kappa_path);
    return std::max(std::min(l_d, l_cap), p.min_arc_ld_m);
  }
  if (kappa_path > 1e-6 && p.ld_coeff > 0.0) return std::max(l_d, p.ld_coeff / kappa_path);
  return l_d;
}

Steering steering_geometry(double dn, double de, double yaw_ned) {
  Steering s;
  s.x_body = dn * std::cos(yaw_ned) + de * std::sin(yaw_ned);
  s.y_body = -dn * std::sin(yaw_ned) + de * std::cos(yaw_ned);
  s.l_actual = std::hypot(s.x_body, s.y_body);
  s.theta_e = std::atan2(s.y_body, s.x_body);
  s.degenerate = s.l_actual < 1e-6;
  s.kappa = s.degenerate ? 0.0 : (2.0 * s.y_body) / (s.l_actual * s.l_actual);
  return s;
}

double pivot_intercept_heading(Point pos, Point a, Point b, double leg_heading, bool enabled,
                               double d_int) {
  if (!enabled) return leg_heading;
  const double dx = b.n - a.n, dy = b.e - a.e;
  const double seg_sq = dx * dx + dy * dy;
  if (d_int <= 0.0 || seg_sq < 1e-12) return leg_heading;
  const double seg_len = std::sqrt(seg_sq);
  const double t_foot =
      std::max(0.0, std::min(1.0, ((pos.n - a.n) * dx + (pos.e - a.e) * dy) / seg_sq));
  const double t_int = std::min(t_foot + d_int / seg_len, 1.0);
  const double int_n = a.n + t_int * dx, int_e = a.e + t_int * dy;
  const double d_to_int = dist(pos.n, pos.e, int_n, int_e);
  if (d_to_int < std::max(0.05, 0.25 * d_int)) return leg_heading;
  return std::atan2(int_e - pos.e, int_n - pos.n);
}

}  // namespace dyx3_rpp
