// guidance — lookahead distance and point, steering geometry, pivot intercept.
// Contract: docs/contracts/rpp_guidance.md. Pure C++, no ROS. Ports of the prototype's
// _get_lookahead_point, _segment_lookahead_point, _corner_clip, _pivot_intercept_heading and the
// lookahead-distance blocks of _control_loop_impl, proven against the verbatim Python
// (test/gate4_equivalence_test.cpp).
#pragma once

#include "dyx3_geometry/point.hpp"

namespace dyx3_rpp {

using dyx3_geometry::PathView;
using dyx3_geometry::Point;

struct LookaheadPoint {
  Point p;
  bool hit_end{false};
};

// Smooth profile: walk arc length l_d from the foot on segment seg_idx; off the end returns the
// final waypoint with hit_end = true. Empty path returns {0,0}.
LookaheadPoint smooth_lookahead_point(PathView path, int seg_idx, Point foot, double l_d);

// Segment profile: crosses a vertex only while the junction turn <= max_junction_deg; the path end
// is not a corner (extend_past_end); a real corner pins to the vertex or extends along the INCOMING
// tangent (extend_past_corner).
Point segment_lookahead_point(PathView path, int seg_idx, Point foot, double l_d,
                              double max_junction_deg, bool extend_past_end = true,
                              bool extend_past_corner = false);

// Junction turn at vertex idx+1 in degrees; NaN when idx < 0 or idx + 2 >= n.
double segment_angle_deg(PathView path, int idx);

struct LookaheadParams {
  double min_v;       // min_linear_vel
  double max_v;       // min(max_linear_vel, mission_speed)
  double l_min;       // min_lookahead_dist
  double l_max;       // max_lookahead_dist
  double ld_gain;     // lookahead_time
  double xt_ld_gain;  // xtrack_lookahead_gain
};

struct LookaheadDistance {
  double raw;  // published (/rpp/debug[8]) so saturation is visible
  double l_d;  // clamped to [l_min, l_max]
};

// L_d_raw = lookahead_time * v_for_ld + xtrack_lookahead_gain * |xtrack|, v_for_ld low-passed
// 70/30. last_speed_cmd <= 0 bootstraps with 0.5 * max_v.
LookaheadDistance lookahead_distance(const LookaheadParams& p, double last_speed_cmd,
                                     double signed_xtrack);

struct ArcCapParams {
  double cut_target_m;  // smooth_max_arc_cut_m
  double min_arc_ld_m;  // smooth_min_arc_ld_m
  double ld_coeff;      // smooth_curvature_ld_coeff (legacy floor, used only when the cap is off)
};

// Smooth-profile curvature cap (P5.1): with cut_target > 0 and kappa_path > 1e-6, L_cap = sqrt(8
// e/kappa) and L_d = max(min(L_d, L_cap), min_arc_ld). The cap REPLACES the legacy floor
// ld_coeff/kappa, which only applies when the cap is off.
double apply_arc_cap(double l_d, double kappa_path, const ArcCapParams& p);

struct Steering {
  double x_body{0};       // forward
  double y_body{0};       // right (FRD)
  double l_actual{0};     // hypot
  double theta_e{0};      // atan2(y_body, x_body)
  double kappa{0};        // 2 y_body / l_actual^2 (valid when l_actual >= 1e-6)
  bool degenerate{true};  // l_actual < 1e-6: the aim point landed on the rover
};

// NED offset (dn, de) to the aim point, yaw clockwise from North.
Steering steering_geometry(double dn, double de, double yaw_ned);

// Pivot target heading: bearing from the CURRENT position to a point pivot_intercept_dist beyond
// the rover's projection on the leg a->b (clamped to b), so the release-heading gate nulls the
// lateral offset the corner stop left. Falls back to the leg heading when disabled, degenerate or
// too close.
double pivot_intercept_heading(Point pos, Point a, Point b, double leg_heading, bool enabled,
                               double d_int);

}  // namespace dyx3_rpp
