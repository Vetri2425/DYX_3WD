// speed_profile — the speed laws. Contract: docs/contracts/rpp_speed_profile.md. Pure C++.
// Ports of _alignment_accel_scale, _update_kappa_hard_latch, _apply_smooth_speed_slew and the speed
// blocks of the smooth profile in _control_loop_impl (steps 5-7), proven against the verbatim
// Python (test/gate4_equivalence_test.cpp).
#pragma once

namespace dyx3_rpp {

// Acceleration multiplier in [0, 1]. Both heading error and curvature must settle before full
// acceleration; only scales UPWARD acceleration, never impedes deceleration.
double alignment_accel_scale(double heading_error_rad, double curvature_m_inv,
                             double full_accel_heading_deg, double no_accel_heading_deg,
                             double full_accel_curvature, double no_accel_curvature);

// Hysteresis latch: enter at/above `enter`, leave at/below `exit`; enter <= exit means stateless
// |k| >= enter.
bool update_kappa_hard_latch(bool hard_latched, double kappa_now, double kappa_hard_enter,
                             double kappa_hard_exit);

struct SlewResult {
  double speed;
  int mode;  // 0 accel, 1 normal decel slew, 2 hard / immediate, 3 approach / P4
};

SlewResult apply_smooth_speed_slew(double speed_raw, double last_speed, double dt,
                                   bool hard_latched, double speed_cmd_decel, double max_accel,
                                   double accel_scale, bool approach_active, double p4_floor);

// Braking distance with a safety margin; the configured approach distance is a FLOOR.
double approach_distance(double configured, double max_v, double max_decel);

// Smooth-profile lateral-acceleration speed law: v <= sqrt(a_lat_max / kappa_speed), clamped to
// [min_curv_v, max_v]; kappa_speed ~ 0 gives max_v.
struct LateralLimit {
  double speed;
  double v_lat_limit;
};
LateralLimit lateral_speed_limit(double kappa_speed, double a_lat_max, double min_curv_v,
                                 double max_v);

struct ApproachResult {
  double speed;
  bool approach_active;
};

// Open run: scale on Euclidean distance to goal once path_travel >= approach_d. Closed run: scale
// on the REMAINING along-loop distance (run_length - path_travel), never on Euclidean distance (it
// is ~0 at the seam).
ApproachResult smooth_approach_scaling(double speed, bool run_closed, double run_length,
                                       double path_travel, double dist_to_goal, double approach_d,
                                       double approach_v);

// P4 floor: exact zero below the threshold, only when the INTENDED target is also below the floor
// (otherwise a ramp-up through the dead band deadlocks 0 -> delta -> 0).
inline double apply_p4_floor(double speed, double speed_before_accel, double last_speed_cmd,
                             double p4_floor, int* speed_mode) {
  if (speed < p4_floor && speed_before_accel < p4_floor && last_speed_cmd > 0.0) {
    if (speed_mode != nullptr) *speed_mode = 3;
    return 0.0;
  }
  return speed;
}

}  // namespace dyx3_rpp
