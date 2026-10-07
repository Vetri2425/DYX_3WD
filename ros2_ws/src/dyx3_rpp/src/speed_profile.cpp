#include "dyx3_rpp/speed_profile.hpp"

#include <algorithm>
#include <cmath>

namespace dyx3_rpp {

double alignment_accel_scale(double heading_error_rad, double curvature_m_inv,
                             double full_accel_heading_deg, double no_accel_heading_deg,
                             double full_accel_curvature, double no_accel_curvature) {
  const double heading_error_deg = std::fabs(heading_error_rad) * (180.0 / M_PI);
  const double abs_curvature = std::fabs(curvature_m_inv);

  double heading_scale;
  if (no_accel_heading_deg <= full_accel_heading_deg) {
    heading_scale = heading_error_deg <= full_accel_heading_deg ? 1.0 : 0.0;
  } else if (heading_error_deg <= full_accel_heading_deg) {
    heading_scale = 1.0;
  } else if (heading_error_deg >= no_accel_heading_deg) {
    heading_scale = 0.0;
  } else {
    heading_scale = (no_accel_heading_deg - heading_error_deg) /
                    (no_accel_heading_deg - full_accel_heading_deg);
  }

  double curvature_scale;
  if (no_accel_curvature <= full_accel_curvature) {
    curvature_scale = abs_curvature <= full_accel_curvature ? 1.0 : 0.0;
  } else if (abs_curvature <= full_accel_curvature) {
    curvature_scale = 1.0;
  } else if (abs_curvature >= no_accel_curvature) {
    curvature_scale = 0.0;
  } else {
    curvature_scale =
        (no_accel_curvature - abs_curvature) / (no_accel_curvature - full_accel_curvature);
  }
  return std::max(0.0, std::min(1.0, std::min(heading_scale, curvature_scale)));
}

bool update_kappa_hard_latch(bool hard_latched, double kappa_now, double kappa_hard_enter,
                             double kappa_hard_exit) {
  if (kappa_hard_enter <= kappa_hard_exit) return std::fabs(kappa_now) >= kappa_hard_enter;
  if (std::fabs(kappa_now) >= kappa_hard_enter) return true;
  if (std::fabs(kappa_now) <= kappa_hard_exit) return false;
  return hard_latched;
}

SlewResult apply_smooth_speed_slew(double speed_raw, double last_speed, double dt,
                                   bool hard_latched, double speed_cmd_decel, double max_accel,
                                   double accel_scale, bool approach_active, double p4_floor) {
  if (speed_raw > last_speed) {
    if (max_accel > 0.0) {
      return {std::min(speed_raw,
                       last_speed + max_accel * std::max(0.0, accel_scale) * std::max(0.0, dt)),
              0};
    }
    return {speed_raw, 0};
  }
  if (approach_active || speed_raw < p4_floor) return {speed_raw, 3};
  if (hard_latched) return {speed_raw, 2};
  if (speed_cmd_decel > 0.0)
    return {std::max(speed_raw, last_speed - speed_cmd_decel * std::max(0.0, dt)), 1};
  return {speed_raw, 2};
}

double approach_distance(double configured, double max_v, double max_decel) {
  return std::max(configured, (max_v * max_v) / (2.0 * max_decel) + 0.10);
}

LateralLimit lateral_speed_limit(double kappa_speed, double a_lat_max, double min_curv_v,
                                 double max_v) {
  if (kappa_speed > 1e-9) {
    const double v_lat = std::sqrt(a_lat_max / kappa_speed);
    return {std::max(min_curv_v, std::min(max_v, std::min(max_v, v_lat))), v_lat};
  }
  return {max_v, max_v};
}

ApproachResult smooth_approach_scaling(double speed, bool run_closed, double run_length,
                                       double path_travel, double dist_to_goal, double approach_d,
                                       double approach_v) {
  const auto clamp01 = [](double v) { return std::max(0.0, std::min(1.0, v)); };
  if (run_closed) {
    const double remaining = std::max(0.0, run_length - path_travel);
    if (remaining < approach_d) {
      const double scale = clamp01(remaining / approach_d);
      return {std::min(speed, std::max(approach_v, speed * scale)), true};
    }
  } else if (dist_to_goal < approach_d && path_travel >= approach_d) {
    const double scale = clamp01(dist_to_goal / approach_d);
    const double approach_speed = std::max(approach_v, speed * scale);
    return {std::min(speed, approach_speed), true};
  }
  return {speed, false};
}

}  // namespace dyx3_rpp
