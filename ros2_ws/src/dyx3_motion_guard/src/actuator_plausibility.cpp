#include "dyx3_motion_guard/actuator_plausibility.hpp"

#include <cmath>

namespace dyx3_motion_guard {

bool plausibility_config_valid(const PlausibilityConfig& c) {
  const auto pos = [](double v) { return std::isfinite(v) && v > 0.0; };
  return pos(c.demand_yaw_rate_radps) && pos(c.demand_speed_mps) && pos(c.still_yaw_rate_radps) &&
         pos(c.still_speed_mps) && pos(c.stall_time_s) &&
         c.still_yaw_rate_radps < c.demand_yaw_rate_radps && c.still_speed_mps < c.demand_speed_mps;
}

bool demands_motion(const Motion& m, const PlausibilityConfig& c) {
  if (m.mode == Mode::Stop) return false;
  const bool rate = uses_rate(m.mode) && std::isfinite(m.yaw_rate_setpoint) &&
                    std::fabs(m.yaw_rate_setpoint) >= c.demand_yaw_rate_radps;
  const bool speed =
      std::isfinite(m.speed_body_x) && std::fabs(m.speed_body_x) >= c.demand_speed_mps;
  return rate || speed;
}

bool measurement_usable(const VehicleIn& v) {
  return v.fresh && v.velocity_valid && v.attitude_valid && std::isfinite(v.measured_speed_mps) &&
         std::isfinite(v.measured_yaw_rate_radps);
}

bool measured_still(const VehicleIn& v, const PlausibilityConfig& c) {
  return v.measured_speed_mps < c.still_speed_mps &&
         std::fabs(v.measured_yaw_rate_radps) < c.still_yaw_rate_radps;
}

bool StallDetector::update(const Motion& demand, const VehicleIn& v, double now_s,
                           const PlausibilityConfig& c) {
  if (!c.enabled || !demands_motion(demand, c)) {
    clear();
    return false;
  }
  if (latched_) return true;  // the demand still holds: STOP is held
  if (!measurement_usable(v) || !measured_still(v, c)) {
    timing_ = false;
    return false;
  }
  if (!timing_) {
    timing_ = true;
    since_s_ = now_s;
  }
  if (now_s - since_s_ > c.stall_time_s) {
    latched_ = true;
    timing_ = false;
  }
  return latched_;
}

}  // namespace dyx3_motion_guard
