// actuator_plausibility — "commanded but not moving" detector (REASON_ACTUATOR_STALL, 0.17.0).
// See docs/contracts/dyx3_motion_guard.md section 3.1. Pure C++ on plain structs, no ROS, no heap.
//
// Measured 2026-10-10 (docs/analysis/2026-10-10_last_two_missions_controller_robustness.md section
// 2): a -0.45 rad/s pivot was commanded for 25.8 s with the drivetrain powered off; measured yaw
// rate 0, speed 0, every gate green. The guard never invents a correction, but it may refuse: a
// command the drivetrain demonstrably does not execute is stopped with its own reason.
#pragma once

#include "dyx3_motion_guard/mission_gate.hpp"
#include "dyx3_motion_guard/motion_types.hpp"

namespace dyx3_motion_guard {

// Every threshold is an existing dyx3_rpp parameter re-declared here with the same default; none is
// a new tuning number. The guard reads them once at start (RESTART), like every other parameter.
struct PlausibilityConfig {
  bool enabled{true};  // require_actuator_plausibility
  // DERIVED — NOT FROM V1 SPEC: = dyx3_rpp segment_nominal_pivot_rate_rad_s (0.4).
  float demand_yaw_rate_radps{0.4F};  // stall_yaw_rate_radps
  // DERIVED — NOT FROM V1 SPEC: = dyx3_rpp min_approach_linear_velocity (0.1).
  float demand_speed_mps{0.1F};  // stall_speed_mps
  // DERIVED — NOT FROM V1 SPEC: = dyx3_rpp segment_stop_yaw_rate_threshold (0.05).
  float still_yaw_rate_radps{0.05F};  // stall_measured_yaw_rate_radps
  // DERIVED — NOT FROM V1 SPEC: = dyx3_rpp segment_stop_speed_threshold (0.02).
  float still_speed_mps{0.02F};  // stall_measured_speed_mps
  // DERIVED — NOT FROM V1 SPEC: = dyx3_rpp segment_pivot_spinup_margin_s (1.0).
  double stall_time_s{1.0};  // stall_time_s
};

// Structural validity: finite, positive, and "still" strictly below the matching demand.
bool plausibility_config_valid(const PlausibilityConfig& c);

// The command demands motion the drivetrain must show: |yaw_rate_setpoint| >= demand_yaw_rate in a
// rate mode (TRACK_RATE, PIVOT, CREEP), or |speed_body_x| >= demand_speed in any moving mode. STOP
// never demands. In TRACK_HEADING only the speed counts (the rate is NaN by contract).
bool demands_motion(const Motion& m, const PlausibilityConfig& c);

// The measurement can be judged: vehicle state fresh, velocity and attitude valid, both values
// finite.
bool measurement_usable(const VehicleIn& v);

// The measured vehicle is still: horizontal speed < still_speed and |yaw rate| < still_yaw_rate.
bool measured_still(const VehicleIn& v, const PlausibilityConfig& c);

// Timer + latch. The timer counts only while the forwarded command demands motion and a usable
// measurement shows the vehicle still; any other tick resets it. After more than stall_time_s the
// detector latches, and stays latched (the guard holds STOP) until the demand drops below the
// demand thresholds (a STOP or a smaller command): a stalled pivot does not flap between STOP and
// PIVOT every stall_time_s.
class StallDetector {
public:
  // A tick on which the guard is about to forward `demand`. Returns true while stalled (latched).
  bool update(const Motion& demand, const VehicleIn& v, double now_s, const PlausibilityConfig& c);
  // A tick that did not reach the detector (an earlier check refused the command): the timer
  // restarts, the latch is kept (the demand did not drop).
  void interrupt() { timing_ = false; }
  // The demand dropped (a clean STOP was forwarded): timer and latch clear.
  void clear() {
    timing_ = false;
    latched_ = false;
  }
  bool latched() const { return latched_; }

private:
  bool timing_{false};
  double since_s_{0.0};
  bool latched_{false};
};

}  // namespace dyx3_motion_guard
