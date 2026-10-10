// mission_gate + the remaining safety gates (PX4 link, arming, RTK, heading, estimator).
// Pure functions on plain structs; every "not fresh" input fails. See the contract section 3.
#pragma once

#include <cstdint>

#include "dyx3_motion_guard/motion_types.hpp"
#include "dyx3_motion_guard/rtk_gate.hpp"

namespace dyx3_motion_guard {

struct MissionIn {
  bool fresh{false};
  uint8_t state{0};  // MissionState.STATE_RUNNING == 3
};
struct Px4LinkIn {
  bool fresh{false};
  bool session_alive{false};
  bool handshake_ok{false};
  uint32_t stale_topics_mask{0xFFFFFFFFU};
};
struct VehicleIn {
  bool fresh{false};
  uint8_t arming_state{0};  // 2 = ARMED
  uint8_t nav_state{0};     // 14 = OFFBOARD
  bool failsafe{false};
  bool position_valid{false};
  bool velocity_valid{false};
  bool attitude_valid{false};
  bool global_reference_valid{false};  // pre-arm gate only
  bool preflight_checks_pass{false};   // pre-arm gate only: PX4 would deny the arm if false
  // 0.17.0: RC (manual control) link as PX4 sees it. Pre-arm gate only, and only with
  // GateConfig::require_rc_link; never a gate while running (owner decision: RC loss in OFFBOARD
  // is not a stop).
  bool rc_link_valid{false};
  bool rc_link_ok{false};
  // Actuator-plausibility inputs (never a gate on their own): horizontal speed
  // hypot(velocity_north_mps, velocity_east_mps) and VehicleState.yaw_rate_radps. Judged only with
  // fresh, velocity_valid and attitude_valid.
  float measured_speed_mps{0.0F};
  float measured_yaw_rate_radps{0.0F};
};
struct EstimatorIn {
  bool fresh{false};
  bool flags_valid{false};
  bool gnss_yaw_fusion_intended{false};
  bool gnss_yaw_fault{true};
  bool reject_yaw{true};
  bool reject_hor_pos{true};
  bool reject_hor_vel{true};
  bool inertial_dead_reckoning{true};
};

struct GateInputs {
  bool estop{false};
  Px4LinkIn link;
  VehicleIn vehicle;
  RtkIn rtk;
  EstimatorIn est;
  MissionIn mission;
};

struct GateConfig {
  RtkConfig rtk;
  bool require_gnss_yaw_fusion{true};  // DERIVED: CLAUDE.md section 3
  // 0.17.0: pre-arm requires VehicleState.rc_link_valid && rc_link_ok, so the RC kill switch is a
  // guaranteed stop for every autonomous run. Default false: rc_link_valid depends on PX4
  // publishing failsafe_flags on DDS, which is unproven on the flashed firmware; turn it on once
  // rc_link_valid is observed true on the rover.
  bool require_rc_link{false};
};

// First failing safety gate in the documented priority order, or Reason::Ok. Excludes the mission
// gate. The operator link is not a gate (owner decision 2026-10-10).
Reason first_failing_safety_gate(const GateInputs& in, const GateConfig& cfg);

// First failing PRE-ARM gate, or Reason::Ok: the same gates and order as
// first_failing_safety_gate except that "armed" and "nav_state == OFFBOARD" are not required (the
// arming check keeps "vehicle state fresh, no PX4 failsafe" and adds PX4's own pre-flight checks
// verdict, so a Start is refused before an arm PX4 would deny, except while PX4 sits in OFFBOARD;
// with require_rc_link also the RC link), plus the EKF global reference (GlobalReferenceInvalid,
// checked last). dyx3_mission arms only while this is Ok.
Reason first_failing_pre_arm_gate(const GateInputs& in, const GateConfig& cfg);

// The mission gate on its own: MissionState fresh and RUNNING.
bool mission_running(const MissionIn& m);

}  // namespace dyx3_motion_guard
