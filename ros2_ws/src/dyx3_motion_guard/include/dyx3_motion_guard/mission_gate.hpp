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
};

// First failing safety gate in the documented priority order, or Reason::Ok. Excludes the mission
// gate. The operator link is not a gate (owner decision 2026-10-10).
Reason first_failing_safety_gate(const GateInputs& in, const GateConfig& cfg);

// First failing PRE-ARM gate, or Reason::Ok: the same gates and order as
// first_failing_safety_gate except that "armed" and "nav_state == OFFBOARD" are not required (the
// arming check keeps "vehicle state fresh, no PX4 failsafe"), plus the EKF global reference
// (GlobalReferenceInvalid, checked last). dyx3_mission arms only while this is Ok.
Reason first_failing_pre_arm_gate(const GateInputs& in, const GateConfig& cfg);

// The mission gate on its own: MissionState fresh and RUNNING.
bool mission_running(const MissionIn& m);

}  // namespace dyx3_motion_guard
