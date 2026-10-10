#include "dyx3_motion_guard/mission_gate.hpp"

namespace dyx3_motion_guard {

namespace {
constexpr uint8_t kArmed = 2;
constexpr uint8_t kOffboard = 14;
constexpr uint8_t kMissionRunning = 3;

// Shared by the full and the pre-arm gate: one definition of every gate, one priority order.
// Owner decision 2026-10-10: the tablet heartbeat (operator link) is not a gate at all, as in the
// prototype: the rover starts, runs and stops on its own; E-stop and the RC kill are the stops.
Reason first_failing(const GateInputs& in, const GateConfig& cfg, bool require_armed_offboard) {
  if (in.estop) return Reason::Estop;
  if (!in.link.fresh || !in.link.session_alive || !in.link.handshake_ok ||
      in.link.stale_topics_mask != 0U) {
    return Reason::Px4LinkUnhealthy;
  }
  if (!in.vehicle.fresh || in.vehicle.failsafe ||
      (require_armed_offboard &&
       (in.vehicle.arming_state != kArmed || in.vehicle.nav_state != kOffboard))) {
    return Reason::ArmingGate;
  }
  if (!rtk_ok(in.rtk, cfg.rtk)) return Reason::RtkGate;
  const auto& e = in.est;
  if (!e.fresh || !e.flags_valid || e.gnss_yaw_fault || e.reject_yaw ||
      (cfg.require_gnss_yaw_fusion && !e.gnss_yaw_fusion_intended) || !in.vehicle.attitude_valid) {
    return Reason::HeadingUnhealthy;
  }
  if (!in.vehicle.position_valid || !in.vehicle.velocity_valid || e.inertial_dead_reckoning ||
      e.reject_hor_pos || e.reject_hor_vel) {
    return Reason::EstimatorUnhealthy;
  }
  return Reason::Ok;
}
}  // namespace

Reason first_failing_safety_gate(const GateInputs& in, const GateConfig& cfg) {
  return first_failing(in, cfg, /*require_armed_offboard=*/true);
}

Reason first_failing_pre_arm_gate(const GateInputs& in, const GateConfig& cfg) {
  const Reason r = first_failing(in, cfg, /*require_armed_offboard=*/false);
  if (r != Reason::Ok) return r;
  return in.vehicle.global_reference_valid ? Reason::Ok : Reason::GlobalReferenceInvalid;
}

bool mission_running(const MissionIn& m) { return m.fresh && m.state == kMissionRunning; }

}  // namespace dyx3_motion_guard
