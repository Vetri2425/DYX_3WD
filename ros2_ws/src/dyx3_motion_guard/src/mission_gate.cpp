#include "dyx3_motion_guard/mission_gate.hpp"

namespace dyx3_motion_guard {

namespace {
constexpr uint8_t kArmed = 2;
constexpr uint8_t kOffboard = 14;
constexpr uint8_t kMissionRunning = 3;
}  // namespace

Reason first_failing_safety_gate(const GateInputs& in, const GateConfig& cfg) {
  if (in.estop) return Reason::Estop;
  if (!in.link.fresh || !in.link.session_alive || !in.link.handshake_ok ||
      in.link.stale_topics_mask != 0U) {
    return Reason::Px4LinkUnhealthy;
  }
  if (!in.op.fresh || !in.op.alive) return Reason::OperatorLinkLost;
  if (!in.vehicle.fresh || in.vehicle.arming_state != kArmed || in.vehicle.nav_state != kOffboard ||
      in.vehicle.failsafe) {
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

bool mission_running(const MissionIn& m) { return m.fresh && m.state == kMissionRunning; }

}  // namespace dyx3_motion_guard
