#include "dyx3_motion_guard/mission_gate.hpp"

namespace dyx3_motion_guard {

namespace {
constexpr uint8_t kArmed = 2;
constexpr uint8_t kOffboard = 14;
constexpr uint8_t kMissionRunning = 3;

// Shared by the full and the pre-arm gate: one definition of every gate, one priority order.
// Owner decision 2026-10-10: the tablet heartbeat (operator link) is not a gate at all, as in the
// prototype: the rover starts, runs and stops on its own; E-stop and the RC kill are the stops.
// pre_arm=false: the full gate (armed + OFFBOARD required). pre_arm=true: the pre-arm variant.
Reason first_failing(const GateInputs& in, const GateConfig& cfg, bool pre_arm) {
  if (in.estop) return Reason::Estop;
  if (!in.link.fresh || !in.link.session_alive || !in.link.handshake_ok ||
      in.link.stale_topics_mask != 0U) {
    return Reason::Px4LinkUnhealthy;
  }
  const auto& v = in.vehicle;
  // PX4 sets vehicle_status.pre_flight_checks_pass = canArm(current nav_state)
  // (Commander.cpp:1874). In OFFBOARD without an offboard signal (a run whose MANUAL release did
  // not take) that is false, although the arm dyx3_px4_link performs is valid: the link leaves
  // OFFBOARD for MANUAL first, then arms (fe1770b), and PX4 re-evaluates the flag in MANUAL. So the
  // pre-arm check does not require it while nav_state == OFFBOARD; PX4 still denies an arm it would
  // refuse in MANUAL.
  const bool preflight_refused = pre_arm && v.nav_state != kOffboard && !v.preflight_checks_pass;
  // RC link: pre-arm only (never a gate while running: RC loss in OFFBOARD is not a stop).
  const bool rc_refused = pre_arm && cfg.require_rc_link && !(v.rc_link_valid && v.rc_link_ok);
  const bool not_engaged = !pre_arm && (v.arming_state != kArmed || v.nav_state != kOffboard);
  if (!v.fresh || v.failsafe || preflight_refused || rc_refused || not_engaged) {
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
  return first_failing(in, cfg, /*pre_arm=*/false);
}

Reason first_failing_pre_arm_gate(const GateInputs& in, const GateConfig& cfg) {
  const Reason r = first_failing(in, cfg, /*pre_arm=*/true);
  if (r != Reason::Ok) return r;
  return in.vehicle.global_reference_valid ? Reason::Ok : Reason::GlobalReferenceInvalid;
}

bool mission_running(const MissionIn& m) { return m.fresh && m.state == kMissionRunning; }

}  // namespace dyx3_motion_guard
