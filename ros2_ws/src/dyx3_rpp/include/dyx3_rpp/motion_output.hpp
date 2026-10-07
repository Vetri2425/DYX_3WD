// motion_output — the explicit-control output stage (REWRITTEN, not ported).
// Contract: docs/contracts/rpp_motion_output.md. Pure C++, no ROS.
//
// The prototype published a NED velocity VECTOR plus a yaw rate that the old firmware discarded;
// pivots were small vectors at the exit heading, brakes were +/- body-axis vectors, reverse was
// inferred by the firmware. Here the controller states its intent directly: a mode, a signed body
// speed and either a heading or a rate. Everything past this stage (limits, freshness, gates) is
// dyx3_motion_guard, which never invents a correction.
#pragma once

#include <cstdint>

namespace dyx3_rpp {

// Numeric values are the frozen MotionSetpoint ABI (checked by a static_assert in the node).
enum class MotionMode : uint8_t { Stop = 0, TrackHeading = 1, TrackRate = 2, Pivot = 3, Creep = 4 };

struct MotionCommand {
  MotionMode mode{MotionMode::Stop};
  float speed_body_x{0.0F};  // signed: negative is reverse
  float yaw_setpoint;        // NaN unless TrackHeading
  float yaw_rate_setpoint;   // NaN for TrackHeading, finite otherwise
  MotionCommand();
};

MotionCommand make_stop();

// Heading-hold tracking: bearing to the aim point (NED, wrapped to [-pi, pi]).
MotionCommand make_track_heading(double speed, double yaw_ned);

// Rate tracking (smooth arcs: kappa*v recovers the structural floor the old interface had).
MotionCommand make_track_rate(double speed, double yaw_rate, double max_yaw_rate);

// Terminal creep: small signed speed with a rate.
MotionCommand make_creep(double speed, double yaw_rate, double max_yaw_rate);

// In-place pivot toward the exit heading. DERIVED — NOT FROM V1 SPEC (rpp_motion_output.md section
// 4): the old firmware owned the spot-turn rate. omega = clamp(kPivotRateGain * err, +/-
// max_yaw_rate) with kPivotRateGain = 1.5 = RO_YAW_P (spec 3.1). Re-validate at GATE 1 / GATE 4.
constexpr double kPivotRateGain = 1.5;
MotionCommand make_pivot(double heading_err, double max_yaw_rate);

// Brake / precise-stop reverse: signed speed along the nose with the nose heading held. Never a
// 180-degree spot-turn (BUG-T3). Reverse permission is the guard's decision.
MotionCommand make_brake(double signed_speed, double nose_yaw_ned);

// Fail to zero: any non-finite field or a contract break becomes the canonical STOP. Returns true
// if the command was already clean.
bool sanitize(MotionCommand* c);

}  // namespace dyx3_rpp
