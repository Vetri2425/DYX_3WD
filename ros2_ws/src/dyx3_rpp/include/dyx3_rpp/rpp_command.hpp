// rpp_command — maps one RppCore tick to a MotionCommand. Contract:
// docs/contracts/rpp_motion_output.md section 2. Pure C++, no ROS.
//
// The core states its decision in the controller's own terms (CmdKind); the prototype's velocity
// vector is carried only so a tick can be compared with it. This is the production mapping
// (spec 5.2):
//   STOP   -> MODE_STOP
//   TRACK  -> smooth run: MODE_TRACK_RATE  (speed = |v|, yaw rate = the feed-forward + feedback
//   rate)
//             segment run: MODE_TRACK_HEADING by default, or MODE_TRACK_RATE when the explicit
//             GATE 4 selector requests rate control
//   BRAKE  -> signed speed along the nose with the nose heading held (never a 180 degree spot turn)
//   PIVOT  -> MODE_PIVOT, rate toward the exit heading
//   CREEP  -> MODE_CREEP, signed speed along the nose, no turn, while the core's velocity vector
//             lies within kCreepSteerMinRad of the nose (or behind it); otherwise
//             MODE_TRACK_HEADING toward the vector: forward with heading atan2(v_e, v_n) when it is
//             within +/-90 deg of the nose, else reverse (-|v|) with heading atan2(-v_e, -v_n).
//             XR-RPP-001: the endpoint precise stop aims DIAGONALLY to remove a 2-15 cm lateral
//             miss; the prototype's firmware steered along that vector, so a nose-only CREEP threw
//             the lateral correction away and the rover rocked through the end plane.
// DERIVED — NOT FROM V1 SPEC: segment_command_mode is an IDLE_ONLY A/B selector. "heading" is the
// behavior-compatible default; "rate" uses the rate already computed by RppCore from
// segment_yaw_rate_gain * theta_e and clamped by max_yaw_rate_body.
#pragma once

#include <cmath>

#include "dyx3_rpp/motion_output.hpp"
#include "dyx3_rpp/rpp_core.hpp"

namespace dyx3_rpp {

// CREEP vectors closer than this to the nose (or to its reverse) keep the plain nose-axis CREEP.
// DERIVED — NOT FROM V1 SPEC: 2 deg = segment_heading_tolerance_deg default (XR-RPP-001).
constexpr double kCreepSteerMinRad = 2.0 * M_PI / 180.0;

// max_yaw_rate is max_yaw_rate_body (> 0). A non-finite result becomes STOP (fail to zero).
MotionCommand command_from_tick(const TickOutput& o, bool profile_segment, double max_yaw_rate,
                                bool segment_rate_command = false);

}  // namespace dyx3_rpp
