// rpp_command — maps one RppCore tick to a MotionCommand. Contract:
// docs/contracts/rpp_motion_output.md section 2. Pure C++, no ROS.
//
// The core states its decision in the controller's own terms (CmdKind); the prototype's velocity
// vector is carried only so a tick can be compared with it. This is the production mapping
// (spec 5.2):
//   STOP   -> MODE_STOP
//   TRACK  -> smooth run: MODE_TRACK_RATE  (speed = |v|, yaw rate = the feed-forward + feedback
//   rate)
//             segment run: MODE_TRACK_HEADING (speed = |v|, heading = the bearing to the aim point)
//   BRAKE  -> signed speed along the nose with the nose heading held (never a 180 degree spot turn)
//   PIVOT  -> MODE_PIVOT, rate toward the exit heading
//   CREEP  -> MODE_CREEP, signed speed, no turn
// DERIVED — NOT FROM V1 SPEC: the segment profile maps to TRACK_HEADING (the contract allows
// TRACK_RATE with segment_yaw_rate_gain * theta_e as well). Open question for the human / GATE 4.
#pragma once

#include "dyx3_rpp/motion_output.hpp"
#include "dyx3_rpp/rpp_core.hpp"

namespace dyx3_rpp {

// max_yaw_rate is max_yaw_rate_body (> 0). A non-finite result becomes STOP (fail to zero).
MotionCommand command_from_tick(const TickOutput& o, bool profile_segment, double max_yaw_rate);

}  // namespace dyx3_rpp
