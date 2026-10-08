#include "dyx3_rpp/rpp_command.hpp"

#include <cmath>

namespace dyx3_rpp {

MotionCommand command_from_tick(const TickOutput& o, bool profile_segment, double max_yaw_rate,
                                bool segment_rate_command) {
  MotionCommand c;
  switch (o.cmd) {
    case CmdKind::Stop:
      c = make_stop();
      break;
    case CmdKind::Track: {
      const double speed = std::hypot(o.v_n, o.v_e);
      if (profile_segment) {
        c = segment_rate_command ? make_track_rate(speed, o.yaw_rate, max_yaw_rate)
                                 : make_track_heading(speed, o.track_heading_ned);
      } else {
        c = make_track_rate(speed, o.yaw_rate, max_yaw_rate);
      }
      break;
    }
    case CmdKind::Brake:
      c = make_brake(o.brake_speed, o.yaw_ned);
      break;
    case CmdKind::Pivot:
      c = make_pivot(o.pivot_heading_err, max_yaw_rate);
      break;
    case CmdKind::Creep:
      c = make_creep(o.creep_speed, 0.0, max_yaw_rate);
      break;
  }
  sanitize(&c);
  return c;
}

}  // namespace dyx3_rpp
