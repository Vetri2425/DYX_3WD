#include "dyx3_rpp/rpp_command.hpp"

#include <cmath>

#include "dyx3_geometry/angle_wrap.hpp"

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
        // XR-RPP-007: the heading of any non-zero vector is its own bearing. The core freezes its
        // heading memory below 1 cm/s (no North snap at a stop), so for the first ticks of a ramp
        // after a pivot track_heading_ned is still the PREVIOUS leg's heading: commanding it
        // turns the rover back off the exit leg it has just pivoted to. Only an exactly zero
        // vector keeps the frozen heading.
        const double heading = speed > 1e-9 ? std::atan2(o.v_e, o.v_n) : o.track_heading_ned;
        c = segment_rate_command ? make_track_rate(speed, o.yaw_rate, max_yaw_rate)
                                 : make_track_heading(speed, heading);
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
    case CmdKind::Creep: {
      // XR-RPP-001: carry the direction of the core's correction vector, not only its projection on
      // the nose. Same magnitude (the core's caps are unchanged); a vector off the nose by more
      // than kCreepSteerMinRad becomes a heading command toward it, driven forward when it is
      // ahead of the beam and in reverse (nose kept toward the opposite bearing) when behind.
      const double mag = std::hypot(o.v_n, o.v_e);
      if (mag > 1e-9 && std::isfinite(mag) && std::isfinite(o.yaw_ned)) {
        const double bearing = std::atan2(o.v_e, o.v_n);
        const double off = dyx3_geometry::angle_wrap(bearing - o.yaw_ned);
        const bool ahead = std::fabs(off) <= M_PI / 2.0;
        const double off_axis = ahead ? std::fabs(off) : M_PI - std::fabs(off);
        if (off_axis > kCreepSteerMinRad) {
          c = ahead ? make_track_heading(mag, bearing)
                    : make_track_heading(-mag, std::atan2(-o.v_e, -o.v_n));
          break;
        }
      }
      c = make_creep(o.creep_speed, 0.0, max_yaw_rate);
      break;
    }
  }
  sanitize(&c);
  return c;
}

}  // namespace dyx3_rpp
