#include "dyx3_rpp/motion_output.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dyx3_rpp {
namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

float wrap_pi(double a) {
  double w = std::fmod(a + M_PI, 2.0 * M_PI);
  if (w < 0.0) w += 2.0 * M_PI;
  return static_cast<float>(w - M_PI);
}
double clampd(double v, double lim) { return std::max(-lim, std::min(lim, v)); }
}  // namespace

MotionCommand::MotionCommand() : yaw_setpoint(kNaN), yaw_rate_setpoint(0.0F) {}

MotionCommand make_stop() { return MotionCommand{}; }

MotionCommand make_track_heading(double speed, double yaw_ned) {
  MotionCommand c;
  c.mode = MotionMode::TrackHeading;
  c.speed_body_x = static_cast<float>(speed);
  c.yaw_setpoint = wrap_pi(yaw_ned);
  c.yaw_rate_setpoint = kNaN;
  return c;
}

MotionCommand make_track_rate(double speed, double yaw_rate, double max_yaw_rate) {
  MotionCommand c;
  c.mode = MotionMode::TrackRate;
  c.speed_body_x = static_cast<float>(speed);
  c.yaw_setpoint = kNaN;
  c.yaw_rate_setpoint =
      static_cast<float>(max_yaw_rate > 0.0 ? clampd(yaw_rate, max_yaw_rate) : yaw_rate);
  return c;
}

MotionCommand make_creep(double speed, double yaw_rate, double max_yaw_rate) {
  MotionCommand c = make_track_rate(speed, yaw_rate, max_yaw_rate);
  c.mode = MotionMode::Creep;
  return c;
}

MotionCommand make_pivot(double heading_err, double max_yaw_rate) {
  MotionCommand c;
  c.mode = MotionMode::Pivot;
  c.speed_body_x = 0.0F;
  c.yaw_setpoint = kNaN;
  c.yaw_rate_setpoint = static_cast<float>(clampd(kPivotRateGain * heading_err, max_yaw_rate));
  return c;
}

MotionCommand make_brake(double signed_speed, double nose_yaw_ned) {
  if (signed_speed == 0.0) return make_track_heading(0.0, nose_yaw_ned);
  return make_track_heading(signed_speed, nose_yaw_ned);
}

bool sanitize(MotionCommand* c) {
  const bool sp = std::isfinite(c->speed_body_x);
  const bool yf = std::isfinite(c->yaw_setpoint), yn = std::isnan(c->yaw_setpoint);
  const bool rf = std::isfinite(c->yaw_rate_setpoint), rn = std::isnan(c->yaw_rate_setpoint);
  bool ok = false;
  switch (c->mode) {
    case MotionMode::Stop:
      ok = c->speed_body_x == 0.0F && yn && c->yaw_rate_setpoint == 0.0F;
      break;
    case MotionMode::TrackHeading:
      ok = sp && yf && rn;
      break;
    case MotionMode::TrackRate:
    case MotionMode::Creep:
      ok = sp && rf && yn;
      break;
    case MotionMode::Pivot:
      ok = sp && c->speed_body_x == 0.0F && rf && yn;
      break;
  }
  if (!ok) *c = make_stop();
  return ok;
}

}  // namespace dyx3_rpp
