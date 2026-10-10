#include "dyx3_px4_link/rover_setpoint_writer.hpp"

#include <cmath>
#include <limits>

namespace dyx3_px4_link {
namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
}

Setpoint::Setpoint() : yaw_setpoint(kNaN), yaw_rate_setpoint(0.0F) {}

Setpoint stop_setpoint() {
  Setpoint s;
  s.mode = Mode::Stop;
  s.speed_body_x = 0.0F;
  s.yaw_setpoint = kNaN;
  s.yaw_rate_setpoint = 0.0F;
  return s;
}

Verdict validate(const Command& c) {
  if (!c.valid) return Verdict::NotValid;
  if (c.mode > static_cast<uint8_t>(Mode::Creep)) return Verdict::BadMode;
  const bool speed_f = std::isfinite(c.speed_body_x);
  const bool yaw_f = std::isfinite(c.yaw_setpoint);
  const bool rate_f = std::isfinite(c.yaw_rate_setpoint);
  const bool yaw_nan = std::isnan(c.yaw_setpoint);
  const bool rate_nan = std::isnan(c.yaw_rate_setpoint);
  switch (static_cast<Mode>(c.mode)) {
    case Mode::Stop:
      return Verdict::Ok;
    case Mode::TrackHeading:
      if (!speed_f || !yaw_f) return Verdict::NonFinite;
      return rate_nan ? Verdict::Ok : Verdict::ContractViolation;
    case Mode::TrackRate:
    case Mode::Creep:
      if (!speed_f || !rate_f) return Verdict::NonFinite;
      return yaw_nan ? Verdict::Ok : Verdict::ContractViolation;
    case Mode::Pivot:
      if (!speed_f || !rate_f) return Verdict::NonFinite;
      if (c.speed_body_x != 0.0F) return Verdict::ContractViolation;
      return yaw_nan ? Verdict::Ok : Verdict::ContractViolation;
  }
  return Verdict::BadMode;
}

Setpoint to_setpoint(const Command& c) {
  if (validate(c) != Verdict::Ok) return stop_setpoint();
  Setpoint s;
  s.mode = static_cast<Mode>(c.mode);
  switch (s.mode) {
    case Mode::Stop:
      return stop_setpoint();
    case Mode::TrackHeading:
      s.speed_body_x = c.speed_body_x;
      s.yaw_setpoint = c.yaw_setpoint;
      s.yaw_rate_setpoint = kNaN;
      break;
    case Mode::TrackRate:
    case Mode::Creep:
      s.speed_body_x = c.speed_body_x;
      s.yaw_setpoint = kNaN;
      s.yaw_rate_setpoint = c.yaw_rate_setpoint;
      break;
    case Mode::Pivot:
      s.speed_body_x = 0.0F;
      s.yaw_setpoint = kNaN;
      s.yaw_rate_setpoint = c.yaw_rate_setpoint;
      break;
  }
  return s;
}

void CommandGate::on_command(const Command& c, double now_s) {
  if (have_seq_ && c.seq < last_seq_) {
    // Publisher restarted. Do not resume anything from the old session, and discard this first
    // command of the new one too: it is one tick of STOP in exchange for never acting on a
    // command that raced the restart.
    last_seq_ = c.seq;
    have_cmd_ = false;
    reset_pending_ = true;
    return;
  }
  if (have_seq_ && c.seq == last_seq_) return;  // duplicate: must not refresh freshness
  have_seq_ = true;
  last_seq_ = c.seq;
  cmd_ = c;
  received_s_ = now_s;
  have_cmd_ = true;
  gap_counted_ = false;
}

GateOutput CommandGate::step(const GateInputs& in) {
  GateOutput out;
  out.sp = stop_setpoint();
  out.command_age_s = have_cmd_ ? in.now_s - received_s_ : 1.0e9;

  auto fail = [&](Reason r) {
    out.reason = r;
    out.failing_to_zero = true;
    out.sp = stop_setpoint();
    return out;
  };

  // Link-level conditions first: they make any setpoint untrustworthy regardless of the command.
  if (!in.handshake_ok) return fail(Reason::HandshakeNotOk);
  if (!in.session_alive) return fail(Reason::NoSession);
  if (in.stale_topics_mask != 0U) return fail(Reason::TopicStale);

  if (reset_pending_) {
    reset_pending_ = false;
    return fail(Reason::SequenceReset);
  }
  if (!have_cmd_) return fail(Reason::NoCommand);

  if (out.command_age_s > max_age_) {
    if (!gap_counted_) {  // one event per expired command, however many ticks it stays stale
      ++gap_events_;
      gap_counted_ = true;
    }
    return fail(Reason::CommandStale);
  }

  if (validate(cmd_) != Verdict::Ok) return fail(Reason::CommandInvalid);

  out.sp = to_setpoint(cmd_);
  out.forwarded = true;
  out.seq = cmd_.seq;
  out.source_pose_sample_us = cmd_.source_pose_sample_us;
  if (out.sp.mode == Mode::Stop) {
    out.reason = Reason::GuardStop;
    out.failing_to_zero = false;
  } else {
    out.reason = Reason::None;
    out.failing_to_zero = false;
  }
  return out;
}

}  // namespace dyx3_px4_link
