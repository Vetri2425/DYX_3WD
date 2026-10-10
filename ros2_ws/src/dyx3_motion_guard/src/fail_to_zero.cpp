#include "dyx3_motion_guard/fail_to_zero.hpp"

namespace dyx3_motion_guard {

bool SequenceTracker::on_command(uint64_t seq, double now_s) {
  if (!have_) {
    have_ = true;
    last_ = seq;
    count_ = 1;
    last_new_s_ = now_s;
    return true;
  }
  if (seq < last_) {  // publisher restarted
    last_ = seq;
    count_ = 1;
    last_new_s_ = now_s;
    return true;
  }
  if (seq == last_) return false;
  last_ = seq;
  if (count_ < 0xFFFFFFFFU) ++count_;
  last_new_s_ = now_s;
  return true;
}

void GuardCore::on_command(const Command& c, double now_s) {
  if (seq_.on_command(c.seq, now_s)) {
    cmd_ = c;
    have_cmd_ = true;
  }
}

Decision GuardCore::decide(double now_s, double dt_s, const GateInputs& gates) {
  Decision d;
  d.out = canonical_stop();
  d.input_seq = have_cmd_ ? cmd_.seq : 0;
  d.input_age_s = (have_cmd_ && seq_.have()) ? now_s - seq_.last_new_s() : 1.0e9;

  const auto fail = [&](Reason r) {
    d.reason = r;
    d.accepted = false;
    d.clamped = false;
    d.out = canonical_stop();
    reset(lim_);  // fail-to-zero is immediate and leaves no ramp state behind
    return d;
  };

  if (!have_cmd_) return fail(Reason::Stale);
  if (!command_is_valid(cmd_)) return fail(Reason::InvalidMessage);
  if (d.input_age_s > cfg_.command_max_age_s) return fail(Reason::Stale);
  if (!seq_.accepting()) return fail(Reason::Sequence);

  const Mode mode = static_cast<Mode>(cmd_.mode);
  if (mode == Mode::Stop) {  // a clean STOP is always forwarded, whatever the gates say
    reset(lim_);
    d.reason = Reason::Ok;
    d.accepted = true;
    d.source_pose_sample_ns = cmd_.source_pose_sample_ns;
    return d;
  }

  const Reason g = first_failing_safety_gate(gates, cfg_.gates);
  if (g != Reason::Ok) return fail(g);
  if (!mission_running(gates.mission)) return fail(Reason::MissionGate);

  Motion in;
  in.mode = mode;
  in.speed_body_x = cmd_.speed_body_x;
  in.yaw_setpoint = cmd_.yaw_setpoint;
  in.yaw_rate_setpoint = cmd_.yaw_rate_setpoint;
  const Limited lim = apply_limits(in, dt_s, cfg_.limits, lim_);
  d.out = lim.motion;
  d.accepted = true;
  d.source_pose_sample_ns = cmd_.source_pose_sample_ns;
  d.clamped = lim.clamped;
  d.reason = lim.clamped ? Reason::LimitClamped : Reason::Ok;
  return d;
}

}  // namespace dyx3_motion_guard
