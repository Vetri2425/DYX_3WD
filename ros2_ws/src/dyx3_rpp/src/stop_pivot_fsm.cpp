#include "dyx3_rpp/stop_pivot_fsm.hpp"

#include <algorithm>
#include <cmath>

namespace dyx3_rpp {

const char* to_string(FsmState s) {
  switch (s) {
    case FsmState::Tracking:
      return "TRACKING";
    case FsmState::Brake:
      return "BRAKE";
    case FsmState::Pivot:
      return "PIVOT";
    case FsmState::ReleaseSettle:
      return "RELEASE_SETTLE";
    case FsmState::Advance:
      return "ADVANCE";
    case FsmState::Done:
      return "DONE";
  }
  return "?";
}

double brake_speed(const StopTelemetry& tel, const StopPivotParams& p) {
  if (!tel.vel_fresh) return 0.0;
  if (p.brake_velocity_cap <= 0.0) return 0.0;
  if (tel.speed < p.stop_speed_threshold) return 0.0;
  // Do not invent a longitudinal reverse command for sideways motion.
  if (std::fabs(tel.v_forward) < p.stop_speed_threshold ||
      std::fabs(tel.v_forward) < 0.5 * tel.speed)
    return 0.0;
  const double mag = std::min(p.brake_velocity_cap, std::fabs(tel.v_forward));
  const double sign = tel.v_forward > 0.0 ? -1.0 : 1.0;
  return sign * mag;
}

bool StopConfirm::satisfied(int64_t now_ns, const StopTelemetry& tel, const StopPivotParams& p) {
  if (!entered_) {
    entered_ = true;
    entered_ns_ = now_ns;
  }
  const double held = static_cast<double>(now_ns - entered_ns_) * 1e-9;
  if (!tel.vel_fresh) {
    settle_ = false;
    // Only a STALE velocity may time out into a pivot (I3).
    return held >= p.stale_vel_hold_s;
  }
  const bool speed_ok = tel.speed < p.stop_speed_threshold;
  const bool yaw_rate_ok = std::fabs(tel.yaw_rate) < p.stop_yaw_rate_threshold;
  if (speed_ok && yaw_rate_ok) {
    if (p.stop_dwell_s <= 0.0) return true;
    if (!settle_) {
      settle_ = true;
      settle_ns_ = now_ns;
    } else if (static_cast<double>(now_ns - settle_ns_) * 1e-9 >= p.stop_dwell_s) {
      return true;
    }
  } else {
    settle_ = false;  // any violation resets the dwell
  }
  return false;
}

double PivotWatchdog::budget_s(const StopPivotParams& p) const {
  const double angle = std::max(0.0, angle_);
  double budget =
      p.spinup_margin_s + (p.nominal_pivot_rate > 1e-6 ? angle / p.nominal_pivot_rate : 0.0);
  budget = std::max(budget, p.turn_timeout_s);  // never below the legacy floor
  if (p.pivot_timeout_max_s > 0.0) budget = std::min(budget, p.pivot_timeout_max_s);
  return budget;
}

bool PivotWatchdog::timed_out(int64_t now_ns, double turn_angle_rad, const StopPivotParams& p) {
  if (!started_) {
    started_ = true;
    started_ns_ = now_ns;
    if (std::isfinite(turn_angle_rad)) angle_ = std::fabs(turn_angle_rad);
    return false;
  }
  const double timeout = budget_s(p);
  if (timeout <= 0.0) return false;
  if (static_cast<double>(now_ns - started_ns_) * 1e-9 < timeout) return false;
  warned_ = true;
  return true;
}

void CornerFsm::go(FsmState to, const char* reason, int64_t now_ns) {
  if (to == state_) return;
  log_.push_back({++seq_, state_, to, reason, now_ns});
  if (log_.size() > 256) log_.erase(log_.begin());
  state_ = to;
}

void CornerFsm::reset() {
  stop_->reset();
  pivot_.reset();
  stop_complete_ = false;
  settle_active_ = false;
  state_ = FsmState::Tracking;
}

CornerOutput CornerFsm::step(const CornerInput& in) {
  CornerOutput out;
  out.heading_err = in.heading_err_rad;

  // Tangent junction: keep momentum, never stop for it.
  if (in.corner_deg < p_.corner_threshold_deg) {
    out.action = CornerAction::Advance;
    out.collinear = true;
    go(FsmState::Advance, "collinear junction", in.now_ns);
    out.state = FsmState::Advance;
    reset();
    return out;
  }

  const double angle_rad =
      std::isfinite(in.turn_angle_rad) ? in.turn_angle_rad : in.corner_deg * (M_PI / 180.0);
  const bool timed_out = stop_complete_ && pivot_.timed_out(in.now_ns, angle_rad, p_);
  out.pivot_timed_out = timed_out;

  double release_tol = p_.heading_tolerance_rad;
  if (timed_out) release_tol = std::max(release_tol, p_.timeout_heading_tol_rad);
  release_tol =
      std::min(release_tol, p_.release_max_rad);  // hard cap: never launch grossly mis-headed
  const bool heading_ok = std::fabs(in.heading_err_rad) <= release_tol;
  // With stale velocity the two measured gates are replaced by "timed out": bounded, never a
  // deadlock.
  const bool yaw_rate_ok =
      in.tel.vel_fresh ? std::fabs(in.tel.yaw_rate) < p_.stop_yaw_rate_threshold : timed_out;
  const bool speed_ok = in.tel.vel_fresh ? in.tel.speed < p_.align_speed_threshold : timed_out;

  bool settled = false;
  if (stop_complete_ && heading_ok && yaw_rate_ok && speed_ok) {
    if (!settle_active_) {
      settle_active_ = true;
      settle_since_ns_ = in.now_ns;
    }
    settled = static_cast<double>(in.now_ns - settle_since_ns_) * 1e-9 >= p_.align_settle_s;
  } else {
    settle_active_ = false;
  }

  if (settled) {
    out.action = CornerAction::Advance;
    out.zero_speed_memory = true;
    go(FsmState::Advance, "release gates settled", in.now_ns);
    out.state = FsmState::Advance;
    reset();
    return out;
  }

  // Heading inside the release band: stop driving the pivot and brake, otherwise the pivot command
  // itself keeps speed above the release threshold.
  if (stop_complete_ && heading_ok) {
    go(FsmState::ReleaseSettle, "heading inside release band", in.now_ns);
    out.action = CornerAction::SettleBrake;
    out.state = state_;
    out.brake_speed = brake_speed(in.tel, p_);
    out.zero_speed_memory = true;
    return out;
  }

  if (!stop_complete_) {
    if (!stop_->satisfied(in.now_ns, in.tel, p_)) {
      go(FsmState::Brake, "stop not confirmed", in.now_ns);
      out.action = CornerAction::Brake;
      out.state = state_;
      out.brake_speed = brake_speed(in.tel, p_);
      out.zero_speed_memory = true;
      return out;
    }
    stop_complete_ = true;
    log_.push_back({++seq_, state_, state_, "stop confirmed", in.now_ns});
  }

  go(FsmState::Pivot, "stopped, heading outside release band", in.now_ns);
  out.action = CornerAction::Pivot;
  out.state = state_;
  out.set_speed_memory = true;
  out.speed_memory = p_.corner_speed;
  return out;
}

HoldOutput StopHold::step(int64_t now_ns, const StopTelemetry& tel) {
  HoldOutput out;
  latched_ = true;
  if (stop_->satisfied(now_ns, tel, p_)) {
    out.phase = HoldPhase::Stopped;
    out.stopped = true;
    return out;
  }
  out.phase = HoldPhase::Braking;
  out.brake_speed = brake_speed(tel, p_);
  return out;
}

}  // namespace dyx3_rpp
