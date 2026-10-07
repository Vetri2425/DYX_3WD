#include "dyx3_spray/spray_fsm.hpp"

#include <algorithm>
#include <cmath>

namespace dyx3_spray {

const char* to_string(SprayState s) {
  switch (s) {
    case SprayState::OffUnconfirmed:
      return "OFF_UNCONFIRMED";
    case SprayState::OffConfirmed:
      return "OFF_CONFIRMED";
    case SprayState::OnPending:
      return "ON_PENDING";
    case SprayState::OnConfirmed:
      return "ON_CONFIRMED";
    case SprayState::OffPending:
      return "OFF_PENDING";
    case SprayState::Recovery:
      return "RECOVERY";
    case SprayState::Disabled:
      return "DISABLED";
  }
  return "?";
}

std::optional<SprayCommand> SpraySafetyStateMachine::tick(bool desired, bool safety_ok,
                                                          bool enabled, double now) {
  // enabled edges first: DISABLED refuses ON unconditionally, before any ON dispatch is considered
  const bool edge_to_disabled = !enabled && prev_enabled_;
  const bool edge_to_enabled = enabled && !prev_enabled_;
  prev_enabled_ = enabled;
  if (edge_to_disabled) {
    disabled_ = true;
    recovery_attempt_ = 0;
  }
  if (edge_to_enabled) {  // re-arm through OFF_UNCONFIRMED: a fresh OFF must be confirmed before
                          // any ON
    disabled_ = false;
    state_ = SprayState::OffUnconfirmed;
    recovery_attempt_ = 0;
    have_deadline_ = false;
    have_pending_ = false;
  }

  const bool edge_safety_lost = enabled && !safety_ok && prev_safety_ok_;
  prev_safety_ok_ = safety_ok;
  if (edge_safety_lost) {
    recovery_attempt_ = 0;
    if (state_ == SprayState::Recovery) {  // fast first retry right when it matters most
      have_deadline_ = true;
      recovery_deadline_ = now;
    }
  }

  // the ack-timeout check runs in every branch: a wedged pending state must recover whatever the
  // inputs
  if (auto timed_out = check_pending_timeout(now)) return timed_out;

  if (!enabled) return drive_off_disabled(now);
  if (!safety_ok) return drive_off_safety(now);
  return normal_tick(desired, now);
}

std::optional<SprayCommand> SpraySafetyStateMachine::on_ack(uint32_t seq, bool success,
                                                            double now) {
  if (seq != seq_) return std::nullopt;
  if (state_ == SprayState::OnPending) {
    if (success) {
      have_pending_ = false;
      state_ = SprayState::OnConfirmed;
      return std::nullopt;
    }
    // a failed ON is never latched as ON: straight to a fresh OFF (RECOVERY is OFF-ack-failure
    // territory only)
    return dispatch_off(false, now);
  }
  if (state_ == SprayState::OffPending) {
    if (success) {
      have_pending_ = false;
      recovery_attempt_ = 0;
      have_deadline_ = false;
      state_ = disabled_ ? SprayState::Disabled : SprayState::OffConfirmed;
      return std::nullopt;
    }
    enter_recovery(now);
    return std::nullopt;
  }
  return std::nullopt;  // late duplicate for a state that already moved on
}

void SpraySafetyStateMachine::note_event_reset(double now) {
  recovery_attempt_ = 0;
  if (state_ == SprayState::Recovery) {
    have_deadline_ = true;
    recovery_deadline_ = now;
  }
}

std::optional<SprayCommand> SpraySafetyStateMachine::normal_tick(bool desired, double now) {
  switch (state_) {
    case SprayState::OffUnconfirmed:
      return dispatch_off(
          false, now);  // ON is never accepted until OFF_CONFIRMED, whatever `desired` says
    case SprayState::Recovery:
      if (have_deadline_ && now >= recovery_deadline_) return dispatch_off(false, now);
      return std::nullopt;
    case SprayState::OffConfirmed:
      if (desired) return dispatch_on(now);
      return std::nullopt;
    case SprayState::OnPending:
    case SprayState::OnConfirmed:
      if (!desired) return dispatch_off(false, now);
      return std::nullopt;
    case SprayState::OffPending:
    case SprayState::Disabled:
      return std::nullopt;
  }
  return std::nullopt;
}

std::optional<SprayCommand> SpraySafetyStateMachine::drive_off_disabled(double now) {
  if (state_ == SprayState::OffConfirmed) {
    state_ = SprayState::Disabled;  // already confirmed off: nothing left to confirm
    return std::nullopt;
  }
  if (state_ == SprayState::Disabled || state_ == SprayState::OffPending) return std::nullopt;
  return dispatch_off(true, now);
}

std::optional<SprayCommand> SpraySafetyStateMachine::drive_off_safety(double now) {
  // Quiet once OFF is confirmed: a sustained unsafe condition (the rover sitting disarmed) must not
  // flood the command path.
  if (state_ == SprayState::OffConfirmed || state_ == SprayState::Disabled) return std::nullopt;
  if (state_ == SprayState::OffPending) return std::nullopt;
  if (state_ == SprayState::Recovery) {
    if (have_deadline_ && now >= recovery_deadline_) return dispatch_off(true, now);
    return std::nullopt;
  }
  return dispatch_off(true, now);  // ON_PENDING, ON_CONFIRMED or OFF_UNCONFIRMED: force OFF now
}

SprayCommand SpraySafetyStateMachine::dispatch_off(bool force, double now) {
  ++seq_;
  state_ = SprayState::OffPending;
  have_pending_ = true;
  pending_since_ = now;
  return {false, seq_, force};
}

SprayCommand SpraySafetyStateMachine::dispatch_on(double now) {
  ++seq_;
  state_ = SprayState::OnPending;
  have_pending_ = true;
  pending_since_ = now;
  return {true, seq_, false};
}

void SpraySafetyStateMachine::enter_recovery(double now) {
  const double backoff =
      std::min(p_.backoff_base_s * std::pow(2.0, recovery_attempt_), p_.backoff_max_s);
  recovery_deadline_ = now + backoff;
  have_deadline_ = true;
  ++recovery_attempt_;
  state_ = SprayState::Recovery;
  have_pending_ = false;
}

std::optional<SprayCommand> SpraySafetyStateMachine::check_pending_timeout(double now) {
  if (!have_pending_) return std::nullopt;
  if ((now - pending_since_) < p_.ack_timeout_s) return std::nullopt;
  if (state_ == SprayState::OnPending)
    return dispatch_off(false, now);  // never latch an unconfirmed ON
  if (state_ == SprayState::OffPending) {
    enter_recovery(now);
    return std::nullopt;
  }
  have_pending_ = false;
  return std::nullopt;
}

}  // namespace dyx3_spray
