// spray_fsm — the actuator command state machine. Contract: docs/contracts/dyx3_spray.md section 3.
// Pure C++, no ROS, no wall clock: all timing is caller-injected (monotonic seconds). A port of
// spray_fsm.SpraySafetyStateMachine, proven against the verbatim Python by scripted event
// sequences.
//
// INVARIANT 1 — `spraying` is true only in ON_CONFIRMED (no optimistic-ON window).
// INVARIANT 2 — every dispatched command carries the machine's monotonic cmd_seq; an ack applies
// only if its seq
//               matches, a stale/superseded reply is a no-op.
#pragma once

#include <cstdint>
#include <optional>

namespace dyx3_spray {

enum class SprayState : uint8_t {
  OffUnconfirmed = 0,
  OffConfirmed,
  OnPending,
  OnConfirmed,
  OffPending,
  Recovery,
  Disabled
};
const char* to_string(SprayState s);

struct SprayCommand {
  bool on;
  uint32_t seq;
  bool force;  // bypass retry throttle/backoff (safety loss / disable / shutdown)
};

struct FsmParams {
  double backoff_base_s{0.5};
  double backoff_max_s{5.0};
  // A dispatched command whose ack never arrives must not wedge the FSM: treated as the
  // ack(timeout) failure.
  double ack_timeout_s{1.0};
};

class SpraySafetyStateMachine {
public:
  explicit SpraySafetyStateMachine(const FsmParams& p = FsmParams{}) : p_(p) {}

  SprayState state() const { return state_; }
  bool spraying() const { return state_ == SprayState::OnConfirmed; }
  bool commanded() const {
    return state_ == SprayState::OnPending || state_ == SprayState::OnConfirmed;
  }
  uint32_t cmd_seq() const { return seq_; }

  // Advance one control tick; returns a command to dispatch, or nothing.
  std::optional<SprayCommand> tick(bool desired, bool safety_ok, bool enabled, double now);
  // Apply a command ack. Ignored unless seq == cmd_seq (invariant 2).
  std::optional<SprayCommand> on_ack(uint32_t seq, bool success, double now);
  // Config/mode-change hook: resets the backoff attempt and makes the next RECOVERY retry
  // immediate.
  void note_event_reset(double now);

private:
  std::optional<SprayCommand> normal_tick(bool desired, double now);
  std::optional<SprayCommand> drive_off_disabled(double now);
  std::optional<SprayCommand> drive_off_safety(double now);
  SprayCommand dispatch_off(bool force, double now);
  SprayCommand dispatch_on(double now);
  void enter_recovery(double now);
  std::optional<SprayCommand> check_pending_timeout(double now);

  FsmParams p_;
  SprayState state_{SprayState::OffUnconfirmed};
  uint32_t seq_{0};
  bool have_pending_{false};
  double pending_since_{0.0};
  bool disabled_{false};
  int recovery_attempt_{0};
  bool have_deadline_{false};
  double recovery_deadline_{0.0};
  bool prev_enabled_{true};
  bool prev_safety_ok_{true};
};

}  // namespace dyx3_spray
