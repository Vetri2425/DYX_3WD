// fail_to_zero — the guard's decision: sequence session rule, gates, limits, canonical STOP.
// See docs/contracts/dyx3_motion_guard.md sections 2, 3 and 5. Pure C++.
#pragma once

#include <cstdint>

#include "dyx3_motion_guard/actuator_plausibility.hpp"
#include "dyx3_motion_guard/limits.hpp"
#include "dyx3_motion_guard/mission_gate.hpp"
#include "dyx3_motion_guard/motion_types.hpp"

namespace dyx3_motion_guard {

// Publisher-session sequence rule. Strictly increasing; a decrease or the very first command starts
// a new session and `accept_count` strictly increasing commands must follow before motion is
// accepted. An equal seq never refreshes freshness.
class SequenceTracker {
public:
  explicit SequenceTracker(uint32_t accept_count) : accept_(accept_count) {}
  // Returns true if the command is new (stored by the caller) and false for a duplicate.
  bool on_command(uint64_t seq, double now_s);
  bool accepting() const { return have_ && count_ >= accept_; }
  double last_new_s() const { return last_new_s_; }
  bool have() const { return have_; }
  uint32_t count() const { return count_; }

private:
  uint32_t accept_;
  bool have_{false};
  uint64_t last_{0};
  uint32_t count_{0};
  double last_new_s_{0.0};
};

struct DecisionConfig {
  double command_max_age_s{0.2};
  GateConfig gates;
  Limits limits;
  PlausibilityConfig plausibility;
};

struct Decision {
  Motion out;  // the command to publish: forwarded (limited) or the canonical STOP
  Reason reason{Reason::Stale};
  bool accepted{false};
  bool clamped{false};
  uint64_t input_seq{0};
  double input_age_s{1.0e9};
  // IF-003: the forwarded command's source_pose_sample_stamp (ns) when accepted; 0 otherwise (the
  // node then stamps its own STOP with the newest pose it knows).
  int64_t source_pose_sample_ns{0};
};

class GuardCore {
public:
  GuardCore(uint32_t session_accept_count, const DecisionConfig& cfg)
      : seq_(session_accept_count), cfg_(cfg) {}

  void on_command(const Command& c, double now_s);
  // One decision tick. dt_s is the bounded time since the previous tick.
  Decision decide(double now_s, double dt_s, const GateInputs& gates);

  // The actuator-stall latch (REASON_ACTUATOR_STALL). The node publishes it as the safety-gate
  // verdict while it is set, so dyx3_mission pauses on it.
  bool actuator_stalled() const { return stall_.latched(); }

  DecisionConfig& config() { return cfg_; }
  const DecisionConfig& config() const { return cfg_; }

private:
  SequenceTracker seq_;
  DecisionConfig cfg_;
  bool have_cmd_{false};
  Command cmd_{};
  LimitState lim_{};
  StallDetector stall_{};
};

}  // namespace dyx3_motion_guard
