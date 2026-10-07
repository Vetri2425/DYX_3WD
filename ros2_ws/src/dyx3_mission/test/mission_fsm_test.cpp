// Mission FSM tests: the contract's transition table, enumerated, plus invariants under random
// event sequences. The expectations are the table in docs/contracts/dyx3_mission.md (a definition
// written before the code), not read back from the implementation.
#include "dyx3_mission/mission_fsm.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <functional>
#include <random>
#include <string>
#include <vector>

using namespace dyx3_mission;  // NOLINT

namespace {

constexpr std::int64_t kNow = 123;

MissionFsm in_state(State s) {
  MissionFsm f;
  switch (s) {
    case State::kIdle:
      break;
    case State::kLoading:
      f.start(true, kNow);
      break;
    case State::kReady:
      f.start(true, kNow);
      f.artifact_loaded(true, kNow);
      break;
    case State::kRunning:
      f.start(true, kNow);
      f.artifact_loaded(true, kNow);
      f.rpp_ack(true, 0, kNow);
      break;
    case State::kPaused:
      f.start(true, kNow);
      f.artifact_loaded(true, kNow);
      f.rpp_ack(true, 0, kNow);
      f.pause(kNow);
      break;
    case State::kCompleted:
      f.start(true, kNow);
      f.artifact_loaded(true, kNow);
      f.rpp_ack(true, 0, kNow);
      f.rpp_complete(kNow);
      break;
    case State::kAborted:
      f.start(true, kNow);
      f.abort(1, kNow);
      break;
    case State::kError:
      f.start(true, kNow);
      f.artifact_loaded(false, kNow);
      break;
  }
  EXPECT_EQ(f.state(), s);
  return f;
}

struct Case {
  State from;
  const char* what;
  std::function<Result(MissionFsm&)> act;
  bool accepted;
  Reject reject;
  State to;
  std::uint8_t reason;  // expected fsm.reason() afterwards (only checked when `check_reason`)
  bool check_reason;
};

std::vector<Case> table() {
  using S = State;
  std::vector<Case> c;
  // ---- start
  c.push_back({S::kIdle, "start gate ok", [](MissionFsm& f) { return f.start(true, kNow); }, true,
               Reject::kNone, S::kLoading, 0, true});
  c.push_back({S::kIdle, "start gate NOT ok", [](MissionFsm& f) { return f.start(false, kNow); },
               false, Reject::kSafetyGate, S::kIdle, 0, false});
  for (State s : {S::kLoading, S::kReady, S::kRunning, S::kPaused}) {
    c.push_back({s, "start while active", [](MissionFsm& f) { return f.start(true, kNow); }, false,
                 Reject::kBusy, s, 0, false});
  }
  for (State s : {S::kCompleted, S::kAborted, S::kError}) {
    c.push_back({s, "restart gate ok", [](MissionFsm& f) { return f.start(true, kNow); }, true,
                 Reject::kNone, S::kLoading, 0, true});
    c.push_back({s, "restart gate NOT ok", [](MissionFsm& f) { return f.start(false, kNow); },
                 false, Reject::kSafetyGate, s, 0, false});
  }
  // ---- artifact_loaded
  c.push_back({S::kLoading, "artifact valid",
               [](MissionFsm& f) { return f.artifact_loaded(true, kNow); }, true, Reject::kNone,
               S::kReady, 0, true});
  c.push_back({S::kLoading, "artifact invalid",
               [](MissionFsm& f) { return f.artifact_loaded(false, kNow); }, true, Reject::kNone,
               S::kError, kReasonPathError, true});
  for (State s :
       {S::kIdle, S::kReady, S::kRunning, S::kPaused, S::kCompleted, S::kAborted, S::kError}) {
    c.push_back({s, "artifact_loaded illegal",
                 [](MissionFsm& f) { return f.artifact_loaded(true, kNow); }, false,
                 Reject::kIllegal, s, 0, false});
  }
  // ---- rpp_ack
  c.push_back({S::kReady, "ack gate ok", [](MissionFsm& f) { return f.rpp_ack(true, 0, kNow); },
               true, Reject::kNone, S::kRunning, 0, true});
  c.push_back({S::kReady, "ack gate lost (safety)",
               [](MissionFsm& f) { return f.rpp_ack(false, 5, kNow); }, true, Reject::kNone,
               S::kAborted, kReasonSafety, true});
  c.push_back({S::kReady, "ack gate lost (rtk)",
               [](MissionFsm& f) { return f.rpp_ack(false, 6, kNow); }, true, Reject::kNone,
               S::kAborted, kReasonRtk, true});
  for (State s : {S::kRunning, S::kPaused}) {
    c.push_back({s, "duplicate ack ignored", [](MissionFsm& f) { return f.rpp_ack(true, 0, kNow); },
                 true, Reject::kNone, s, 0, false});
  }
  for (State s : {S::kIdle, S::kLoading, S::kCompleted, S::kAborted, S::kError}) {
    c.push_back({s, "ack illegal", [](MissionFsm& f) { return f.rpp_ack(true, 0, kNow); }, false,
                 Reject::kIllegal, s, 0, false});
  }
  // ---- pause
  c.push_back({S::kRunning, "pause", [](MissionFsm& f) { return f.pause(kNow); }, true,
               Reject::kNone, S::kPaused, kReasonOperator, true});
  for (State s :
       {S::kIdle, S::kLoading, S::kReady, S::kPaused, S::kCompleted, S::kAborted, S::kError}) {
    c.push_back({s, "pause not running", [](MissionFsm& f) { return f.pause(kNow); }, false,
                 Reject::kNotRunning, s, 0, false});
  }
  // ---- resume
  c.push_back({S::kPaused, "resume gate ok", [](MissionFsm& f) { return f.resume(true, kNow); },
               true, Reject::kNone, S::kRunning, 0, true});
  c.push_back({S::kPaused, "resume gate NOT ok",
               [](MissionFsm& f) { return f.resume(false, kNow); }, false, Reject::kSafetyGate,
               S::kPaused, 0, false});
  for (State s :
       {S::kIdle, S::kLoading, S::kReady, S::kRunning, S::kCompleted, S::kAborted, S::kError}) {
    c.push_back({s, "resume not paused", [](MissionFsm& f) { return f.resume(true, kNow); }, false,
                 Reject::kNotPaused, s, 0, false});
  }
  // ---- abort
  for (State s : {S::kLoading, S::kReady, S::kRunning, S::kPaused}) {
    c.push_back({s, "abort operator", [](MissionFsm& f) { return f.abort(1, kNow); }, true,
                 Reject::kNone, S::kAborted, kReasonOperator, true});
    c.push_back({s, "abort unspecified == operator", [](MissionFsm& f) { return f.abort(0, kNow); },
                 true, Reject::kNone, S::kAborted, kReasonOperator, true});
    c.push_back({s, "abort safety", [](MissionFsm& f) { return f.abort(2, kNow); }, true,
                 Reject::kNone, S::kAborted, kReasonSafety, true});
  }
  for (State s : {S::kIdle, S::kCompleted, S::kAborted, S::kError}) {
    c.push_back({s, "abort not active", [](MissionFsm& f) { return f.abort(1, kNow); }, false,
                 Reject::kNotActive, s, 0, false});
  }
  // ---- rpp_complete
  c.push_back({S::kRunning, "complete", [](MissionFsm& f) { return f.rpp_complete(kNow); }, true,
               Reject::kNone, S::kCompleted, 0, true});
  c.push_back({S::kPaused, "complete while paused ignored",
               [](MissionFsm& f) { return f.rpp_complete(kNow); }, true, Reject::kNone, S::kPaused,
               0, false});
  for (State s : {S::kIdle, S::kLoading, S::kReady, S::kCompleted, S::kAborted, S::kError}) {
    c.push_back({s, "complete illegal", [](MissionFsm& f) { return f.rpp_complete(kNow); }, false,
                 Reject::kIllegal, s, 0, false});
  }
  // ---- rpp_error
  for (State s : {S::kLoading, S::kReady, S::kRunning, S::kPaused}) {
    c.push_back({s, "rpp error", [](MissionFsm& f) { return f.rpp_error(kNow); }, true,
                 Reject::kNone, S::kError, kReasonInternalError, true});
  }
  for (State s : {S::kIdle, S::kCompleted, S::kAborted, S::kError}) {
    c.push_back({s, "rpp error illegal", [](MissionFsm& f) { return f.rpp_error(kNow); }, false,
                 Reject::kIllegal, s, 0, false});
  }
  // ---- gate_lost
  for (State s : {S::kLoading, S::kReady}) {
    c.push_back({s, "gate lost before motion -> abort",
                 [](MissionFsm& f) { return f.gate_lost(5, kNow); }, true, Reject::kNone,
                 S::kAborted, kReasonSafety, true});
  }
  c.push_back({S::kRunning, "gate lost (rtk) -> pause",
               [](MissionFsm& f) { return f.gate_lost(6, kNow); }, true, Reject::kNone, S::kPaused,
               kReasonRtk, true});
  c.push_back({S::kRunning, "gate lost (other) -> pause",
               [](MissionFsm& f) { return f.gate_lost(9, kNow); }, true, Reject::kNone, S::kPaused,
               kReasonSafety, true});
  for (State s : {S::kIdle, S::kPaused, S::kCompleted, S::kAborted, S::kError}) {
    c.push_back({s, "gate lost no effect", [](MissionFsm& f) { return f.gate_lost(5, kNow); }, true,
                 Reject::kNone, s, 0, false});
  }
  // ---- estop
  for (State s : {S::kLoading, S::kReady, S::kRunning, S::kPaused}) {
    c.push_back({s, "estop aborts", [](MissionFsm& f) { return f.estop(kNow); }, true,
                 Reject::kNone, S::kAborted, kReasonSafety, true});
  }
  for (State s : {S::kIdle, S::kCompleted, S::kAborted, S::kError}) {
    c.push_back({s, "estop no effect", [](MissionFsm& f) { return f.estop(kNow); }, true,
                 Reject::kNone, s, 0, false});
  }
  // ---- skip_point
  for (State s : {S::kRunning, S::kPaused}) {
    c.push_back({s, "skip with active point",
                 [](MissionFsm& f) { return f.skip_point(true, kNow); }, true, Reject::kNone, s, 0,
                 false});
    c.push_back({s, "skip without active point",
                 [](MissionFsm& f) { return f.skip_point(false, kNow); }, false,
                 Reject::kNoActivePoint, s, 0, false});
  }
  for (State s : {S::kIdle, S::kLoading, S::kReady, S::kCompleted, S::kAborted, S::kError}) {
    c.push_back({s, "skip not running", [](MissionFsm& f) { return f.skip_point(true, kNow); },
                 false, Reject::kNotRunning, s, 0, false});
  }
  return c;
}

}  // namespace

TEST(MissionFsmTable, EveryStateEventPairMatchesTheContract) {
  const auto cases = table();
  EXPECT_GT(cases.size(), 100U);
  for (const auto& c : cases) {
    SCOPED_TRACE(std::string(to_string(c.from)) + " / " + c.what);
    MissionFsm f = in_state(c.from);
    const std::uint32_t id_before = f.mission_id();
    const std::size_t log_before = f.log().size();
    const Result r = c.act(f);
    EXPECT_EQ(r.accepted, c.accepted);
    EXPECT_EQ(r.reject, c.reject);
    EXPECT_EQ(f.state(), c.to);
    EXPECT_EQ(r.state, c.to);
    if (c.check_reason) EXPECT_EQ(f.reason(), c.reason);
    // Every call is logged (accepted, refused or ignored): nothing happens silently.
    EXPECT_GT(f.log().size() + (f.log().size() == MissionFsm::kLogCapacity ? 1U : 0U), log_before);
    // The mission id moves exactly when a start is accepted.
    const bool started = std::string(c.what).find("start") != std::string::npos ||
                         std::string(c.what).find("restart") != std::string::npos;
    if (started && c.accepted && std::string(c.what).find("while") == std::string::npos) {
      EXPECT_EQ(f.mission_id(), id_before + 1);
    } else {
      EXPECT_EQ(f.mission_id(), id_before);
    }
    // A refusal never changes state.
    if (!c.accepted) EXPECT_EQ(f.state(), c.from);
  }
}

TEST(MissionFsmGuards, NeverAutoResumesAndNeverRunsWithoutAGate) {
  MissionFsm f = in_state(State::kRunning);
  EXPECT_EQ(f.gate_lost(6, kNow).state, State::kPaused);
  // The gate recovering changes nothing: only an explicit resume does.
  EXPECT_EQ(f.gate_lost(0, kNow).state, State::kPaused);
  EXPECT_EQ(f.rpp_ack(true, 0, kNow).state, State::kPaused);
  EXPECT_FALSE(f.resume(false, kNow).accepted);
  EXPECT_EQ(f.state(), State::kPaused);
  EXPECT_TRUE(f.resume(true, kNow).accepted);
  EXPECT_EQ(f.state(), State::kRunning);
}

TEST(MissionFsmGuards, TerminalStatesAreLeftOnlyByANewStart) {
  for (State s : {State::kCompleted, State::kAborted, State::kError}) {
    MissionFsm f = in_state(s);
    f.pause(kNow);
    f.resume(true, kNow);
    f.abort(1, kNow);
    f.rpp_complete(kNow);
    f.rpp_error(kNow);
    f.gate_lost(5, kNow);
    f.estop(kNow);
    f.skip_point(true, kNow);
    f.artifact_loaded(true, kNow);
    f.rpp_ack(true, 0, kNow);
    EXPECT_EQ(f.state(), s);
    EXPECT_TRUE(f.start(true, kNow).accepted);
    EXPECT_EQ(f.state(), State::kLoading);
  }
}

TEST(MissionFsmGuards, GuardReasonMapping) {
  EXPECT_EQ(map_guard_reason(6), kReasonRtk);  // MotionSetpointStatus::REASON_RTK_GATE
  for (std::uint8_t r : {0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 200}) {
    EXPECT_EQ(map_guard_reason(r), kReasonSafety);
  }
}

TEST(MissionFsmLog, TransitionsCarryReasonSequenceAndAreBounded) {
  MissionFsm f;
  std::vector<Transition> seen;
  f.set_observer([&](const Transition& t) { seen.push_back(t); });
  f.start(true, 10);
  f.artifact_loaded(true, 20);
  f.rpp_ack(true, 0, 30);
  f.gate_lost(6, 40);
  ASSERT_EQ(seen.size(), 4U);
  EXPECT_EQ(seen[0].seq, 1U);
  EXPECT_EQ(seen[3].seq, 4U);
  EXPECT_EQ(seen[3].from, State::kRunning);
  EXPECT_EQ(seen[3].to, State::kPaused);
  EXPECT_EQ(seen[3].reason, kReasonRtk);
  EXPECT_EQ(seen[3].stamp_ns, 40);
  EXPECT_FALSE(seen[3].detail.empty());
  for (int i = 0; i < 1000; ++i) f.pause(i);  // refusals are logged too, but the log stays bounded
  EXPECT_LE(f.log().size(), MissionFsm::kLogCapacity);
  EXPECT_GT(f.transitions_total(), 1000U);
}

TEST(MissionFsmProperty, InvariantsHoldUnderRandomEventSequences) {
  std::mt19937 rng(20261007);
  for (int run = 0; run < 200; ++run) {
    MissionFsm f;
    std::vector<Transition> log;
    f.set_observer([&](const Transition& t) { log.push_back(t); });
    std::uint32_t last_id = 0;
    for (int i = 0; i < 400; ++i) {
      const bool gate = rng() % 4 != 0;
      const std::uint8_t reason = static_cast<std::uint8_t>(rng() % 14);
      const State before = f.state();
      switch (rng() % 11) {
        case 0:
          f.start(gate, i);
          break;
        case 1:
          f.artifact_loaded(rng() % 2, i);
          break;
        case 2:
          f.rpp_ack(gate, reason, i);
          break;
        case 3:
          f.pause(i);
          break;
        case 4:
          f.resume(gate, i);
          break;
        case 5:
          f.abort(reason % 3, i);
          break;
        case 6:
          f.rpp_complete(i);
          break;
        case 7:
          f.rpp_error(i);
          break;
        case 8:
          f.gate_lost(reason, i);
          break;
        case 9:
          f.estop(i);
          break;
        default:
          f.skip_point(rng() % 2, i);
          break;
      }
      const State after = f.state();
      ASSERT_LE(static_cast<int>(after), 7);
      if (after == State::kRunning) EXPECT_EQ(f.reason(), kReasonNone);
      if (after == State::kAborted)
        EXPECT_TRUE(f.reason() == kReasonOperator || f.reason() == kReasonSafety ||
                    f.reason() == kReasonRtk);
      if (after == State::kError)
        EXPECT_TRUE(f.reason() == kReasonPathError || f.reason() == kReasonInternalError);
      if (after == State::kPaused)
        EXPECT_TRUE(f.reason() == kReasonOperator || f.reason() == kReasonSafety ||
                    f.reason() == kReasonRtk);
      // terminal states are left only through an accepted start into LOADING
      if ((before == State::kCompleted || before == State::kAborted || before == State::kError) &&
          after != before) {
        EXPECT_EQ(after, State::kLoading);
      }
      EXPECT_GE(f.mission_id(), last_id);
      EXPECT_LE(f.mission_id(), last_id + 1);
      last_id = f.mission_id();
    }
    for (const auto& t : log) {
      EXPECT_FALSE(t.detail.empty());
      if (t.refused) EXPECT_EQ(t.from, t.to);
      // RUNNING is only ever entered from READY (rpp_ack) or PAUSED (resume): never by a
      // gate/estop/etc.
      if (!t.refused && t.to == State::kRunning && t.from != State::kRunning) {
        EXPECT_TRUE((t.from == State::kReady && t.event == Event::kRppAck) ||
                    (t.from == State::kPaused && t.event == Event::kResume));
      }
      // The only way out of PAUSED into RUNNING is an explicit resume.
      if (!t.refused && t.from == State::kPaused && t.to == State::kRunning)
        EXPECT_EQ(t.event, Event::kResume);
    }
  }
}
