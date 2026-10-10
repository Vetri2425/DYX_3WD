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
const std::string kD = "detail";

using S = State;
const std::vector<State> kAll = {S::kIdle,      S::kLoading, S::kPlacing, S::kArming,
                                 S::kEngaging,  S::kReady,   S::kRunning, S::kPaused,
                                 S::kCompleted, S::kAborted, S::kError};
const std::vector<State> kBeforeRunning = {S::kLoading, S::kPlacing, S::kArming, S::kEngaging,
                                           S::kReady};
const std::vector<State> kActive = {S::kLoading, S::kPlacing, S::kArming, S::kEngaging,
                                    S::kReady,   S::kRunning, S::kPaused};
const std::vector<State> kInactive = {S::kIdle, S::kCompleted, S::kAborted, S::kError};

std::vector<State> all_but(std::initializer_list<State> skip) {
  std::vector<State> out;
  for (State s : kAll) {
    bool keep = true;
    for (State k : skip) keep = keep && k != s;
    if (keep) out.push_back(s);
  }
  return out;
}

// Drives a fresh FSM along the happy path up to `s`.
MissionFsm in_state(State s) {
  MissionFsm f;
  const auto upto = [&f](State target) {
    f.start(true, kNow);
    if (target == S::kLoading) return;
    f.artifact_loaded(true, kD, kNow);
    if (target == S::kPlacing) return;
    f.placed(true, 0, kD, kNow);
    if (target == S::kArming) return;
    f.armed(true, 0, kD, kNow);
    if (target == S::kEngaging) return;
    f.engaged(true, 0, kD, kNow);
    if (target == S::kReady) return;
    f.rpp_ack(true, kNow);
  };
  switch (s) {
    case S::kIdle:
      break;
    case S::kLoading:
    case S::kPlacing:
    case S::kArming:
    case S::kEngaging:
    case S::kReady:
    case S::kRunning:
      upto(s);
      break;
    case S::kPaused:
      upto(S::kRunning);
      f.pause(kNow);
      break;
    case S::kCompleted:
      upto(S::kRunning);
      f.rpp_complete(kNow);
      break;
    case S::kAborted:
      upto(S::kLoading);
      f.abort(1, kNow);
      break;
    case S::kError:
      upto(S::kLoading);
      f.artifact_loaded(false, kD, kNow);
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

// One pre-RUNNING step event: legal only in `in`, success -> `next`, failure -> ERROR(reason).
void step_cases(std::vector<Case>& c, State in, State next, const char* name,
                const std::function<Result(MissionFsm&, bool, std::uint8_t)>& ev,
                std::uint8_t fail_reason) {
  c.push_back({in, name, [ev](MissionFsm& f) { return ev(f, true, 0); }, true, Reject::kNone, next,
               kReasonNone, true});
  c.push_back({in, name, [ev, fail_reason](MissionFsm& f) { return ev(f, false, fail_reason); },
               true, Reject::kNone, S::kError, fail_reason, true});
  for (State s : all_but({in})) {
    c.push_back({s, name, [ev](MissionFsm& f) { return ev(f, true, 0); }, false, Reject::kIllegal,
                 s, 0, false});
  }
}

std::vector<Case> table() {
  std::vector<Case> c;
  // ---- start (pre-arm gate)
  c.push_back({S::kIdle, "start gate ok", [](MissionFsm& f) { return f.start(true, kNow); }, true,
               Reject::kNone, S::kLoading, 0, true});
  c.push_back({S::kIdle, "start gate NOT ok", [](MissionFsm& f) { return f.start(false, kNow); },
               false, Reject::kSafetyGate, S::kIdle, 0, false});
  for (State s : kActive) {
    c.push_back({s, "start while active", [](MissionFsm& f) { return f.start(true, kNow); }, false,
                 Reject::kBusy, s, 0, false});
  }
  for (State s : {S::kCompleted, S::kAborted, S::kError}) {
    c.push_back({s, "restart gate ok", [](MissionFsm& f) { return f.start(true, kNow); }, true,
                 Reject::kNone, S::kLoading, 0, true});
    c.push_back({s, "restart gate NOT ok", [](MissionFsm& f) { return f.start(false, kNow); },
                 false, Reject::kSafetyGate, s, 0, false});
  }
  // ---- the pre-RUNNING steps
  step_cases(
      c, S::kLoading, S::kPlacing, "artifact_loaded",
      [](MissionFsm& f, bool ok, std::uint8_t) { return f.artifact_loaded(ok, kD, kNow); },
      kReasonPathError);
  for (std::uint8_t r :
       {kReasonNoPlacementFrame, kReasonEkfReferenceInvalid, kReasonPlacementOutOfBounds}) {
    step_cases(
        c, S::kPlacing, S::kArming, "placed",
        [](MissionFsm& f, bool ok, std::uint8_t why) { return f.placed(ok, why, kD, kNow); }, r);
  }
  for (std::uint8_t r : {kReasonArmRefused, kReasonArmTimeout}) {
    step_cases(
        c, S::kArming, S::kEngaging, "armed",
        [](MissionFsm& f, bool ok, std::uint8_t why) { return f.armed(ok, why, kD, kNow); }, r);
  }
  for (std::uint8_t r : {kReasonOffboardRefused, kReasonOffboardTimeout}) {
    step_cases(
        c, S::kEngaging, S::kReady, "engaged",
        [](MissionFsm& f, bool ok, std::uint8_t why) { return f.engaged(ok, why, kD, kNow); }, r);
  }
  // ---- rpp_ack: READY -> RUNNING only with the full gate; otherwise held
  c.push_back({S::kReady, "ack gate ok", [](MissionFsm& f) { return f.rpp_ack(true, kNow); }, true,
               Reject::kNone, S::kRunning, 0, true});
  c.push_back({S::kReady, "ack held, gate not ok",
               [](MissionFsm& f) { return f.rpp_ack(false, kNow); }, true, Reject::kNone, S::kReady,
               0, true});
  for (State s : {S::kRunning, S::kPaused}) {
    c.push_back({s, "duplicate ack ignored", [](MissionFsm& f) { return f.rpp_ack(true, kNow); },
                 true, Reject::kNone, s, 0, false});
  }
  for (State s : all_but({S::kReady, S::kRunning, S::kPaused})) {
    c.push_back({s, "ack illegal", [](MissionFsm& f) { return f.rpp_ack(true, kNow); }, false,
                 Reject::kIllegal, s, 0, false});
  }
  // ---- pause
  c.push_back({S::kRunning, "pause", [](MissionFsm& f) { return f.pause(kNow); }, true,
               Reject::kNone, S::kPaused, kReasonOperator, true});
  for (State s : all_but({S::kRunning})) {
    c.push_back({s, "pause not running", [](MissionFsm& f) { return f.pause(kNow); }, false,
                 Reject::kNotRunning, s, 0, false});
  }
  // ---- resume
  c.push_back({S::kPaused, "resume gate ok", [](MissionFsm& f) { return f.resume(true, kNow); },
               true, Reject::kNone, S::kRunning, 0, true});
  c.push_back({S::kPaused, "resume gate NOT ok",
               [](MissionFsm& f) { return f.resume(false, kNow); }, false, Reject::kSafetyGate,
               S::kPaused, 0, false});
  for (State s : all_but({S::kPaused})) {
    c.push_back({s, "resume not paused", [](MissionFsm& f) { return f.resume(true, kNow); }, false,
                 Reject::kNotPaused, s, 0, false});
  }
  // ---- abort
  for (State s : kActive) {
    c.push_back({s, "abort operator", [](MissionFsm& f) { return f.abort(1, kNow); }, true,
                 Reject::kNone, S::kAborted, kReasonOperator, true});
    c.push_back({s, "abort unspecified == operator", [](MissionFsm& f) { return f.abort(0, kNow); },
                 true, Reject::kNone, S::kAborted, kReasonOperator, true});
    c.push_back({s, "abort safety", [](MissionFsm& f) { return f.abort(2, kNow); }, true,
                 Reject::kNone, S::kAborted, kReasonSafety, true});
  }
  for (State s : kInactive) {
    c.push_back({s, "abort not active", [](MissionFsm& f) { return f.abort(1, kNow); }, false,
                 Reject::kNotActive, s, 0, false});
  }
  // ---- rpp_complete
  c.push_back({S::kRunning, "complete", [](MissionFsm& f) { return f.rpp_complete(kNow); }, true,
               Reject::kNone, S::kCompleted, 0, true});
  c.push_back({S::kPaused, "complete while paused ignored",
               [](MissionFsm& f) { return f.rpp_complete(kNow); }, true, Reject::kNone, S::kPaused,
               0, false});
  for (State s : all_but({S::kRunning, S::kPaused})) {
    c.push_back({s, "complete illegal", [](MissionFsm& f) { return f.rpp_complete(kNow); }, false,
                 Reject::kIllegal, s, 0, false});
  }
  // ---- rpp_error: only an execution RPP holds (READY, RUNNING, PAUSED)
  for (State s : {S::kReady, S::kRunning, S::kPaused}) {
    c.push_back({s, "rpp error", [](MissionFsm& f) { return f.rpp_error(kNow); }, true,
                 Reject::kNone, S::kError, kReasonRppError, true});
  }
  for (State s : {S::kLoading, S::kPlacing, S::kArming, S::kEngaging}) {
    c.push_back({s, "rpp error before RPP holds the path",
                 [](MissionFsm& f) { return f.rpp_error(kNow); }, true, Reject::kNone, s, 0,
                 false});
  }
  for (State s : kInactive) {
    c.push_back({s, "rpp error illegal", [](MissionFsm& f) { return f.rpp_error(kNow); }, false,
                 Reject::kIllegal, s, 0, false});
  }
  // ---- gate_lost: ERROR before RUNNING, PAUSED while RUNNING
  for (State s : kBeforeRunning) {
    c.push_back({s, "gate lost before motion -> error",
                 [](MissionFsm& f) { return f.gate_lost(9, kNow); }, true, Reject::kNone, S::kError,
                 kReasonSafety, true});
    c.push_back({s, "rtk lost before motion -> error",
                 [](MissionFsm& f) { return f.gate_lost(6, kNow); }, true, Reject::kNone, S::kError,
                 kReasonRtk, true});
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
  // ---- estop: every active state aborts with REASON_ESTOP
  for (State s : kActive) {
    c.push_back({s, "estop aborts", [](MissionFsm& f) { return f.estop(kNow); }, true,
                 Reject::kNone, S::kAborted, kReasonEstop, true});
  }
  for (State s : kInactive) {
    c.push_back({s, "estop no effect", [](MissionFsm& f) { return f.estop(kNow); }, true,
                 Reject::kNone, s, 0, false});
  }
  // ---- ekf_reset
  for (State s : {S::kReady, S::kRunning, S::kPaused}) {
    c.push_back({s, "ekf reset -> paused", [](MissionFsm& f) { return f.ekf_reset(kD, kNow); },
                 true, Reject::kNone, S::kPaused, kReasonEkfReset, true});
  }
  for (State s : {S::kArming, S::kEngaging}) {
    c.push_back({s, "ekf reset before motion -> error",
                 [](MissionFsm& f) { return f.ekf_reset(kD, kNow); }, true, Reject::kNone,
                 S::kError, kReasonEkfReset, true});
  }
  for (State s : {S::kIdle, S::kLoading, S::kPlacing, S::kCompleted, S::kAborted, S::kError}) {
    c.push_back({s, "ekf reset no effect", [](MissionFsm& f) { return f.ekf_reset(kD, kNow); },
                 true, Reject::kNone, s, 0, false});
  }
  // ---- rpp_stale
  c.push_back({S::kRunning, "rpp stale -> automatic pause",
               [](MissionFsm& f) { return f.rpp_stale(kNow); }, true, Reject::kNone, S::kPaused,
               kReasonRppStale, true});
  for (State s : all_but({S::kRunning})) {
    c.push_back({s, "rpp stale no effect", [](MissionFsm& f) { return f.rpp_stale(kNow); }, true,
                 Reject::kNone, s, 0, false});
  }
  // ---- rpp_ack_timeout
  c.push_back({S::kReady, "rpp ack timeout",
               [](MissionFsm& f) { return f.rpp_ack_timeout(true, 0, kNow); }, true, Reject::kNone,
               S::kError, kReasonRppAckTimeout, true});
  c.push_back({S::kReady, "ack timeout while the full gate never passed",
               [](MissionFsm& f) { return f.rpp_ack_timeout(false, 11, kNow); }, true,
               Reject::kNone, S::kError, kReasonSafety, true});
  for (State s : all_but({S::kReady})) {
    c.push_back({s, "rpp ack timeout no effect",
                 [](MissionFsm& f) { return f.rpp_ack_timeout(true, 0, kNow); }, true,
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
  for (State s : all_but({S::kRunning, S::kPaused})) {
    c.push_back({s, "skip not running", [](MissionFsm& f) { return f.skip_point(true, kNow); },
                 false, Reject::kNotRunning, s, 0, false});
  }
  return c;
}

}  // namespace

TEST(MissionFsmTable, EveryStateEventPairMatchesTheContract) {
  const auto cases = table();
  EXPECT_GT(cases.size(), 250U);
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
    const bool started = std::string(c.what).find("start") != std::string::npos;
    if (started && c.accepted && std::string(c.what).find("while") == std::string::npos) {
      EXPECT_EQ(f.mission_id(), id_before + 1);
    } else {
      EXPECT_EQ(f.mission_id(), id_before);
    }
    // A refusal never changes state.
    if (!c.accepted) EXPECT_EQ(f.state(), c.from);
  }
}

TEST(MissionFsmLifecycle, HappyPathVisitsEveryStepInOrderWithTimestamps) {
  MissionFsm f;
  std::vector<Transition> seen;
  f.set_observer([&](const Transition& t) { seen.push_back(t); });
  f.start(true, 10);
  f.artifact_loaded(true, "artifact verified", 20);
  f.placed(true, 0, "placed", 30);
  f.armed(true, 0, "armed", 40);
  f.engaged(true, 0, "offboard", 50);
  f.rpp_ack(true, 60);
  const std::vector<State> order = {S::kLoading,  S::kPlacing, S::kArming,
                                    S::kEngaging, S::kReady,   S::kRunning};
  ASSERT_EQ(seen.size(), order.size());
  for (std::size_t i = 0; i < order.size(); ++i) {
    EXPECT_EQ(seen[i].to, order[i]);
    EXPECT_EQ(seen[i].stamp_ns, static_cast<std::int64_t>(10 * (i + 1)));
    EXPECT_FALSE(seen[i].refused);
  }
  EXPECT_EQ(f.state_entered_ns(), 60);
  EXPECT_EQ(f.changes(), 6U);
  EXPECT_EQ(f.detail(), "rpp acknowledged the execution");
}

TEST(MissionFsmGuards, NeverAutoResumesAndNeverRunsWithoutAGate) {
  MissionFsm f = in_state(S::kRunning);
  EXPECT_EQ(f.gate_lost(6, kNow).state, S::kPaused);
  EXPECT_EQ(f.gate_reason(), 6U);
  // The gate recovering changes nothing: only an explicit resume does.
  EXPECT_EQ(f.gate_lost(0, kNow).state, S::kPaused);
  EXPECT_EQ(f.rpp_ack(true, kNow).state, S::kPaused);
  EXPECT_FALSE(f.resume(false, kNow).accepted);
  EXPECT_EQ(f.state(), S::kPaused);
  EXPECT_TRUE(f.resume(true, kNow).accepted);
  EXPECT_EQ(f.state(), S::kRunning);
  EXPECT_EQ(f.gate_reason(), 0U);
}

TEST(MissionFsmGuards, ReadyNeverRunsWithoutTheFullGate) {
  MissionFsm f = in_state(S::kReady);
  for (int i = 0; i < 5; ++i) EXPECT_EQ(f.rpp_ack(false, kNow).state, S::kReady);
  EXPECT_EQ(f.rpp_ack(true, kNow).state, S::kRunning);
}

TEST(MissionFsmGuards, EkfResetPausesAndNeverAutoResumes) {
  MissionFsm f = in_state(S::kRunning);
  const auto before = f.changes();
  EXPECT_EQ(f.ekf_reset("xy reset", kNow).state, S::kPaused);
  EXPECT_EQ(f.reason(), kReasonEkfReset);
  EXPECT_EQ(f.ekf_reset("again", kNow).state, S::kPaused);  // reported, still paused
  EXPECT_EQ(f.changes(), before + 2);
  f.gate_lost(0, kNow);
  f.rpp_ack(true, kNow);
  EXPECT_EQ(f.state(), S::kPaused);
  EXPECT_TRUE(f.resume(true, kNow).accepted);
}

TEST(MissionFsmGuards, OperatorPauseThenEkfResetReplacesThePauseReason) {
  MissionFsm f = in_state(S::kPaused);
  EXPECT_EQ(f.reason(), kReasonOperator);
  f.ekf_reset("xy reset", kNow);
  EXPECT_EQ(f.state(), S::kPaused);
  EXPECT_EQ(f.reason(), kReasonEkfReset);
}

TEST(MissionFsmGuards, StaleRppPausesWithADistinctReasonAndNeverAutoResumes) {
  MissionFsm f = in_state(S::kRunning);
  std::vector<Transition> seen;
  f.set_observer([&](const Transition& t) { seen.push_back(t); });
  EXPECT_EQ(f.rpp_stale(kNow).state, S::kPaused);
  ASSERT_EQ(seen.size(), 1U);
  EXPECT_EQ(seen[0].event, Event::kRppStale);
  EXPECT_STREQ(to_string(seen[0].event), "rpp_stale");
  EXPECT_EQ(seen[0].reason, kReasonRppStale);
  f.rpp_stale(kNow);
  f.gate_lost(0, kNow);
  f.rpp_ack(true, kNow);
  EXPECT_EQ(f.state(), S::kPaused);
  EXPECT_TRUE(f.resume(true, kNow).accepted);  // only an explicit resume
  EXPECT_EQ(f.state(), S::kRunning);
}

TEST(MissionFsmGuards, TerminalStatesAreLeftOnlyByANewStart) {
  for (State s : {S::kCompleted, S::kAborted, S::kError}) {
    MissionFsm f = in_state(s);
    f.pause(kNow);
    f.resume(true, kNow);
    f.abort(1, kNow);
    f.rpp_complete(kNow);
    f.rpp_error(kNow);
    f.gate_lost(5, kNow);
    f.estop(kNow);
    f.ekf_reset(kD, kNow);
    f.rpp_stale(kNow);
    f.rpp_ack_timeout(true, 0, kNow);
    f.skip_point(true, kNow);
    f.artifact_loaded(true, kD, kNow);
    f.placed(true, 0, kD, kNow);
    f.armed(true, 0, kD, kNow);
    f.engaged(true, 0, kD, kNow);
    f.rpp_ack(true, kNow);
    EXPECT_EQ(f.state(), s);
    EXPECT_TRUE(f.start(true, kNow).accepted);
    EXPECT_EQ(f.state(), S::kLoading);
  }
}

TEST(MissionFsmGuards, GuardReasonMapping) {
  EXPECT_EQ(map_guard_reason(6), kReasonRtk);  // MotionSetpointStatus::REASON_RTK_GATE
  for (std::uint8_t r : {0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 200}) {
    EXPECT_EQ(map_guard_reason(r), kReasonSafety);
  }
}

TEST(MissionFsmLog, TransitionsCarryReasonSequenceAndAreBounded) {
  MissionFsm f = in_state(S::kRunning);
  std::vector<Transition> seen;
  f.set_observer([&](const Transition& t) { seen.push_back(t); });
  f.gate_lost(6, 40);
  ASSERT_EQ(seen.size(), 1U);
  EXPECT_EQ(seen[0].from, S::kRunning);
  EXPECT_EQ(seen[0].to, S::kPaused);
  EXPECT_EQ(seen[0].reason, kReasonRtk);
  EXPECT_EQ(seen[0].stamp_ns, 40);
  EXPECT_FALSE(seen[0].detail.empty());
  for (int i = 0; i < 1000; ++i) f.pause(i);  // refusals are logged too, but the log stays bounded
  EXPECT_LE(f.log().size(), MissionFsm::kLogCapacity);
  EXPECT_GT(f.transitions_total(), 1000U);
}

TEST(MissionFsmProperty, InvariantsHoldUnderRandomEventSequences) {
  std::mt19937 rng(20261010);
  for (int run = 0; run < 200; ++run) {
    MissionFsm f;
    std::vector<Transition> log;
    f.set_observer([&](const Transition& t) { log.push_back(t); });
    std::uint32_t last_id = 0;
    for (int i = 0; i < 400; ++i) {
      const bool ok = rng() % 4 != 0;
      const std::uint8_t reason = static_cast<std::uint8_t>(rng() % 18);
      const State before = f.state();
      switch (rng() % 17) {
        case 0:
          f.start(ok, i);
          break;
        case 1:
          f.artifact_loaded(ok, kD, i);
          break;
        case 2:
          f.placed(ok, kReasonEkfReferenceInvalid, kD, i);
          break;
        case 3:
          f.armed(ok, kReasonArmTimeout, kD, i);
          break;
        case 4:
          f.engaged(ok, kReasonOffboardRefused, kD, i);
          break;
        case 5:
          f.rpp_ack(ok, i);
          break;
        case 6:
          f.pause(i);
          break;
        case 7:
          f.resume(ok, i);
          break;
        case 8:
          f.abort(reason % 3, i);
          break;
        case 9:
          f.rpp_complete(i);
          break;
        case 10:
          f.rpp_error(i);
          break;
        case 11:
          f.gate_lost(reason, i);
          break;
        case 12:
          f.estop(i);
          break;
        case 13:
          f.rpp_stale(i);
          break;
        case 14:
          f.rpp_ack_timeout(ok, reason, i);
          break;
        case 15:
          f.ekf_reset(kD, i);
          break;
        default:
          f.skip_point(rng() % 2, i);
          break;
      }
      const State after = f.state();
      ASSERT_LE(static_cast<int>(after), 10);
      if (after == S::kRunning) EXPECT_EQ(f.reason(), kReasonNone);
      if (after == S::kAborted)
        EXPECT_TRUE(f.reason() == kReasonOperator || f.reason() == kReasonSafety ||
                    f.reason() == kReasonEstop);
      if (after == S::kPaused)
        EXPECT_TRUE(f.reason() == kReasonOperator || f.reason() == kReasonSafety ||
                    f.reason() == kReasonRtk || f.reason() == kReasonEkfReset ||
                    f.reason() == kReasonRppStale);
      if (after == S::kError) EXPECT_NE(f.reason(), kReasonNone);
      // terminal states are left only through an accepted start into LOADING
      if ((before == S::kCompleted || before == S::kAborted || before == S::kError) &&
          after != before) {
        EXPECT_EQ(after, S::kLoading);
      }
      EXPECT_GE(f.mission_id(), last_id);
      EXPECT_LE(f.mission_id(), last_id + 1);
      last_id = f.mission_id();
    }
    for (const auto& t : log) {
      EXPECT_FALSE(t.detail.empty());
      if (t.refused) EXPECT_EQ(t.from, t.to);
      // RUNNING is only ever entered from READY (rpp_ack) or PAUSED (resume).
      if (!t.refused && t.to == S::kRunning && t.from != S::kRunning) {
        EXPECT_TRUE((t.from == S::kReady && t.event == Event::kRppAck) ||
                    (t.from == S::kPaused && t.event == Event::kResume));
      }
      // ARMING is only ever entered from PLACING, ENGAGING only from ARMING, READY only from
      // ENGAGING: the vehicle is never armed or engaged out of order.
      if (!t.refused && t.to != t.from) {
        if (t.to == S::kArming) EXPECT_EQ(t.from, S::kPlacing);
        if (t.to == S::kEngaging) EXPECT_EQ(t.from, S::kArming);
        if (t.to == S::kReady) EXPECT_EQ(t.from, S::kEngaging);
      }
      if (!t.refused && t.from == S::kPaused && t.to == S::kRunning)
        EXPECT_EQ(t.event, Event::kResume);
    }
  }
}
