// Px4Sequencer: what is sent, in which order, with which timeout, and what the release disarms.
// Pure logic on an injected clock: every timeout branch is exercised without waiting.
#include "dyx3_mission/px4_sequencer.hpp"

#include <gtest/gtest.h>

#include <vector>

using namespace dyx3_mission;  // NOLINT

namespace {

constexpr std::int64_t kS = 1'000'000'000;

// Sends everything next() offers at `now` and returns the ops in order.
std::vector<Px4Request> drain(Px4Sequencer& s, std::int64_t now) {
  std::vector<Px4Request> out;
  while (auto r = s.next(now)) out.push_back(*r);
  return out;
}

// Arms (confirmed) and returns the sequencer with nothing in flight.
Px4Sequencer armed_and_engaged() {
  Px4Sequencer s;
  s.begin_execution();
  s.engage(Px4Op::kArm);
  auto r = drain(s, 0);
  EXPECT_TRUE(s.on_response(r.at(0).id, true, 0, 1)->result == Px4Result::kOk);
  s.engage(Px4Op::kOffboardOn);
  r = drain(s, 2);
  EXPECT_TRUE(s.on_response(r.at(0).id, true, 0, 3)->result == Px4Result::kOk);
  EXPECT_FALSE(s.busy());
  return s;
}

}  // namespace

TEST(Px4Sequencer, OneEngageStepInFlightAtATime) {
  Px4Sequencer s;
  s.begin_execution();
  s.engage(Px4Op::kArm);
  s.engage(Px4Op::kOffboardOn);
  auto r = drain(s, 0);
  ASSERT_EQ(r.size(), 1U);  // OFFBOARD waits for the arm
  EXPECT_EQ(r[0].op, Px4Op::kArm);
  EXPECT_TRUE(s.busy());
  ASSERT_TRUE(s.on_response(r[0].id, true, 0, 1).has_value());
  r = drain(s, 1);
  ASSERT_EQ(r.size(), 1U);
  EXPECT_EQ(r[0].op, Px4Op::kOffboardOn);
}

TEST(Px4Sequencer, TimeoutsArePerOperation) {
  Px4Sequencer s;
  s.set_timeouts({4.0, 5.0});
  s.begin_execution();
  s.engage(Px4Op::kArm);
  const auto arm = drain(s, 0).at(0);
  EXPECT_TRUE(s.on_tick(4 * kS - 1).empty());
  const auto t = s.on_tick(4 * kS);
  ASSERT_EQ(t.size(), 1U);
  EXPECT_EQ(t[0].op, Px4Op::kArm);
  EXPECT_EQ(t[0].result, Px4Result::kTimeout);
  EXPECT_FALSE(s.on_response(arm.id, true, 0, 5 * kS).has_value());  // a late reply is ignored

  Px4Sequencer o;
  o.set_timeouts({4.0, 5.0});
  o.begin_execution();
  o.engage(Px4Op::kOffboardOn);
  drain(o, 0);
  EXPECT_TRUE(o.on_tick(5 * kS - 1).empty());
  EXPECT_EQ(o.on_tick(5 * kS).at(0).result, Px4Result::kTimeout);
}

TEST(Px4Sequencer, RefusedArmNeedsNoDisarm) {
  Px4Sequencer s;
  s.begin_execution();
  s.engage(Px4Op::kArm);
  const auto r = drain(s, 0).at(0);
  EXPECT_EQ(s.on_response(r.id, false, 1, 1)->result, Px4Result::kRefused);  // LINK_UNHEALTHY
  EXPECT_FALSE(s.arm_owned());
  s.release(false);
  EXPECT_TRUE(drain(s, 2).empty());
  EXPECT_FALSE(s.busy());
}

TEST(Px4Sequencer, ArmTimeoutIsADoubtAndIsDisarmed) {
  for (const bool px4_link_timeout : {true, false}) {
    Px4Sequencer s;
    s.begin_execution();
    s.engage(Px4Op::kArm);
    const auto r = drain(s, 0).at(0);
    if (px4_link_timeout) {
      s.on_response(r.id, false, kArmReasonTimeout, 1);  // px4_link: PX4 did not confirm
    } else {
      s.on_tick(10 * kS);  // no reply at all
    }
    EXPECT_TRUE(s.arm_owned());
    s.release(false);
    const auto rel = drain(s, 10 * kS);
    ASSERT_EQ(rel.size(), 1U);
    EXPECT_EQ(rel[0].op, Px4Op::kDisarm);  // no OFFBOARD was ever requested
  }
}

TEST(Px4Sequencer, ReleaseIsOffboardOffThenDisarmOneAtATime) {
  Px4Sequencer s = armed_and_engaged();
  s.release(false);
  auto r = drain(s, 10);
  ASSERT_EQ(r.size(), 1U);
  EXPECT_EQ(r[0].op, Px4Op::kOffboardOff);
  EXPECT_EQ(s.release_pending(), Px4Op::kOffboardOff);
  s.on_response(r[0].id, true, 0, 11);
  r = drain(s, 11);
  ASSERT_EQ(r.size(), 1U);
  EXPECT_EQ(r[0].op, Px4Op::kDisarm);
  EXPECT_EQ(s.release_pending(), Px4Op::kDisarm);
  s.on_response(r[0].id, true, 0, 12);
  EXPECT_FALSE(s.busy());
  EXPECT_FALSE(s.release_pending().has_value());
}

TEST(Px4Sequencer, AFailedOrSilentOffboardOffStillDisarms) {
  Px4Sequencer s = armed_and_engaged();
  s.release(false);
  drain(s, 0);
  const auto t = s.on_tick(100 * kS);
  ASSERT_EQ(t.size(), 1U);
  EXPECT_EQ(t[0].op, Px4Op::kOffboardOff);
  const auto r = drain(s, 100 * kS);
  ASSERT_EQ(r.size(), 1U);
  EXPECT_EQ(r[0].op, Px4Op::kDisarm);
}

TEST(Px4Sequencer, OffboardRequestedButRefusedIsReleasedAndDisarmed) {
  Px4Sequencer s;
  s.begin_execution();
  s.engage(Px4Op::kArm);
  s.on_response(drain(s, 0).at(0).id, true, 0, 1);
  s.engage(Px4Op::kOffboardOn);
  s.on_response(drain(s, 1).at(0).id, false, 2, 2);  // NOT_ARMED_OR_REJECTED
  s.release(false);
  auto r = drain(s, 3);
  ASSERT_EQ(r.size(), 1U);
  EXPECT_EQ(r[0].op, Px4Op::kOffboardOff);  // the heartbeat may be streaming: withdraw it
  s.on_response(r[0].id, true, 0, 4);
  EXPECT_EQ(drain(s, 4).at(0).op, Px4Op::kDisarm);
}

TEST(Px4Sequencer, ReleaseOvertakesAnInFlightEngageStep) {
  Px4Sequencer s;
  s.begin_execution();
  s.engage(Px4Op::kArm);
  const auto arm = drain(s, 0).at(0);
  s.release(false);  // e.g. E-stop while PX4 is still confirming the arm
  const auto r = drain(s, 1);
  ASSERT_EQ(r.size(), 1U);
  EXPECT_EQ(r[0].op, Px4Op::kDisarm);  // sent at once, not after the arm's reply
  const auto late = s.on_response(arm.id, true, 0, 2);
  ASSERT_TRUE(late.has_value());
  EXPECT_TRUE(late->abandoned);
}

TEST(Px4Sequencer, ForcedDisarmEvenWhenNothingWasArmed) {
  Px4Sequencer s;
  s.begin_execution();
  s.release(true);  // E-stop before ARMING: the owner rule is "E-stop disarms"
  const auto r = drain(s, 0);
  ASSERT_EQ(r.size(), 1U);
  EXPECT_EQ(r[0].op, Px4Op::kDisarm);
}

TEST(Px4Sequencer, EstopDuringAReleaseAddsTheDisarmOnce) {
  Px4Sequencer s;
  s.begin_execution();
  s.engage(Px4Op::kArm);
  s.on_response(drain(s, 0).at(0).id, false, 1, 1);  // refused: nothing to disarm
  s.release(false);
  EXPECT_FALSE(s.busy());
  s.release(true);
  s.release(true);
  const auto r = drain(s, 2);
  ASSERT_EQ(r.size(), 1U);
  EXPECT_EQ(r[0].op, Px4Op::kDisarm);
  s.release(true);  // the disarm is in flight: not queued twice
  EXPECT_TRUE(drain(s, 3).empty());
}

TEST(Px4Sequencer, NothingEngagesDuringARelease) {
  Px4Sequencer s = armed_and_engaged();
  s.release(false);
  s.engage(Px4Op::kArm);
  for (const auto& r : drain(s, 0)) EXPECT_NE(r.op, Px4Op::kArm);
}

TEST(Px4Sequencer, BeginExecutionForgetsThePrevious) {
  Px4Sequencer s = armed_and_engaged();
  EXPECT_TRUE(s.arm_owned());
  EXPECT_TRUE(s.offboard_requested());
  s.begin_execution();
  EXPECT_FALSE(s.arm_owned());
  EXPECT_FALSE(s.offboard_requested());
  EXPECT_FALSE(s.releasing());
  s.release(false);
  EXPECT_FALSE(s.busy());
}

// The deadline is relative to the clock the node passes to next(): the steady clock counts from
// boot, so its values are arbitrary and large, and no epoch is assumed.
TEST(Px4Sequencer, DeadlinesAreRelativeToTheClockThatSentTheRequest) {
  constexpr std::int64_t kBase = 987'654 * kS;
  Px4Sequencer s;
  s.set_timeouts({4.0, 5.0});
  s.begin_execution();
  s.engage(Px4Op::kArm);
  drain(s, kBase);
  EXPECT_TRUE(s.on_tick(0).empty());  // an earlier reading (another epoch) never expires it
  EXPECT_TRUE(s.on_tick(kBase + 4 * kS - 1).empty());
  const auto t = s.on_tick(kBase + 4 * kS);
  ASSERT_EQ(t.size(), 1U);
  EXPECT_EQ(t[0].result, Px4Result::kTimeout);
}

// px4_link's REASON_REJECTED_BY_FCU (2) is a definitive refusal, not a doubt: it is reported at
// once with its code, owns nothing to disarm and is not a timeout.
TEST(Px4Sequencer, ArmRejectedByTheFcuIsADefiniteRefusal) {
  Px4Sequencer s;
  s.begin_execution();
  s.engage(Px4Op::kArm);
  const auto r = drain(s, 0).at(0);
  const auto o = s.on_response(r.id, false, 2, 1);  // REJECTED_BY_FCU
  ASSERT_TRUE(o.has_value());
  EXPECT_EQ(o->result, Px4Result::kRefused);
  EXPECT_EQ(o->reason_code, 2);
  EXPECT_FALSE(s.arm_owned());
  s.release(false);
  EXPECT_FALSE(s.busy());  // nothing to release
}
