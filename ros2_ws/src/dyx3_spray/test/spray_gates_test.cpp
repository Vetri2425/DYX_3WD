#include "dyx3_spray/spray_gates.hpp"

#include <gtest/gtest.h>

using namespace dyx3_spray;

namespace {
RtkGateConfig cfg() {
  RtkGateConfig c;
  c.require_rtk_fix = true;
  c.min_fix_type = 6;
  c.max_h_acc_m = 0.10;
  c.require_accuracy = true;
  c.fix_timeout_s = 0.5;
  c.recover_hold_s = 1.0;
  return c;
}
GateInputs all_good() {
  GateInputs g;
  g.armed = g.offboard = g.path_loaded = g.pose_fresh = g.velocity_fresh = g.tracking_seen =
      g.estop_clear = true;
  g.require_offboard = true;
  g.rtk = {true, ""};
  return g;
}
}  // namespace

TEST(RtkQuality, OnlyRoverRtkFixTypesAreAcceptable) {
  for (int fix = 0; fix <= 9; ++fix) {
    const auto q = evaluate_rtk_quality(fix, 0.02, 0.1, 0.5, 5, 0.1, true);
    EXPECT_EQ(q.acceptable, fix == 5 || fix == 6)
        << "fix " << fix;  // STATIC (7) and PPP (8) are not a rover solution
  }
}

TEST(RtkQuality, UnknownAccuracyFailsClosedUnlessTheBenchSwitchIsOff) {
  EXPECT_FALSE(evaluate_rtk_quality(6, std::nullopt, 0.1, 0.5, 6, 0.1, true).acceptable);
  EXPECT_TRUE(evaluate_rtk_quality(6, std::nullopt, 0.1, 0.5, 6, 0.1, false).acceptable);
}

TEST(RtkQuality, NeverReceivedAndStaleAreNotFresh) {
  EXPECT_FALSE(evaluate_rtk_quality(6, 0.02, std::nullopt, 0.5, 6, 0.1, true).fresh);
  EXPECT_FALSE(evaluate_rtk_quality(6, 0.02, 0.6, 0.5, 6, 0.1, true).fresh);
  EXPECT_FALSE(evaluate_rtk_quality(6, 0.02, -0.01, 0.5, 6, 0.1, true).fresh);
}

TEST(RtkGate, DropIsInstantRecoveryNeedsAContinuousHold) {
  RtkGate g;
  const auto c = cfg();
  double t = 10.0;
  EXPECT_FALSE(g.evaluate(c, 6, 0.02, 0.05, t).ok);  // first good sample starts the hold
  t += 0.6;
  EXPECT_FALSE(g.evaluate(c, 6, 0.02, 0.05, t).ok);
  t += 0.5;
  EXPECT_TRUE(g.evaluate(c, 6, 0.02, 0.05, t).ok);  // 1.1 s of continuous good fix
  t += 0.02;
  EXPECT_FALSE(g.evaluate(c, 5, 0.5, 0.05, t).ok);  // float with poor accuracy: instant drop
  t += 0.02;
  const auto back = g.evaluate(c, 6, 0.02, 0.05, t);
  EXPECT_FALSE(back.ok);  // one good sample after a dropout does NOT reopen the valve
  EXPECT_NE(back.reason.find("recovering"), std::string::npos);
  t += 1.1;
  EXPECT_TRUE(g.evaluate(c, 6, 0.02, 0.05, t).ok);
}

TEST(RtkGate, AStaleSampleResetsTheHold) {
  RtkGate g;
  const auto c = cfg();
  g.evaluate(c, 6, 0.02, 0.05, 1.0);
  EXPECT_TRUE(g.evaluate(c, 6, 0.02, 0.05, 2.5).ok);
  EXPECT_FALSE(g.evaluate(c, 6, 0.02, 0.9, 2.52).ok);  // sample older than the timeout
  EXPECT_FALSE(g.evaluate(c, 6, 0.02, 0.05, 2.54).ok);
}

TEST(PivotGate, OnlyPivotingCountsAndStaleStateFailsOpenImmediately) {
  PivotGate p;
  EXPECT_FALSE(p.active(true, 1.0, 5.0));  // absent state is "not pivoting"
  p.note_state(true, 5.0);
  EXPECT_TRUE(p.active(true, 1.0, 5.5));
  EXPECT_FALSE(p.active(false, 1.0, 5.5));  // switched off
  EXPECT_FALSE(p.active(true, 1.0, 6.2));   // older than the timeout: released...
  EXPECT_FALSE(p.active(true, 1.0, 6.3));   // ...and stays released until a genuinely fresh message
  p.note_state(false, 6.4);
  EXPECT_FALSE(p.active(true, 1.0, 6.5));
}

TEST(GateStack, EveryGateFailsOnItsOwnAndTheFirstFailureWinsInOrder) {
  EXPECT_TRUE(auto_safety_status(all_good()).ok);
  struct Case {
    void (*mutate)(GateInputs&);
    const char* reason;
  };
  const Case cases[] = {
      {[](GateInputs& g) { g.estop_clear = false; }, "emergency stop asserted or unknown"},
      {[](GateInputs& g) { g.armed = false; }, "disarmed"},
      {[](GateInputs& g) { g.offboard = false; }, "not OFFBOARD"},
      {[](GateInputs& g) { g.path_loaded = false; }, "path not loaded"},
      {[](GateInputs& g) { g.pose_fresh = false; }, "pose stale"},
      {[](GateInputs& g) { g.velocity_fresh = false; }, "velocity stale"},
      {[](GateInputs& g) { g.rtk = {false, "gps stale"}; }, "gps stale"},
      {[](GateInputs& g) { g.tracking_seen = false; }, "awaiting tracking"},
      {[](GateInputs& g) { g.pivoting = true; }, "pivoting in place"},
  };
  for (const auto& c : cases) {
    GateInputs g = all_good();
    c.mutate(g);
    const auto r = auto_safety_status(g);
    EXPECT_FALSE(r.ok) << c.reason;
    EXPECT_EQ(r.reason, c.reason);
  }
  // order: with everything failing, E-stop reports first; clearing it exposes the next one
  GateInputs g;
  g.require_offboard = true;
  const char* order[] = {"emergency stop asserted or unknown",
                         "disarmed",
                         "not OFFBOARD",
                         "path not loaded",
                         "pose stale",
                         "velocity stale",
                         "gps stale",
                         "awaiting tracking",
                         "pivoting in place"};
  g.rtk = {false, "gps stale"};
  g.pivoting = true;
  EXPECT_EQ(auto_safety_status(g).reason, order[0]);
  g.estop_clear = true;
  EXPECT_EQ(auto_safety_status(g).reason, order[1]);
  g.armed = true;
  EXPECT_EQ(auto_safety_status(g).reason, order[2]);
  g.offboard = true;
  EXPECT_EQ(auto_safety_status(g).reason, order[3]);
  g.path_loaded = true;
  EXPECT_EQ(auto_safety_status(g).reason, order[4]);
  g.pose_fresh = true;
  EXPECT_EQ(auto_safety_status(g).reason, order[5]);
  g.velocity_fresh = true;
  EXPECT_EQ(auto_safety_status(g).reason, order[6]);
  g.rtk = {true, ""};
  EXPECT_EQ(auto_safety_status(g).reason, order[7]);
  g.tracking_seen = true;
  EXPECT_EQ(auto_safety_status(g).reason, order[8]);
}

TEST(GateStack, OffboardNotRequiredWhenSwitchedOff) {
  GateInputs g = all_good();
  g.offboard = false;
  g.require_offboard = false;
  EXPECT_TRUE(auto_safety_status(g).ok);
}
