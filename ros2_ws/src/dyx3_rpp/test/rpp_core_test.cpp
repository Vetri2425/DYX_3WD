// Behaviour tests of RppCore beyond the prototype (the tick-by-tick equivalence with the carried
// node is orchestrator_equivalence_test). Each case pins a production fix that the prototype did
// not have, on plain inputs and an injected clock.
#include "dyx3_rpp/rpp_core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

using namespace dyx3_rpp;

namespace {

constexpr int64_t kTickNs = 20'000'000;

// A straight run due North from (0, 0), `len` metres, with points every `step` metres.
ConditionedRun north_run(double len, Profile profile, double step) {
  ConditionedRun r;
  r.profile = profile;
  const int n = static_cast<int>(std::lround(len / step));
  for (int i = 0; i <= n; ++i) {
    r.pts.push_back({len * i / n, 0.0});
    r.flags.push_back(0);
    r.must_hit.push_back(0);
    r.cum_s.push_back(len * i / n);
  }
  r.length = len;
  return r;
}

struct CoreRig {
  ParamSet params;
  std::unique_ptr<RppCore> core;
  int64_t now{1'000'000'000};
  double n{0.0}, e{0.0}, yaw{0.0}, vn{0.0}, ve{0.0};
  bool feed_pose{true}, feed_vel{true};

  CoreRig(std::vector<ConditionedRun> runs, std::vector<Item> extra = {}) {
    std::vector<Item> items = {{"require_rtk_fix", 0.0, ""}, {"entry_prealign_enabled", 0.0, ""}};
    items.insert(items.end(), extra.begin(), extra.end());
    const SetResult r = params.init_many(items);
    EXPECT_TRUE(r.ok) << r.reason;
    core = std::make_unique<RppCore>(params);
    core->install_mission(std::move(runs));
  }
  const TickOutput& step() {
    now += kTickNs;
    if (feed_pose) core->on_pose(NedPose{n, e, yaw}, now);
    if (feed_vel) core->on_velocity(vn, ve, 0.0, now);
    return core->tick(now);
  }
};

}  // namespace

// XR-RPP-001 safety net (BEHAVIOUR CHANGE): a precise stop that reaches its timeout while the
// rover is still moving brakes to a confirmed stop and completes; it never creeps on forever.
TEST(RppCore, APreciseStopPastItsTimeoutBrakesAndCompletes) {
  CoreRig r({north_run(6.0, Profile::Segment, 6.0)});
  r.n = 5.99;  // inside the arrival tolerance, but the velocity never settles below the threshold
  r.vn = 0.05;
  bool engaged = false;
  for (int i = 0; i < 300; ++i) {  // 6 s: the default timeout is 8 s
    const TickOutput& o = r.step();
    engaged = engaged || r.core->snapshot().endpoint_stop_active;
    EXPECT_NE(o.cmd, CmdKind::Brake) << "tick " << i;
  }
  ASSERT_TRUE(engaged);
  CmdKind last = CmdKind::Stop;
  for (int i = 0; i < 150; ++i) last = r.step().cmd;  // past 8 s
  EXPECT_EQ(last, CmdKind::Brake);
  EXPECT_FALSE(r.core->path_done());
  r.vn = 0.0;  // the brake took effect
  for (int i = 0; i < 30 && !r.core->path_done(); ++i) r.step();
  EXPECT_TRUE(r.core->path_done());
}

// XR-RPP-005: RppStatus.cross_track_right_m is right-positive (frames.md). The precise stop's
// legacy debug cross-track is left-positive; the status value must not be.
TEST(RppCore, ThePreciseStopReportsCrossTrackRightPositive) {
  CoreRig r({north_run(6.0, Profile::Segment, 6.0)});
  r.n = 5.95;
  r.e = 0.03;  // RIGHT of a northbound line
  bool seen = false;
  for (int i = 0; i < 5; ++i) {
    const TickOutput& o = r.step();
    if (!r.core->snapshot().endpoint_stop_active) continue;
    seen = true;
    EXPECT_NEAR(o.debug.cross_track, -0.03, 1e-9);  // the prototype's sign, kept for equivalence
    EXPECT_NEAR(o.cross_track_right, 0.03, 1e-9);
  }
  EXPECT_TRUE(seen);
  // and while tracking it is the geometry's right-positive cross-track
  CoreRig t({north_run(6.0, Profile::Segment, 6.0)});
  t.n = 1.0;
  t.e = 0.03;
  const TickOutput& o = t.step();
  EXPECT_FALSE(t.core->snapshot().endpoint_stop_active);
  EXPECT_NEAR(o.cross_track_right, 0.03, 1e-9);
  EXPECT_DOUBLE_EQ(o.cross_track_right, o.debug.cross_track);
}

// XR-RPP-008: pause, the rover coasts 0.2 m, resume. The coast is not a position jump: no
// JumpSkip, and with EKF reset compensation on, no permanent offset equal to the coast.
TEST(RppCore, AResumeAfterACoastIsAFreshStartNotAJump) {
  for (const bool comp : {false, true}) {
    CoreRig r({north_run(6.0, Profile::Segment, 6.0)},
              {{"ekf_reset_compensation", comp ? 1.0 : 0.0, ""}});
    r.vn = 0.5;
    for (int i = 0; i < 50; ++i) {  // 1 s at 0.5 m/s
      r.n += 0.5 * 0.02;
      const TickOutput& o = r.step();
      ASSERT_NE(o.state, StateCode::JumpSkip) << "tick " << i;
    }
    r.core->pause();
    for (int i = 0; i < 20; ++i) {  // the node does not tick while paused; poses keep arriving
      r.n += 0.01;
      r.now += kTickNs;
      r.core->on_pose(NedPose{r.n, r.e, r.yaw}, r.now);
      r.core->on_velocity(0.5, 0.0, 0.0, r.now);
    }
    r.vn = 0.0;
    for (int i = 0; i < 10; ++i) {
      const TickOutput& o = r.step();
      EXPECT_NE(o.state, StateCode::JumpSkip) << "comp " << comp << " tick " << i;
    }
    const CoreState st = r.core->snapshot();
    EXPECT_EQ(st.ekf_reset_count, 0) << "comp " << comp;
    EXPECT_DOUBLE_EQ(st.ekf_offset_n, 0.0) << "comp " << comp;
    EXPECT_DOUBLE_EQ(st.ekf_offset_e, 0.0) << "comp " << comp;
    EXPECT_NEAR(st.tick_dt, 0.02, 1e-9);
  }
}

// XR-RPP-008: the precise-stop timeout does not count through a pause.
TEST(RppCore, APauseRestartsThePreciseStopTimer) {
  CoreRig r({north_run(6.0, Profile::Segment, 6.0)});
  r.n = 5.99;
  r.vn = 0.05;  // never settles: only the timeout can end the precise stop
  for (int i = 0; i < 300; ++i) r.step();  // 6 s of the 8 s budget
  ASSERT_TRUE(r.core->snapshot().endpoint_stop_active);
  r.core->pause();
  r.now += 10'000'000'000;       // a 10 s pause
  for (int i = 0; i < 150; ++i)  // 3 s after the resume: still inside a fresh 8 s budget
    EXPECT_NE(r.step().cmd, CmdKind::Brake) << "tick " << i;
  for (int i = 0; i < 300; ++i) r.step();  // past 8 s since the re-engagement
  EXPECT_EQ(r.step().cmd, CmdKind::Brake);
}
