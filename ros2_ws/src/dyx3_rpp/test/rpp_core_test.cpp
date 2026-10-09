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
