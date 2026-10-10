// Behaviour tests of RppCore beyond the prototype (the tick-by-tick equivalence with the carried
// node is orchestrator_equivalence_test). Each case pins a production fix that the prototype did
// not have, on plain inputs and an injected clock.
#include "dyx3_rpp/rpp_core.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>
#include <vector>

using namespace dyx3_rpp;

// Global allocation counter (replaceable operator new): counts only while armed.
namespace {
std::atomic<bool> g_count_allocs{false};
std::atomic<long> g_allocs{0};
}  // namespace
void* operator new(std::size_t n) {
  if (g_count_allocs.load(std::memory_order_relaxed)) g_allocs.fetch_add(1);
  if (void* p = std::malloc(n == 0 ? 1 : n)) return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

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

// A straight segment-profile run from `a` to `b` with points every `step` metres.
ConditionedRun leg_run(dyx3_geometry::Point a, dyx3_geometry::Point b, double step) {
  ConditionedRun r;
  r.profile = Profile::Segment;
  const double len = std::hypot(b.n - a.n, b.e - a.e);
  const int n = std::max(1, static_cast<int>(std::lround(len / step)));
  for (int i = 0; i <= n; ++i) {
    const double f = static_cast<double>(i) / n;
    r.pts.push_back({a.n + f * (b.n - a.n), a.e + f * (b.e - a.e)});
    r.flags.push_back(0);
    r.must_hit.push_back(0);
    r.cum_s.push_back(len * f);
  }
  r.length = len;
  return r;
}

// Three 2 m runs with 90 deg run boundaries: North, then East, then South.
std::vector<ConditionedRun> three_runs() {
  return {leg_run({0.0, 0.0}, {2.0, 0.0}, 0.5), leg_run({2.0, 0.0}, {2.0, 2.0}, 0.5),
          leg_run({2.0, 2.0}, {0.0, 2.0}, 0.5)};
}

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

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
// The pose is 3 cm short of the end, outside the 2 cm arrival band (inside it the dead band brakes
// at once, see TheEndpointDeadBand* below), and the velocity never settles below the threshold.
TEST(RppCore, APreciseStopPastItsTimeoutBrakesAndCompletes) {
  CoreRig r({north_run(6.0, Profile::Segment, 6.0)});
  r.n = 5.97;
  r.vn = 0.05;
  bool engaged = false;
  for (int i = 0; i < 300; ++i) {  // 6 s: the default timeout is 8 s
    const TickOutput& o = r.step();
    engaged = engaged || r.core->snapshot().endpoint_stop_active;
    EXPECT_NE(o.cmd, CmdKind::Brake) << "tick " << i;
    EXPECT_FALSE(o.pivot_timed_out) << "no pivot at the endpoint, tick " << i;
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

// ---- Endpoint precise stop: dead band, brake-hold hysteresis, feed-forward to the plane -------
// BEHAVIOUR CHANGE, not in the prototype (2026-10-10, mission 0001 run 3, recorder bag): the final
// endpoint rocked through the end plane, 14 forward/reverse command reversals in 8.6 s (+0.07 /
// -0.085 m/s at dist_to_goal 0.000-0.010 m, measured speed up to 0.125 m/s) until the 8 s timeout
// braked it 7 mm from the point. The creep law commanded sqrt(2*decel*|residual|) toward residual
// = 0 for ANY residual (0.084 m/s at 1 cm), so a vehicle with speed-loop latency overshot the
// plane, the sign flipped, and "stopped" was never satisfied. Inside the arrival band the finish
// geometry is met and only the stop confirmation is missing: the command is a brake. The tolerances
// are the defaults of segment_endpoint_arrival_tolerance_m / segment_endpoint_cross_tolerance_m
// (0.02) and endpoint_capture_past_m (0.02).

// One tick of the final-run precise stop with the pose `ahead` metres before the end plane on the
// line (negative: past it), moving forward at `speed`.
TEST(RppCore, TheEndpointDeadBandBrakesInsteadOfCreepingThroughThePlane) {
  struct Case {
    double ahead, speed;
  };
  // 1 cm short: the old law published a creep of sqrt(2 * 0.35 * 0.01) = 0.084 m/s forward; 3 mm
  // past the plane: a reverse creep of 0.046 m/s. Inside the band both are a brake now.
  for (const Case c : {Case{0.010, 0.05}, Case{-0.003, 0.05}, Case{0.0, 0.05}, Case{0.019, 0.08}}) {
    CoreRig r({north_run(6.0, Profile::Segment, 6.0)});
    r.n = 6.0 - c.ahead;
    r.vn = c.speed;  // moving: the stop is not confirmed
    const TickOutput& o = r.step();
    ASSERT_TRUE(r.core->snapshot().endpoint_stop_active) << "ahead " << c.ahead;
    EXPECT_EQ(o.cmd, CmdKind::Brake) << "ahead " << c.ahead;
    EXPECT_LE(o.brake_speed, 0.0) << "a brake never drives the moving rover forward";
    EXPECT_NEAR(o.brake_speed, -c.speed, 1e-9) << "the velocity-reversal brake of the stop machine";
    EXPECT_NEAR(o.v_e, 0.0, 1e-12);
    EXPECT_FALSE(r.core->path_done());
  }
}

TEST(RppCore, TheEndpointBrakeIsHeldThroughAFewMillimetresOfCoastThenYieldsToACorrection) {
  CoreRig r({north_run(6.0, Profile::Segment, 6.0)});
  r.n = 5.99;  // 1 cm short, moving: the dead band brakes and latches
  r.vn = 0.05;
  ASSERT_EQ(r.step().cmd, CmdKind::Brake);
  // the coast carries it 3 cm past the plane: outside the 2 cm arrival band but inside
  // along_tol + endpoint_capture_past_m = 4 cm, so the brake is NOT abandoned for a reverse creep
  r.n = 6.03;
  r.vn = 0.03;
  EXPECT_EQ(r.step().cmd, CmdKind::Brake);
  // the same pose without the latch (a fresh approach at that pose) is a correction
  CoreRig f({north_run(6.0, Profile::Segment, 6.0)});
  f.n = 6.03;
  f.vn = 0.03;
  const TickOutput& fo = f.step();
  EXPECT_EQ(fo.cmd, CmdKind::Creep);
  EXPECT_LT(fo.creep_speed, 0.0) << "past the plane: reverse";
  // a real overshoot (5 cm, beyond the hold band) releases the brake: reverse creep as before
  r.n = 6.05;
  r.vn = 0.03;
  const TickOutput& o = r.step();
  EXPECT_EQ(o.cmd, CmdKind::Creep);
  EXPECT_LT(o.creep_speed, 0.0);
  // and the latch is released: back inside the hold band but outside the arrival band, a creep
  r.n = 6.03;
  EXPECT_EQ(r.step().cmd, CmdKind::Creep);
}

TEST(RppCore, AnEndpointStoppedOffTheMarkStillGetsTheCreepNudge) {
  CoreRig r({north_run(6.0, Profile::Segment, 6.0)});
  r.n = 5.99;  // brake latched inside the band
  r.vn = 0.05;
  ASSERT_EQ(r.step().cmd, CmdKind::Brake);
  r.n = 6.03;  // the coast ended 3 cm past: stopped, outside the arrival band, inside the hold band
  r.vn = 0.0;
  CmdKind last = CmdKind::Brake;
  double creep = 0.0;
  for (int i = 0; i < 40; ++i) {  // 0.8 s: the 0.3 s dwell confirms the stop
    const TickOutput& o = r.step();
    last = o.cmd;
    creep = o.creep_speed;
    if (last == CmdKind::Creep) break;
  }
  EXPECT_EQ(last, CmdKind::Creep) << "a stopped rover off the mark must be nudged back";
  EXPECT_NEAR(creep, -0.1, 1e-9) << "segment_endpoint_creep_speed, toward the plane";
  EXPECT_FALSE(r.core->path_done());
}

TEST(RppCore, TheEndpointFeedForwardOutsideTheBandIsEvaluatedToThePlane) {
  // Approaching from 3 cm short (outside the band) at 0.2 m/s: the feed-forward speed is
  // sqrt(2 * decel * distance-to-the-plane) = 0.145 m/s (under the cap max(speed, creep)), exactly
  // the prototype law. The band-edge variant (sqrt(2 * decel * (distance - along_tol)) = 0.084
  // m/s) aims the stop short of the point; it was taken back (stop-position distribution below).
  CoreRig r({north_run(6.0, Profile::Segment, 6.0)});
  r.n = 6.0 - 0.03;
  r.vn = 0.2;
  const TickOutput& o = r.step();
  EXPECT_EQ(o.cmd, CmdKind::Creep);
  EXPECT_NEAR(o.creep_speed, std::sqrt(2.0 * 0.35 * 0.03), 1e-9);
  CoreRig e({north_run(6.0, Profile::Segment, 6.0)});
  e.n = 6.0 - 0.0201;  // just outside the band: still the plane law, not a vanishing creep
  e.vn = 0.2;
  EXPECT_NEAR(e.step().creep_speed, std::sqrt(2.0 * 0.35 * 0.0201), 1e-9);
}

// Closed loop on a first-order vehicle: the speed follows the commanded body speed with a time
// constant tau, optionally behind a pure command latency, the position integrates, 50 Hz ticks.
// DERIVED — NOT FROM V1 SPEC: test-only lags taken from the rover's measured speed-loop /
// drivetrain response in the mission 0001 bag (0.1-0.3 s; the heading lags the command by 0-0.4 s,
// irrelevant on a straight leg). The old law (min(sqrt(2*decel*|residual|), cap) toward residual =
// 0 with a sign flip at the plane, no brake in the band) rocks this model through the end plane
// and only the 8 s timeout ends it; the shipped law settles inside the arrival band.
namespace {
struct EndpointRun {
  bool completed{false};
  double t_complete{0.0};  // s from the start of the approach to the completion latch
  int sign_changes{0};     // forward/reverse reversals of the commanded speed (|v| >= 2 cm/s)
  double residual{0.0};    // signed distance to the end plane at rest (+ short, - past)
};
EndpointRun run_endpoint(double tau_s, int delay_ticks) {
  CoreRig r({north_run(6.0, Profile::Segment, 6.0)});
  r.n = 6.0 - 0.6;  // 0.6 m before the endpoint
  r.vn = 0.45;
  const double dt = 0.02;
  EndpointRun res;
  int last_sign = 0;
  std::vector<double> queue(static_cast<size_t>(delay_ticks), 0.45);
  size_t qi = 0;
  int rest_ticks = -1;  // after path_done: 1 s of STOP so the residual is the rest position
  for (int i = 0; i < 500 && rest_ticks != 0; ++i) {  // 10 s
    const TickOutput& o = r.step();
    if (!res.completed && (r.core->snapshot().completion_stop_pending || r.core->path_done())) {
      res.completed = true;
      res.t_complete = (i + 1) * dt;
    }
    if (r.core->path_done() && rest_ticks < 0) rest_ticks = 50;
    if (rest_ticks > 0) --rest_ticks;
    double cmd = o.v_n;  // heading 0: the commanded NED vector is the body speed
    if (delay_ticks > 0) {
      const double delayed = queue[qi];
      queue[qi] = cmd;
      qi = (qi + 1) % queue.size();
      cmd = delayed;
    }
    if (r.core->snapshot().endpoint_stop_active && std::fabs(o.v_n) >= 0.02) {
      const int s = o.v_n > 0.0 ? 1 : -1;
      if (last_sign != 0 && s != last_sign) ++res.sign_changes;
      last_sign = s;
    }
    r.vn += (cmd - r.vn) * (dt / tau_s);
    r.n += r.vn * dt;
  }
  res.residual = 6.0 - r.n;
  return res;
}
}  // namespace

// Stop-position distribution over the drivetrain lags (docs/contracts/rpp_stop_pivot_fsm.md
// section 3.6). Both variants keep the brake inside the arrival band and the brake-hold
// hysteresis; they differ only in where the feed-forward outside the band reaches zero. Measured
// with this harness (residual at rest, + short of the plane, - past it):
//
//   lag                      | feed-forward to the band EDGE   | feed-forward to the PLANE (ships)
//                            | residual  reversals  complete   | residual  reversals  complete
//   tau 0.1 s                |  +16.4 mm  1          3.66 s   |  +13.6 mm  1          3.64 s
//   tau 0.2 s                |   +6.8 mm  1          3.44 s   |   +1.0 mm  1          3.44 s
//   tau 0.3 s                |   -6.6 mm  1          3.42 s   |  -15.0 mm  1          3.44 s
//   tau 0.1 s + 0.1 s delay  |   +7.6 mm  2          3.56 s   |   -0.1 mm  2          3.58 s
//   tau 0.2 s + 0.1 s delay  |   -5.8 mm  1          3.28 s   |  -16.7 mm  1          3.32 s
//   tau 0.3 s + 0.1 s delay  |  -10.7 mm  2          4.06 s   |   -4.7 mm  2          4.32 s
//   mean residual            |   +1.3 mm                       |   -3.6 mm
//   max |residual|, rms      |   16.4 mm, 9.7 mm               |   16.7 mm, 10.9 mm
//
// In this model neither variant is biased short: the spread (about +/- 1.7 cm) comes from the
// in-band brake and the coast behind the lag, not from where the feed-forward reaches zero. The
// plane variant ships (review decision 2026-10-10: the prototype law, no stop short of the point
// by construction): it completes within 5 s with at most 2 reversals and |residual| <= along_tol
// for every lag, and its mean rest position is within 1 cm of the plane. Field re-validation of
// the stop position on the rover is still owed.
TEST(RppCore, TheEndpointStopPositionDistributionOnAFirstOrderVehicle) {
  struct Lag {
    double tau_s;
    int delay_ticks;
  };
  const Lag lags[] = {{0.1, 0}, {0.2, 0}, {0.3, 0}, {0.1, 5}, {0.2, 5}, {0.3, 5}};
  double sum = 0.0;
  int n = 0;
  for (const Lag& l : lags) {
    const EndpointRun a = run_endpoint(l.tau_s, l.delay_ticks);
    std::printf("endpoint tau %.1f s delay %.2f s: residual %+.1f mm, %d reversals, %s %.2f s\n",
                l.tau_s, l.delay_ticks * 0.02, a.residual * 1000.0, a.sign_changes,
                a.completed ? "complete" : "NOT complete", a.t_complete);
    EXPECT_TRUE(a.completed) << "tau " << l.tau_s << " delay " << l.delay_ticks;
    EXPECT_LE(a.t_complete, 5.0) << "tau " << l.tau_s << " delay " << l.delay_ticks;
    EXPECT_LE(a.sign_changes, 2) << "tau " << l.tau_s << " delay " << l.delay_ticks;
    EXPECT_LE(std::fabs(a.residual), 0.02) << "tau " << l.tau_s << " delay " << l.delay_ticks;
    sum += a.residual;
    ++n;
  }
  const double mean = sum / n;
  std::printf("endpoint mean residual %+.1f mm\n", mean * 1000.0);
  EXPECT_LE(std::fabs(mean), 0.01) << "mean rest position " << mean << " m from the plane";
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
  r.n = 5.97;  // 3 cm short: outside the 2 cm arrival band, where the dead band would brake at once
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

// RPP-002: a non-finite pose is not fed; the last good pose ages out into STALE (STOP).
TEST(RppCore, ANonFinitePoseIsNeverFedAndAgesOut) {
  CoreRig r({north_run(6.0, Profile::Segment, 6.0)});
  r.n = 1.0;
  EXPECT_NE(r.step().state, StateCode::Stale);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const NedPose bad : {NedPose{nan, 0.0, 0.0}, NedPose{1.0, nan, 0.0}, NedPose{1.0, 0.0, nan},
                            NedPose{std::numeric_limits<double>::infinity(), 0.0, 0.0}}) {
    CoreRig q({north_run(6.0, Profile::Segment, 6.0)});
    q.n = 1.0;
    q.step();
    q.feed_pose = false;
    StateCode last = StateCode::Idle;
    for (int i = 0; i < 30; ++i) {  // 0.6 s of non-finite poses: pose_max_age_s is 0.5 s
      q.now += kTickNs;
      q.core->on_pose(bad, q.now);
      q.core->on_velocity(0.0, 0.0, 0.0, q.now);
      const TickOutput& o = q.core->tick(q.now);
      EXPECT_TRUE(std::isfinite(o.v_n) && std::isfinite(o.v_e)) << i;
      last = o.state;
    }
    EXPECT_EQ(last, StateCode::Stale);
  }
}

// RPP-002 (CR-1): a NaN velocity with a fresh stamp made the body-axis brake +cap FORWARD. It is
// not fed: the velocity goes stale and the brake is zero.
TEST(RppCore, ANonFiniteVelocityNeverBecomesAForwardBrake) {
  CoreRig r({north_run(6.0, Profile::Segment, 6.0)},
            {{"segment_precise_endpoint_stop_enabled", 0.0, ""}});
  r.n = 6.0;  // at the final point: the completion hold brakes until a confirmed stop
  bool braked = false;
  r.vn = 0.0;
  r.step();
  r.feed_vel = false;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int i = 0; i < 40; ++i) {
    r.core->on_velocity(nan, 0.0, 0.0, r.now + kTickNs);
    const TickOutput& o = r.step();
    if (o.cmd == CmdKind::Brake) {
      braked = true;
      EXPECT_LE(o.brake_speed, 0.0) << "tick " << i;
    }
    EXPECT_TRUE(std::isfinite(o.v_n) && std::isfinite(o.v_e)) << i;
  }
  EXPECT_TRUE(braked) << "the completion hold was not reached";
}

// RPP-004: a pose stamped after the tick (negative age) is not fresh: STALE, no extrapolation.
TEST(RppCore, ANegativePoseAgeStops) {
  CoreRig r({north_run(6.0, Profile::Segment, 6.0)}, {{"use_imu_extrapolation", 1.0, ""}});
  r.n = 1.0;
  r.vn = 0.5;
  r.step();
  r.now += kTickNs;
  r.core->on_pose(NedPose{1.0, 0.0, 0.0}, r.now + 200'000'000);  // 0.2 s in the future
  r.core->on_velocity(0.5, 0.0, 0.0, r.now);
  const TickOutput& o = r.core->tick(r.now);
  EXPECT_EQ(o.state, StateCode::Stale);
  EXPECT_EQ(o.cmd, CmdKind::Stop);
}

// XR-RPP-009 / CLAUDE.md section 7: tick() allocates nothing, including through a hard corner
// (brake, stop confirmation, pivot, settle, advance: every CornerFsm transition is recorded) and
// the endpoint precise stop and completion.
// ---- RppStatus.pivot_timed_out (interfaces 0.17.0) --------------------------------------------
// Measured 2026-10-10: 25.8 s in PIVOT with an 86 deg heading error and every gate green, with no
// signal: the watchdog widened the band and nothing else happened. The core now exports it on
// every pivot tick past the budget while the heading is outside the release band; it keeps
// pivoting (the mission decides to pause). Budgets (defaults): max(segment_pivot_spinup_margin_s
// + angle / segment_nominal_pivot_rate_rad_s, segment_turn_timeout_s) = max(1.0 + (pi/2) / 0.4,
// 5.0) = 5.0 s for a 90 deg turn, counted from the first pivot tick.
TEST(RppCore, ACornerPivotThatNeverTurnsReportsTheTimeoutUntilReleased) {
  // One run, North 2 m then East 2 m: the rover stands still at the corner with its nose North.
  ConditionedRun r;
  r.profile = Profile::Segment;
  r.pts = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}};
  r.flags = {0, 0, 0};
  r.must_hit = {0, 0, 0};
  r.cum_s = {0.0, 2.0, 4.0};
  r.length = 4.0;
  CoreRig g({r});
  g.n = 2.0;
  double t_pivot = -1.0;
  bool fired = false;
  for (int i = 0; i < 400; ++i) {  // 8 s, the heading never changes
    const TickOutput& o = g.step();
    const double t = (i + 1) * 0.02;
    if (o.cmd == CmdKind::Pivot && t_pivot < 0.0) t_pivot = t;
    if (t_pivot < 0.0) {
      EXPECT_FALSE(o.pivot_timed_out) << "braking for the corner, t " << t;
      continue;
    }
    ASSERT_EQ(o.cmd, CmdKind::Pivot) << "t " << t;
    EXPECT_NEAR(o.pivot_heading_err, M_PI / 2, 1e-6);
    if (t - t_pivot < 5.0 - 1e-9) {
      EXPECT_FALSE(o.pivot_timed_out) << "inside the budget, t " << t;
    } else if (t - t_pivot > 5.0 + 0.02 + 1e-9) {
      EXPECT_TRUE(o.pivot_timed_out) << "past the budget, t " << t;
    }
    fired = fired || o.pivot_timed_out;
  }
  ASSERT_GT(t_pivot, 0.0);
  EXPECT_TRUE(fired);
  // The heading reaches the release band: settle brake, then the next leg. Not reported again.
  const TickOutput& o0 = g.step();
  ASSERT_EQ(o0.cmd, CmdKind::Pivot);
  g.yaw = wrap(g.yaw + o0.pivot_heading_err);
  bool tracked = false;
  for (int i = 0; i < 100 && !tracked; ++i) {
    const TickOutput& o = g.step();
    EXPECT_FALSE(o.pivot_timed_out) << "released, tick " << i;
    EXPECT_NE(o.cmd, CmdKind::Pivot) << "released, tick " << i;
    tracked = o.cmd == CmdKind::Track;
  }
  EXPECT_TRUE(tracked) << "the next leg never started";
}

// ---- MissionState.start_run_index (interfaces 0.17.0) -----------------------------------------
TEST(RppCore, AStartAtRunNBeginsWithTheEntryAlignmentOfRunN) {
  CoreRig g(three_runs());
  ASSERT_TRUE(g.core->start_at_run(2));
  // The state a hard run boundary leaves behind (apply_run(2, pre_stopped = true)): the entry
  // alignment is pending with the stop already confirmed. The entry pose is unknown, so the
  // watchdog budgets the worst case (pi), as run 0 does, not the 90 deg boundary turn.
  const CoreState st = g.core->snapshot();
  EXPECT_EQ(st.run_idx, 2);
  EXPECT_TRUE(st.run_align_pending);
  EXPECT_TRUE(st.corner_stop_complete);
  EXPECT_NEAR(st.run_align_turn_rad, M_PI, 1e-12);
  // The rover stands still at an arbitrary pose, nose North; run 2 heads South.
  g.n = 1.0;
  g.e = 1.0;
  const TickOutput& o = g.step();
  EXPECT_EQ(o.cmd, CmdKind::Pivot) << "the first command is the alignment pivot of run 2";
  EXPECT_GT(std::fabs(o.pivot_heading_err), M_PI / 4) << o.pivot_heading_err;
  EXPECT_EQ(g.core->run_index(), 2u);
  EXPECT_FALSE(o.pivot_timed_out);
  // A stationary rover that never turns: the alignment watchdog fires after its budget
  // (max(1.0 + pi / 0.4, 5.0) = 8.85 s, under the 9.0 s clamp) and is reported while the heading
  // is outside the band.
  bool fired = false;
  double t_fired = 0.0;
  for (int i = 1; i < 500; ++i) {
    const TickOutput& p = g.step();
    ASSERT_EQ(p.cmd, CmdKind::Pivot) << "tick " << i;
    EXPECT_EQ(g.core->run_index(), 2u) << "never a command for an earlier run";
    if (p.pivot_timed_out && !fired) t_fired = i * 0.02;
    fired = fired || p.pivot_timed_out;
    if (fired) EXPECT_TRUE(p.pivot_timed_out) << "tick " << i;
  }
  EXPECT_TRUE(fired);
  EXPECT_NEAR(t_fired, 1.0 + M_PI / 0.4, 0.03);
  // Aligned: released, then tracking run 2 (South). Never reported once released.
  const TickOutput& q = g.step();
  g.yaw = wrap(g.yaw + q.pivot_heading_err);
  bool tracked = false;
  for (int i = 0; i < 100 && !tracked; ++i) {
    const TickOutput& p = g.step();
    EXPECT_FALSE(p.pivot_timed_out) << "tick " << i;
    tracked = p.cmd == CmdKind::Track;
  }
  EXPECT_TRUE(tracked);
  EXPECT_EQ(g.core->run_index(), 2u);
}

TEST(RppCore, AStartAtRunNEqualsTheStateOfRunNReachedAtAHardBoundary) {
  // Reference: run 1 reached normally. The rover tracks run 0 to its end, the boundary hold stops
  // it (stationary), the core advances with pre_stopped.
  CoreRig a(three_runs());
  a.n = 2.0;  // at the end of run 0, standing still
  for (int i = 0; i < 100 && a.core->run_index() == 0; ++i) a.step();
  ASSERT_EQ(a.core->run_index(), 1u);
  CoreRig b(three_runs());
  ASSERT_TRUE(b.core->start_at_run(1));
  const CoreState sa = a.core->snapshot();
  const CoreState sb = b.core->snapshot();
  EXPECT_EQ(sb.run_idx, sa.run_idx);
  EXPECT_EQ(sb.segment_idx, sa.segment_idx);
  EXPECT_EQ(sb.segment_state, sa.segment_state);
  EXPECT_EQ(sb.run_align_pending, sa.run_align_pending);
  EXPECT_EQ(sb.corner_stop_complete, sa.corner_stop_complete);
  // the one difference: the resumed entry pose is unknown, the watchdog budgets pi
  EXPECT_NEAR(sa.run_align_turn_rad, M_PI / 2, 1e-12);
  EXPECT_NEAR(sb.run_align_turn_rad, M_PI, 1e-12);
  EXPECT_EQ(sb.run_boundary_stop_pending, sa.run_boundary_stop_pending);
  EXPECT_EQ(sb.completion_stop_pending, sa.completion_stop_pending);
  EXPECT_EQ(sb.endpoint_stop_active, sa.endpoint_stop_active);
  EXPECT_EQ(sb.path_done, sa.path_done);
  EXPECT_DOUBLE_EQ(sb.path_travel_m, sa.path_travel_m);
  EXPECT_EQ(sb.hint_seg, sa.hint_seg);
  EXPECT_EQ(sb.hint_valid, sa.hint_valid);
  // From the same pose both publish the same first command of run 1.
  a.n = b.n = 2.0;
  const TickOutput oa = a.step();
  const TickOutput ob = b.step();
  EXPECT_EQ(ob.cmd, CmdKind::Pivot);
  EXPECT_EQ(ob.cmd, oa.cmd);
  EXPECT_DOUBLE_EQ(ob.pivot_heading_err, oa.pivot_heading_err);
}

TEST(RppCore, AnInvalidStartRunIsRefusedAndChangesNothing) {
  CoreRig g(three_runs());
  EXPECT_FALSE(g.core->start_at_run(3));
  EXPECT_FALSE(g.core->start_at_run(1000));
  EXPECT_EQ(g.core->run_index(), 0u);
  EXPECT_TRUE(g.core->start_at_run(0));  // the plain start
  EXPECT_EQ(g.core->run_index(), 0u);
  EXPECT_FALSE(g.core->snapshot().corner_stop_complete)
      << "run 0 is a fresh start, not pre-stopped";
  RppCore empty(g.params);
  EXPECT_FALSE(empty.start_at_run(0)) << "no mission installed";
}

TEST(RppCore, TickNeverAllocates) {
  ConditionedRun run;
  run.profile = Profile::Segment;
  run.pts = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 3.0}};
  run.flags = {0, 0, 0};
  run.must_hit = {0, 0, 0};
  run.cum_s = {0.0, 3.0, 6.0};
  run.length = 6.0;
  CoreRig r({run}, {{"mission_speed", 0.4, ""}});
  r.feed_pose = r.feed_vel = false;
  long allocs = 0;
  bool pivoted = false;
  int i = 0;
  for (; i < 3000 && !r.core->path_done(); ++i) {
    r.now += kTickNs;
    r.core->on_pose(NedPose{r.n, r.e, r.yaw}, r.now);
    r.core->on_velocity(r.vn, r.ve, 0.0, r.now);
    g_allocs = 0;
    g_count_allocs = i >= 5;  // the first ticks may warm up lazily initialised statics
    const TickOutput& o = r.core->tick(r.now);
    g_count_allocs = false;
    allocs += g_allocs;
    // a kinematic vehicle: the ground velocity slews toward the core's vector at 1 m/s^2 (a
    // brake decelerates through zero instead of reversing at once); a pivot turns in place
    const double dt = 0.02;
    double tn = o.v_n, te = o.v_e;
    if (o.cmd == CmdKind::Pivot) {
      pivoted = true;
      r.yaw += std::max(-0.45, std::min(0.45, 1.5 * o.pivot_heading_err)) * dt;
      tn = te = 0.0;
    } else if (o.cmd == CmdKind::Track && std::hypot(o.v_n, o.v_e) > 1e-3) {
      r.yaw = std::atan2(o.v_e, o.v_n);
    }
    const double dn = tn - r.vn, de = te - r.ve, dv = std::hypot(dn, de), step = 1.0 * dt;
    const double k = dv > step ? step / dv : 1.0;
    r.vn += dn * k;
    r.ve += de * k;
    r.n += r.vn * dt;
    r.e += r.ve * dt;
  }
  EXPECT_TRUE(pivoted) << "the corner was not pivoted";
  EXPECT_TRUE(r.core->path_done()) << "not done after " << i << " ticks at " << r.n << ", " << r.e;
  EXPECT_EQ(allocs, 0) << "heap allocations inside tick()";
}

// Field-test speed defaults: mission_speed starts at 0.6 m/s, max_linear_vel (the physical cap,
// PX4 RO_SPEED_LIM) is 0.85, and the speed used is min(the two). mission_speed is LIVE, so the
// owner steps it up with `ros2 param set` mid-drive; the cap still holds however high it is set.
TEST(RppCore, MissionSpeedStartsAt06AndIsStillCappedByMaxLinearVel) {
  CoreRig r({north_run(150.0, Profile::Smooth, 0.08)});
  ASSERT_DOUBLE_EQ(r.params.num(P::mission_speed), 0.6) << "the default under test";
  ASSERT_DOUBLE_EQ(r.params.num(P::max_linear_vel), 0.85) << "the default under test";
  const double dt = 0.02;
  double peak = 0.0;
  const auto drive = [&](int ticks, int settle = 0) {  // peak is taken after `settle` ticks
    peak = 0.0;
    for (int i = 0; i < ticks && !r.core->path_done(); ++i) {
      const TickOutput& o = r.step();
      ASSERT_EQ(o.cmd, CmdKind::Track);
      const double v = std::hypot(o.v_n, o.v_e);
      if (i >= settle) peak = std::max(peak, v);
      r.vn = o.v_n;  // a kinematic vehicle that follows the command exactly
      r.ve = o.v_e;
      r.n += r.vn * dt;
      r.e += r.ve * dt;
    }
  };
  SetContext running;
  running.mission_running = true;
  running.source = "test";

  drive(1500);  // defaults: 0.6
  EXPECT_LE(peak, 0.6 + 1e-9);
  EXPECT_GE(peak, 0.55);

  ASSERT_TRUE(r.params.set({"mission_speed", 0.8, ""}, running).ok);  // LIVE, mid-mission
  drive(1500);
  EXPECT_LE(peak, 0.8 + 1e-9);
  EXPECT_GE(peak, 0.75);

  ASSERT_TRUE(r.params.set({"mission_speed", 5.0, ""}, running).ok);  // far above the cap
  drive(1500);
  EXPECT_LE(peak, 0.85 + 1e-9) << "max_linear_vel must cap mission_speed";
  EXPECT_GE(peak, 0.80);

  ASSERT_TRUE(
      r.params.set({"max_linear_vel", 0.7, ""}, running).ok);  // lowering the cap also bites
  drive(1500, 500);  // let the 0.85 -> 0.7 deceleration finish first
  EXPECT_LE(peak, 0.7 + 1e-9);
  EXPECT_GE(peak, 0.65);
}
