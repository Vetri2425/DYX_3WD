// Behaviour tests of RppCore beyond the prototype (the tick-by-tick equivalence with the carried
// node is orchestrator_equivalence_test). Each case pins a production fix that the prototype did
// not have, on plain inputs and an injected clock.
#include "dyx3_rpp/rpp_core.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
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

// ---- Endpoint precise stop: dead band, brake-hold hysteresis, band-edge feed-forward ----------
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

TEST(RppCore, TheEndpointFeedForwardReachesZeroAtTheEdgeOfTheArrivalBand) {
  // Approaching from 3 cm short (outside the band): the feed-forward speed is
  // sqrt(2 * decel * (distance - along_tol)) = 0.084 m/s, zero at 2 cm short instead of at the
  // plane (the old law: sqrt(2 * 0.35 * 0.03) = 0.145 m/s, held to the 0.1 m/s creep cap).
  CoreRig r({north_run(6.0, Profile::Segment, 6.0)});
  r.n = 6.0 - 0.03;
  r.vn = 0.05;
  const TickOutput& o = r.step();
  EXPECT_EQ(o.cmd, CmdKind::Creep);
  EXPECT_NEAR(o.creep_speed, std::sqrt(2.0 * 0.35 * (0.03 - 0.02)), 1e-9);
  CoreRig e({north_run(6.0, Profile::Segment, 6.0)});
  e.n = 6.0 - 0.0201;  // just outside the band: a vanishing creep, not 0.084 m/s
  e.vn = 0.05;
  EXPECT_LT(e.step().creep_speed, 0.02);
}

// Closed loop on a first-order vehicle: the speed follows the commanded body speed with a 0.2 s
// time constant, the position integrates, 50 Hz ticks. DERIVED — NOT FROM V1 SPEC: test-only lag
// taken from the rover's measured speed-loop/drivetrain response in the same bag (0.1-0.3 s; the
// heading lags the command by 0-0.4 s, irrelevant on a straight leg). The old law
// (min(sqrt(2*decel*|residual|), cap) toward residual = 0 with a sign flip at the plane) rocks
// this model through the end plane and only the 8 s timeout ends it; the new law settles inside
// the arrival band.
namespace {
struct EndpointRun {
  bool completed{false};
  double t_complete{0.0};
  int sign_changes{0};
  double residual{0.0};
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
  for (int i = 0; i < 400 && !res.completed; ++i) {  // 8 s
    const TickOutput& o = r.step();
    if (r.core->snapshot().completion_stop_pending || r.core->path_done()) {
      res.completed = true;
      res.t_complete = (i + 1) * dt;
      break;
    }
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

TEST(RppCore, TheEndpointSettlesWithoutRockingOnAFirstOrderVehicle) {
  const EndpointRun a = run_endpoint(0.2, 0);
  EXPECT_TRUE(a.completed) << "not complete in 8 s, residual " << a.residual;
  EXPECT_LE(a.t_complete, 4.0) << "tc " << a.t_complete;
  EXPECT_LE(a.sign_changes, 2) << "sc " << a.sign_changes;
  EXPECT_LE(std::fabs(a.residual), 0.02);
  // a harsher drivetrain: 0.3 s time constant plus 0.1 s of command latency (looser time bound)
  const EndpointRun b = run_endpoint(0.3, 5);
  EXPECT_TRUE(b.completed) << "not complete in 8 s, residual " << b.residual;
  EXPECT_LE(b.t_complete, 5.0) << "tc " << b.t_complete;
  EXPECT_LE(b.sign_changes, 2) << "sc " << b.sign_changes;
  EXPECT_LE(std::fabs(b.residual), 0.02);
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
