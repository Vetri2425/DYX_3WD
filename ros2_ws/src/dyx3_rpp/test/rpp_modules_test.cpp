// Definitional tests of the pure RPP modules (the equivalence against the prototype is
// gate4_equivalence_test).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "dyx3_rpp/guidance.hpp"
#include "dyx3_rpp/motion_output.hpp"
#include "dyx3_rpp/speed_profile.hpp"
#include "dyx3_rpp/stop_pivot_fsm.hpp"

using namespace dyx3_rpp;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr int64_t kMs = 1000000;

// ---- guidance
// --------------------------------------------------------------------------------------
TEST(Guidance, LookaheadDistanceBootstrapAndClamp) {
  LookaheadParams p{0.15, 0.35, 0.52, 1.0, 1.6, 0.05};
  const auto a = lookahead_distance(p, 0.0, 0.0);  // bootstrap: 0.5 * max_v
  const double v = 0.7 * std::max(0.15, 0.175) + 0.3 * 0.35;
  EXPECT_NEAR(a.raw, 1.6 * v, 1e-12);
  EXPECT_NEAR(a.l_d, std::max(0.52, std::min(1.0, 1.6 * v)), 1e-12);
  const auto far = lookahead_distance(
      p, 0.35, 20.0);  // large cross-track: saturates at l_max, raw stays visible
  EXPECT_DOUBLE_EQ(far.l_d, 1.0);
  EXPECT_GT(far.raw, 1.0);
}
TEST(Guidance, ArcCapReplacesTheLegacyFloor) {
  // Field anchor: kappa 0.43, e 0.005 => L <= 0.305 m.
  const double l = apply_arc_cap(1.0, 0.43, {0.005, 0.40, 0.20});
  EXPECT_NEAR(std::sqrt(8.0 * 0.005 / 0.43), 0.305, 1e-3);
  EXPECT_DOUBLE_EQ(l, 0.40);  // min_arc_ld floors the cap
  EXPECT_NEAR(apply_arc_cap(1.0, 0.1, {0.005, 0.10, 0.20}), std::sqrt(8.0 * 0.005 / 0.1), 1e-12);
  EXPECT_DOUBLE_EQ(apply_arc_cap(0.6, 0.5, {0.0, 0.4, 0.2}),
                   std::max(0.6, 0.2 / 0.5));                         // cap off: legacy floor
  EXPECT_DOUBLE_EQ(apply_arc_cap(0.6, 0.0, {0.005, 0.4, 0.2}), 0.6);  // straight: untouched
}
TEST(Guidance, SteeringGeometryBodyFrame) {
  const auto s = steering_geometry(1.0, 0.0, 0.0);  // aim due North, nose North
  EXPECT_NEAR(s.x_body, 1.0, 1e-12);
  EXPECT_NEAR(s.y_body, 0.0, 1e-12);
  EXPECT_NEAR(s.kappa, 0.0, 1e-12);
  const auto r =
      steering_geometry(0.0, 1.0, 0.0);  // aim due East, nose North: target is to the RIGHT
  EXPECT_NEAR(r.y_body, 1.0, 1e-12);
  EXPECT_GT(r.kappa, 0.0);
  EXPECT_NEAR(r.theta_e, M_PI / 2, 1e-12);
  EXPECT_TRUE(steering_geometry(0.0, 0.0, 1.0).degenerate);
}
TEST(Guidance, SegmentLookaheadRules) {
  // L-shape: (0,0)-(2,0)-(2,2); real corner at (2,0).
  const std::vector<Point> path = {{0, 0}, {2, 0}, {2, 2}};
  // a real corner is never crossed: pins to the vertex
  const auto pin = segment_lookahead_point(path, 0, {1.8, 0}, 1.0, 5.0, true, false);
  EXPECT_NEAR(pin.n, 2.0, 1e-12);
  EXPECT_NEAR(pin.e, 0.0, 1e-12);
  // with the corner extension the aim continues along the INCOMING tangent
  const auto ext = segment_lookahead_point(path, 0, {1.8, 0}, 1.0, 5.0, true, true);
  EXPECT_NEAR(ext.n, 2.8, 1e-12);
  EXPECT_NEAR(ext.e, 0.0, 1e-12);
  // the path end is not a corner: extend along the final bearing
  const auto end = segment_lookahead_point(path, 1, {2, 1.8}, 1.0, 5.0, true, false);
  EXPECT_NEAR(end.n, 2.0, 1e-12);
  EXPECT_NEAR(end.e, 2.8, 1e-12);
  const auto noext = segment_lookahead_point(path, 1, {2, 1.8}, 1.0, 5.0, false, false);
  EXPECT_NEAR(noext.e, 2.0, 1e-12);
  // a collinear vertex is crossed
  const std::vector<Point> col = {{0, 0}, {1, 0}, {2, 0}};
  EXPECT_NEAR(segment_lookahead_point(col, 0, {0.8, 0}, 0.9, 5.0).n, 1.7, 1e-12);
  EXPECT_NEAR(segment_lookahead_point(col, 0, {0.8, 0}, 0.9, 0.0).n, 1.0,
              1e-12);  // walk disabled: clips at the vertex
}
TEST(Guidance, SmoothLookaheadEndAndWalk) {
  const std::vector<Point> path = {{0, 0}, {1, 0}, {1, 1}};
  const auto mid = smooth_lookahead_point(path, 0, {0.5, 0}, 1.0);
  EXPECT_NEAR(mid.p.n, 1.0, 1e-12);
  EXPECT_NEAR(mid.p.e, 0.5, 1e-12);
  EXPECT_FALSE(mid.hit_end);
  const auto off = smooth_lookahead_point(path, 0, {0.5, 0}, 10.0);
  EXPECT_TRUE(off.hit_end);
  EXPECT_DOUBLE_EQ(off.p.e, 1.0);
}
TEST(Guidance, PivotInterceptFallbacks) {
  // On the leg: intercept bearing == leg heading.
  EXPECT_NEAR(pivot_intercept_heading({0.5, 0}, {0, 0}, {5, 0}, 0.0, true, 0.35), 0.0, 1e-12);
  // 10 cm off the leg: aims at a point 35 cm ahead of the projection, not along the leg
  const double h = pivot_intercept_heading({0.5, 0.1}, {0, 0}, {5, 0}, 0.0, true, 0.35);
  EXPECT_NEAR(h, std::atan2(-0.1, 0.35), 1e-12);
  EXPECT_DOUBLE_EQ(pivot_intercept_heading({0.5, 0.1}, {0, 0}, {5, 0}, 0.0, false, 0.35), 0.0);
  EXPECT_DOUBLE_EQ(pivot_intercept_heading({0.5, 0.1}, {0, 0}, {5, 0}, 0.0, true, 0.0), 0.0);
}

// ---- speed profile
// -----------------------------------------------------------------------------------
TEST(SpeedProfile, DerivedApproachDistanceIsAFloor) {
  EXPECT_DOUBLE_EQ(approach_distance(0.6, 0.35, 0.5), std::max(0.6, 0.35 * 0.35 / 1.0 + 0.10));
  EXPECT_GT(approach_distance(0.1, 1.0, 0.5), 1.0);  // braking distance wins at speed
}
TEST(SpeedProfile, LateralLimit) {
  const auto a = lateral_speed_limit(0.0, 0.04, 0.3, 0.35);
  EXPECT_DOUBLE_EQ(a.speed, 0.35);
  const auto b = lateral_speed_limit(0.43, 0.04, 0.3, 0.35);  // sqrt(0.04/0.43) = 0.305
  EXPECT_NEAR(b.speed, 0.305, 1e-3);
  const auto c = lateral_speed_limit(5.0, 0.04, 0.3, 0.35);  // never below the regulated minimum
  EXPECT_DOUBLE_EQ(c.speed, 0.3);
}
TEST(SpeedProfile, ClosedRunApproachScalesOnAlongLoopDistance) {
  // At the seam the Euclidean distance is ~0: the closed run must NOT crawl while remaining is
  // large.
  const auto r = smooth_approach_scaling(0.35, true, 9.4, 0.5, 0.0, 0.6, 0.1);
  EXPECT_FALSE(r.approach_active);
  EXPECT_DOUBLE_EQ(r.speed, 0.35);
  const auto near_end = smooth_approach_scaling(0.35, true, 9.4, 9.1, 0.0, 0.6, 0.1);
  EXPECT_TRUE(near_end.approach_active);
  EXPECT_NEAR(near_end.speed, std::max(0.1, 0.35 * (0.3 / 0.6)), 1e-12);
  // Open run: needs path_travel >= approach_d so a mission starting near its goal does not throttle
  // from tick 0.
  EXPECT_FALSE(smooth_approach_scaling(0.35, false, 5.0, 0.2, 0.3, 0.6, 0.1).approach_active);
  EXPECT_TRUE(smooth_approach_scaling(0.35, false, 5.0, 1.0, 0.3, 0.6, 0.1).approach_active);
}
TEST(SpeedProfile, P4FloorOnlyWhenTheIntendedTargetIsBelow) {
  int mode = 0;
  EXPECT_EQ(apply_p4_floor(0.01, 0.30, 0.2, 0.02, &mode),
            0.01);  // ramp-up through the dead band: not zeroed
  EXPECT_EQ(apply_p4_floor(0.01, 0.01, 0.2, 0.02, &mode), 0.0);
  EXPECT_EQ(mode, 3);
  EXPECT_EQ(apply_p4_floor(0.01, 0.01, 0.0, 0.02, &mode),
            0.01);  // already at zero command: nothing to snap
}

// ---- stop / pivot
// -----------------------------------------------------------------------------------
namespace {
StopTelemetry stopped() {
  StopTelemetry t;
  t.vel_fresh = true;
  return t;
}
StopTelemetry moving(double speed, double fwd) {
  StopTelemetry t;
  t.vel_fresh = true;
  t.speed = speed;
  t.v_forward = fwd;
  return t;
}
}  // namespace

TEST(StopPivot, BrakeIsExactlyAlongTheBodyAxisAndCapped) {  // I1
  StopPivotParams p;
  EXPECT_NEAR(brake_speed(moving(0.30, 0.30), p), -0.08,
              1e-12);  // forward motion: reverse brake, capped
  EXPECT_NEAR(brake_speed(moving(0.30, -0.30), p), 0.08, 1e-12);
  EXPECT_NEAR(brake_speed(moving(0.05, 0.05), p), -0.05, 1e-12);
  EXPECT_EQ(brake_speed(moving(0.01, 0.01), p), 0.0);  // already stopped
  EXPECT_EQ(brake_speed(moving(0.30, 0.10), p), 0.0);  // lateral-dominant: invent nothing
  StopTelemetry stale = moving(0.3, 0.3);
  stale.vel_fresh = false;
  EXPECT_EQ(brake_speed(stale, p), 0.0);
  p.brake_velocity_cap = 0.0;
  EXPECT_EQ(brake_speed(moving(0.3, 0.3), p), 0.0);
}
TEST(StopPivot, FreshVelocityAboveThresholdNeverTimesOutIntoAPivot) {  // I3
  StopPivotParams p;
  StopConfirm c;
  StopTelemetry t = moving(0.14, 0.14);  // the 0.14 m/s field bug
  for (int i = 0; i < 1000; ++i) EXPECT_FALSE(c.satisfied(i * 20 * kMs, t, p)) << i;  // 20 s of it
}
TEST(StopPivot, StaleVelocityTimesOutAfterTheCapOnly) {
  StopPivotParams p;
  StopConfirm c;
  StopTelemetry t = moving(0.0, 0.0);
  t.vel_fresh = false;
  EXPECT_FALSE(c.satisfied(0, t, p));
  EXPECT_FALSE(c.satisfied(1900 * kMs, t, p));
  EXPECT_TRUE(c.satisfied(2000 * kMs, t, p));
}
// XR-RPP-011: the stale cap counts from when the velocity went stale, not from the hold entry.
TEST(StopPivot, TheStaleCapCountsFromTheStalenessNotFromTheHoldEntry) {
  const StopPivotParams p;
  StopConfirm c;
  StopTelemetry moving_fresh;
  moving_fresh.vel_fresh = true;
  moving_fresh.speed = 0.3;  // braking, still above the threshold
  for (int t = 0; t <= 2500; t += 20) EXPECT_FALSE(c.satisfied(t * kMs, moving_fresh, p)) << t;
  StopTelemetry stale;
  stale.vel_fresh = false;
  EXPECT_FALSE(c.satisfied(2520 * kMs, stale, p)) << "one stale tick after 2.5 s of braking";
  EXPECT_FALSE(c.satisfied(4500 * kMs, stale, p)) << "1.98 s stale";
  EXPECT_TRUE(c.satisfied(4520 * kMs, stale, p)) << "2.0 s stale";
  // a fresh sample restarts the stale count
  StopConfirm d;
  EXPECT_FALSE(d.satisfied(0, stale, p));
  EXPECT_FALSE(d.satisfied(1900 * kMs, stale, p));
  EXPECT_FALSE(d.satisfied(1920 * kMs, moving_fresh, p));
  EXPECT_FALSE(d.satisfied(3000 * kMs, stale, p));
  EXPECT_TRUE(d.satisfied(5000 * kMs, stale, p));
}
TEST(StopPivot, DwellAndItsReset) {
  StopPivotParams p;  // dwell 0.30 s
  StopConfirm c;
  EXPECT_FALSE(c.satisfied(0, stopped(), p));
  EXPECT_FALSE(c.satisfied(200 * kMs, stopped(), p));
  StopTelemetry bump = moving(0.05, 0.05);  // any violation resets the dwell
  EXPECT_FALSE(c.satisfied(250 * kMs, bump, p));
  EXPECT_FALSE(c.satisfied(300 * kMs, stopped(), p));
  EXPECT_FALSE(c.satisfied(550 * kMs, stopped(), p));
  EXPECT_TRUE(c.satisfied(600 * kMs, stopped(), p));
  p.stop_dwell_s = 0.0;
  StopConfirm d;
  EXPECT_TRUE(d.satisfied(0, stopped(), p));
}
TEST(StopPivot, PivotBudgetIsAngleAwareFlooredAndClamped) {
  StopPivotParams p;
  PivotWatchdog w;
  EXPECT_FALSE(w.timed_out(0, M_PI, p));  // first call starts the clock and captures the angle
  EXPECT_NEAR(w.budget_s(p), std::min(9.0, std::max(1.0 + M_PI / 0.40, 5.0)), 1e-12);
  EXPECT_FALSE(w.timed_out(8000 * kMs, M_PI, p));
  EXPECT_TRUE(w.timed_out(9000 * kMs, M_PI, p));
  PivotWatchdog small;
  small.timed_out(0, 0.1, p);
  EXPECT_NEAR(small.budget_s(p), 5.0, 1e-12);  // never below the legacy floor
}

namespace {
CornerInput corner_in(int64_t ms, double err, double deg, StopTelemetry tel) {
  return {ms * kMs, err, deg, tel};
}
}  // namespace

TEST(CornerFsm, CollinearJunctionAdvancesWithoutStoppingAndKeepsMomentum) {
  CornerFsm f{StopPivotParams{}};
  const auto o = f.step(corner_in(0, 0.0, 10.0, moving(0.3, 0.3)));
  EXPECT_EQ(o.action, CornerAction::Advance);
  EXPECT_TRUE(o.collinear);
  EXPECT_FALSE(o.zero_speed_memory);
}
TEST(CornerFsm, HardCornerWalksBrakePivotSettleAdvance) {
  StopPivotParams p;
  CornerFsm f(p);
  const double deg = 90.0;
  const double err0 = M_PI / 2;
  int64_t t = 0;
  // BRAKE while moving
  auto o = f.step(corner_in(t, err0, deg, moving(0.3, 0.3)));
  EXPECT_EQ(o.action, CornerAction::Brake);
  EXPECT_NEAR(o.brake_speed, -0.08, 1e-12);
  EXPECT_TRUE(o.zero_speed_memory);
  // stopped for the dwell -> PIVOT
  for (t = 20; t < 400; t += 20) o = f.step(corner_in(t, err0, deg, stopped()));
  EXPECT_EQ(o.action, CornerAction::Pivot);
  EXPECT_EQ(o.state, FsmState::Pivot);
  EXPECT_TRUE(o.set_speed_memory);
  EXPECT_NEAR(o.heading_err, err0, 1e-12);
  // heading inside the release band -> settle brake, still no advance
  o = f.step(corner_in(t, 0.01, deg, stopped()));
  EXPECT_EQ(o.action, CornerAction::SettleBrake);
  EXPECT_EQ(o.state, FsmState::ReleaseSettle);
  // the release gates hold for align_settle_s -> ADVANCE across a real corner
  for (int k = 0; k < 20 && o.action != CornerAction::Advance; ++k) {
    t += 20;
    o = f.step(corner_in(t, 0.01, deg, stopped()));
  }
  EXPECT_EQ(o.action, CornerAction::Advance);
  EXPECT_TRUE(o.zero_speed_memory);
  EXPECT_FALSE(o.collinear);
  EXPECT_EQ(f.state(), FsmState::Tracking);  // reset for the next corner
  // every transition was logged with a reason
  ASSERT_GE(f.log().size(), 4U);
  for (const auto& tr : f.log()) EXPECT_FALSE(tr.reason.empty());
}
TEST(CornerFsm, NeverReleasesGrosslyMisheadedEvenAfterTheWatchdog) {
  StopPivotParams p;
  CornerFsm f(p);
  CornerOutput o;
  int64_t t = 0;
  for (; t < 400; t += 20) o = f.step(corner_in(t, 1.0, 90.0, stopped()));
  ASSERT_EQ(o.action, CornerAction::Pivot);
  // 60 s of a 1 rad error: the watchdog fires, the tolerance widens to 3 deg, never beyond
  // segment_pivot_release_max
  for (; t < 60000; t += 20) {
    o = f.step(corner_in(t, 0.2, 90.0, stopped()));
    ASSERT_NE(o.action, CornerAction::Advance);
  }
  EXPECT_TRUE(o.pivot_timed_out);
}
TEST(CornerFsm, StaleVelocityIsABoundedFallbackNotADeadlock) {
  StopPivotParams p;
  CornerFsm f(p);
  StopTelemetry stale;
  stale.vel_fresh = false;
  CornerOutput o;
  int64_t t = 0;
  for (; t <= 2000; t += 20) o = f.step(corner_in(t, 1.0, 90.0, stale));
  EXPECT_EQ(o.action, CornerAction::Pivot);  // the stale cap confirmed the stop
  // heading reaches the band with stale velocity: the measured gates are replaced by "timed out"
  for (int k = 0; k < 2000; ++k) {
    t += 20;
    o = f.step(corner_in(t, 0.01, 90.0, stale));
    if (o.action == CornerAction::Advance) break;
  }
  EXPECT_EQ(o.action, CornerAction::Advance);
}
TEST(CornerFsm, CarriedStopSkipsTheSecondStop) {  // run boundary: pre_stopped
  StopPivotParams p;
  CornerFsm f(p);
  f.carry_stop_complete();
  const auto o = f.step(corner_in(0, 1.0, 90.0, moving(0.0, 0.0)));
  EXPECT_EQ(o.action, CornerAction::Pivot);
}
TEST(StopHold, BrakesUntilAConfirmedStop) {  // completion hold, D3
  StopPivotParams p;
  StopHold h(p);
  auto o = h.step(0, moving(0.3, 0.3));
  EXPECT_EQ(o.phase, HoldPhase::Braking);
  EXPECT_LT(o.brake_speed, 0.0);
  EXPECT_TRUE(h.latched());
  o = h.step(100 * kMs, stopped());
  o = h.step(500 * kMs, stopped());
  EXPECT_EQ(o.phase, HoldPhase::Stopped);
  EXPECT_TRUE(o.stopped);
}

// ---- motion output
// ------------------------------------------------------------------------------------
TEST(MotionOutput, EveryBuilderSatisfiesTheMotionSetpointContract) {
  const MotionCommand cmds[] = {make_stop(),
                                make_track_heading(0.35, 1.2),
                                make_track_rate(0.35, 0.2, 0.45),
                                make_creep(0.03, 0.0, 0.45),
                                make_pivot(0.7, 0.45),
                                make_brake(-0.08, 2.0),
                                make_brake(0.0, 2.0)};
  for (auto c : cmds) EXPECT_TRUE(sanitize(&c)) << static_cast<int>(c.mode);
}
TEST(MotionOutput, PivotRateLawIsClampedAndSigned) {
  EXPECT_NEAR(make_pivot(0.1, 0.45).yaw_rate_setpoint, 0.15, 1e-6);
  EXPECT_NEAR(make_pivot(1.5, 0.45).yaw_rate_setpoint, 0.45, 1e-6);
  EXPECT_NEAR(make_pivot(-1.5, 0.45).yaw_rate_setpoint, -0.45, 1e-6);
  EXPECT_EQ(make_pivot(0.7, 0.45).speed_body_x, 0.0F);
}
TEST(MotionOutput, ReverseIsASignedSpeedWithTheNoseHeldNeverASpotTurn) {
  const auto b = make_brake(-0.08, 3.0);
  EXPECT_EQ(b.mode, MotionMode::TrackHeading);
  EXPECT_LT(b.speed_body_x, 0.0F);
  EXPECT_NEAR(b.yaw_setpoint, 3.0, 1e-6);
}
TEST(MotionOutput, SanitizeFailsToZero) {
  MotionCommand c = make_track_rate(0.3, 0.1, 0.45);
  c.speed_body_x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(sanitize(&c));
  EXPECT_EQ(c.mode, MotionMode::Stop);
  EXPECT_EQ(c.speed_body_x, 0.0F);
  MotionCommand p = make_pivot(0.3, 0.45);
  p.speed_body_x = 0.1F;  // PIVOT with speed breaks the contract
  EXPECT_FALSE(sanitize(&p));
  EXPECT_EQ(p.mode, MotionMode::Stop);
}
TEST(MotionOutput, HeadingIsWrapped) {
  EXPECT_NEAR(make_track_heading(0.3, 4.0).yaw_setpoint, 4.0 - 2.0 * M_PI, 1e-6);
}

// ---------------------------------------------------------------------------------------------------
// diagnostics and the tick -> MotionSetpoint mapping
#include "dyx3_rpp/diagnostics.hpp"
#include "dyx3_rpp/rpp_command.hpp"

TEST(LoopTimer, JitterIsActualMinusTargetAndAnOverrunIsAStall) {
  LoopTimer t(20000.0);  // 50 Hz
  t.note(1'000'000'000);
  EXPECT_EQ(t.overruns(), 0U);
  t.note(1'020'000'000);  // exactly on time
  EXPECT_NEAR(t.jitter_us(), 0.0, 1e-6);
  t.note(1'041'500'000);  // 1.5 ms late
  EXPECT_NEAR(t.jitter_us(), 1500.0, 1e-6);
  EXPECT_NEAR(t.max_abs_jitter_us(), 1500.0, 1e-6);
  EXPECT_EQ(t.overruns(), 0U);
  t.note(1'065'000'000);  // 23.5 ms: early-ish, still not an overrun (< 1.5 x)
  EXPECT_EQ(t.overruns(), 0U);
  t.note(1'125'000'000);  // 60 ms: a stall
  EXPECT_EQ(t.overruns(), 1U);
  EXPECT_NEAR(t.max_abs_jitter_us(), 40000.0, 1e-6);
  t.note(1'144'000'000);  // 19 ms: early by 1 ms
  EXPECT_NEAR(t.jitter_us(), -1000.0, 1e-6);
  EXPECT_NEAR(t.max_abs_jitter_us(), 40000.0, 1e-6);  // the maximum is of |jitter|
}

TEST(RppCommand, EveryKindMapsToAContractConformingCommand) {
  TickOutput o;
  // STOP
  EXPECT_EQ(command_from_tick(o, true, 0.45).mode, MotionMode::Stop);
  // TRACK: segment -> heading; smooth -> rate
  o.cmd = CmdKind::Track;
  o.v_n = 0.3 * std::cos(0.1);  // the core's vector and its heading agree above 1 cm/s
  o.v_e = 0.3 * std::sin(0.1);
  o.track_heading_ned = 0.1;
  o.yaw_rate = 0.2;
  MotionCommand c = command_from_tick(o, true, 0.45);
  EXPECT_EQ(c.mode, MotionMode::TrackHeading);
  EXPECT_FLOAT_EQ(c.speed_body_x, 0.3F);
  EXPECT_NEAR(c.yaw_setpoint, 0.1F, 1e-6);
  EXPECT_TRUE(std::isnan(c.yaw_rate_setpoint));
  // B3: the explicit segment rate selector uses the rate RppCore already computed from
  // segment_yaw_rate_gain * theta_e.
  c = command_from_tick(o, true, 0.45, true);
  EXPECT_EQ(c.mode, MotionMode::TrackRate);
  EXPECT_FLOAT_EQ(c.speed_body_x, 0.3F);
  EXPECT_TRUE(std::isnan(c.yaw_setpoint));
  EXPECT_FLOAT_EQ(c.yaw_rate_setpoint, 0.2F);
  c = command_from_tick(o, false, 0.45);
  EXPECT_EQ(c.mode, MotionMode::TrackRate);
  EXPECT_FLOAT_EQ(c.yaw_rate_setpoint, 0.2F);
  EXPECT_TRUE(std::isnan(c.yaw_setpoint));
  // XR-RPP-007: a slow but non-zero vector (below the core's 1 cm/s heading-memory threshold)
  // is commanded along its own bearing, not the stale frozen heading
  o.v_n = 0.004 * std::cos(1.5);
  o.v_e = 0.004 * std::sin(1.5);
  o.track_heading_ned = 0.0;  // the previous leg's heading, still in the core's memory
  c = command_from_tick(o, true, 0.45);
  EXPECT_EQ(c.mode, MotionMode::TrackHeading);
  EXPECT_NEAR(c.yaw_setpoint, 1.5, 1e-5);
  // zero speed keeps the frozen heading: no snap to North
  o.v_n = 0.0;
  o.v_e = 0.0;
  o.track_heading_ned = 1.2;
  c = command_from_tick(o, true, 0.45);
  EXPECT_EQ(c.mode, MotionMode::TrackHeading);
  EXPECT_FLOAT_EQ(c.yaw_setpoint, 1.2F);
  // the rate is clamped
  o.v_n = 0.3;
  o.yaw_rate = 3.0;
  EXPECT_FLOAT_EQ(command_from_tick(o, false, 0.45).yaw_rate_setpoint, 0.45F);
  // BRAKE: signed along the nose, nose heading held (reverse is a negative speed, never a spot
  // turn)
  o.cmd = CmdKind::Brake;
  o.brake_speed = -0.08;
  o.yaw_ned = 2.0;
  c = command_from_tick(o, true, 0.45);
  EXPECT_EQ(c.mode, MotionMode::TrackHeading);
  EXPECT_FLOAT_EQ(c.speed_body_x, -0.08F);
  EXPECT_FLOAT_EQ(c.yaw_setpoint, 2.0F);
  // PIVOT: speed 0, a finite clamped rate toward the exit heading
  o.cmd = CmdKind::Pivot;
  o.pivot_heading_err = -1.0;
  c = command_from_tick(o, true, 0.45);
  EXPECT_EQ(c.mode, MotionMode::Pivot);
  EXPECT_EQ(c.speed_body_x, 0.0F);
  EXPECT_FLOAT_EQ(c.yaw_rate_setpoint, -0.45F);
  // CREEP: signed, no turn (the vector lies on the nose axis, here behind it: reverse)
  o.cmd = CmdKind::Creep;
  o.creep_speed = -0.03;
  o.v_n = -0.03 * std::cos(2.0);
  o.v_e = -0.03 * std::sin(2.0);
  c = command_from_tick(o, true, 0.45);
  EXPECT_EQ(c.mode, MotionMode::Creep);
  EXPECT_FLOAT_EQ(c.speed_body_x, -0.03F);
  EXPECT_FLOAT_EQ(c.yaw_rate_setpoint, 0.0F);
  // a non-finite value anywhere fails to zero
  o.cmd = CmdKind::Track;
  o.v_n = std::nan("");
  c = command_from_tick(o, true, 0.45);
  EXPECT_EQ(c.mode, MotionMode::Stop);
}

// XR-RPP-001: the precise stop's diagonal correction must reach the vehicle. A CREEP vector off the
// nose becomes a heading command toward it (forward ahead of the beam, reverse behind it), with
// the same magnitude; a vector within kCreepSteerMinRad of the nose axis keeps the plain CREEP.
TEST(RppCommand, CreepOffTheNoseSteersTowardTheCorrectionVector) {
  TickOutput o;
  o.cmd = CmdKind::Creep;
  o.yaw_ned = 0.0;  // nose North
  // ahead and to the right (east): forward, heading toward the vector
  const double b = std::atan2(-0.03, 0.07);  // endpoint 7 cm ahead, rover 3 cm east of the line
  o.v_n = 0.1 * std::cos(b);
  o.v_e = 0.1 * std::sin(b);
  o.creep_speed = 0.1;
  MotionCommand c = command_from_tick(o, true, 0.45);
  EXPECT_EQ(c.mode, MotionMode::TrackHeading);
  EXPECT_FLOAT_EQ(c.speed_body_x, 0.1F);
  EXPECT_NEAR(c.yaw_setpoint, b, 1e-5);
  EXPECT_TRUE(std::isnan(c.yaw_rate_setpoint));
  // behind and to the left: reverse, the nose turned toward the opposite bearing (never a spot
  // turn to drive it forward)
  const double back = M_PI - 0.4;  // 157 deg: behind the beam, to the right
  o.v_n = 0.05 * std::cos(back);
  o.v_e = 0.05 * std::sin(back);
  o.creep_speed = -0.05;
  c = command_from_tick(o, true, 0.45);
  EXPECT_EQ(c.mode, MotionMode::TrackHeading);
  EXPECT_FLOAT_EQ(c.speed_body_x, -0.05F);
  EXPECT_NEAR(c.yaw_setpoint, std::atan2(-o.v_e, -o.v_n), 1e-5);
  EXPECT_NEAR(c.yaw_setpoint, -0.4, 1e-5);
  // nearly on the nose (1 deg): the plain CREEP mapping, unchanged
  o.v_n = 0.05 * std::cos(0.0175);
  o.v_e = 0.05 * std::sin(0.0175);
  o.creep_speed = 0.05;
  c = command_from_tick(o, true, 0.45);
  EXPECT_EQ(c.mode, MotionMode::Creep);
  EXPECT_FLOAT_EQ(c.speed_body_x, 0.05F);
  EXPECT_FLOAT_EQ(c.yaw_rate_setpoint, 0.0F);
  // nearly straight behind (179 deg): plain reverse CREEP
  o.v_n = -0.05 * std::cos(0.0175);
  o.v_e = -0.05 * std::sin(0.0175);
  o.creep_speed = -0.05;
  c = command_from_tick(o, true, 0.45);
  EXPECT_EQ(c.mode, MotionMode::Creep);
  EXPECT_FLOAT_EQ(c.speed_body_x, -0.05F);
  // the nose wraps: nose 179 deg, vector -170 deg is 11 deg off, forward
  o.yaw_ned = M_PI - 0.0175;
  o.v_n = 0.05 * std::cos(-M_PI + 0.1745);
  o.v_e = 0.05 * std::sin(-M_PI + 0.1745);
  o.creep_speed = 0.05;
  c = command_from_tick(o, true, 0.45);
  EXPECT_EQ(c.mode, MotionMode::TrackHeading);
  EXPECT_FLOAT_EQ(c.speed_body_x, 0.05F);
  EXPECT_NEAR(c.yaw_setpoint, -M_PI + 0.1745, 1e-5);
  // zero vector: plain CREEP (speed 0)
  o.v_n = 0.0;
  o.v_e = 0.0;
  o.creep_speed = 0.0;
  EXPECT_EQ(command_from_tick(o, true, 0.45).mode, MotionMode::Creep);
}
