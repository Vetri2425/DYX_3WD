#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "dyx3_motion_guard/estop_gate.hpp"
#include "dyx3_motion_guard/fail_to_zero.hpp"

using namespace dyx3_motion_guard;
constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

namespace {

Command cmd(uint64_t seq, Mode m, float v, float yaw, float rate, bool valid = true) {
  Command c;
  c.seq = seq;
  c.mode = static_cast<uint8_t>(m);
  c.speed_body_x = v;
  c.yaw_setpoint = yaw;
  c.yaw_rate_setpoint = rate;
  c.valid = valid;
  return c;
}

GateInputs good_gates() {
  GateInputs g;
  g.estop = false;
  g.link = {true, true, true, 0U};
  g.op = {true, true};
  g.vehicle = {true, 2, 14, false, true, true, true};
  g.rtk = {true, 6, true, 0.02F};
  g.est = {true, true, true, false, false, false, false, false};
  g.mission = {true, 3};
  return g;
}

DecisionConfig cfg() {
  DecisionConfig c;
  c.command_max_age_s = 0.2;
  c.limits.max_forward_speed_mps = 1.0F;
  c.limits.max_reverse_speed_mps = 0.3F;
  return c;
}

// Brings a core to "accepting" with a TRACK_RATE command at t = 0..: accept_count 3.
GuardCore accepting_core(const DecisionConfig& c = cfg()) {
  GuardCore g(3, c);
  for (uint64_t i = 1; i <= 3; ++i)
    g.on_command(cmd(i, Mode::TrackRate, 0.3F, NaN, 0.1F), 0.01 * static_cast<double>(i));
  return g;
}

void expect_stop(const Motion& m) {
  EXPECT_EQ(m.mode, Mode::Stop);
  EXPECT_EQ(m.speed_body_x, 0.0F);
  EXPECT_TRUE(std::isnan(m.yaw_setpoint));
  EXPECT_EQ(m.yaw_rate_setpoint, 0.0F);
}

}  // namespace

// ---- limiter
// -------------------------------------------------------------------------------------
TEST(StepToward, AccelDecelAndCrossing) {
  EXPECT_NEAR(step_toward(0.0, 1.0, 0.1, 0.2, 0.5), 0.02, 1e-12);  // from rest: accel
  EXPECT_NEAR(step_toward(0.3, 1.0, 0.1, 0.2, 0.5), 0.32, 1e-12);  // away from zero: accel
  EXPECT_NEAR(step_toward(0.3, 0.0, 0.1, 0.2, 0.5), 0.25, 1e-12);  // toward zero: decel
  EXPECT_NEAR(step_toward(0.3, 0.1, 0.1, 0.2, 0.5), 0.25, 1e-12);
  EXPECT_NEAR(step_toward(0.3, 0.29, 0.1, 0.2, 0.5), 0.29, 1e-12);    // reaches the target
  EXPECT_NEAR(step_toward(-0.3, -1.0, 0.1, 0.2, 0.5), -0.32, 1e-12);  // mirrored
  EXPECT_NEAR(step_toward(0.02, -0.5, 0.1, 0.2, 0.5), -0.01 * 0.0 - 0.2 * (0.1 - 0.04),
              1e-12);                                               // crosses zero
  EXPECT_NEAR(step_toward(0.3, -0.5, 0.1, 0.2, 0.5), 0.25, 1e-12);  // still decelerating to zero
}
TEST(Limits, Validation) {
  Limits l;
  EXPECT_TRUE(limits_valid(l));
  l.max_accel_mps2 = 0.0F;
  EXPECT_FALSE(limits_valid(l));
  l = Limits{};
  l.max_reverse_speed_mps = -0.1F;
  EXPECT_FALSE(limits_valid(l));
  l = Limits{};
  l.max_jerk_mps3 = -1.0F;
  EXPECT_FALSE(limits_valid(l));
  l = Limits{};
  l.max_forward_speed_mps = NaN;
  EXPECT_FALSE(limits_valid(l));
}
TEST(Limits, ClampsSpeedAndRate) {
  Limits l = cfg().limits;
  LimitState st;
  Motion m;
  m.mode = Mode::TrackRate;
  m.speed_body_x = 5.0F;
  m.yaw_setpoint = NaN;
  m.yaw_rate_setpoint = 2.0F;
  auto o = apply_limits(m, 0.02, l, st);
  EXPECT_TRUE(o.clamped);
  EXPECT_FLOAT_EQ(o.motion.speed_body_x, 1.0F);
  EXPECT_FLOAT_EQ(o.motion.yaw_rate_setpoint, 0.45F);
  m.speed_body_x = -5.0F;
  m.yaw_rate_setpoint = -2.0F;
  o = apply_limits(m, 0.02, l, st);
  EXPECT_FLOAT_EQ(o.motion.yaw_rate_setpoint, -0.45F);
  EXPECT_GE(o.motion.speed_body_x, -0.3F);
}
TEST(Limits, ReverseDefaultAllowsBoundedBrakeEnvelope) {
  Limits l;
  Motion m;
  m.mode = Mode::TrackRate;
  m.yaw_setpoint = NaN;
  m.yaw_rate_setpoint = 0.0F;

  // RPP's active brake is capped at 0.08 m/s: the 0.10 m/s hard envelope must not clip it.
  {
    LimitState st;
    m.speed_body_x = -0.08F;
    const auto o = apply_limits(m, 0.02, l, st);
    EXPECT_FALSE(o.clamped);
    EXPECT_FLOAT_EQ(o.motion.speed_body_x, -0.08F);
  }

  // A larger reverse request is bounded by the production hard envelope.
  {
    LimitState st;
    m.speed_body_x = -0.5F;
    const auto o = apply_limits(m, 0.02, l, st);
    EXPECT_TRUE(o.clamped);
    EXPECT_FLOAT_EQ(o.motion.speed_body_x, -0.10F);
  }
}

TEST(Limits, ProductionPathPassesRppProfileUnshapedInsideHardEnvelopes) {
  Limits l;
  LimitState st;
  Motion m;
  m.mode = Mode::TrackRate;
  m.yaw_setpoint = NaN;

  // A tiny dt would have heavily ramped both values in the old normal path.
  m.speed_body_x = 0.35F;
  m.yaw_rate_setpoint = 0.40F;
  auto o = apply_limits(m, 0.001, l, st);
  EXPECT_FALSE(o.clamped);
  EXPECT_FLOAT_EQ(o.motion.speed_body_x, 0.35F);
  EXPECT_FLOAT_EQ(o.motion.yaw_rate_setpoint, 0.40F);

  m.speed_body_x = -0.08F;
  m.yaw_rate_setpoint = -0.40F;
  o = apply_limits(m, 0.001, l, st);
  EXPECT_FALSE(o.clamped);
  EXPECT_FLOAT_EQ(o.motion.speed_body_x, -0.08F);
  EXPECT_FLOAT_EQ(o.motion.yaw_rate_setpoint, -0.40F);
}

TEST(Limits, HeadingModeKeepsNanRateAndIgnoresRateLimits) {
  Limits l = cfg().limits;
  l.profile_shaping_test_mode = true;
  l.max_yaw_accel_radps2 = 0.1F;
  LimitState st;
  Motion m;
  m.mode = Mode::TrackHeading;
  m.speed_body_x = 0.3F;
  m.yaw_setpoint = 1.0F;
  m.yaw_rate_setpoint = NaN;
  const auto o = apply_limits(m, 0.02, l, st);
  EXPECT_TRUE(std::isnan(o.motion.yaw_rate_setpoint));
  EXPECT_FLOAT_EQ(o.motion.yaw_setpoint, 1.0F);
}
TEST(Limits, AccelerationRampAndStopBypass) {
  Limits l = cfg().limits;
  l.profile_shaping_test_mode = true;
  l.max_accel_mps2 = 0.2F;
  l.max_decel_mps2 = 0.5F;
  LimitState st;
  Motion m;
  m.mode = Mode::TrackRate;
  m.speed_body_x = 0.35F;
  m.yaw_setpoint = NaN;
  m.yaw_rate_setpoint = 0.0F;
  double v = 0.0;
  for (int i = 0; i < 1000 && v < 0.35 - 1e-6; ++i) {
    const auto o = apply_limits(m, 0.02, l, st);
    EXPECT_LE(o.motion.speed_body_x - v, 0.2 * 0.02 + 1e-6);
    v = o.motion.speed_body_x;
  }
  EXPECT_NEAR(v, 0.35, 1e-6);
  m.mode = Mode::Stop;  // STOP is immediate, never ramped
  const auto s = apply_limits(m, 0.02, l, st);
  expect_stop(s.motion);
  EXPECT_EQ(st.last_speed, 0.0);
  Motion p;
  p.mode = Mode::Pivot;
  p.speed_body_x = 0.0F;
  p.yaw_setpoint = NaN;
  p.yaw_rate_setpoint = 0.4F;
  EXPECT_EQ(apply_limits(p, 0.02, l, st).motion.speed_body_x, 0.0F);
}
TEST(Limits, JerkBoundsAccelerationChange) {
  Limits l = cfg().limits;
  l.profile_shaping_test_mode = true;
  l.max_accel_mps2 = 1.0F;
  l.max_jerk_mps3 = 2.0F;
  LimitState st;
  Motion m;
  m.mode = Mode::TrackRate;
  m.speed_body_x = 0.5F;
  m.yaw_setpoint = NaN;
  m.yaw_rate_setpoint = 0.0F;
  double prev_a = 0.0;
  double v = 0.0;
  for (int i = 0; i < 300; ++i) {
    const auto o = apply_limits(m, 0.02, l, st);
    const double a = (o.motion.speed_body_x - v) / 0.02;
    EXPECT_LE(std::fabs(a - prev_a), 2.0 * 0.02 + 1e-5);
    prev_a = a;
    v = o.motion.speed_body_x;
  }
}
TEST(Limits, YawAccelLimit) {
  Limits l = cfg().limits;
  l.profile_shaping_test_mode = true;
  l.max_yaw_accel_radps2 = 1.0F;
  LimitState st;
  Motion m;
  m.mode = Mode::Pivot;
  m.speed_body_x = 0.0F;
  m.yaw_setpoint = NaN;
  m.yaw_rate_setpoint = 0.4F;
  double r = 0.0;
  for (int i = 0; i < 100; ++i) {
    const auto o = apply_limits(m, 0.02, l, st);
    EXPECT_LE(o.motion.yaw_rate_setpoint - r, 0.02 + 1e-6);
    r = o.motion.yaw_rate_setpoint;
  }
  EXPECT_NEAR(r, 0.4, 1e-5);
}

// ---- sequence
// ------------------------------------------------------------------------------------
TEST(Sequence, NewSessionNeedsAcceptCountAndDuplicatesNeverRefresh) {
  SequenceTracker s(3);
  EXPECT_TRUE(s.on_command(10, 0.0));
  EXPECT_FALSE(s.accepting());
  EXPECT_FALSE(s.on_command(10, 0.05));  // duplicate
  EXPECT_EQ(s.last_new_s(), 0.0);
  EXPECT_TRUE(s.on_command(11, 0.1));
  EXPECT_FALSE(s.accepting());
  EXPECT_TRUE(s.on_command(12, 0.2));
  EXPECT_TRUE(s.accepting());
  EXPECT_TRUE(s.on_command(3, 0.3));  // restarted publisher
  EXPECT_FALSE(s.accepting());
  EXPECT_EQ(s.count(), 1U);
  s.on_command(4, 0.32);
  s.on_command(5, 0.34);
  EXPECT_TRUE(s.accepting());
}

// ---- decision table
// ------------------------------------------------------------------------------
TEST(Decision, ForwardsWhenEverythingPasses) {
  auto g = accepting_core();
  const auto d = g.decide(0.05, 0.02, good_gates());
  EXPECT_EQ(d.reason, Reason::Ok);
  EXPECT_TRUE(d.accepted);
  EXPECT_EQ(d.out.mode, Mode::TrackRate);
  EXPECT_FLOAT_EQ(d.out.speed_body_x, 0.3F);
  EXPECT_EQ(d.input_seq, 3U);
}
TEST(Decision, NoCommandIsStale) {
  GuardCore g(3, cfg());
  const auto d = g.decide(1.0, 0.02, good_gates());
  EXPECT_EQ(d.reason, Reason::Stale);
  expect_stop(d.out);
}
TEST(Decision, SessionNotYetAcceptingIsSequence) {
  GuardCore g(3, cfg());
  g.on_command(cmd(1, Mode::TrackRate, 0.3F, NaN, 0.1F), 0.0);
  g.on_command(cmd(2, Mode::TrackRate, 0.3F, NaN, 0.1F), 0.01);
  const auto d = g.decide(0.02, 0.02, good_gates());
  EXPECT_EQ(d.reason, Reason::Sequence);
  expect_stop(d.out);
}
TEST(Decision, StaleBoundary) {
  auto g = accepting_core();                                         // last new command at t = 0.03
  EXPECT_EQ(g.decide(0.23, 0.02, good_gates()).reason, Reason::Ok);  // age 0.20: still fresh
  const auto d = g.decide(0.2301, 0.02, good_gates());
  EXPECT_EQ(d.reason, Reason::Stale);
  expect_stop(d.out);
}
TEST(Decision, DuplicateSequenceDoesNotKeepItAlive) {
  auto g = accepting_core();
  for (int i = 0; i < 50; ++i)
    g.on_command(cmd(3, Mode::TrackRate, 0.3F, NaN, 0.1F), 0.1 + 0.01 * i);
  EXPECT_EQ(g.decide(0.6, 0.02, good_gates()).reason, Reason::Stale);
}
TEST(Decision, InvalidMessages) {
  {
    GuardCore g(1, cfg());
    g.on_command(cmd(1, Mode::TrackHeading, 0.3F, 1.0F, NaN, false), 0.0);
    EXPECT_EQ(g.decide(0.01, 0.02, good_gates()).reason, Reason::InvalidMessage);
  }
  const Command bad[] = {
      cmd(1, Mode::TrackHeading, 0.3F, NaN, NaN), cmd(1, Mode::TrackHeading, 0.3F, 1.0F, 0.1F),
      cmd(1, Mode::TrackRate, 0.3F, 1.0F, 0.1F),  cmd(1, Mode::TrackRate, NaN, NaN, 0.1F),
      cmd(1, Mode::Pivot, 0.1F, NaN, 0.3F),       cmd(1, Mode::Creep, 0.1F, NaN, NaN)};
  for (const auto& c : bad) {
    GuardCore g(1, cfg());
    g.on_command(c, 0.0);
    const auto d = g.decide(0.01, 0.02, good_gates());
    EXPECT_EQ(d.reason, Reason::InvalidMessage);
    expect_stop(d.out);
  }
  GuardCore g(1, cfg());
  Command c = cmd(1, Mode::Stop, 0, NaN, 0);
  c.mode = 9;
  g.on_command(c, 0.0);
  EXPECT_EQ(g.decide(0.01, 0.02, good_gates()).reason, Reason::InvalidMessage);
}

struct GateCase {
  const char* name;
  void (*break_it)(GateInputs&);
  Reason expect;
};
const GateCase kGateCases[] = {
    {"estop", [](GateInputs& g) { g.estop = true; }, Reason::Estop},
    {"link not fresh", [](GateInputs& g) { g.link.fresh = false; }, Reason::Px4LinkUnhealthy},
    {"link session dead", [](GateInputs& g) { g.link.session_alive = false; },
     Reason::Px4LinkUnhealthy},
    {"link handshake", [](GateInputs& g) { g.link.handshake_ok = false; },
     Reason::Px4LinkUnhealthy},
    {"link stale topic", [](GateInputs& g) { g.link.stale_topics_mask = 2U; },
     Reason::Px4LinkUnhealthy},
    {"operator stale", [](GateInputs& g) { g.op.fresh = false; }, Reason::OperatorLinkLost},
    {"operator dead", [](GateInputs& g) { g.op.alive = false; }, Reason::OperatorLinkLost},
    {"vehicle stale", [](GateInputs& g) { g.vehicle.fresh = false; }, Reason::ArmingGate},
    {"disarmed", [](GateInputs& g) { g.vehicle.arming_state = 1; }, Reason::ArmingGate},
    {"not offboard", [](GateInputs& g) { g.vehicle.nav_state = 4; }, Reason::ArmingGate},
    {"px4 failsafe", [](GateInputs& g) { g.vehicle.failsafe = true; }, Reason::ArmingGate},
    {"rtk stale", [](GateInputs& g) { g.rtk.fresh = false; }, Reason::RtkGate},
    {"rtk float below min", [](GateInputs& g) { g.rtk.fix_type = 5; }, Reason::RtkGate},
    {"rtk 3d", [](GateInputs& g) { g.rtk.fix_type = 3; }, Reason::RtkGate},
    {"rtk static 7", [](GateInputs& g) { g.rtk.fix_type = 7; }, Reason::RtkGate},
    {"rtk corrections", [](GateInputs& g) { g.rtk.corrections_fresh = false; }, Reason::RtkGate},
    {"rtk accuracy unknown", [](GateInputs& g) { g.rtk.horizontal_accuracy_m = 0.0F; },
     Reason::RtkGate},
    {"rtk accuracy poor", [](GateInputs& g) { g.rtk.horizontal_accuracy_m = 0.11F; },
     Reason::RtkGate},
    {"est stale", [](GateInputs& g) { g.est.fresh = false; }, Reason::HeadingUnhealthy},
    {"est no flags", [](GateInputs& g) { g.est.flags_valid = false; }, Reason::HeadingUnhealthy},
    {"gnss yaw fault", [](GateInputs& g) { g.est.gnss_yaw_fault = true; },
     Reason::HeadingUnhealthy},
    {"yaw rejected", [](GateInputs& g) { g.est.reject_yaw = true; }, Reason::HeadingUnhealthy},
    {"gnss yaw not fused", [](GateInputs& g) { g.est.gnss_yaw_fusion_intended = false; },
     Reason::HeadingUnhealthy},
    {"attitude invalid", [](GateInputs& g) { g.vehicle.attitude_valid = false; },
     Reason::HeadingUnhealthy},
    {"position invalid", [](GateInputs& g) { g.vehicle.position_valid = false; },
     Reason::EstimatorUnhealthy},
    {"velocity invalid", [](GateInputs& g) { g.vehicle.velocity_valid = false; },
     Reason::EstimatorUnhealthy},
    {"dead reckoning", [](GateInputs& g) { g.est.inertial_dead_reckoning = true; },
     Reason::EstimatorUnhealthy},
    {"reject pos", [](GateInputs& g) { g.est.reject_hor_pos = true; }, Reason::EstimatorUnhealthy},
    {"reject vel", [](GateInputs& g) { g.est.reject_hor_vel = true; }, Reason::EstimatorUnhealthy},
    {"mission stale", [](GateInputs& g) { g.mission.fresh = false; }, Reason::MissionGate},
    {"mission paused", [](GateInputs& g) { g.mission.state = 4; }, Reason::MissionGate},
    {"mission ready", [](GateInputs& g) { g.mission.state = 2; }, Reason::MissionGate},
};

TEST(Decision, EveryGateZeroesTheOutputWithItsReason) {
  for (const auto& gc : kGateCases) {
    auto g = accepting_core();
    GateInputs gi = good_gates();
    gc.break_it(gi);
    const auto d = g.decide(0.05, 0.02, gi);
    EXPECT_EQ(d.reason, gc.expect) << gc.name;
    EXPECT_FALSE(d.accepted) << gc.name;
    expect_stop(d.out);
  }
}
TEST(Decision, GatePriorityOrder) {
  auto g = accepting_core();
  GateInputs gi = good_gates();
  gi.mission.state = 0;
  gi.est.reject_hor_pos = true;
  EXPECT_EQ(g.decide(0.05, 0.02, gi).reason, Reason::EstimatorUnhealthy);
  gi.est.reject_yaw = true;
  EXPECT_EQ(g.decide(0.05, 0.02, gi).reason, Reason::HeadingUnhealthy);
  gi.rtk.fix_type = 3;
  EXPECT_EQ(g.decide(0.05, 0.02, gi).reason, Reason::RtkGate);
  gi.vehicle.arming_state = 1;
  EXPECT_EQ(g.decide(0.05, 0.02, gi).reason, Reason::ArmingGate);
  gi.op.alive = false;
  EXPECT_EQ(g.decide(0.05, 0.02, gi).reason, Reason::OperatorLinkLost);
  gi.link.handshake_ok = false;
  EXPECT_EQ(g.decide(0.05, 0.02, gi).reason, Reason::Px4LinkUnhealthy);
  gi.estop = true;
  EXPECT_EQ(g.decide(0.05, 0.02, gi).reason, Reason::Estop);
}
TEST(Decision, NeverHeardGatesFail) {
  EXPECT_NE(first_failing_safety_gate(GateInputs{}, GateConfig{}), Reason::Ok);
  EXPECT_FALSE(mission_running(MissionIn{}));
}
TEST(Decision, CleanStopPassesWhateverTheGatesSay) {
  GuardCore g(3, cfg());
  for (uint64_t i = 1; i <= 3; ++i)
    g.on_command(cmd(i, Mode::Stop, 0.0F, NaN, 0.0F), 0.01 * static_cast<double>(i));
  GateInputs gi = good_gates();
  gi.estop = true;
  const auto d = g.decide(0.05, 0.02, gi);
  EXPECT_EQ(d.reason, Reason::Ok);
  EXPECT_TRUE(d.accepted);
  expect_stop(d.out);
}
TEST(Decision, RecoversWithoutLatchWhenGatesReturn) {
  auto g = accepting_core();
  GateInputs gi = good_gates();
  gi.rtk.fix_type = 3;
  EXPECT_EQ(g.decide(0.05, 0.02, gi).reason, Reason::RtkGate);
  g.on_command(cmd(4, Mode::TrackRate, 0.3F, NaN, 0.1F), 0.06);
  EXPECT_EQ(g.decide(0.07, 0.02, good_gates()).reason, Reason::Ok);
}
TEST(Decision, ClampIsReportedAndAccepted) {
  auto g = GuardCore(3, cfg());
  for (uint64_t i = 1; i <= 3; ++i)
    g.on_command(cmd(i, Mode::TrackRate, 5.0F, NaN, 0.1F), 0.01 * static_cast<double>(i));
  const auto d = g.decide(0.05, 0.02, good_gates());
  EXPECT_EQ(d.reason, Reason::LimitClamped);
  EXPECT_TRUE(d.accepted);
  EXPECT_TRUE(d.clamped);
  EXPECT_FLOAT_EQ(d.out.speed_body_x, 1.0F);
}
TEST(Decision, FailToZeroResetsTheTestModeShaper) {
  DecisionConfig c = cfg();
  c.limits.profile_shaping_test_mode = true;
  c.limits.max_accel_mps2 = 0.2F;
  GuardCore g(1, c);
  g.on_command(cmd(1, Mode::TrackRate, 0.35F, NaN, 0.1F), 0.0);
  double t = 0.0;
  float v = 0.0F;
  for (int i = 0; i < 60; ++i) {
    t += 0.02;
    g.on_command(cmd(2 + static_cast<uint64_t>(i), Mode::TrackRate, 0.35F, NaN, 0.1F), t);
    v = g.decide(t, 0.02, good_gates()).out.speed_body_x;
  }
  EXPECT_GT(v, 0.2F);
  GateInputs bad = good_gates();
  bad.rtk.fix_type = 3;
  t += 0.02;
  g.on_command(cmd(100, Mode::TrackRate, 0.35F, NaN, 0.1F), t);
  EXPECT_EQ(g.decide(t, 0.02, bad).out.speed_body_x, 0.0F);  // immediate, not ramped
  t += 0.02;
  g.on_command(cmd(101, Mode::TrackRate, 0.35F, NaN, 0.1F), t);
  EXPECT_LE(g.decide(t, 0.02, good_gates()).out.speed_body_x,
            0.2F * 0.02F + 1e-6F);  // restarts from zero
}

// Property: whatever the inputs, a non-STOP output only ever leaves while the command is valid,
// fresh, accepted, every safety gate passes and the mission is RUNNING; and STOP is always
// canonical.
TEST(Decision, RandomisedNoMotionLeaksThroughAFailure) {
  uint32_t s = 987654U;
  const auto rnd = [&s]() {
    s = s * 1664525U + 1013904223U;
    return s >> 8;
  };
  GuardCore g(3, cfg());
  double now = 0.0;
  uint64_t seq = 0;
  for (int i = 0; i < 50000; ++i) {
    now += 0.01;
    if (rnd() % 3 == 0) {
      Command c;
      c.seq = (rnd() % 25 == 0) ? seq / 3 : ++seq;
      c.mode = static_cast<uint8_t>(rnd() % 7);
      c.speed_body_x = (rnd() % 6 == 0) ? NaN : static_cast<float>(rnd() % 400) / 100.0F - 1.0F;
      c.yaw_setpoint = (rnd() % 2) ? NaN : 0.7F;
      c.yaw_rate_setpoint = (rnd() % 2) ? NaN : 0.3F;
      c.valid = rnd() % 9 != 0;
      g.on_command(c, now);
    }
    GateInputs gi = good_gates();
    bool all_good = true;
    for (const auto& gc : kGateCases) {
      if (rnd() % 40 == 0) {
        gc.break_it(gi);
        all_good = false;
      }
    }
    const auto d = g.decide(now, 0.01, gi);
    if (d.out.mode == Mode::Stop) {
      ASSERT_EQ(d.out.speed_body_x, 0.0F);
      ASSERT_EQ(d.out.yaw_rate_setpoint, 0.0F);
      ASSERT_TRUE(std::isnan(d.out.yaw_setpoint));
    } else {
      ASSERT_TRUE(all_good);
      ASSERT_TRUE(d.accepted);
      ASSERT_LE(d.input_age_s, 0.2 + 1e-9);
      ASSERT_TRUE(std::isfinite(d.out.speed_body_x));
      ASSERT_LE(d.out.speed_body_x, 1.0F + 1e-6F);
      ASSERT_GE(d.out.speed_body_x, -0.3F - 1e-6F);
    }
  }
}

// ---- e-stop latch -------------------------------------------------------------------------------
TEST(Estop, LatchAndSources) {
  EstopLatch e;
  EXPECT_FALSE(e.asserted());
  EXPECT_FALSE(e.request(true, "nobody"));
  EXPECT_FALSE(e.asserted());
  EXPECT_TRUE(e.request(true, "tablet"));
  EXPECT_TRUE(e.asserted());
  EXPECT_EQ(e.source(), "tablet");
  EXPECT_FALSE(e.request(false, ""));  // clearing needs a valid source too
  EXPECT_TRUE(e.asserted());
  EXPECT_TRUE(e.request(false, "backend"));
  EXPECT_FALSE(e.asserted());
  EXPECT_TRUE(e.source().empty());
}
