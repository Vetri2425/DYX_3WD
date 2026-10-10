#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "dyx3_px4_link/rover_setpoint_writer.hpp"

using namespace dyx3_px4_link;
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
GateInputs ok(double now) { return {now, true, true, 0U}; }
void expect_stop(const Setpoint& s) {
  EXPECT_EQ(s.mode, Mode::Stop);
  EXPECT_EQ(s.speed_body_x, 0.0F);
  EXPECT_TRUE(std::isnan(s.yaw_setpoint));
  EXPECT_EQ(s.yaw_rate_setpoint, 0.0F);
}
}  // namespace

// --- mapping table of docs/contracts/dyx3_px4_link.md section 3 ---------------------------------
TEST(Mapper, TrackHeading) {
  const auto s = to_setpoint(cmd(1, Mode::TrackHeading, 0.35F, 1.2F, NaN));
  EXPECT_EQ(s.mode, Mode::TrackHeading);
  EXPECT_FLOAT_EQ(s.speed_body_x, 0.35F);
  EXPECT_FLOAT_EQ(s.yaw_setpoint, 1.2F);
  EXPECT_TRUE(std::isnan(s.yaw_rate_setpoint));
}
TEST(Mapper, TrackRate) {
  const auto s = to_setpoint(cmd(1, Mode::TrackRate, -0.2F, NaN, 0.1F));
  EXPECT_FLOAT_EQ(s.speed_body_x, -0.2F);  // reverse is carried, never folded to |v|
  EXPECT_TRUE(std::isnan(s.yaw_setpoint));
  EXPECT_FLOAT_EQ(s.yaw_rate_setpoint, 0.1F);
}
TEST(Mapper, PivotAndCreep) {
  const auto p = to_setpoint(cmd(1, Mode::Pivot, 0.0F, NaN, 0.45F));
  EXPECT_EQ(p.speed_body_x, 0.0F);
  EXPECT_FLOAT_EQ(p.yaw_rate_setpoint, 0.45F);
  const auto c = to_setpoint(cmd(1, Mode::Creep, 0.05F, NaN, 0.0F));
  EXPECT_FLOAT_EQ(c.speed_body_x, 0.05F);
  EXPECT_TRUE(std::isnan(c.yaw_setpoint));
}
TEST(Mapper, StopIsCanonicalWhateverTheFieldsSay) {
  expect_stop(to_setpoint(cmd(1, Mode::Stop, 9.0F, 1.0F, 5.0F)));
}

// --- contract violations all become STOP
// ----------------------------------------------------------
TEST(Validate, Violations) {
  EXPECT_EQ(validate(cmd(1, Mode::TrackHeading, 0.3F, 1.0F, NaN, false)), Verdict::NotValid);
  Command bad = cmd(1, Mode::Stop, 0, NaN, 0);
  bad.mode = 9;
  EXPECT_EQ(validate(bad), Verdict::BadMode);
  EXPECT_EQ(validate(cmd(1, Mode::TrackHeading, 0.3F, NaN, NaN)), Verdict::NonFinite);
  EXPECT_EQ(validate(cmd(1, Mode::TrackHeading, NaN, 1.0F, NaN)), Verdict::NonFinite);
  EXPECT_EQ(validate(cmd(1, Mode::TrackHeading, 0.3F, 1.0F, 0.1F)), Verdict::ContractViolation);
  EXPECT_EQ(validate(cmd(1, Mode::TrackRate, 0.3F, 1.0F, 0.1F)), Verdict::ContractViolation);
  EXPECT_EQ(validate(cmd(1, Mode::TrackRate, 0.3F, NaN, NaN)), Verdict::NonFinite);
  EXPECT_EQ(validate(cmd(1, Mode::Pivot, 0.1F, NaN, 0.3F)), Verdict::ContractViolation);
  EXPECT_EQ(validate(cmd(1, Mode::Pivot, 0.0F, 1.0F, 0.3F)), Verdict::ContractViolation);
  const float inf = std::numeric_limits<float>::infinity();
  EXPECT_EQ(validate(cmd(1, Mode::TrackRate, inf, NaN, 0.1F)), Verdict::NonFinite);
  EXPECT_EQ(validate(cmd(1, Mode::Creep, 0.1F, NaN, -inf)), Verdict::NonFinite);
  for (const auto& c :
       {cmd(1, Mode::TrackHeading, 0.3F, NaN, NaN), cmd(1, Mode::Pivot, 0.1F, NaN, 0.3F)}) {
    expect_stop(to_setpoint(c));
  }
}

// --- gate: fail-to-zero cases
// ---------------------------------------------------------------------
TEST(Gate, NoCommandYet) {
  CommandGate g(0.2);
  const auto o = g.step(ok(1.0));
  EXPECT_EQ(o.reason, Reason::NoCommand);
  EXPECT_TRUE(o.failing_to_zero);
  expect_stop(o.sp);
}
TEST(Gate, FreshCommandForwarded) {
  CommandGate g(0.2);
  g.on_command(cmd(1, Mode::TrackHeading, 0.35F, 0.5F, NaN), 1.0);
  const auto o = g.step(ok(1.05));
  EXPECT_EQ(o.reason, Reason::None);
  EXPECT_FALSE(o.failing_to_zero);
  EXPECT_FLOAT_EQ(o.sp.speed_body_x, 0.35F);
}
TEST(Gate, StaleBoundary) {
  CommandGate g(0.2);
  g.on_command(cmd(1, Mode::TrackRate, 0.35F, NaN, 0.1F), 1.0);
  EXPECT_EQ(g.step(ok(1.2)).reason, Reason::None);  // age == max: still fresh
  const auto o = g.step(ok(1.2001));
  EXPECT_EQ(o.reason, Reason::CommandStale);
  expect_stop(o.sp);
  EXPECT_EQ(g.gap_events(), 1U);
  g.step(ok(1.3));  // still stale: not counted twice
  EXPECT_EQ(g.gap_events(), 1U);
}
TEST(Gate, RecoveryAfterGapNeedsNewSeq) {
  CommandGate g(0.2);
  g.on_command(cmd(1, Mode::TrackRate, 0.35F, NaN, 0.1F), 1.0);
  g.step(ok(2.0));                                               // stale
  g.on_command(cmd(1, Mode::TrackRate, 0.35F, NaN, 0.1F), 2.0);  // same seq re-sent: not fresh
  EXPECT_EQ(g.step(ok(2.01)).reason, Reason::CommandStale);
  g.on_command(cmd(2, Mode::TrackRate, 0.35F, NaN, 0.1F), 2.02);
  EXPECT_EQ(g.step(ok(2.03)).reason, Reason::None);
  g.step(ok(3.0));
  EXPECT_EQ(g.gap_events(), 2U);
}
TEST(Gate, DuplicateSeqNeverRefreshesFreshness) {
  CommandGate g(0.2);
  g.on_command(cmd(5, Mode::TrackRate, 0.35F, NaN, 0.1F), 1.0);
  for (int i = 1; i <= 30; ++i)
    g.on_command(cmd(5, Mode::TrackRate, 0.35F, NaN, 0.1F), 1.0 + 0.01 * i);
  EXPECT_EQ(g.step(ok(1.31)).reason, Reason::CommandStale);
}
TEST(Gate, InvalidCommand) {
  CommandGate g(0.2);
  g.on_command(cmd(1, Mode::TrackHeading, 0.35F, 0.5F, NaN, false), 1.0);
  const auto o = g.step(ok(1.01));
  EXPECT_EQ(o.reason, Reason::CommandInvalid);
  expect_stop(o.sp);
  g.on_command(cmd(2, Mode::Pivot, 0.2F, NaN, 0.3F), 1.02);  // contract break
  EXPECT_EQ(g.step(ok(1.03)).reason, Reason::CommandInvalid);
}
TEST(Gate, LinkConditionsOverrideAFreshCommand) {
  CommandGate g(0.2);
  g.on_command(cmd(1, Mode::TrackHeading, 0.35F, 0.5F, NaN), 1.0);
  GateInputs in = ok(1.01);
  in.handshake_ok = false;
  EXPECT_EQ(g.step(in).reason, Reason::HandshakeNotOk);
  in = ok(1.01);
  in.session_alive = false;
  EXPECT_EQ(g.step(in).reason, Reason::NoSession);
  in = ok(1.01);
  in.stale_topics_mask = 1U << 1;
  const auto o = g.step(in);
  EXPECT_EQ(o.reason, Reason::TopicStale);
  expect_stop(o.sp);
  EXPECT_EQ(g.step(ok(1.02)).reason, Reason::None);  // condition gone: forwarded again, no latch
}
TEST(Gate, SequenceResetDiscardsAndStopsOneTick) {
  CommandGate g(0.2);
  g.on_command(cmd(500, Mode::TrackHeading, 0.35F, 0.5F, NaN), 1.0);
  EXPECT_EQ(g.step(ok(1.01)).reason, Reason::None);
  g.on_command(cmd(0, Mode::TrackHeading, 0.35F, 0.5F, NaN), 1.02);  // restarted publisher
  const auto o = g.step(ok(1.03));
  EXPECT_EQ(o.reason, Reason::SequenceReset);
  expect_stop(o.sp);
  EXPECT_EQ(g.step(ok(1.04)).reason, Reason::NoCommand);  // old command is gone too
  g.on_command(cmd(1, Mode::TrackHeading, 0.35F, 0.5F, NaN), 1.05);
  EXPECT_EQ(g.step(ok(1.06)).reason, Reason::None);
}
TEST(Gate, GuardStopIsForwardedNotAFault) {
  CommandGate g(0.2);
  g.on_command(cmd(1, Mode::Stop, 0, NaN, 0), 1.0);
  const auto o = g.step(ok(1.01));
  EXPECT_EQ(o.reason, Reason::GuardStop);
  EXPECT_FALSE(o.failing_to_zero);
  expect_stop(o.sp);
}

// IF-003: the forwarded command names its seq and pose stamp; a link-made STOP names neither.
TEST(Gate, ForwardedOutputCarriesSeqAndPoseStamp) {
  CommandGate g(0.2);
  Command c = cmd(7, Mode::TrackRate, 0.35F, NaN, 0.1F);
  c.source_pose_sample_us = 1791590000120000ULL;
  g.on_command(c, 1.0);
  auto o = g.step(ok(1.01));
  EXPECT_TRUE(o.forwarded);
  EXPECT_EQ(o.seq, 7U);
  EXPECT_EQ(o.source_pose_sample_us, 1791590000120000ULL);
  o = g.step(ok(1.5));  // stale: the link's own STOP
  EXPECT_FALSE(o.forwarded);
  EXPECT_EQ(o.source_pose_sample_us, 0U);
  Command s = cmd(8, Mode::Stop, 0, NaN, 0);
  s.source_pose_sample_us = 5;
  g.on_command(s, 1.6);
  o = g.step(ok(1.61));
  EXPECT_EQ(o.reason, Reason::GuardStop);
  EXPECT_TRUE(o.forwarded);  // the guard's STOP is the guard's command
  EXPECT_EQ(o.source_pose_sample_us, 5U);
}

// Property: whatever the inputs, the output is either a contract-conforming forward of a fresh
// valid command or the canonical STOP. Never a finite non-zero speed with a failing reason.
TEST(Gate, RandomisedNeverLeaksMotionOnAFailure) {
  uint32_t s = 12345U;
  auto rnd = [&s]() {
    s = s * 1664525U + 1013904223U;
    return s;
  };
  CommandGate g(0.2);
  double now = 0.0;
  uint64_t seq = 0;
  for (int i = 0; i < 20000; ++i) {
    now += 0.005;
    if (rnd() % 3 == 0) {
      Command c;
      c.seq = (rnd() % 20 == 0) ? seq / 2 : ++seq;
      c.mode = static_cast<uint8_t>(rnd() % 7);
      c.speed_body_x = (rnd() % 5 == 0) ? NaN : static_cast<float>(rnd() % 100) / 100.0F - 0.3F;
      c.yaw_setpoint = (rnd() % 2) ? NaN : 1.0F;
      c.yaw_rate_setpoint = (rnd() % 2) ? NaN : 0.2F;
      c.valid = rnd() % 8 != 0;
      g.on_command(c, now);
    }
    GateInputs in{now, rnd() % 10 != 0, rnd() % 10 != 0, (rnd() % 10 == 0) ? 1U : 0U};
    const auto o = g.step(in);
    if (o.failing_to_zero) {
      ASSERT_EQ(o.sp.mode, Mode::Stop);
      ASSERT_EQ(o.sp.speed_body_x, 0.0F);
      ASSERT_EQ(o.sp.yaw_rate_setpoint, 0.0F);
      ASSERT_TRUE(std::isnan(o.sp.yaw_setpoint));
    } else {
      ASSERT_TRUE(std::isfinite(o.sp.speed_body_x));
      ASSERT_LE(o.command_age_s, 0.2 + 1e-9);
    }
  }
}
