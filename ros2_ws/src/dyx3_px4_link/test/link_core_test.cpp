#include <gtest/gtest.h>

#include <cmath>

#include "dyx3_px4_link/dds_session.hpp"
#include "dyx3_px4_link/msg_version_handshake.hpp"
#include "dyx3_px4_link/offboard_heartbeat.hpp"
#include "dyx3_px4_link/vehicle_state_assembler.hpp"

using namespace dyx3_px4_link;

namespace {
void feed_all(StalenessMonitor& m, double t) {
  for (int i = 0; i < kTopicCount; ++i) m.on_sample(static_cast<TopicBit>(i), t);
}
}  // namespace

// --- staleness -----------------------------------------------------------------------------------
TEST(Staleness, NeverSeenIsStale) {
  StalenessMonitor m{StalenessLimits{}};
  const auto r = m.evaluate(10.0);
  EXPECT_EQ(r.mask, 0x3FU);
  EXPECT_FALSE(r.session_alive);
  EXPECT_FALSE(r.any_seen);
}
TEST(Staleness, PerTopicBoundaryAndMaskBits) {
  StalenessMonitor m{StalenessLimits{}};
  feed_all(m, 1.0);
  EXPECT_EQ(m.evaluate(1.2).mask, 0U);  // local position limit is 0.2: at the limit, fresh
  const auto r = m.evaluate(1.25);
  EXPECT_EQ(r.mask, (1U << kLocalPosition) | (1U << kAttitude));
  EXPECT_TRUE(r.session_alive);  // timesync still fresh: the session is up while topics died
  EXPECT_NEAR(r.worst_age_s, 0.25, 1e-9);
}
// Rover 2026-10-08 (fw 9ab2ad3162): timesync_status and estimator_status_flags arrive every
// 1.010 s. With the old 1.0 s limits the session was declared dead and re-established every
// second (handshake re-armed, link failing to zero). The defaults must ride through that cadence.
TEST(Staleness, MeasuredOneHzCadenceKeepsSessionAlive) {
  StalenessMonitor m{StalenessLimits{}};
  constexpr double kSlowPeriod = 1.010;
  double next_slow = 0.0;
  for (int k = 0; k <= 2000; ++k) {  // 20 s at 100 Hz
    const double t = k * 0.01;
    if (t >= next_slow) {
      m.on_sample(kTimesync, t);
      m.on_sample(kEstimatorFlags, t);
      next_slow += kSlowPeriod;
    }
    m.on_sample(kLocalPosition, t);
    m.on_sample(kAttitude, t);
    if (k % 50 == 0) m.on_sample(kVehicleStatus, t);
    if (k % 20 == 0) m.on_sample(kGps, t);
    const auto r = m.evaluate(t);
    ASSERT_TRUE(r.session_alive) << "t=" << t;
    ASSERT_EQ(r.mask, 0U) << "t=" << t;
  }
  EXPECT_EQ(m.session_resets(), 0U);
}

TEST(Staleness, SilentTopicWhileSessionUpIsDetected) {  // upstream #27388
  StalenessMonitor m{StalenessLimits{}};
  feed_all(m, 0.0);
  for (int i = 1; i <= 20; ++i) {
    const double t = 0.1 * i;
    m.on_sample(kTimesync, t);
    m.on_sample(kVehicleStatus, t);
    m.on_sample(kEstimatorFlags, t);
    m.on_sample(kGps, t);
    m.on_sample(kAttitude, t);  // local position silently stops
  }
  const auto r = m.evaluate(2.0);
  EXPECT_TRUE(r.session_alive);
  EXPECT_EQ(r.mask, 1U << kLocalPosition);
}
TEST(Staleness, ImmediateReadinessExpiresBetweenStatusCycles) {
  StalenessMonitor m{StalenessLimits{}};
  EXPECT_FALSE(m.all_fresh(0.0));
  feed_all(m, 0.0);
  EXPECT_TRUE(m.all_fresh(0.1));
  EXPECT_FALSE(m.all_fresh(0.25));  // local position and attitude exceeded 0.2 s
}
TEST(Staleness, SessionResetIsCountedOnceAndConsumedOnce) {
  StalenessMonitor m{StalenessLimits{}};
  feed_all(m, 0.0);
  EXPECT_TRUE(m.evaluate(0.1).session_alive);
  EXPECT_FALSE(m.consume_reset());
  EXPECT_FALSE(
      m.evaluate(5.0).session_alive);  // everything silent past the 3.0 s limit: agent died
  EXPECT_EQ(m.session_resets(), 0U);
  feed_all(m, 5.5);
  EXPECT_TRUE(m.evaluate(5.5).session_alive);
  EXPECT_EQ(m.session_resets(), 1U);
  EXPECT_TRUE(m.consume_reset());
  EXPECT_FALSE(m.consume_reset());
  m.evaluate(5.51);
  EXPECT_EQ(m.session_resets(), 1U);
}

// --- handshake
// ------------------------------------------------------------------------------------
namespace {
Handshake make_hs() {
  return Handshake(
      {{"/fmu/in/vehicle_command", 111, false}, {"/fmu/out/vehicle_status", 222, false}}, 1.0);
}
}  // namespace

TEST(Handshake, PendingUntilAllMatch) {
  auto h = make_hs();
  EXPECT_EQ(h.state(), HandshakeState::Pending);
  h.on_response("/fmu/in/vehicle_command", true, 111);
  EXPECT_EQ(h.state(), HandshakeState::Pending);
  EXPECT_EQ(h.pending_count(), 1U);
  h.on_response("/fmu/out/vehicle_status", true, 222);
  EXPECT_EQ(h.state(), HandshakeState::Ok);
}
TEST(Handshake, RequestsAreRetriedOnlyForPendingTopics) {
  auto h = make_hs();
  EXPECT_EQ(h.due_requests(0.0).size(), 2U);
  EXPECT_TRUE(h.due_requests(0.5).empty());
  h.on_response("/fmu/in/vehicle_command", true, 111);
  const auto d = h.due_requests(1.0);
  ASSERT_EQ(d.size(), 1U);
  EXPECT_EQ(d[0], 1U);
}
TEST(Handshake, HashMismatchIsLatchedAndLoud) {
  auto h = make_hs();
  h.on_response("/fmu/in/vehicle_command", true, 999);
  EXPECT_EQ(h.state(), HandshakeState::Mismatch);
  EXPECT_NE(h.first_mismatch_reason().find("mismatch"), std::string::npos);
  h.on_response("/fmu/in/vehicle_command", true, 111);  // a later matching reply does not clear it
  EXPECT_EQ(h.state(), HandshakeState::Mismatch);
  EXPECT_TRUE(h.due_requests(100.0).size() <= 1U);  // never re-asks the mismatched one
}
TEST(Handshake, FirmwareDoesNotKnowTheTopic) {
  auto h = make_hs();
  h.on_response("/fmu/out/vehicle_status", false, 0);
  EXPECT_EQ(h.state(), HandshakeState::Mismatch);
}
TEST(Handshake, MissingLocalDefinitionIsAMismatchNotASkip) {
  Handshake h({{"/fmu/in/foo", 0, true}}, 1.0);
  EXPECT_EQ(h.state(), HandshakeState::Mismatch);
}
TEST(Handshake, UnknownResponseIgnoredAndRearmClears) {
  auto h = make_hs();
  h.on_response("/fmu/out/other", true, 5);
  EXPECT_EQ(h.state(), HandshakeState::Pending);
  h.on_response("/fmu/in/vehicle_command", true, 999);
  ASSERT_EQ(h.state(), HandshakeState::Mismatch);
  h.rearm();  // session reset: possibly new firmware
  EXPECT_EQ(h.state(), HandshakeState::Pending);
  EXPECT_EQ(h.due_requests(0.0).size(), 2U);
  h.on_response("/fmu/in/vehicle_command", true, 111);
  h.on_response("/fmu/out/vehicle_status", true, 222);
  EXPECT_EQ(h.state(), HandshakeState::Ok);
  h.rearm();  // an OK handshake is also re-proven after a reset
  EXPECT_EQ(h.state(), HandshakeState::Pending);
}
TEST(Handshake, EmptyTopicSetIsNeverOk) {
  Handshake h({}, 1.0);
  EXPECT_EQ(h.state(), HandshakeState::Pending);
}

// --- offboard
// -------------------------------------------------------------------------------------
TEST(Offboard, HeartbeatBeforeModeCommandAndConfirmation) {
  OffboardSession s{OffboardTiming{}};
  EXPECT_FALSE(s.step(0.0, true, false).publish_heartbeat);  // disabled
  s.enable(true, 0.0);
  auto o = s.step(0.0, true, false);
  EXPECT_TRUE(o.publish_heartbeat);
  EXPECT_FALSE(o.send_mode_command);
  o = s.step(0.4, true, false);
  EXPECT_FALSE(o.send_mode_command);  // still pre-streaming
  o = s.step(0.5, true, false);
  EXPECT_TRUE(o.send_mode_command);
  EXPECT_EQ(o.state, OffboardState::Requested);
  EXPECT_FALSE(s.step(0.51, true, false).send_mode_command);  // exactly once
  EXPECT_EQ(s.step(0.6, true, true).state, OffboardState::Active);
}
TEST(Offboard, ConfirmTimeoutFailsWithoutRetry) {
  OffboardSession s{OffboardTiming{}};
  s.enable(true, 0.0);
  s.step(0.5, true, false);
  EXPECT_EQ(s.step(2.6, true, false).state, OffboardState::Failed);
  const auto o = s.step(10.0, true, false);
  EXPECT_EQ(o.state, OffboardState::Failed);
  EXPECT_FALSE(o.send_mode_command);
  EXPECT_TRUE(o.publish_heartbeat);
}
TEST(Offboard, RefusalFailsOnlyARequestedSessionAndKeepsStop) {
  OffboardSession s{OffboardTiming{}};
  EXPECT_FALSE(s.fail_requested());  // disabled
  s.enable(true, 0.0);
  EXPECT_FALSE(s.fail_requested());  // prestream: no mode command was sent yet
  EXPECT_EQ(s.state(), OffboardState::Prestream);
  s.step(0.5, true, false);  // mode command sent
  ASSERT_EQ(s.state(), OffboardState::Requested);
  EXPECT_TRUE(s.fail_requested());
  auto o = s.step(0.51, true, false);
  EXPECT_EQ(o.state, OffboardState::Failed);
  EXPECT_TRUE(o.publish_heartbeat);
  EXPECT_TRUE(o.stop_only);
  EXPECT_FALSE(o.send_mode_command);
  EXPECT_FALSE(s.fail_requested());  // already failed
  // PX4 reporting OFFBOARD later does not revive a failed session.
  EXPECT_EQ(s.step(0.6, true, true).state, OffboardState::Failed);
  OffboardSession a{OffboardTiming{}};
  a.enable(true, 0.0);
  a.step(0.5, true, false);
  a.step(0.6, true, true);
  ASSERT_EQ(a.state(), OffboardState::Active);
  EXPECT_FALSE(a.fail_requested());  // a late refusal cannot stop an established session
  EXPECT_EQ(a.state(), OffboardState::Active);
}
TEST(Offboard, LeavingOffboardIsLostAndNeverReRequested) {
  OffboardSession s{OffboardTiming{}};
  s.enable(true, 0.0);
  s.step(0.5, true, false);
  s.step(0.6, true, true);
  const auto o = s.step(0.7, true, false);
  EXPECT_EQ(o.state, OffboardState::Lost);
  EXPECT_FALSE(s.step(5.0, true, false).send_mode_command);
}
TEST(Offboard, LinkLossWithdrawsHeartbeatAndRestartsPrestream) {
  OffboardSession s{OffboardTiming{}};
  s.enable(true, 0.0);
  s.step(0.5, true, false);  // Requested
  auto o = s.step(0.6, false, false);
  EXPECT_FALSE(o.publish_heartbeat);
  EXPECT_EQ(o.state, OffboardState::Prestream);
  o = s.step(0.7, true, false);
  EXPECT_TRUE(o.publish_heartbeat);
  EXPECT_FALSE(o.send_mode_command);  // must pre-stream again first
  EXPECT_TRUE(s.step(1.2, true, false).send_mode_command);
}
TEST(Offboard, OnlyActiveForwardsTheCommandEveryOtherStateStreamsStop) {  // XR-GPX-007
  OffboardSession s{OffboardTiming{}};
  s.enable(true, 0.0);
  auto o = s.step(0.0, true, false);
  EXPECT_EQ(o.state, OffboardState::Prestream);
  EXPECT_TRUE(o.stop_only);
  o = s.step(0.5, true, false);
  EXPECT_EQ(o.state, OffboardState::Requested);
  EXPECT_TRUE(o.stop_only);
  o = s.step(0.6, true, true);
  EXPECT_EQ(o.state, OffboardState::Active);
  EXPECT_FALSE(o.stop_only);
  o = s.step(0.7, true, false);
  EXPECT_EQ(o.state, OffboardState::Lost);
  EXPECT_TRUE(o.publish_heartbeat);
  EXPECT_TRUE(o.stop_only);
  OffboardSession f{OffboardTiming{}};
  f.enable(true, 0.0);
  f.step(0.5, true, false);
  o = f.step(2.6, true, false);
  EXPECT_EQ(o.state, OffboardState::Failed);
  EXPECT_TRUE(o.stop_only);
}
TEST(Offboard, LinkLossFromActiveIsLostAndNeverReRequested) {  // XR-GPX-007
  OffboardSession s{OffboardTiming{}};
  s.enable(true, 0.0);
  s.step(0.5, true, false);
  ASSERT_EQ(s.step(0.6, true, true).state, OffboardState::Active);
  auto o = s.step(0.7, false, true);
  EXPECT_EQ(o.state, OffboardState::Lost);
  EXPECT_FALSE(o.publish_heartbeat);
  for (double t = 0.8; t < 5.0; t += 0.1) {  // the link returns, PX4 may still report OFFBOARD
    o = s.step(t, true, t > 2.0);
    EXPECT_EQ(o.state, OffboardState::Lost);
    EXPECT_FALSE(o.send_mode_command);
    EXPECT_TRUE(o.publish_heartbeat);
    EXPECT_TRUE(o.stop_only);
  }
  s.enable(true, 5.0);  // only a new operator request starts a new session
  EXPECT_EQ(s.step(5.0, true, false).state, OffboardState::Prestream);
  EXPECT_TRUE(s.step(5.5, true, false).send_mode_command);
}
TEST(Offboard, DisableStreamsStopForTheWindowThenStopsHeartbeat) {  // PXL-002
  OffboardSession s{OffboardTiming{}};
  s.enable(true, 0.0);
  s.step(0.5, true, false);
  ASSERT_EQ(s.step(0.6, true, true).state, OffboardState::Active);
  s.enable(false, 0.7);
  for (const double t : {0.7, 0.8, 0.99}) {
    const auto o = s.step(t, true, true);
    EXPECT_EQ(o.state, OffboardState::Disabled);
    EXPECT_TRUE(o.publish_heartbeat) << t;
    EXPECT_TRUE(o.stop_only) << t;
    EXPECT_FALSE(o.send_mode_command);
  }
  EXPECT_FALSE(s.step(1.0, true, true).publish_heartbeat);  // 0.3 s window over: withdrawn
  EXPECT_FALSE(s.step(5.0, true, true).publish_heartbeat);
  s.enable(false, 6.0);  // a repeated disable does not restart the window
  EXPECT_FALSE(s.step(6.1, true, true).publish_heartbeat);
}
TEST(Offboard, DisableWindowNeedsAPriorEnableAndALink) {
  OffboardSession never{OffboardTiming{}};
  never.enable(false, 0.0);
  EXPECT_FALSE(never.step(0.1, true, false).publish_heartbeat);
  OffboardSession s{OffboardTiming{}};
  s.enable(true, 0.0);
  s.step(0.1, true, false);
  s.enable(false, 0.2);
  EXPECT_FALSE(s.step(0.25, false, false).publish_heartbeat);  // no trustworthy zero without link
  const auto o = s.step(0.3, true, false);
  EXPECT_TRUE(o.publish_heartbeat);
  EXPECT_TRUE(o.stop_only);
  s.enable(true, 0.4);  // re-enable cancels the window and restarts the sequence
  EXPECT_EQ(s.step(0.4, true, false).state, OffboardState::Prestream);
}

// --- state assembler ---------------------------------------------------------------------------
TEST(Assembler, StaleSourcesContributeNothing) {
  LocalPositionSample lp;
  lp.xy_valid = lp.v_xy_valid = lp.heading_good_for_control = true;
  lp.x = 5;
  lp.y = 6;
  lp.heading = 1.0F;
  AttitudeSample att;
  att.q = {0.5F, 0.5F, 0.5F, 0.5F};
  StatusSample st;
  st.arming_state = 2;
  st.nav_state = 14;
  const auto none = assemble(lp, att, st, Freshness{});
  EXPECT_FALSE(none.position_valid);
  EXPECT_FALSE(none.velocity_valid);
  EXPECT_FALSE(none.attitude_valid);
  EXPECT_EQ(none.north, 0.0F);
  EXPECT_EQ(none.arming_state, 0);
  const auto all = assemble(lp, att, st, Freshness{true, true, true});
  EXPECT_TRUE(all.position_valid);
  EXPECT_TRUE(all.attitude_valid);
  EXPECT_FLOAT_EQ(all.north, 5.0F);
  EXPECT_FLOAT_EQ(all.east, 6.0F);
  EXPECT_EQ(all.nav_state, 14);
  EXPECT_FLOAT_EQ(all.q[0], 0.5F);
}
TEST(Assembler, AttitudeNeedsHeadingGoodAndFreshPosition) {
  LocalPositionSample lp;
  lp.xy_valid = true;
  lp.heading = 0.3F;
  AttitudeSample att;
  StatusSample st;
  EXPECT_FALSE(
      assemble(lp, att, st, Freshness{true, true, true}).attitude_valid);  // heading not good
  lp.heading_good_for_control = true;
  EXPECT_TRUE(assemble(lp, att, st, Freshness{true, true, true}).attitude_valid);
  EXPECT_FALSE(assemble(lp, att, st, Freshness{false, true, true}).attitude_valid);
  EXPECT_FALSE(assemble(lp, att, st, Freshness{true, false, true}).attitude_valid);
  att.q[2] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(assemble(lp, att, st, Freshness{true, true, true}).attitude_valid);
}
TEST(Assembler, GlobalReferenceValidNeedsFiniteLatLonAlt) {  // PXL-006
  LocalPositionSample lp;
  lp.xy_global = true;
  lp.ref_lat = 52.1;
  lp.ref_lon = 4.3;
  lp.ref_alt = 3.0F;
  const Freshness f{true, true, true};
  EXPECT_TRUE(assemble(lp, AttitudeSample{}, StatusSample{}, f).global_reference_valid);
  EXPECT_FALSE(assemble(lp, AttitudeSample{}, StatusSample{}, Freshness{}).global_reference_valid);
  for (int i = 0; i < 3; ++i) {
    LocalPositionSample bad = lp;
    if (i == 0) bad.ref_lat = std::numeric_limits<double>::quiet_NaN();
    if (i == 1) bad.ref_lon = std::numeric_limits<double>::infinity();
    if (i == 2) bad.ref_alt = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(assemble(bad, AttitudeSample{}, StatusSample{}, f).global_reference_valid) << i;
  }
  lp.xy_global = false;
  EXPECT_FALSE(assemble(lp, AttitudeSample{}, StatusSample{}, f).global_reference_valid);
}
TEST(Assembler, NonFinitePositionIsInvalid) {
  LocalPositionSample lp;
  lp.xy_valid = true;
  lp.x = std::numeric_limits<float>::quiet_NaN();
  const auto o = assemble(lp, AttitudeSample{}, StatusSample{}, Freshness{true, true, true});
  EXPECT_FALSE(o.position_valid);
}
TEST(Assembler, VerticalValidityFollowsZValidAndFreshness) {  // IF-004
  LocalPositionSample lp;
  lp.xy_valid = lp.v_xy_valid = true;
  const Freshness f{true, true, true};
  auto o = assemble(lp, AttitudeSample{}, StatusSample{}, f);
  EXPECT_FALSE(o.vertical_position_valid);  // PX4 did not set z_valid / v_z_valid
  EXPECT_FALSE(o.vertical_velocity_valid);
  lp.z_valid = lp.v_z_valid = true;
  lp.z = -1.5F;
  lp.vz = 0.1F;
  o = assemble(lp, AttitudeSample{}, StatusSample{}, f);
  EXPECT_TRUE(o.vertical_position_valid);
  EXPECT_TRUE(o.vertical_velocity_valid);
  EXPECT_FLOAT_EQ(o.down, -1.5F);
  o = assemble(lp, AttitudeSample{}, StatusSample{}, Freshness{});  // stale: nothing
  EXPECT_FALSE(o.vertical_position_valid);
  EXPECT_FALSE(o.vertical_velocity_valid);
  lp.z = std::numeric_limits<float>::quiet_NaN();
  lp.vz = std::numeric_limits<float>::infinity();
  o = assemble(lp, AttitudeSample{}, StatusSample{}, f);
  EXPECT_FALSE(o.vertical_position_valid);  // a non-finite value is never presented as valid
  EXPECT_FALSE(o.vertical_velocity_valid);
  EXPECT_TRUE(o.position_valid);  // independent of the horizontal flags
}

// --- yaw rate (RPP-009) --------------------------------------------------------------------------
namespace {
std::array<float, 4> q_of_yaw(double yaw) {  // rotation about NED down: positive is clockwise
  return {static_cast<float>(std::cos(yaw / 2)), 0.0F, 0.0F, static_cast<float>(std::sin(yaw / 2))};
}
double wrap_pi(double a) { return std::remainder(a, 2.0 * 3.141592653589793); }
}  // namespace

TEST(YawRate, YawOfMatchesTheHeadingConvention) {
  EXPECT_NEAR(yaw_of(q_of_yaw(0.5)), 0.5, 1e-6);
  EXPECT_NEAR(yaw_of(q_of_yaw(-2.0)), -2.0, 1e-6);
  EXPECT_NEAR(std::abs(yaw_of(q_of_yaw(3.141592653589793))), 3.141592653589793, 1e-6);
}
TEST(YawRate, ConstantRotationIsRecoveredWithinFivePercent) {
  for (const double rate : {0.2, -0.2, 0.05, 1.0}) {
    YawRateEstimator e{0.05};
    EXPECT_FALSE(e.valid());
    EXPECT_EQ(e.rate(), 0.0F);
    const uint64_t t0 = 1'700'000'000'000'000ULL;  // system-clock domain microseconds
    for (int k = 0; k <= 100; ++k) {               // 1 s at 100 Hz
      e.update(q_of_yaw(wrap_pi(0.3 + rate * k * 0.01)), t0 + static_cast<uint64_t>(k) * 10000U, 0);
      if (k >= 20)
        ASSERT_NEAR(e.rate(), rate, 0.05 * std::abs(rate)) << "rate " << rate << " k " << k;
    }
    EXPECT_TRUE(e.valid());
  }
}
TEST(YawRate, WrapAcrossPlusMinusPiIsContinuous) {
  for (const double rate : {0.5, -0.5}) {
    YawRateEstimator e{0.05};
    const double start = rate > 0 ? 3.0 : -3.0;  // crosses +-pi after ~0.28 s
    for (int k = 0; k <= 100; ++k) {
      e.update(q_of_yaw(wrap_pi(start + rate * k * 0.01)), static_cast<uint64_t>(k) * 10000U + 1U,
               0);
      if (k >= 1)
        ASSERT_NEAR(e.rate(), rate, 0.05 * std::abs(rate)) << "rate " << rate << " k " << k;
    }
  }
}
TEST(YawRate, UsesSampleTimeNotArrivalCadence) {
  YawRateEstimator e{0.0};
  e.update(q_of_yaw(0.0), 1000000U, 0);
  e.update(q_of_yaw(0.02), 1020000U, 0);  // 0.02 rad over a 20 ms sample interval
  EXPECT_NEAR(e.rate(), 1.0, 1e-4);
}
TEST(YawRate, StaleGapResetNonFiniteAndBackwardsTimeGiveNoRate) {
  YawRateEstimator e{0.05};
  uint64_t t = 1000000U;
  double yaw = 0.0;
  auto feed = [&](int n, uint8_t reset = 0) {
    for (int k = 0; k < n; ++k) {
      t += 10000U;
      yaw += 0.003;
      e.update(q_of_yaw(yaw), t, reset);
    }
  };
  feed(20);
  ASSERT_TRUE(e.valid());
  EXPECT_NEAR(e.rate(), 0.3, 0.015);
  e.update(q_of_yaw(yaw), t, 0);  // the same sample again: ignored
  EXPECT_TRUE(e.valid());
  t += 210000U;  // a gap over 0.2 s
  yaw += 2.0;
  e.update(q_of_yaw(yaw), t, 0);
  EXPECT_FALSE(e.valid());
  EXPECT_EQ(e.rate(), 0.0F);
  feed(1);
  EXPECT_TRUE(e.valid());
  EXPECT_NEAR(e.rate(), 0.3, 0.015);  // restarted from the next delta, no spike from the gap
  yaw += 1.0;                         // EKF yaw reset: the quaternion jumps, the counter moves
  feed(1, 1);
  EXPECT_FALSE(e.valid());
  feed(1, 1);
  EXPECT_NEAR(e.rate(), 0.3, 0.015);
  e.update({std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 0.0F}, t + 10000U, 1);
  EXPECT_FALSE(e.valid());
  t -= 50000U;  // sample time going backwards (FCU reboot): restart
  feed(1, 1);
  feed(1, 1);
  EXPECT_TRUE(e.valid());
  e.update(q_of_yaw(yaw), t - 5000U, 1);
  EXPECT_FALSE(e.valid());
}
