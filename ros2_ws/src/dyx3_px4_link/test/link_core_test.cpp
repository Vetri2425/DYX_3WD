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
  EXPECT_FALSE(m.evaluate(3.0).session_alive);  // everything silent: agent died
  EXPECT_EQ(m.session_resets(), 0U);
  feed_all(m, 3.5);
  EXPECT_TRUE(m.evaluate(3.5).session_alive);
  EXPECT_EQ(m.session_resets(), 1U);
  EXPECT_TRUE(m.consume_reset());
  EXPECT_FALSE(m.consume_reset());
  m.evaluate(3.51);
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
TEST(Offboard, DisableStopsHeartbeat) {
  OffboardSession s{OffboardTiming{}};
  s.enable(true, 0.0);
  s.step(0.6, true, true);
  s.enable(false, 0.7);
  EXPECT_FALSE(s.step(0.8, true, true).publish_heartbeat);
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
TEST(Assembler, NonFinitePositionIsInvalid) {
  LocalPositionSample lp;
  lp.xy_valid = true;
  lp.x = std::numeric_limits<float>::quiet_NaN();
  const auto o = assemble(lp, AttitudeSample{}, StatusSample{}, Freshness{true, true, true});
  EXPECT_FALSE(o.position_valid);
}
