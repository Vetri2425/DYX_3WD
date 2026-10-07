#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "dyx3_spray/spray_controller.hpp"
#include "dyx3_spray/watchdog_core.hpp"

using namespace dyx3_spray;

// ------------------------------------------------------------------------------------------------
// lease ---------------
TEST(Lease, ValidationRejectsEveryUnsafeValue) {
  Lease ok;
  EXPECT_TRUE(validate_lease(ok).empty());
  const auto bad = [&](auto mutate) {
    Lease l = ok;
    mutate(l);
    return !validate_lease(l).empty();
  };
  EXPECT_TRUE(bad([](Lease& l) { l.command_seq = -1; }));
  EXPECT_TRUE(bad([](Lease& l) { l.backend = 2; }));
  EXPECT_TRUE(bad([](Lease& l) { l.actuator_set_index = 0; }));
  EXPECT_TRUE(bad([](Lease& l) { l.actuator_set_index = 7; }));
  EXPECT_TRUE(bad([](Lease& l) { l.off_value = NAN; }));
  EXPECT_TRUE(bad([](Lease& l) { l.off_value = 1.01; }));
  EXPECT_TRUE(bad([](Lease& l) { l.off_value = -1.01; }));
  EXPECT_TRUE(bad([](Lease& l) { l.servo_instance = 0; }));
  EXPECT_TRUE(bad([](Lease& l) { l.servo_instance = 17; }));
  EXPECT_TRUE(bad([](Lease& l) { l.off_pwm_us = -1; }));
  EXPECT_TRUE(bad([](Lease& l) { l.off_pwm_us = 2201; }));
}

TEST(LeaseMonitor, EachWayToNeedOffHasItsOwnCause) {
  LeaseMonitor m(0.35);
  EXPECT_EQ(m.off_reason(1.0).cause, OffCause::NoLease);
  Lease l;
  l.allow_on = true;
  ASSERT_TRUE(m.observe(l, 1.0));
  EXPECT_FALSE(m.off_reason(1.2).required());  // fresh and allowing ON
  EXPECT_EQ(m.off_reason(1.4).cause, OffCause::Stale);
  l.allow_on = false;
  ASSERT_TRUE(m.observe(l, 2.0));
  EXPECT_EQ(m.off_reason(2.1).cause, OffCause::Denied);
  l.allow_on = true;
  ASSERT_TRUE(m.observe(l, 3.0));
  Lease bad = l;
  bad.backend = 7;
  EXPECT_FALSE(m.observe(bad, 3.1));
  EXPECT_EQ(m.off_reason(3.1).cause, OffCause::Invalidated);  // ON revoked at once, mapping kept
  EXPECT_EQ(m.last_lease().backend, kBackendActuator);
  ASSERT_TRUE(m.observe(l, 3.2));  // a fresh valid lease restores
  EXPECT_FALSE(m.off_reason(3.2).required());
  m.invalidate("service shutdown", true);
  EXPECT_EQ(m.off_reason(3.2).cause, OffCause::Shutdown);
}

TEST(LeaseMonitor, ReceiveTimeNotSenderTimeAndAClockStepBackIsNotAnExtension) {
  LeaseMonitor m(0.35);
  Lease l;
  l.allow_on = true;
  ASSERT_TRUE(m.observe(l, 10.0));
  EXPECT_FALSE(
      m.off_reason(9.0).required());  // negative age clamps to 0, never "very fresh forever"
  EXPECT_TRUE(m.off_reason(10.4).required());
  EXPECT_THROW(LeaseMonitor(0.0), std::invalid_argument);
  EXPECT_THROW(LeaseMonitor(NAN), std::invalid_argument);
}

// ------------------------------------------------------------------------------------------------
// watchdog core ---------
namespace {
Lease allowing() {
  Lease l;
  l.allow_on = true;
  return l;
}
}  // namespace

TEST(WatchdogCore, StartsFailClosedAndProvesItCanCloseTheValve) {
  WatchdogCore w{WatchdogParams{}};
  const auto c = w.tick(1.0);
  ASSERT_TRUE(c.has_value());  // before any lease: OFF with the fallback mapping
  EXPECT_EQ(c->mapping.backend, kBackendActuator);
  EXPECT_FALSE(w.status(1.0).off_authority_ready);
  EXPECT_TRUE(w.status(1.0).off_inflight);
  EXPECT_FALSE(w.tick(1.001).has_value());  // one OFF in flight at a time
  w.on_ack(c->seq + 5, true, 1.01);         // someone else's ack: ignored
  EXPECT_FALSE(w.status(1.01).off_authority_ready);
  w.on_ack(c->seq, true, 1.02);
  EXPECT_TRUE(w.status(1.02).off_authority_ready);
  EXPECT_FALSE(w.status(1.02).allow_on);
}

TEST(WatchdogCore, OffBurstThenBackgroundRate) {
  WatchdogParams p;
  WatchdogCore w{p};
  int sent_burst = 0, sent_after = 0;
  for (double t = 1.0; t < 1.0 + 1.5; t += 0.01) {
    if (const auto c = w.tick(t)) {
      ++sent_burst;
      w.on_ack(c->seq, true, t);
    }
  }
  for (double t = 2.5; t < 4.5; t += 0.01) {
    if (const auto c = w.tick(t)) {
      ++sent_after;
      w.on_ack(c->seq, true, t);
    }
  }
  EXPECT_NEAR(sent_burst, 30, 3);  // 20 Hz for 1.5 s
  EXPECT_NEAR(sent_after, 4, 1);   // 2 Hz for 2 s
}

TEST(WatchdogCore, AFreshAllowingLeaseSilencesOffAndAStaleOneResumesIt) {
  WatchdogCore w{WatchdogParams{}};
  auto c = w.tick(1.0);
  w.on_ack(c->seq, true, 1.0);
  w.on_lease(allowing(), 3.0);
  for (double t = 3.0; t < 3.3; t += 0.02) EXPECT_FALSE(w.tick(t).has_value()) << t;
  EXPECT_TRUE(w.status(3.2).allow_on);
  // lease goes stale at 3.35 s: OFF at once, with a new burst
  std::optional<OffCommand> first;
  for (double t = 3.3; t < 3.5 && !first; t += 0.01) first = w.tick(t);
  ASSERT_TRUE(first.has_value());
  EXPECT_FALSE(w.status(3.5).allow_on);
  EXPECT_NE(w.status(3.5).off_reason.find("stale"), std::string::npos);
}

TEST(WatchdogCore, StaleAgeTextDoesNotRearmTheBurstEveryTick) {
  // DERIVED fix of the prototype quirk (see watchdog_core.hpp): while the lease stays stale, OFFs
  // go out at the background rate after the first burst, not at 20 Hz forever.
  WatchdogCore w{WatchdogParams{}};
  w.on_lease(allowing(), 1.0);
  int sent = 0;
  for (double t = 1.0; t < 12.0; t += 0.01) {
    if (const auto c = w.tick(t)) {
      ++sent;
      w.on_ack(c->seq, true, t);
    }
  }
  // 1.5 s burst (<= 30) + 2 Hz for ~9 s (<= 20) + slack; the prototype's behaviour would be ~200+
  EXPECT_LT(sent, 60);
  EXPECT_GT(sent, 20);
}

TEST(WatchdogCore, MalformedLeaseRevokesOnAtOnceAndKeepsTheLastGoodMapping) {
  WatchdogCore w{WatchdogParams{}};
  Lease good = allowing();
  good.backend = kBackendServoPwm;
  good.servo_instance = 3;
  good.off_pwm_us = 1000;
  w.on_lease(good, 1.0);
  auto first = w.tick(1.0);
  EXPECT_FALSE(first.has_value());  // allowed: nothing to do
  Lease bad = good;
  bad.off_pwm_us = 5000;
  w.on_lease(bad, 1.1);
  const auto c = w.tick(1.1);
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->mapping.backend, kBackendServoPwm);
  EXPECT_EQ(c->mapping.servo_instance, 3);
  EXPECT_EQ(c->mapping.off_pwm_us, 1000);  // the last VALID mapping, never the malformed one
}

TEST(WatchdogCore, AnAckThatNeverArrivesIsAFailureAndTheNextOffGoesOutAtOnce) {
  WatchdogCore w{WatchdogParams{}};
  const auto c = w.tick(1.0);
  ASSERT_TRUE(c.has_value());
  EXPECT_FALSE(w.tick(1.9).has_value());  // still within the 1.0 s ack window
  const auto retry = w.tick(2.05);
  ASSERT_TRUE(retry.has_value());
  EXPECT_NE(retry->seq, c->seq);
  EXPECT_FALSE(w.status(2.05).off_authority_ready);
}

TEST(WatchdogCore, ARejectedOffWithdrawsAuthorityAndRetriesImmediately) {
  WatchdogCore w{WatchdogParams{}};
  auto c = w.tick(1.0);
  w.on_ack(c->seq, true, 1.01);
  EXPECT_TRUE(w.status(1.01).off_authority_ready);
  w.begin_shutdown(1.5);
  auto c2 = w.tick(1.5);
  ASSERT_TRUE(c2.has_value());
  w.on_ack(c2->seq, false, 1.51);  // the link refused (result 255): we cannot prove OFF
  EXPECT_FALSE(w.status(1.51).off_authority_ready);
  EXPECT_TRUE(w.tick(1.52).has_value());
  EXPECT_EQ(w.status(1.52).off_reason, "service shutdown");
}

TEST(WatchdogCore, InvalidFallbackMappingRefusesToStart) {
  WatchdogParams p;
  p.fallback.off_value = 2.0;
  EXPECT_THROW(WatchdogCore{p}, std::invalid_argument);
}

// ------------------------------------------------------------------------------------------------
// params ---------------
TEST(SprayParams, StartupLoadIsStructuralOnlyRuntimeEnforcesClassesAndRecords) {
  ParamSet p;
  EXPECT_FALSE(p.init_many({{"max_xtrack_error_m", -1.0, ""}}).ok);
  EXPECT_TRUE(p.init_many({{"max_xtrack_error_m", 0.04, ""},
                           {"actuator_backend", 0.0, "mavlink_servo_pwm"}})
                  .ok);
  EXPECT_EQ(p.num(P::max_xtrack_error_m), 0.04);
  SetContext ctx;
  ctx.source = "test";
  EXPECT_FALSE(p.set({"actuator_backend", 0.0, "mavlink_actuator"}, ctx).ok);  // RESTART
  EXPECT_TRUE(p.set({"max_xtrack_error_m", 0.03, ""}, ctx).ok);  // IDLE_ONLY while idle
  ctx.mission_running = true;
  EXPECT_FALSE(p.set({"max_xtrack_error_m", 0.02, ""}, ctx).ok);  // refused while running
  EXPECT_EQ(p.num(P::max_xtrack_error_m), 0.03);
  EXPECT_FALSE(p.set({"actuator_backend", 0.0, "bogus"}, SetContext{}).ok);
  EXPECT_EQ(p.journal().back().source, "test");
}

// ------------------------------------------------------------------------------------------------
// controller ----------
namespace {

// A 10 m straight line: TRANSIT 0..2 m, MARK 2..9 m (vertices 2..8 are MARK: the last MARK segment
// is 8->9), TRANSIT 9..10 m.
std::shared_ptr<const PathModel> straight() {
  std::vector<double> n, e;
  std::vector<bool> f;
  for (int i = 0; i <= 10; ++i) {
    n.push_back(i);
    e.push_back(0.0);
    f.push_back(i >= 2 && i <= 8);
  }
  auto m = std::make_shared<PathModel>();
  EXPECT_TRUE(build_path_model(n, e, f, m.get()));
  return m;
}

struct Rig {
  ParamSet params;
  std::unique_ptr<SprayController> c;
  double t{100.0};
  uint32_t pending_seq{0};
  bool have_pending{false};
  bool ack_ok{true};
  int ons{0}, offs{0};
  bool ack_enabled{true};
  bool tracking{true};
  bool pivot{false};

  explicit Rig(bool tracking0 = true) {
    tracking = tracking0;
    params.init_many(
        {{"gps_recover_hold_s", 0.0, ""}});  // recovery hold is covered in the gate tests
    c = std::make_unique<SprayController>(&params);
    c->load_path(straight());
    world(0.0, true);
  }

  void world(double north, bool armed, double speed = 0.35, bool estop = false) {
    VehicleSnapshot v;
    v.armed = armed;
    v.offboard = true;
    v.position_valid = v.attitude_valid = v.velocity_valid = true;
    v.north_m = north;
    v.east_m = 0.0;
    v.heading_rad = 0.0;
    v.vel_n_mps = speed;
    c->note_vehicle(v, t);
    RtkSnapshot r;
    r.fix_type = 6;
    r.h_acc_m = 0.02;
    c->note_rtk(r, t);
    c->note_rpp(tracking, pivot, t);
    c->note_estop(estop, t);
    c->note_watchdog(true, true, t);
  }

  void run_cmd(std::optional<SprayCommand> cmd) {
    while (cmd) {
      (cmd->on ? ons : offs)++;
      pending_seq = cmd->seq;
      have_pending = true;
      if (!ack_enabled) return;
      cmd = c->on_ack(pending_seq, ack_ok, t);
      have_pending = false;
    }
  }

  void step(double north, double speed = 0.35, bool estop = false, bool armed = true) {
    t += 0.02;
    world(north, armed, speed, estop);
    run_cmd(c->tick(t));
  }
};

}  // namespace

TEST(Controller, DoesNotSprayBeforeOffIsConfirmedAndNeverOnATransitLeg) {
  Rig r;
  r.step(0.5);
  EXPECT_FALSE(r.c->status(r.t).spraying);
  for (double n = 0.5; n < 1.7; n += 0.007) r.step(n);
  EXPECT_EQ(r.ons, 0);
  EXPECT_FALSE(r.c->status(r.t).spraying);
  EXPECT_EQ(r.c->fsm().state(), SprayState::OffConfirmed);
}

TEST(Controller, OpensEarlyByTheLeadSpraysTheMarkAndClosesBeforeTheBoundary) {
  Rig r;
  double on_at = -1, off_at = -1;
  bool prev = false;
  for (double n = 0.0; n < 9.9; n += 0.007) {
    r.step(n);
    const bool s = r.c->status(r.t).spraying;
    if (s && !prev && on_at < 0) on_at = n;
    if (!s && prev) off_at = n;
    prev = s;
  }
  // lead = 0.35*0.18 + 0.02 = 0.083 m before the MARK start at 2.0 (debounce 3 ticks and the ack
  // tick cost ~4 cm more)
  EXPECT_NEAR(on_at, 2.0 - 0.083, 0.06);
  EXPECT_LT(on_at, 2.0);
  // The MARK ends at 9.0 (vertex 8 is the last MARK vertex). off lead = max(0, 0.35*0.05 - 0) =
  // 0.0175 m before it; the debounce (3 ticks = 2.1 cm at 0.35 m/s) then delays the CLOSE by 2.1
  // cm. That latency is carried from the prototype and is part of the boundary budget (see the
  // contract, section 5).
  EXPECT_NEAR(off_at, 9.0 - 0.0175 + 0.021, 0.02);
  EXPECT_GT(off_at, 9.0 - 0.0175);
  EXPECT_EQ(r.ons, 1);
}

TEST(Controller, EmergencyStopForcesOffAtTheNextTickAndRevokesTheLease) {
  Rig r;
  double n = 0.0;
  for (; n < 4.0; n += 0.007) r.step(n);
  ASSERT_TRUE(r.c->status(r.t).spraying);
  ASSERT_TRUE(r.c->lease(r.t).allow_on);
  const int offs_before = r.offs;
  r.step(n, 0.35, /*estop=*/true);
  EXPECT_GT(r.offs, offs_before);
  EXPECT_FALSE(r.c->lease(r.t).allow_on);
  EXPECT_FALSE(r.c->status(r.t).spraying);
  EXPECT_NE(r.c->status(r.t).safety_reason.find("emergency stop"), std::string::npos);
}

TEST(Controller, MissingEstopStateIsTreatedAsAsserted) {
  ParamSet params;
  SprayController c(&params);
  c.load_path(straight());
  VehicleSnapshot v;
  v.armed = v.offboard = v.position_valid = v.attitude_valid = v.velocity_valid = true;
  v.north_m = 3.0;
  c.note_vehicle(v, 1.0);
  c.note_watchdog(true, true, 1.0);
  c.tick(1.0);
  EXPECT_FALSE(c.status(1.0).safety_ok);
  EXPECT_FALSE(c.lease(1.0).allow_on);
  c.note_estop(false, 1.0);
  EXPECT_FALSE(c.status(1.6).safety_ok);  // the clear itself goes stale after 0.5 s
}

TEST(Controller, AFailedOnIsNeverLatchedAndLeadsToAFreshOff) {
  Rig r;
  r.ack_ok = false;
  for (double n = 1.5; n < 3.0; n += 0.007) r.step(n);
  EXPECT_FALSE(r.c->status(r.t).spraying);  // invariant 1: spraying only in ON_CONFIRMED
  EXPECT_GT(r.offs, 0);
}

TEST(Controller, AckTimeoutDoesNotWedgeThePendingOn) {
  Rig r;
  r.ack_enabled = false;  // dispatched commands are never acknowledged
  for (double n = 0.0; n < 3.0; n += 0.007) r.step(n);
  EXPECT_FALSE(r.c->status(r.t).spraying);
  EXPECT_GE(r.offs, 2);  // the pending commands time out and a fresh OFF goes out
}

TEST(Controller, DisarmedOrWatchdogLostOrDisabledBlocksSprayAndTheLease) {
  Rig r;
  double n = 0.0;
  for (; n < 4.0; n += 0.007) r.step(n);
  ASSERT_TRUE(r.c->lease(r.t).allow_on);
  r.step(n, 0.35, false, /*armed=*/false);
  EXPECT_FALSE(r.c->lease(r.t).allow_on);
  EXPECT_FALSE(r.c->status(r.t).safety_ok);
  EXPECT_EQ(r.c->status(r.t).safety_reason, "disarmed");

  Rig w;
  for (n = 0.0; n < 4.0; n += 0.007) w.step(n);
  ASSERT_TRUE(w.c->lease(w.t).allow_on);
  w.t += 1.2;  // the watchdog heartbeat is not refreshed (note_watchdog only runs in world())
  w.c->note_vehicle(VehicleSnapshot{true, true, true, true, true, 4.0, 0.0, 0.0, 0.35, 0.0}, w.t);
  w.c->note_rtk(RtkSnapshot{6, 0.02}, w.t);
  w.c->note_rpp(true, false, w.t);
  w.c->note_estop(false, w.t);
  w.run_cmd(w.c->tick(w.t));
  EXPECT_FALSE(w.c->lease(w.t).allow_on);
  EXPECT_NE(w.c->status(w.t).safety_reason.find("watchdog"), std::string::npos);
}

TEST(Controller, RtkLossCutsTheValveWithinOneTick) {
  Rig r;
  double n = 0.0;
  for (; n < 4.0; n += 0.007) r.step(n);
  ASSERT_TRUE(r.c->status(r.t).spraying);
  r.t += 0.02;
  r.world(n, true);
  r.c->note_rtk(RtkSnapshot{5, 0.4}, r.t);  // float with poor accuracy
  r.run_cmd(r.c->tick(r.t));
  EXPECT_FALSE(r.c->status(r.t).spraying);
  EXPECT_FALSE(r.c->lease(r.t).allow_on);
}

TEST(Controller, NoSprayUntilTrackingIsSeenAndPivotingSuppressesIt) {
  Rig r(/*tracking0=*/false);  // the rover is parked ON a spray-flagged vertex but RPP has not
                               // started tracking this path
  for (int i = 0; i < 100; ++i) r.step(3.0, 0.0);
  EXPECT_EQ(r.ons, 0);
  EXPECT_EQ(r.c->status(r.t).safety_reason, "awaiting tracking");
  r.tracking = true;
  for (int i = 0; i < 20; ++i) r.step(3.0, 0.35);
  EXPECT_TRUE(r.c->status(r.t).spraying);
  r.pivot = true;  // CORNER_ALIGN: the valve closes while the rover pivots in place
  for (int i = 0; i < 5; ++i) r.step(3.0, 0.0);
  EXPECT_FALSE(r.c->status(r.t).spraying);
  EXPECT_EQ(r.c->status(r.t).safety_reason, "pivoting in place");
  r.pivot = false;
  r.tracking = false;  // tracking evidence is latched for the rest of this path
  for (int i = 0; i < 20; ++i) r.step(3.0, 0.35);
  EXPECT_TRUE(r.c->status(r.t).spraying);
  r.c->load_path(straight());  // a new path resets the evidence
  r.step(3.0);
  EXPECT_EQ(r.c->status(r.t).safety_reason, "awaiting tracking");
}

TEST(Controller, ManualOverrideObeysFailSafesAndExpires) {
  Rig r;
  r.step(0.5);
  r.step(0.5);
  EXPECT_EQ(r.c->set_manual(true, r.t), ManualResult::Ok);
  for (int i = 0; i < 5; ++i) r.step(0.5);
  EXPECT_TRUE(r.c->status(r.t).spraying);
  EXPECT_TRUE(r.c->status(r.t).manual_active);
  // expiry after manual_override_timeout_s (10 s)
  for (int i = 0; i < 520; ++i) r.step(0.5);
  EXPECT_FALSE(r.c->status(r.t).manual_active);
  EXPECT_FALSE(r.c->status(r.t).spraying);

  Rig d;
  d.step(0.5);
  EXPECT_EQ(d.c->set_manual(true, d.t), ManualResult::Ok);
  d.step(0.5, 0.35, false, /*armed=*/false);  // fail-safes outrank the override
  EXPECT_FALSE(d.c->status(d.t).manual_active);
  EXPECT_EQ(d.c->set_manual(true, d.t), ManualResult::Disarmed);
}

TEST(Controller, ShutdownRevokesTheLeaseAndForcesOff) {
  Rig r;
  double n = 0.0;
  for (; n < 4.0; n += 0.007) r.step(n);
  ASSERT_TRUE(r.c->status(r.t).spraying);
  r.run_cmd(r.c->shutdown(r.t));
  EXPECT_FALSE(r.c->lease(r.t).allow_on);
  EXPECT_FALSE(r.c->status(r.t).spraying);
  EXPECT_GE(r.offs, 2);
}

TEST(Controller, WireValuesFollowTheBackendAndNeverExceedOnValue) {
  Rig r;
  const ActuatorWire off = r.c->wire_for(false);
  EXPECT_EQ(off.backend, kBackendActuator);
  EXPECT_EQ(off.value, -1.0);
  const ActuatorWire on = r.c->wire_for(true);
  EXPECT_EQ(on.value, 1.0);
  EXPECT_EQ(on.actuator_set_index, 1);
}
