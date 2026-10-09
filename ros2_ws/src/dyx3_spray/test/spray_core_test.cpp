#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

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

TEST(WatchdogCore, OffProofIsBoundToTheCurrentMappingAndOldAckCannotProveNewMapping) {
  WatchdogCore w{WatchdogParams{}};
  const auto off_a = w.tick(1.0);  // startup fallback mapping A
  ASSERT_TRUE(off_a.has_value());
  w.on_ack(off_a->seq, true, 1.01);
  ASSERT_TRUE(w.status(1.01).off_authority_ready);

  Lease mapping_b = allowing();
  mapping_b.backend = kBackendServoPwm;
  mapping_b.servo_instance = 3;
  mapping_b.off_pwm_us = 1000;
  w.on_lease(mapping_b, 1.1);
  EXPECT_FALSE(w.status(1.1).off_authority_ready);
  EXPECT_FALSE(w.status(1.1).allow_on);
  w.on_ack(off_a->seq, true, 1.11);  // delayed A ACK is no longer in flight
  EXPECT_FALSE(w.status(1.11).off_authority_ready);

  const auto off_b = w.tick(1.11);
  ASSERT_TRUE(off_b.has_value());
  EXPECT_EQ(off_b->mapping.backend, kBackendServoPwm);
  EXPECT_EQ(off_b->mapping.servo_instance, 3);
  EXPECT_EQ(off_b->mapping.off_pwm_us, 1000);
  EXPECT_NE(off_b->seq, off_a->seq);
  w.on_ack(off_a->seq, true, 1.12);  // stale A ACK while B is in flight
  EXPECT_FALSE(w.status(1.12).off_authority_ready);
  w.on_ack(off_b->seq, true, 1.13);
  EXPECT_TRUE(w.status(1.13).off_authority_ready);
}

TEST(WatchdogCore, MappingChangeDuringInflightOffInvalidatesSequenceAndRequiresNewOff) {
  WatchdogCore w{WatchdogParams{}};
  const auto off_a = w.tick(1.0);
  ASSERT_TRUE(off_a.has_value());
  Lease mapping_b = allowing();
  mapping_b.off_value = -0.5;
  w.on_lease(mapping_b, 1.01);
  EXPECT_FALSE(w.status(1.01).off_authority_ready);

  const auto off_b = w.tick(1.01);
  ASSERT_TRUE(off_b.has_value());
  EXPECT_EQ(off_b->mapping.off_value, -0.5);
  w.on_ack(off_a->seq, true, 1.02);
  EXPECT_FALSE(w.status(1.02).off_authority_ready);
  w.on_ack(off_b->seq, true, 1.03);
  EXPECT_TRUE(w.status(1.03).off_authority_ready);
}

TEST(WatchdogCore, EveryActuatorMappingFieldInvalidatesOffProof) {
  const auto verify_change = [](const auto& mutate) {
    WatchdogCore w{WatchdogParams{}};
    const auto off_a = w.tick(1.0);
    if (!off_a) return false;
    w.on_ack(off_a->seq, true, 1.01);
    if (!w.status(1.01).off_authority_ready) return false;
    Lease changed = allowing();
    mutate(changed);
    w.on_lease(changed, 1.1);
    if (w.status(1.1).off_authority_ready || w.status(1.1).allow_on) return false;
    const auto off_b = w.tick(1.1);
    return off_b.has_value() &&
           same_actuator_mapping(actuator_mapping(off_b->mapping), actuator_mapping(changed));
  };

  EXPECT_TRUE(verify_change([](Lease& l) { l.backend = kBackendServoPwm; }));
  EXPECT_TRUE(verify_change([](Lease& l) { l.actuator_set_index = 2; }));
  EXPECT_TRUE(verify_change([](Lease& l) { l.off_value = -0.5; }));
  EXPECT_TRUE(verify_change([](Lease& l) { l.servo_instance = 2; }));
  EXPECT_TRUE(verify_change([](Lease& l) { l.off_pwm_us = 1000; }));
}

TEST(WatchdogCore, IdenticalMappingUpdatePreservesProofAndErrorStaysFailClosed) {
  WatchdogCore w{WatchdogParams{}};
  const auto off = w.tick(1.0);
  ASSERT_TRUE(off.has_value());
  w.on_ack(off->seq, true, 1.01);
  ASSERT_TRUE(w.status(1.01).off_authority_ready);
  w.on_lease(Lease{}, 1.1);  // exactly the fallback mapping
  EXPECT_TRUE(w.status(1.1).off_authority_ready);

  w.begin_shutdown(1.2);
  EXPECT_FALSE(w.status(1.2).allow_on);
  EXPECT_TRUE(w.tick(1.2).has_value());
  EXPECT_FALSE(w.status(1.2).allow_on);
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

TEST(WatchdogCore, TenThousandPhysicalOffSendsKeepOneLogicalEpoch) {
  WatchdogCore w{WatchdogParams{}};
  uint32_t epoch = 0;
  int burst = 0;
  for (int i = 0; i < 10000; ++i) {
    const double t = i < 30 ? 1.0 + i * 0.051 : 2.55 + (i - 30) * 0.501;
    const auto wire = w.tick(t);
    ASSERT_TRUE(wire) << i;
    if (i == 0) epoch = wire->seq;
    EXPECT_EQ(wire->seq, epoch);
    if (i < 30) ++burst;
    w.on_ack(wire->seq, true, t);
  }
  EXPECT_EQ(burst, 30);
  EXPECT_TRUE(w.status(5000.0).off_authority_ready);
}

TEST(WatchdogCore, OffCadenceContinuesWhileAckIsMissing) {
  WatchdogCore w{WatchdogParams{}};
  int burst = 0;
  int background = 0;
  uint32_t epoch = 0;
  for (int i = 0; i < 350; ++i) {
    const double t = 1.0 + i * 0.01;
    if (const auto wire = w.tick(t)) {
      if (epoch == 0) epoch = wire->seq;
      EXPECT_EQ(wire->seq, epoch);
      (t < 2.5 ? burst : background)++;
    }
  }
  EXPECT_NEAR(burst, 30, 3);
  EXPECT_NEAR(background, 4, 1);
  EXPECT_FALSE(w.status(4.5).off_authority_ready);
}

TEST(WatchdogCore, LaterOffPeriodGetsNewIdentity) {
  WatchdogCore w{WatchdogParams{}};
  const auto first = w.tick(1.0);
  ASSERT_TRUE(first);
  w.on_ack(first->seq, true, 1.01);
  w.on_lease(allowing(), 2.0);
  EXPECT_FALSE(w.tick(2.0));
  EXPECT_TRUE(w.status(2.0).allow_on);
  const auto later = w.tick(2.4);  // lease stale: a new OFF-required period
  ASSERT_TRUE(later);
  EXPECT_NE(later->seq, first->seq);
  w.on_ack(first->seq, true, 2.41);
  EXPECT_TRUE(w.status(2.41).off_inflight);
  EXPECT_FALSE(w.status(2.41).off_authority_ready);
  w.on_ack(later->seq, true, 2.42);
  EXPECT_TRUE(w.status(2.42).off_authority_ready);
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
  ASSERT_TRUE(first.has_value());  // mapping changed from fallback; prove OFF with this mapping
  EXPECT_EQ(first->mapping.backend, kBackendServoPwm);
  w.on_ack(first->seq, true, 1.01);
  ASSERT_TRUE(w.status(1.01).off_authority_ready);
  Lease bad = good;
  bad.off_pwm_us = 5000;
  w.on_lease(bad, 1.1);
  const auto c = w.tick(1.1);
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->mapping.backend, kBackendServoPwm);
  EXPECT_EQ(c->mapping.servo_instance, 3);
  EXPECT_EQ(c->mapping.off_pwm_us, 1000);  // the last VALID mapping, never the malformed one
}

TEST(WatchdogCore, MissingAckNeverProvesOffAndPhysicalRetriesKeepEpoch) {
  WatchdogCore w{WatchdogParams{}};
  const auto c = w.tick(1.0);
  ASSERT_TRUE(c.has_value());
  const auto repeated = w.tick(1.9);  // physical reassert continues while proof is pending
  ASSERT_TRUE(repeated);
  EXPECT_EQ(repeated->seq, c->seq);
  const auto retry = w.tick(2.05);
  ASSERT_TRUE(retry.has_value());
  EXPECT_EQ(retry->seq, c->seq);  // same physical OFF mapping, same logical epoch
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
  bool rpp_alive{true};
  bool mission_running{true};
  std::optional<RppState> rpp_override;  // publish this state instead of tracking/pivot
  double period{0.02};                   // control tick period handed to the controller

  explicit Rig(bool tracking0 = true, double period_s = 0.02,
               const std::vector<Item>& extra_params = {}) {
    tracking = tracking0;
    period = period_s;
    params.init_many(
        {{"gps_recover_hold_s", 0.0, ""}});  // recovery hold is covered in the gate tests
    if (!extra_params.empty()) EXPECT_TRUE(params.init_many(extra_params).ok);
    c = std::make_unique<SprayController>(&params, period);
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
    r.corrections_fresh = true;
    c->note_rtk(r, t);
    c->set_mission(mission_running, 1);
    if (rpp_alive) {
      // not tracking: an RPP state that may paint but is not TRACKING ("awaiting tracking")
      const RppState st = rpp_override ? *rpp_override
                          : pivot      ? RppState::Pivoting
                          : tracking   ? RppState::Tracking
                                       : RppState::Stopping;
      c->note_rpp(static_cast<uint8_t>(st), 1, 0, 0.0, 1.0, true, t);
    }
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
    t += period;
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

TEST(Controller, MappingProofLossDuringOnPendingForcesOffAndRevokesLease) {
  Rig r;
  for (double n = 0.0; n < 1.7 && r.c->fsm().state() != SprayState::OffConfirmed; n += 0.007)
    r.step(n);
  ASSERT_EQ(r.c->fsm().state(), SprayState::OffConfirmed);
  r.ack_enabled = false;
  bool on_pending = false;
  for (double n = 1.7; n < 2.5 && !on_pending; n += 0.007) {
    r.step(n);
    on_pending = r.c->fsm().state() == SprayState::OnPending;
  }
  ASSERT_TRUE(on_pending);

  r.c->note_watchdog(true, false, r.t + 0.02);  // D2 mapping change withdrew OFF authority
  const auto off = r.c->tick(r.t + 0.02);
  ASSERT_TRUE(off.has_value());
  EXPECT_FALSE(off->on);
  EXPECT_FALSE(r.c->lease(r.t + 0.02).allow_on);
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
  // The command goes out where the valve still needs the solenoid delay to move: lead = 0.35*0.18 +
  // 0.02 = 0.083 m before the MARK start at 2.0. The debounce (3 samples = 2 ticks = 0.014 m at
  // 0.35 m/s) is led as well (SP-002), so only the sampling of the crossing (< one tick, 0.007 m)
  // remains.
  EXPECT_GE(on_at, 2.0 - 0.083 - 1e-9);
  EXPECT_LT(on_at, 2.0 - 0.083 + 0.007 + 1e-9);
  // The MARK ends at 9.0 (vertex 8 is the last MARK vertex): off lead = 0.35*0.05 - 0 = 0.0175 m.
  EXPECT_GE(off_at, 9.0 - 0.0175 - 1e-9);
  EXPECT_LT(off_at, 9.0 - 0.0175 + 0.007 + 1e-9);
  EXPECT_EQ(r.ons, 1);
}

// SP-002: constant-speed passes. The position of the tick that dispatches the command plus the
// distance covered during the solenoid delay is where paint starts/stops. It must land on the
// boundary (less the deliberate overspray margin) up to the sampling of the crossing, which is
// always late by [0, one tick of travel). Every phase of the samples against the boundary is tried.
namespace {
struct Edges {
  double paint_on_err{1e9};   // where paint starts minus (2.0 - on_overspray_margin_m)
  double paint_off_err{1e9};  // where paint stops minus (9.0 + off_overspray_margin_m)
  int ons{0};
};

Edges constant_speed_pass(double v, double period, double phase, int debounce_samples = 3) {
  Rig r(true, period, {{"debounce_samples", static_cast<double>(debounce_samples), ""}});
  const double open_delay = r.params.num(P::solenoid_open_delay_s);
  const double close_delay = r.params.num(P::solenoid_close_delay_s);
  const double on_margin = r.params.num(P::on_overspray_margin_m);
  const double off_margin = r.params.num(P::off_overspray_margin_m);
  Edges e;
  bool on_seen = false, off_seen = false;
  const double ds = v * period;
  for (double n = phase; n < 9.9; n += ds) {
    const int ons = r.ons, offs = r.offs;
    r.step(n, v);
    if (!on_seen && r.ons > ons) {
      on_seen = true;
      e.paint_on_err = (n + v * open_delay) - (2.0 - on_margin);
    } else if (on_seen && !off_seen && r.offs > offs) {
      off_seen = true;
      e.paint_off_err = (n + v * close_delay) - (9.0 + off_margin);
    }
  }
  e.ons = r.ons;
  return e;
}
}  // namespace

TEST(Controller, DebounceIsLedSoValveEdgesLandOnTheBoundaryAtProductionSpeeds) {
  for (const double v : {0.35, 0.5, 1.0}) {
    const double tick_travel = v * 0.02;
    double worst_on = 0.0, worst_off = 0.0;
    for (int k = 0; k < 8; ++k) {
      const double phase = tick_travel * k / 8.0;
      const Edges e = constant_speed_pass(v, 0.02, phase);
      ASSERT_EQ(e.ons, 1) << v;
      // never early, and late by less than one tick of travel (the sampling of the crossing)
      EXPECT_GE(e.paint_on_err, -1e-9) << "v=" << v << " phase=" << phase;
      EXPECT_LT(e.paint_on_err, tick_travel + 1e-9) << "v=" << v << " phase=" << phase;
      EXPECT_GE(e.paint_off_err, -1e-9) << "v=" << v << " phase=" << phase;
      EXPECT_LT(e.paint_off_err, tick_travel + 1e-9) << "v=" << v << " phase=" << phase;
      worst_on = std::max(worst_on, e.paint_on_err);
      worst_off = std::max(worst_off, e.paint_off_err);
    }
    if (v <= 0.5) {
      EXPECT_LE(worst_on, 0.01) << v;
      EXPECT_LE(worst_off, 0.01) << v;
    }
  }
  // Without the compensation the debounce alone would put both edges 2 ticks (4 cm at 1 m/s) late:
  // the term is load-bearing, not absorbed by the tolerance above.
  const Edges e = constant_speed_pass(1.0, 0.02, 0.0);
  EXPECT_LT(e.paint_on_err, 0.04 - 0.005);
  EXPECT_LT(e.paint_off_err, 0.04 - 0.005);
}

TEST(Controller, DebounceLeadUsesTheRealTickPeriod) {
  // At 100 Hz the same 3 samples are 20 ms, not 40 ms; and 1 m/s stays within 1 cm at every phase.
  for (int k = 0; k < 8; ++k) {
    const double phase = 0.01 * k / 8.0;
    const Edges e = constant_speed_pass(1.0, 0.01, phase);
    ASSERT_EQ(e.ons, 1);
    EXPECT_GE(e.paint_on_err, -1e-9) << phase;
    EXPECT_LE(e.paint_on_err, 0.01) << phase;
    EXPECT_GE(e.paint_off_err, -1e-9) << phase;
    EXPECT_LE(e.paint_off_err, 0.01) << phase;
  }
  // A longer debounce is led by its own length.
  const Edges e = constant_speed_pass(0.5, 0.02, 0.0, /*debounce_samples=*/8);
  EXPECT_GE(e.paint_on_err, -1e-9);
  EXPECT_LT(e.paint_on_err, 0.01 + 1e-9);
  EXPECT_GE(e.paint_off_err, -1e-9);
  EXPECT_LT(e.paint_off_err, 0.01 + 1e-9);
  ParamSet ps;
  EXPECT_THROW(SprayController(&ps, 0.0), std::invalid_argument);
  EXPECT_THROW(SprayController(&ps, NAN), std::invalid_argument);
}

TEST(Controller, SafetyOffBypassesTheDebounceAndTheLead) {
  // Even with a long debounce, a safety refusal sends OFF in the very tick it is seen.
  Rig r(true, 0.02, {{"debounce_samples", 10.0, ""}});
  double n = 0.0;
  for (; n < 5.0; n += 0.02) r.step(n, 1.0);
  ASSERT_TRUE(r.c->status(r.t).spraying);
  int offs = r.offs;
  r.step(n += 0.02, 1.0, /*estop=*/true);
  EXPECT_EQ(r.offs, offs + 1);
  EXPECT_FALSE(r.c->status(r.t).spraying);
  EXPECT_FALSE(r.c->lease(r.t).allow_on);

  Rig d(true, 0.02, {{"debounce_samples", 10.0, ""}});
  for (n = 0.0; n < 5.0; n += 0.02) d.step(n, 1.0);
  ASSERT_TRUE(d.c->status(d.t).spraying);
  offs = d.offs;
  d.step(n += 0.02, 1.0, false, /*armed=*/false);
  EXPECT_EQ(d.offs, offs + 1);
  EXPECT_FALSE(d.c->status(d.t).spraying);
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
  SprayController c(&params, 0.02);
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
  w.c->note_rtk(RtkSnapshot{6, 0.02, true}, w.t);
  w.c->note_rpp(static_cast<uint8_t>(RppState::Tracking), 1, 0, 0.0, 1.0, true, w.t);
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
  r.c->note_rtk(RtkSnapshot{5, 0.4, true}, r.t);  // float with poor accuracy
  r.run_cmd(r.c->tick(r.t));
  EXPECT_FALSE(r.c->status(r.t).spraying);
  EXPECT_FALSE(r.c->lease(r.t).allow_on);
}

TEST(Controller, StaleCorrectionsCutValveAndRecoveryDoesNotBypassOtherGates) {
  Rig r;
  double n = 0.0;
  for (; n < 4.0; n += 0.007) r.step(n);
  ASSERT_TRUE(r.c->status(r.t).spraying);

  r.t += 0.02;
  r.world(n, true);
  r.c->note_rtk(RtkSnapshot{6, 0.02, false}, r.t);
  r.run_cmd(r.c->tick(r.t));
  EXPECT_FALSE(r.c->status(r.t).spraying);
  EXPECT_FALSE(r.c->lease(r.t).allow_on);
  EXPECT_EQ(r.c->status(r.t).safety_reason, "RTK corrections stale");

  // Fresh corrections restore only the RTK gate; invalid heading evidence still blocks ON.
  r.t += 0.02;
  r.world(n, true);
  r.c->note_rpp(static_cast<uint8_t>(RppState::Tracking), 1, 0, 0.0, 1.0, false, r.t);
  r.run_cmd(r.c->tick(r.t));
  EXPECT_FALSE(r.c->lease(r.t).allow_on);
  EXPECT_NE(r.c->status(r.t).safety_reason.find("heading"), std::string::npos);

  // Fresh RTK and valid heading still do not spray on a TRANSIT section of the conditioned path.
  r.t += 0.02;
  r.world(0.5, true);
  r.run_cmd(r.c->tick(r.t));
  EXPECT_FALSE(r.c->status(r.t).geometry_desired);
  EXPECT_FALSE(r.c->status(r.t).spraying);

  // Restoring RPP evidence does not bypass mission ownership either.
  r.t += 0.02;
  r.world(n, true);
  r.c->set_mission(false, 1);
  r.run_cmd(r.c->tick(r.t));
  EXPECT_FALSE(r.c->lease(r.t).allow_on);
  EXPECT_EQ(r.c->status(r.t).safety_reason, "mission not running");
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

// Review C1 / fix plan A1: the valve follows the mission and RPP, not a permanent latch.
TEST(Controller, MissionOrRppLeavingAMarkingStateClosesTheValveAtOnce) {
  struct Case {
    const char* name;
    void (*apply)(Rig&);
    const char* reason;
  };
  const Case cases[] = {
      {"mission paused/aborted/completed/error", [](Rig& r) { r.mission_running = false; },
       "mission not running"},
      {"rpp ERROR", [](Rig& r) { r.rpp_override = RppState::Error; }, "rpp not marking"},
      {"rpp COMPLETE", [](Rig& r) { r.rpp_override = RppState::Complete; }, "rpp not marking"},
      {"rpp LOADED", [](Rig& r) { r.rpp_override = RppState::Loaded; }, "rpp not marking"},
      {"rpp IDLE", [](Rig& r) { r.rpp_override = RppState::Idle; }, "rpp not marking"},
  };
  for (const auto& k : cases) {
    Rig r;
    double n = 0.0;
    for (; n < 4.0; n += 0.007) r.step(n);
    ASSERT_TRUE(r.c->status(r.t).spraying) << k.name;
    const int offs = r.offs;
    k.apply(r);
    r.step(n);
    EXPECT_FALSE(r.c->lease(r.t).allow_on) << k.name;
    EXPECT_FALSE(r.c->status(r.t).spraying) << k.name;  // OFF commanded and acked in one tick
    EXPECT_GT(r.offs, offs) << k.name;
    EXPECT_EQ(r.c->status(r.t).safety_reason, k.reason) << k.name;
  }
}

TEST(Controller, RppSilenceBeyondItsTimeoutClosesTheValve) {
  Rig r;
  double n = 0.0;
  for (; n < 4.0; n += 0.007) r.step(n);
  ASSERT_TRUE(r.c->status(r.t).spraying);
  r.rpp_alive = false;                              // RPP process killed: no more RppStatus
  for (int i = 0; i < 24; ++i) r.step(n += 0.007);  // 0.48 s: still within rpp_timeout_s 0.5
  EXPECT_TRUE(r.c->status(r.t).spraying);
  for (int i = 0; i < 2; ++i) r.step(n += 0.007);
  EXPECT_FALSE(r.c->status(r.t).spraying);
  EXPECT_FALSE(r.c->lease(r.t).allow_on);
  EXPECT_EQ(r.c->status(r.t).safety_reason, "rpp stale");
}

TEST(Controller, StoppingKeepsTheMarkAndResumeNeedsFreshTracking) {
  Rig r;
  double n = 0.0;
  for (; n < 4.0; n += 0.007) r.step(n);
  ASSERT_TRUE(r.c->status(r.t).spraying);
  r.rpp_override = RppState::Stopping;  // corner stop lays the last ~2 cm of the leg
  for (int i = 0; i < 5; ++i) r.step(n += 0.004);
  EXPECT_TRUE(r.c->status(r.t).spraying);
  // pause, then resume while RPP is still STOPPING: the old tracking evidence is gone
  r.mission_running = false;
  r.step(n);
  ASSERT_FALSE(r.c->status(r.t).spraying);
  r.mission_running = true;
  for (int i = 0; i < 10; ++i) r.step(n += 0.007);
  EXPECT_FALSE(r.c->status(r.t).spraying);
  EXPECT_EQ(r.c->status(r.t).safety_reason, "awaiting tracking");
  r.rpp_override.reset();  // TRACKING again
  for (int i = 0; i < 10; ++i) r.step(n += 0.007);
  EXPECT_TRUE(r.c->status(r.t).spraying);
}

TEST(Controller, TrackingOfAnotherMissionIsNotEvidence) {
  Rig r(/*tracking0=*/false);
  double n = 0.0;
  for (; n < 1.0; n += 0.007) r.step(n);
  r.c->note_rpp(static_cast<uint8_t>(RppState::Tracking), 2, 0, 0.0, 1.0, true,
                r.t);                     // stale RPP of mission 2
  for (; n < 4.0; n += 0.007) r.step(n);  // RPP of mission 1 never TRACKING
  EXPECT_FALSE(r.c->status(r.t).spraying);
  EXPECT_EQ(r.c->status(r.t).safety_reason, "awaiting tracking");
}

// SP-004: min_spray_speed_mps. No paint blob while (nearly) stationary on a MARK leg, but no new gap
// at a line start or in the terminal creep.
namespace {
// 0..5 m north: TRANSIT 0..2, MARK 2..5; the path ends on MARK (synthetic terminal boundary at 5.0).
std::shared_ptr<const PathModel> ends_on_mark() {
  std::vector<double> n, e;
  std::vector<bool> f;
  for (int i = 0; i <= 5; ++i) {
    n.push_back(i);
    e.push_back(0.0);
    f.push_back(i >= 2);
  }
  auto m = std::make_shared<PathModel>();
  EXPECT_TRUE(build_path_model(n, e, f, m.get()));
  return m;
}
}  // namespace

TEST(Controller, StationaryOnMarkAfterDrivingItClosesTheValve) {
  Rig r;
  double n = 0.0;
  for (; n < 4.0; n += 0.007) r.step(n);
  ASSERT_TRUE(r.c->status(r.t).spraying);
  const int offs = r.offs;
  for (int i = 0; i < 5; ++i) r.step(n, 0.0);  // still TRACKING, but stalled on the mark
  EXPECT_FALSE(r.c->status(r.t).spraying);
  EXPECT_EQ(r.offs, offs + 1);
  EXPECT_TRUE(r.c->status(r.t).geometry_desired);  // a process cut, not a geometry or safety one
  EXPECT_TRUE(r.c->status(r.t).safety_ok);
  for (int i = 0; i < 5; ++i) r.step(n += 0.007);  // moving again: paints again
  EXPECT_TRUE(r.c->status(r.t).spraying);
}

TEST(Controller, CornerStoppingClosesTheValveBeforeTheRoverStops) {
  for (const bool gate_on : {true, false}) {
    Rig r(true, 0.02, {{"min_spray_speed_mps", gate_on ? 0.05 : 0.0, ""}});
    double n = 0.0;
    for (; n < 4.0; n += 0.007) r.step(n);
    ASSERT_TRUE(r.c->status(r.t).spraying);
    r.rpp_override = RppState::Stopping;  // corner stop: brake at 0.5 m/s^2 to standstill
    double v = 0.35, v_at_off = -1.0;
    const int offs = r.offs;
    while (v > 0.0) {
      v = std::max(0.0, v - 0.5 * 0.02);
      n += v * 0.02;
      r.step(n, v);
      if (v_at_off < 0.0 && r.offs > offs) v_at_off = v;
    }
    for (int i = 0; i < 10; ++i) r.step(n, 0.0);  // standing at the corner, not yet pivoting
    if (gate_on) {
      ASSERT_GT(v_at_off, 0.0);  // the OFF went out while the rover was still rolling
      EXPECT_LT(v_at_off, 0.05);
      EXPECT_FALSE(r.c->status(r.t).spraying);
    } else {
      EXPECT_TRUE(r.c->status(r.t).spraying);  // the blob the gate removes
    }
  }
}

TEST(Controller, TerminalCreepBelowMinSpeedHasNoGap) {
  Rig r;
  r.c->load_path(ends_on_mark());
  double n = 0.0;
  for (; n < 4.6; n += 0.007) r.step(n);
  ASSERT_TRUE(r.c->status(r.t).spraying);
  r.rpp_override = RppState::Creeping;  // endpoint precise stop at 0.03 m/s (< min 0.05)
  const int offs = r.offs;
  double off_at = -1.0;
  LeadEvent off_event = LeadEvent::None;
  for (; n < 5.0; n += 0.03 * 0.02) {
    r.step(n, 0.03);
    if (off_at < 0.0 && r.offs > offs) {
      off_at = n;
      off_event = r.c->status(r.t).event;
    }
  }
  // The only OFF is the terminal shutoff within terminal_off_epsilon_m (0.05) of the final station.
  ASSERT_GT(off_at, 0.0);
  EXPECT_GE(off_at, 5.0 - 0.05 - 1e-9);
  EXPECT_LT(off_at, 5.0 - 0.05 + 0.002);
  EXPECT_EQ(off_event, LeadEvent::TerminalOff);
}

TEST(Controller, StandingStartOnAMarkLegStillOpensAtOnce) {
  // After a corner the next leg starts from standstill on MARK: the valve opens as before (the
  // opening delay elapses while the rover pulls away) instead of waiting for min_spray_speed_mps.
  Rig r;
  for (int i = 0; i < 5; ++i) r.step(3.0, 0.0);
  EXPECT_TRUE(r.c->status(r.t).spraying);
  // Same after a corner stop: STOPPING at standstill is OFF, the TRACKING pull-away opens at once.
  r.rpp_override = RppState::Stopping;
  for (int i = 0; i < 5; ++i) r.step(3.0, 0.0);
  ASSERT_FALSE(r.c->status(r.t).spraying);
  r.rpp_override.reset();
  double v = 0.0, n = 3.0;
  for (int i = 0; i < 4; ++i) {  // accelerate at 0.5 m/s^2: 0.01 .. 0.04 m/s, all below 0.05
    v += 0.5 * 0.02;
    n += v * 0.02;
    r.step(n, v);
  }
  EXPECT_LT(v, 0.05);
  EXPECT_TRUE(r.c->status(r.t).spraying);
}
