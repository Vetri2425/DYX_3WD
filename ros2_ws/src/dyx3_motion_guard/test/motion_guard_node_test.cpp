// In-process tests of the guard node: a fake world publishes every input and records every output,
// with an injected clock and a private DDS domain.
#include "dyx3_motion_guard/motion_guard_node.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include "dds_test_support.hpp"

using namespace dyx3_motion_guard;
using namespace std::chrono_literals;

namespace {
constexpr float NaN = std::numeric_limits<float>::quiet_NaN();
using dyx3_interfaces::msg::MotionSetpoint;

// The whole suite runs twice: motion_guard_node_test in timer mode (event_driven=false: tick()
// calls step(), the timer-mode regression) and motion_guard_node_event_test in event-driven mode
// (the production default: each RPP command is decided in its callback, tick() only calls the
// watchdog). Tests about one mode set event_driven explicitly.
#ifdef DYX3_TEST_EVENT_DRIVEN
constexpr bool kSuiteEventDriven = true;
#else
constexpr bool kSuiteEventDriven = false;
#endif

struct Rig {
  std::shared_ptr<rclcpp::Context> ctx;
  double now{50.0};
  std::shared_ptr<MotionGuardNode> guard;
  std::shared_ptr<rclcpp::Node> world;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
  rclcpp::Publisher<MotionSetpoint>::SharedPtr p_cmd;
  rclcpp::Publisher<dyx3_interfaces::msg::MissionState>::SharedPtr p_mission;
  rclcpp::Publisher<dyx3_interfaces::msg::VehicleState>::SharedPtr p_veh;
  rclcpp::Publisher<dyx3_interfaces::msg::EstimatorHealth>::SharedPtr p_est;
  rclcpp::Publisher<dyx3_interfaces::msg::RtkStatus>::SharedPtr p_rtk;
  rclcpp::Publisher<dyx3_interfaces::msg::Px4LinkStatus>::SharedPtr p_link;
  rclcpp::Client<dyx3_interfaces::srv::SetEmergencyStop>::SharedPtr cli_estop;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> keep;
  MotionSetpoint last_out;
  std::vector<MotionSetpoint> outs;
  std::vector<dyx3_interfaces::msg::MotionSetpointStatus> statuses;
  std::vector<dyx3_interfaces::msg::SafetyGateStatus> gates;
  std::vector<dyx3_interfaces::msg::EmergencyStopState> estops;
  dyx3_interfaces::msg::MotionSetpointStatus last_status;
  dyx3_interfaces::msg::SafetyGateStatus last_gate;
  dyx3_interfaces::msg::EmergencyStopState last_estop;
  uint64_t seq{0};
  builtin_interfaces::msg::Time rpp_stamp{}, veh_stamp{};  // IF-003 pose sample stamps
  bool event{false};                                       // the node's mode (event_driven)
  bool veh_ok{true}, rtk_ok{true}, link_ok{true}, est_ok{true}, mission_running{true};
  bool veh_armed_offboard{true}, veh_global_ref{true};  // pre-arm gate inputs (0.15.0)
  bool veh_preflight_pass{true};                        // pre-arm gate input
  int veh_nav{-1};  // >= 0: nav_state override (otherwise from veh_armed_offboard)
  bool veh_rc_valid{false}, veh_rc_ok{false};            // RC link (0.17.0)
  float veh_vn{0.0F}, veh_ve{0.0F}, veh_yaw_rate{0.0F};  // measured motion (actuator plausibility)

  explicit Rig(const std::vector<rclcpp::Parameter>& params = {}) {
    ctx = std::make_shared<rclcpp::Context>();
    dyx3_test::init_isolated(ctx);
    rclcpp::NodeOptions no;
    no.context(ctx);
    no.append_parameter_override("max_reverse_speed_mps", 0.3);
    bool mode_given = false;
    for (const auto& p : params) {
      no.append_parameter_override(p.get_name(), p.get_parameter_value());
      mode_given = mode_given || p.get_name() == "event_driven";
    }
    if (!mode_given) no.append_parameter_override("event_driven", kSuiteEventDriven);
    guard = std::make_shared<MotionGuardNode>(no, [this]() { return now; }, false);
    event = guard->event_driven();
    rclcpp::NodeOptions wo;
    wo.context(ctx);
    world = std::make_shared<rclcpp::Node>("world", wo);
    rclcpp::ExecutorOptions eo;
    eo.context = ctx;
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(eo);
    exec->add_node(guard);
    exec->add_node(world);
    const auto r1 = rclcpp::QoS(1).reliable();
    p_cmd = world->create_publisher<MotionSetpoint>("/dyx3/rpp/motion_setpoint", r1);
    p_mission =
        world->create_publisher<dyx3_interfaces::msg::MissionState>("/dyx3/mission/state", r1);
    p_veh = world->create_publisher<dyx3_interfaces::msg::VehicleState>("/dyx3/vehicle_state", r1);
    p_est = world->create_publisher<dyx3_interfaces::msg::EstimatorHealth>("/dyx3/estimator_health",
                                                                           r1);
    p_rtk = world->create_publisher<dyx3_interfaces::msg::RtkStatus>("/dyx3/rtk_status", r1);
    p_link =
        world->create_publisher<dyx3_interfaces::msg::Px4LinkStatus>("/dyx3/px4_link/status", r1);
    keep.push_back(world->create_subscription<MotionSetpoint>(
        "/dyx3/motion_guard/command", r1, [this](MotionSetpoint::ConstSharedPtr m) {
          last_out = *m;
          outs.push_back(*m);
        }));
    keep.push_back(world->create_subscription<dyx3_interfaces::msg::MotionSetpointStatus>(
        "/dyx3/motion_guard/status", rclcpp::QoS(10).reliable(),
        [this](dyx3_interfaces::msg::MotionSetpointStatus::ConstSharedPtr m) {
          last_status = *m;
          statuses.push_back(*m);
        }));
    keep.push_back(world->create_subscription<dyx3_interfaces::msg::SafetyGateStatus>(
        "/dyx3/safety_gate", r1, [this](dyx3_interfaces::msg::SafetyGateStatus::ConstSharedPtr m) {
          last_gate = *m;
          gates.push_back(*m);
        }));
    keep.push_back(world->create_subscription<dyx3_interfaces::msg::EmergencyStopState>(
        "/dyx3/emergency_stop_state", r1,
        [this](dyx3_interfaces::msg::EmergencyStopState::ConstSharedPtr m) {
          last_estop = *m;
          estops.push_back(*m);
        }));
    cli_estop = world->create_client<dyx3_interfaces::srv::SetEmergencyStop>(
        "/dyx3/motion_guard/set_emergency_stop");
    const auto end = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < end) {
      exec->spin_some(5ms);
      if (guard->count_subscribers("/dyx3/motion_guard/command") > 0 &&
          world->count_subscribers("/dyx3/rpp/motion_setpoint") > 0 &&
          world->count_subscribers("/dyx3/mission/state") > 0 &&
          world->count_subscribers("/dyx3/vehicle_state") > 0 &&
          world->count_subscribers("/dyx3/estimator_health") > 0 &&
          world->count_subscribers("/dyx3/rtk_status") > 0 &&
          world->count_subscribers("/dyx3/px4_link/status") > 0 && cli_estop->service_is_ready()) {
        // every later deliver() relies on synchronous delivery: prove it
        EXPECT_TRUE(dyx3_test::delivery_is_synchronous(ctx));
        return;
      }
    }
    ADD_FAILURE() << "DDS discovery did not complete";
  }
  ~Rig() {
    exec.reset();
    keep.clear();
    guard.reset();
    world.reset();
    ctx->shutdown("test done");
  }

  // Delivers until `done` holds or the wall-clock deadline passes. The injected clock does not
  // move here; the deadline only bounds a genuinely missing message.
  template <typename Done>
  bool pump_until(Done done, int timeout_ms = 3000) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < end) {
      deliver();
      if (done()) return true;
      exec->spin_once(1ms);  // blocks only while nothing is ready
    }
    deliver();
    return done();
  }
  // Delivers everything published so far and runs every callback it causes (and the ones those
  // cause), then returns: nothing is left in flight, so a negative check after it is exact.
  void deliver() { dyx3_test::drain(*exec); }
  void publish_world() {
    if (mission_running) {
      dyx3_interfaces::msg::MissionState m;
      m.state = 3;
      p_mission->publish(m);
    }
    if (veh_ok) {
      dyx3_interfaces::msg::VehicleState v;
      v.arming_state = veh_armed_offboard ? 2 : 1;
      v.nav_state = veh_nav >= 0 ? static_cast<uint8_t>(veh_nav) : (veh_armed_offboard ? 14 : 4);
      v.rc_link_valid = veh_rc_valid;
      v.rc_link_ok = veh_rc_ok;
      v.velocity_north_mps = veh_vn;
      v.velocity_east_mps = veh_ve;
      v.yaw_rate_radps = veh_yaw_rate;
      v.global_reference_valid = veh_global_ref;
      v.preflight_checks_pass = veh_preflight_pass;
      v.position_valid = v.velocity_valid = v.attitude_valid = true;
      v.px4_sample_stamp = veh_stamp;
      p_veh->publish(v);
    }
    if (est_ok) {
      dyx3_interfaces::msg::EstimatorHealth e;
      e.flags_valid = true;
      e.gnss_yaw_fusion_intended = true;
      p_est->publish(e);
    }
    if (rtk_ok) {
      dyx3_interfaces::msg::RtkStatus r;
      r.fix_type = 6;
      r.corrections_fresh = true;
      r.horizontal_accuracy_m = 0.02F;
      p_rtk->publish(r);
    }
    if (link_ok) {
      dyx3_interfaces::msg::Px4LinkStatus l;
      l.session_alive = true;
      l.handshake_ok = true;
      p_link->publish(l);
    }
  }
  void rpp(uint8_t mode, float v, float yaw, float rate, bool valid = true) {
    MotionSetpoint m;
    m.seq = ++seq;
    m.mode = mode;
    m.speed_body_x = v;
    m.yaw_setpoint = yaw;
    m.yaw_rate_setpoint = rate;
    m.valid = valid;
    m.source_pose_sample_stamp = rpp_stamp;
    p_cmd->publish(m);
  }
  // Calls the E-stop service and pumps (no guard step, no world traffic) until the reply arrives.
  // Returns whether the guard accepted the request.
  bool call_estop(bool asserted, const char* source) {
    auto req = std::make_shared<dyx3_interfaces::srv::SetEmergencyStop::Request>();
    req->asserted = asserted;
    req->source = source;
    auto fut = cli_estop->async_send_request(req);
    for (int i = 0; i < 500 && fut.wait_for(0ms) != std::future_status::ready; ++i)
      exec->spin_some(2ms);
    if (fut.wait_for(0ms) != std::future_status::ready) return false;
    return fut.get()->accepted;
  }
  // One cycle: world publishes, deliver, guard decides, deliver outputs.
  void tick(double dt = 0.02, bool with_rpp = true, uint8_t mode = 2, float v = 0.3F,
            float yaw = NaN, float rate = 0.1F) {
    now += dt;
    publish_world();
    if (with_rpp) rpp(mode, v, yaw, rate);
    deliver();
    if (event) {
      guard->on_watchdog(now);  // a no-op when the RPP command was decided on arrival
    } else {
      guard->step(now);
    }
    deliver();
  }
  void run(double seconds, bool with_rpp = true) {
    for (double t = 0; t < seconds; t += 0.02) tick(0.02, with_rpp);
  }
};

}  // namespace

TEST(MotionGuardNode, RejectsBadParameters) {
  auto ctx = std::make_shared<rclcpp::Context>();
  dyx3_test::init_isolated(ctx);
  rclcpp::NodeOptions no;
  no.context(ctx);
  no.append_parameter_override("max_yaw_rate_radps", -1.0);
  EXPECT_THROW(MotionGuardNode(no, nullptr, false), std::invalid_argument);
  rclcpp::NodeOptions no2;
  no2.context(ctx);
  no2.append_parameter_override("rtk_min_fix_type", 3);
  EXPECT_THROW(MotionGuardNode(no2, nullptr, false), std::invalid_argument);
  for (const auto& bad :
       {rclcpp::Parameter("stall_time_s", 0.0), rclcpp::Parameter("stall_yaw_rate_radps", -0.4),
        rclcpp::Parameter("stall_measured_speed_mps", 0.2),  // >= stall_speed_mps
        rclcpp::Parameter("stall_measured_yaw_rate_radps", 0.4)}) {
    rclcpp::NodeOptions o;
    o.context(ctx);
    o.append_parameter_override(bad.get_name(), bad.get_parameter_value());
    EXPECT_THROW(MotionGuardNode(o, nullptr, false), std::invalid_argument) << bad.get_name();
  }
  ctx->shutdown("test done");
}

TEST(MotionGuardNode, RuntimeParameterChangesCannotMisstateEffectiveLimits) {
  Rig r;
  for (const auto& change :
       {rclcpp::Parameter("max_forward_speed_mps", 0.2),
        rclcpp::Parameter("max_reverse_speed_mps", 0.01),
        rclcpp::Parameter("max_yaw_rate_radps", 0.1), rclcpp::Parameter("rtk_max_hrms_m", -1.0),
        rclcpp::Parameter("command_max_age_s", 2.0), rclcpp::Parameter("publish_rate_hz", 100.0),
        rclcpp::Parameter("require_actuator_plausibility", false),
        rclcpp::Parameter("stall_time_s", 5.0), rclcpp::Parameter("require_rc_link", true)}) {
    const auto result = r.guard->set_parameters_atomically({change});
    EXPECT_FALSE(result.successful) << change.get_name();
    EXPECT_NE(result.reason.find("startup-only"), std::string::npos);
  }
  EXPECT_DOUBLE_EQ(r.guard->get_parameter("max_forward_speed_mps").as_double(), 1.0);
  EXPECT_DOUBLE_EQ(r.guard->get_parameter("max_reverse_speed_mps").as_double(), 0.3);
  EXPECT_DOUBLE_EQ(r.guard->get_parameter("max_yaw_rate_radps").as_double(), 0.45);
  const auto batch =
      r.guard->set_parameters_atomically({rclcpp::Parameter("max_forward_speed_mps", 0.2),
                                          rclcpp::Parameter("max_yaw_rate_radps", 0.1)});
  EXPECT_FALSE(batch.successful);

  r.run(0.2);
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
  EXPECT_FLOAT_EQ(r.last_out.speed_body_x, 0.3F);
  r.tick(0.02, true, MotionSetpoint::MODE_TRACK_RATE, -0.08F, NaN, 0.1F);
  EXPECT_FLOAT_EQ(r.last_out.speed_body_x, -0.08F);
  r.tick(0.02, true, MotionSetpoint::MODE_TRACK_RATE, 2.0F, NaN, 0.8F);
  EXPECT_FLOAT_EQ(r.last_out.speed_body_x, 1.0F);
  EXPECT_FLOAT_EQ(r.last_out.yaw_rate_setpoint, 0.45F);
  r.rtk_ok = false;
  r.now += 0.6;
  r.tick();
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_STOP);
  EXPECT_EQ(r.last_out.speed_body_x, 0.0F);
}

TEST(MotionGuardNode, PublishesAtFixedRateEvenWhenRppIsSilent) {
  Rig r;
  r.run(0.5, /*with_rpp=*/false);
  ASSERT_FALSE(r.outs.empty());
  const size_t before = r.outs.size();
  r.tick(0.02, false);
  EXPECT_EQ(r.outs.size(), before + 1);  // one command per tick, no RPP needed
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_STOP);
  EXPECT_EQ(r.last_out.speed_body_x, 0.0F);
  EXPECT_TRUE(std::isnan(r.last_out.yaw_setpoint));
  EXPECT_TRUE(r.last_out.valid);
  EXPECT_EQ(r.last_status.reason_code, dyx3_interfaces::msg::MotionSetpointStatus::REASON_STALE);
  // the guard's own seq is strictly increasing
  for (size_t i = 1; i < r.outs.size(); ++i) EXPECT_GT(r.outs[i].seq, r.outs[i - 1].seq);
}

TEST(MotionGuardNode, ForwardsAfterSessionAcceptanceWhenAllGatesPass) {
  Rig r;
  r.tick();  // first command of a new session: not yet accepted
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_STOP);
  EXPECT_EQ(r.last_status.reason_code, dyx3_interfaces::msg::MotionSetpointStatus::REASON_SEQUENCE);
  r.run(0.2);
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
  EXPECT_FLOAT_EQ(r.last_out.speed_body_x, 0.3F);  // B2: exact RPP speed, no guard ramp
  EXPECT_TRUE(r.last_status.accepted);
  EXPECT_TRUE(r.last_gate.ok);
  EXPECT_FALSE(r.last_estop.asserted);

  // B1 + B2 together: the active-brake command is inside the hard reverse envelope and leaves the
  // production guard unchanged.
  r.tick(0.02, true, MotionSetpoint::MODE_TRACK_RATE, -0.08F, NaN, 0.0F);
  EXPECT_FLOAT_EQ(r.last_out.speed_body_x, -0.08F);
}

// Owner decision 2026-10-10: the tablet heartbeat is not a gate (prototype behaviour). The guard
// does not even subscribe to it: a silent tablet changes nothing, running or before a start.
TEST(MotionGuardNode, TheOperatorLinkIsNotAGate) {
  Rig r;
  r.run(0.3);
  ASSERT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
  EXPECT_EQ(r.world->count_publishers("/dyx3/operator_link"), 0U);
  EXPECT_EQ(r.guard->count_subscribers("/dyx3/operator_link"), 0U);
  EXPECT_TRUE(r.last_gate.ok);
}

TEST(MotionGuardNode, EachInputLossZeroesWithItsReasonAndRecovers) {
  struct Case {
    const char* name;
    bool Rig::* flag;
    uint8_t reason;
  };
  using S = dyx3_interfaces::msg::MotionSetpointStatus;
  const Case cases[] = {{"mission", &Rig::mission_running, S::REASON_MISSION_GATE},
                        {"vehicle", &Rig::veh_ok, S::REASON_ARMING_GATE},
                        {"rtk", &Rig::rtk_ok, S::REASON_RTK_GATE},
                        {"px4 link", &Rig::link_ok, S::REASON_PX4_LINK_UNHEALTHY},
                        {"estimator", &Rig::est_ok, S::REASON_HEADING_UNHEALTHY}};
  for (const auto& c : cases) {
    Rig r;
    r.run(0.3);
    ASSERT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE) << c.name;
    r.*(c.flag) = false;  // the source goes silent: its data ages out (0.5 s)
    r.run(0.8);
    EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_STOP) << c.name;
    EXPECT_EQ(r.last_out.speed_body_x, 0.0F) << c.name;
    EXPECT_EQ(r.last_status.reason_code, c.reason) << c.name;
    EXPECT_FALSE(r.last_gate.ok && std::string(c.name) != "mission") << c.name;
    r.*(c.flag) = true;
    r.run(0.3);
    EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE)
        << c.name << " must recover without a latch";
  }
}

TEST(MotionGuardNode, EmergencyStopLatchesUntilExplicitlyCleared) {
  Rig r;
  r.run(0.3);
  ASSERT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
  auto req = std::make_shared<dyx3_interfaces::srv::SetEmergencyStop::Request>();
  req->asserted = true;
  req->source = "tablet";
  auto fut = r.cli_estop->async_send_request(req);
  for (int i = 0; i < 100 && fut.wait_for(0ms) != std::future_status::ready; ++i) r.tick();
  ASSERT_EQ(fut.wait_for(0ms), std::future_status::ready);
  EXPECT_TRUE(fut.get()->accepted);
  r.run(0.3);
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_STOP);
  EXPECT_EQ(r.last_status.reason_code, dyx3_interfaces::msg::MotionSetpointStatus::REASON_ESTOP);
  EXPECT_TRUE(r.last_estop.asserted);
  EXPECT_EQ(r.last_estop.source, "tablet");
  EXPECT_FALSE(r.last_gate.ok);
  r.run(2.0);  // everything else healthy for a long time: still latched
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_STOP);
  // invalid source is refused
  req = std::make_shared<dyx3_interfaces::srv::SetEmergencyStop::Request>();
  req->asserted = false;
  req->source = "stranger";
  auto bad = r.cli_estop->async_send_request(req);
  for (int i = 0; i < 100 && bad.wait_for(0ms) != std::future_status::ready; ++i) r.tick();
  ASSERT_EQ(bad.wait_for(0ms), std::future_status::ready);
  auto resp = bad.get();
  EXPECT_FALSE(resp->accepted);
  EXPECT_EQ(resp->reason_code,
            dyx3_interfaces::srv::SetEmergencyStop::Response::REASON_INVALID_SOURCE);
  r.run(0.2);
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_STOP);
  req->source = "backend";
  auto clr = r.cli_estop->async_send_request(req);
  for (int i = 0; i < 100 && clr.wait_for(0ms) != std::future_status::ready; ++i) r.tick();
  ASSERT_EQ(clr.wait_for(0ms), std::future_status::ready);
  EXPECT_TRUE(clr.get()->accepted);
  r.run(0.3);
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
}

// MG-007: the latch change is published at once, so an assert and a clear less than one 100 ms
// gate period apart are both visible to the mission and the spray node.
TEST(MotionGuardNode, EmergencyStopAssertAndClearWithinOneGatePeriodAreBothPublished) {
  Rig r;
  r.run(0.3);
  ASSERT_TRUE(r.last_gate.ok);
  r.gates.clear();
  r.estops.clear();

  ASSERT_TRUE(r.call_estop(true, "tablet"));
  r.deliver();
  ASSERT_FALSE(r.gates.empty()) << "assert must publish the gate state without waiting for 10 Hz";
  ASSERT_FALSE(r.estops.empty());
  EXPECT_FALSE(r.gates.back().ok);
  EXPECT_EQ(r.gates.back().reason_code, dyx3_interfaces::msg::MotionSetpointStatus::REASON_ESTOP);
  EXPECT_TRUE(r.estops.back().asserted);
  EXPECT_EQ(r.estops.back().source, "tablet");

  r.now += 0.03;  // 30 ms later: still inside the same 100 ms gate period
  ASSERT_TRUE(r.call_estop(false, "tablet"));
  r.deliver();
  ASSERT_GE(r.gates.size(), 2U) << "the clear must be published as well";
  ASSERT_GE(r.estops.size(), 2U);
  EXPECT_TRUE(r.gates.back().ok);
  EXPECT_FALSE(r.estops.back().asserted);

  // Exactly the sequence a consumer must have seen: ESTOP-failing gate first, then ok.
  bool saw_estop = false, ok_after = false;
  for (const auto& g : r.gates) {
    if (!g.ok && g.reason_code == dyx3_interfaces::msg::MotionSetpointStatus::REASON_ESTOP)
      saw_estop = true;
    else if (saw_estop && g.ok)
      ok_after = true;
  }
  EXPECT_TRUE(saw_estop);
  EXPECT_TRUE(ok_after);
}

// MG-003: the shutdown path publishes a canonical STOP even while a motion command is being
// forwarded and every gate passes.
TEST(MotionGuardNode, ShutdownStopPublishesCanonicalStopWhileMoving) {
  Rig r;
  r.run(0.3);
  ASSERT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
  const uint64_t seq_before = r.last_out.seq;
  const size_t n_before = r.outs.size();
  for (int i = 0; i < 5; ++i) {
    r.guard->shutdown_stop();
    r.deliver();
  }
  ASSERT_EQ(r.outs.size(), n_before + 5);
  for (size_t i = n_before; i < r.outs.size(); ++i) {
    const auto& m = r.outs[i];
    EXPECT_EQ(m.mode, MotionSetpoint::MODE_STOP);
    EXPECT_EQ(m.speed_body_x, 0.0F);
    EXPECT_TRUE(std::isnan(m.yaw_setpoint));
    EXPECT_EQ(m.yaw_rate_setpoint, 0.0F);
    EXPECT_TRUE(m.valid);
    EXPECT_GT(m.seq, i == n_before ? seq_before : r.outs[i - 1].seq);
  }
}

// IF-003: a forwarded command keeps RPP's pose stamp; the guard's own STOP (refusal, shutdown)
// names the newest VehicleState sample it received, and zero when it never received one.
TEST(MotionGuardNode, ForwardedCommandsKeepThePoseStampOwnStopsUseTheNewestVehicleSample) {
  {
    Rig r;
    r.rpp_stamp.sec = 1791590000;
    r.rpp_stamp.nanosec = 100'000'000;
    r.veh_stamp.sec = 1791590000;
    r.veh_stamp.nanosec = 120'000'000;
    r.run(0.3);
    ASSERT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
    EXPECT_EQ(r.last_out.source_pose_sample_stamp, r.rpp_stamp);
    r.tick(0.02, true, MotionSetpoint::MODE_STOP, 0.0F, NaN, 0.0F);  // a clean STOP is forwarded
    EXPECT_TRUE(r.last_status.accepted);
    EXPECT_EQ(r.last_out.source_pose_sample_stamp, r.rpp_stamp);
    r.rtk_ok = false;  // RTK gate lost: the guard refuses and publishes its own STOP
    r.now += 0.6;
    r.tick();
    EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_STOP);
    EXPECT_FALSE(r.last_status.accepted);
    EXPECT_EQ(r.last_out.source_pose_sample_stamp, r.veh_stamp);
    r.guard->shutdown_stop();
    r.deliver();
    EXPECT_EQ(r.last_out.source_pose_sample_stamp, r.veh_stamp);
  }
  Rig never;             // after r is gone: two rigs share one DDS domain
  never.veh_ok = false;  // no VehicleState ever: the guard's STOP has no pose to name
  never.rpp_stamp.sec = 5;
  never.run(0.2);
  EXPECT_EQ(never.last_out.mode, MotionSetpoint::MODE_STOP);
  EXPECT_EQ(never.last_out.source_pose_sample_stamp.sec, 0);
  EXPECT_EQ(never.last_out.source_pose_sample_stamp.nanosec, 0U);
}

// MG-006: the reason-change log line is throttled, but every transition stays observable on the
// status topic.
TEST(MotionGuardNode, EveryReasonChangeIsOnTheStatusTopicWhateverTheLogRate) {
  using S = dyx3_interfaces::msg::MotionSetpointStatus;
  Rig r;
  r.run(0.3);
  ASSERT_EQ(r.last_status.reason_code, S::REASON_OK);
  r.statuses.clear();
  constexpr int kFlaps = 8;
  for (int i = 0; i < kFlaps; ++i) {
    for (const bool valid : {false, true}) {
      r.now += 0.02;
      r.publish_world();
      r.rpp(MotionSetpoint::MODE_TRACK_RATE, 0.3F, NaN, 0.1F, valid);
      r.deliver();
      r.guard->step(r.now);
      r.deliver();
    }
  }
  int to_invalid = 0, to_ok = 0;
  for (size_t i = 0; i < r.statuses.size(); ++i) {
    const uint8_t prev = i == 0 ? S::REASON_OK : r.statuses[i - 1].reason_code;
    if (r.statuses[i].reason_code == prev) continue;
    if (r.statuses[i].reason_code == S::REASON_INVALID_MESSAGE) ++to_invalid;
    if (r.statuses[i].reason_code == S::REASON_OK) ++to_ok;
  }
  EXPECT_EQ(to_invalid, kFlaps);
  EXPECT_EQ(to_ok, kFlaps);
}

// XR-GPX-009: E-stop immediacy. Nothing but the service call happens: no tick, no world traffic,
// no explicit step. The STOP must already be on the command topic one deliver() after the reply.
TEST(MotionGuardNode, EmergencyStopOutputsStopWithinOnePumpOfTheReply) {
  Rig r;
  r.run(0.3);
  ASSERT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
  ASSERT_GT(r.last_out.speed_body_x, 0.0F);
  const size_t n_before = r.outs.size();
  ASSERT_TRUE(r.call_estop(true, "physical"));
  r.deliver();
  ASSERT_GT(r.outs.size(), n_before) << "the service callback must publish at once";
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_STOP);
  EXPECT_EQ(r.last_out.speed_body_x, 0.0F);
  EXPECT_EQ(r.last_status.reason_code, dyx3_interfaces::msg::MotionSetpointStatus::REASON_ESTOP);
}

// XR-GPX-009: the 0.5 s freshness boundary of every status input, on the injected clock. Only the
// source under test goes silent; RPP and the other inputs keep arriving.
TEST(MotionGuardNode, StatusInputFreshnessBoundaryIsHalfASecond) {
  struct Case {
    const char* name;
    bool Rig::* flag;
    uint8_t reason;
  };
  using S = dyx3_interfaces::msg::MotionSetpointStatus;
  const Case cases[] = {{"mission", &Rig::mission_running, S::REASON_MISSION_GATE},
                        {"vehicle", &Rig::veh_ok, S::REASON_ARMING_GATE},
                        {"rtk", &Rig::rtk_ok, S::REASON_RTK_GATE},
                        {"px4 link", &Rig::link_ok, S::REASON_PX4_LINK_UNHEALTHY},
                        {"estimator", &Rig::est_ok, S::REASON_HEADING_UNHEALTHY}};
  for (const auto& c : cases) {
    Rig r;
    r.run(0.3);
    ASSERT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE) << c.name;
    r.*(c.flag) = false;
    const double last_heard = r.now;  // the last tick that published this source
    r.now = last_heard + 0.46;
    r.tick();  // input age 0.48 s: still inside the limit
    EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE) << c.name << " at 0.48 s";
    EXPECT_FLOAT_EQ(r.last_out.speed_body_x, 0.3F) << c.name << " at 0.48 s";
    r.tick(0.04);  // input age 0.52 s: past the limit
    EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_STOP) << c.name << " at 0.52 s";
    EXPECT_EQ(r.last_out.speed_body_x, 0.0F) << c.name << " at 0.52 s";
    EXPECT_EQ(r.last_status.reason_code, c.reason) << c.name << " at 0.52 s";
  }
}

TEST(MotionGuardNode, RestartedPublisherIsStoppedUntilItRebuildsASession) {
  Rig r;
  r.run(0.3);
  ASSERT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
  r.seq = 0;  // RPP restarted: seq starts over
  r.tick();
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_STOP);
  EXPECT_EQ(r.last_status.reason_code, dyx3_interfaces::msg::MotionSetpointStatus::REASON_SEQUENCE);
  r.run(0.2);
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
}

TEST(MotionGuardNode, InvalidAndClampedCommands) {
  Rig r;
  r.run(0.2);
  r.tick(0.02, true, 2, 5.0F, NaN, 2.0F);  // over the limits: clamp
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
  EXPECT_FLOAT_EQ(r.last_out.speed_body_x, 1.0F);
  EXPECT_FLOAT_EQ(r.last_out.yaw_rate_setpoint, 0.45F);
  EXPECT_TRUE(r.last_status.clamped);
  EXPECT_EQ(r.last_status.reason_code,
            dyx3_interfaces::msg::MotionSetpointStatus::REASON_LIMIT_CLAMPED);
  r.tick(0.02, true, 1, 0.3F, NaN, NaN);  // TRACK_HEADING without a heading: invalid
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_STOP);
  EXPECT_EQ(r.last_status.reason_code,
            dyx3_interfaces::msg::MotionSetpointStatus::REASON_INVALID_MESSAGE);
}

TEST(MotionGuardNode, GateStatusIsPublishedWithoutAnyMotionCommand) {
  Rig r;
  r.run(0.5, false);
  EXPECT_TRUE(r.last_gate.ok);
  r.rtk_ok = false;
  r.run(0.8, false);
  EXPECT_FALSE(r.last_gate.ok);
  EXPECT_EQ(r.last_gate.reason_code, dyx3_interfaces::msg::MotionSetpointStatus::REASON_RTK_GATE);
}

// Mission contract v2: the pre-arm verdict is published next to the full gate, from the same
// inputs.
TEST(MotionGuardNode, PreArmGateIsPublishedAndIgnoresArmedAndOffboard) {
  using S = dyx3_interfaces::msg::MotionSetpointStatus;
  Rig r;
  r.veh_armed_offboard = false;  // disarmed, manual mode: the rover before ARMING
  r.mission_running = false;
  r.run(0.5, false);
  EXPECT_FALSE(r.last_gate.ok);
  EXPECT_EQ(r.last_gate.reason_code, S::REASON_ARMING_GATE);
  EXPECT_TRUE(r.last_gate.pre_arm_ok);
  r.veh_global_ref = false;
  r.run(0.3, false);
  EXPECT_FALSE(r.last_gate.pre_arm_ok);
  EXPECT_EQ(r.last_gate.pre_arm_reason_code, S::REASON_GLOBAL_REFERENCE_INVALID);
  r.veh_global_ref = true;
  r.rtk_ok = false;  // RTK lost: pre-arm fails with the same reason as the full gate
  r.run(0.8, false);
  EXPECT_FALSE(r.last_gate.pre_arm_ok);
  EXPECT_EQ(r.last_gate.pre_arm_reason_code, S::REASON_RTK_GATE);
  r.rtk_ok = true;
  r.run(0.3, false);
  ASSERT_TRUE(r.last_gate.pre_arm_ok);
  ASSERT_TRUE(r.call_estop(true, "tablet"));  // E-stop is first in both orders
  r.deliver();
  EXPECT_FALSE(r.last_gate.pre_arm_ok);
  EXPECT_EQ(r.last_gate.pre_arm_reason_code, S::REASON_ESTOP);
  // A global reference loss never touches the full gate (fail-to-zero unchanged).
  ASSERT_TRUE(r.call_estop(false, "tablet"));
  r.veh_armed_offboard = true;
  r.veh_global_ref = false;
  r.run(0.3, false);
  EXPECT_TRUE(r.last_gate.ok);
  EXPECT_FALSE(r.last_gate.pre_arm_ok);
}

// PX4's pre-flight checks verdict refuses the pre-arm verdict only; the full gate (armed) ignores
// it.
TEST(MotionGuardNode, PreArmGateRefusesWhenPx4PreflightChecksFail) {
  using S = dyx3_interfaces::msg::MotionSetpointStatus;
  Rig r;
  r.veh_armed_offboard = false;
  r.mission_running = false;
  r.run(0.5, false);
  ASSERT_TRUE(r.last_gate.pre_arm_ok);
  r.veh_preflight_pass = false;
  r.run(0.3, false);
  EXPECT_FALSE(r.last_gate.pre_arm_ok);
  EXPECT_EQ(r.last_gate.pre_arm_reason_code, S::REASON_ARMING_GATE);
  r.veh_preflight_pass = true;
  r.run(0.3, false);
  EXPECT_TRUE(r.last_gate.pre_arm_ok);
  // Armed + OFFBOARD with the flag false: the full gate still passes.
  r.veh_armed_offboard = true;
  r.veh_preflight_pass = false;
  r.run(0.3, false);
  EXPECT_TRUE(r.last_gate.ok);
}
// A never-seen vehicle state still fails pre-arm, with the existing reason.
TEST(MotionGuardNode, PreArmGateFailsWithArmingReasonWithoutAnyVehicleState) {
  using S = dyx3_interfaces::msg::MotionSetpointStatus;
  Rig r;
  r.veh_ok = false;
  r.mission_running = false;
  r.run(0.5, false);
  EXPECT_FALSE(r.last_gate.pre_arm_ok);
  EXPECT_EQ(r.last_gate.pre_arm_reason_code, S::REASON_ARMING_GATE);
}

// 2026-10-10: PX4 left in OFFBOARD after a run whose MANUAL release did not take reports
// pre_flight_checks_pass = false (canArm(OFFBOARD), Commander.cpp:1874). dyx3_px4_link leaves
// OFFBOARD for MANUAL before it arms, so the pre-arm verdict must not refuse the Start there.
TEST(MotionGuardNode, PreArmGatePassesInOffboardWithPreflightFalse) {
  using S = dyx3_interfaces::msg::MotionSetpointStatus;
  Rig r;
  r.veh_armed_offboard = false;  // disarmed
  r.veh_nav = 14;                // but PX4 still in OFFBOARD
  r.veh_preflight_pass = false;
  r.mission_running = false;
  r.run(0.5, false);
  EXPECT_TRUE(r.last_gate.pre_arm_ok);
  EXPECT_FALSE(r.last_gate.ok);  // the full gate still needs armed
  r.veh_nav = 0;                 // MANUAL: the flag is PX4's real verdict
  r.run(0.3, false);
  EXPECT_FALSE(r.last_gate.pre_arm_ok);
  EXPECT_EQ(r.last_gate.pre_arm_reason_code, S::REASON_ARMING_GATE);
}

// 0.17.0 require_rc_link: a pre-arm requirement only, off by default.
TEST(MotionGuardNode, RequireRcLinkGatesOnlyThePreArmVerdict) {
  using S = dyx3_interfaces::msg::MotionSetpointStatus;
  {
    Rig off;  // default false: the RC fields are ignored
    off.veh_armed_offboard = false;
    off.mission_running = false;
    off.run(0.5, false);
    EXPECT_TRUE(off.last_gate.pre_arm_ok);
  }
  Rig r({rclcpp::Parameter("require_rc_link", true)});
  r.veh_armed_offboard = false;
  r.mission_running = false;
  struct Combo {
    bool valid, ok, pre_arm_ok;
  };
  for (const Combo c : {Combo{false, false, false}, Combo{false, true, false},
                        Combo{true, false, false}, Combo{true, true, true}}) {
    r.veh_rc_valid = c.valid;
    r.veh_rc_ok = c.ok;
    r.run(0.3, false);
    EXPECT_EQ(r.last_gate.pre_arm_ok, c.pre_arm_ok) << c.valid << c.ok;
    if (!c.pre_arm_ok) {
      EXPECT_EQ(r.last_gate.pre_arm_reason_code, S::REASON_ARMING_GATE) << c.valid << c.ok;
    }
  }
  // Running (armed + OFFBOARD): RC loss is not a stop and does not fail the full gate.
  r.veh_armed_offboard = true;
  r.mission_running = true;
  r.veh_rc_valid = false;
  r.veh_rc_ok = false;
  r.veh_vn = 0.3F;  // moving
  r.run(0.3);
  EXPECT_TRUE(r.last_gate.ok);
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
}

// 0.17.0 actuator plausibility, measured 2026-10-10: a -0.45 rad/s pivot commanded with the
// drivetrain powered off, measured yaw rate and speed 0. STOP after the 1.0 s stall time, held
// until the demand drops, and published as the safety-gate verdict so the mission pauses.
TEST(MotionGuardNode, ActuatorStallStopsAndFailsTheSafetyGateUntilTheDemandDrops) {
  using S = dyx3_interfaces::msg::MotionSetpointStatus;
  Rig r;
  const auto pivot = [&r]() { r.tick(0.02, true, MotionSetpoint::MODE_PIVOT, 0.0F, NaN, -0.45F); };
  double first_forward = -1.0, stalled_at = -1.0;
  for (int i = 0; i < 150 && stalled_at < 0.0; ++i) {
    pivot();
    if (first_forward < 0.0 && r.last_out.mode == MotionSetpoint::MODE_PIVOT) first_forward = r.now;
    if (r.last_status.reason_code == S::REASON_ACTUATOR_STALL) stalled_at = r.now;
  }
  ASSERT_GE(first_forward, 0.0);
  ASSERT_GE(stalled_at, 0.0);
  // Longer than 1.0 s of still measurement, within one tick of it (injected clock).
  EXPECT_GT(stalled_at - first_forward, 1.0 - 1e-6);
  EXPECT_LE(stalled_at - first_forward, 1.0 + 0.02 + 1e-6);
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_STOP);
  EXPECT_EQ(r.last_out.speed_body_x, 0.0F);
  EXPECT_EQ(r.last_out.yaw_rate_setpoint, 0.0F);
  EXPECT_FALSE(r.last_status.accepted);
  // The latch is published at once on the safety gate (no wait for the 10 Hz slot).
  ASSERT_FALSE(r.gates.empty());
  EXPECT_FALSE(r.gates.back().ok);
  EXPECT_EQ(r.gates.back().reason_code, S::REASON_ACTUATOR_STALL);
  EXPECT_TRUE(r.gates.back().pre_arm_ok);  // never a pre-arm reason
  // Held: the same demand never gets through again, no STOP/PIVOT flapping.
  const size_t n = r.outs.size();
  for (int i = 0; i < 100; ++i) pivot();
  for (size_t i = n; i < r.outs.size(); ++i)
    ASSERT_EQ(r.outs[i].mode, MotionSetpoint::MODE_STOP) << i;
  EXPECT_EQ(r.last_status.reason_code, S::REASON_ACTUATOR_STALL);
  EXPECT_FALSE(r.last_gate.ok);
  EXPECT_EQ(r.last_gate.reason_code, S::REASON_ACTUATOR_STALL);
  // The mission pauses, RPP sends STOP: the latch clears and the gate passes again at once.
  r.tick(0.02, true, MotionSetpoint::MODE_STOP, 0.0F, NaN, 0.0F);
  EXPECT_EQ(r.last_status.reason_code, S::REASON_OK);
  EXPECT_TRUE(r.gates.back().ok);
  pivot();  // a new demand starts a new stall time
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_PIVOT);
}

TEST(MotionGuardNode, ActuatorStallIgnoresALegitimateRampAndAMovingVehicle) {
  using S = dyx3_interfaces::msg::MotionSetpointStatus;
  Rig r;
  // 0.6 m/s demand from rest; the measured speed passes 0.02 m/s at 0.9 s.
  double t = 0.0;
  for (int i = 0; i < 150; ++i) {
    t += 0.02;
    r.veh_vn = t > 0.9 ? 0.05F : 0.0F;
    r.tick(0.02, true, MotionSetpoint::MODE_TRACK_RATE, 0.6F, NaN, 0.0F);
  }
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
  for (const auto& s : r.statuses) EXPECT_NE(s.reason_code, S::REASON_ACTUATOR_STALL);
  EXPECT_TRUE(r.last_gate.ok);
  // Speed is horizontal: east-only motion counts as moving.
  r.veh_vn = 0.0F;
  r.veh_ve = -0.3F;
  r.run(1.5);
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
}

TEST(MotionGuardNode, RequireActuatorPlausibilityFalseDisablesTheStall) {
  using S = dyx3_interfaces::msg::MotionSetpointStatus;
  Rig r({rclcpp::Parameter("require_actuator_plausibility", false)});
  for (int i = 0; i < 150; ++i) r.tick(0.02, true, MotionSetpoint::MODE_PIVOT, 0.0F, NaN, -0.45F);
  EXPECT_EQ(r.last_out.mode, MotionSetpoint::MODE_PIVOT);
  for (const auto& s : r.statuses) EXPECT_NE(s.reason_code, S::REASON_ACTUATOR_STALL);
  EXPECT_TRUE(r.last_gate.ok);
}

// --- C3: decide and forward on each RPP command
// --------------------------------------------------- (a) The command is decided and forwarded
// inside its own callback: neither step() nor the watchdog runs, the injected clock does not move,
// and one output per command appears.
TEST(MotionGuardNode, EventDrivenForwardsEachCommandInItsCallback) {
  Rig r({rclcpp::Parameter("event_driven", true)});
  r.run(0.2);  // session accepted, every gate fresh
  ASSERT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
  for (int i = 1; i <= 5; ++i) {
    const size_t before = r.outs.size();
    const float v = 0.1F * static_cast<float>(i);
    r.rpp(MotionSetpoint::MODE_TRACK_RATE, v, NaN, 0.1F);
    ASSERT_TRUE(r.pump_until([&] { return r.outs.size() == before + 1; })) << i;
    EXPECT_FLOAT_EQ(r.last_out.speed_body_x, v);
    r.deliver();
    EXPECT_EQ(r.outs.size(), before + 1) << "one decision per command";
  }
}

// No double decision: after a command decision the watchdog waits two periods, then keeps the
// period; a command always decides.
TEST(MotionGuardNode, EventDrivenWatchdogNeverDoubleDecidesACommand) {
  Rig r({rclcpp::Parameter("event_driven", true)});
  r.run(0.2);
  const double t0 = r.now;  // the last command was decided at t0
  const size_t n = r.outs.size();
  const auto watchdog = [&](double at) {
    r.now = at;
    r.guard->on_watchdog(at);
    r.deliver();
    return r.outs.size();
  };
  EXPECT_EQ(watchdog(t0 + 0.001), n);
  EXPECT_EQ(watchdog(t0 + 0.020), n);      // one period: a late command is not silence
  EXPECT_EQ(watchdog(t0 + 0.030), n + 1);  // 1.5 periods of silence: decided
  EXPECT_EQ(watchdog(t0 + 0.035), n + 1);
  EXPECT_EQ(watchdog(t0 + 0.040), n + 2);
  r.now = t0 + 0.041;
  r.rpp(MotionSetpoint::MODE_TRACK_RATE, 0.3F, NaN, 0.1F);
  EXPECT_TRUE(r.pump_until([&] { return r.outs.size() == n + 3; }));
}

// (b) A silent RPP is stopped on the same command_max_age_s deadline in both modes.
TEST(MotionGuardNode, ASilentRppIsStoppedOnTheSameDeadlineInBothModes) {
  int stop_tick[2] = {-1, -1};
  for (int mode = 0; mode < 2; ++mode) {
    Rig r({rclcpp::Parameter("event_driven", mode == 1)});
    r.run(0.3);
    ASSERT_EQ(r.last_out.mode, MotionSetpoint::MODE_TRACK_RATE);
    for (int k = 1; k <= 30 && stop_tick[mode] < 0; ++k) {
      r.tick(0.02, /*with_rpp=*/false);
      if (r.last_out.mode == MotionSetpoint::MODE_STOP) {
        stop_tick[mode] = k;
        EXPECT_EQ(r.last_status.reason_code,
                  dyx3_interfaces::msg::MotionSetpointStatus::REASON_STALE);
      }
    }
  }
  // command_max_age_s 0.2 on a 50 Hz decision: the 10th or 11th tick (0.2 s is the boundary).
  EXPECT_GE(stop_tick[0], 10);
  EXPECT_LE(stop_tick[0], 11);
  EXPECT_EQ(stop_tick[1], stop_tick[0]);
}

// (c) Timer mode: a command alone produces nothing; the timer decides.
TEST(MotionGuardNode, TimerModeDecidesOnlyOnTheTimer) {
  Rig r({rclcpp::Parameter("event_driven", false)});
  r.run(0.2);
  const size_t before = r.outs.size();
  r.rpp(MotionSetpoint::MODE_TRACK_RATE, 0.3F, NaN, 0.1F);
  r.deliver();
  EXPECT_EQ(r.outs.size(), before);
  r.guard->on_watchdog(r.now + 0.001);
  EXPECT_TRUE(r.pump_until([&] { return r.outs.size() == before + 1; }));
}
