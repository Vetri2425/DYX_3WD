// In-process tests of the guard node: a fake world publishes every input and records every output,
// with an injected clock and a private DDS domain.
#include "dyx3_motion_guard/motion_guard_node.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

using namespace dyx3_motion_guard;
using namespace std::chrono_literals;

namespace {
constexpr float NaN = std::numeric_limits<float>::quiet_NaN();
using dyx3_interfaces::msg::MotionSetpoint;

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
  rclcpp::Publisher<dyx3_interfaces::msg::OperatorLinkStatus>::SharedPtr p_op;
  rclcpp::Publisher<dyx3_interfaces::msg::Px4LinkStatus>::SharedPtr p_link;
  rclcpp::Client<dyx3_interfaces::srv::SetEmergencyStop>::SharedPtr cli_estop;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> keep;
  MotionSetpoint last_out;
  std::vector<MotionSetpoint> outs;
  dyx3_interfaces::msg::MotionSetpointStatus last_status;
  dyx3_interfaces::msg::SafetyGateStatus last_gate;
  dyx3_interfaces::msg::EmergencyStopState last_estop;
  uint64_t seq{0};
  bool veh_ok{true}, rtk_ok{true}, op_ok{true}, link_ok{true}, est_ok{true}, mission_running{true};

  explicit Rig(const std::vector<rclcpp::Parameter>& params = {}) {
    ctx = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions io;
    io.set_domain_id(120 + (getpid() % 100));
    ctx->init(0, nullptr, io);
    rclcpp::NodeOptions no;
    no.context(ctx);
    no.append_parameter_override("max_reverse_speed_mps", 0.3);
    for (const auto& p : params)
      no.append_parameter_override(p.get_name(), p.get_parameter_value());
    guard = std::make_shared<MotionGuardNode>(no, [this]() { return now; }, false);
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
    p_op = world->create_publisher<dyx3_interfaces::msg::OperatorLinkStatus>("/dyx3/operator_link",
                                                                             r1);
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
        }));
    keep.push_back(world->create_subscription<dyx3_interfaces::msg::SafetyGateStatus>(
        "/dyx3/safety_gate", r1,
        [this](dyx3_interfaces::msg::SafetyGateStatus::ConstSharedPtr m) { last_gate = *m; }));
    keep.push_back(world->create_subscription<dyx3_interfaces::msg::EmergencyStopState>(
        "/dyx3/emergency_stop_state", r1,
        [this](dyx3_interfaces::msg::EmergencyStopState::ConstSharedPtr m) { last_estop = *m; }));
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
          world->count_subscribers("/dyx3/operator_link") > 0 &&
          world->count_subscribers("/dyx3/px4_link/status") > 0 && cli_estop->service_is_ready()) {
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

  void pump(int ms = 12) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) exec->spin_some(2ms);
  }
  void publish_world() {
    if (mission_running) {
      dyx3_interfaces::msg::MissionState m;
      m.state = 3;
      p_mission->publish(m);
    }
    if (veh_ok) {
      dyx3_interfaces::msg::VehicleState v;
      v.arming_state = 2;
      v.nav_state = 14;
      v.position_valid = v.velocity_valid = v.attitude_valid = true;
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
    if (op_ok) {
      dyx3_interfaces::msg::OperatorLinkStatus o;
      o.alive = true;
      p_op->publish(o);
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
    p_cmd->publish(m);
  }
  // One cycle: world publishes, deliver, guard decides, deliver outputs.
  void tick(double dt = 0.02, bool with_rpp = true, uint8_t mode = 2, float v = 0.3F,
            float yaw = NaN, float rate = 0.1F) {
    now += dt;
    publish_world();
    if (with_rpp) rpp(mode, v, yaw, rate);
    pump();
    guard->step(now);
    pump();
  }
  void run(double seconds, bool with_rpp = true) {
    for (double t = 0; t < seconds; t += 0.02) tick(0.02, with_rpp);
  }
};

}  // namespace

TEST(MotionGuardNode, RejectsBadParameters) {
  auto ctx = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions io;
  io.set_domain_id(120 + (getpid() % 100));
  ctx->init(0, nullptr, io);
  rclcpp::NodeOptions no;
  no.context(ctx);
  no.append_parameter_override("max_yaw_rate_radps", -1.0);
  EXPECT_THROW(MotionGuardNode(no, nullptr, false), std::invalid_argument);
  rclcpp::NodeOptions no2;
  no2.context(ctx);
  no2.append_parameter_override("rtk_min_fix_type", 3);
  EXPECT_THROW(MotionGuardNode(no2, nullptr, false), std::invalid_argument);
  ctx->shutdown("test done");
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
                        {"operator", &Rig::op_ok, S::REASON_OPERATOR_LINK_LOST},
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
