// Fault-injection tests: a fake FCU (publishes /fmu/out, records /fmu/in) against the real node,
// with an injected clock. Run in a private DDS domain so it cannot talk to other test processes.
#include "dyx3_px4_link/px4_link_node.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <map>
#include <thread>

using namespace dyx3_px4_link;
using namespace std::chrono_literals;

namespace {

constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

// Hashes come from the FIRMWARE's own Python (fixtures), independent of the node's C++ hash.
std::map<std::string, uint32_t> firmware_hashes() {
  std::map<std::string, uint32_t> m;
  std::ifstream f(std::string(DYX3_FIXTURES) + "/px4_msg_hash_vectors.txt");
  std::string n;
  uint64_t h;
  while (f >> n >> h) m[n] = static_cast<uint32_t>(h);
  return m;
}
const std::map<std::string, std::string> kTopicType = {
    {"/fmu/in/offboard_control_mode", "OffboardControlMode"},
    {"/fmu/in/trajectory_setpoint", "TrajectorySetpoint"},
    {"/fmu/in/rover_speed_setpoint", "RoverSpeedSetpoint"},
    {"/fmu/in/rover_attitude_setpoint", "RoverAttitudeSetpoint"},
    {"/fmu/in/rover_rate_setpoint", "RoverRateSetpoint"},
    {"/fmu/in/vehicle_command", "VehicleCommand"},
    {"/fmu/in/gps_inject_data", "GpsInjectData"},
    {"/fmu/in/ulog_stream_ack", "UlogStreamAck"},
    {"/fmu/out/timesync_status", "TimesyncStatus"},
    {"/fmu/out/vehicle_local_position", "VehicleLocalPosition"},
    {"/fmu/out/vehicle_status", "VehicleStatus"},
    {"/fmu/out/vehicle_attitude", "VehicleAttitude"},
    {"/fmu/out/estimator_status_flags", "EstimatorStatusFlags"},
    {"/fmu/out/vehicle_gps_position", "SensorGps"},
    {"/fmu/out/ulog_stream", "UlogStream"},
    {"/fmu/out/vehicle_command_ack", "VehicleCommandAck"}};

void init_ctx(const std::shared_ptr<rclcpp::Context>& ctx) {
  rclcpp::InitOptions io;
  io.set_domain_id(100 + (getpid() % 100));
  ctx->init(0, nullptr, io);
}

struct Rig {
  std::shared_ptr<rclcpp::Context> ctx;
  double now{100.0};
  std::shared_ptr<Px4LinkNode> link;
  std::shared_ptr<rclcpp::Node> fcu;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;

  // fake-FCU publishers (/fmu/out)
  rclcpp::Publisher<px4_msgs::msg::TimesyncStatus>::SharedPtr p_ts;
  int64_t ts_offset{0};
  uint32_t ts_rtt{0};
  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr p_lp;
  rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr p_st;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitude>::SharedPtr p_att;
  rclcpp::Publisher<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr p_fl;
  rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr p_gps;
  rclcpp::Publisher<px4_msgs::msg::MessageFormatResponse>::SharedPtr p_resp;
  rclcpp::Publisher<px4_msgs::msg::UlogStream>::SharedPtr p_ulog;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommandAck>::SharedPtr p_ack;
  rclcpp::Publisher<dyx3_interfaces::msg::SprayActuatorCommand>::SharedPtr p_spray;
  std::vector<dyx3_interfaces::msg::SprayActuatorAck> spray_acks;
  rclcpp::Publisher<dyx3_interfaces::msg::MotionSetpoint>::SharedPtr p_cmd;
  rclcpp::Publisher<dyx3_interfaces::msg::RtcmData>::SharedPtr p_rtcm;
  // recorded /fmu/in
  std::vector<px4_msgs::msg::OffboardControlMode> ocm;
  std::vector<px4_msgs::msg::TrajectorySetpoint> traj;
  std::vector<px4_msgs::msg::RoverSpeedSetpoint> speed;
  std::vector<px4_msgs::msg::RoverAttitudeSetpoint> att_sp;
  std::vector<px4_msgs::msg::RoverRateSetpoint> rate;
  std::vector<px4_msgs::msg::VehicleCommand> cmds;
  std::vector<px4_msgs::msg::GpsInjectData> inject;
  std::vector<px4_msgs::msg::UlogStreamAck> acks;
  std::vector<px4_msgs::msg::MessageFormatRequest> reqs;
  std::vector<dyx3_interfaces::msg::UlogChunk> chunks;
  dyx3_interfaces::msg::Px4LinkStatus status;
  dyx3_interfaces::msg::VehicleState state;
  dyx3_interfaces::msg::EstimatorHealth health;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> keep;
  rclcpp::Client<dyx3_interfaces::srv::SetOffboard>::SharedPtr cli_off;
  rclcpp::Client<dyx3_interfaces::srv::ArmDisarm>::SharedPtr cli_arm;
  uint64_t seq{0};
  bool fcu_answers_handshake{true};
  int hash_error_on{-1};  // lie about this topic's hash
  bool alive{true}, lp_alive{true};
  uint8_t nav_state{0}, arming_state{1};

  explicit Rig(const rclcpp::ParameterValue* extra = nullptr, const std::string& defs = "") {
    ctx = std::make_shared<rclcpp::Context>();
    init_ctx(ctx);
    rclcpp::NodeOptions no;
    no.context(ctx);
    const std::string dir = defs.empty() ? std::string(DYX3_FIXTURES) + "/msgdefs" : defs;
    no.append_parameter_override("msg_definitions_dir", dir);
    if (extra != nullptr) {
    }
    link = std::make_shared<Px4LinkNode>(no, [this]() { return now; }, false);
    rclcpp::NodeOptions fo;
    fo.context(ctx);
    fcu = std::make_shared<rclcpp::Node>("fake_fcu", fo);
    rclcpp::ExecutorOptions eo;
    eo.context = ctx;
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(eo);
    exec->add_node(link);
    exec->add_node(fcu);

    const auto sensor = rclcpp::SensorDataQoS();
    p_ts = fcu->create_publisher<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", sensor);
    p_lp = fcu->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", sensor);
    p_st =
        fcu->create_publisher<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v1", sensor);
    p_att =
        fcu->create_publisher<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", sensor);
    p_fl = fcu->create_publisher<px4_msgs::msg::EstimatorStatusFlags>(
        "/fmu/out/estimator_status_flags", sensor);
    p_gps =
        fcu->create_publisher<px4_msgs::msg::SensorGps>("/fmu/out/vehicle_gps_position", sensor);
    p_resp = fcu->create_publisher<px4_msgs::msg::MessageFormatResponse>(
        "/fmu/out/message_format_response", rclcpp::QoS(10).best_effort());
    p_ulog = fcu->create_publisher<px4_msgs::msg::UlogStream>("/fmu/out/ulog_stream",
                                                              rclcpp::QoS(16).reliable());
    p_ack = fcu->create_publisher<px4_msgs::msg::VehicleCommandAck>("/fmu/out/vehicle_command_ack",
                                                                    sensor);
    p_spray = fcu->create_publisher<dyx3_interfaces::msg::SprayActuatorCommand>(
        "/dyx3/spray/actuator_command", rclcpp::QoS(16).reliable());
    keep.push_back(fcu->create_subscription<dyx3_interfaces::msg::SprayActuatorAck>(
        "/dyx3/spray/actuator_ack", rclcpp::QoS(16).reliable(),
        [this](dyx3_interfaces::msg::SprayActuatorAck::ConstSharedPtr m) {
          spray_acks.push_back(*m);
        }));
    p_cmd = fcu->create_publisher<dyx3_interfaces::msg::MotionSetpoint>(
        "/dyx3/motion_guard/command", rclcpp::QoS(1).reliable());
    p_rtcm = fcu->create_publisher<dyx3_interfaces::msg::RtcmData>("/dyx3/rtcm",
                                                                   rclcpp::QoS(32).reliable());
    const auto r1 = rclcpp::QoS(1).reliable();
    const auto r10 = rclcpp::QoS(10).reliable();
    keep.push_back(fcu->create_subscription<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", r1,
        [this](px4_msgs::msg::OffboardControlMode::ConstSharedPtr m) { ocm.push_back(*m); }));
    keep.push_back(fcu->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", r1,
        [this](px4_msgs::msg::TrajectorySetpoint::ConstSharedPtr m) { traj.push_back(*m); }));
    keep.push_back(fcu->create_subscription<px4_msgs::msg::RoverSpeedSetpoint>(
        "/fmu/in/rover_speed_setpoint", r1,
        [this](px4_msgs::msg::RoverSpeedSetpoint::ConstSharedPtr m) { speed.push_back(*m); }));
    keep.push_back(fcu->create_subscription<px4_msgs::msg::RoverAttitudeSetpoint>(
        "/fmu/in/rover_attitude_setpoint", r1,
        [this](px4_msgs::msg::RoverAttitudeSetpoint::ConstSharedPtr m) { att_sp.push_back(*m); }));
    keep.push_back(fcu->create_subscription<px4_msgs::msg::RoverRateSetpoint>(
        "/fmu/in/rover_rate_setpoint", r1,
        [this](px4_msgs::msg::RoverRateSetpoint::ConstSharedPtr m) { rate.push_back(*m); }));
    keep.push_back(fcu->create_subscription<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", r10,
        [this](px4_msgs::msg::VehicleCommand::ConstSharedPtr m) { cmds.push_back(*m); }));
    keep.push_back(fcu->create_subscription<px4_msgs::msg::GpsInjectData>(
        "/fmu/in/gps_inject_data", r10,
        [this](px4_msgs::msg::GpsInjectData::ConstSharedPtr m) { inject.push_back(*m); }));
    keep.push_back(fcu->create_subscription<px4_msgs::msg::UlogStreamAck>(
        "/fmu/in/ulog_stream_ack", rclcpp::QoS(16).reliable(),
        [this](px4_msgs::msg::UlogStreamAck::ConstSharedPtr m) { acks.push_back(*m); }));
    keep.push_back(fcu->create_subscription<px4_msgs::msg::MessageFormatRequest>(
        "/fmu/in/message_format_request", r10,
        [this](px4_msgs::msg::MessageFormatRequest::ConstSharedPtr m) {
          reqs.push_back(*m);
          answer(*m);
        }));
    keep.push_back(fcu->create_subscription<dyx3_interfaces::msg::UlogChunk>(
        "/dyx3/ulog_chunk", rclcpp::QoS(64).reliable(),
        [this](dyx3_interfaces::msg::UlogChunk::ConstSharedPtr m) { chunks.push_back(*m); }));
    keep.push_back(fcu->create_subscription<dyx3_interfaces::msg::Px4LinkStatus>(
        "/dyx3/px4_link/status", r1,
        [this](dyx3_interfaces::msg::Px4LinkStatus::ConstSharedPtr m) { status = *m; }));
    keep.push_back(fcu->create_subscription<dyx3_interfaces::msg::VehicleState>(
        "/dyx3/vehicle_state", r1,
        [this](dyx3_interfaces::msg::VehicleState::ConstSharedPtr m) { state = *m; }));
    keep.push_back(fcu->create_subscription<dyx3_interfaces::msg::EstimatorHealth>(
        "/dyx3/estimator_health", r1,
        [this](dyx3_interfaces::msg::EstimatorHealth::ConstSharedPtr m) { health = *m; }));
    cli_off = fcu->create_client<dyx3_interfaces::srv::SetOffboard>("/dyx3/px4_link/set_offboard");
    cli_arm = fcu->create_client<dyx3_interfaces::srv::ArmDisarm>("/dyx3/px4_link/arm");
    wait_discovery();
  }

  ~Rig() {
    exec.reset();
    keep.clear();
    link.reset();
    fcu.reset();
    ctx->shutdown("test done");
  }

  void wait_discovery() {
    const auto end = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < end) {
      exec->spin_some(5ms);
      if (link->count_subscribers("/fmu/in/offboard_control_mode") > 0 &&
          link->count_subscribers("/fmu/in/message_format_request") > 0 &&
          link->count_subscribers("/dyx3/px4_link/status") > 0 &&
          fcu->count_subscribers("/dyx3/motion_guard/command") > 0 &&
          fcu->count_subscribers("/fmu/out/vehicle_status_v1") > 0 &&
          fcu->count_subscribers("/fmu/out/message_format_response") > 0 &&
          cli_off->service_is_ready() && cli_arm->service_is_ready() &&
          link->count_publishers("/dyx3/vehicle_state") > 0) {
        return;
      }
    }
    FAIL() << "DDS discovery did not complete";
  }

  void answer(const px4_msgs::msg::MessageFormatRequest& rq) {
    if (!fcu_answers_handshake) return;
    size_t n = 0;
    while (n < rq.topic_name.size() && rq.topic_name[n] != 0) ++n;
    const std::string name(rq.topic_name.begin(), rq.topic_name.begin() + n);
    px4_msgs::msg::MessageFormatResponse r;
    r.protocol_version = 1;
    r.topic_name = rq.topic_name;
    const auto it = kTopicType.find(name);
    r.success = it != kTopicType.end();
    r.message_hash = r.success ? firmware_hashes().at(it->second) : 0;
    if (r.success) {
      int idx = 0;
      for (const auto& kv : kTopicType) {
        if (kv.first == name && idx == hash_error_on) r.message_hash ^= 0x1U;
        ++idx;
      }
    }
    p_resp->publish(r);
  }

  // Fake FCU samples at the current clock.
  void publish_fcu() {
    if (alive) {
      px4_msgs::msg::TimesyncStatus ts;
      ts.estimated_offset = ts_offset;
      ts.round_trip_time = ts_rtt;
      p_ts->publish(ts);
    }
    if (alive) {
      px4_msgs::msg::VehicleStatus st;
      st.arming_state = arming_state;
      st.nav_state = nav_state;
      p_st->publish(st);
      px4_msgs::msg::VehicleAttitude a;
      a.q = {1.0F, 0.0F, 0.0F, 0.0F};
      p_att->publish(a);
      px4_msgs::msg::EstimatorStatusFlags fl;
      p_fl->publish(fl);
      px4_msgs::msg::SensorGps g;
      g.fix_type = 6;
      g.eph = 0.01F;
      p_gps->publish(g);
    }
    if (alive && lp_alive) {
      px4_msgs::msg::VehicleLocalPosition lp;
      lp.xy_valid = lp.v_xy_valid = lp.heading_good_for_control = true;
      lp.x = 3.0F;
      lp.y = 4.0F;
      lp.heading = 0.5F;
      p_lp->publish(lp);
    }
  }
  void guard(uint8_t mode, float v, float yaw, float rate, bool valid = true) {
    dyx3_interfaces::msg::MotionSetpoint m;
    m.seq = ++seq;
    m.mode = mode;
    m.speed_body_x = v;
    m.yaw_setpoint = yaw;
    m.yaw_rate_setpoint = rate;
    m.valid = valid;
    p_cmd->publish(m);
  }
  void pump(int ms = 30) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) exec->spin_some(2ms);
  }
  // Advance link-time by dt and run one cycle with the fake FCU alive, then deliver.
  void tick(double dt = 0.01, bool publish = true) {
    now += dt;
    if (publish) publish_fcu();
    pump(12);
    link->step(now);
    pump(12);
  }
  void run(double seconds, double dt = 0.01, bool publish = true) {
    for (double t = 0; t < seconds; t += dt) tick(dt, publish);
  }
  void clear() {
    ocm.clear();
    traj.clear();
    speed.clear();
    att_sp.clear();
    rate.clear();
    cmds.clear();
    inject.clear();
    acks.clear();
    reqs.clear();
    chunks.clear();
  }
  bool handshake_ok() { return link->handshake().state() == HandshakeState::Ok; }
  void bring_up() {
    for (int i = 0; i < 400 && !handshake_ok(); ++i) tick(0.01);
    ASSERT_TRUE(handshake_ok());
  }
  bool call_offboard(bool enable, bool* accepted, uint8_t* reason) {
    auto req = std::make_shared<dyx3_interfaces::srv::SetOffboard::Request>();
    req->enable = enable;
    auto fut = cli_off->async_send_request(req);
    for (int i = 0; i < 600; ++i) {
      tick(0.01);
      if (fut.wait_for(0ms) == std::future_status::ready) {
        const auto resp = fut.get();
        *accepted = resp->accepted;
        *reason = resp->reason_code;
        return true;
      }
    }
    return false;
  }
};

bool all_nan3(const std::array<float, 3>& a) {
  return std::isnan(a[0]) && std::isnan(a[1]) && std::isnan(a[2]);
}

}  // namespace

TEST(Px4LinkNode, RejectsPublishRateBelow100Hz) {
  auto ctx = std::make_shared<rclcpp::Context>();
  init_ctx(ctx);
  rclcpp::NodeOptions no;
  no.context(ctx);
  no.append_parameter_override("publish_rate_hz", 50.0);
  no.append_parameter_override("msg_definitions_dir", std::string(DYX3_FIXTURES) + "/msgdefs");
  EXPECT_THROW(Px4LinkNode(no, nullptr, false), std::invalid_argument);
  ctx->shutdown("test done");
}

TEST(Px4LinkNode, HandshakeCompletesAndRequestsUseBaseTopicNames) {
  Rig r;
  r.bring_up();
  ASSERT_FALSE(r.reqs.empty());
  bool saw_versioned_base = false;
  for (const auto& q : r.reqs) {
    const std::string n(
        q.topic_name.begin(),
        q.topic_name.begin() + std::strlen(reinterpret_cast<const char*>(q.topic_name.data())));
    EXPECT_EQ(n.find("_v1"), std::string::npos) << n;  // firmware matches the uORB name
    if (n == "/fmu/out/vehicle_status") saw_versioned_base = true;
    EXPECT_EQ(q.protocol_version, 1);
  }
  EXPECT_TRUE(saw_versioned_base);
  r.run(0.3);
  EXPECT_TRUE(r.status.handshake_ok);
  EXPECT_TRUE(r.status.session_alive);
  EXPECT_EQ(r.status.stale_topics_mask, 0U);
}

TEST(Px4LinkNode, NoSetpointsBeforeHandshakeAndNoneWithoutOffboardEnable) {
  Rig r;
  r.fcu_answers_handshake = false;
  r.run(1.0);
  EXPECT_TRUE(r.ocm.empty());
  EXPECT_TRUE(r.speed.empty());
  EXPECT_EQ(r.status.fault, dyx3_interfaces::msg::Px4LinkStatus::FAULT_HANDSHAKE_PENDING);
  r.fcu_answers_handshake = true;
  r.bring_up();
  r.run(0.5);
  EXPECT_TRUE(r.ocm.empty());  // handshake alone never starts a stream: offboard not enabled
}

TEST(Px4LinkNode, HashMismatchIsLoudAndRefusesEverything) {
  Rig r;
  r.hash_error_on = 5;  // lie about vehicle_command
  r.run(1.5);
  EXPECT_EQ(r.link->handshake().state(), HandshakeState::Mismatch);
  EXPECT_EQ(r.status.fault, dyx3_interfaces::msg::Px4LinkStatus::FAULT_HANDSHAKE_MISMATCH);
  EXPECT_FALSE(r.status.handshake_ok);
  bool accepted = true;
  uint8_t reason = 0;
  ASSERT_TRUE(r.call_offboard(true, &accepted, &reason));
  EXPECT_FALSE(accepted);
  r.run(0.5);
  EXPECT_TRUE(r.ocm.empty());
  EXPECT_TRUE(r.cmds.empty());
}

TEST(Px4LinkNode, MissingLocalDefinitionIsAMismatch) {
  Rig r(nullptr, "/nonexistent_dir");
  r.run(1.0);
  EXPECT_EQ(r.link->handshake().state(), HandshakeState::Mismatch);
  EXPECT_TRUE(r.ocm.empty());
}

TEST(Px4LinkNode, ExplicitControlSetEveryCycleForEachMode) {
  Rig r;
  r.bring_up();
  bool acc = false;
  uint8_t rs = 0;
  r.nav_state = 14;  // the fake FCU grants OFFBOARD once asked
  ASSERT_TRUE(r.call_offboard(true, &acc, &rs));
  EXPECT_TRUE(acc) << "reason " << int(rs) << " mask " << r.status.stale_topics_mask << " fault "
                   << int(r.status.fault);
  // DO_SET_MODE sent after the heartbeat started, exactly once.
  int mode_cmds = 0;
  for (const auto& c : r.cmds) mode_cmds += (c.command == 176) ? 1 : 0;
  EXPECT_EQ(mode_cmds, 1);
  ASSERT_FALSE(r.ocm.empty());
  EXPECT_LT(r.ocm.front().timestamp, r.cmds.back().timestamp + 1);

  struct Case {
    uint8_t mode;
    float v, yaw, rate;
  };
  const Case cases[] = {
      {1, 0.35F, 0.7F, NaN}, {2, -0.2F, NaN, 0.1F}, {3, 0.0F, NaN, 0.4F}, {4, 0.05F, NaN, 0.0F}};
  for (const auto& c : cases) {
    r.clear();
    r.guard(c.mode, c.v, c.yaw, c.rate);
    r.pump(40);
    r.run(0.05);
    ASSERT_FALSE(r.speed.empty());
    ASSERT_EQ(r.ocm.size(), r.traj.size());
    ASSERT_EQ(r.ocm.size(), r.speed.size());
    ASSERT_EQ(r.ocm.size(), r.att_sp.size());
    ASSERT_EQ(r.ocm.size(), r.rate.size());  // all five, every cycle
    EXPECT_TRUE(r.ocm.back().velocity);
    EXPECT_FALSE(r.ocm.back().position || r.ocm.back().attitude || r.ocm.back().body_rate ||
                 r.ocm.back().acceleration || r.ocm.back().thrust_and_torque ||
                 r.ocm.back().direct_actuator);
    EXPECT_TRUE(all_nan3(r.traj.back().velocity));  // finite velocity must never reach PX4
    EXPECT_TRUE(all_nan3(r.traj.back().position));
    EXPECT_FLOAT_EQ(r.speed.back().speed_body_x, c.v);
    if (std::isnan(c.yaw)) {
      EXPECT_TRUE(std::isnan(r.att_sp.back().yaw_setpoint)) << "mode " << int(c.mode);
    } else {
      EXPECT_FLOAT_EQ(r.att_sp.back().yaw_setpoint, c.yaw);
    }
    if (std::isnan(c.rate)) {
      EXPECT_TRUE(std::isnan(r.rate.back().yaw_rate_setpoint));
    } else {
      EXPECT_FLOAT_EQ(r.rate.back().yaw_rate_setpoint, c.rate);
    }
  }
}

TEST(Px4LinkNode, GuardSilenceFallsToExplicitZeroWithHeartbeatKept) {
  Rig r;
  r.bring_up();
  r.nav_state = 14;
  bool acc = false;
  uint8_t rs = 0;
  ASSERT_TRUE(r.call_offboard(true, &acc, &rs));
  r.guard(2, 0.35F, NaN, 0.1F);
  r.run(0.1);
  EXPECT_FLOAT_EQ(r.speed.empty() ? 0.0F : r.speed.back().speed_body_x, 0.35F);
  r.clear();
  r.run(0.5);  // the guard says nothing more: > command_max_age_s
  ASSERT_FALSE(r.speed.empty());
  EXPECT_EQ(r.speed.back().speed_body_x, 0.0F);
  EXPECT_EQ(r.rate.back().yaw_rate_setpoint, 0.0F);
  EXPECT_TRUE(std::isnan(r.att_sp.back().yaw_setpoint));
  EXPECT_TRUE(all_nan3(r.traj.back().velocity));
  EXPECT_FALSE(r.ocm.empty());  // fresh explicit zero, heartbeat kept
  EXPECT_TRUE(r.status.failing_to_zero);
  EXPECT_EQ(r.status.fault, dyx3_interfaces::msg::Px4LinkStatus::FAULT_COMMAND_STALE);
  EXPECT_GE(r.status.command_gap_events, 1U);
  // a command with a new seq recovers motion without any latch
  r.guard(2, 0.3F, NaN, 0.1F);
  r.run(0.1);
  EXPECT_FLOAT_EQ(r.speed.back().speed_body_x, 0.3F);
}

TEST(Px4LinkNode, SilentTopicWhileSessionUpForcesZero) {  // upstream #27388
  Rig r;
  r.bring_up();
  r.nav_state = 14;
  bool acc = false;
  uint8_t rs = 0;
  ASSERT_TRUE(r.call_offboard(true, &acc, &rs));
  for (int i = 0; i < 10; ++i) {
    r.guard(2, 0.35F, NaN, 0.1F);
    r.tick();
  }
  EXPECT_FLOAT_EQ(r.speed.back().speed_body_x, 0.35F);
  r.lp_alive = false;  // only vehicle_local_position stops; the session stays up
  r.clear();
  for (int i = 0; i < 60; ++i) {
    r.guard(2, 0.35F, NaN, 0.1F);  // guard keeps commanding motion
    r.tick();
  }
  EXPECT_TRUE(r.status.session_alive);
  EXPECT_EQ(r.status.stale_topics_mask, 1U << 1);
  EXPECT_EQ(r.status.fault, dyx3_interfaces::msg::Px4LinkStatus::FAULT_TOPIC_STALE);
  EXPECT_EQ(r.speed.back().speed_body_x, 0.0F);
  EXPECT_FALSE(r.state.position_valid);  // fan-out clears validity in the same condition
}

TEST(Px4LinkNode, SessionLossThenRecoveryReArmsTheHandshake) {
  Rig r;
  r.bring_up();
  r.run(0.3);
  r.alive = false;
  r.run(1.5);
  EXPECT_FALSE(r.status.session_alive);
  EXPECT_EQ(r.status.fault, dyx3_interfaces::msg::Px4LinkStatus::FAULT_NO_SESSION);
  r.reqs.clear();
  r.fcu_answers_handshake = false;  // new firmware not answering yet
  r.alive = true;
  r.run(0.5);
  EXPECT_FALSE(r.handshake_ok());  // re-armed: setpoints stay off until proven again
  EXPECT_FALSE(r.reqs.empty());
  EXPECT_GE(r.status.session_resets, 1U);
  r.fcu_answers_handshake = true;
  // Responses are best effort and retried every handshake_retry_s: allow a few retry periods.
  for (int i = 0; i < 500 && !r.handshake_ok(); ++i) r.tick();
  EXPECT_TRUE(r.handshake_ok());
}

TEST(Px4LinkNode, ArmRefusedWhenUnhealthyConfirmedByStatus) {
  Rig r;
  r.fcu_answers_handshake = false;
  r.run(0.3);
  auto req = std::make_shared<dyx3_interfaces::srv::ArmDisarm::Request>();
  req->arm = true;
  auto fut = r.cli_arm->async_send_request(req);
  for (int i = 0; i < 100 && fut.wait_for(0ms) != std::future_status::ready; ++i) r.tick();
  ASSERT_EQ(fut.wait_for(0ms), std::future_status::ready);
  auto resp = fut.get();
  EXPECT_FALSE(resp->accepted);
  EXPECT_EQ(resp->reason_code, dyx3_interfaces::srv::ArmDisarm::Response::REASON_LINK_UNHEALTHY);

  r.fcu_answers_handshake = true;
  r.bring_up();
  r.run(0.3);
  r.cmds.clear();
  r.arming_state = 2;  // the fake FCU arms
  auto fut2 = r.cli_arm->async_send_request(req);
  for (int i = 0; i < 300 && fut2.wait_for(0ms) != std::future_status::ready; ++i) r.tick();
  ASSERT_EQ(fut2.wait_for(0ms), std::future_status::ready);
  EXPECT_TRUE(fut2.get()->accepted);
  int arm_cmds = 0;
  for (const auto& c : r.cmds) arm_cmds += (c.command == 400 && c.param1 == 1.0F) ? 1 : 0;
  EXPECT_EQ(arm_cmds, 1);
}

TEST(Px4LinkNode, UlogChunksAreAckedThenRepublished) {
  Rig r;
  r.bring_up();
  r.run(0.1);
  px4_msgs::msg::UlogStream u;
  u.msg_sequence = 7;
  u.flags = px4_msgs::msg::UlogStream::FLAGS_NEED_ACK;
  u.length = 3;
  u.data[0] = 1;
  u.data[1] = 2;
  u.data[2] = 3;
  r.p_ulog->publish(u);
  r.pump(100);
  ASSERT_EQ(r.acks.size(), 1U);
  EXPECT_EQ(r.acks[0].msg_sequence, 7);
  ASSERT_EQ(r.chunks.size(), 1U);
  EXPECT_EQ(r.chunks[0].msg_sequence, 7);
  ASSERT_EQ(r.chunks[0].data.size(), 3U);
  EXPECT_EQ(r.chunks[0].data[2], 3);
  // logging start requested once after the handshake
  int starts = 0;
  for (const auto& c : r.cmds) starts += (c.command == 2510) ? 1 : 0;
  EXPECT_EQ(starts, 1);
}

TEST(Px4LinkNode, RtcmForwardedOnlyWhenLinkHealthyAndNeverTruncated) {
  Rig r;
  r.fcu_answers_handshake = false;
  r.run(0.2);
  dyx3_interfaces::msg::RtcmData d;
  d.flags = 1;
  d.data.assign(120, 0xD3);
  r.p_rtcm->publish(d);
  r.pump(100);
  EXPECT_TRUE(r.inject.empty());
  r.fcu_answers_handshake = true;
  r.bring_up();
  r.run(0.1);
  r.p_rtcm->publish(d);
  r.pump(100);
  ASSERT_EQ(r.inject.size(), 1U);
  EXPECT_EQ(r.inject[0].len, 120);
  EXPECT_EQ(r.inject[0].flags, 1);
  EXPECT_EQ(r.inject[0].data[119], 0xD3);
  d.data.assign(301, 0x01);
  r.p_rtcm->publish(d);
  r.pump(100);
  EXPECT_EQ(r.inject.size(), 1U);  // oversize rejected, not truncated
}

TEST(Px4LinkNode, SprayActuatorCommandsBecomeVehicleCommandsAndFcuAcksAreMapped) {
  Rig r;
  r.bring_up();
  r.run(0.2);
  r.clear();
  dyx3_interfaces::msg::SprayActuatorCommand m;
  m.seq = 41;
  m.source = dyx3_interfaces::msg::SprayActuatorCommand::SOURCE_CONTROLLER;
  m.backend = dyx3_interfaces::msg::SprayActuatorCommand::BACKEND_ACTUATOR;
  m.on = true;
  m.actuator_set_index = 2;
  m.value = 0.7F;
  r.p_spray->publish(m);
  r.pump(150);
  int n187 = 0;
  for (const auto& c : r.cmds) {
    if (c.command != 187) continue;
    ++n187;
    EXPECT_TRUE(std::isnan(c.param1));  // only the chosen slot carries a value
    EXPECT_FLOAT_EQ(c.param2, 0.7F);
    EXPECT_TRUE(std::isnan(c.param3));
    EXPECT_TRUE(std::isnan(c.param4));
  }
  EXPECT_EQ(n187, 1);
  EXPECT_TRUE(r.spray_acks.empty());  // no ack until the FCU answers
  px4_msgs::msg::VehicleCommandAck a;
  a.command = 187;
  a.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS;
  r.p_ack->publish(a);
  r.pump(100);
  EXPECT_TRUE(r.spray_acks.empty());  // in progress is not a result
  a.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  r.p_ack->publish(a);
  r.pump(150);
  ASSERT_EQ(r.spray_acks.size(), 1U);
  EXPECT_EQ(r.spray_acks[0].seq, 41U);
  EXPECT_TRUE(r.spray_acks[0].success);
  // servo backend and a rejected result
  m.seq = 42;
  m.backend = dyx3_interfaces::msg::SprayActuatorCommand::BACKEND_SERVO_PWM;
  m.servo_instance = 3;
  m.pwm_us = 60000;  // out of range: clamped to 2200
  r.p_spray->publish(m);
  r.pump(150);
  bool saw_servo = false;
  for (const auto& c : r.cmds) {
    if (c.command == 183) {
      saw_servo = true;
      EXPECT_FLOAT_EQ(c.param1, 3.0F);
      EXPECT_FLOAT_EQ(c.param2, 2200.0F);
    }
  }
  EXPECT_TRUE(saw_servo);
  a.command = 183;
  a.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED;
  r.p_ack->publish(a);
  r.pump(150);
  ASSERT_EQ(r.spray_acks.size(), 2U);
  EXPECT_EQ(r.spray_acks[1].seq, 42U);
  EXPECT_FALSE(r.spray_acks[1].success);
}

TEST(Px4LinkNode, SprayCommandIsRefusedAtOnceWhenTheLinkIsNotProven) {
  Rig r;
  r.fcu_answers_handshake = false;
  r.run(0.3);
  dyx3_interfaces::msg::SprayActuatorCommand m;
  m.seq = 7;
  m.source = dyx3_interfaces::msg::SprayActuatorCommand::SOURCE_WATCHDOG;
  m.on = false;
  m.actuator_set_index = 1;
  m.value = -1.0F;
  r.p_spray->publish(m);
  r.pump(200);
  ASSERT_EQ(r.spray_acks.size(), 1U);
  EXPECT_FALSE(r.spray_acks[0].success);
  EXPECT_EQ(r.spray_acks[0].result, dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);
  EXPECT_EQ(r.spray_acks[0].source, dyx3_interfaces::msg::SprayActuatorCommand::SOURCE_WATCHDOG);
  for (const auto& c : r.cmds) EXPECT_NE(c.command, 187U);  // nothing was sent to the FCU
  // an uninterpretable request is never sent either
  r.fcu_answers_handshake = true;
  r.bring_up();
  r.run(0.2);
  r.clear();
  r.spray_acks.clear();
  m.actuator_set_index = 9;  // out of range
  r.p_spray->publish(m);
  r.pump(200);
  ASSERT_EQ(r.spray_acks.size(), 1U);
  EXPECT_FALSE(r.spray_acks[0].success);
  for (const auto& c : r.cmds) EXPECT_NE(c.command, 187U);
}

TEST(Px4LinkNode, TimesyncEvidenceIsPublishedAsValuesOnlyAndZeroedWhenTheSessionIsLost) {
  Rig r;
  r.ts_offset = -40000;  // the upstream #28519 symptom: ~40 ms right after boot
  r.ts_rtt = 1234;
  r.bring_up();
  r.run(0.5);
  EXPECT_TRUE(r.status.timesync_valid);
  EXPECT_EQ(r.status.timesync_offset_us, -40000);
  EXPECT_EQ(r.status.timesync_round_trip_us, 1234U);
  EXPECT_EQ(r.status.stale_topics_mask,
            0U);  // a 40 ms offset does not make the link unhealthy: values only, no gate
  EXPECT_TRUE(r.status.session_alive);
  r.ts_offset = 4000;
  r.run(0.3);
  EXPECT_EQ(r.status.timesync_offset_us, 4000);
  r.alive = false;
  r.run(1.5);
  EXPECT_FALSE(r.status.timesync_valid);
  EXPECT_EQ(r.status.timesync_offset_us, 0);  // a stale offset is never presented as current
}

TEST(Px4LinkNode, EstimatorHealthDefaultsUnhealthyUntilFlagsArrive) {
  Rig r;
  r.bring_up();
  r.run(0.3);
  EXPECT_TRUE(r.health.flags_valid);
  EXPECT_FALSE(r.health.test_ratios_valid);  // estimator_status is not on DDS
  r.alive = false;
  r.run(1.5);
  EXPECT_FALSE(r.health.flags_valid);
}

TEST(Px4LinkNode, VehicleStateFanOut) {
  Rig r;
  r.bring_up();
  r.run(0.3);
  EXPECT_TRUE(r.state.position_valid);
  EXPECT_TRUE(r.state.attitude_valid);
  EXPECT_FLOAT_EQ(r.state.north_m, 3.0F);
  EXPECT_FLOAT_EQ(r.state.east_m, 4.0F);
  EXPECT_FLOAT_EQ(r.state.heading_rad, 0.5F);
}
