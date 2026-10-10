// Fault-injection tests: a fake FCU (publishes /fmu/out, records /fmu/in) against the real node,
// with an injected clock. Run in a private DDS domain so it cannot talk to other test processes.
// Delivery is work-driven, never a fixed wall-clock wait: every publish lands in the reader
// before publish() returns (dds_test_support.hpp), so deliver() runs exactly the callbacks the
// published samples cause and then returns; the link clock only moves through `now`.
#include "dyx3_px4_link/px4_link_node.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <map>
#include <thread>

#include "dds_test_support.hpp"

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
    {"/fmu/out/vehicle_command_ack", "VehicleCommandAck"},
    {"/fmu/out/battery_status", "BatteryStatus"}};

void init_ctx(const std::shared_ptr<rclcpp::Context>& ctx) { dyx3_test::init_isolated(ctx); }

// The whole suite runs twice: px4_link_node_test in timer mode (event_driven=false, the timer-mode
// regression) and px4_link_node_event_test in event-driven mode (the production default). Tests
// that are about one mode set event_driven explicitly.
#ifdef DYX3_TEST_EVENT_DRIVEN
constexpr bool kSuiteEventDriven = true;
#else
constexpr bool kSuiteEventDriven = false;
#endif

struct Rig {
  static std::string unique_token_path() {
    static std::atomic<unsigned> serial{0};
    return "/tmp/dyx3_px4_link_test_tokens_" + std::to_string(getpid()) + "_" +
           std::to_string(serial.fetch_add(1));
  }
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
  rclcpp::Publisher<px4_msgs::msg::BatteryStatus>::SharedPtr p_bat;
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
  int state_count{0};
  dyx3_interfaces::msg::EstimatorHealth health;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> keep;
  rclcpp::Client<dyx3_interfaces::srv::SetOffboard>::SharedPtr cli_off;
  rclcpp::Client<dyx3_interfaces::srv::ArmDisarm>::SharedPtr cli_arm;
  uint64_t seq{0};
  bool fcu_answers_handshake{true};
  int hash_error_on{-1};  // lie about this topic's hash
  bool alive{true}, lp_alive{true}, att_alive{true};
  double att_yaw_rate{0.0};  // rad/s, NED (clockwise positive); yaw = rate * (now - 100)
  uint8_t nav_state{0}, arming_state{1};

  explicit Rig(const rclcpp::ParameterValue* extra = nullptr, const std::string& defs = "",
               const std::string& token_path = "",
               const std::vector<rclcpp::Parameter>& params = {}) {
    ctx = std::make_shared<rclcpp::Context>();
    init_ctx(ctx);
    rclcpp::NodeOptions no;
    no.context(ctx);
    const std::string dir = defs.empty() ? std::string(DYX3_FIXTURES) + "/msgdefs" : defs;
    no.append_parameter_override("msg_definitions_dir", dir);
    const std::string ack_state = token_path.empty() ? unique_token_path() : token_path;
    if (token_path.empty()) {
      std::ofstream state(ack_state);
      state << "2\n";
    }
    no.append_parameter_override("spray_ack_token_state_path", ack_state);
    if (extra != nullptr) {
    }
    bool mode_given = false;
    for (const auto& p : params) {
      no.append_parameter_override(p.get_name(), p.get_parameter_value());
      mode_given = mode_given || p.get_name() == "event_driven";
    }
    if (!mode_given) no.append_parameter_override("event_driven", kSuiteEventDriven);
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
    p_bat =
        fcu->create_publisher<px4_msgs::msg::BatteryStatus>("/fmu/out/battery_status_v1", sensor);
    p_att =
        fcu->create_publisher<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", sensor);
    p_fl = fcu->create_publisher<px4_msgs::msg::EstimatorStatusFlags>(
        "/fmu/out/estimator_status_flags", sensor);
    p_gps =
        fcu->create_publisher<px4_msgs::msg::SensorGps>("/fmu/out/vehicle_gps_position", sensor);
    p_resp = fcu->create_publisher<px4_msgs::msg::MessageFormatResponse>(
        "/fmu/out/message_format_response", rclcpp::QoS(10).best_effort());
    // Every /fmu/out writer of the FCU is best effort (uxrce_dds_client/utilities.hpp): the fake
    // FCU must be too, or a reliable-only subscription would pass here and never match on target.
    p_ulog = fcu->create_publisher<px4_msgs::msg::UlogStream>("/fmu/out/ulog_stream",
                                                              rclcpp::QoS(16).best_effort());
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
        "/dyx3/vehicle_state", r1, [this](dyx3_interfaces::msg::VehicleState::ConstSharedPtr m) {
          state = *m;
          ++state_count;
        }));
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
          link->count_publishers("/dyx3/vehicle_state") > 0 &&
          // spray path: command in, VehicleCommand out, FCU ack in, spray ack out
          fcu->count_subscribers("/dyx3/spray/actuator_command") > 0 &&
          link->count_subscribers("/fmu/in/vehicle_command") > 0 &&
          fcu->count_subscribers("/fmu/out/vehicle_command_ack") > 0 &&
          link->count_subscribers("/dyx3/spray/actuator_ack") > 0) {
        // Every later deliver() relies on synchronous delivery: prove it, or fail here.
        ASSERT_TRUE(dyx3_test::delivery_is_synchronous(ctx));
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
      if (att_alive) {
        px4_msgs::msg::VehicleAttitude a;
        const double yaw = std::remainder(att_yaw_rate * (now - 100.0), 2.0 * 3.141592653589793);
        a.q = {static_cast<float>(std::cos(yaw / 2)), 0.0F, 0.0F,
               static_cast<float>(std::sin(yaw / 2))};
        a.timestamp_sample = static_cast<uint64_t>(std::llround(now * 1e6));
        p_att->publish(a);
      }
      px4_msgs::msg::EstimatorStatusFlags fl;
      fl.timestamp_sample = static_cast<uint64_t>(std::llround(now * 1e6));
      p_fl->publish(fl);
      px4_msgs::msg::SensorGps g;
      g.fix_type = 6;
      g.eph = 0.01F;
      p_gps->publish(g);
    }
    if (alive && lp_alive) {
      px4_msgs::msg::VehicleLocalPosition lp;
      lp.xy_valid = lp.v_xy_valid = lp.heading_good_for_control = true;
      lp.z_valid = true;  // v_z_valid stays false: the two vertical flags are mapped separately
      lp.x = 3.0F;
      lp.y = 4.0F;
      lp.heading = 0.5F;
      lp.timestamp_sample =
          static_cast<uint64_t>(std::llround(now * 1e6));  // a new sample per tick
      p_lp->publish(lp);
    }
  }
  builtin_interfaces::msg::Time cmd_pose_stamp{};  // IF-003 source_pose_sample_stamp
  void guard(uint8_t mode, float v, float yaw, float rate, bool valid = true) {
    dyx3_interfaces::msg::MotionSetpoint m;
    m.source_pose_sample_stamp = cmd_pose_stamp;
    m.seq = ++seq;
    m.mode = mode;
    m.speed_body_x = v;
    m.yaw_setpoint = yaw;
    m.yaw_rate_setpoint = rate;
    m.valid = valid;
    p_cmd->publish(m);
  }
  // Delivers everything published so far and runs every callback it causes (and the ones those
  // cause), then returns: nothing is left in flight. A negative check after deliver() ("nothing was
  // sent") is therefore exact, not a bet on a wait being long enough.
  void deliver() { dyx3_test::drain(*exec); }
  // Settle until `done` holds or the wall-clock deadline passes. For positive expectations
  // ("this message arrives"). The link clock (`now`) does not advance here, so no link timeout can
  // fire while waiting; the deadline only bounds a genuinely missing message.
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
  // Advance link-time by dt and run one cycle with the fake FCU alive, then deliver.
  void tick(double dt = 0.01, bool publish = true) {
    now += dt;
    if (publish) publish_fcu();
    deliver();  // the cycle sees this tick's samples
    if (link->params().event_driven) {
      link->on_timer(now);  // the writer timer; a guard command already cycled on arrival
    } else {
      link->step(now);
    }
    deliver();  // the recorders hold this cycle's output
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

TEST(Px4LinkNode, LoopOverrunWarningExpiresFromLastActualOverrun) {
  Rig r;
  r.bring_up();
  auto tick_with_command = [&](double dt) {
    r.guard(2, 0.3F, NaN, 0.1F);
    r.tick(dt);
  };
  for (int i = 0; i < 20; ++i) tick_with_command(0.01);
  EXPECT_EQ(r.status.loop_overrun_count, 0U);
  EXPECT_EQ(r.status.fault, dyx3_interfaces::msg::Px4LinkStatus::FAULT_NONE);

  tick_with_command(0.11);  // > 1.5 of the 100 Hz period, but inside topic freshness bounds
  EXPECT_EQ(r.status.loop_overrun_count, 1U);
  EXPECT_EQ(r.status.fault, dyx3_interfaces::msg::Px4LinkStatus::FAULT_LOOP_OVERRUN);
  for (int i = 0; i < 50; ++i) tick_with_command(0.01);
  EXPECT_EQ(r.status.loop_overrun_count, 1U);
  EXPECT_EQ(r.status.fault, dyx3_interfaces::msg::Px4LinkStatus::FAULT_LOOP_OVERRUN);
  for (int i = 0; i < 60; ++i) tick_with_command(0.01);
  EXPECT_EQ(r.status.loop_overrun_count, 1U);  // lifetime diagnostic is retained
  EXPECT_EQ(r.status.fault, dyx3_interfaces::msg::Px4LinkStatus::FAULT_NONE);

  tick_with_command(0.11);
  EXPECT_EQ(r.status.loop_overrun_count, 2U);
  EXPECT_EQ(r.status.fault, dyx3_interfaces::msg::Px4LinkStatus::FAULT_LOOP_OVERRUN);
  const auto count = r.status.loop_overrun_count;
  const size_t speed_before = r.speed.size();
  // An unusable clock fails to zero: one explicit STOP setpoint per bad tick, not silence.
  r.link->step(std::numeric_limits<double>::quiet_NaN());
  r.deliver();
  ASSERT_EQ(r.speed.size(), speed_before + 1);
  EXPECT_EQ(r.speed.back().speed_body_x, 0.0F);
  r.link->step(r.now - 0.5);
  r.deliver();
  ASSERT_EQ(r.speed.size(), speed_before + 2);
  EXPECT_EQ(r.speed.back().speed_body_x, 0.0F);
  EXPECT_EQ(r.status.loop_overrun_count, count);
  tick_with_command(0.01);
  EXPECT_EQ(r.status.loop_overrun_count, count);
  EXPECT_EQ(r.status.fault, dyx3_interfaces::msg::Px4LinkStatus::FAULT_LOOP_OVERRUN);
}

TEST(Px4LinkNode, InvalidInjectedClockPublishesZeroThenNormalForwardingRecovers) {
  Rig r;
  r.bring_up();
  r.nav_state = 14;
  bool accepted = false;
  uint8_t reason = 0;
  ASSERT_TRUE(r.call_offboard(true, &accepted, &reason));
  ASSERT_TRUE(accepted);
  r.guard(2, 0.3F, NaN, 0.1F);
  r.tick();
  ASSERT_FALSE(r.speed.empty());
  ASSERT_FLOAT_EQ(r.speed.back().speed_body_x, 0.3F);

  const auto count = r.status.loop_overrun_count;
  for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(), r.now - 0.5}) {
    r.clear();
    r.link->step(invalid);
    r.deliver();
    ASSERT_EQ(r.ocm.size(), 1U);
    ASSERT_EQ(r.traj.size(), 1U);
    ASSERT_EQ(r.speed.size(), 1U);
    ASSERT_EQ(r.att_sp.size(), 1U);
    ASSERT_EQ(r.rate.size(), 1U);
    EXPECT_FLOAT_EQ(r.speed.back().speed_body_x, 0.0F);
    EXPECT_TRUE(std::isnan(r.att_sp.back().yaw_setpoint));
    EXPECT_FLOAT_EQ(r.rate.back().yaw_rate_setpoint, 0.0F);
    EXPECT_TRUE(all_nan3(r.traj.back().velocity));
    EXPECT_EQ(r.status.loop_overrun_count, count);
  }

  r.clear();
  r.guard(2, 0.3F, NaN, 0.1F);
  r.tick();
  ASSERT_FALSE(r.speed.empty());
  EXPECT_FLOAT_EQ(r.speed.back().speed_body_x, 0.3F);
  EXPECT_EQ(r.status.loop_overrun_count, count);
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
    r.deliver();
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

// XR-GPX-007: the guard's command reaches PX4 only in a confirmed OFFBOARD session; before the
// confirmation and after PX4 leaves OFFBOARD the heartbeat carries the explicit STOP.
TEST(Px4LinkNode, HeartbeatCarriesStopOutsideAnActiveOffboardSession) {
  Rig r;
  r.bring_up();
  r.nav_state = 0;  // PX4 has not entered OFFBOARD
  auto req = std::make_shared<dyx3_interfaces::srv::SetOffboard::Request>();
  req->enable = true;
  auto fut = r.cli_off->async_send_request(req);
  r.clear();
  for (int i = 0; i < 80; ++i) {  // prestream, then the mode request, without confirmation
    r.guard(2, 0.5F, NaN, 0.1F);
    r.tick();
  }
  ASSERT_FALSE(r.speed.empty());
  for (const auto& sp : r.speed) EXPECT_EQ(sp.speed_body_x, 0.0F);
  ASSERT_TRUE(
      std::any_of(r.cmds.begin(), r.cmds.end(), [](const auto& c) { return c.command == 176; }));
  r.nav_state = 14;
  for (int i = 0; i < 300 && fut.wait_for(0ms) != std::future_status::ready; ++i) {
    r.guard(2, 0.5F, NaN, 0.1F);
    r.tick();
  }
  ASSERT_EQ(fut.wait_for(0ms), std::future_status::ready);
  ASSERT_TRUE(fut.get()->accepted);
  r.guard(2, 0.5F, NaN, 0.1F);
  r.tick();
  r.guard(2, 0.5F, NaN, 0.1F);
  r.tick();
  EXPECT_FLOAT_EQ(r.speed.back().speed_body_x, 0.5F);  // Active: the guard's command goes through
  r.nav_state = 0;                                     // PX4 left OFFBOARD (its own failsafe)
  for (int i = 0; i < 20; ++i) {
    r.guard(2, 0.5F, NaN, 0.1F);
    r.tick();
  }
  r.clear();
  for (int i = 0; i < 50; ++i) {
    r.guard(2, 0.5F, NaN, 0.1F);
    r.tick();
  }
  ASSERT_FALSE(r.speed.empty());
  for (const auto& sp : r.speed) EXPECT_EQ(sp.speed_body_x, 0.0F);
  EXPECT_TRUE(std::none_of(r.cmds.begin(), r.cmds.end(), [](const auto& c) {
    return c.command == 176;
  }));  // Lost: OFFBOARD is not re-requested
}

// PXL-002: SetOffboard(false) while moving puts STOP on the wire at the next writer tick, keeps
// the heartbeat with STOP for offboard_disable_stop_s, then withdraws it.
TEST(Px4LinkNode, DisableOffboardStreamsStopBeforeWithdrawingTheHeartbeat) {
  Rig r;
  r.bring_up();
  r.nav_state = 14;
  bool acc = false;
  uint8_t rs = 0;
  ASSERT_TRUE(r.call_offboard(true, &acc, &rs));
  ASSERT_TRUE(acc);
  for (int i = 0; i < 10; ++i) {
    r.guard(2, 0.5F, NaN, 0.1F);
    r.tick();
  }
  ASSERT_FLOAT_EQ(r.speed.back().speed_body_x, 0.5F);
  r.clear();
  auto req = std::make_shared<dyx3_interfaces::srv::SetOffboard::Request>();
  req->enable = false;
  auto fut = r.cli_off->async_send_request(req);
  ASSERT_TRUE(r.pump_until([&] { return fut.wait_for(0ms) == std::future_status::ready; }));
  EXPECT_TRUE(fut.get()->accepted);
  r.guard(2, 0.5F, NaN, 0.1F);  // the guard still commands motion
  r.tick();                     // next writer tick
  ASSERT_TRUE(r.pump_until([&] { return !r.speed.empty(); }));
  ASSERT_EQ(r.speed.size(), 1U);
  EXPECT_EQ(r.speed.back().speed_body_x, 0.0F);
  EXPECT_EQ(r.rate.back().yaw_rate_setpoint, 0.0F);
  EXPECT_TRUE(std::isnan(r.att_sp.back().yaw_setpoint));
  for (int i = 0; i < 15; ++i) {  // 0.15 s more: still inside the 0.3 s window
    r.guard(2, 0.5F, NaN, 0.1F);
    r.tick();
  }
  EXPECT_GE(r.ocm.size(), 10U);
  EXPECT_EQ(r.ocm.size(), r.speed.size());
  for (const auto& sp : r.speed) EXPECT_EQ(sp.speed_body_x, 0.0F);
  EXPECT_TRUE(r.status.offboard_heartbeat_active);
  for (int i = 0; i < 20; ++i) {
    r.guard(2, 0.5F, NaN, 0.1F);
    r.tick();
  }
  r.deliver();
  const size_t after_window = r.ocm.size();
  for (const auto& sp : r.speed) EXPECT_EQ(sp.speed_body_x, 0.0F);
  for (int i = 0; i < 20; ++i) {
    r.guard(2, 0.5F, NaN, 0.1F);
    r.tick();
  }
  r.deliver();
  EXPECT_EQ(r.ocm.size(), after_window);  // heartbeat withdrawn after the window
  EXPECT_LE(after_window, 32U);
  EXPECT_FALSE(r.status.offboard_heartbeat_active);
}

// X-010: the shutdown path publishes the explicit STOP set while the heartbeat is live, and
// nothing when it was not running.
TEST(Px4LinkNode, ShutdownStopPublishesStopOnlyWhereTheHeartbeatWasLive) {
  Rig r;
  r.bring_up();
  r.run(0.1);
  r.clear();
  EXPECT_FALSE(r.link->publish_shutdown_stop());  // offboard never enabled: no stream started
  r.deliver();
  EXPECT_TRUE(r.ocm.empty());

  r.nav_state = 14;
  bool acc = false;
  uint8_t rs = 0;
  ASSERT_TRUE(r.call_offboard(true, &acc, &rs));
  ASSERT_TRUE(acc);
  for (int i = 0; i < 5; ++i) {
    r.guard(2, 0.5F, NaN, 0.1F);
    r.tick();
  }
  ASSERT_FLOAT_EQ(r.speed.back().speed_body_x, 0.5F);
  r.clear();
  r.guard(2, 0.5F, NaN, 0.1F);  // a fresh motion command is pending
  // Not pumped before the burst: in event-driven mode it is delivered (and would be written) by
  // the first pump below, after shutdown has started. It must not reach PX4 (X-010 with C4).
  for (size_t i = 1; i <= 30; ++i) {  // the recording readers keep depth 1: drain each one
    ASSERT_TRUE(r.link->publish_shutdown_stop());
    ASSERT_TRUE(r.pump_until([&] { return r.speed.size() >= i && r.ocm.size() >= i; })) << i;
  }
  r.deliver();
  EXPECT_EQ(r.speed.size(), 30U);  // the writer tick did not run in between
  for (const auto& sp : r.speed) EXPECT_EQ(sp.speed_body_x, 0.0F);
  EXPECT_EQ(r.rate.back().yaw_rate_setpoint, 0.0F);
  EXPECT_TRUE(std::isnan(r.att_sp.back().yaw_setpoint));
  EXPECT_TRUE(all_nan3(r.traj.back().velocity));
  EXPECT_TRUE(r.ocm.back().velocity);
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
  r.run(3.5);  // beyond the 3.0 s session limit (stale_timesync_s / stale_estimator_flags_s)
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

// Leaving OFFBOARD (prototype behaviour): after a release PX4 is sent to MANUAL, never during the
// STOP window, and no more once it has left OFFBOARD.
namespace {
int manual_mode_cmds(const std::vector<px4_msgs::msg::VehicleCommand>& cmds) {
  int n = 0;
  for (const auto& c : cmds)
    n += (c.command == 176 && c.param1 == 1.0F && c.param2 == 1.0F) ? 1 : 0;
  return n;
}
int arm_cmds(const std::vector<px4_msgs::msg::VehicleCommand>& cmds) {
  int n = 0;
  for (const auto& c : cmds) n += (c.command == 400 && c.param1 == 1.0F) ? 1 : 0;
  return n;
}
}  // namespace

TEST(Px4LinkNode, ReleasingOffboardSwitchesPx4ToManualAfterTheStopWindow) {
  Rig r;
  r.bring_up();
  r.nav_state = 14;
  bool acc = false;
  uint8_t rs = 0;
  ASSERT_TRUE(r.call_offboard(true, &acc, &rs));
  ASSERT_TRUE(acc);
  r.run(0.1);
  r.clear();
  auto req = std::make_shared<dyx3_interfaces::srv::SetOffboard::Request>();
  req->enable = false;
  auto fut = r.cli_off->async_send_request(req);
  ASSERT_TRUE(r.pump_until([&] { return fut.wait_for(0ms) == std::future_status::ready; }));
  ASSERT_TRUE(fut.get()->accepted);
  r.run(0.2);  // inside the 0.3 s STOP window: no mode change yet
  r.deliver();
  EXPECT_EQ(manual_mode_cmds(r.cmds), 0);
  r.run(0.3);  // window over, PX4 still reports OFFBOARD
  r.deliver();
  EXPECT_EQ(manual_mode_cmds(r.cmds), 1);
  r.nav_state = 1;  // PX4 is in MANUAL
  r.run(1.0);
  r.deliver();
  EXPECT_EQ(manual_mode_cmds(r.cmds), 1);  // not repeated once it has left OFFBOARD
}

TEST(Px4LinkNode, ArmWithAStaleOffboardModeLeavesItForManualFirst) {
  Rig r;
  r.bring_up();
  r.nav_state = 14;  // left over: PX4 in OFFBOARD, our heartbeat never started
  r.run(0.1);
  r.clear();
  auto req = std::make_shared<dyx3_interfaces::srv::ArmDisarm::Request>();
  req->arm = true;
  auto fut = r.cli_arm->async_send_request(req);
  r.run(0.2);
  r.deliver();
  EXPECT_EQ(manual_mode_cmds(r.cmds), 1);
  EXPECT_EQ(arm_cmds(r.cmds), 0);  // the arm is held until PX4 has left OFFBOARD
  r.nav_state = 1;
  r.arming_state = 2;  // the fake FCU arms once asked
  for (int i = 0; i < 300 && fut.wait_for(0ms) != std::future_status::ready; ++i) r.tick();
  ASSERT_EQ(fut.wait_for(0ms), std::future_status::ready);
  EXPECT_TRUE(fut.get()->accepted);
  EXPECT_EQ(arm_cmds(r.cmds), 1);
}

TEST(Px4LinkNode, ArmIsRefusedWhenPx4NeverLeavesAStaleOffboard) {
  Rig r;
  r.bring_up();
  r.nav_state = 14;
  r.run(0.1);
  r.clear();
  auto req = std::make_shared<dyx3_interfaces::srv::ArmDisarm::Request>();
  req->arm = true;
  auto fut = r.cli_arm->async_send_request(req);
  for (int i = 0; i < 300 && fut.wait_for(0ms) != std::future_status::ready; ++i) r.tick();
  ASSERT_EQ(fut.wait_for(0ms), std::future_status::ready);
  auto resp = fut.get();
  EXPECT_FALSE(resp->accepted);
  EXPECT_EQ(resp->reason_code, dyx3_interfaces::srv::ArmDisarm::Response::REASON_TIMEOUT);
  EXPECT_EQ(arm_cmds(r.cmds), 0);  // never armed into OFFBOARD without a signal
  EXPECT_GE(manual_mode_cmds(r.cmds), 2);
}

TEST(Px4LinkNode, UlogChunksAreAckedThenRepublished) {
  Rig r;
  r.bring_up();
  r.run(0.1);
  ASSERT_TRUE(r.pump_until([&] { return r.fcu->count_subscribers("/fmu/out/ulog_stream") > 0; }));
  // A best-effort FCU writer must actually match the link's reader (XR-GPX-004).
  ASSERT_TRUE(r.pump_until([&] { return r.p_ulog->get_subscription_count() > 0; }));
  px4_msgs::msg::UlogStream u;
  u.msg_sequence = 7;
  u.flags = px4_msgs::msg::UlogStream::FLAGS_NEED_ACK;
  u.length = 3;
  u.data[0] = 1;
  u.data[1] = 2;
  u.data[2] = 3;
  r.p_ulog->publish(u);
  r.deliver();
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
  r.deliver();
  EXPECT_TRUE(r.inject.empty());
  r.run(0.1);
  EXPECT_EQ(r.status.rtcm_chunks_dropped, 1U);
  EXPECT_EQ(r.status.rtcm_chunks_accepted, 0U);
  r.fcu_answers_handshake = true;
  r.bring_up();
  r.run(0.1);
  r.p_rtcm->publish(d);
  r.deliver();
  ASSERT_EQ(r.inject.size(), 1U);
  EXPECT_EQ(r.inject[0].len, 120);
  EXPECT_EQ(r.inject[0].flags, 1);
  EXPECT_EQ(r.inject[0].data[119], 0xD3);
  r.run(0.1);
  EXPECT_EQ(r.status.rtcm_chunks_accepted, 1U);
  d.data.assign(301, 0x01);
  r.p_rtcm->publish(d);
  r.deliver();
  EXPECT_EQ(r.inject.size(), 1U);  // oversize rejected, not truncated
  r.run(0.1);
  EXPECT_EQ(r.status.rtcm_chunks_dropped, 2U);
  EXPECT_EQ(r.status.rtcm_chunks_accepted + r.status.rtcm_chunks_dropped, 3U);
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
  r.deliver();
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
  const auto actuator_command =
      std::find_if(r.cmds.rbegin(), r.cmds.rend(), [](const auto& c) { return c.command == 187; });
  ASSERT_NE(actuator_command, r.cmds.rend());
  EXPECT_NE(actuator_command->source_component, 1U);
  EXPECT_TRUE(r.spray_acks.empty());  // no ack until the FCU answers
  px4_msgs::msg::VehicleCommandAck a;
  a.target_system = 1;
  a.command = 187;
  a.target_component = actuator_command->source_component;
  a.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS;
  r.p_ack->publish(a);
  r.deliver();
  EXPECT_TRUE(r.spray_acks.empty());  // in progress is not a result
  a.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  r.p_ack->publish(a);
  r.deliver();
  ASSERT_EQ(r.spray_acks.size(), 1U);
  EXPECT_EQ(r.spray_acks[0].seq, 41U);
  EXPECT_TRUE(r.spray_acks[0].success);
  // servo backend and a rejected result
  m.seq = 42;
  m.backend = dyx3_interfaces::msg::SprayActuatorCommand::BACKEND_SERVO_PWM;
  m.servo_instance = 3;
  m.pwm_us = 60000;  // out of range: clamped to 2200
  r.p_spray->publish(m);
  r.deliver();
  bool saw_servo = false;
  for (const auto& c : r.cmds) {
    if (c.command == 183) {
      saw_servo = true;
      EXPECT_FLOAT_EQ(c.param1, 3.0F);
      EXPECT_FLOAT_EQ(c.param2, 2200.0F);
    }
  }
  EXPECT_TRUE(saw_servo);
  const auto servo_command =
      std::find_if(r.cmds.rbegin(), r.cmds.rend(), [](const auto& c) { return c.command == 183; });
  ASSERT_NE(servo_command, r.cmds.rend());
  EXPECT_NE(servo_command->source_component, actuator_command->source_component);
  a.command = 183;
  a.target_component = servo_command->source_component;
  a.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED;
  r.p_ack->publish(a);
  r.deliver();
  ASSERT_EQ(r.spray_acks.size(), 2U);
  EXPECT_EQ(r.spray_acks[1].seq, 42U);
  EXPECT_FALSE(r.spray_acks[1].success);
}

TEST(Px4LinkNode, TenThousandIdenticalOnReassertsReuseTheConfirmedLogicalTransaction) {
  using Cmd = dyx3_interfaces::msg::SprayActuatorCommand;
  const std::string path = Rig::unique_token_path();
  {
    std::ofstream state(path);
    state << "2\n";
  }
  Rig r(nullptr, "", path);
  r.bring_up();
  r.run(0.2);
  r.clear();

  Cmd on;
  on.seq = 42;
  on.source = Cmd::SOURCE_CONTROLLER;
  on.backend = Cmd::BACKEND_ACTUATOR;
  on.on = true;
  on.actuator_set_index = 2;
  on.value = 1.0F;
  r.p_spray->publish(on);
  r.deliver();
  ASSERT_EQ(r.cmds.size(), 1U);
  const uint16_t token = r.cmds.back().source_component;

  px4_msgs::msg::VehicleCommandAck ack;
  ack.target_system = 1;
  ack.command = 187;
  ack.target_component = token;
  ack.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  r.p_ack->publish(ack);
  r.deliver();
  ASSERT_EQ(r.spray_acks.size(), 1U);
  ASSERT_TRUE(r.spray_acks.front().success);

  for (int i = 0; i < 10000; ++i) {
    r.p_spray->publish(on);
    r.deliver();
    ASSERT_EQ(r.cmds.size(), static_cast<size_t>(i + 2)) << i;
  }
  r.deliver();

  ASSERT_EQ(r.cmds.size(), 10001U);
  for (const auto& wire : r.cmds) {
    EXPECT_EQ(wire.source_system, 1U);
    EXPECT_EQ(wire.source_component, token);
  }
  EXPECT_EQ(r.spray_acks.size(), 1U);  // the original logical transaction was already confirmed
  EXPECT_EQ(r.link->spray_identities_used(), 1U);
  unlink(path.c_str());
}

// XR-GPX-005: ACKs drawn by reasserts of the in-flight or confirmed epoch are expected and are
// not counted as unmatched; an ACK for an identity no epoch holds still is.
TEST(Px4LinkNode, AcksOfReassertedSprayCommandsAreExpectedNotUnmatched) {
  using Cmd = dyx3_interfaces::msg::SprayActuatorCommand;
  Rig r;
  r.bring_up();
  r.run(0.2);
  r.clear();
  Cmd on;
  on.seq = 3;
  on.source = Cmd::SOURCE_CONTROLLER;
  on.backend = Cmd::BACKEND_ACTUATOR;
  on.on = true;
  on.actuator_set_index = 1;
  on.value = 1.0F;
  r.p_spray->publish(on);
  ASSERT_TRUE(r.pump_until([&] { return r.cmds.size() >= 1U; }));
  r.p_spray->publish(on);  // reassert while in flight: a second physical send
  ASSERT_TRUE(r.pump_until([&] { return r.cmds.size() >= 2U; }));
  px4_msgs::msg::VehicleCommandAck ack;
  ack.command = 187;
  ack.target_system = r.cmds.back().source_system;
  ack.target_component = r.cmds.back().source_component;
  ack.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  r.p_ack->publish(ack);  // answers the first send: completes the transaction
  ASSERT_TRUE(r.pump_until([&] { return r.spray_acks.size() >= 1U; }));
  r.p_ack->publish(ack);  // answers the second send
  r.deliver();
  for (int i = 0; i < 5; ++i) {  // reasserts of the confirmed epoch, each answered by the FCU
    r.p_spray->publish(on);
    ASSERT_TRUE(r.pump_until([&] { return r.cmds.size() >= static_cast<size_t>(3 + i); }));
    r.p_ack->publish(ack);
    r.deliver();
  }
  r.deliver();
  EXPECT_EQ(r.spray_acks.size(), 1U);
  EXPECT_TRUE(r.spray_acks[0].success);
  EXPECT_EQ(r.link->spray_late_ack_count(), 0U);
  ack.target_component = static_cast<uint16_t>(ack.target_component + 500);  // nobody's identity
  r.p_ack->publish(ack);
  ASSERT_TRUE(r.pump_until([&] { return r.link->spray_late_ack_count() == 1U; }));
  r.tick(0.11);
  EXPECT_EQ(r.status.spray_unmatched_ack_count, 1U);
}

TEST(Px4LinkNode, TenThousandWatchdogOffReassertsUseOnePair) {
  using Cmd = dyx3_interfaces::msg::SprayActuatorCommand;
  Rig r;
  r.bring_up();
  r.run(0.2);
  r.clear();
  Cmd off;
  off.seq = 77;
  off.source = Cmd::SOURCE_WATCHDOG;
  off.backend = Cmd::BACKEND_ACTUATOR;
  off.on = false;
  off.actuator_set_index = 1;
  off.value = -1.0F;
  r.p_spray->publish(off);
  r.deliver();
  ASSERT_EQ(r.cmds.size(), 1U);
  const auto system = r.cmds.back().source_system;
  const auto component = r.cmds.back().source_component;
  px4_msgs::msg::VehicleCommandAck ack;
  ack.command = 187;
  ack.target_system = system;
  ack.target_component = component;
  ack.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  r.p_ack->publish(ack);
  r.deliver();
  for (int i = 0; i < 10000; ++i) {
    r.p_spray->publish(off);
    r.deliver();
    ASSERT_EQ(r.cmds.size(), static_cast<size_t>(i + 2)) << i;
  }
  r.deliver();
  ASSERT_EQ(r.cmds.size(), 10001U);
  for (const auto& wire : r.cmds) {
    EXPECT_EQ(wire.source_system, system);
    EXPECT_EQ(wire.source_component, component);
  }
  EXPECT_EQ(r.link->spray_identities_used(), 1U);
}

TEST(Px4LinkNode, BothAckTargetFieldsMustMatchAcrossPairBoundary) {
  using Cmd = dyx3_interfaces::msg::SprayActuatorCommand;
  const std::string path = Rig::unique_token_path();
  {
    std::ofstream state(path);
    state << "v2 998\n";
  }
  Rig r(nullptr, "", path);
  r.bring_up();
  r.run(0.2);
  r.clear();
  Cmd on;
  on.seq = 1;
  on.source = Cmd::SOURCE_CONTROLLER;
  on.backend = Cmd::BACKEND_ACTUATOR;
  on.on = true;
  on.actuator_set_index = 1;
  on.value = 1.0F;
  r.p_spray->publish(on);
  r.deliver();
  ASSERT_EQ(r.cmds.size(), 1U);
  EXPECT_EQ(r.cmds.back().source_system, 2U);
  EXPECT_EQ(r.cmds.back().source_component, 2U);
  px4_msgs::msg::VehicleCommandAck ack;
  ack.command = 187;
  ack.target_system = 1;
  ack.target_component = 2;
  ack.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  r.p_ack->publish(ack);  // stale system-1/comp-2 ACK
  r.deliver();
  EXPECT_TRUE(r.spray_acks.empty());
  EXPECT_EQ(r.link->spray_late_ack_count(), 1U);
  ack.target_system = 2;
  r.p_ack->publish(ack);
  r.deliver();
  ASSERT_EQ(r.spray_acks.size(), 1U);
  EXPECT_TRUE(r.spray_acks.back().success);
  unlink(path.c_str());
}

TEST(Px4LinkNode, AnEarlierIdenticalOffTransmissionCanProveItsEpoch) {
  using Cmd = dyx3_interfaces::msg::SprayActuatorCommand;
  Rig r;
  r.bring_up();
  r.run(0.2);
  r.clear();
  Cmd off;
  off.seq = 40;
  off.source = Cmd::SOURCE_WATCHDOG;
  off.backend = Cmd::BACKEND_ACTUATOR;
  off.on = false;
  off.actuator_set_index = 1;
  off.value = -1.0F;
  r.p_spray->publish(off);
  r.deliver();
  ASSERT_EQ(r.cmds.size(), 1U);
  const auto first = r.cmds.back();
  r.p_spray->publish(off);
  r.deliver();
  ASSERT_EQ(r.cmds.size(), 2U);
  EXPECT_EQ(r.cmds.back().source_system, first.source_system);
  EXPECT_EQ(r.cmds.back().source_component, first.source_component);
  EXPECT_EQ(r.link->spray_identities_used(), 1U);
  px4_msgs::msg::VehicleCommandAck ack;
  ack.command = 187;
  ack.target_system = first.source_system;
  ack.target_component = first.source_component;
  ack.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  r.p_ack->publish(ack);  // ACK for the earlier identical physical transmission
  r.deliver();
  ASSERT_EQ(r.spray_acks.size(), 1U);
  EXPECT_TRUE(r.spray_acks.back().success);
  off.seq = 41;
  off.value = -0.5F;
  r.p_spray->publish(off);
  r.deliver();
  ASSERT_EQ(r.cmds.size(), 3U);
  const auto second = r.cmds.back();
  EXPECT_NE(second.source_component, first.source_component);
  r.p_ack->publish(ack);  // a later ACK for the old OFF epoch
  r.deliver();
  EXPECT_EQ(r.spray_acks.size(), 1U);
  ack.target_component = second.source_component;
  r.p_ack->publish(ack);
  r.deliver();
  ASSERT_EQ(r.spray_acks.size(), 2U);
  EXPECT_TRUE(r.spray_acks.back().success);
}

TEST(Px4LinkNode, WatchdogOffRetiresTheControllerOnHeartbeatItDisplaced) {
  using Cmd = dyx3_interfaces::msg::SprayActuatorCommand;
  Rig r;
  r.bring_up();
  r.run(0.2);
  r.clear();
  Cmd controller;
  controller.seq = 10;
  controller.source = Cmd::SOURCE_CONTROLLER;
  controller.backend = Cmd::BACKEND_ACTUATOR;
  controller.on = true;
  controller.actuator_set_index = 1;
  controller.value = 1.0F;
  r.p_spray->publish(controller);
  ASSERT_TRUE(r.pump_until([&] { return !r.cmds.empty(); }));
  ASSERT_EQ(r.cmds.size(), 1U);
  px4_msgs::msg::VehicleCommandAck ack;
  ack.command = 187;
  ack.target_system = r.cmds.back().source_system;
  ack.target_component = r.cmds.back().source_component;
  ack.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  r.p_ack->publish(ack);
  ASSERT_TRUE(r.pump_until([&] { return r.spray_acks.size() >= 1; }));  // ON confirmed
  Cmd watchdog = controller;
  watchdog.seq = 20;
  watchdog.source = Cmd::SOURCE_WATCHDOG;
  watchdog.on = false;
  watchdog.value = -1.0F;
  r.p_spray->publish(watchdog);
  ASSERT_TRUE(r.pump_until([&] { return r.cmds.size() >= 2; }));
  ASSERT_EQ(r.cmds.size(), 2U);
  ack.target_component = r.cmds.back().source_component;
  r.p_ack->publish(ack);
  ASSERT_TRUE(r.pump_until([&] { return r.spray_acks.size() >= 2; }));  // OFF confirmed
  r.p_spray->publish(controller);  // stale heartbeat from the displaced ON epoch
  r.deliver();                     // negative check: nothing may be sent
  EXPECT_EQ(r.cmds.size(), 2U);
  controller.seq = 11;  // a new controller verdict after safety recovery
  r.p_spray->publish(controller);
  ASSERT_TRUE(r.pump_until([&] { return r.cmds.size() >= 3; }));
  ASSERT_EQ(r.cmds.size(), 3U);
  EXPECT_EQ(r.link->spray_identities_used(), 3U);
}

TEST(Px4LinkNode, OnToOffIsANewTransactionAndOldOnAckCannotConfirmOff) {
  using Cmd = dyx3_interfaces::msg::SprayActuatorCommand;
  Rig r;
  r.bring_up();
  r.run(0.2);
  r.clear();

  Cmd request;
  request.seq = 42;
  request.source = Cmd::SOURCE_CONTROLLER;
  request.backend = Cmd::BACKEND_ACTUATOR;
  request.on = true;
  request.actuator_set_index = 2;
  request.value = 1.0F;
  r.p_spray->publish(request);
  r.deliver();
  ASSERT_EQ(r.cmds.size(), 1U);
  const uint16_t on_token = r.cmds.back().source_component;

  px4_msgs::msg::VehicleCommandAck ack;
  ack.target_system = 1;
  ack.command = 187;
  ack.target_component = on_token;
  ack.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  r.p_ack->publish(ack);
  r.deliver();
  ASSERT_EQ(r.spray_acks.size(), 1U);

  request.seq = 43;
  request.on = false;
  request.value = -1.0F;
  r.p_spray->publish(request);
  r.deliver();
  ASSERT_EQ(r.cmds.size(), 2U);
  const uint16_t off_token = r.cmds.back().source_component;
  EXPECT_NE(off_token, on_token);

  ack.target_component = on_token;
  r.p_ack->publish(ack);
  r.deliver();
  ASSERT_EQ(r.spray_acks.size(), 1U);
  EXPECT_EQ(r.link->spray_late_ack_count(), 1U);

  ack.target_component = off_token;
  r.p_ack->publish(ack);
  r.deliver();
  ASSERT_EQ(r.spray_acks.size(), 2U);
  EXPECT_EQ(r.spray_acks.back().seq, 43U);
  EXPECT_TRUE(r.spray_acks.back().success);
}

TEST(Px4LinkNode, ActuatorMappingChangeAndWatchdogOffIdentityAreHandledPrecisely) {
  using Cmd = dyx3_interfaces::msg::SprayActuatorCommand;
  const std::string path = Rig::unique_token_path();
  {
    std::ofstream state(path);
    state << "2\n";
  }
  Rig r(nullptr, "", path);
  r.bring_up();
  r.run(0.2);
  r.clear();

  Cmd request;
  request.seq = 1;
  request.source = Cmd::SOURCE_CONTROLLER;
  request.backend = Cmd::BACKEND_ACTUATOR;
  request.on = true;
  request.actuator_set_index = 1;
  request.value = 0.8F;
  r.p_spray->publish(request);
  r.deliver();
  ASSERT_EQ(r.cmds.size(), 1U);
  px4_msgs::msg::VehicleCommandAck ack;
  ack.target_system = 1;
  ack.command = 187;
  ack.target_component = r.cmds.back().source_component;
  ack.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  r.p_ack->publish(ack);
  r.deliver();

  request.actuator_set_index = 2;  // same seq and intent, a different physical actuator mapping
  r.p_spray->publish(request);
  r.deliver();
  ASSERT_EQ(r.cmds.size(), 2U);
  const uint16_t mapping_token = r.cmds.back().source_component;
  EXPECT_NE(mapping_token, ack.target_component);
  ack.target_component = mapping_token;
  r.p_ack->publish(ack);
  r.deliver();

  Cmd watchdog = request;
  watchdog.seq = 77;
  watchdog.source = Cmd::SOURCE_WATCHDOG;
  watchdog.on = false;
  watchdog.value = -1.0F;
  r.p_spray->publish(watchdog);
  r.deliver();
  ASSERT_EQ(r.cmds.size(), 3U);
  const uint16_t watchdog_token = r.cmds.back().source_component;
  ack.target_component = watchdog_token;
  r.p_ack->publish(ack);
  r.deliver();

  for (int i = 0; i < 100; ++i) {
    r.p_spray->publish(watchdog);
    r.deliver();
  }
  EXPECT_EQ(r.cmds.size(), 103U);
  EXPECT_EQ(r.link->spray_identities_used(), 3U);
  unlink(path.c_str());
}

TEST(Px4LinkNode, ExhaustedIdentityCannotConfirmWatchdogOff) {
  const std::string path = Rig::unique_token_path();
  {
    std::ofstream state(path);
    state << "v2 254490\n";
  }
  Rig r(nullptr, "", path);
  r.bring_up();
  r.run(0.2);
  r.clear();

  dyx3_interfaces::msg::SprayActuatorCommand off;
  off.seq = 991;
  off.source = dyx3_interfaces::msg::SprayActuatorCommand::SOURCE_WATCHDOG;
  off.backend = dyx3_interfaces::msg::SprayActuatorCommand::BACKEND_ACTUATOR;
  off.on = false;
  off.actuator_set_index = 1;
  off.value = 0.0F;
  r.p_spray->publish(off);
  r.deliver();

  EXPECT_TRUE(std::none_of(r.cmds.begin(), r.cmds.end(),
                           [](const auto& c) { return c.command == 187 || c.command == 183; }));
  ASSERT_EQ(r.spray_acks.size(), 1U);
  EXPECT_EQ(r.spray_acks.front().seq, off.seq);
  EXPECT_FALSE(r.spray_acks.front().success);
  EXPECT_EQ(r.spray_acks.front().result,
            dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);
  EXPECT_TRUE(r.link->spray_identities_exhausted());
  EXPECT_EQ(r.link->spray_identities_remaining(), 0U);
  r.tick(0.11);
  EXPECT_TRUE(r.status.spray_identities_exhausted);
  EXPECT_EQ(r.status.spray_identities_remaining, 0U);
  unlink(path.c_str());
}

TEST(Px4LinkNode, SprayTransactionsSerializeAndWatchdogOffPreemptsQueuedAndInFlightOn) {
  using Cmd = dyx3_interfaces::msg::SprayActuatorCommand;
  Rig r;
  r.bring_up();
  r.run(0.2);
  r.clear();

  auto publish = [&](uint32_t seq, uint8_t source, bool on, float value) {
    Cmd m;
    m.seq = seq;
    m.source = source;
    m.backend = Cmd::BACKEND_ACTUATOR;
    m.on = on;
    m.actuator_set_index = 1;
    m.value = value;
    r.p_spray->publish(m);
    r.deliver();
  };
  auto last_spray_command = [&]() {
    return std::find_if(r.cmds.rbegin(), r.cmds.rend(),
                        [](const auto& c) { return c.command == 187 || c.command == 183; });
  };
  auto spray_sent = [&]() {
    size_t sent = 0;
    for (const auto& c : r.cmds)
      if (c.command == 183 || c.command == 187) ++sent;
    return sent;
  };

  publish(1, Cmd::SOURCE_CONTROLLER, true, 1.0F);
  ASSERT_TRUE(r.pump_until([&] { return spray_sent() >= 1U; }));
  ASSERT_NE(last_spray_command(), r.cmds.rend());
  const auto first_token = last_spray_command()->source_component;
  ASSERT_EQ(last_spray_command()->command, 187U);
  for (uint32_t seq = 2; seq <= 10; ++seq)
    publish(seq, Cmd::SOURCE_CONTROLLER, true, 0.8F);  // reassert flood; only newest remains queued
  ASSERT_TRUE(r.pump_until([&] { return r.spray_acks.size() >= 8U; }));
  EXPECT_EQ(spray_sent(), 1U);  // only one spray VehicleCommand may be in flight
  ASSERT_EQ(r.spray_acks.size(), 8U);
  for (size_t i = 0; i < r.spray_acks.size(); ++i) {
    EXPECT_EQ(r.spray_acks[i].seq, i + 2);
    EXPECT_FALSE(r.spray_acks[i].success);
  }

  // Watchdog OFF purges the queued ON and pre-empts the unacknowledged in-flight ON: it goes on the
  // wire at once instead of waiting for the ON's ACK or timeout.
  publish(11, Cmd::SOURCE_WATCHDOG, false, -1.0F);
  ASSERT_TRUE(r.pump_until([&] { return spray_sent() >= 2U && r.spray_acks.size() >= 10U; }));
  EXPECT_EQ(spray_sent(), 2U);
  ASSERT_EQ(r.spray_acks.size(), 10U);
  EXPECT_EQ(r.spray_acks[8].seq, 10U);  // queued ON purged
  EXPECT_EQ(r.spray_acks[9].seq, 1U);   // in-flight ON pre-empted
  EXPECT_FALSE(r.spray_acks[9].success);
  EXPECT_EQ(r.spray_acks[9].result, dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);
  const auto watchdog_off = *last_spray_command();
  ASSERT_EQ(watchdog_off.command, 187U);
  EXPECT_EQ(watchdog_off.param1, -1.0F);
  EXPECT_EQ(watchdog_off.source_component, 3U);
  EXPECT_NE(watchdog_off.source_component, first_token);

  // A late ACK for the pre-empted controller ON cannot confirm watchdog OFF.
  px4_msgs::msg::VehicleCommandAck ack;
  ack.target_system = 1;
  ack.command = 187;
  ack.target_component = first_token;
  ack.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  r.p_ack->publish(ack);
  r.deliver();
  ASSERT_EQ(r.spray_acks.size(), 10U);
  ack.target_component = watchdog_off.source_component;
  r.p_ack->publish(ack);
  ASSERT_TRUE(r.pump_until([&] { return r.spray_acks.size() >= 11U; }));
  ASSERT_EQ(r.spray_acks.size(), 11U);
  EXPECT_EQ(r.spray_acks.back().seq, 11U);
  EXPECT_EQ(r.spray_acks.back().source, Cmd::SOURCE_WATCHDOG);
  EXPECT_TRUE(r.spray_acks.back().success);
}

// XR-GPX-001: an OFF never waits behind an ON whose ACK was lost. The link clock does not advance
// between the OFF request and its dispatch, so no transaction timeout is involved.
TEST(Px4LinkNode, OffPreemptsAnUnacknowledgedInFlightOnWithinOneTick) {
  using Cmd = dyx3_interfaces::msg::SprayActuatorCommand;
  for (const uint8_t off_source : {Cmd::SOURCE_WATCHDOG, Cmd::SOURCE_CONTROLLER}) {
    Rig r;
    r.bring_up();
    r.run(0.2);
    r.clear();
    Cmd on;
    on.seq = 5;
    on.source = Cmd::SOURCE_CONTROLLER;
    on.backend = Cmd::BACKEND_ACTUATOR;
    on.on = true;
    on.actuator_set_index = 1;
    on.value = 1.0F;
    r.p_spray->publish(on);
    ASSERT_TRUE(r.pump_until([&] { return r.cmds.size() >= 1U; }));
    const auto on_wire = r.cmds.back();
    r.tick();  // the ON stays in flight: no ACK from the FCU

    Cmd off = on;
    off.seq = 6;
    off.source = off_source;
    off.on = false;
    off.value = -1.0F;
    const double requested_at = r.now;
    r.p_spray->publish(off);
    r.tick();  // one writer tick
    ASSERT_TRUE(r.pump_until([&] { return r.cmds.size() >= 2U && r.spray_acks.size() >= 1U; }));
    EXPECT_NEAR(r.now - requested_at, 0.01, 1e-9);
    const auto off_wire = r.cmds.back();
    EXPECT_EQ(off_wire.command, 187U);
    EXPECT_FLOAT_EQ(off_wire.param1, -1.0F);
    EXPECT_NE(off_wire.source_component, on_wire.source_component);
    ASSERT_EQ(r.spray_acks.size(), 1U);
    EXPECT_EQ(r.spray_acks[0].seq, 5U);  // the pre-empted ON fails
    EXPECT_FALSE(r.spray_acks[0].success);
    EXPECT_EQ(r.spray_acks[0].result, dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);

    px4_msgs::msg::VehicleCommandAck ack;
    ack.command = 187;
    ack.target_system = off_wire.source_system;
    ack.target_component = off_wire.source_component;
    ack.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
    r.p_ack->publish(ack);
    ASSERT_TRUE(r.pump_until([&] { return r.spray_acks.size() >= 2U; }));
    EXPECT_EQ(r.spray_acks.back().seq, 6U);
    EXPECT_EQ(r.spray_acks.back().source, off_source);
    EXPECT_TRUE(r.spray_acks.back().success);
  }
}

TEST(Px4LinkNode, UnansweredSprayTransactionTimesOutAfterTheConfiguredWindow) {
  using Cmd = dyx3_interfaces::msg::SprayActuatorCommand;
  Rig r;
  r.bring_up();
  r.run(0.2);
  r.clear();
  EXPECT_DOUBLE_EQ(r.link->params().spray_transaction_timeout_s, 0.3);
  Cmd on;
  on.seq = 9;
  on.source = Cmd::SOURCE_CONTROLLER;
  on.backend = Cmd::BACKEND_ACTUATOR;
  on.on = true;
  on.actuator_set_index = 1;
  on.value = 1.0F;
  r.p_spray->publish(on);
  ASSERT_TRUE(r.pump_until([&] { return r.cmds.size() >= 1U; }));
  const auto first = r.cmds.back();
  r.tick(0.15);
  r.p_spray->publish(on);  // a reassert while in flight is sent again with the same identity
  ASSERT_TRUE(r.pump_until([&] { return r.cmds.size() >= 2U; }));
  EXPECT_EQ(r.cmds.back().source_component, first.source_component);
  EXPECT_EQ(r.cmds.back().source_system, first.source_system);
  r.tick(0.14);  // 0.29 s after dispatch: still in flight
  EXPECT_TRUE(r.spray_acks.empty());
  r.tick(0.02);  // 0.31 s
  ASSERT_TRUE(r.pump_until([&] { return !r.spray_acks.empty(); }));
  EXPECT_EQ(r.spray_acks[0].seq, 9U);
  EXPECT_FALSE(r.spray_acks[0].success);
  EXPECT_EQ(r.spray_acks[0].result, dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);
}

TEST(Px4LinkNode, RejectsNonPositiveSprayTransactionTimeout) {
  auto ctx = std::make_shared<rclcpp::Context>();
  init_ctx(ctx);
  rclcpp::NodeOptions no;
  no.context(ctx);
  no.append_parameter_override("spray_transaction_timeout_s", 0.0);
  no.append_parameter_override("msg_definitions_dir", std::string(DYX3_FIXTURES) + "/msgdefs");
  EXPECT_THROW(Px4LinkNode(no, nullptr, false), std::invalid_argument);
  ctx->shutdown("test done");
}

TEST(Px4LinkNode, LateSprayAckAfterTimeoutCannotMatchTheNextSameCommand) {
  using Cmd = dyx3_interfaces::msg::SprayActuatorCommand;
  Rig r;
  r.bring_up();
  r.run(0.2);
  r.clear();

  Cmd on;
  on.seq = 10;
  on.source = Cmd::SOURCE_CONTROLLER;
  on.backend = Cmd::BACKEND_ACTUATOR;
  on.on = true;
  on.actuator_set_index = 1;
  on.value = 1.0F;
  r.p_spray->publish(on);
  r.deliver();
  const auto old_command =
      std::find_if(r.cmds.rbegin(), r.cmds.rend(), [](const auto& c) { return c.command == 187; });
  ASSERT_NE(old_command, r.cmds.rend());
  const uint16_t old_token = old_command->source_component;

  r.tick(5.01);
  ASSERT_EQ(r.spray_acks.size(), 1U);
  EXPECT_EQ(r.spray_acks[0].seq, 10U);
  EXPECT_FALSE(r.spray_acks[0].success);
  EXPECT_EQ(r.spray_acks[0].result, dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);

  Cmd off = on;
  off.seq = 11;
  off.on = false;
  off.value = -1.0F;
  r.p_spray->publish(off);
  r.deliver();
  const auto current_command =
      std::find_if(r.cmds.rbegin(), r.cmds.rend(), [](const auto& c) { return c.command == 187; });
  ASSERT_NE(current_command, r.cmds.rend());
  ASSERT_NE(current_command->source_component, old_token);

  px4_msgs::msg::VehicleCommandAck ack;
  ack.target_system = 1;
  ack.command = 187;
  ack.target_component = old_token;
  ack.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  r.p_ack->publish(ack);
  r.deliver();
  EXPECT_EQ(r.spray_acks.size(), 1U);  // the old token cannot acknowledge seq 11
  EXPECT_EQ(r.link->spray_late_ack_count(), 1U);

  ack.target_component = current_command->source_component;
  r.p_ack->publish(ack);
  r.deliver();
  ASSERT_EQ(r.spray_acks.size(), 2U);
  EXPECT_EQ(r.spray_acks.back().seq, 11U);
  EXPECT_TRUE(r.spray_acks.back().success);
}

TEST(Px4LinkNode, WatchdogOffWinsQueuedControllerOffAndOldAckCannotConfirmIt) {
  using Cmd = dyx3_interfaces::msg::SprayActuatorCommand;
  Rig r;
  r.bring_up();
  r.run(0.2);
  r.clear();

  auto publish_off = [&](uint32_t seq, uint8_t source) {
    Cmd m;
    m.seq = seq;
    m.source = source;
    m.backend = Cmd::BACKEND_ACTUATOR;
    m.on = false;
    m.actuator_set_index = 1;
    m.value = -1.0F;
    r.p_spray->publish(m);
    r.deliver();
  };
  auto is_187 = [](const auto& c) { return c.command == 187; };
  publish_off(20, Cmd::SOURCE_CONTROLLER);
  ASSERT_TRUE(r.pump_until([&] { return std::any_of(r.cmds.begin(), r.cmds.end(), is_187); }));
  auto first =
      std::find_if(r.cmds.rbegin(), r.cmds.rend(), [](const auto& c) { return c.command == 187; });
  ASSERT_NE(first, r.cmds.rend());
  const uint16_t first_token = first->source_component;
  publish_off(21, Cmd::SOURCE_CONTROLLER);
  publish_off(22, Cmd::SOURCE_WATCHDOG);
  publish_off(23, Cmd::SOURCE_WATCHDOG);  // newest watchdog OFF replaces queued watchdog OFF
  ASSERT_TRUE(r.pump_until([&] { return r.spray_acks.size() >= 2; }));
  r.deliver();  // negative check: no further ack while seq 20 is still in flight
  ASSERT_EQ(r.spray_acks.size(), 2U);
  EXPECT_EQ(r.spray_acks[0].seq, 21U);
  EXPECT_EQ(r.spray_acks[1].seq, 22U);
  EXPECT_FALSE(r.spray_acks[0].success);
  EXPECT_FALSE(r.spray_acks[1].success);

  px4_msgs::msg::VehicleCommandAck ack;
  ack.target_system = 1;
  ack.command = 187;
  ack.target_component = first_token;
  ack.result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  r.p_ack->publish(ack);
  auto is_watchdog_cmd = [&](const auto& c) {
    return c.command == 187 && c.source_component != first_token;
  };
  ASSERT_TRUE(r.pump_until([&] {
    return r.spray_acks.size() >= 3 && std::any_of(r.cmds.begin(), r.cmds.end(), is_watchdog_cmd);
  }));
  auto watchdog = std::find_if(r.cmds.rbegin(), r.cmds.rend(), is_watchdog_cmd);
  ASSERT_NE(watchdog, r.cmds.rend());
  EXPECT_EQ(watchdog->source_component, 3U);
  r.p_ack->publish(ack);  // duplicate controller OFF response
  r.deliver();            // negative check: a stale ack confirms nothing
  ASSERT_EQ(r.spray_acks.size(), 3U);
  EXPECT_NE(r.spray_acks.back().source, Cmd::SOURCE_WATCHDOG);

  ack.target_component = watchdog->source_component;
  r.p_ack->publish(ack);
  ASSERT_TRUE(r.pump_until([&] { return r.spray_acks.size() >= 4; }));
  ASSERT_EQ(r.spray_acks.size(), 4U);
  EXPECT_EQ(r.spray_acks.back().seq, 23U);
  EXPECT_EQ(r.spray_acks.back().source, Cmd::SOURCE_WATCHDOG);
  EXPECT_TRUE(r.spray_acks.back().success);
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
  r.deliver();
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
  r.deliver();
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
  r.run(3.5);  // beyond the 3.0 s session limit (stale_timesync_s / stale_estimator_flags_s)
  EXPECT_FALSE(r.status.timesync_valid);
  EXPECT_EQ(r.status.timesync_offset_us, 0);  // a stale offset is never presented as current
}

TEST(Px4LinkNode, EstimatorHealthDefaultsUnhealthyUntilFlagsArrive) {
  Rig r;
  r.bring_up();
  r.run(0.3);
  EXPECT_TRUE(r.health.flags_valid);
  // IF-002: the PX4 sample time of the flags, same convention as VehicleState.px4_sample_stamp.
  const double stamp_s = r.health.px4_sample_stamp.sec + r.health.px4_sample_stamp.nanosec * 1e-9;
  EXPECT_GT(stamp_s, r.now - 0.15);
  EXPECT_LE(stamp_s, r.now + 1e-6);
  EXPECT_EQ(r.health.px4_sample_stamp.nanosec % 1000U, 0U);  // microsecond source
  EXPECT_FALSE(r.health.test_ratios_valid);                  // estimator_status is not on DDS
  r.alive = false;
  r.run(3.5);  // beyond the 3.0 s session limit (stale_timesync_s / stale_estimator_flags_s)
  EXPECT_FALSE(r.health.flags_valid);
  EXPECT_EQ(r.health.px4_sample_stamp.sec, 0);  // stale: no sample time is presented
  EXPECT_EQ(r.health.px4_sample_stamp.nanosec, 0U);
}

// RPP-009: yaw rate from attitude deltas on the PX4 sample clock; 0 when the attitude is stale.
TEST(Px4LinkNode, VehicleStateYawRateFromAttitudeAndZeroWhenStale) {
  Rig r;
  r.bring_up();
  r.att_yaw_rate = 0.2;
  r.run(0.5);
  EXPECT_TRUE(r.state.attitude_valid);
  EXPECT_NEAR(r.state.yaw_rate_radps, 0.2, 0.01);
  r.att_alive = false;
  r.run(0.3);
  EXPECT_FALSE(r.state.attitude_valid);
  EXPECT_EQ(r.state.yaw_rate_radps, 0.0F);
  r.att_alive = true;
  r.att_yaw_rate = -0.3;  // counter-clockwise
  r.run(0.5);
  EXPECT_TRUE(r.state.attitude_valid);
  EXPECT_NEAR(r.state.yaw_rate_radps, -0.3, 0.015);
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
  EXPECT_TRUE(r.state.vertical_position_valid);   // IF-004: z_valid
  EXPECT_FALSE(r.state.vertical_velocity_valid);  // v_z_valid false
}

// IF-003: the status reports the pose-to-write age of each new forwarded guard command, on the
// system clock the stamps share; a window without such a write reports nothing.
TEST(Px4LinkNode, StatusReportsPoseToWriteAgeOfForwardedCommands) {
  Rig r;
  r.bring_up();
  r.nav_state = 14;
  bool acc = false;
  uint8_t rs = 0;
  ASSERT_TRUE(r.call_offboard(true, &acc, &rs));
  ASSERT_TRUE(acc);
  r.run(0.2);
  EXPECT_FALSE(r.status.pose_to_write_age_valid);  // no stamped command written yet
  // The pose is 50 ms old (system clock) when the guard command is published.
  const auto sys_us = std::chrono::duration_cast<std::chrono::microseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count() -
                      50'000;
  r.cmd_pose_stamp.sec = static_cast<int32_t>(sys_us / 1'000'000);
  r.cmd_pose_stamp.nanosec = static_cast<uint32_t>((sys_us % 1'000'000) * 1000);
  r.guard(2, 0.3F, NaN, 0.1F);
  ASSERT_TRUE(r.pump_until([&] {
    r.tick(0.01);
    return r.status.pose_to_write_age_valid;
  }));
  // >= 50 ms by construction; the upper bound only rules out a unit or epoch error (test DDS and
  // pumping add wall time on a loaded runner).
  EXPECT_GE(r.status.pose_to_write_age_s, 0.05F);
  EXPECT_LT(r.status.pose_to_write_age_s, 5.0F);
  EXPECT_GE(r.status.pose_to_write_age_max_s, r.status.pose_to_write_age_s);
  r.run(0.15);  // the same command rewritten: not a new measurement, the window empties
  EXPECT_FALSE(r.status.pose_to_write_age_valid);
  EXPECT_FLOAT_EQ(r.status.pose_to_write_age_s, 0.0F);
  r.cmd_pose_stamp = builtin_interfaces::msg::Time{};  // no pose named: nothing to measure
  r.guard(2, 0.3F, NaN, 0.1F);
  r.run(0.25);
  EXPECT_FALSE(r.status.pose_to_write_age_valid);
}

// --- battery (interfaces 0.16.0): display only, valid while a sample is fresh --------------------
TEST(Px4LinkNode, BatteryIsReportedInVehicleStateOnlyWhileFresh) {
  Rig r;
  r.bring_up();
  px4_msgs::msg::BatteryStatus b;
  b.connected = true;
  b.voltage_v = 25.1F;
  b.current_a = 3.4F;
  b.remaining = 0.82F;
  r.p_bat->publish(b);
  r.run(0.1);
  EXPECT_TRUE(r.state.battery_valid);
  EXPECT_FLOAT_EQ(r.state.battery_voltage_v, 25.1F);
  EXPECT_FLOAT_EQ(r.state.battery_current_a, 3.4F);
  EXPECT_FLOAT_EQ(r.state.battery_remaining, 0.82F);
  r.run(3.2);  // no new sample for longer than 3 s: unknown, never the last value
  EXPECT_FALSE(r.state.battery_valid);
  EXPECT_TRUE(std::isnan(r.state.battery_voltage_v));
  b.connected = false;  // PX4 says no battery: never shown
  r.p_bat->publish(b);
  r.run(0.1);
  EXPECT_FALSE(r.state.battery_valid);
}

// --- C1: VehicleState on each new local-position sample ------------------------------------------
namespace {
px4_msgs::msg::VehicleLocalPosition lp_sample(uint64_t t_us) {
  px4_msgs::msg::VehicleLocalPosition lp;
  lp.xy_valid = lp.v_xy_valid = lp.heading_good_for_control = true;
  lp.x = 1.0F;
  lp.y = 2.0F;
  lp.timestamp_sample = t_us;
  return lp;
}
}  // namespace

// (a) The sample is fanned out inside its own callback: no writer tick runs here (the link clock
// does not move and step() is never called), yet the VehicleState carrying it arrives.
TEST(Px4LinkNode, EventDrivenStateIsPublishedInTheSampleCallback) {
  Rig r(nullptr, "", "", {rclcpp::Parameter("event_driven", true)});
  r.bring_up();
  r.deliver();
  const uint64_t t1 = static_cast<uint64_t>(std::llround(r.now * 1e6)) + 1000;
  r.p_lp->publish(lp_sample(t1));
  ASSERT_TRUE(
      r.pump_until([&] { return r.state.px4_sample_stamp.nanosec == (t1 % 1000000) * 1000; }));
  EXPECT_TRUE(r.state.position_valid);
  EXPECT_FLOAT_EQ(r.state.north_m, 1.0F);
  const int after_first = r.state_count;
  r.p_lp->publish(lp_sample(t1));  // the same sample again: nothing new to fan out
  r.deliver();
  EXPECT_EQ(r.state_count, after_first);
  r.p_lp->publish(lp_sample(t1 + 20000));
  ASSERT_TRUE(r.pump_until([&] { return r.state_count == after_first + 1; }));
  EXPECT_EQ(r.state.px4_sample_stamp.nanosec, ((t1 + 20000) % 1000000) * 1000);
}

// (b) Silence: nothing is republished while the last sample is still fresh (the stale-pose path
// downstream ages it from its own receipt), and once the local position is stale the 50 Hz fallback
// publishes the cleared validity flags, exactly when the timer mode would show them.
TEST(Px4LinkNode, EventDrivenFallbackRepublishesOnlyWhileTheLocalPositionIsStale) {
  Rig r(nullptr, "", "", {rclcpp::Parameter("event_driven", true)});
  r.bring_up();
  r.run(0.1);
  r.deliver();
  ASSERT_TRUE(r.state.position_valid);
  r.lp_alive = false;
  r.deliver();
  const int before = r.state_count;
  r.run(0.18);  // < stale_local_position_s (0.2): silence, not a cached republish
  r.deliver();
  EXPECT_EQ(r.state_count, before);
  r.run(0.1);  // now stale: fallback at the 20 ms gate with the flags cleared
  ASSERT_TRUE(r.pump_until([&] { return r.state_count >= before + 3; }));
  EXPECT_FALSE(r.state.position_valid);
  EXPECT_EQ(r.state.px4_sample_stamp.sec, 0);
  r.lp_alive = true;  // samples return: event publication resumes at once
  r.tick(0.01);
  ASSERT_TRUE(r.pump_until([&] { return r.state.position_valid; }));
}

// (c) Timer mode: the sample callback publishes nothing; the 20 ms gate republishes the cache.
TEST(Px4LinkNode, TimerModeStateComesOnlyFromTheTwentyMillisecondGate) {
  Rig r(nullptr, "", "", {rclcpp::Parameter("event_driven", false)});
  r.bring_up();
  r.deliver();
  const int before = r.state_count;
  r.p_lp->publish(lp_sample(static_cast<uint64_t>(std::llround(r.now * 1e6)) + 1000));
  r.deliver();
  EXPECT_EQ(r.state_count, before);  // no writer tick, no VehicleState
  r.lp_alive = false;                // the cache alone keeps the 50 Hz stream while it is fresh
  r.run(0.1);
  ASSERT_TRUE(r.pump_until([&] { return r.state_count >= before + 4; }));
  EXPECT_TRUE(r.state.position_valid);
}

// --- C4: write the guard command to PX4 on arrival
// ------------------------------------------------
namespace {
// Brings a rig to an Active OFFBOARD session with a forwarded command.
void activate(Rig& r) {
  r.bring_up();
  r.nav_state = 14;
  bool acc = false;
  uint8_t rs = 0;
  ASSERT_TRUE(r.call_offboard(true, &acc, &rs));
  ASSERT_TRUE(acc);
  r.guard(2, 0.2F, NaN, 0.1F);
  r.run(0.05);
  r.deliver();
}
}  // namespace

// (a) The command is written inside its own callback: no writer tick runs (the link clock does not
// move, step()/on_timer() are not called), yet the full explicit-control set carrying it arrives,
// exactly one set per new command, none for a duplicate.
TEST(Px4LinkNode, EventDrivenWritesEachGuardCommandInItsCallback) {
  Rig r(nullptr, "", "", {rclcpp::Parameter("event_driven", true)});
  activate(r);
  for (int i = 1; i <= 4; ++i) {
    r.clear();
    const float v = 0.1F * static_cast<float>(i);
    r.guard(2, v, NaN, 0.1F);
    ASSERT_TRUE(r.pump_until([&] { return r.speed.size() == 1 && r.rate.size() == 1; })) << i;
    EXPECT_FLOAT_EQ(r.speed.back().speed_body_x, v);
    EXPECT_EQ(r.ocm.size(), 1U);
    EXPECT_EQ(r.traj.size(), 1U);
    EXPECT_EQ(r.att_sp.size(), 1U);
    r.deliver();
    EXPECT_EQ(r.speed.size(), 1U) << "one write per command";
  }
  r.clear();
  r.seq -= 1;  // the same seq again: ignored by the gate, nothing written
  r.guard(2, 0.9F, NaN, 0.1F);
  r.deliver();
  EXPECT_TRUE(r.speed.empty());
}

// The heartbeat keeps its period around event writes: a timer tick due within half a period of a
// command cycle is skipped (no second write of that command in one tick), the next is not.
TEST(Px4LinkNode, EventDrivenTimerNeverRewritesACommandWithinATick) {
  Rig r(nullptr, "", "", {rclcpp::Parameter("event_driven", true)});
  activate(r);
  r.now += 0.01;
  r.clear();
  r.guard(2, 0.3F, NaN, 0.1F);  // cycles at r.now
  ASSERT_TRUE(r.pump_until([&] { return r.speed.size() == 1; }));
  const double t = r.now;
  r.link->on_timer(t + 0.002);
  r.deliver();
  EXPECT_EQ(r.speed.size(), 1U);  // within half a period: skipped
  r.link->on_timer(t + 0.010);
  ASSERT_TRUE(r.pump_until([&] { return r.speed.size() == 2; }));  // the next tick writes
  EXPECT_FLOAT_EQ(r.speed.back().speed_body_x, 0.3F);
  r.now = t + 0.010;
}

// (b) A silent guard falls to the explicit zero on the same command_max_age_s deadline in both
// modes; the heartbeat (OffboardControlMode) never stops.
TEST(Px4LinkNode, ASilentGuardFallsToZeroOnTheSameDeadlineInBothModes) {
  int zero_tick[2] = {-1, -1};
  for (int mode = 0; mode < 2; ++mode) {
    Rig r(nullptr, "", "", {rclcpp::Parameter("event_driven", mode == 1)});
    activate(r);
    r.guard(2, 0.3F, NaN, 0.1F);
    r.tick(0.01);
    ASSERT_FLOAT_EQ(r.speed.back().speed_body_x, 0.3F);
    for (int k = 1; k <= 40 && zero_tick[mode] < 0; ++k) {
      r.clear();
      r.tick(0.01);
      ASSERT_EQ(r.ocm.size(), 1U) << "heartbeat every writer tick, mode " << mode;
      if (r.speed.back().speed_body_x == 0.0F) zero_tick[mode] = k;
    }
  }
  EXPECT_GE(zero_tick[0], 19);  // command_max_age_s 0.2 on a 100 Hz writer
  EXPECT_LE(zero_tick[0], 21);
  EXPECT_EQ(zero_tick[1], zero_tick[0]);
}

// (c) Timer mode: a command is written by the next writer tick only.
TEST(Px4LinkNode, TimerModeWritesOnlyOnTheWriterTick) {
  Rig r(nullptr, "", "", {rclcpp::Parameter("event_driven", false)});
  activate(r);
  r.clear();
  r.guard(2, 0.4F, NaN, 0.1F);
  r.deliver();
  EXPECT_TRUE(r.speed.empty());
  r.link->on_timer(r.now + 0.001);
  ASSERT_TRUE(r.pump_until([&] { return !r.speed.empty(); }));
  EXPECT_FLOAT_EQ(r.speed.back().speed_body_x, 0.4F);
}
