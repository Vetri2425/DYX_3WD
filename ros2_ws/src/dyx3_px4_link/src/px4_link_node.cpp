#include "dyx3_px4_link/px4_link_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dyx3_px4_link/message_hash.hpp"

namespace dyx3_px4_link {
namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr uint8_t kNavStateOffboard = 14;    // px4_msgs VehicleStatus NAVIGATION_STATE_OFFBOARD
constexpr uint8_t kArmed = 2;                // ARMING_STATE_ARMED
constexpr uint32_t kCmdDoSetMode = 176;      // VEHICLE_CMD_DO_SET_MODE
constexpr uint32_t kCmdArmDisarm = 400;      // VEHICLE_CMD_COMPONENT_ARM_DISARM
constexpr uint32_t kCmdLoggingStart = 2510;  // VEHICLE_CMD_LOGGING_START
constexpr uint32_t kCmdDoSetServo = 183;     // VEHICLE_CMD_DO_SET_SERVO
constexpr uint32_t kCmdDoSetActuator = 187;  // VEHICLE_CMD_DO_SET_ACTUATOR
constexpr size_t kMaxSprayTransactions = 16;

struct UsedTopic {
  const char* request_name;  // BASE topic name: the firmware matches the uORB name, no _vN suffix
  const char* msg_type;
};
const UsedTopic kUsedTopics[] = {
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
};

double steady_now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

template <typename T>
T declare_checked(rclcpp::Node& n, const std::string& name, T def) {
  return n.declare_parameter<T>(name, def);
}

void require(bool ok, const std::string& what) {
  if (!ok) throw std::invalid_argument("px4_link_node parameter invalid: " + what);
}

}  // namespace

Px4LinkNode::Px4LinkNode(const rclcpp::NodeOptions& options, ClockFn clock, bool create_timer)
    : rclcpp::Node("px4_link", options), clock_(clock ? std::move(clock) : ClockFn(steady_now_s)) {
  declare_and_validate_params();
  mon_ = std::make_unique<StalenessMonitor>(p_.stale);
  gate_ = std::make_unique<CommandGate>(p_.command_max_age_s);
  offboard_ = std::make_unique<OffboardSession>(p_.offboard);
  spray_ack_tokens_ = std::make_unique<SprayAckTokens>(p_.spray_ack_token_state_path);
  build_handshake();

  const auto reliable1 = rclcpp::QoS(1).reliable();
  const auto reliable10 = rclcpp::QoS(10).reliable();
  const auto sensor = rclcpp::SensorDataQoS();

  // --- /fmu/in : reliable, matching the uXRCE agent's reader (a best-effort writer would not
  // match).
  pub_ocm_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode",
                                                                  reliable1);
  pub_traj_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", reliable1);
  pub_speed_ = create_publisher<px4_msgs::msg::RoverSpeedSetpoint>("/fmu/in/rover_speed_setpoint",
                                                                   reliable1);
  pub_att_sp_ = create_publisher<px4_msgs::msg::RoverAttitudeSetpoint>(
      "/fmu/in/rover_attitude_setpoint", reliable1);
  pub_rate_ =
      create_publisher<px4_msgs::msg::RoverRateSetpoint>("/fmu/in/rover_rate_setpoint", reliable1);
  pub_cmd_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", reliable10);
  pub_gps_inject_ =
      create_publisher<px4_msgs::msg::GpsInjectData>("/fmu/in/gps_inject_data", reliable10);
  pub_ulog_ack_ = create_publisher<px4_msgs::msg::UlogStreamAck>("/fmu/in/ulog_stream_ack",
                                                                 rclcpp::QoS(16).reliable());
  pub_fmt_req_ = create_publisher<px4_msgs::msg::MessageFormatRequest>(
      "/fmu/in/message_format_request", reliable10);

  // --- /fmu/out
  sub_timesync_ = create_subscription<px4_msgs::msg::TimesyncStatus>(
      "/fmu/out/timesync_status", sensor, [this](px4_msgs::msg::TimesyncStatus::ConstSharedPtr m) {
        mon_->on_sample(kTimesync, clock_());
        ts_offset_us_ = m->estimated_offset;
        ts_rtt_us_ = m->round_trip_time;
        ts_seen_ = true;
      });
  sub_lp_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", sensor,
      [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr m) {
        lp_.xy_valid = m->xy_valid;
        lp_.v_xy_valid = m->v_xy_valid;
        lp_.x = m->x;
        lp_.y = m->y;
        lp_.z = m->z;
        lp_.vx = m->vx;
        lp_.vy = m->vy;
        lp_.vz = m->vz;
        lp_.heading = m->heading;
        lp_.heading_good_for_control = m->heading_good_for_control;
        lp_.xy_reset_counter = m->xy_reset_counter;
        lp_.delta_x = m->delta_xy[0];
        lp_.delta_y = m->delta_xy[1];
        lp_.xy_global = m->xy_global;
        lp_.ref_lat = m->ref_lat;
        lp_.ref_lon = m->ref_lon;
        lp_.ref_alt = m->ref_alt;
        lp_.timestamp_sample_us = m->timestamp_sample;
        lp_t_ = clock_();
        mon_->on_sample(kLocalPosition, lp_t_);
      });
  sub_status_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", sensor, [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr m) {
        st_.arming_state = m->arming_state;
        st_.nav_state = m->nav_state;
        st_.failsafe = m->failsafe;
        st_.pre_flight_checks_pass = m->pre_flight_checks_pass;
        st_t_ = clock_();
        mon_->on_sample(kVehicleStatus, st_t_);
      });
  sub_att_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", sensor,
      [this](px4_msgs::msg::VehicleAttitude::ConstSharedPtr m) {
        for (size_t i = 0; i < 4; ++i) att_.q[i] = m->q[i];
        att_t_ = clock_();
        mon_->on_sample(kAttitude, att_t_);
      });
  sub_flags_ = create_subscription<px4_msgs::msg::EstimatorStatusFlags>(
      "/fmu/out/estimator_status_flags", sensor,
      [this](px4_msgs::msg::EstimatorStatusFlags::ConstSharedPtr m) {
        flags_ = *m;
        flags_t_ = clock_();
        mon_->on_sample(kEstimatorFlags, flags_t_);
      });
  sub_gps_ = create_subscription<px4_msgs::msg::SensorGps>(
      "/fmu/out/vehicle_gps_position", sensor, [this](px4_msgs::msg::SensorGps::ConstSharedPtr m) {
        mon_->on_sample(kGps, clock_());
        dyx3_interfaces::msg::GnssReport r;  // fail-safe defaults: valid=false
        r.stamp = ros_now();
        r.valid = true;
        r.fix_type = m->fix_type;
        r.horizontal_accuracy_m = m->eph;
        r.vertical_accuracy_m = m->epv;
        r.satellites_used = m->satellites_used;
        r.heading_rad = m->heading;
        r.heading_accuracy_rad = m->heading_accuracy;
        r.latitude_deg = m->latitude_deg;
        r.longitude_deg = m->longitude_deg;
        r.altitude_msl_m = static_cast<float>(m->altitude_msl_m);
        r.hdop = m->hdop;
        pub_gnss_->publish(r);
      });
  // Best effort: a lost response is simply re-requested; a reliable subscription would silently
  // fail to match a best-effort writer.
  sub_fmt_resp_ = create_subscription<px4_msgs::msg::MessageFormatResponse>(
      "/fmu/out/message_format_response", rclcpp::QoS(10).best_effort(),
      [this](px4_msgs::msg::MessageFormatResponse::ConstSharedPtr m) {
        size_t n = 0;
        while (n < m->topic_name.size() && m->topic_name[n] != 0) ++n;
        handshake_->on_response(std::string(m->topic_name.begin(), m->topic_name.begin() + n),
                                m->success, m->message_hash);
      });
  // Best effort: the FCU's uXRCE-DDS writers are all best effort, and a reliable reader never
  // matches a best-effort writer (no chunk would ever arrive). Loss shows as a msg_sequence gap.
  sub_ulog_ = create_subscription<px4_msgs::msg::UlogStream>(
      "/fmu/out/ulog_stream", rclcpp::QoS(16).best_effort(),
      [this](px4_msgs::msg::UlogStream::ConstSharedPtr m) {
        // The FCU blocks its stream on the ack, so ack first, always, regardless of link state.
        if ((m->flags & px4_msgs::msg::UlogStream::FLAGS_NEED_ACK) != 0) {
          px4_msgs::msg::UlogStreamAck ack;
          ack.timestamp = stamp_us();
          ack.msg_sequence = m->msg_sequence;
          pub_ulog_ack_->publish(ack);
        }
        dyx3_interfaces::msg::UlogChunk c;
        c.stamp = ros_now();
        c.msg_sequence = m->msg_sequence;
        c.first_message_offset = m->first_message_offset;
        c.flags = m->flags;
        const size_t len = std::min<size_t>(m->length, m->data.size());
        c.data.assign(m->data.begin(), m->data.begin() + static_cast<std::ptrdiff_t>(len));
        pub_chunk_->publish(c);
      });

  sub_cmd_ack_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", sensor,
      [this](px4_msgs::msg::VehicleCommandAck::ConstSharedPtr m) { on_vehicle_command_ack(*m); });

  // --- repo interfaces
  pub_state_ = create_publisher<dyx3_interfaces::msg::VehicleState>("/dyx3/vehicle_state",
                                                                    rclcpp::QoS(1).reliable());
  pub_health_ = create_publisher<dyx3_interfaces::msg::EstimatorHealth>("/dyx3/estimator_health",
                                                                        rclcpp::QoS(1).reliable());
  pub_gnss_ =
      create_publisher<dyx3_interfaces::msg::GnssReport>("/dyx3/gnss_report", rclcpp::QoS(5));
  pub_status_ = create_publisher<dyx3_interfaces::msg::Px4LinkStatus>("/dyx3/px4_link/status",
                                                                      rclcpp::QoS(1).reliable());
  pub_chunk_ = create_publisher<dyx3_interfaces::msg::UlogChunk>("/dyx3/ulog_chunk",
                                                                 rclcpp::QoS(64).reliable());
  sub_cmd_ = create_subscription<dyx3_interfaces::msg::MotionSetpoint>(
      "/dyx3/motion_guard/command", rclcpp::QoS(1).reliable(),
      [this](dyx3_interfaces::msg::MotionSetpoint::ConstSharedPtr m) {
        Command c;
        c.seq = m->seq;
        c.mode = m->mode;
        c.speed_body_x = m->speed_body_x;
        c.yaw_setpoint = m->yaw_setpoint;
        c.yaw_rate_setpoint = m->yaw_rate_setpoint;
        c.valid = m->valid;
        gate_->on_command(c, clock_());
      });
  sub_rtcm_ = create_subscription<dyx3_interfaces::msg::RtcmData>(
      "/dyx3/rtcm", rclcpp::QoS(32).reliable(),
      [this](dyx3_interfaces::msg::RtcmData::ConstSharedPtr m) {
        // RTCM is not motion, but writing it through a format that is not proven would corrupt the
        // receiver feed: refuse until the handshake passed. Oversize is rejected, never truncated.
        if (!link_healthy_now() || m->data.empty() || m->data.size() > 300) {
          ++rtcm_chunks_dropped_;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "RTCM chunk dropped (link_ok=%d, size=%zu)", link_healthy_now(),
                               m->data.size());
          return;
        }
        px4_msgs::msg::GpsInjectData g;
        g.timestamp = stamp_us();
        g.device_id = 0;
        g.len = static_cast<uint16_t>(m->data.size());
        g.flags = m->flags;
        std::copy(m->data.begin(), m->data.end(), g.data.begin());
        try {
          pub_gps_inject_->publish(g);
          ++rtcm_chunks_accepted_;
        } catch (const std::exception& e) {
          ++rtcm_chunks_dropped_;
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                "RTCM injection publication failed: %s", e.what());
        }
      });

  pub_spray_ack_ = create_publisher<dyx3_interfaces::msg::SprayActuatorAck>(
      "/dyx3/spray/actuator_ack", rclcpp::QoS(16).reliable());
  sub_spray_ = create_subscription<dyx3_interfaces::msg::SprayActuatorCommand>(
      "/dyx3/spray/actuator_command", rclcpp::QoS(16).reliable(),
      [this](dyx3_interfaces::msg::SprayActuatorCommand::ConstSharedPtr m) {
        on_spray_command(*m);
      });

  srv_arm_ = create_service<ArmSrv>(
      "/dyx3/px4_link/arm", [this](const std::shared_ptr<rmw_request_id_t> hdr,
                                   const std::shared_ptr<ArmSrv::Request> req) {
        const double now = clock_();
        const bool healthy = link_healthy_now();
        if (req->arm && !healthy) {
          ArmSrv::Response r;
          r.accepted = false;
          r.reason_code = ArmSrv::Response::REASON_LINK_UNHEALTHY;
          srv_arm_->send_response(*hdr, r);
          return;
        }
        publish_vehicle_command(kCmdArmDisarm, req->arm ? 1.0F : 0.0F, 0.0F, stamp_us());
        Pending p;
        p.is_arm = true;
        p.arm_target = req->arm;
        p.header = hdr;
        p.deadline_s = now + p_.arm_confirm_timeout_s;
        pending_.push_back(p);
      });
  srv_off_ = create_service<OffSrv>(
      "/dyx3/px4_link/set_offboard", [this](const std::shared_ptr<rmw_request_id_t> hdr,
                                            const std::shared_ptr<OffSrv::Request> req) {
        const double now = clock_();
        OffSrv::Response r;
        if (!req->enable) {
          // OK means the link has stopped commanding motion: STOP is streamed for
          // offboard_disable_stop_s, then the heartbeat is withdrawn. It does not mean PX4 has
          // left OFFBOARD or the rover has stopped.
          offboard_->enable(false, now);
          r.accepted = true;
          r.reason_code = OffSrv::Response::REASON_OK;
          srv_off_->send_response(*hdr, r);
          return;
        }
        if (!link_healthy_now()) {
          r.accepted = false;
          r.reason_code = OffSrv::Response::REASON_LINK_UNHEALTHY;
          srv_off_->send_response(*hdr, r);
          return;
        }
        offboard_->enable(true, now);
        Pending p;
        p.is_arm = false;
        p.header = hdr;
        p.deadline_s = now + p_.offboard.prestream_s + p_.offboard.confirm_timeout_s + 1.0;
        pending_.push_back(p);
      });

  if (create_timer) {
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / p_.publish_rate_hz),
                               [this]() { step(clock_()); });
  }
  RCLCPP_INFO(get_logger(),
              "px4_link up: %.0f Hz setpoints, command_max_age %.3f s, %zu handshake topics, "
              "message definitions from %s",
              p_.publish_rate_hz, p_.command_max_age_s, std::size(kUsedTopics),
              p_.msg_definitions_dir.c_str());
}

// Health at the moment of a service request: the handshake state is read live (a response may have
// arrived since the last cycle); session and topic freshness come from the last cycle (< 1 period
// old).
bool Px4LinkNode::link_healthy_now() const {
  return handshake_->state() == HandshakeState::Ok && mon_->all_fresh(clock_());
}

void Px4LinkNode::declare_and_validate_params() {
  p_.publish_rate_hz = declare_checked<double>(*this, "publish_rate_hz", 100.0);
  // F1.7: the explicit-control set must be published at >= 100 Hz.
  require(std::isfinite(p_.publish_rate_hz) && p_.publish_rate_hz >= 100.0,
          "publish_rate_hz must be >= 100");
  p_.command_max_age_s = declare_checked<double>(*this, "command_max_age_s", 0.2);
  require(std::isfinite(p_.command_max_age_s) && p_.command_max_age_s > 0.0,
          "command_max_age_s must be > 0");
  static const char* kNames[kTopicCount] = {"stale_timesync_s",        "stale_local_position_s",
                                            "stale_vehicle_status_s",  "stale_attitude_s",
                                            "stale_estimator_flags_s", "stale_gps_s"};
  for (int i = 0; i < kTopicCount; ++i) {
    const double v = declare_checked<double>(*this, kNames[i], p_.stale.max_age_s[i]);
    require(std::isfinite(v) && v > 0.0, std::string(kNames[i]) + " must be > 0");
    p_.stale.max_age_s[i] = v;
  }
  p_.handshake_retry_s = declare_checked<double>(*this, "handshake_retry_s", 1.0);
  require(p_.handshake_retry_s > 0.0, "handshake_retry_s must be > 0");
  p_.offboard.prestream_s = declare_checked<double>(*this, "offboard_prestream_s", 0.5);
  p_.offboard.confirm_timeout_s = declare_checked<double>(*this, "offboard_confirm_timeout_s", 2.0);
  p_.offboard.disable_stop_s =
      declare_checked<double>(*this, "offboard_disable_stop_s", p_.offboard.disable_stop_s);
  require(p_.offboard.prestream_s >= 0.0 && p_.offboard.confirm_timeout_s > 0.0 &&
              std::isfinite(p_.offboard.disable_stop_s) && p_.offboard.disable_stop_s > 0.0,
          "offboard timings");
  p_.arm_confirm_timeout_s = declare_checked<double>(*this, "arm_confirm_timeout_s", 2.0);
  require(p_.arm_confirm_timeout_s > 0.0, "arm_confirm_timeout_s must be > 0");
  p_.ulog_streaming_enabled = declare_checked<bool>(*this, "ulog_streaming_enabled", true);
  p_.spray_transaction_timeout_s =
      declare_checked<double>(*this, "spray_transaction_timeout_s", p_.spray_transaction_timeout_s);
  require(std::isfinite(p_.spray_transaction_timeout_s) && p_.spray_transaction_timeout_s > 0.0,
          "spray_transaction_timeout_s must be > 0");
  p_.spray_ack_token_state_path = declare_checked<std::string>(*this, "spray_ack_token_state_path",
                                                               p_.spray_ack_token_state_path);
  p_.msg_definitions_dir = declare_checked<std::string>(*this, "msg_definitions_dir", "");
  if (p_.msg_definitions_dir.empty()) {
    p_.msg_definitions_dir = ament_index_cpp::get_package_share_directory("px4_msgs") + "/msg";
  }
}

void Px4LinkNode::build_handshake() {
  const std::string dir = p_.msg_definitions_dir;
  const MsgResolver res = [dir](const std::string& n) -> std::optional<std::string> {
    std::ifstream f(dir + "/" + n + ".msg", std::ios::binary);
    if (!f) return std::nullopt;
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
  };
  std::vector<HandshakeTopic> topics;
  for (const auto& u : kUsedTopics) {
    HandshakeTopic t;
    t.request_name = u.request_name;
    std::string err;
    const auto h = message_hash(u.msg_type, res, &err);
    if (h) {
      t.expected_hash = *h;
    } else {
      t.definition_missing = true;
      RCLCPP_ERROR(get_logger(), "px4_msgs handshake: %s (%s)", u.msg_type, err.c_str());
    }
    topics.push_back(std::move(t));
  }
  handshake_ = std::make_unique<Handshake>(std::move(topics), p_.handshake_retry_s);
}

uint64_t Px4LinkNode::stamp_us() const {
  // The firmware converts timestamps against the agent's system clock in its serialisers
  // (contract section 7): publish the Jetson system clock, never an offset-corrected value.
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                   std::chrono::system_clock::now().time_since_epoch())
                                   .count());
}

void Px4LinkNode::publish_setpoint_set(const Setpoint& sp, uint64_t t_us) {
  px4_msgs::msg::OffboardControlMode ocm;
  ocm.timestamp = t_us;
  ocm.position = false;
  ocm.velocity =
      true;  // the only stock flag enabling speed + attitude + rate + allocation together
  ocm.acceleration = false;
  ocm.attitude = false;
  ocm.body_rate = false;
  ocm.thrust_and_torque = false;
  ocm.direct_actuator = false;
  pub_ocm_->publish(ocm);

  px4_msgs::msg::TrajectorySetpoint ts;
  ts.timestamp = t_us;
  for (size_t i = 0; i < 3; ++i) {
    ts.position[i] = kNaN;
    ts.velocity[i] = kNaN;  // sent EVERY cycle: PX4 retains the last finite value otherwise
    ts.acceleration[i] = kNaN;
    ts.jerk[i] = kNaN;
  }
  ts.yaw = kNaN;
  ts.yawspeed = kNaN;
  pub_traj_->publish(ts);

  px4_msgs::msg::RoverSpeedSetpoint rs;
  rs.timestamp = t_us;
  rs.speed_body_x = sp.speed_body_x;
  rs.speed_body_y = kNaN;
  pub_speed_->publish(rs);

  px4_msgs::msg::RoverAttitudeSetpoint ra;
  ra.timestamp = t_us;
  ra.yaw_setpoint = sp.yaw_setpoint;  // NaN unless TRACK_HEADING: sent, never omitted
  pub_att_sp_->publish(ra);

  px4_msgs::msg::RoverRateSetpoint rr;
  rr.timestamp = t_us;
  rr.yaw_rate_setpoint = sp.yaw_rate_setpoint;  // NaN for TRACK_HEADING
  pub_rate_->publish(rr);
}

void Px4LinkNode::publish_vehicle_command(uint32_t command, float p1, float p2, uint64_t t_us) {
  px4_msgs::msg::VehicleCommand c;
  c.timestamp = t_us;
  c.command = command;
  c.param1 = p1;
  c.param2 = p2;
  // DERIVED — NOT FROM V1 SPEC: stock companion addressing (system 1, component 1, external).
  c.target_system = 1;
  c.target_component = 1;
  c.source_system = 1;
  c.source_component = 1;
  c.from_external = true;
  pub_cmd_->publish(c);
}

bool Px4LinkNode::same_spray_transaction(const SprayPending& a, const SprayPending& b) {
  const auto same_float = [](float lhs, float rhs) {
    return lhs == rhs || (std::isnan(lhs) && std::isnan(rhs));
  };
  const auto same_param = [](double lhs, double rhs) {
    return lhs == rhs || (std::isnan(lhs) && std::isnan(rhs));
  };
  const auto& lhs = a.vehicle_command;
  const auto& rhs = b.vehicle_command;
  return a.source == b.source && a.seq == b.seq && a.on == b.on && a.backend == b.backend &&
         a.actuator_set_index == b.actuator_set_index && same_float(a.value, b.value) &&
         a.servo_instance == b.servo_instance && a.pwm_us == b.pwm_us && a.command == b.command &&
         lhs.target_system == rhs.target_system && lhs.target_component == rhs.target_component &&
         lhs.source_system == rhs.source_system && lhs.from_external == rhs.from_external &&
         same_param(lhs.param1, rhs.param1) && same_param(lhs.param2, rhs.param2) &&
         same_param(lhs.param3, rhs.param3) && same_param(lhs.param4, rhs.param4) &&
         same_param(lhs.param5, rhs.param5) && same_param(lhs.param6, rhs.param6) &&
         same_param(lhs.param7, rhs.param7);
}

void Px4LinkNode::on_spray_command(const dyx3_interfaces::msg::SprayActuatorCommand& m) {
  using Cmd = dyx3_interfaces::msg::SprayActuatorCommand;
  // The VehicleCommand format must be proven identical on both sides and the session alive:
  // otherwise refuse AT ONCE so the sender's FSM takes its failure path (a forced OFF retries, an
  // ON is never latched) instead of waiting for a timeout.
  if (handshake_->state() != HandshakeState::Ok || !last_rep_.session_alive) {
    publish_spray_ack(m.seq, m.source, false,
                      dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);
    return;
  }
  px4_msgs::msg::VehicleCommand c;
  c.timestamp = stamp_us();
  c.target_system = 1;
  c.target_component = 1;
  c.source_system = 1;
  // The allocator assigns source_system/component before dispatch; pinned PX4 echoes both into
  // ACK target_system/component. The default here is overwritten for each spray epoch.
  c.from_external = true;
  uint32_t command;
  if (m.backend == Cmd::BACKEND_SERVO_PWM) {
    const uint16_t pwm = std::min<uint16_t>(m.pwm_us, 2200);  // _SERVO_PWM_MAX_US of the prototype
    command = kCmdDoSetServo;
    c.command = command;
    c.param1 = static_cast<float>(m.servo_instance);
    c.param2 = static_cast<float>(pwm);
    c.param3 = c.param4 = 0.0F;
    c.param5 = c.param6 = 0.0;
    c.param7 = 0.0F;
  } else if (m.backend == Cmd::BACKEND_ACTUATOR && m.actuator_set_index >= 1 &&
             m.actuator_set_index <= 6 && std::isfinite(m.value)) {
    command = kCmdDoSetActuator;
    c.command = command;
    float p[6] = {kNaN, kNaN, kNaN, kNaN, kNaN, kNaN};
    p[m.actuator_set_index - 1] = m.value;
    c.param1 = p[0];
    c.param2 = p[1];
    c.param3 = p[2];
    c.param4 = p[3];
    c.param5 = static_cast<double>(p[4]);
    c.param6 = static_cast<double>(p[5]);
    c.param7 = 0.0F;
  } else {
    publish_spray_ack(m.seq, m.source, false,
                      dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);
    return;
  }

  SprayPending request;
  request.vehicle_command = c;
  request.command = command;
  request.seq = m.seq;
  request.source = m.source;
  request.on = m.on;
  request.backend = m.backend;
  request.actuator_set_index = m.actuator_set_index;
  request.value = m.value;
  request.servo_instance = m.servo_instance;
  request.pwm_us = m.pwm_us;

  if (m.source == Cmd::SOURCE_WATCHDOG && !m.on) {
    const auto controller = spray_epochs_.find(Cmd::SOURCE_CONTROLLER);
    if (controller != spray_epochs_.end() && controller->second.on)
      spray_barred_on_ = controller->second;
  }
  if (m.source == Cmd::SOURCE_CONTROLLER && m.on && spray_barred_on_) {
    if (same_spray_transaction(*spray_barred_on_, request)) return;
    spray_barred_on_.reset();  // a genuinely new controller ON epoch may follow safety recovery
  }

  // Once watchdog OFF is waiting for or receiving FCU proof, do not put a stale controller ON
  // back on the wire. The controller's ON heartbeat is no longer a required reassert then.
  if (request.on && ((spray_inflight_ && spray_inflight_->source == Cmd::SOURCE_WATCHDOG &&
                      !spray_inflight_->on) ||
                     std::any_of(spray_queue_.begin(), spray_queue_.end(), [](const auto& queued) {
                       return queued.source == Cmd::SOURCE_WATCHDOG && !queued.on;
                     }))) {
    return;
  }

  // The spray node publishes the same cmd_seq at reassert_hz. An exact match is another physical
  // send of the same logical request. The request key
  // includes producer/sequence/intent plus the full ROS mapping and physical VehicleCommand
  // payload.
  if (spray_inflight_ && same_spray_transaction(*spray_inflight_, request)) {
    auto wire = spray_inflight_->vehicle_command;
    wire.timestamp = stamp_us();
    pub_cmd_->publish(wire);
    return;
  }
  if (std::any_of(spray_queue_.begin(), spray_queue_.end(), [&](const SprayPending& queued) {
        return same_spray_transaction(queued, request);
      })) {
    return;
  }
  const auto confirmed = spray_confirmed_.find(request.source);
  if (confirmed != spray_confirmed_.end()) {
    if (same_spray_transaction(confirmed->second, request)) {
      auto wire = confirmed->second.vehicle_command;
      wire.timestamp = stamp_us();
      pub_cmd_->publish(wire);
      return;
    }
    spray_confirmed_.erase(confirmed);
  }
  const auto epoch = spray_epochs_.find(request.source);
  if (epoch != spray_epochs_.end()) {
    if (same_spray_transaction(epoch->second, request)) {
      request.ack_token = epoch->second.ack_token;
      request.ack_system = epoch->second.ack_system;
    } else {
      spray_epochs_.erase(epoch);
    }
  }

  const bool watchdog_off = m.source == Cmd::SOURCE_WATCHDOG && !m.on;
  // Newest request from each source wins while queued. A dropped request is explicitly failed so
  // its sender cannot mistake queue replacement for an FCU confirmation.
  for (auto it = spray_queue_.begin(); it != spray_queue_.end();) {
    // Watchdog OFF wins over everything queued from other sources, ON or OFF.
    if ((watchdog_off && it->source != Cmd::SOURCE_WATCHDOG) || it->source == m.source) {
      publish_spray_ack(it->seq, it->source, false,
                        dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);
      it = spray_queue_.erase(it);
    } else {
      ++it;
    }
  }

  // An OFF never waits behind an in-flight ON: the ON's ACK may be lost (best-effort FCU topic) and
  // the valve would stay open until the transaction timed out. The ON is failed, the OFF goes now.
  const bool preempt_on = !request.on && spray_inflight_ && spray_inflight_->on;
  if (preempt_on) {
    publish_spray_ack(spray_inflight_->seq, spray_inflight_->source, false,
                      dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);
    spray_inflight_.reset();
  }

  const size_t active_count = spray_queue_.size() + (spray_inflight_ ? 1U : 0U);
  if (active_count >= kMaxSprayTransactions) {
    auto evict = std::find_if(spray_queue_.begin(), spray_queue_.end(), [](const SprayPending& p) {
      return !(p.source == Cmd::SOURCE_WATCHDOG && !p.on);
    });
    if (evict == spray_queue_.end()) {
      publish_spray_ack(request.seq, request.source, false,
                        dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);
      return;
    }
    publish_spray_ack(evict->seq, evict->source, false,
                      dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);
    spray_queue_.erase(evict);
  }

  if (watchdog_off || preempt_on) {
    spray_queue_.push_front(std::move(request));
  } else {
    spray_queue_.push_back(std::move(request));
  }
  dispatch_next_spray_transaction();
}

void Px4LinkNode::on_vehicle_command_ack(const px4_msgs::msg::VehicleCommandAck& a) {
  if (a.result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS) return;
  if (!spray_inflight_ || spray_inflight_->command != a.command ||
      spray_inflight_->ack_system != a.target_system ||
      spray_inflight_->ack_token != a.target_component) {
    if (a.command == kCmdDoSetServo || a.command == kCmdDoSetActuator) {
      ++spray_late_ack_count_;
      RCLCPP_WARN(
          get_logger(),
          "discarded unmatched spray ACK command=%u target_component=%u (late/unmatched=%llu)",
          static_cast<unsigned>(a.command), static_cast<unsigned>(a.target_component),
          static_cast<unsigned long long>(spray_late_ack_count_));
    }
    return;
  }

  const SprayPending completed = *spray_inflight_;
  spray_inflight_.reset();
  const bool accepted = a.result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  if (accepted) spray_confirmed_[completed.source] = completed;
  publish_spray_ack(completed.seq, completed.source, accepted, a.result);
  dispatch_next_spray_transaction();
}

void Px4LinkNode::publish_spray_ack(uint32_t seq, uint8_t source, bool success, uint8_t result) {
  dyx3_interfaces::msg::SprayActuatorAck ack;
  ack.stamp = ros_now();
  ack.seq = seq;
  ack.source = source;
  ack.success = success;
  ack.result = result;
  pub_spray_ack_->publish(ack);
}

void Px4LinkNode::dispatch_next_spray_transaction() {
  if (spray_inflight_ || spray_queue_.empty()) return;
  if (handshake_->state() != HandshakeState::Ok || !last_rep_.session_alive) return;

  SprayPending request = std::move(spray_queue_.front());
  spray_queue_.pop_front();
  if (request.ack_token == 0) {
    const auto token = spray_ack_tokens_->reserve();
    if (token) {
      request.ack_system = token->system;
      request.ack_token = token->component;
    }
  }
  if (request.ack_token == 0) {
    publish_spray_ack(request.seq, request.source, false,
                      dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);
    RCLCPP_ERROR(get_logger(),
                 "spray ACK identity exhausted or state unavailable; refusing actuator command");
    return;
  }
  request.vehicle_command.source_system = request.ack_system;
  request.vehicle_command.source_component = request.ack_token;
  request.vehicle_command.timestamp = stamp_us();
  request.sent_s = clock_();
  spray_epochs_[request.source] = request;
  spray_inflight_ = std::move(request);
  pub_cmd_->publish(spray_inflight_->vehicle_command);
}

void Px4LinkNode::service_spray_transactions(double now_s) {
  if (!last_link_ok_) {
    if (spray_inflight_) {
      publish_spray_ack(spray_inflight_->seq, spray_inflight_->source, false,
                        dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);
      spray_inflight_.reset();
    }
    while (!spray_queue_.empty()) {
      const auto request = spray_queue_.front();
      spray_queue_.pop_front();
      publish_spray_ack(request.seq, request.source, false,
                        dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);
    }
    return;
  }

  if (spray_inflight_ && now_s - spray_inflight_->sent_s > p_.spray_transaction_timeout_s) {
    publish_spray_ack(spray_inflight_->seq, spray_inflight_->source, false,
                      dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED);
    spray_inflight_.reset();
  }
  dispatch_next_spray_transaction();
}

void Px4LinkNode::start_ulog_if_due(double /*now_s*/, bool link_ok) {
  if (!p_.ulog_streaming_enabled || !link_ok) return;
  if (ulog_started_ && last_ulog_gen_ == handshake_->generation()) return;
  // param1 = 0: ULog format (MAVLink LOGGING_START). DERIVED / not verified against the firmware
  // logger source here (sparse checkout): confirm on the bench that chunks arrive.
  publish_vehicle_command(kCmdLoggingStart, 0.0F, 0.0F, stamp_us());
  ulog_started_ = true;
  last_ulog_gen_ = handshake_->generation();
}

void Px4LinkNode::step(double now_s) {
  if (!std::isfinite(now_s) || now_s < 0.0 || (last_step_s_ >= 0.0 && now_s < last_step_s_)) {
    // An unusable clock cannot age anything, so no timing or gate state is touched. Fail to zero:
    // keep the setpoint heartbeat alive with an explicit STOP rather than omitting the tick.
    publish_setpoint_set(stop_setpoint(), stamp_us());
    return;
  }
  // Loop overrun evidence: a tick later than 1.5 periods. Reported, not acted on (contract s.11).
  const double period = 1.0 / p_.publish_rate_hz;
  if (last_step_s_ >= 0.0 && now_s - last_step_s_ > 1.5 * period) {
    ++overruns_;
    last_overrun_s_ = now_s;
  }
  last_step_s_ = now_s;

  last_rep_ = mon_->evaluate(now_s);
  if (mon_->consume_reset()) {
    handshake_->rearm();  // possibly a different firmware on the other side
    RCLCPP_WARN(get_logger(), "uXRCE-DDS session was lost and re-established: handshake re-armed");
  }
  if (last_rep_.session_alive) {
    for (const size_t i : handshake_->due_requests(now_s)) {
      px4_msgs::msg::MessageFormatRequest rq;
      rq.timestamp = stamp_us();
      rq.protocol_version = px4_msgs::msg::MessageFormatRequest::LATEST_PROTOCOL_VERSION;
      const std::string name = kUsedTopics[i].request_name;
      std::fill(rq.topic_name.begin(), rq.topic_name.end(), '\0');
      std::memcpy(rq.topic_name.data(), name.data(),
                  std::min(name.size(), rq.topic_name.size() - 1));
      pub_fmt_req_->publish(rq);
    }
  }
  const bool hs_ok = handshake_->state() == HandshakeState::Ok;
  if (handshake_->state() == HandshakeState::Mismatch) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "px4_msgs HANDSHAKE MISMATCH: %s",
                          handshake_->first_mismatch_reason().c_str());
  }
  last_link_ok_ = hs_ok && last_rep_.session_alive;
  service_spray_transactions(now_s);

  last_gate_ = gate_->step({now_s, hs_ok, last_rep_.session_alive, last_rep_.mask});
  const std::string reason_key = std::to_string(static_cast<int>(last_gate_.reason));
  if (last_gate_.failing_to_zero && reason_key != last_logged_reason_) {
    RCLCPP_WARN(get_logger(), "failing to zero: reason code %s", reason_key.c_str());
  }
  last_logged_reason_ = last_gate_.failing_to_zero ? reason_key : std::string();

  const bool nav_fresh = (now_s - st_t_) <= p_.stale.max_age_s[kVehicleStatus];
  nav_offboard_ = nav_fresh && st_.nav_state == kNavStateOffboard;
  const OffboardStep ofb = offboard_->step(now_s, last_link_ok_, nav_offboard_);
  const uint64_t t_us = stamp_us();
  last_heartbeat_published_ = ofb.publish_heartbeat;
  if (ofb.publish_heartbeat)
    publish_setpoint_set(ofb.stop_only ? stop_setpoint() : last_gate_.sp, t_us);
  if (ofb.send_mode_command)
    publish_vehicle_command(kCmdDoSetMode, 1.0F, 6.0F, t_us);  // 6 = OFFBOARD
  start_ulog_if_due(now_s, last_link_ok_);

  service_pending(now_s, last_link_ok_, ofb);
  publish_state_and_health(now_s);
  publish_status(now_s, last_rep_, last_gate_);
}

bool Px4LinkNode::publish_shutdown_stop() {
  if (timer_) timer_->cancel();
  // Only where the stream was live: never start a heartbeat, and never write an unproven format.
  if (!last_heartbeat_published_ || handshake_->state() != HandshakeState::Ok) return false;
  publish_setpoint_set(stop_setpoint(), stamp_us());
  return true;
}

void Px4LinkNode::service_pending(double now_s, bool link_healthy, const OffboardStep& ofb) {
  for (auto it = pending_.begin(); it != pending_.end();) {
    bool done = false;
    if (it->is_arm) {
      ArmSrv::Response r;
      const bool st_fresh = (now_s - st_t_) <= p_.stale.max_age_s[kVehicleStatus];
      const bool confirmed =
          st_fresh && ((st_.arming_state == kArmed) == it->arm_target) && st_.arming_state != 0;
      if (confirmed) {
        r.accepted = true;
        r.reason_code = ArmSrv::Response::REASON_OK;
        done = true;
      } else if (now_s > it->deadline_s) {
        r.accepted = false;
        r.reason_code = ArmSrv::Response::REASON_TIMEOUT;
        done = true;
      }
      if (done) srv_arm_->send_response(*it->header, r);
    } else {
      OffSrv::Response r;
      if (ofb.state == OffboardState::Active) {
        r.accepted = true;
        r.reason_code = OffSrv::Response::REASON_OK;
        done = true;
      } else if (ofb.state == OffboardState::Failed || ofb.state == OffboardState::Lost) {
        r.accepted = false;
        r.reason_code = OffSrv::Response::REASON_NOT_ARMED_OR_REJECTED;
        done = true;
      } else if (ofb.state == OffboardState::Disabled || !link_healthy || now_s > it->deadline_s) {
        r.accepted = false;
        r.reason_code = OffSrv::Response::REASON_LINK_UNHEALTHY;
        done = true;
      }
      if (done) srv_off_->send_response(*it->header, r);
    }
    it = done ? pending_.erase(it) : std::next(it);
  }
}

void Px4LinkNode::publish_state_and_health(double now_s) {
  if (now_s - last_state_pub_s_ >= 0.02 - 1e-9) {
    last_state_pub_s_ = now_s;
    const double fresh_lp = p_.stale.max_age_s[kLocalPosition];
    const double fresh_att = p_.stale.max_age_s[kAttitude];
    const double fresh_st = p_.stale.max_age_s[kVehicleStatus];
    Freshness f;
    f.local_position = (now_s - lp_t_) <= fresh_lp;
    f.attitude = (now_s - att_t_) <= fresh_att;
    f.status = (now_s - st_t_) <= fresh_st;
    const auto o = assemble(lp_, att_, st_, f);
    dyx3_interfaces::msg::VehicleState s;
    s.stamp = ros_now();
    // PX4 timestamps already arrive in the system-clock domain (contract section 7): no offset to
    // apply.
    s.px4_sample_stamp.sec = static_cast<int32_t>(o.px4_sample_us / 1000000ULL);
    s.px4_sample_stamp.nanosec = static_cast<uint32_t>((o.px4_sample_us % 1000000ULL) * 1000ULL);
    s.position_valid = o.position_valid;
    s.velocity_valid = o.velocity_valid;
    s.attitude_valid = o.attitude_valid;
    s.north_m = o.north;
    s.east_m = o.east;
    s.down_m = o.down;
    s.velocity_north_mps = o.vn;
    s.velocity_east_mps = o.ve;
    s.velocity_down_mps = o.vd;
    for (size_t i = 0; i < 4; ++i) s.q_frd_to_ned[i] = o.q[i];
    s.heading_rad = o.heading;
    s.xy_reset_counter = o.xy_reset_counter;
    s.delta_north_m = o.delta_north;
    s.delta_east_m = o.delta_east;
    s.global_reference_valid = o.global_reference_valid;
    s.reference_latitude_deg = o.ref_lat;
    s.reference_longitude_deg = o.ref_lon;
    s.reference_altitude_m_amsl = o.ref_alt;
    s.arming_state = o.arming_state;
    s.nav_state = o.nav_state;
    s.failsafe = o.failsafe;
    s.preflight_checks_pass = o.preflight_checks_pass;
    pub_state_->publish(s);
  }
  if (now_s - last_health_pub_s_ >= 0.1 - 1e-9) {
    last_health_pub_s_ = now_s;
    dyx3_interfaces::msg::EstimatorHealth h;  // flags_valid=false default: unhealthy
    h.stamp = ros_now();
    const bool fresh = (now_s - flags_t_) <= p_.stale.max_age_s[kEstimatorFlags];
    if (fresh) {
      h.flags_valid = true;
      h.gnss_yaw_fusion_intended = flags_.cs_gnss_yaw;
      h.gnss_yaw_fault = flags_.cs_gnss_yaw_fault;
      h.reject_yaw = flags_.reject_yaw;
      h.reject_hor_pos = flags_.reject_hor_pos;
      h.reject_hor_vel = flags_.reject_hor_vel;
      h.inertial_dead_reckoning = flags_.cs_inertial_dead_reckoning;
      h.innovation_fault_status_changes = flags_.fault_status_changes;
    }
    // test_ratios_valid stays false: estimator_status is not on DDS at the flashed firmware.
    pub_health_->publish(h);
  }
}

void Px4LinkNode::publish_status(double now_s, const StalenessReport& rep, const GateOutput& g) {
  if (now_s - last_status_pub_s_ < 0.1 - 1e-9) return;
  last_status_pub_s_ = now_s;
  dyx3_interfaces::msg::Px4LinkStatus s;
  s.stamp = ros_now();
  s.session_alive = rep.session_alive;
  s.handshake_ok = handshake_->state() == HandshakeState::Ok;
  s.offboard_heartbeat_active = last_heartbeat_published_;
  s.failing_to_zero = g.failing_to_zero;
  uint8_t fault = dyx3_interfaces::msg::Px4LinkStatus::FAULT_NONE;
  if (!rep.session_alive) {
    fault = dyx3_interfaces::msg::Px4LinkStatus::FAULT_NO_SESSION;
  } else if (handshake_->state() == HandshakeState::Mismatch) {
    fault = dyx3_interfaces::msg::Px4LinkStatus::FAULT_HANDSHAKE_MISMATCH;
  } else if (handshake_->state() == HandshakeState::Pending) {
    fault = dyx3_interfaces::msg::Px4LinkStatus::FAULT_HANDSHAKE_PENDING;
  } else if (rep.mask != 0U) {
    fault = dyx3_interfaces::msg::Px4LinkStatus::FAULT_TOPIC_STALE;
  } else if (g.reason == Reason::CommandStale || g.reason == Reason::NoCommand ||
             g.reason == Reason::CommandInvalid || g.reason == Reason::SequenceReset) {
    fault = dyx3_interfaces::msg::Px4LinkStatus::FAULT_COMMAND_STALE;
  } else if (last_overrun_s_ >= 0.0 && now_s - last_overrun_s_ < 1.0) {
    fault = dyx3_interfaces::msg::Px4LinkStatus::FAULT_LOOP_OVERRUN;
  }
  s.fault = fault;
  s.stale_topics_mask = rep.mask;
  s.worst_topic_age_s = static_cast<float>(rep.worst_age_s);
  s.command_age_s = static_cast<float>(g.command_age_s);
  s.loop_overrun_count = overruns_;
  s.command_gap_events = gate_->gap_events();
  s.session_resets = mon_->session_resets();
  // Timesync values only (OPEN: no convergence criterion has a source, so nothing gates on them).
  s.timesync_valid = ts_seen_ && rep.session_alive;
  s.timesync_offset_us = s.timesync_valid ? ts_offset_us_ : 0;
  s.timesync_round_trip_us = s.timesync_valid ? ts_rtt_us_ : 0U;
  s.spray_identities_used = spray_ack_tokens_->used();
  s.spray_identities_remaining = spray_ack_tokens_->remaining();
  s.spray_identities_exhausted = spray_ack_tokens_->exhausted();
  s.spray_unmatched_ack_count = spray_late_ack_count_;
  s.rtcm_chunks_accepted = rtcm_chunks_accepted_;
  s.rtcm_chunks_dropped = rtcm_chunks_dropped_;
  pub_status_->publish(s);
}

}  // namespace dyx3_px4_link
