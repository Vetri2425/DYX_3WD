// px4_link_node — the only node that touches /fmu/**. See docs/contracts/dyx3_px4_link.md.
#pragma once

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "dyx3_interfaces/msg/estimator_health.hpp"
#include "dyx3_interfaces/msg/gnss_report.hpp"
#include "dyx3_interfaces/msg/motion_setpoint.hpp"
#include "dyx3_interfaces/msg/px4_link_status.hpp"
#include "dyx3_interfaces/msg/rtcm_data.hpp"
#include "dyx3_interfaces/msg/spray_actuator_ack.hpp"
#include "dyx3_interfaces/msg/spray_actuator_command.hpp"
#include "dyx3_interfaces/msg/ulog_chunk.hpp"
#include "dyx3_interfaces/msg/vehicle_state.hpp"
#include "dyx3_interfaces/srv/arm_disarm.hpp"
#include "dyx3_interfaces/srv/set_offboard.hpp"
#include "dyx3_px4_link/dds_session.hpp"
#include "dyx3_px4_link/msg_version_handshake.hpp"
#include "dyx3_px4_link/offboard_heartbeat.hpp"
#include "dyx3_px4_link/rover_setpoint_writer.hpp"
#include "dyx3_px4_link/vehicle_state_assembler.hpp"
#include "px4_msgs/msg/estimator_status_flags.hpp"
#include "px4_msgs/msg/gps_inject_data.hpp"
#include "px4_msgs/msg/message_format_request.hpp"
#include "px4_msgs/msg/message_format_response.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/rover_attitude_setpoint.hpp"
#include "px4_msgs/msg/rover_rate_setpoint.hpp"
#include "px4_msgs/msg/rover_speed_setpoint.hpp"
#include "px4_msgs/msg/sensor_gps.hpp"
#include "px4_msgs/msg/timesync_status.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/ulog_stream.hpp"
#include "px4_msgs/msg/ulog_stream_ack.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dyx3_px4_link {

// Seconds on a monotonic clock. Injected so tests can drive time deterministically.
using ClockFn = std::function<double()>;

struct LinkParams {
  double publish_rate_hz{100.0};
  double command_max_age_s{0.2};
  StalenessLimits stale;
  double handshake_retry_s{1.0};
  OffboardTiming offboard;
  double arm_confirm_timeout_s{2.0};
  bool ulog_streaming_enabled{true};
  std::string msg_definitions_dir;  // empty: <share of px4_msgs>/msg
};

class Px4LinkNode : public rclcpp::Node {
public:
  // create_timer=false: the owner calls step() (tests with an injected clock).
  explicit Px4LinkNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                       ClockFn clock = nullptr, bool create_timer = true);

  // One publish cycle at link-clock time `now_s`. Public for deterministic tests.
  void step(double now_s);

  // Read-only views for tests.
  const Handshake& handshake() const { return *handshake_; }
  const CommandGate& gate() const { return *gate_; }
  const LinkParams& params() const { return p_; }

private:
  using ArmSrv = dyx3_interfaces::srv::ArmDisarm;
  using OffSrv = dyx3_interfaces::srv::SetOffboard;

  struct Pending {
    bool is_arm{true};
    bool arm_target{false};
    std::shared_ptr<rmw_request_id_t> header;
    double deadline_s{0.0};
  };

  void declare_and_validate_params();
  bool link_healthy_now() const;
  void build_handshake();
  uint64_t stamp_us() const;
  rclcpp::Time ros_now() { return this->get_clock()->now(); }

  void publish_setpoint_set(const Setpoint& sp, uint64_t t_us);
  void publish_vehicle_command(uint32_t command, float p1, float p2, uint64_t t_us);
  void publish_status(double now_s, const StalenessReport& rep, const GateOutput& g);
  void publish_state_and_health(double now_s);
  void service_pending(double now_s, bool link_healthy, const OffboardStep& ofb);
  void start_ulog_if_due(double now_s, bool link_ok);
  void on_spray_command(const dyx3_interfaces::msg::SprayActuatorCommand& m);
  void on_vehicle_command_ack(const px4_msgs::msg::VehicleCommandAck& a);

  ClockFn clock_;
  LinkParams p_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<StalenessMonitor> mon_;
  std::unique_ptr<Handshake> handshake_;
  std::unique_ptr<CommandGate> gate_;
  std::unique_ptr<OffboardSession> offboard_;

  // latest samples + arrival
  LocalPositionSample lp_;
  int64_t ts_offset_us_{0};
  uint32_t ts_rtt_us_{0};
  bool ts_seen_{false};
  AttitudeSample att_;
  StatusSample st_;
  double lp_t_{-1e18}, att_t_{-1e18}, st_t_{-1e18};
  px4_msgs::msg::EstimatorStatusFlags flags_;
  double flags_t_{-1e18};
  bool nav_offboard_{false};
  uint8_t arming_state_{0};

  // outputs of the last step
  GateOutput last_gate_;
  StalenessReport last_rep_;
  bool last_link_ok_{false};
  std::string last_logged_reason_;

  double last_step_s_{-1.0};
  uint64_t overruns_{0};
  double last_status_pub_s_{-1e18}, last_state_pub_s_{-1e18}, last_health_pub_s_{-1e18};
  bool ulog_started_{false};
  uint64_t last_ulog_gen_{~0ULL};
  std::deque<Pending> pending_;
  struct SprayPending {
    uint32_t command;  // MAVLink command id the ack will name
    uint32_t seq;
    uint8_t source;
    double sent_s;
  };
  std::deque<SprayPending> spray_pending_;

  // /fmu publishers
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_ocm_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_traj_;
  rclcpp::Publisher<px4_msgs::msg::RoverSpeedSetpoint>::SharedPtr pub_speed_;
  rclcpp::Publisher<px4_msgs::msg::RoverAttitudeSetpoint>::SharedPtr pub_att_sp_;
  rclcpp::Publisher<px4_msgs::msg::RoverRateSetpoint>::SharedPtr pub_rate_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<px4_msgs::msg::GpsInjectData>::SharedPtr pub_gps_inject_;
  rclcpp::Publisher<px4_msgs::msg::UlogStreamAck>::SharedPtr pub_ulog_ack_;
  rclcpp::Publisher<px4_msgs::msg::MessageFormatRequest>::SharedPtr pub_fmt_req_;
  // /fmu subscribers
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr sub_timesync_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_lp_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_att_;
  rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr sub_flags_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr sub_gps_;
  rclcpp::Subscription<px4_msgs::msg::MessageFormatResponse>::SharedPtr sub_fmt_resp_;
  rclcpp::Subscription<px4_msgs::msg::UlogStream>::SharedPtr sub_ulog_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr sub_cmd_ack_;
  // repo interfaces
  rclcpp::Publisher<dyx3_interfaces::msg::VehicleState>::SharedPtr pub_state_;
  rclcpp::Publisher<dyx3_interfaces::msg::EstimatorHealth>::SharedPtr pub_health_;
  rclcpp::Publisher<dyx3_interfaces::msg::GnssReport>::SharedPtr pub_gnss_;
  rclcpp::Publisher<dyx3_interfaces::msg::Px4LinkStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<dyx3_interfaces::msg::UlogChunk>::SharedPtr pub_chunk_;
  rclcpp::Subscription<dyx3_interfaces::msg::MotionSetpoint>::SharedPtr sub_cmd_;
  rclcpp::Subscription<dyx3_interfaces::msg::RtcmData>::SharedPtr sub_rtcm_;
  rclcpp::Subscription<dyx3_interfaces::msg::SprayActuatorCommand>::SharedPtr sub_spray_;
  rclcpp::Publisher<dyx3_interfaces::msg::SprayActuatorAck>::SharedPtr pub_spray_ack_;
  rclcpp::Service<ArmSrv>::SharedPtr srv_arm_;
  rclcpp::Service<OffSrv>::SharedPtr srv_off_;
};

}  // namespace dyx3_px4_link
