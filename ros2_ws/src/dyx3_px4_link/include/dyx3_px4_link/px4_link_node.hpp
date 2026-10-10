// px4_link_node — the only node that touches /fmu/**. See docs/contracts/dyx3_px4_link.md.
#pragma once

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
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
#include "dyx3_px4_link/spray_ack_tokens.hpp"
#include "dyx3_px4_link/vehicle_state_assembler.hpp"
#include "px4_msgs/msg/battery_status.hpp"
#include "px4_msgs/msg/estimator_status_flags.hpp"
#include "px4_msgs/msg/failsafe_flags.hpp"
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
  // Event-driven chain (RESTART). true: VehicleState is published from the vehicle_local_position
  // callback on each new sample (new timestamp_sample) instead of the 20 ms gate; the gate stays
  // only as a fallback while the local position is stale. false: the 20 ms gate (timer mode).
  bool event_driven{true};
  double command_max_age_s{0.2};
  StalenessLimits stale;
  double handshake_retry_s{1.0};
  OffboardTiming offboard;
  double arm_confirm_timeout_s{2.0};
  // Off by default: the best-effort DDS stream loses chunks (about 30/s in the 2026-10-10 run) and
  // the file cannot be decoded; the SD log is the evidence source (contract section 10).
  bool ulog_streaming_enabled{false};
  // An unanswered spray VehicleCommand is failed after this long (its reasserts keep republishing
  // it until then). Short, because an OFF queued behind it waits that long.
  double spray_transaction_timeout_s{0.3};
  double yaw_rate_lpf_tau_s{0.05};
  std::string msg_definitions_dir;  // empty: <share of px4_msgs>/msg
  std::string spray_ack_token_state_path{"/var/lib/dyx3/state/px4_link_spray_ack_next"};
};

class Px4LinkNode : public rclcpp::Node {
public:
  // create_timer=false: the owner calls step() (tests with an injected clock).
  explicit Px4LinkNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                       ClockFn clock = nullptr, bool create_timer = true);

  // One publish cycle at link-clock time `now_s`. Public for deterministic tests.
  void step(double now_s);
  // The writer timer's callback (C4). Timer mode: always a cycle. Event-driven: the timer is reset
  // by every cycle a guard command triggered, so it fires one period after it; a cycle that would
  // land within half a period of the previous one is skipped (no second write of the same command
  // inside one tick). Public for deterministic tests.
  void on_timer(double now_s);

  // Process shutdown (X-010): cancels the writer timer and publishes one explicit STOP set if the
  // heartbeat was running on the last tick. Returns false (nothing sent) otherwise. main() calls it
  // at 100 Hz for a bounded time before exiting.
  bool publish_shutdown_stop();

  // Read-only views for tests/diagnostics.
  const Handshake& handshake() const { return *handshake_; }
  const CommandGate& gate() const { return *gate_; }
  const LinkParams& params() const { return p_; }
  uint64_t spray_late_ack_count() const { return spray_late_ack_count_; }
  uint32_t spray_identities_used() const { return spray_ack_tokens_->used(); }
  uint32_t spray_identities_remaining() const { return spray_ack_tokens_->remaining(); }
  bool spray_identities_exhausted() const { return spray_ack_tokens_->exhausted(); }

private:
  using ArmSrv = dyx3_interfaces::srv::ArmDisarm;
  using OffSrv = dyx3_interfaces::srv::SetOffboard;

  struct Pending {
    bool is_arm{true};
    bool arm_target{false};
    // Arm requested while PX4 was still in OFFBOARD without our heartbeat: the arm command is held
    // until PX4 has left OFFBOARD (MANUAL requested first), then sent with a fresh confirm window.
    bool arm_after_leave_offboard{false};
    std::shared_ptr<rmw_request_id_t> header;
    double deadline_s{0.0};
    // VehicleCommandAck matching (contract section 9). cmd_sent_s is the link-clock time at which
    // the request's own command (arm/disarm 400, or OFFBOARD DO_SET_MODE 176) was published; an
    // ack that arrived earlier cannot answer it. Not sent yet (held arm, offboard prestream) means
    // no ack can match. fcu_accepted: PX4 acknowledged it ACCEPTED, so the next ack belongs to a
    // later request; confirmation still comes from vehicle_status.
    bool cmd_sent{false};
    double cmd_sent_s{0.0};
    bool fcu_accepted{false};
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
  void publish_state(double now_s);
  void service_pending(double now_s, bool link_healthy, const OffboardStep& ofb);
  void start_ulog_if_due(double now_s, bool link_ok);
  void on_spray_command(const dyx3_interfaces::msg::SprayActuatorCommand& m);
  void on_vehicle_command_ack(const px4_msgs::msg::VehicleCommandAck& a);
  void on_arm_mode_command_ack(const px4_msgs::msg::VehicleCommandAck& a);
  // The DO_SET_MODE just published is the OFFBOARD request (true) or a MANUAL release (false): the
  // pending set_offboard requests can match an ack only for the former.
  void note_mode_command_published(bool offboard, double now_s);
  void service_spray_transactions(double now_s);
  void dispatch_next_spray_transaction();
  void publish_spray_ack(uint32_t seq, uint8_t source, bool success, uint8_t result);

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
  std::unique_ptr<YawRateEstimator> yaw_rate_;
  StatusSample st_;
  double lp_t_{-1e18}, att_t_{-1e18}, st_t_{-1e18};
  // PX4 battery_status (1 Hz): display only, never a gate or a stale-topic bit.
  struct Battery {
    bool connected{false};
    float voltage_v{0.0F}, current_a{-1.0F}, remaining{-1.0F};
  } bat_;
  double bat_t_{-1e18};
  // PX4 failsafe_flags (interfaces 0.17.0, RC link): an OPTIONAL topic (contract sections 1, 6).
  // Requested in the handshake but never part of its state(), no staleness bit, no fault, no gate:
  // if it never arrives only VehicleState.rc_link_valid stays false. Its data is used only while
  // its own handshake entry is Ok.
  bool rc_signal_lost_{true};
  double rc_t_{-1e18};
  bool rc_first_sample_logged_{false};
  size_t rc_hs_index_{0};                           // the failsafe_flags entry of handshake_
  std::optional<uint32_t> rc_unusable_warned_gen_;  // one WARN per handshake generation
  // C1: timestamp_sample of the last local-position sample published on arrival.
  bool lp_published_valid_{false};
  uint64_t lp_published_sample_us_{0};
  px4_msgs::msg::EstimatorStatusFlags flags_;
  double flags_t_{-1e18};
  bool nav_offboard_{false};
  // Leaving OFFBOARD to MANUAL (prototype behaviour): requested by set_offboard(false) and by an
  // arm while a stale OFFBOARD is left over. While this deadline is in the future, the heartbeat is
  // off and PX4 still reports OFFBOARD, DO_SET_MODE MANUAL is sent every kLeaveOffboardRetryS.
  double leave_offboard_until_s_{-1e18};
  double leave_offboard_sent_s_{-1e18};
  void request_leave_offboard(double now_s);
  uint8_t arming_state_{0};

  // outputs of the last step
  GateOutput last_gate_;
  StalenessReport last_rep_;
  bool last_link_ok_{false};
  bool last_heartbeat_published_{false};
  // X-010 with C4: set once publish_shutdown_stop() sends its STOP set. From then on no writer
  // cycle runs, whatever triggers it (timer, or a guard command delivered by a late spin): STOP is
  // the last set PX4 sees.
  bool shutting_down_{false};
  std::optional<Reason> logged_zero_reason_;  // empty while not failing to zero

  double last_step_s_{-1.0};
  double last_overrun_s_{-1.0};
  uint64_t overruns_{0};
  double last_status_pub_s_{-1e18}, last_state_pub_s_{-1e18}, last_health_pub_s_{-1e18};
  // IF-003 pose-to-write age over the current status window (first write of each forwarded
  // command only; reset by publish_status).
  bool age_measured_seq_valid_{false};
  uint64_t age_measured_seq_{0};
  bool age_valid_{false};
  float age_last_s_{0.0F};
  float age_max_s_{0.0F};
  bool ulog_started_{false};
  uint64_t last_ulog_gen_{~0ULL};
  std::deque<Pending> pending_;
  struct SprayPending {
    px4_msgs::msg::VehicleCommand vehicle_command;
    uint32_t command;  // MAVLink command id the ack will name
    uint32_t seq;
    uint8_t source;
    bool on;
    uint8_t backend;
    uint8_t actuator_set_index;
    float value;
    uint8_t servo_instance;
    uint16_t pwm_us;
    uint16_t ack_token{0};
    uint8_t ack_system{0};
    double sent_s;
  };
  static bool same_spray_transaction(const SprayPending& a, const SprayPending& b);
  std::deque<SprayPending> spray_queue_;
  std::optional<SprayPending> spray_inflight_;
  // Last dispatched logical epoch per producer, including a timed-out epoch. Exact reasserts
  // retain its identity even when prior ACKs are delayed or missing.
  std::unordered_map<uint8_t, SprayPending> spray_epochs_;
  // A watchdog OFF retires the controller ON it displaced. Late heartbeats of that ON
  // must not reopen the valve after OFF has been acknowledged.
  std::optional<SprayPending> spray_barred_on_;
  // Last positively acknowledged epoch per producer.
  std::unordered_map<uint8_t, SprayPending> spray_confirmed_;
  std::unique_ptr<SprayAckTokens> spray_ack_tokens_;
  uint64_t spray_late_ack_count_{0};

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
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr sub_battery_;
  rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr sub_failsafe_;
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
  uint64_t rtcm_chunks_accepted_{0};
  uint64_t rtcm_chunks_dropped_{0};
  rclcpp::Subscription<dyx3_interfaces::msg::SprayActuatorCommand>::SharedPtr sub_spray_;
  rclcpp::Publisher<dyx3_interfaces::msg::SprayActuatorAck>::SharedPtr pub_spray_ack_;
  rclcpp::Service<ArmSrv>::SharedPtr srv_arm_;
  rclcpp::Service<OffSrv>::SharedPtr srv_off_;
};

}  // namespace dyx3_px4_link
