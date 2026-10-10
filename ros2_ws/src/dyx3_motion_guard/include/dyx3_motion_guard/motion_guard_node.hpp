// motion_guard_node — see docs/contracts/dyx3_motion_guard.md
#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "dyx3_interfaces/msg/emergency_stop_state.hpp"
#include "dyx3_interfaces/msg/estimator_health.hpp"
#include "dyx3_interfaces/msg/mission_state.hpp"
#include "dyx3_interfaces/msg/motion_setpoint.hpp"
#include "dyx3_interfaces/msg/motion_setpoint_status.hpp"
#include "dyx3_interfaces/msg/px4_link_status.hpp"
#include "dyx3_interfaces/msg/rtk_status.hpp"
#include "dyx3_interfaces/msg/safety_gate_status.hpp"
#include "dyx3_interfaces/msg/vehicle_state.hpp"
#include "dyx3_interfaces/srv/set_emergency_stop.hpp"
#include "dyx3_motion_guard/estop_gate.hpp"
#include "dyx3_motion_guard/fail_to_zero.hpp"
#include "dyx3_motion_guard/freshness_watchdog.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dyx3_motion_guard {

using ClockFn = std::function<double()>;

struct MaxAges {
  double command{0.2};
  double vehicle{0.5};
  double rtk{0.5};
  double estimator{0.5};
  double px4_link{0.5};
  double mission{0.5};
};

class MotionGuardNode : public rclcpp::Node {
public:
  explicit MotionGuardNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                           ClockFn clock = nullptr, bool create_timer = true);

  // One decision cycle at guard-clock time now_s. Public for deterministic tests.
  void step(double now_s);
  // The decision timer's callback (C3). Timer mode: always a decision. Event-driven: a watchdog
  // that decides only when no RPP command did: two periods after a command decision (a late
  // command is not silence), then once per period. Public for deterministic tests.
  void on_watchdog(double now_s);
  bool event_driven() const { return event_driven_; }

  // Publishes one canonical STOP on /dyx3/motion_guard/command, independent of the gates and of
  // the last RPP command. Called by main in a bounded burst after SIGINT/SIGTERM, while the
  // context is still up, so the last word the downstream hop sees is STOP.
  void shutdown_stop();

private:
  void declare_and_validate_params();
  rcl_interfaces::msg::SetParametersResult on_parameters(
      const std::vector<rclcpp::Parameter>& params);
  GateInputs gather(double now_s) const;
  rclcpp::Time ros_now() { return get_clock()->now(); }

  ClockFn clock_;
  double publish_rate_hz_{50.0};
  // C3: decide and forward on each RPP command (startup-only parameter event_driven, default true).
  bool event_driven_{true};
  bool have_tick_{false};
  bool last_tick_was_command_{false};
  double last_tick_s_{0.0};
  uint32_t accept_count_{3};
  MaxAges age_;
  GateConfig gate_cfg_;
  Limits limits_;
  std::unique_ptr<GuardCore> core_;
  EstopLatch estop_;

  // latest inputs + arrival
  Watch w_cmd_, w_veh_, w_rtk_, w_est_, w_link_, w_mission_;
  VehicleIn veh_;
  RtkIn rtk_;
  EstimatorIn est_;
  Px4LinkIn link_;
  MissionIn mission_;
  // IF-003: px4_sample_stamp of the newest VehicleState; stamps the guard's own canonical STOP.
  builtin_interfaces::msg::Time veh_pose_stamp_{};
  double last_step_s_{-1.0};
  double last_gate_pub_s_{-1e18};
  bool force_safety_pub_{false};  // E-stop latch changed: publish gate + E-stop state this step
  double last_status_pub_s_{-1e18};
  uint64_t out_seq_{0};
  Reason last_reason_{Reason::Ok};
  uint64_t reason_changes_{0};
  uint64_t last_status_input_seq_{~0ULL};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::Subscription<dyx3_interfaces::msg::MotionSetpoint>::SharedPtr sub_cmd_;
  rclcpp::Subscription<dyx3_interfaces::msg::MissionState>::SharedPtr sub_mission_;
  rclcpp::Subscription<dyx3_interfaces::msg::VehicleState>::SharedPtr sub_vehicle_;
  rclcpp::Subscription<dyx3_interfaces::msg::EstimatorHealth>::SharedPtr sub_est_;
  rclcpp::Subscription<dyx3_interfaces::msg::RtkStatus>::SharedPtr sub_rtk_;
  rclcpp::Subscription<dyx3_interfaces::msg::Px4LinkStatus>::SharedPtr sub_link_;
  rclcpp::Service<dyx3_interfaces::srv::SetEmergencyStop>::SharedPtr srv_estop_;
  rclcpp::Publisher<dyx3_interfaces::msg::MotionSetpoint>::SharedPtr pub_cmd_;
  rclcpp::Publisher<dyx3_interfaces::msg::MotionSetpointStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<dyx3_interfaces::msg::SafetyGateStatus>::SharedPtr pub_gate_;
  rclcpp::Publisher<dyx3_interfaces::msg::EmergencyStopState>::SharedPtr pub_estop_;
};

}  // namespace dyx3_motion_guard
