// spray_node — ROS wiring around SprayController. Contract: docs/contracts/dyx3_spray.md section 8.
// The node never talks to the FCU: valve commands go out as SprayActuatorCommand to dyx3_px4_link
// and come back as SprayActuatorAck. The independent dyx3_spray_watchdog is a separate process
// (safety_watchdog_node).
#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "dyx3_interfaces/msg/emergency_stop_state.hpp"
#include "dyx3_interfaces/msg/mission_state.hpp"
#include "dyx3_interfaces/msg/rpp_status.hpp"
#include "dyx3_interfaces/msg/rtk_status.hpp"
#include "dyx3_interfaces/msg/spray_actuator_ack.hpp"
#include "dyx3_interfaces/msg/spray_actuator_command.hpp"
#include "dyx3_interfaces/msg/spray_lease.hpp"
#include "dyx3_interfaces/msg/spray_state.hpp"
#include "dyx3_interfaces/msg/spray_status.hpp"
#include "dyx3_interfaces/msg/spray_watchdog_status.hpp"
#include "dyx3_interfaces/msg/vehicle_state.hpp"
#include "dyx3_interfaces/srv/set_spray_manual.hpp"
#include "dyx3_spray/spray_controller.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dyx3_spray {

using ClockFn = std::function<double()>;

class SprayNode : public rclcpp::Node {
public:
  explicit SprayNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                     ClockFn clock = nullptr, bool create_timer = true);

  // One control tick at node-clock time now_s. Public for deterministic tests.
  void step(double now_s);
  // Revoke the lease and force an immediate OFF (called by main on SIGINT/SIGTERM, before the
  // executor stops).
  void shutdown_off();
  bool off_confirmed() const;

private:
  void declare_and_validate_params();
  void publish_command(uint32_t seq, bool on);
  void publish_lease(double now_s);
  void publish_status(double now_s);
  void on_mission_state(const dyx3_interfaces::msg::MissionState& m);
  void load_artifact(const std::string& sha, const std::string& source_sha);
  rclcpp::Time ros_now() { return get_clock()->now(); }

  ClockFn clock_;
  ParamSet params_;
  std::unique_ptr<SprayController> ctl_;
  double tick_hz_{50.0};
  std::string artifact_dir_;
  std::string loaded_sha_;
  std::string mission_source_sha_;
  std::string rpp_conditioned_sha_;
  uint32_t rpp_mission_id_{0};
  uint32_t mission_id_{0};
  bool mission_running_{false};
  double next_reassert_s_{0.0};
  double last_status_pub_s_{-1e18};
  SprayState last_fsm_state_{SprayState::OffUnconfirmed};
  LeadEvent last_event_{LeadEvent::None};
  std::string last_block_reason_;

  rclcpp::Publisher<dyx3_interfaces::msg::SprayActuatorCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<dyx3_interfaces::msg::SprayLease>::SharedPtr pub_lease_;
  rclcpp::Publisher<dyx3_interfaces::msg::SprayState>::SharedPtr pub_state_;
  rclcpp::Publisher<dyx3_interfaces::msg::SprayStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<dyx3_interfaces::msg::VehicleState>::SharedPtr sub_vehicle_;
  rclcpp::Subscription<dyx3_interfaces::msg::RtkStatus>::SharedPtr sub_rtk_;
  rclcpp::Subscription<dyx3_interfaces::msg::RppStatus>::SharedPtr sub_rpp_;
  rclcpp::Subscription<dyx3_interfaces::msg::MissionState>::SharedPtr sub_mission_;
  rclcpp::Subscription<dyx3_interfaces::msg::EmergencyStopState>::SharedPtr sub_estop_;
  rclcpp::Subscription<dyx3_interfaces::msg::SprayWatchdogStatus>::SharedPtr sub_wd_;
  rclcpp::Subscription<dyx3_interfaces::msg::SprayActuatorAck>::SharedPtr sub_ack_;
  rclcpp::Service<dyx3_interfaces::srv::SetSprayManual>::SharedPtr srv_manual_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dyx3_spray
