// rpp_node — ROS wiring around RppCore. Contract: docs/contracts/rpp_node.md.
// It owns: the parameter store (ParamSet, with the LIVE / IDLE_ONLY / RESTART rules), loading the
// mission artifact BY ID (the same content-addressed file dyx3_mission and dyx3_spray load),
// conditioning it into runs, the 50 Hz tick, the MotionSetpoint stream and RppStatus. It publishes
// no safety verdict: dyx3_motion_guard is the last authority before PX4, and a tablet or backend
// request never reaches this node.
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "dyx3_interfaces/msg/mission_state.hpp"
#include "dyx3_interfaces/msg/motion_setpoint.hpp"
#include "dyx3_interfaces/msg/rpp_status.hpp"
#include "dyx3_interfaces/msg/rtk_status.hpp"
#include "dyx3_interfaces/msg/vehicle_state.hpp"
#include "dyx3_rpp/diagnostics.hpp"
#include "dyx3_rpp/rpp_command.hpp"
#include "dyx3_rpp/rpp_core.hpp"
#include "dyx3_rpp/rpp_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dyx3_rpp {

// Monotonic nanoseconds. Injected so the tests run on a clock they control.
using ClockFn = std::function<int64_t()>;

class RppNode : public rclcpp::Node {
public:
  explicit RppNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                   ClockFn clock = nullptr, bool create_timer = true);

  // One control tick at monotonic time now_ns. Public for deterministic tests.
  void step(int64_t now_ns);
  // Publish STOP once more and stop publishing motion (main calls this on SIGINT/SIGTERM).
  void shutdown_stop();

  const ParamSet& params() const { return params_; }

private:
  void declare_and_validate_params();
  ConditionParams condition_params() const;
  void on_mission_state(const dyx3_interfaces::msg::MissionState& m);
  void load_mission(uint32_t mission_id, const std::string& sha);
  void load_mission_impl(uint32_t mission_id, const std::string& sha);
  void unload_mission();
  void publish_motion(const MotionCommand& c);
  void publish_status(uint8_t state, const TickOutput* out, const MotionCommand& cmd);
  rclcpp::Time ros_now() { return get_clock()->now(); }

  ClockFn clock_;
  ParamSet params_;
  RppCore core_;
  LoopTimer timer_stats_;
  double tick_hz_{50.0};
  std::string artifact_dir_;

  // mission
  bool mission_running_{false};
  // RPP-006: a mission is loaded or active (MissionState LOADING, READY, RUNNING or PAUSED).
  // IDLE_ONLY parameters are refused while true: several are read every tick (require_rtk_fix,
  // pose_max_age_s, ...) and a change while PAUSED would apply on resume.
  bool mission_active_{false};
  bool wants_mission_{false};
  bool loaded_{false};
  bool load_failed_{false};
  uint32_t mission_id_{0};
  std::string sha_;
  std::string conditioned_sha_;
  int64_t retry_load_at_ns_{0};
  uint32_t pending_mission_id_{0};
  std::string pending_sha_;

  uint64_t seq_{0};
  // IF-003: VehicleState.px4_sample_stamp of the newest pose fed into the core; copied into every
  // MotionSetpoint.source_pose_sample_stamp. Zero until a valid pose arrived.
  builtin_interfaces::msg::Time pose_sample_stamp_{};
  uint8_t last_state_{255};
  // XR-RPP-002: a running tick on which the core publishes no command (a run handover that needs
  // no alignment) repeats the previous running tick's command once, instead of a one-tick STOP
  // while driving. Bounded: a second consecutive silent tick is STOP.
  bool repeat_available_{false};
  MotionCommand last_running_cmd_;
  uint8_t last_running_state_{0};
  bool stopped_for_shutdown_{false};

  rclcpp::Publisher<dyx3_interfaces::msg::MotionSetpoint>::SharedPtr pub_motion_;
  rclcpp::Publisher<dyx3_interfaces::msg::RppStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<dyx3_interfaces::msg::VehicleState>::SharedPtr sub_vehicle_;
  rclcpp::Subscription<dyx3_interfaces::msg::RtkStatus>::SharedPtr sub_rtk_;
  rclcpp::Subscription<dyx3_interfaces::msg::MissionState>::SharedPtr sub_mission_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dyx3_rpp
