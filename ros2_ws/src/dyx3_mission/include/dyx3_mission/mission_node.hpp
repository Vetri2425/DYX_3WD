// mission_node — ROS wiring for the mission FSM. See docs/contracts/dyx3_mission.md.
// Thin: every decision is made by MissionFsm / PointJournal; this class only translates messages.
#pragma once

#include <memory>
#include <optional>
#include <string>

#include "dyx3_interfaces/action/execute_mission.hpp"
#include "dyx3_interfaces/msg/mission_state.hpp"
#include "dyx3_interfaces/msg/point_result.hpp"
#include "dyx3_interfaces/msg/rpp_status.hpp"
#include "dyx3_interfaces/msg/safety_gate_status.hpp"
#include "dyx3_interfaces/msg/vehicle_state.hpp"
#include "dyx3_interfaces/srv/abort_mission.hpp"
#include "dyx3_interfaces/srv/pause_mission.hpp"
#include "dyx3_interfaces/srv/resume_mission.hpp"
#include "dyx3_interfaces/srv/skip_point.hpp"
#include "dyx3_interfaces/srv/start_mission.hpp"
#include "dyx3_mission/mission_fsm.hpp"
#include "dyx3_mission/path_artifact.hpp"
#include "dyx3_mission/point_journal.hpp"
#include "dyx3_mission/run_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace dyx3_mission {

class MissionNode : public rclcpp::Node {
public:
  using ExecuteMission = dyx3_interfaces::action::ExecuteMission;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteMission>;

  explicit MissionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  struct StartOutcome {
    bool accepted = false;
    std::uint8_t reason = 0;  ///< StartMission.Response::REASON_*
    std::uint32_t mission_id = 0;
  };
  /// THE single mission-start entry point (the action server and StartMission both call it).
  StartOutcome begin_mission(const std::string& artifact_sha256);

  const MissionFsm& fsm() const { return fsm_; }

private:
  using Clock = rclcpp::Clock;

  std::int64_t now_ns() { return get_clock()->now().nanoseconds(); }
  /// Gate verdict from the guard's SafetyGateStatus; stale or never-seen == not ok.
  bool gate_ok(std::uint8_t* reason = nullptr);
  void evaluate_gate();
  void on_gate(const dyx3_interfaces::msg::SafetyGateStatus& m);
  void on_rpp(const dyx3_interfaces::msg::RppStatus& m);
  void on_vehicle(const dyx3_interfaces::msg::VehicleState& m);
  void on_timer();
  void publish_state();
  void publish_point(const PointEvent& ev);
  void publish_points(const std::vector<PointEvent>& evs);
  void finish_goal_if_terminal();
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter>& ps);

  // parameters (docs/contracts/dyx3_mission.md section 8)
  std::string missions_dir_;
  double state_publish_hz_;
  double gate_max_age_s_;
  double point_capture_radius_m_;
  double rpp_ack_timeout_s_;

  MissionFsm fsm_;
  RunState run_;
  std::optional<PathArtifact> artifact_;
  std::unique_ptr<PointJournal> journal_;

  // latest guard verdict
  std::optional<rclcpp::Time> gate_stamp_;
  bool gate_flag_ = false;
  std::uint8_t gate_reason_ = 0;
  std::optional<std::int64_t> ready_since_ns_;

  std::shared_ptr<GoalHandle> goal_;

  rclcpp::Publisher<dyx3_interfaces::msg::MissionState>::SharedPtr state_pub_;
  rclcpp::Publisher<dyx3_interfaces::msg::PointResult>::SharedPtr point_pub_;
  rclcpp::Subscription<dyx3_interfaces::msg::SafetyGateStatus>::SharedPtr gate_sub_;
  rclcpp::Subscription<dyx3_interfaces::msg::RppStatus>::SharedPtr rpp_sub_;
  rclcpp::Subscription<dyx3_interfaces::msg::VehicleState>::SharedPtr vehicle_sub_;
  rclcpp::Service<dyx3_interfaces::srv::StartMission>::SharedPtr start_srv_;
  rclcpp::Service<dyx3_interfaces::srv::PauseMission>::SharedPtr pause_srv_;
  rclcpp::Service<dyx3_interfaces::srv::ResumeMission>::SharedPtr resume_srv_;
  rclcpp::Service<dyx3_interfaces::srv::AbortMission>::SharedPtr abort_srv_;
  rclcpp::Service<dyx3_interfaces::srv::SkipPoint>::SharedPtr skip_srv_;
  rclcpp_action::Server<ExecuteMission>::SharedPtr action_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace dyx3_mission
