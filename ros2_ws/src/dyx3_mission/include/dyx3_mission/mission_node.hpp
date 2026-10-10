// mission_node — ROS wiring of the mission lifecycle. See docs/contracts/dyx3_mission.md.
// Thin: states are decided by MissionFsm, PX4 calls are sequenced by Px4Sequencer, the frame is
// placed by place_artifact, point results come from PointJournal. This class translates messages,
// runs the file work off the executor thread, and performs each state's entry actions.
#pragma once

#include <cstdint>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "dyx3_interfaces/action/execute_mission.hpp"
#include "dyx3_interfaces/msg/mission_state.hpp"
#include "dyx3_interfaces/msg/point_result.hpp"
#include "dyx3_interfaces/msg/rpp_status.hpp"
#include "dyx3_interfaces/msg/safety_gate_status.hpp"
#include "dyx3_interfaces/msg/vehicle_state.hpp"
#include "dyx3_interfaces/srv/abort_mission.hpp"
#include "dyx3_interfaces/srv/arm_disarm.hpp"
#include "dyx3_interfaces/srv/pause_mission.hpp"
#include "dyx3_interfaces/srv/resume_mission.hpp"
#include "dyx3_interfaces/srv/set_offboard.hpp"
#include "dyx3_interfaces/srv/skip_point.hpp"
#include "dyx3_interfaces/srv/start_mission.hpp"
#include "dyx3_mission/frame_placement.hpp"
#include "dyx3_mission/mission_fsm.hpp"
#include "dyx3_mission/path_artifact.hpp"
#include "dyx3_mission/point_journal.hpp"
#include "dyx3_mission/px4_sequencer.hpp"
#include "dyx3_mission/run_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace dyx3_mission {

class MissionNode : public rclcpp::Node {
public:
  using ExecuteMission = dyx3_interfaces::action::ExecuteMission;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteMission>;

  /// Monotonic nanoseconds. Every age, freshness check, timeout and deadline of the node and of
  /// the PX4 sequencer is measured on this clock, never on ROS time: a wall-clock step (the first
  /// NTP sync) must not make a fresh input look stale or let a deadline expire early.
  using ClockFn = std::function<std::int64_t()>;

  /// `clock` defaults to std::chrono::steady_clock; tests inject their own.
  explicit MissionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                       ClockFn clock = nullptr);
  ~MissionNode() override;

  struct StartOutcome {
    bool accepted = false;
    std::uint8_t reason = 0;  ///< StartMission.Response::REASON_*
    std::uint32_t mission_id = 0;
    bool duplicate = false;
    std::uint8_t gate_reason = 0;
  };
  /// THE single mission-start entry point (the action server and StartMission both call it).
  /// Admission only: no file I/O, no hashing, no PX4 call; the lifecycle then runs on its own.
  StartOutcome begin_mission(const std::string& artifact_sha256, const std::string& request_id);

  const MissionFsm& fsm() const { return fsm_; }

private:
  using ArmSrv = dyx3_interfaces::srv::ArmDisarm;
  using OffboardSrv = dyx3_interfaces::srv::SetOffboard;

  /// The EKF placement this execution's path was placed with.
  struct PlacementRecord {
    bool anchored = false;
    double ref_lat_deg = 0.0;
    double ref_lon_deg = 0.0;
    std::uint8_t xy_reset_counter = 0;
  };
  struct PlaceResult {
    Placement placement;
    bool stored = false;
    std::string store_error;
  };
  struct PendingPx4 {
    bool arm_client = true;
    std::int64_t client_request_id = 0;
  };

  /// The node's monotonic clock (see ClockFn): the ONLY source for ages and deadlines. ROS time
  /// (get_clock()) is used for message stamp fields only.
  std::int64_t steady_ns() const { return clock_(); }
  /// Full guard verdict (SafetyGateStatus.ok); stale or never-seen == not ok.
  bool gate_ok(std::uint8_t* reason = nullptr);
  /// Pre-arm verdict (SafetyGateStatus.pre_arm_ok); stale or never-seen == not ok.
  bool pre_arm_ok(std::uint8_t* reason = nullptr);
  bool vehicle_fresh();
  void evaluate_gate();
  void check_ekf_reset();
  /// The EKF reference still matches the placement (resume is refused otherwise).
  bool reference_matches_placement();
  /// An RppStatus of the current mission arrived within rpp_status_max_age_s (receipt time).
  bool rpp_status_fresh();
  void on_gate(const dyx3_interfaces::msg::SafetyGateStatus& m);
  void on_rpp(const dyx3_interfaces::msg::RppStatus& m);
  void on_vehicle(const dyx3_interfaces::msg::VehicleState& m);
  void on_timer();
  void on_work_timer();

  /// Runs everything the lifecycle owes: finished jobs, PX4 replies/timeouts, entry actions of
  /// the states entered since the last call, pending PX4 requests; publishes when anything moved.
  void advance();
  void enter(State s, std::int64_t now);
  void on_terminal_entry();
  void poll_jobs();
  void handle_px4(const Px4Outcome& o);
  void send_px4(const Px4Request& r);
  void on_px4_reply(std::uint64_t id, bool accepted, std::uint8_t reason);
  bool jobs_pending() const { return load_job_.valid() || place_job_.valid(); }
  std::uint8_t waiting_on() const;

  void publish_state();
  void publish_point(const PointEvent& ev);
  void publish_points(const std::vector<PointEvent>& evs);
  void finish_goal_if_terminal();
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter>& ps);

  // parameters (docs/contracts/dyx3_mission.md section 9)
  std::string missions_dir_;
  double state_publish_hz_;
  double gate_max_age_s_;
  double point_capture_radius_m_;
  double rpp_ack_timeout_s_;
  double rpp_status_max_age_s_;
  double arm_timeout_s_;
  double offboard_timeout_s_;
  double placement_max_distance_m_;
  double vehicle_state_max_age_s_;

  ClockFn clock_;
  MissionFsm fsm_;
  Px4Sequencer px4_;
  RunState run_;
  std::optional<PathArtifact> artifact_;   ///< the source artifact (LOADING result)
  std::unique_ptr<PointJournal> journal_;  ///< over the EXECUTION points (EKF frame)
  std::optional<PlacementRecord> placement_;
  bool ekf_reset_reported_ = false;
  std::string release_note_;  ///< outcome of a failed release step, appended to reason_detail
  std::uint64_t entered_changes_ = 0;
  // MissionState.state_entered is documented as ROS time, while the FSM times its transitions on
  // the steady clock: the ROS time of the latest state change is recorded here for the message.
  std::int64_t state_entered_ros_ns_ = 0;
  std::uint64_t stamped_changes_ = 0;
  bool dirty_ = false;

  // file work off the executor thread (one at a time); results of an execution that is no longer
  // in that step are discarded
  std::future<ArtifactResult> load_job_;
  std::future<PlaceResult> place_job_;
  std::uint32_t job_mission_id_ = 0;

  // latest guard verdict
  // every *_ns_ below is on the steady clock (receipt times and READY entry), never ROS time
  std::optional<std::int64_t> gate_stamp_ns_;
  bool gate_flag_ = false;
  std::uint8_t gate_reason_ = 0;
  bool pre_arm_flag_ = false;
  std::uint8_t pre_arm_reason_ = 0;
  // latest vehicle state (receipt time)
  std::optional<dyx3_interfaces::msg::VehicleState> vehicle_;
  std::int64_t vehicle_stamp_ns_ = 0;

  std::optional<std::int64_t> ready_since_ns_;
  std::optional<std::int64_t> rpp_stamp_ns_;  // receipt time of the last RppStatus of this mission

  std::map<std::uint64_t, PendingPx4> px4_pending_;
  std::shared_ptr<GoalHandle> goal_;
  bool cancel_pending_ = false;

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
  rclcpp::Client<ArmSrv>::SharedPtr arm_cli_;
  rclcpp::Client<OffboardSrv>::SharedPtr offboard_cli_;
  rclcpp_action::Server<ExecuteMission>::SharedPtr action_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr work_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace dyx3_mission
