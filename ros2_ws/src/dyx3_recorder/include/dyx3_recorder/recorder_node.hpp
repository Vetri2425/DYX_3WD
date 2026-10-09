// recorder_node — ROS wiring of the run recorder. Contract: docs/contracts/dyx3_recorder.md. It
// never publishes anything but RecorderStatus and never gates or delays a mission.
#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "dyx3_interfaces/msg/mission_state.hpp"
#include "dyx3_interfaces/msg/px4_link_status.hpp"
#include "dyx3_interfaces/msg/recorder_status.hpp"
#include "dyx3_interfaces/msg/ulog_chunk.hpp"
#include "dyx3_recorder/bag_writer.hpp"
#include "dyx3_recorder/param_snapshot.hpp"
#include "dyx3_recorder/run_lifecycle.hpp"
#include "dyx3_recorder/run_manifest.hpp"
#include "dyx3_recorder/ulog_capture.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dyx3_recorder {

using ClockFn = std::function<double()>;  // monotonic seconds
using WallFn = std::function<time_t()>;   // UTC wall time
using ParamCollector =
    std::function<std::vector<NodeParams>(const std::vector<std::string>& nodes, double timeout_s)>;

// Parameter value as text; doubles and double arrays with %.17g (exact round trip), the rest as
// rclcpp::to_string.
std::string param_value_text(const rclcpp::ParameterValue& v);

// Default `param_nodes`: the control graph (dyx3_bringup control_graph.launch.py) + the separate
// services (gnss_rtk, spray_watchdog, recorder itself).
std::vector<std::string> default_param_nodes();

// Default collector: SyncParametersClient per node on a helper node with its own executor. Never
// throws (REC-018): a node that does not answer, or answers with fewer values than names, is
// reachable=false with a note (REC-019). `context` null = the global context.
std::vector<NodeParams> collect_ros_params(const std::vector<std::string>& nodes, double timeout_s,
                                           rclcpp::Context::SharedPtr context = nullptr);

class RecorderNode : public rclcpp::Node {
public:
  explicit RecorderNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                        ClockFn clock = nullptr, WallFn wall = nullptr,
                        ParamCollector collector = nullptr, bool create_timer = true);
  ~RecorderNode() override;

  void step(double now_s);  // health + status; public for deterministic tests
  bool recording() const;
  std::string current_run_dir() const;

private:
  void declare_params();
  std::vector<NodeParams> collect_params() const;
  void on_mission(const dyx3_interfaces::msg::MissionState& m);
  void start_run(uint32_t mission_id, uint32_t run_index, const std::string& sha, bool running);
  void mark_running();
  void join_param_job();
  void stop_run(const std::string& final_state);
  void publish_status(double now_s);

  ClockFn clock_;
  WallFn wall_;
  ParamCollector collector_;
  std::string runs_dir_, versions_file_, config_dir_, vehicle_id_, operator_;
  std::vector<std::string> topics_, param_nodes_, bag_command_;
  double bag_finalize_timeout_s_{5.0}, param_timeout_s_{2.0}, status_hz_{2.0};
  uint64_t min_free_bytes_{0};

  mutable std::mutex mu_;
  RunLifecycle lifecycle_;
  BagWriter bag_;
  UlogCapture ulog_;
  std::string run_dir_;
  RunInfo info_;
  RunSummary summary_;
  std::string params_start_;
  // Start-of-run parameter snapshot, collected on its own thread once the bag runs (REC-004).
  std::thread param_thread_;
  double run_start_s_{0.0};
  bool error_{false};
  bool finalizing_{false};
  bool bag_died_{false};
  double last_status_s_{-1e18};
  // newest FCU timesync evidence from dyx3_px4_link (guarded by mu_); stale after link_max_age_s
  bool ts_valid_{false};
  int64_t ts_offset_us_{0};
  uint32_t ts_rtt_us_{0};
  double ts_stamp_s_{-1e18};

  rclcpp::CallbackGroup::SharedPtr cb_mission_, cb_ulog_;
  rclcpp::Publisher<dyx3_interfaces::msg::RecorderStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<dyx3_interfaces::msg::MissionState>::SharedPtr sub_mission_;
  rclcpp::Subscription<dyx3_interfaces::msg::UlogChunk>::SharedPtr sub_ulog_;
  rclcpp::Subscription<dyx3_interfaces::msg::Px4LinkStatus>::SharedPtr sub_link_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dyx3_recorder
