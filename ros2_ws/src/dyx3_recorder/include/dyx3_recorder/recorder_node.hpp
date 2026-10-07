// recorder_node — ROS wiring of the run recorder. Contract: docs/contracts/dyx3_recorder.md. It
// never publishes anything but RecorderStatus and never gates or delays a mission.
#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dyx3_interfaces/msg/mission_state.hpp"
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

// Default collector: SyncParametersClient per node on a helper node with its own executor.
std::vector<NodeParams> collect_ros_params(const std::vector<std::string>& nodes, double timeout_s);

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
  void on_mission(const dyx3_interfaces::msg::MissionState& m);
  void start_run(uint32_t mission_id, uint32_t run_index, const std::string& sha);
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
  double run_start_s_{0.0};
  bool error_{false};
  bool finalizing_{false};
  bool bag_died_{false};
  double last_status_s_{-1e18};

  rclcpp::CallbackGroup::SharedPtr cb_mission_, cb_ulog_;
  rclcpp::Publisher<dyx3_interfaces::msg::RecorderStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<dyx3_interfaces::msg::MissionState>::SharedPtr sub_mission_;
  rclcpp::Subscription<dyx3_interfaces::msg::UlogChunk>::SharedPtr sub_ulog_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dyx3_recorder
