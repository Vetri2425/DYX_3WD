#include <atomic>
#include <csignal>
#include <cstdio>
#include <memory>

#include "dyx3_recorder/recorder_node.hpp"

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }
}  // namespace

int main(int argc, char** argv) {
  // rclcpp's own handler would shut the context down before the run is finalised; ours only raises
  // a flag.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  int rc = 0;
  try {
    auto node = std::make_shared<dyx3_recorder::RecorderNode>();
    rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 3);
    ex.add_node(node);
    while (rclcpp::ok() && !g_stop.load()) ex.spin_some(std::chrono::milliseconds(20));
    node.reset();  // destructor finalises an open run (bag stop, summary.json)
  } catch (const std::exception& e) {
    std::fprintf(stderr, "recorder: %s\n", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
