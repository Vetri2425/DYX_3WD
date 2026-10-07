// spray_node main. rclcpp's own signal handler shuts the context down, after which nothing can be
// published, so it is disabled: SIGINT/SIGTERM only raise a flag, the loop exits, then the valve is
// closed while the context is still up.
#include <atomic>
#include <csignal>
#include <cstdio>
#include <memory>

#include "dyx3_spray/spray_node.hpp"

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  int rc = 0;
  try {
    auto node = std::make_shared<dyx3_spray::SprayNode>();
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(node);
    while (rclcpp::ok() && !g_stop.load()) ex.spin_some(std::chrono::milliseconds(10));
    // Close the valve and flush briefly (bounded) so the OFF reaches dyx3_px4_link before exit.
    node->shutdown_off();
    for (int i = 0; i < 20 && !node->off_confirmed(); ++i) {
      ex.spin_some(std::chrono::milliseconds(50));
      node->shutdown_off();
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "spray_node: %s\n", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
