// rpp_node main. rclcpp's own signal handler shuts the context down, after which nothing can be
// published, so it is disabled: SIGINT/SIGTERM only raise a flag, the loop exits, then a last STOP
// goes out while the context is up.
#include <sys/mman.h>

#include <atomic>
#include <csignal>
#include <cstdio>
#include <memory>

#include "dyx3_rpp/rpp_node.hpp"

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  // Real-time discipline (CLAUDE.md section 7): no page faults on the control path. Best effort: a
  // container without the capability keeps running and says so. Scheduling priority and CPU
  // affinity are an open item (systemd unit).
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    std::fprintf(stderr, "rpp_node: mlockall failed (continuing unlocked)\n");
  int rc = 0;
  try {
    auto node = std::make_shared<dyx3_rpp::RppNode>();
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(node);
    while (rclcpp::ok() && !g_stop.load()) ex.spin_some(std::chrono::milliseconds(5));
    for (int i = 0; i < 5;
         ++i) {  // a bounded burst of STOPs so the guard sees the last word before we go
      node->shutdown_stop();
      ex.spin_some(std::chrono::milliseconds(5));
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "rpp_node: %s\n", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
