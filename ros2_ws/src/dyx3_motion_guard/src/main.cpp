// motion_guard_node main. rclcpp's own signal handler shuts the context down, after which nothing
// can be published, so it is disabled: SIGINT/SIGTERM only raise a flag, the loop exits, then a
// last burst of STOP goes out while the context is up (mirrors dyx3_rpp/src/main.cpp).
#include <sys/mman.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <memory>

#include "dyx3_motion_guard/motion_guard_node.hpp"

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  // A closed pipe (a log consumer that went away) must not kill the safety layer.
  std::signal(SIGPIPE, SIG_IGN);
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::fprintf(stderr, "motion_guard_node: mlockall failed (continuing unlocked)\n");
  }
  int rc = 0;
  try {
    auto node = std::make_shared<dyx3_motion_guard::MotionGuardNode>();
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(node);
    // spin_once blocks until work is ready (at most 5 ms, so a stop request is seen within 5 ms)
    // and never busy-polls on the core shared with rpp_node at the same FIFO priority.
    while (rclcpp::ok() && !g_stop.load()) ex.spin_once(std::chrono::milliseconds(5));
    for (int i = 0; i < 5; ++i) {  // bounded burst: px4_link sees the last word before we go
      node->shutdown_stop();
      ex.spin_some(std::chrono::milliseconds(5));
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "motion_guard_node: %s\n", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
