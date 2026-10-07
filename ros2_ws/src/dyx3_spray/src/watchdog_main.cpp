// spray_watchdog main. See spray_node's main.cpp: the signal handler only raises a flag so the
// shutdown OFF can still be published while the context is up.
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <memory>

#include "dyx3_spray/safety_watchdog_node.hpp"

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }
double now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  int rc = 0;
  try {
    auto node = std::make_shared<dyx3_spray::SafetyWatchdogNode>();
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(node);
    while (rclcpp::ok() && !g_stop.load()) ex.spin_some(std::chrono::milliseconds(10));
    node->shutdown_off();
    for (int i = 0; i < 30; ++i) {  // bounded flush: at most ~1.5 s
      ex.spin_some(std::chrono::milliseconds(50));
      node->step(now_s());
      if (!node->inflight()) break;
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "spray_watchdog: %s\n", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
