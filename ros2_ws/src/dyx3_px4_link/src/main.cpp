// px4_link_node main. rclcpp's own signal handler shuts the context down, after which nothing can
// be published, so it is disabled: SIGINT/SIGTERM only raise a flag, the loop exits, then the STOP
// set is streamed for a bounded time while the context is still up (X-010). px4_link is the last
// hop to PX4: on a graph stop every node gets the signal together, so only this node can make STOP
// the last setpoint PX4 sees before its offboard-loss handling takes over.
#include <sys/mman.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <memory>
#include <thread>

#include "dyx3_px4_link/px4_link_node.hpp"

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }
constexpr int kShutdownStopTicks = 30;  // 0.3 s at the 100 Hz writer rate
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::signal(SIGPIPE, SIG_IGN);  // a closed socket must never kill the last hop to PX4
  // Real-time discipline: no page faults on the tick path. Failure is reported, not fatal
  // (containers and CI often forbid it).
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::fprintf(stderr, "px4_link_node: mlockall failed (continuing unlocked)\n");
  }
  int rc = 0;
  try {
    auto node = std::make_shared<dyx3_px4_link::Px4LinkNode>();
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(node);
    // spin_once waits for work (at most 5 ms), so a stop request is seen within 5 ms.
    while (rclcpp::ok() && !g_stop.load()) ex.spin_once(std::chrono::milliseconds(5));
    // The executor is not spun again: the writer timer must not interleave a guard command with
    // the STOP burst. Nothing is sent when the heartbeat was not running (no proven link or
    // offboard not enabled).
    for (int i = 0; i < kShutdownStopTicks && node->publish_shutdown_stop(); ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  } catch (const std::exception& e) {
    std::fprintf(stderr, "px4_link_node: %s\n", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
