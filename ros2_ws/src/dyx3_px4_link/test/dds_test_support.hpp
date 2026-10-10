// Test-only DDS support for the node rigs: a private ROS domain per test process and
// deterministic, work-driven delivery instead of fixed wall-clock waits.
// Identical copies live in the test/ directory of every dyx3_* package with a node rig; keep them
// identical.
#pragma once

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/file.h>
#include <unistd.h>

#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <thread>

namespace dyx3_test {

// A ROS domain no other running test process holds. Every candidate id has a lock file in /tmp;
// the process takes the first one it can flock() exclusively and holds it until it exits (the
// kernel releases the lock even on a crash). Concurrent test processes (ctest -j, colcon's parallel
// packages, repeat loops) therefore never share a domain, unlike a pid-derived id. Never used:
// 0 (the default), 42 (the rover) and the caller's own ROS_DOMAIN_ID. 232 is Humble's highest id.
inline int isolated_domain_id() {
  static const int id = [] {
    constexpr int kLo = 1;
    constexpr int kHi = 232;
    constexpr int kSpan = kHi - kLo + 1;
    int own = -1;
    if (const char* env = std::getenv("ROS_DOMAIN_ID")) own = std::atoi(env);
    const int start = static_cast<int>(getpid() % kSpan);  // spreads the first probe only
    for (int k = 0; k < kSpan; ++k) {
      const int d = kLo + (start + k) % kSpan;
      if (d == 42 || d == own) continue;
      const std::string path = "/tmp/dyx3_test_ros_domain_" + std::to_string(d) + ".lock";
      const int fd = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0666);
      if (fd < 0) continue;
      if (flock(fd, LOCK_EX | LOCK_NB) == 0) return d;  // fd stays open: the lock is held
      close(fd);
    }
    throw std::runtime_error("no free ROS domain id for this test process");
  }();
  return id;
}

// The rigs rely on a sample published in this process being in every matched reader of this
// process (its subscription ready) when publish() returns. Fast DDS delivers intra-process inside
// the write; SYNCHRONOUS publication mode additionally keeps every writer off the asynchronous
// background thread, and delivery_is_synchronous() proves the property at runtime. Read when the
// context's participant is created: call before every init. Affects this test process only.
inline void use_synchronous_delivery() {
  setenv("RMW_FASTRTPS_PUBLICATION_MODE", "SYNCHRONOUS", 1);
}

inline void init_isolated(const std::shared_ptr<rclcpp::Context>& ctx) {
  use_synchronous_delivery();
  rclcpp::InitOptions io;
  io.set_domain_id(static_cast<size_t>(isolated_domain_id()));
  ctx->init(0, nullptr, io);
}

// Runs ready callbacks until none is left: every sample published so far has been processed,
// including chains (a request whose callback publishes the response, and so on). It never waits
// for work that is not already there. The bound only stops a callback storm.
inline void drain(rclcpp::Executor& exec) { exec.spin_all(std::chrono::seconds(2)); }

// Proves the precondition of drain(): a sample published in this context is ready in the reader
// as soon as publish() returns, for best-effort and reliable QoS alike. Without it (another RMW,
// the mode overridden) a drain right after a publish would race delivery, so the rig must fail
// loudly here instead of flaking later. Checked once per process (the mode is process-wide).
inline ::testing::AssertionResult delivery_is_synchronous(
    const std::shared_ptr<rclcpp::Context>& ctx) {
  static int verdict = -1;  // -1 unknown, 0 no, 1 yes
  static std::string why;
  if (verdict < 0) {
    rclcpp::NodeOptions no;
    no.context(ctx);
    auto node = std::make_shared<rclcpp::Node>("dyx3_delivery_probe", no);
    rclcpp::ExecutorOptions eo;
    eo.context = ctx;
    rclcpp::executors::SingleThreadedExecutor ex(eo);
    ex.add_node(node);
    verdict = 1;
    int i = 0;
    for (const auto& qos : {rclcpp::QoS(1).best_effort(), rclcpp::QoS(1).reliable()}) {
      const std::string topic = "/dyx3_test/delivery_probe_" + std::to_string(i++);
      int got = 0;
      auto sub = node->create_subscription<builtin_interfaces::msg::Time>(
          topic, qos, [&got](builtin_interfaces::msg::Time::ConstSharedPtr) { ++got; });
      auto pub = node->create_publisher<builtin_interfaces::msg::Time>(topic, qos);
      const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < end) {
        ex.spin_some(std::chrono::milliseconds(1));
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      drain(ex);
      for (int n = 1; n <= 20 && verdict == 1; ++n) {
        pub->publish(builtin_interfaces::msg::Time{});
        drain(ex);  // no wait: the sample must already be in the reader
        if (got != n) {
          verdict = 0;
          why = topic + ": sample " + std::to_string(n) + " not ready right after publish()";
        }
      }
      if (verdict == 0) break;
    }
  }
  if (verdict == 1) return ::testing::AssertionSuccess();
  const char* rmw = std::getenv("RMW_IMPLEMENTATION");
  return ::testing::AssertionFailure()
         << "synchronous intra-process delivery is not in effect (" << why
         << "), RMW_IMPLEMENTATION=" << (rmw != nullptr ? rmw : "(default)");
}

}  // namespace dyx3_test
