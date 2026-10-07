// safety_watchdog_node — the independent, fail-closed spray watchdog process. Contract:
// docs/contracts/dyx3_spray.md section 4. It lives in its OWN executable so a crashed or frozen
// controller cannot take it down. It only ever asks for OFF (through dyx3_px4_link, the only
// package that touches /fmu) and proves it can close the valve before the controller is allowed to
// open it (off_authority_ready).
#pragma once

#include <chrono>
#include <functional>
#include <memory>

#include "dyx3_interfaces/msg/spray_actuator_ack.hpp"
#include "dyx3_interfaces/msg/spray_actuator_command.hpp"
#include "dyx3_interfaces/msg/spray_lease.hpp"
#include "dyx3_interfaces/msg/spray_watchdog_status.hpp"
#include "dyx3_spray/watchdog_core.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dyx3_spray {

class SafetyWatchdogNode : public rclcpp::Node {
public:
  explicit SafetyWatchdogNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                              std::function<double()> clock = nullptr, bool create_timer = true);

  void step(double now_s);  // public for deterministic tests
  void shutdown_off();      // revoke ON and send OFF at once (called by main on SIGINT/SIGTERM)
  bool inflight() const { return core_->inflight(); }

private:
  void send(const OffCommand& c);
  rclcpp::Time ros_now() { return get_clock()->now(); }

  std::function<double()> clock_;
  std::unique_ptr<WatchdogCore> core_;
  double last_status_s_{-1e18};
  double status_period_s_{0.2};
  rclcpp::Publisher<dyx3_interfaces::msg::SprayActuatorCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<dyx3_interfaces::msg::SprayWatchdogStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<dyx3_interfaces::msg::SprayLease>::SharedPtr sub_lease_;
  rclcpp::Subscription<dyx3_interfaces::msg::SprayActuatorAck>::SharedPtr sub_ack_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dyx3_spray
