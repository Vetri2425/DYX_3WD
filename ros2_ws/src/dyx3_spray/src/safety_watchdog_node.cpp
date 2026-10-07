// safety_watchdog_node — see docs/contracts/dyx3_spray.md
#include "dyx3_spray/safety_watchdog_node.hpp"

#include <cmath>
#include <stdexcept>

namespace dyx3_spray {
namespace {

using dyx3_interfaces::msg::SprayActuatorAck;
using dyx3_interfaces::msg::SprayActuatorCommand;

double steady_now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void require(bool ok, const std::string& what) {
  if (!ok) throw std::invalid_argument("spray watchdog parameter invalid: " + what);
}

}  // namespace

SafetyWatchdogNode::SafetyWatchdogNode(const rclcpp::NodeOptions& options,
                                       std::function<double()> clock, bool create_timer)
    : rclcpp::Node("spray_watchdog", options),
      clock_(clock ? std::move(clock) : std::function<double()>(steady_now_s)) {
  WatchdogParams p;
  p.lease_timeout_s = declare_parameter<double>("lease_timeout_s", 0.35);
  p.off_retry_hz = declare_parameter<double>("off_retry_hz", 2.0);
  p.off_burst_hz = declare_parameter<double>("off_burst_hz", 20.0);
  p.off_burst_duration_s = declare_parameter<double>("off_burst_duration_s", 1.5);
  p.command_ack_timeout_s = declare_parameter<double>("command_ack_timeout_s", 1.0);
  for (const double v :
       {p.lease_timeout_s, p.off_retry_hz, p.off_burst_hz, p.command_ack_timeout_s}) {
    require(std::isfinite(v) && v > 0.0, "timeouts and rates must be finite and > 0");
  }
  require(std::isfinite(p.off_burst_duration_s) && p.off_burst_duration_s >= 0.0,
          "off_burst_duration_s must be >= 0");
  // Startup mapping: used for OFF until a valid lease provides the authoritative one.
  const std::string backend =
      declare_parameter<std::string>("actuator_backend", "mavlink_actuator");
  require(backend == "mavlink_actuator" || backend == "mavlink_servo_pwm", "actuator_backend");
  p.fallback.backend = backend == "mavlink_actuator" ? kBackendActuator : kBackendServoPwm;
  p.fallback.actuator_set_index = static_cast<int>(declare_parameter<int>("actuator_set_index", 1));
  p.fallback.off_value = declare_parameter<double>("off_value", -1.0);
  p.fallback.servo_instance = static_cast<int>(declare_parameter<int>("servo_instance", 1));
  p.fallback.off_pwm_us = static_cast<int>(declare_parameter<int>("off_pwm_us", 0));
  const double tick_hz =
      declare_parameter<double>("tick_hz", 50.0);  // DERIVED: must exceed off_burst_hz
  require(std::isfinite(tick_hz) && tick_hz >= p.off_burst_hz, "tick_hz must be >= off_burst_hz");
  core_ = std::make_unique<WatchdogCore>(p);  // throws on an invalid mapping: fail loud

  pub_cmd_ = create_publisher<SprayActuatorCommand>("/dyx3/spray/actuator_command",
                                                    rclcpp::QoS(10).reliable());
  pub_status_ = create_publisher<dyx3_interfaces::msg::SprayWatchdogStatus>(
      "/dyx3/spray/watchdog_status", rclcpp::QoS(1).reliable());
  sub_lease_ = create_subscription<dyx3_interfaces::msg::SprayLease>(
      "/dyx3/spray/lease", rclcpp::QoS(1).reliable(),
      [this](dyx3_interfaces::msg::SprayLease::ConstSharedPtr m) {
        Lease l;
        l.allow_on = m->allow_on;
        l.command_seq = m->command_seq;
        l.backend = m->backend;
        l.actuator_set_index = m->actuator_set_index;
        l.off_value = m->off_value;
        l.servo_instance = m->servo_instance;
        l.off_pwm_us = m->off_pwm_us;
        core_->on_lease(l, clock_());
      });
  sub_ack_ = create_subscription<SprayActuatorAck>(
      "/dyx3/spray/actuator_ack", rclcpp::QoS(10).reliable(),
      [this](SprayActuatorAck::ConstSharedPtr m) {
        if (m->source != SprayActuatorCommand::SOURCE_WATCHDOG) return;
        core_->on_ack(m->seq, m->success, clock_());
      });
  if (create_timer) {
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tick_hz),
                               [this]() { step(clock_()); });
  }
  RCLCPP_INFO(get_logger(), "spray watchdog up: lease timeout %.3f s", p.lease_timeout_s);
  step(clock_());  // fail-closed before any controller lease
}

void SafetyWatchdogNode::send(const OffCommand& c) {
  SprayActuatorCommand m;
  m.stamp = ros_now();
  m.seq = c.seq;
  m.source = SprayActuatorCommand::SOURCE_WATCHDOG;
  m.backend = static_cast<uint8_t>(c.mapping.backend);
  m.on = false;
  m.actuator_set_index = static_cast<uint8_t>(c.mapping.actuator_set_index);
  m.value = static_cast<float>(c.mapping.off_value);
  m.servo_instance = static_cast<uint8_t>(c.mapping.servo_instance);
  m.pwm_us = static_cast<uint16_t>(c.mapping.off_pwm_us);
  pub_cmd_->publish(m);
}

void SafetyWatchdogNode::step(double now_s) {
  if (const auto c = core_->tick(now_s)) send(*c);
  if (now_s - last_status_s_ >= status_period_s_) {
    last_status_s_ = now_s;
    const WatchdogStatus s = core_->status(now_s);
    dyx3_interfaces::msg::SprayWatchdogStatus m;
    m.stamp = ros_now();
    m.allow_on = s.allow_on;
    m.watchdog_alive = true;
    m.off_authority_ready = s.off_authority_ready;
    m.off_inflight = s.off_inflight;
    m.off_reason = s.off_reason;
    m.lease_age_s = static_cast<float>(s.lease_age_s);
    m.backend = static_cast<uint8_t>(s.backend);
    pub_status_->publish(m);
  }
}

void SafetyWatchdogNode::shutdown_off() {
  const double now = clock_();
  core_->begin_shutdown(now);
  step(now);
}

}  // namespace dyx3_spray
