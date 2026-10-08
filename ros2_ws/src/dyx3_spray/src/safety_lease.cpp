#include "dyx3_spray/safety_lease.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace dyx3_spray {

ActuatorMapping actuator_mapping(const Lease& lease) {
  return {lease.backend, lease.actuator_set_index, lease.off_value, lease.servo_instance,
          lease.off_pwm_us};
}

bool same_actuator_mapping(const ActuatorMapping& a, const ActuatorMapping& b) {
  return a.backend == b.backend && a.actuator_set_index == b.actuator_set_index &&
         a.off_value == b.off_value && a.servo_instance == b.servo_instance &&
         a.off_pwm_us == b.off_pwm_us;
}

std::string validate_lease(const Lease& l) {
  if (l.command_seq < 0) return "command_seq must be non-negative";
  if (l.backend != kBackendActuator && l.backend != kBackendServoPwm) {
    return "unsupported backend " + std::to_string(l.backend);
  }
  if (!(l.actuator_set_index >= 1 && l.actuator_set_index <= 6))
    return "actuator_set_index must be in 1..6";
  if (!std::isfinite(l.off_value) || l.off_value < -1.0 || l.off_value > 1.0) {
    return "off_value must be finite and in [-1, 1]";
  }
  if (!(l.servo_instance >= 1 && l.servo_instance <= 16)) return "servo_instance must be in 1..16";
  if (!(l.off_pwm_us >= 0 && l.off_pwm_us <= 2200)) return "off_pwm_us must be in 0..2200";
  return "";
}

LeaseMonitor::LeaseMonitor(double timeout_s) : timeout_s_(timeout_s) {
  if (!std::isfinite(timeout_s) || timeout_s <= 0.0)
    throw std::invalid_argument("timeout_s must be finite and positive");
}

bool LeaseMonitor::observe(const Lease& lease, double now_s, std::string* err) {
  if (!std::isfinite(now_s)) {
    invalidate("invalid controller lease: now_s must be finite");
    if (err) *err = "now_s must be finite";
    return false;
  }
  const std::string why = validate_lease(lease);
  if (!why.empty()) {
    invalidate("invalid controller lease: " + why);
    if (err) *err = why;
    return false;
  }
  last_lease_ = lease;
  last_receive_s_ = now_s;
  have_lease_ = true;
  invalid_ = false;
  invalid_reason_.clear();
  return true;
}

void LeaseMonitor::invalidate(const std::string& reason, bool shutdown) {
  invalid_ = true;
  shutdown_ = shutdown_ || shutdown;
  invalid_reason_ = reason.empty() ? "invalid controller lease" : reason;
}

OffReason LeaseMonitor::off_reason(double now_s) const {
  if (invalid_) return {shutdown_ ? OffCause::Shutdown : OffCause::Invalidated, invalid_reason_};
  if (!have_lease_) return {OffCause::NoLease, "no controller lease"};
  const double age = std::fmax(0.0, now_s - last_receive_s_);
  if (age > timeout_s_) {
    char buf[96];
    std::snprintf(buf, sizeof buf, "controller lease stale (%.3fs > %.3fs)", age, timeout_s_);
    return {OffCause::Stale, buf};
  }
  if (!last_lease_.allow_on) return {OffCause::Denied, "controller lease denies ON"};
  return {OffCause::None, ""};
}

}  // namespace dyx3_spray
