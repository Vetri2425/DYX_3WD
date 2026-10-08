// safety_lease — strict validation of the spray lease and the freshness monitor the independent
// watchdog runs. Contract: docs/contracts/dyx3_spray.md section 4. Pure C++, no ROS,
// caller-injected monotonic time. Port of spray_safety_lease.validate_lease / SprayLeaseMonitor
// (the JSON transport is replaced by the typed dyx3_interfaces/SprayLease; the validation rules are
// carried).
#pragma once

#include <cstdint>
#include <string>

namespace dyx3_spray {

constexpr int kBackendActuator = 0;  // mavlink_actuator -> DO_SET_ACTUATOR (187)
constexpr int kBackendServoPwm = 1;  // mavlink_servo_pwm -> DO_SET_SERVO (183)

// Plain wide ints so that an out-of-range wire value is representable and rejected, never silently
// wrapped.
struct Lease {
  bool allow_on{false};
  int64_t command_seq{0};
  int backend{kBackendActuator};
  int actuator_set_index{1};
  double off_value{-1.0};
  int servo_instance{1};
  int off_pwm_us{0};
};

// Fields that determine the physical destination/value of an OFF command. Lease permissions,
// sequence, and freshness are deliberately excluded from actuator identity.
struct ActuatorMapping {
  int backend{kBackendActuator};
  int actuator_set_index{1};
  double off_value{-1.0};
  int servo_instance{1};
  int off_pwm_us{0};
};

ActuatorMapping actuator_mapping(const Lease& lease);
bool same_actuator_mapping(const ActuatorMapping& a, const ActuatorMapping& b);

// Empty string when valid; otherwise the reason. A lease that cannot safely describe the physical
// OFF command is invalid.
std::string validate_lease(const Lease& l);

// Why OFF is currently required. None means a fresh, valid lease explicitly allows ON.
enum class OffCause : uint8_t { None = 0, NoLease, Invalidated, Stale, Denied, Shutdown };

struct OffReason {
  OffCause cause{OffCause::NoLease};
  std::string text;  // Python-identical text ("" for None)
  bool required() const { return cause != OffCause::None; }
};

class LeaseMonitor {
public:
  explicit LeaseMonitor(
      double timeout_s = 0.35);  // throws std::invalid_argument unless finite and > 0

  // Returns true when accepted. An invalid lease is NOT stored: the monitor is invalidated (ON
  // revoked at once, the last known OFF mapping retained) and `*err` says why.
  bool observe(const Lease& lease, double now_s, std::string* err = nullptr);
  void invalidate(const std::string& reason, bool shutdown = false);
  OffReason off_reason(double now_s) const;

  bool has_lease() const { return have_lease_; }
  const Lease& last_lease() const { return last_lease_; }
  bool has_receive() const { return have_lease_; }
  double last_receive_s() const { return last_receive_s_; }
  double timeout_s() const { return timeout_s_; }

private:
  double timeout_s_;
  bool have_lease_{false};
  Lease last_lease_;
  double last_receive_s_{0.0};
  bool invalid_{false};
  bool shutdown_{false};
  std::string invalid_reason_;
};

}  // namespace dyx3_spray
