// watchdog_core — the decision logic of the independent spray watchdog. Contract:
// docs/contracts/dyx3_spray.md section 4. Pure C++, clock-injected. It only ever asks for OFF. Port
// of SpraySafetyWatchdogNode._tick/_dispatch_off/_off_done.
//
// DERIVED — NOT FROM V1 SPEC: the prototype compared the full reason TEXT to detect an edge, and
// the stale text carries the lease age, so while a lease stayed stale it re-armed the 20 Hz burst
// on every tick forever. Here the edge is the CAUSE (no lease / invalidated / stale / denied /
// shutdown): one 20 Hz burst per cause change, then the 2 Hz retry.
#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "dyx3_spray/safety_lease.hpp"

namespace dyx3_spray {

struct WatchdogParams {
  double lease_timeout_s{0.35};
  double off_retry_hz{2.0};
  double off_burst_hz{20.0};
  double off_burst_duration_s{1.5};
  double command_ack_timeout_s{1.0};
  // Startup mapping, used until a valid lease provides the authoritative one.
  Lease fallback{false, 0, kBackendActuator, 1, -1.0, 1, 0};
};

// A request to close the valve. `lease` carries the OFF mapping (backend, index, value, servo,
// pwm).
struct OffCommand {
  uint32_t seq;
  Lease mapping;
};

struct WatchdogStatus {
  bool allow_on{false};
  bool alive{true};
  bool off_authority_ready{false};
  bool off_inflight{false};
  std::string off_reason;
  double lease_age_s{1e9};
  int backend{kBackendActuator};
};

class WatchdogCore {
public:
  explicit WatchdogCore(const WatchdogParams& p);  // throws if the fallback mapping is invalid

  void on_lease(const Lease& l, double now_s);
  // The FCU path answered the OFF with `seq`. Only the in-flight sequence counts.
  void on_ack(uint32_t seq, bool success, double now_s);
  void begin_shutdown(double now_s);
  // Returns an OFF to send when one is due. Call at >= 20 Hz.
  std::optional<OffCommand> tick(double now_s);
  WatchdogStatus status(double now_s) const;
  const Lease& mapping() const { return mon_.has_lease() ? mon_.last_lease() : fallback_; }
  bool inflight() const { return inflight_; }

private:
  WatchdogParams p_;
  Lease fallback_;
  LeaseMonitor mon_;
  bool inflight_{false};
  uint32_t seq_{0};
  uint32_t inflight_seq_{0};
  double inflight_since_{0.0};
  std::optional<ActuatorMapping> inflight_mapping_;
  std::optional<ActuatorMapping> off_confirmed_mapping_;
  bool mapping_off_required_{false};
  double next_off_s_{0.0};
  double burst_until_s_{0.0};
  bool have_last_cause_{false};
  OffCause last_cause_{OffCause::None};
};

}  // namespace dyx3_spray
