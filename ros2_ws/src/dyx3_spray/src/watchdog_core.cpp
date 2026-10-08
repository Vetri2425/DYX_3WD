#include "dyx3_spray/watchdog_core.hpp"

#include <algorithm>
#include <stdexcept>

namespace dyx3_spray {

WatchdogCore::WatchdogCore(const WatchdogParams& p)
    : p_(p), fallback_(p.fallback), mon_(p.lease_timeout_s) {
  const std::string why = validate_lease(fallback_);
  if (!why.empty()) throw std::invalid_argument("watchdog fallback mapping invalid: " + why);
}

void WatchdogCore::on_lease(const Lease& l, double now_s) {
  const ActuatorMapping previous = actuator_mapping(mapping());
  std::string err;
  if (!mon_.observe(l, now_s, &err)) {
    next_off_s_ = 0.0;  // malformed: revoke ON and retry OFF at once
    return;
  }
  if (!same_actuator_mapping(previous, actuator_mapping(mapping()))) {
    // Proof and any old OFF transaction belong to the previous physical mapping. Its eventual ACK
    // is stale even if it arrives after a command for the new mapping has been dispatched.
    off_confirmed_mapping_.reset();
    inflight_ = false;
    inflight_mapping_.reset();
    mapping_off_required_ = true;
    next_off_s_ = 0.0;
    burst_until_s_ = now_s + std::max(0.0, p_.off_burst_duration_s);
  }
}

void WatchdogCore::on_ack(uint32_t seq, bool success, double /*now_s*/) {
  if (!inflight_ || seq != inflight_seq_) return;
  inflight_ = false;
  const bool proves_current_mapping =
      inflight_mapping_ && same_actuator_mapping(*inflight_mapping_, actuator_mapping(mapping()));
  inflight_mapping_.reset();
  if (success && proves_current_mapping) {
    off_confirmed_mapping_ = actuator_mapping(mapping());
    mapping_off_required_ = false;
  } else {
    off_confirmed_mapping_.reset();
    next_off_s_ = 0.0;
  }
}

void WatchdogCore::begin_shutdown(double /*now_s*/) {
  mon_.invalidate("service shutdown", true);
  next_off_s_ = 0.0;
}

std::optional<OffCommand> WatchdogCore::tick(double now_s) {
  if (inflight_ && now_s - inflight_since_ > std::max(0.1, p_.command_ack_timeout_s)) {
    inflight_ =
        false;  // an acknowledgement that never arrives is a failure: the next OFF goes out at once
    inflight_mapping_.reset();
    off_confirmed_mapping_.reset();
    next_off_s_ = 0.0;
  }
  const OffReason r = mon_.off_reason(now_s);
  if (!have_last_cause_ || r.cause != last_cause_) {
    if (r.required()) {
      next_off_s_ = 0.0;
      burst_until_s_ = now_s + std::max(0.0, p_.off_burst_duration_s);
    }
    last_cause_ = r.cause;
    have_last_cause_ = true;
  }
  const bool off_required = r.required() || mapping_off_required_;
  if (off_required && !inflight_ && now_s >= next_off_s_) {
    const double hz = std::max(0.2, now_s < burst_until_s_ ? p_.off_burst_hz : p_.off_retry_hz);
    next_off_s_ = now_s + 1.0 / hz;
    inflight_ = true;
    inflight_seq_ = ++seq_;
    inflight_since_ = now_s;
    inflight_mapping_ = actuator_mapping(mapping());
    return OffCommand{inflight_seq_, mapping()};
  }
  return std::nullopt;
}

WatchdogStatus WatchdogCore::status(double now_s) const {
  WatchdogStatus s;
  const OffReason r = mon_.off_reason(now_s);
  s.allow_on = !r.required() && !mapping_off_required_;
  s.alive = true;
  s.off_authority_ready =
      off_confirmed_mapping_ &&
      same_actuator_mapping(*off_confirmed_mapping_, actuator_mapping(mapping()));
  s.off_inflight = inflight_;
  s.off_reason = r.text;
  s.lease_age_s = mon_.has_receive() ? std::max(0.0, now_s - mon_.last_receive_s()) : 1e9;
  s.backend = mapping().backend;
  return s;
}

}  // namespace dyx3_spray
