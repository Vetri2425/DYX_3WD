// offboard_heartbeat — when the OffboardControlMode heartbeat runs and when OFFBOARD is requested.
// See docs/contracts/dyx3_px4_link.md sections 4 and 9. Pure C++.
#pragma once

#include <cstdint>

namespace dyx3_px4_link {

enum class OffboardState : uint8_t {
  Disabled = 0,  // no heartbeat
  Prestream,     // heartbeat with explicit STOP, waiting prestream_s before asking for the mode
  Requested,     // mode command sent (heartbeat STOP), waiting for nav_state == OFFBOARD
  Active,        // PX4 confirmed OFFBOARD: the only state that forwards the guard's command
  Failed,        // PX4 did not enter OFFBOARD in time (heartbeat STOP, no automatic retry)
  Lost  // was Active, then PX4 left OFFBOARD or the link was lost (heartbeat STOP while the
        // link is up): never re-requested silently
};

struct OffboardTiming {
  // DERIVED — NOT FROM V1 SPEC: PX4 needs a setpoint stream before it accepts OFFBOARD.
  double prestream_s{0.5};
  double confirm_timeout_s{2.0};
  // After enable(false) the heartbeat keeps running this long with the explicit STOP set, so PX4's
  // last setpoint is a zero before the stream is withdrawn (PXL-002).
  double disable_stop_s{0.3};
};

struct OffboardStep {
  bool publish_heartbeat{false};
  bool send_mode_command{false};  // true for exactly one step per request
  bool stop_only{false};          // the heartbeat must carry the explicit STOP set
  OffboardState state{OffboardState::Disabled};
};

class OffboardSession {
public:
  explicit OffboardSession(const OffboardTiming& t) : timing_(t) {}

  // Operator/gateway request. enable(true) from any state restarts the sequence. enable(false)
  // after enable(true) starts the disable_stop_s STOP window before the heartbeat is withdrawn.
  void enable(bool on, double now_s);

  // link_ok = handshake ok && session alive. Without it the heartbeat is withdrawn (the link cannot
  // produce a trustworthy zero): an Active session becomes Lost, a pending request restarts from
  // Prestream when it returns.
  // nav_state_offboard = latest vehicle_status.nav_state == 14.
  OffboardStep step(double now_s, bool link_ok, bool nav_state_offboard);

  // PX4 answered the mode request with a terminal refusal (VehicleCommandAck): Requested ->
  // Failed at once instead of after confirm_timeout_s (heartbeat STOP, no automatic retry).
  // Returns false and changes nothing in any other state.
  bool fail_requested();

  OffboardState state() const { return state_; }

private:
  OffboardTiming timing_;
  OffboardState state_{OffboardState::Disabled};
  bool enabled_{false};
  double since_s_{0.0};
  double stop_until_s_{-1.0};
};

}  // namespace dyx3_px4_link
