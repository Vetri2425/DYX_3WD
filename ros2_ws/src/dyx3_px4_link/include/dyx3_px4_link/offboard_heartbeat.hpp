// offboard_heartbeat — when the OffboardControlMode heartbeat runs and when OFFBOARD is requested.
// See docs/contracts/dyx3_px4_link.md sections 4 and 9. Pure C++.
#pragma once

#include <cstdint>

namespace dyx3_px4_link {

enum class OffboardState : uint8_t {
  Disabled = 0,  // no heartbeat
  Prestream,     // heartbeat with explicit STOP, waiting prestream_s before asking for the mode
  Requested,     // mode command sent, waiting for nav_state == OFFBOARD
  Active,        // PX4 confirmed OFFBOARD
  Failed,        // PX4 did not enter OFFBOARD in time (heartbeat continues, no automatic retry)
  Lost           // was Active, PX4 left OFFBOARD (its own failsafe): never re-requested silently
};

struct OffboardTiming {
  // DERIVED — NOT FROM V1 SPEC: PX4 needs a setpoint stream before it accepts OFFBOARD.
  double prestream_s{0.5};
  double confirm_timeout_s{2.0};
};

struct OffboardStep {
  bool publish_heartbeat{false};
  bool send_mode_command{false};  // true for exactly one step per request
  OffboardState state{OffboardState::Disabled};
};

class OffboardSession {
public:
  explicit OffboardSession(const OffboardTiming& t) : timing_(t) {}

  // Operator/gateway request. enable(true) from any state restarts the sequence.
  void enable(bool on, double now_s);

  // link_ok = handshake ok && session alive. Without it the heartbeat is withdrawn (the link cannot
  // produce a trustworthy zero) and a pending request restarts from Prestream when it returns.
  // nav_state_offboard = latest vehicle_status.nav_state == 14.
  OffboardStep step(double now_s, bool link_ok, bool nav_state_offboard);

  OffboardState state() const { return state_; }

private:
  OffboardTiming timing_;
  OffboardState state_{OffboardState::Disabled};
  bool enabled_{false};
  double since_s_{0.0};
};

}  // namespace dyx3_px4_link
