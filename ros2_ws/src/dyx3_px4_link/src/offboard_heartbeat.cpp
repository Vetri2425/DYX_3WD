#include "dyx3_px4_link/offboard_heartbeat.hpp"

namespace dyx3_px4_link {

void OffboardSession::enable(bool on, double now_s) {
  enabled_ = on;
  if (on) {
    state_ = OffboardState::Prestream;
    since_s_ = now_s;
  } else {
    state_ = OffboardState::Disabled;
  }
}

OffboardStep OffboardSession::step(double now_s, bool link_ok, bool nav_state_offboard) {
  OffboardStep out;
  if (!enabled_) {
    out.state = OffboardState::Disabled;
    return out;
  }
  if (!link_ok) {
    // No trustworthy zero can be published: withdraw the heartbeat and restart the sequence.
    if (state_ != OffboardState::Failed && state_ != OffboardState::Lost) {
      state_ = OffboardState::Prestream;
    }
    since_s_ = now_s;
    out.state = state_;
    return out;
  }
  out.publish_heartbeat = true;
  switch (state_) {
    case OffboardState::Prestream:
      if (now_s - since_s_ >= timing_.prestream_s) {
        state_ = OffboardState::Requested;
        since_s_ = now_s;
        out.send_mode_command = true;
      }
      break;
    case OffboardState::Requested:
      if (nav_state_offboard) {
        state_ = OffboardState::Active;
      } else if (now_s - since_s_ > timing_.confirm_timeout_s) {
        state_ = OffboardState::Failed;
      }
      break;
    case OffboardState::Active:
      if (!nav_state_offboard) state_ = OffboardState::Lost;
      break;
    case OffboardState::Failed:
    case OffboardState::Lost:
    case OffboardState::Disabled:
      break;
  }
  out.state = state_;
  return out;
}

}  // namespace dyx3_px4_link
