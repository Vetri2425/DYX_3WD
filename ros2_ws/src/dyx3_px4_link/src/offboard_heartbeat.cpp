#include "dyx3_px4_link/offboard_heartbeat.hpp"

namespace dyx3_px4_link {

void OffboardSession::enable(bool on, double now_s) {
  if (on) {
    state_ = OffboardState::Prestream;
    since_s_ = now_s;
    stop_until_s_ = -1.0;
  } else {
    if (enabled_) stop_until_s_ = now_s + timing_.disable_stop_s;
    state_ = OffboardState::Disabled;
  }
  enabled_ = on;
}

OffboardStep OffboardSession::step(double now_s, bool link_ok, bool nav_state_offboard) {
  OffboardStep out;
  if (!enabled_) {
    // Disabling never just drops the stream: PX4 would keep the last setpoint until its offboard
    // loss timeout. A trustworthy STOP is streamed for the window first (link permitting).
    out.state = OffboardState::Disabled;
    if (link_ok && now_s < stop_until_s_) {
      out.publish_heartbeat = true;
      out.stop_only = true;
    }
    return out;
  }
  if (!link_ok) {
    // No trustworthy zero can be published: withdraw the heartbeat. An established session is
    // Lost (OFFBOARD is never re-requested silently); a pending request restarts the sequence.
    if (state_ == OffboardState::Active) {
      state_ = OffboardState::Lost;
    } else if (state_ != OffboardState::Failed && state_ != OffboardState::Lost) {
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
  // Only a confirmed OFFBOARD session forwards the guard's command; every other state streams the
  // explicit STOP (PX4 may enter OFFBOARD before vehicle_status shows it).
  out.stop_only = state_ != OffboardState::Active;
  out.state = state_;
  return out;
}

}  // namespace dyx3_px4_link
