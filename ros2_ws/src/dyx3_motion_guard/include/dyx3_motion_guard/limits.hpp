// limits — hard speed/yaw envelopes plus a test-only legacy profile shaper.
// See docs/contracts/dyx3_motion_guard.md section 5. Production Motion Guard never reshapes a
// valid RPP speed/yaw-rate profile; RPP owns that profile.
#pragma once

#include <limits>

#include "dyx3_motion_guard/motion_types.hpp"

namespace dyx3_motion_guard {

struct Limits {
  // Production hard envelopes. Reverse is a DERIVED production value: 0.10 m/s passes the RPP
  // active-brake cap (0.08 m/s) while bounding larger reverse requests.
  float max_forward_speed_mps{1.0F};
  float max_reverse_speed_mps{0.10F};
  float max_yaw_rate_radps{0.45F};

  // Legacy shaping retained only for pure-C++ safety-test coverage. The ROS production node never
  // enables this mode and does not expose these four values as ROS parameters (B2 / review H5).
  bool profile_shaping_test_mode{false};
  float max_accel_mps2{0.20F};
  float max_decel_mps2{0.50F};
  float max_yaw_accel_radps2{std::numeric_limits<float>::quiet_NaN()};
  float max_jerk_mps3{std::numeric_limits<float>::quiet_NaN()};
};

// Hard envelopes are always validated; test-mode shaping values remain structurally validated too.
bool limits_valid(const Limits& l);

struct LimitState {
  double last_speed{0.0};
  double last_accel{0.0};
  double last_rate{0.0};
};

inline void reset(LimitState& s) { s = LimitState{}; }

struct Limited {
  Motion motion;
  bool clamped{false};
};

// Signed one-dimensional rate limiter: away from zero uses `accel`, toward zero `decel`, a sign
// change passes through zero. Exposed for tests.
double step_toward(double v, double target, double dt, double accel, double decel);

// dt in seconds (caller bounds it). STOP and PIVOT bypass the speed ramp: the contract requires
// zero speed there, so the deceleration limit is not applied to them and the state is zeroed.
Limited apply_limits(const Motion& in, double dt, const Limits& lim, LimitState& st);

}  // namespace dyx3_motion_guard
