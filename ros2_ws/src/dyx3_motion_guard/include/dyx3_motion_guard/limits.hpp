// limits — speed, yaw-rate, acceleration and jerk limiting. See docs/contracts/dyx3_motion_guard.md
// section 5. Pure C++. Never applied to STOP; a limit is a clamp, not a refusal.
#pragma once

#include <limits>

#include "dyx3_motion_guard/motion_types.hpp"

namespace dyx3_motion_guard {

struct Limits {
  // Carried from the prototype's RPP defaults and flagged for GATE 4 re-validation. Reverse is a
  // DERIVED production hard envelope: 0.10 m/s passes the RPP active-brake cap (0.08 m/s) while
  // bounding larger reverse requests. It is an initial bench value, not a field-tuned final value.
  float max_forward_speed_mps{1.0F};
  float max_reverse_speed_mps{0.10F};
  float max_yaw_rate_radps{0.45F};
  float max_accel_mps2{0.20F};
  float max_decel_mps2{0.50F};
  float max_yaw_accel_radps2{std::numeric_limits<float>::quiet_NaN()};  // NaN = off, no source
  float max_jerk_mps3{std::numeric_limits<float>::quiet_NaN()};         // NaN = off, no source
};

// Finite and positive where required; NaN allowed only for the two optional limits.
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
