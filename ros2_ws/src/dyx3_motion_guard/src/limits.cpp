#include "dyx3_motion_guard/limits.hpp"

#include <algorithm>
#include <cmath>

namespace dyx3_motion_guard {

bool limits_valid(const Limits& l) {
  const auto pos = [](float v) { return std::isfinite(v) && v > 0.0F; };
  const auto opt = [](float v) { return std::isnan(v) || (std::isfinite(v) && v > 0.0F); };
  return pos(l.max_forward_speed_mps) && std::isfinite(l.max_reverse_speed_mps) &&
         l.max_reverse_speed_mps >= 0.0F && pos(l.max_yaw_rate_radps) && pos(l.max_accel_mps2) &&
         pos(l.max_decel_mps2) && opt(l.max_yaw_accel_radps2) && opt(l.max_jerk_mps3);
}

double step_toward(double v, double target, double dt, double accel, double decel) {
  if (v < 0.0) return -step_toward(-v, -target, dt, accel, decel);
  if (v == 0.0) {
    if (target > 0.0) return std::min(target, accel * dt);
    if (target < 0.0) return std::max(target, -accel * dt);
    return 0.0;
  }
  if (target >= v) return std::min(target, v + accel * dt);    // away from zero
  if (target >= 0.0) return std::max(target, v - decel * dt);  // toward zero
  const double t0 = v / decel;  // crossing zero: decelerate to zero first
  if (dt <= t0) return v - decel * dt;
  return std::max(target, -accel * (dt - t0));
}

Limited apply_limits(const Motion& in, double dt, const Limits& lim, LimitState& st) {
  Limited out;
  out.motion = in;
  if (in.mode == Mode::Stop) {
    out.motion = canonical_stop();
    reset(st);
    return out;
  }
  // 1. static clamps
  float v = in.speed_body_x;
  const float vmax = lim.max_forward_speed_mps;
  const float vmin = -lim.max_reverse_speed_mps;
  if (v > vmax) {
    v = vmax;
    out.clamped = true;
  }
  if (v < vmin) {
    v = vmin;
    out.clamped = true;
  }
  float r = in.yaw_rate_setpoint;
  if (uses_rate(in.mode)) {
    const float rmax = lim.max_yaw_rate_radps;
    if (r > rmax) {
      r = rmax;
      out.clamped = true;
    }
    if (r < -rmax) {
      r = -rmax;
      out.clamped = true;
    }
  }

  // 2. speed ramp. PIVOT requires speed 0 (contract), so it is never stretched.
  if (in.mode == Mode::Pivot) {
    v = 0.0F;
    st.last_speed = 0.0;
    st.last_accel = 0.0;
  } else {
    double nv = step_toward(st.last_speed, static_cast<double>(v), dt, lim.max_accel_mps2,
                            lim.max_decel_mps2);
    if (!std::isnan(lim.max_jerk_mps3) && dt > 0.0) {
      // Change of acceleration per step is bounded; the result may fall short of the target.
      const double a_want = (nv - st.last_speed) / dt;
      const double da = static_cast<double>(lim.max_jerk_mps3) * dt;
      const double a = std::clamp(a_want, st.last_accel - da, st.last_accel + da);
      nv = st.last_speed + a * dt;
    }
    if (std::fabs(nv - static_cast<double>(v)) > 1e-9) out.clamped = true;
    st.last_accel = dt > 0.0 ? (nv - st.last_speed) / dt : 0.0;
    st.last_speed = nv;
    v = static_cast<float>(nv);
  }

  // 3. yaw-rate change limit (optional)
  if (uses_rate(in.mode)) {
    if (!std::isnan(lim.max_yaw_accel_radps2)) {
      const double step = static_cast<double>(lim.max_yaw_accel_radps2) * dt;
      const double nr =
          std::clamp(static_cast<double>(r), st.last_rate - step, st.last_rate + step);
      if (std::fabs(nr - static_cast<double>(r)) > 1e-9) out.clamped = true;
      r = static_cast<float>(nr);
    }
    st.last_rate = static_cast<double>(r);
  } else {
    st.last_rate = 0.0;
  }

  out.motion.speed_body_x = v;
  out.motion.yaw_rate_setpoint = uses_rate(in.mode) ? r : in.yaw_rate_setpoint;
  return out;
}

}  // namespace dyx3_motion_guard
