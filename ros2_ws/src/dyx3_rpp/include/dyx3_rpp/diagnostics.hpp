// diagnostics — control-loop timing for RppStatus (loop_jitter_us / loop_jitter_max_us /
// loop_overrun_count). Pure C++, no ROS. Contract: docs/contracts/dyx3_rpp.md.
#pragma once

#include <cstdint>

namespace dyx3_rpp {

// Measures the actual period between control ticks against the target. Jitter is actual minus
// target (microseconds, signed). An overrun is a period longer than overrun_factor x target.
//
// DERIVED — NOT FROM V1 SPEC: overrun_factor 1.5. The prototype measured nothing here; the figure
// only has to separate "one late tick" from "a stall" and is a diagnostic, not a gate. Timing is
// NOT provable off-target: these numbers are only meaningful on the Jetson.
class LoopTimer {
public:
  explicit LoopTimer(double target_period_us, double overrun_factor = 1.5)
      : target_us_(target_period_us), factor_(overrun_factor) {}

  // Call once per control tick with a monotonic clock in ns. The first call only records the time.
  void note(int64_t now_ns) {
    if (have_last_) {
      const double actual_us = static_cast<double>(now_ns - last_ns_) * 1e-3;
      jitter_us_ = actual_us - target_us_;
      const double a = jitter_us_ < 0.0 ? -jitter_us_ : jitter_us_;
      if (a > max_abs_jitter_us_) max_abs_jitter_us_ = a;
      if (actual_us > factor_ * target_us_) ++overruns_;
    }
    last_ns_ = now_ns;
    have_last_ = true;
  }

  double jitter_us() const { return jitter_us_; }
  double max_abs_jitter_us() const { return max_abs_jitter_us_; }
  uint64_t overruns() const { return overruns_; }
  double target_us() const { return target_us_; }

private:
  double target_us_;
  double factor_;
  bool have_last_{false};
  int64_t last_ns_{0};
  double jitter_us_{0.0};
  double max_abs_jitter_us_{0.0};
  uint64_t overruns_{0};
};

}  // namespace dyx3_rpp
