// freshness_watchdog — arrival-time freshness on the guard's steady clock. A source that was
// never heard is not fresh. Message stamps are never used (replay, clock skew).
#pragma once

namespace dyx3_motion_guard {

struct Watch {
  bool seen{false};
  double t_s{0.0};
  void touch(double now_s) {
    seen = true;
    t_s = now_s;
  }
  bool fresh(double now_s, double max_age_s) const {
    return seen && now_s >= t_s && (now_s - t_s) <= max_age_s;
  }
  double age(double now_s) const { return seen ? now_s - t_s : 1.0e9; }
};

}  // namespace dyx3_motion_guard
