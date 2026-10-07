#include "dyx3_px4_link/dds_session.hpp"

namespace dyx3_px4_link {

StalenessReport StalenessMonitor::evaluate(double now_s) {
  StalenessReport r;
  for (int i = 0; i < kTopicCount; ++i) {
    if (!seen_[i]) {
      r.mask |= (1U << i);
      continue;
    }
    r.any_seen = true;
    const double age = now_s - last_[i];
    if (age > r.worst_age_s) r.worst_age_s = age;
    if (age > limits_.max_age_s[i]) r.mask |= (1U << i);
  }
  r.session_alive = (r.mask & (1U << kTimesync)) == 0U;

  if (r.session_alive) {
    if (had_dead_after_alive_) {
      ++resets_;
      reset_flag_ = true;
      had_dead_after_alive_ = false;
    }
    was_alive_ = true;
  } else if (was_alive_) {
    had_dead_after_alive_ = true;
  }
  return r;
}

}  // namespace dyx3_px4_link
