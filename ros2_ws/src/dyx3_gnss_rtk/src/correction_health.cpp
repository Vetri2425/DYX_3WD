#include "dyx3_gnss_rtk/correction_health.hpp"

namespace dyx3_gnss_rtk {

void CorrectionHealth::on_frame(double now_s, size_t bytes) {
  have_frame_ = true;
  last_frame_s_ = now_s;
  ++frames_;
  bytes_ += bytes;
  stamps_.push_back(now_s);
  while (!stamps_.empty() && now_s - stamps_.front() > window_s_) stamps_.pop_front();
}

double CorrectionHealth::age_s(double now_s) const {
  return have_frame_ ? now_s - last_frame_s_ : 1.0e9;
}

bool CorrectionHealth::fresh(double now_s, double limit_s) const {
  return have_frame_ && connected() && age_s(now_s) >= 0.0 && age_s(now_s) <= limit_s;
}

double CorrectionHealth::rate_hz(double now_s) const {
  size_t n = 0;
  double first = 0.0;
  for (const double t : stamps_) {
    if (now_s - t <= window_s_) {
      if (n == 0) first = t;
      ++n;
    }
  }
  if (n < 2) return 0.0;
  const double span = last_frame_s_ - first;  // first..last frame inside the window
  return span > 0.0 ? static_cast<double>(n - 1) / span : 0.0;
}

std::optional<FixTransition> FixMonitor::update(double now_s, uint8_t fix_type,
                                                bool corrections_fresh, double correction_age_s) {
  if (!have_) {
    have_ = true;
    fix_ = fix_type;
    return std::nullopt;
  }
  if (fix_type == fix_) return std::nullopt;
  FixTransition t;
  t.t_s = now_s;
  t.from = fix_;
  t.to = fix_type;
  t.correction_age_s = correction_age_s;
  t.corrections_fresh = corrections_fresh;
  const bool was_rtk = fix_ == 5 || fix_ == 6;
  const bool is_rtk = fix_type == 5 || fix_type == 6;
  t.suspicious = was_rtk && !is_rtk && corrections_fresh;
  fix_ = fix_type;
  ++count_;
  log_.push_back(t);
  if (log_.size() > 256) log_.erase(log_.begin());
  return t;
}

}  // namespace dyx3_gnss_rtk
