// rtk_gate — RTK quality gate. Rules carried from the prototype's rtk_quality.py (re-validate at
// GATE 4): only RTK_FLOAT (5) / RTK_FIXED (6) qualify, at or above the configured minimum;
// unknown accuracy (0) fails closed; stale or never-heard fails.
#pragma once

#include <cmath>
#include <cstdint>

namespace dyx3_motion_guard {

struct RtkIn {
  bool fresh{false};
  uint8_t fix_type{0};
  bool corrections_fresh{false};
  float horizontal_accuracy_m{0.0F};  // 0 is the "unknown" sentinel (A14)
};

struct RtkConfig {
  uint8_t min_fix_type{6};
  float max_hrms_m{0.10F};
};

inline bool rtk_ok(const RtkIn& r, const RtkConfig& c) {
  if (!r.fresh || !r.corrections_fresh) return false;
  if (r.fix_type != 5 && r.fix_type != 6) return false;
  if (r.fix_type < c.min_fix_type) return false;
  const float a = r.horizontal_accuracy_m;
  if (!std::isfinite(a) || a <= 0.0F) return false;
  return !(c.max_hrms_m > 0.0F && a > c.max_hrms_m);
}

}  // namespace dyx3_motion_guard
