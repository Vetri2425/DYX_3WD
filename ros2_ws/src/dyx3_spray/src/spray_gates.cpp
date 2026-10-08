#include "dyx3_spray/spray_gates.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace dyx3_spray {

namespace {
std::string fix_name_of(int f) {
  switch (f) {
    case 0:
      return "NO_GPS";
    case 1:
      return "NO_FIX";
    case 2:
      return "2D_FIX";
    case 3:
      return "3D_FIX";
    case 4:
      return "DGPS";
    case 5:
      return "RTK_FLOAT";
    case 6:
      return "RTK_FIXED";
    case 7:
      return "STATIC";
    case 8:
      return "PPP";
    default:
      return "FIX_" + std::to_string(f);
  }
}
}  // namespace

RtkQuality evaluate_rtk_quality(int fix_type, std::optional<double> h_acc_m,
                                std::optional<double> sample_age_s, double timeout_s,
                                int min_fix_type, double max_h_acc_m, bool require_accuracy) {
  RtkQuality q;
  q.fix_name = fix_name_of(fix_type);
  timeout_s = std::max(0.0, timeout_s);
  if (!sample_age_s || !std::isfinite(*sample_age_s)) {
    q.reason = "gps data unavailable";
    return q;
  }
  const double age = *sample_age_s;
  if (age < 0.0 || age > timeout_s) {
    char b[96];
    std::snprintf(b, sizeof b, "gps stale (%.2fs > %.2fs)", std::max(0.0, age), timeout_s);
    q.reason = b;
    return q;
  }
  q.fresh = true;
  if (fix_type < min_fix_type) {
    q.reason =
        "gps fix " + std::to_string(fix_type) + " < required " + std::to_string(min_fix_type);
    return q;
  }
  // GPS_FIX_TYPE is an enum, not an accuracy scale: STATIC (7) and PPP (8) are not a rover
  // solution.
  if (fix_type != 5 && fix_type != 6) {
    q.reason = "gps fix " + std::to_string(fix_type) + " is not rover RTK_FLOAT/RTK_FIXED";
    return q;
  }
  std::optional<double> acc;
  if (h_acc_m && std::isfinite(*h_acc_m) && *h_acc_m >= 0.0) acc = *h_acc_m;
  if (!acc) {
    if (require_accuracy) {
      q.reason = "gps horizontal accuracy unknown";
      return q;
    }
    q.acceptable = true;
    return q;
  }
  if (max_h_acc_m > 0.0 && *acc > max_h_acc_m) {
    char b[120];
    std::snprintf(b, sizeof b, "gps horizontal accuracy %.3fm > %.3fm", *acc, max_h_acc_m);
    q.reason = b;
    return q;
  }
  q.acceptable = true;
  return q;
}

GateResult RtkGate::evaluate(const RtkGateConfig& cfg, int fix_type, std::optional<double> h_acc_m,
                             std::optional<double> sample_age_s, bool corrections_fresh,
                             double now_s) {
  if (!corrections_fresh) {
    have_since_ = false;
    return {false, "RTK corrections stale"};
  }
  if (!cfg.require_rtk_fix) {
    have_since_ = false;  // re-enabling the production gate must earn a new recovery hold
    return {true, ""};
  }
  const RtkQuality q =
      evaluate_rtk_quality(fix_type, h_acc_m, sample_age_s, cfg.fix_timeout_s, cfg.min_fix_type,
                           cfg.max_h_acc_m, cfg.require_accuracy);
  if (!q.fresh) {
    have_since_ = false;
    return {false, "gps stale"};
  }
  if (!q.acceptable) {
    have_since_ = false;
    return {false, q.reason};
  }
  if (!have_since_) {
    have_since_ = true;
    since_s_ = now_s;
  }
  const double hold = std::max(0.0, cfg.recover_hold_s);
  const double held = now_s - since_s_;
  if (held < hold) {
    char b[96];
    std::snprintf(b, sizeof b, "gps recovering (%.1f/%.1fs)", held, hold);
    return {false, b};
  }
  return {true, ""};
}

bool PivotGate::active(bool off_during_pivot, double timeout_s, double now_s) {
  if (!off_during_pivot) return false;
  if (!have_) return false;
  if (now_s - recv_s_ > std::max(0.0, timeout_s)) {
    have_ = false;  // stale: fail OPEN immediately and stay open until a fresh message arrives
    return false;
  }
  return pivoting_;
}

GateResult ownership_status(const OwnershipInputs& in) {
  if (!in.mission_running) return {false, "mission not running"};
  if (!in.rpp_known || !(in.rpp_age_s <= std::max(0.0, in.rpp_timeout_s)))
    return {false, "rpp stale"};
  if (in.rpp_mission_id != in.mission_id) return {false, "rpp mission mismatch"};
  switch (static_cast<RppState>(in.rpp_state)) {
    case RppState::Tracking:
    case RppState::Stopping:
    case RppState::Pivoting:
    case RppState::Creeping:
      return {true, ""};
    default:
      return {false, "rpp not marking"};
  }
}

GateResult auto_safety_status(const GateInputs& in) {
  if (!in.estop_clear) return {false, "emergency stop asserted or unknown"};
  if (!in.ownership.ok) return in.ownership;
  if (!in.heading_evidence.ok) return in.heading_evidence;
  if (!in.armed) return {false, "disarmed"};
  if (in.require_offboard && !in.offboard) return {false, "not OFFBOARD"};
  if (!in.path_loaded) return {false, "path not loaded"};
  if (!in.pose_fresh) return {false, "pose stale"};
  if (!in.velocity_fresh) return {false, "velocity stale"};
  if (!in.rtk.ok) return {false, in.rtk.reason};
  if (!in.tracking_seen) return {false, "awaiting tracking"};
  if (in.pivoting) return {false, "pivoting in place"};
  return {true, ""};
}

}  // namespace dyx3_spray
