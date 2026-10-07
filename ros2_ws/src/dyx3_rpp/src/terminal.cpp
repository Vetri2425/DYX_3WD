#include "dyx3_rpp/terminal.hpp"

#include <algorithm>
#include <cmath>

#include "dyx3_geometry/path_length.hpp"

namespace dyx3_rpp {

bool is_closed_run(PathView pts, double close_loop_threshold_m, double close_loop_min_len_m) {
  if (pts.n < 3) return false;
  if (dyx3_geometry::path_length(pts) < close_loop_min_len_m) return false;
  const double gap = std::hypot(pts[0].n - pts[pts.n - 1].n, pts[0].e - pts[pts.n - 1].e);
  return gap <= close_loop_threshold_m;
}

double run_min_travel(const RunInfo& run, double min_goal_travel_m,
                      double closed_loop_min_travel_frac) {
  if (!run.valid) return min_goal_travel_m;
  if (run.closed) return closed_loop_min_travel_frac * run.length;
  return std::min(min_goal_travel_m, 0.5 * run.length);
}

double path_progress_at(const std::vector<double>& path_s, size_t path_size, int seg_idx,
                        double t) {
  if (path_s.empty() || path_s.size() != path_size) return 0.0;
  if (path_s.size() == 1) return 0.0;
  const int i = std::max(0, std::min(seg_idx, static_cast<int>(path_s.size()) - 2));
  const double alpha = std::max(0.0, std::min(1.0, t));
  return path_s[static_cast<size_t>(i)] +
         alpha * (path_s[static_cast<size_t>(i) + 1] - path_s[static_cast<size_t>(i)]);
}

double measure_tail_transit_m(PathView path, const std::vector<bool>& flags) {
  if (flags.empty() || flags.back()) return 0.0;
  int last_paint = -1;
  for (int i = static_cast<int>(flags.size()) - 1; i >= 0; --i) {
    if (flags[static_cast<size_t>(i)]) {
      last_paint = i;
      break;
    }
  }
  if (last_paint < 0) return 0.0;  // nothing painted anywhere: not a run-out
  const size_t n = std::min(path.n, flags.size());
  double total = 0.0;
  for (size_t i = static_cast<size_t>(last_paint); i + 1 < n; ++i) {
    total += std::hypot(path[i].n - path[i + 1].n, path[i].e - path[i + 1].e);
  }
  return total;
}

std::optional<double> run_remaining_along(const RunInfo& run, const std::vector<double>& path_s,
                                          size_t path_size, double path_travel_m) {
  if (!run.valid) return std::nullopt;
  if (run.length <= 0.0) return std::nullopt;
  if (path_s.empty() || path_s.size() != path_size) return std::nullopt;
  return std::max(0.0, run.length - path_travel_m);
}

double goal_tol_effective(double goal_tol, double tail_transit_m,
                          double transit_runout_goal_tolerance_m) {
  if (tail_transit_m <= 0.0) return goal_tol;
  if (transit_runout_goal_tolerance_m <= 0.0) return goal_tol;
  return std::max(goal_tol, std::min(transit_runout_goal_tolerance_m, 0.5 * tail_transit_m));
}

CaptureVerdict endpoint_capture_recovered(PathView path, Point pos,
                                          std::optional<double> remaining_along,
                                          const CaptureParams& p) {
  if (!p.enabled) return CaptureVerdict::Refused;
  if (path.n < 2) return CaptureVerdict::Refused;
  const double tol_eff =
      goal_tol_effective(p.goal_tol, p.tail_transit_m, p.transit_runout_goal_tolerance_m);
  if (!remaining_along || *remaining_along > tol_eff) return CaptureVerdict::Refused;

  const Point a = path[path.n - 2];
  const Point b = path[path.n - 1];
  double un = b.n - a.n, ue = b.e - a.e;
  const double seg = std::hypot(un, ue);
  if (seg < 1e-6) return CaptureVerdict::Refused;
  un /= seg;
  ue /= seg;
  const double dn = pos.n - b.n, de = pos.e - b.e;
  const double along = dn * un + de * ue;
  const double perp = std::fabs(dn * ue - de * un);
  if (along <= p.past_m) return CaptureVerdict::Refused;
  if (perp > p.max_miss_m) return CaptureVerdict::RefusedWideMiss;
  return CaptureVerdict::Recovered;
}

}  // namespace dyx3_rpp
