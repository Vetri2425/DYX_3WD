// terminal — run-level quantities, goal tolerance and endpoint-capture recovery.
// Contract: docs/contracts/rpp_terminal.md. Pure C++. Ports of _is_closed_run, _run_min_travel,
// _path_progress_at, _measure_tail_transit_m, _run_remaining_along, _goal_tol_effective and the
// pure part of _endpoint_capture_recovered, proven against the verbatim Python
// (gate4_equivalence_test).
#pragma once

#include <optional>
#include <vector>

#include "dyx3_geometry/point.hpp"

namespace dyx3_rpp {

using dyx3_geometry::PathView;
using dyx3_geometry::Point;

// A closed run has >= 3 points, length >= min_len and |first - last| <= threshold.
bool is_closed_run(PathView pts, double close_loop_threshold_m, double close_loop_min_len_m);

struct RunInfo {
  double length{0.0};
  bool closed{false};
  bool valid{false};  // a run is installed
};

// Along-path travel required before the active run may declare DONE: closed => frac * length; open
// => min(min_goal_travel_m, 0.5 * length). No run => min_goal_travel_m.
double run_min_travel(const RunInfo& run, double min_goal_travel_m,
                      double closed_loop_min_travel_frac);

// Arc length of a segment projection given the cumulative lengths; 0 if the cache is unusable.
double path_progress_at(const std::vector<double>& path_s, size_t path_size, int seg_idx, double t);

// Along-path length of an unpainted TAIL on a run that paints; 0 when the run does not end
// unpainted or paints nowhere (a transit leg is not a run-out).
double measure_tail_transit_m(PathView path, const std::vector<bool>& spray_flags);

// Monotonic remaining distance to the run end; nullopt when the progress cache is unusable (the
// caller must then keep its per-segment measure: a broken cache never removes braking that
// existed).
std::optional<double> run_remaining_along(const RunInfo& run, const std::vector<double>& path_s,
                                          size_t path_size, double path_travel_m);

// goal_tol, or max(goal_tol, min(runout_tol, 0.5 * tail)) when the run has an unpainted tail.
double goal_tol_effective(double goal_tol, double tail_transit_m,
                          double transit_runout_goal_tolerance_m);

struct CaptureParams {
  bool enabled{true};
  double goal_tol{0.02};
  double tail_transit_m{0.0};
  double transit_runout_goal_tolerance_m{0.10};
  double past_m{0.02};
  double max_miss_m{0.10};
};

enum class CaptureVerdict { Refused, Recovered, RefusedWideMiss };

// "Drove THROUGH the run end without entering the ball". All required: remaining_along <=
// goal_tol_eff (nullopt refuses), the rover is past the end plane by more than past_m along the
// final tangent, and the perpendicular residual <= max_miss_m (a wide miss is refused and reported,
// never accepted).
CaptureVerdict endpoint_capture_recovered(PathView path, Point pos,
                                          std::optional<double> remaining_along,
                                          const CaptureParams& p);

}  // namespace dyx3_rpp
