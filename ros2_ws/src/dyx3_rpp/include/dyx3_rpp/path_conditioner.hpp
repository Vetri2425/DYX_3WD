// path_conditioner — turns the raw mission polyline into ordered runs (segment / smooth) ready to
// track. Contract: docs/contracts/rpp_path_conditioner.md. Pure C++, no ROS. Ports of the
// prototype's _path_cb run building and its classmethods
// (_simplify_path_for_profile, _classify_auto_profile, _split_runs_by_flag, _merge_collinear_runs,
// _absorb_short_connectors, _split_run_at_corners, _smooth_corners, _is_closed_run), proven against
// the verbatim Python (gate4). Runs ONCE per mission install, never in the control loop: it
// allocates freely.
#pragma once

#include <cstdint>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "dyx3_geometry/point.hpp"

namespace dyx3_rpp {

using dyx3_geometry::Point;
using Flags = std::vector<unsigned char>;  // 0/1 per point
using PtKey = std::pair<int64_t, int64_t>;
using KeySet = std::set<PtKey>;

enum class Profile : uint8_t { Segment = 0, Smooth = 1 };
const char* to_string(Profile p);

struct PointRun {
  std::vector<Point> pts;
  Flags flags;
};

// 1 mm quantisation (round half to even, as Python's round()).
PtKey pt_key(Point p);

// Drops duplicates (<1 um) and same-heading vertices; keeps endpoints, flag boundaries, must-hit
// points, corners (heading change from the last RETAINED point > tol) and, when max_offset_m > 0,
// Douglas-Peucker anchors against the collapsing span.
PointRun simplify_for_profile(const std::vector<Point>& pts, const Flags* flags,
                              double collinear_tol_deg = 5.0, double max_offset_m = 0.0,
                              const KeySet* must_hit = nullptr);

Profile classify_auto_profile(const std::vector<Point>& pts, double threshold_deg);

std::vector<PointRun> split_runs_by_flag(const std::vector<Point>& pts, const Flags& flags);
bool runs_collinear(const std::vector<Point>& prev, const std::vector<Point>& next,
                    double threshold_deg);
bool is_short_transit_run(const std::vector<Point>& pts, const Flags& flags, double max_len_m);
std::vector<PointRun> merge_collinear_runs(const std::vector<PointRun>& runs, double threshold_deg,
                                           double transit_merge_max_len_m = 0.0);

struct SimplifiedIdx {
  std::vector<Point> pts;
  Flags flags;
  std::vector<int> idx;
};
SimplifiedIdx simplify_with_indices(const std::vector<Point>& pts, const Flags& flags,
                                    double collinear_tol_deg = 5.0);

PointRun absorb_short_connectors(const std::vector<Point>& pts, const Flags& flags,
                                 double threshold_deg, double connector_absorb_m,
                                 double min_corner_deg = 20.0);
std::vector<PointRun> split_run_at_corners(const std::vector<Point>& pts, const Flags& flags,
                                           double threshold_deg);

// Replaces each interior vertex with an inscribed arc (kappa_max = 1/radius). `skipped` (optional)
// counts vertices left sharp because the adjoining segments are too short for the radius.
PointRun smooth_corners(const std::vector<Point>& pts, double radius, int arc_pts,
                        const Flags* flags, int* skipped = nullptr);

double pts_length(const std::vector<Point>& pts);
std::vector<double> pts_cumulative_lengths(const std::vector<Point>& pts);

struct ConditionParams {
  std::string tracking_profile{"auto"};  // normalised by normalize_tracking_profile
  double segment_corner_threshold_deg;
  double connector_absorb_m;
  double connector_min_corner_deg;
  double transit_merge_max_len_m;
  double segment_simplify_max_offset_m;
  double corner_smooth_radius_m;
  int corner_smooth_arc_pts;
  double path_resample_spacing_m;
  double close_loop_threshold_m;
  double close_loop_min_len_m;
};

std::string normalize_tracking_profile(const std::string& value);

struct ConditionedRun {
  std::vector<Point> pts;
  Flags flags;
  std::vector<unsigned char> must_hit;  // per conditioned point (bit1 of the artifact z)
  Profile profile{Profile::Segment};
  double length{0.0};
  std::vector<double> cum_s;
  bool closed{false};
};

struct RawPoint {
  double n;
  double e;
  int z;  // bitfield: bit0 spray ON, bit1 must-hit
};

// The whole of _path_cb after the frame check: runs, slivers (< 5 cm, when more than one run)
// dropped. Empty input -> empty result. `dropped_slivers` (optional) reports how many runs were
// dropped.
std::vector<ConditionedRun> condition_path(const std::vector<RawPoint>& raw,
                                           const ConditionParams& p,
                                           KeySet* must_hit_keys_out = nullptr,
                                           int* dropped_slivers = nullptr);

bool is_closed_run(const std::vector<Point>& pts, double threshold_m, double min_len_m);

}  // namespace dyx3_rpp
