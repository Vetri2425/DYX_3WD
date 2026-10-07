// replay — drives a synthetic rover ALONG a planned path through the boundary projection and
// reports where the reported station teleports or the reported MARK/TRANSIT flag is wrong. Pure
// C++, no ROS.
//
// Why: the open projection-continuity defect (docs/contracts/dyx3_spray.md section 6) appears when
// a path doubles back within centimetres of itself. The decision whether to enable
// `projection_direction_gate_deg` needs 5 to 10 real missions with the gate off and on. This does
// that on the PLANNED geometry of any artifact; it is not a recorded trace (bag replay stays a
// LOCAL ACTION), but it already shows which missions are exposed.
#pragma once

#include <cstddef>

#include "dyx3_spray/boundary_projection.hpp"

namespace dyx3_spray {

struct ReplayConfig {
  double step_m{0.007};            // 0.35 m/s at 50 Hz
  double lateral_offset_m{0.010};  // mean offset to the right of the path
  double noise_amp_m{0.003};       // lateral wobble amplitude
  double window_back_m{0.5};       // the projection window (the shipped defaults)
  double window_fwd_m{2.0};
  double reacquire_dist_m{1.0};
  double gate_deg{0.0};       // projection_direction_gate_deg (0 = disabled, the shipped default)
  double teleport_m{0.25};    // a station change larger than this in one sample
  double flag_guard_m{0.05};  // flag disagreement within this distance of a boundary is not counted
};

struct ReplayStats {
  size_t samples{0};
  double path_length_m{0.0};
  double max_jump_m{0.0};
  size_t teleports{0};
  size_t wrong_flag_samples{
      0};  // projected flag != planned flag at the true station (outside the guard band)
  size_t spurious_mark_samples{
      0};  // projected MARK where the plan is TRANSIT: paint where there must be none
  size_t missed_mark_samples{0};  // projected TRANSIT where the plan is MARK: a hole in the line
  size_t boundaries{0};
  double final_s_error_m{0.0};
};

ReplayStats replay_drive_along(const PathModel& model, const ReplayConfig& cfg);

}  // namespace dyx3_spray
