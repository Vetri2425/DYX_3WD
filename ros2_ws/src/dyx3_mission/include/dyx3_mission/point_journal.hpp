// point_journal — per must-hit-vertex results. See docs/contracts/dyx3_mission.md section 6.
//
// Pure C++. Fed rover positions (local NED), it reports, for each must-hit vertex in path order,
// the closest approach: that is the recorded evidence of how accurately the rover reached the
// operator's surveyed point. Dwell/handshake logic is NOT here (that is RPP/spray).
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "dyx3_mission/path_artifact.hpp"

namespace dyx3_mission {

/// PointResult.RESULT_* values.
enum class PointOutcome : std::uint8_t { kNone = 0, kCompleted = 1, kSkipped = 2, kFailed = 3 };

struct PointEvent {
  std::uint32_t point_index = 0;  ///< rank among the must-hit vertices, in path order
  PointOutcome outcome = PointOutcome::kNone;
  double north_m = 0.0;  ///< rover position at closest approach (COMPLETED) else the planned vertex
  double east_m = 0.0;
  double error_m = 0.0;  ///< closest-approach distance (COMPLETED); 0 otherwise. Never negative.
};

class PointJournal {
public:
  /// `capture_radius_m` must be > 0.
  PointJournal(const std::vector<ArtifactPoint>& path, double capture_radius_m);

  std::size_t point_count() const { return vertices_.size(); }
  /// Index of the first unresolved must-hit point, or nullopt when all are resolved / none exist.
  std::optional<std::uint32_t> active_point() const;

  /// Feed one rover position. Returns the points resolved by this update (usually none).
  std::vector<PointEvent> update(double north_m, double east_m);
  /// Operator skip of the active point (nullopt when there is none).
  std::optional<PointEvent> skip();
  /// End of the path: resolves everything still open (a capture in progress completes; the rest
  /// FAIL).
  std::vector<PointEvent> finish();
  /// Resume of an earlier execution of the same path (docs/contracts/dyx3_mission.md section 9a):
  /// the first `resolved` points were resolved then (their PointResults were issued) and are never
  /// reported again; the journal continues at point `resolved`. Only before anything is resolved;
  /// false and no change when `resolved` > point_count() or the journal already moved.
  bool restore_resolved(std::size_t resolved);

private:
  struct Vertex {
    double n;
    double e;
  };
  PointEvent complete_current();
  PointEvent fail_one(std::size_t i) const;

  std::vector<Vertex> vertices_;
  double radius_;
  std::size_t next_ = 0;
  bool capturing_ = false;
  double min_d_ = 0.0;
  double min_n_ = 0.0;
  double min_e_ = 0.0;
};

}  // namespace dyx3_mission
