// frame_placement — places a source path artifact in the EKF local NED frame. SAFETY-CRITICAL.
// See docs/contracts/dyx3_mission.md section 6.
//
// Pure C++ (no ROS). The rule (owner decision, mission contract v2):
//  * meta "frame" == "local_ned" with an "anchor" {lat, lon}: the anchor IS the trajectory's local
//    origin. Execution point = (anchor projected into the EKF frame with the EKF reference
//    VehicleState.reference_latitude_deg / reference_longitude_deg) + (n, e). The projection is
//    PX4's own MapProjection::project (azimuthal equidistant on the 6371000 m sphere), the exact
//    function EKF2 uses to produce vehicle_local_position from its global estimate, so the anchor
//    lands where the EKF will report the rover when it stands on the anchor.
//  * meta "frame" == "ekf_local_ned" with no anchor (null or absent): the points are already EKF
//    local NED; no transform, the execution artifact IS the source artifact.
//  * anything else (no "frame" key: old and planner-made artifacts; "local_ned" without an anchor;
//    "ekf_local_ned" with one; an unknown frame) is refused: EKF-local is never assumed.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "dyx3_mission/path_artifact.hpp"

namespace dyx3_mission {

struct GeoPoint {
  double lat_deg = 0.0;
  double lon_deg = 0.0;
};

struct NePoint {
  double north_m = 0.0;
  double east_m = 0.0;
};

/// PX4 MapProjection::project (src/lib/geo/geo.cpp at the pinned firmware), in double precision:
/// `p` in the local frame whose origin is `ref`.
NePoint project_to_ekf(const GeoPoint& ref, const GeoPoint& p);

enum class FrameKind : std::uint8_t { kAnchored, kEkfLocal };

struct FrameSpec {
  bool ok = false;
  FrameKind kind = FrameKind::kEkfLocal;
  GeoPoint anchor;    ///< meaningful for kAnchored
  std::string error;  ///< why !ok
};

/// Reads "frame" and "anchor" from the artifact's canonical meta JSON and applies the rule above.
FrameSpec read_frame_spec(const std::string& meta_json);

/// The EKF reference at placement (from VehicleState).
struct EkfReference {
  bool global_reference_valid = false;
  double lat_deg = 0.0;
  double lon_deg = 0.0;
};

enum class PlacementError : std::uint8_t {
  kNone = 0,
  kNoPlacementFrame,  ///< MissionState.REASON_NO_PLACEMENT_FRAME
  kReferenceInvalid,  ///< MissionState.REASON_EKF_REFERENCE_INVALID
  kOutOfBounds,       ///< MissionState.REASON_PLACEMENT_OUT_OF_BOUNDS
  kNotSerializable,   ///< MissionState.REASON_INTERNAL_ERROR
};

struct Placement {
  bool ok = false;
  PlacementError error = PlacementError::kNone;
  std::string detail;
  bool transformed = false;  ///< false: "ekf_local_ned" passthrough (execution == source)
  NePoint anchor_ekf;        ///< the anchor in the EKF frame (0, 0 for a passthrough)
  /// The execution artifact: sha256, points in the EKF frame, meta. For a transform its bytes are
  /// in `bytes` (to be stored as <sha256>.dyx3path); for a passthrough `bytes` is empty.
  PathArtifact execution;
  std::string bytes;
};

/// Places `source`. Every placed point and the anchor must lie within `max_distance_m` of the EKF
/// origin (the projection's accuracy argument holds there; contract section 6). Deterministic: the
/// same source and reference give the same execution sha256.
Placement place_artifact(const PathArtifact& source, const EkfReference& ref,
                         double max_distance_m);

/// Writes a transformed placement's bytes to `<dir>/<sha256>.dyx3path` (temporary file + rename, so
/// a reader never sees a partial file). A passthrough writes nothing. False with `error` on
/// failure.
bool store_execution_artifact(const std::string& dir, const Placement& p, std::string* error);

}  // namespace dyx3_mission
