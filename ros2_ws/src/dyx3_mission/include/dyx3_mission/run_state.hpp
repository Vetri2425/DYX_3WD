// run_state — the execution identity and the run/point counters mirrored into MissionState. Pure
// data, no logic beyond resets. See docs/contracts/dyx3_mission.md.
#pragma once

#include <cstdint>
#include <string>

namespace dyx3_mission {

struct RunState {
  std::uint32_t mission_id = 0;
  std::uint32_t run_index = 0;         ///< mirrored from RppStatus (RPP owns run sequencing)
  std::uint32_t point_index = 0;       ///< active must-hit point (journal)
  std::string path_artifact_sha256;    ///< the EXECUTION artifact (what RPP loads); set from ARMING
  std::string source_artifact_sha256;  ///< the artifact the operator started
  std::string request_id;              ///< StartMission.request_id of this execution

  void clear_progress() {
    run_index = 0;
    point_index = 0;
  }
  void clear() {
    mission_id = 0;
    clear_progress();
    path_artifact_sha256.clear();
    source_artifact_sha256.clear();
    request_id.clear();
  }
};

}  // namespace dyx3_mission
