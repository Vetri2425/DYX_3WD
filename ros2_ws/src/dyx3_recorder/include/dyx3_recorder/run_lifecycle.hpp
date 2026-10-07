// run_lifecycle — when a run starts and stops. Contract: docs/contracts/dyx3_recorder.md section 2.
// Pure C++.
#pragma once

#include <cstdint>
#include <string>

namespace dyx3_recorder {

// Values of dyx3_interfaces/MissionState.state.
constexpr uint8_t kMissionIdle = 0, kMissionLoading = 1, kMissionReady = 2, kMissionRunning = 3,
                  kMissionPaused = 4, kMissionCompleted = 5, kMissionAborted = 6, kMissionError = 7;

const char* mission_state_name(uint8_t s);

struct LifecycleAction {
  bool stop{false};
  std::string final_state;  // meaningful when stop
  bool start{false};
};

class RunLifecycle {
public:
  // Feed every MissionState. Returns what the recorder must do (stop first, then start).
  LifecycleAction on_mission(uint8_t state, uint32_t mission_id, uint32_t run_index);
  bool recording() const { return recording_; }
  uint32_t mission_id() const { return mission_id_; }
  uint32_t run_index() const { return run_index_; }

private:
  bool recording_{false};
  uint32_t mission_id_{0}, run_index_{0};
};

}  // namespace dyx3_recorder
