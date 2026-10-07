#include "dyx3_recorder/run_lifecycle.hpp"

namespace dyx3_recorder {

const char* mission_state_name(uint8_t s) {
  switch (s) {
    case kMissionIdle:
      return "IDLE";
    case kMissionLoading:
      return "LOADING";
    case kMissionReady:
      return "READY";
    case kMissionRunning:
      return "RUNNING";
    case kMissionPaused:
      return "PAUSED";
    case kMissionCompleted:
      return "COMPLETED";
    case kMissionAborted:
      return "ABORTED";
    case kMissionError:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

LifecycleAction RunLifecycle::on_mission(uint8_t state, uint32_t mission_id, uint32_t run_index) {
  LifecycleAction a;
  if (recording_) {
    const bool terminal = state == kMissionCompleted || state == kMissionAborted ||
                          state == kMissionError || state == kMissionIdle;
    if (terminal) {
      a.stop = true;
      a.final_state = mission_state_name(state);
      recording_ = false;
      return a;
    }
    if (state == kMissionRunning && (mission_id != mission_id_ || run_index != run_index_)) {
      a.stop = true;
      a.final_state = "SUPERSEDED";
      a.start = true;
      mission_id_ = mission_id;
      run_index_ = run_index;
      return a;
    }
    return a;  // PAUSED / READY / LOADING / RUNNING of the same run: keep recording
  }
  if (state == kMissionRunning) {
    a.start = true;
    recording_ = true;
    mission_id_ = mission_id;
    run_index_ = run_index;
  }
  return a;
}

}  // namespace dyx3_recorder
