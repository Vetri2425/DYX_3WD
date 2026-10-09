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
    const bool same = mission_id == mission_id_ && run_index == run_index_;
    const char* superseded = running_seen_ ? "SUPERSEDED" : "NOT_STARTED";
    if (terminal) {
      a.stop = true;
      if (running_seen_) {
        a.final_state = mission_state_name(state);
      } else {
        a.final_state = "NOT_STARTED";
        a.stop_note = std::string("mission went from READY to ") + mission_state_name(state) +
                      " without RUNNING";
      }
      reset();
      return a;
    }
    if ((state == kMissionRunning || state == kMissionReady) && !same) {
      // A different mission or run while one is open: close it, open the new one.
      a.stop = true;
      a.final_state = superseded;
      a.start = true;
      a.start_running = state == kMissionRunning;
      mission_id_ = mission_id;
      run_index_ = run_index;
      running_seen_ = a.start_running;
      return a;
    }
    if (state == kMissionRunning && !running_seen_) {
      a.running = true;
      running_seen_ = true;
    }
    return a;  // PAUSED / LOADING / READY / RUNNING of the same run: keep recording
  }
  if (state == kMissionReady || state == kMissionRunning) {
    a.start = true;
    a.start_running = state == kMissionRunning;  // e.g. the recorder restarted mid-run
    recording_ = true;
    running_seen_ = a.start_running;
    mission_id_ = mission_id;
    run_index_ = run_index;
  }
  return a;
}

}  // namespace dyx3_recorder
