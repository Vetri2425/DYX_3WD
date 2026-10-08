#include "dyx3_gnss_rtk/worker_state.hpp"

namespace dyx3_gnss_rtk {

const char* state_name(WorkerState s) {
  switch (s) {
    case WorkerState::Stopped:
      return "STOPPED";
    case WorkerState::Starting:
      return "STARTING";
    case WorkerState::WaitSource:
      return "WAIT_SOURCE";
    case WorkerState::WaitRtcm:
      return "WAIT_RTCM";
    case WorkerState::WaitTransport:
      return "WAIT_TRANSPORT";
    case WorkerState::Injecting:
      return "INJECTING";
    case WorkerState::Degraded:
      return "DEGRADED";
    case WorkerState::Reconfiguring:
      return "RECONFIGURING";
    case WorkerState::Error:
      return "ERROR";
  }
  return "ERROR";
}

void WorkerStateMachine::transition(WorkerState next, const std::string& code,
                                    const std::string& reason, const std::string& timestamp_utc,
                                    uint64_t revision) {
  if (next == state_ && code == reason_code_) return;
  events_.push_back({{"timestamp_utc", timestamp_utc},
                     {"previous", state_name(state_)},
                     {"state", state_name(next)},
                     {"reason_code", code},
                     {"reason", reason},
                     {"config_revision", revision}});
  while (events_.size() > 100) events_.pop_front();
  state_ = next;
  reason_code_ = code;
}

Json WorkerStateMachine::events() const {
  Json out = Json::array();
  for (const auto& event : events_) out.push_back(event);
  return out;
}

}  // namespace dyx3_gnss_rtk
