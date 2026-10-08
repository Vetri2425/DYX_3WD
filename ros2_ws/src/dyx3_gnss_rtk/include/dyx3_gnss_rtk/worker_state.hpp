#pragma once

#include <cstdint>
#include <deque>
#include <string>

#include "dyx3_gnss_rtk/rtk_config.hpp"

namespace dyx3_gnss_rtk {

enum class WorkerState {
  Stopped,
  Starting,
  WaitSource,
  WaitRtcm,
  WaitTransport,
  Injecting,
  Degraded,
  Reconfiguring,
  Error,
};

const char* state_name(WorkerState state);

class WorkerStateMachine {
public:
  void transition(WorkerState next, const std::string& reason_code, const std::string& reason,
                  const std::string& timestamp_utc, uint64_t revision);
  WorkerState state() const { return state_; }
  const std::string& reason_code() const { return reason_code_; }
  Json events() const;

private:
  WorkerState state_{WorkerState::Stopped};
  std::string reason_code_{"SERVICE_STOPPED"};
  std::deque<Json> events_;
};

}  // namespace dyx3_gnss_rtk
