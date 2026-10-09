// mission_fsm — see docs/contracts/dyx3_mission.md (the transition table is normative).
#include "dyx3_mission/mission_fsm.hpp"

#include <utility>

namespace dyx3_mission {

const char* to_string(State s) {
  switch (s) {
    case State::kIdle:
      return "IDLE";
    case State::kLoading:
      return "LOADING";
    case State::kReady:
      return "READY";
    case State::kRunning:
      return "RUNNING";
    case State::kPaused:
      return "PAUSED";
    case State::kCompleted:
      return "COMPLETED";
    case State::kAborted:
      return "ABORTED";
    case State::kError:
      return "ERROR";
  }
  return "?";
}

const char* to_string(Event e) {
  switch (e) {
    case Event::kStart:
      return "start";
    case Event::kArtifactLoaded:
      return "artifact_loaded";
    case Event::kRppAck:
      return "rpp_ack";
    case Event::kPause:
      return "pause";
    case Event::kResume:
      return "resume";
    case Event::kAbort:
      return "abort";
    case Event::kRppComplete:
      return "rpp_complete";
    case Event::kRppError:
      return "rpp_error";
    case Event::kGateLost:
      return "gate_lost";
    case Event::kEstop:
      return "estop";
    case Event::kSkipPoint:
      return "skip_point";
    case Event::kRppStale:
      return "rpp_stale";
  }
  return "?";
}

const char* to_string(Reject r) {
  switch (r) {
    case Reject::kNone:
      return "none";
    case Reject::kBusy:
      return "busy";
    case Reject::kSafetyGate:
      return "safety_gate";
    case Reject::kNotRunning:
      return "not_running";
    case Reject::kNotPaused:
      return "not_paused";
    case Reject::kNotActive:
      return "not_active";
    case Reject::kInvalidArtifact:
      return "invalid_artifact";
    case Reject::kNoActivePoint:
      return "no_active_point";
    case Reject::kIllegal:
      return "illegal_in_state";
  }
  return "?";
}

void MissionFsm::record(const Transition& t) {
  log_.push_back(t);
  while (log_.size() > kLogCapacity) {
    log_.pop_front();
  }
  if (observer_) {
    observer_(t);
  }
}

Result MissionFsm::go(State to, Event ev, std::uint8_t reason, const char* detail,
                      std::int64_t now_ns) {
  Transition t;
  t.seq = ++seq_;
  t.from = state_;
  t.to = to;
  t.event = ev;
  t.reason = reason;
  t.detail = detail;
  t.stamp_ns = now_ns;
  state_ = to;
  reason_ = reason;
  record(t);
  Result r;
  r.accepted = true;
  r.transitioned = true;
  r.state = state_;
  return r;
}

Result MissionFsm::refuse(Event ev, Reject why, bool illegal, const char* detail,
                          std::int64_t now_ns) {
  Transition t;
  t.seq = ++seq_;
  t.from = state_;
  t.to = state_;
  t.event = ev;
  t.reason = reason_;
  t.refused = true;
  t.reject = illegal ? Reject::kIllegal : why;
  t.detail = detail;
  t.stamp_ns = now_ns;
  record(t);
  Result r;
  r.accepted = false;
  r.reject = t.reject;
  r.state = state_;
  return r;
}

Result MissionFsm::ok_no_change(Event ev, const char* detail, std::int64_t now_ns) {
  Transition t;
  t.seq = ++seq_;
  t.from = state_;
  t.to = state_;
  t.event = ev;
  t.reason = reason_;
  t.detail = detail;
  t.stamp_ns = now_ns;
  record(t);
  Result r;
  r.accepted = true;
  r.state = state_;
  return r;
}

Result MissionFsm::start(bool gate_ok, std::int64_t now_ns) {
  if (active()) {
    return refuse(Event::kStart, Reject::kBusy, false, "a mission is already active", now_ns);
  }
  if (!gate_ok) {
    return refuse(Event::kStart, Reject::kSafetyGate, false, "safety gate not ok", now_ns);
  }
  ++mission_id_;
  return go(State::kLoading, Event::kStart, kReasonNone, "start accepted", now_ns);
}

Result MissionFsm::artifact_loaded(bool valid, std::int64_t now_ns) {
  if (state_ != State::kLoading) {
    return refuse(Event::kArtifactLoaded, Reject::kIllegal, true, "not loading", now_ns);
  }
  if (valid) {
    return go(State::kReady, Event::kArtifactLoaded, kReasonNone, "artifact verified", now_ns);
  }
  return go(State::kError, Event::kArtifactLoaded, kReasonPathError, "artifact invalid", now_ns);
}

Result MissionFsm::rpp_ack(bool gate_ok, std::uint8_t guard_reason, std::int64_t now_ns) {
  switch (state_) {
    case State::kReady:
      if (!gate_ok) {
        return go(State::kAborted, Event::kRppAck, map_guard_reason(guard_reason),
                  "gate lost before motion started", now_ns);
      }
      return go(State::kRunning, Event::kRppAck, kReasonNone, "rpp acknowledged artifact", now_ns);
    case State::kRunning:
    case State::kPaused:
      return ok_no_change(Event::kRppAck, "duplicate ack ignored", now_ns);
    default:
      return refuse(Event::kRppAck, Reject::kIllegal, true, "no mission awaiting ack", now_ns);
  }
}

Result MissionFsm::pause(std::int64_t now_ns) {
  if (state_ == State::kRunning) {
    return go(State::kPaused, Event::kPause, kReasonOperator, "operator pause", now_ns);
  }
  return refuse(Event::kPause, Reject::kNotRunning, false, "not running", now_ns);
}

Result MissionFsm::resume(bool gate_ok, std::int64_t now_ns) {
  if (state_ != State::kPaused) {
    return refuse(Event::kResume, Reject::kNotPaused, false, "not paused", now_ns);
  }
  if (!gate_ok) {
    return refuse(Event::kResume, Reject::kSafetyGate, false, "safety gate not ok", now_ns);
  }
  return go(State::kRunning, Event::kResume, kReasonNone, "operator resume", now_ns);
}

Result MissionFsm::abort(std::uint8_t reason, std::int64_t now_ns) {
  if (!active()) {
    return refuse(Event::kAbort, Reject::kNotActive, false, "no active mission", now_ns);
  }
  // REASON_UNSPECIFIED (0) is treated as an operator abort (AbortMission.srv).
  const std::uint8_t r = reason == kReasonSafety ? kReasonSafety : kReasonOperator;
  return go(State::kAborted, Event::kAbort, r, "abort requested", now_ns);
}

Result MissionFsm::rpp_complete(std::int64_t now_ns) {
  if (state_ == State::kRunning) {
    return go(State::kCompleted, Event::kRppComplete, kReasonNone, "rpp reports complete", now_ns);
  }
  if (state_ == State::kPaused) {
    return ok_no_change(Event::kRppComplete, "complete while paused ignored", now_ns);
  }
  return refuse(Event::kRppComplete, Reject::kIllegal, true, "not running", now_ns);
}

Result MissionFsm::rpp_error(std::int64_t now_ns) {
  if (active()) {
    return go(State::kError, Event::kRppError, kReasonInternalError, "rpp reports error", now_ns);
  }
  return refuse(Event::kRppError, Reject::kIllegal, true, "no active mission", now_ns);
}

Result MissionFsm::gate_lost(std::uint8_t guard_reason, std::int64_t now_ns) {
  const std::uint8_t r = map_guard_reason(guard_reason);
  switch (state_) {
    case State::kLoading:
    case State::kReady:
      return go(State::kAborted, Event::kGateLost, r, "gate lost before motion", now_ns);
    case State::kRunning:
      return go(State::kPaused, Event::kGateLost, r, "automatic safety pause", now_ns);
    default:
      return ok_no_change(Event::kGateLost, "no running mission", now_ns);
  }
}

Result MissionFsm::estop(std::int64_t now_ns) {
  if (active()) {
    return go(State::kAborted, Event::kEstop, kReasonSafety, "emergency stop", now_ns);
  }
  return ok_no_change(Event::kEstop, "no active mission", now_ns);
}

Result MissionFsm::rpp_stale(std::int64_t now_ns) {
  if (state_ == State::kRunning) {
    return go(State::kPaused, Event::kRppStale, kReasonSafety, "rpp status stale: automatic pause",
              now_ns);
  }
  return ok_no_change(Event::kRppStale, "no running mission", now_ns);
}

Result MissionFsm::skip_point(bool has_active_point, std::int64_t now_ns) {
  if (state_ != State::kRunning && state_ != State::kPaused) {
    return refuse(Event::kSkipPoint, Reject::kNotRunning, false, "not running", now_ns);
  }
  if (!has_active_point) {
    return refuse(Event::kSkipPoint, Reject::kNoActivePoint, false, "no active point", now_ns);
  }
  return ok_no_change(Event::kSkipPoint, "point skip accepted", now_ns);
}

}  // namespace dyx3_mission
