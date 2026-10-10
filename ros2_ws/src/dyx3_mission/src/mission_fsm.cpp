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
    case State::kPlacing:
      return "PLACING";
    case State::kArming:
      return "ARMING";
    case State::kEngaging:
      return "ENGAGING";
  }
  return "?";
}

const char* to_string(Event e) {
  switch (e) {
    case Event::kStart:
      return "start";
    case Event::kArtifactLoaded:
      return "artifact_loaded";
    case Event::kPlaced:
      return "placed";
    case Event::kArmed:
      return "armed";
    case Event::kEngaged:
      return "engaged";
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
    case Event::kEkfReset:
      return "ekf_reset";
    case Event::kSkipPoint:
      return "skip_point";
    case Event::kRppStale:
      return "rpp_stale";
    case Event::kRppAckTimeout:
      return "rpp_ack_timeout";
    case Event::kReengage:
      return "reengage";
    case Event::kReengaged:
      return "reengaged";
    case Event::kReengageTimeout:
      return "reengage_timeout";
    case Event::kRppPivotTimeout:
      return "rpp_pivot_timeout";
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

Result MissionFsm::go(State to, Event ev, std::uint8_t reason, const std::string& detail,
                      std::int64_t now_ns, std::uint8_t gate_reason) {
  Transition t;
  t.seq = ++seq_;
  t.from = state_;
  t.to = to;
  t.event = ev;
  t.reason = reason;
  t.detail = detail;
  t.stamp_ns = now_ns;
  state_ = to;
  // The re-engage marks live exactly as long as its ARMING / ENGAGING (reengage() sets them first).
  if (to != State::kArming && to != State::kEngaging) {
    reengaging_ = false;
    reengage_confirmed_ = false;
  }
  reason_ = reason;
  gate_reason_ = gate_reason;
  detail_ = detail;
  entered_ns_ = now_ns;
  ++changes_;
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

// One pre-RUNNING step: `expected` -> `next` on success, -> ERROR(reason) on failure.
Result MissionFsm::step(State expected, State next, Event ev, bool ok, std::uint8_t reason,
                        const std::string& detail, std::int64_t now_ns) {
  if (state_ != expected) {
    return refuse(ev, Reject::kIllegal, true, "not in the step this event completes", now_ns);
  }
  if (ok) return go(next, ev, kReasonNone, detail, now_ns);
  return go(State::kError, ev, reason, detail, now_ns);
}

Result MissionFsm::start(bool pre_arm_ok, std::int64_t now_ns) {
  if (active()) {
    return refuse(Event::kStart, Reject::kBusy, false, "a mission is already active", now_ns);
  }
  if (!pre_arm_ok) {
    return refuse(Event::kStart, Reject::kSafetyGate, false, "pre-arm gate not ok", now_ns);
  }
  ++mission_id_;
  return go(State::kLoading, Event::kStart, kReasonNone, "start accepted", now_ns);
}

Result MissionFsm::artifact_loaded(bool valid, const std::string& detail, std::int64_t now_ns) {
  return step(State::kLoading, State::kPlacing, Event::kArtifactLoaded, valid, kReasonPathError,
              detail, now_ns);
}

Result MissionFsm::placed(bool ok, std::uint8_t reason, const std::string& detail,
                          std::int64_t now_ns) {
  return step(State::kPlacing, State::kArming, Event::kPlaced, ok, reason, detail, now_ns);
}

Result MissionFsm::armed(bool ok, std::uint8_t reason, const std::string& detail,
                         std::int64_t now_ns) {
  return step(State::kArming, State::kEngaging, Event::kArmed, ok, reason, detail, now_ns);
}

Result MissionFsm::engaged(bool ok, std::uint8_t reason, const std::string& detail,
                           std::int64_t now_ns) {
  if (state_ == State::kEngaging && reengaging_ && ok) {
    if (reengage_confirmed_) return ok_no_change(Event::kEngaged, "duplicate", now_ns);
    reengage_confirmed_ = true;  // RUNNING only with the full gate: reengage_gate()
    return ok_no_change(Event::kEngaged, "re-engage: OFFBOARD confirmed, waiting for the full gate",
                        now_ns);
  }
  return step(State::kEngaging, State::kReady, Event::kEngaged, ok, reason, detail, now_ns);
}

Result MissionFsm::rpp_ack(bool gate_ok, std::int64_t now_ns) {
  switch (state_) {
    case State::kReady:
      if (!gate_ok) {
        return ok_no_change(Event::kRppAck, "ack held: full safety gate not ok yet", now_ns);
      }
      return go(State::kRunning, Event::kRppAck, kReasonNone, "rpp acknowledged the execution",
                now_ns);
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
  if (reengaging_) {
    return ok_no_change(Event::kResume, "re-engage already in progress", now_ns);
  }
  if (state_ != State::kPaused) {
    return refuse(Event::kResume, Reject::kNotPaused, false, "not paused", now_ns);
  }
  if (!gate_ok) {
    return refuse(Event::kResume, Reject::kSafetyGate, false, "resume conditions not met", now_ns);
  }
  return go(State::kRunning, Event::kResume, kReasonNone, "operator resume", now_ns);
}

Result MissionFsm::reengage(std::int64_t now_ns) {
  if (state_ != State::kPaused) {
    if (reengaging_) {
      return ok_no_change(Event::kReengage, "re-engage already in progress", now_ns);
    }
    return refuse(Event::kReengage, Reject::kNotPaused, false, "not paused", now_ns);
  }
  reengaging_ = true;  // before the transition: the observer already sees a re-engage
  reengage_confirmed_ = false;
  return go(State::kArming, Event::kReengage, kReasonNone,
            "operator resume: re-engaging (arm, then OFFBOARD)", now_ns);
}

Result MissionFsm::reengage_gate(bool gate_ok, std::int64_t now_ns) {
  if (!awaiting_reengage_gate() || state_ != State::kEngaging) {
    return refuse(Event::kReengaged, Reject::kIllegal, true, "no re-engage awaiting the gate",
                  now_ns);
  }
  if (!gate_ok) {
    return ok_no_change(Event::kReengaged, "re-engage held: full safety gate not ok yet", now_ns);
  }
  return go(State::kRunning, Event::kReengaged, kReasonNone, "re-engaged: armed and OFFBOARD",
            now_ns);
}

Result MissionFsm::reengage_timeout(std::uint8_t guard_reason, std::int64_t now_ns) {
  if (!awaiting_reengage_gate() || state_ != State::kEngaging) {
    return ok_no_change(Event::kReengageTimeout, "no re-engage awaiting the gate", now_ns);
  }
  return go(State::kError, Event::kReengageTimeout, map_guard_reason(guard_reason),
            "full safety gate never passed after the re-engage", now_ns, guard_reason);
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
  if (state_ == State::kPaused || reengaging_) {
    return ok_no_change(Event::kRppComplete, "complete while paused ignored", now_ns);
  }
  return refuse(Event::kRppComplete, Reject::kIllegal, true, "not running", now_ns);
}

Result MissionFsm::rpp_error(std::int64_t now_ns) {
  // A re-engage's RPP still holds the execution (loaded and paused).
  if (state_ == State::kReady || state_ == State::kRunning || state_ == State::kPaused ||
      reengaging_) {
    return go(State::kError, Event::kRppError, kReasonRppError, "rpp reports error", now_ns);
  }
  if (active()) {
    return ok_no_change(Event::kRppError, "rpp holds no path of this execution yet", now_ns);
  }
  return refuse(Event::kRppError, Reject::kIllegal, true, "no active mission", now_ns);
}

Result MissionFsm::gate_lost(std::uint8_t guard_reason, std::int64_t now_ns) {
  const std::uint8_t r = map_guard_reason(guard_reason);
  if (before_running()) {
    return go(State::kError, Event::kGateLost, r, "safety gate lost before motion", now_ns,
              guard_reason);
  }
  if (state_ == State::kRunning) {
    return go(State::kPaused, Event::kGateLost, r, "automatic safety pause", now_ns, guard_reason);
  }
  return ok_no_change(Event::kGateLost, "no running mission", now_ns);
}

Result MissionFsm::estop(std::int64_t now_ns) {
  if (active()) {
    return go(State::kAborted, Event::kEstop, kReasonEstop, "emergency stop", now_ns);
  }
  return ok_no_change(Event::kEstop, "no active mission", now_ns);
}

Result MissionFsm::ekf_reset(const std::string& detail, std::int64_t now_ns) {
  switch (state_) {
    case State::kArming:
    case State::kEngaging:
      return go(State::kError, Event::kEkfReset, kReasonEkfReset, detail, now_ns);
    case State::kReady:
    case State::kRunning:
    case State::kPaused:  // a self-transition: the pause reason becomes EKF_RESET
      return go(State::kPaused, Event::kEkfReset, kReasonEkfReset, detail, now_ns);
    default:
      return ok_no_change(Event::kEkfReset, "no placed execution", now_ns);
  }
}

Result MissionFsm::rpp_stale(std::int64_t now_ns) {
  if (state_ == State::kRunning) {
    return go(State::kPaused, Event::kRppStale, kReasonRppStale,
              "rpp status stale: automatic pause", now_ns);
  }
  return ok_no_change(Event::kRppStale, "no running mission", now_ns);
}

Result MissionFsm::rpp_pivot_timeout(std::int64_t now_ns) {
  if (state_ == State::kRunning) {
    return go(State::kPaused, Event::kRppPivotTimeout, kReasonRppPivotTimeout,
              "rpp pivot watchdog expired (heading not reached)", now_ns);
  }
  return ok_no_change(Event::kRppPivotTimeout, "no running mission", now_ns);
}

Result MissionFsm::rpp_ack_timeout(bool gate_ok, std::uint8_t guard_reason, std::int64_t now_ns) {
  if (state_ != State::kReady) {
    return ok_no_change(Event::kRppAckTimeout, "not waiting for an ack", now_ns);
  }
  if (!gate_ok) {
    return go(State::kError, Event::kRppAckTimeout, map_guard_reason(guard_reason),
              "full safety gate never passed while READY", now_ns, guard_reason);
  }
  return go(State::kError, Event::kRppAckTimeout, kReasonRppAckTimeout,
            "rpp never acknowledged the execution", now_ns);
}

Result MissionFsm::skip_point(bool has_active_point, std::int64_t now_ns) {
  // A re-engage is still a paused execution to the operator (MissionState says PAUSED).
  if (state_ != State::kRunning && state_ != State::kPaused && !reengaging_) {
    return refuse(Event::kSkipPoint, Reject::kNotRunning, false, "not running", now_ns);
  }
  if (!has_active_point) {
    return refuse(Event::kSkipPoint, Reject::kNoActivePoint, false, "no active point", now_ns);
  }
  return ok_no_change(Event::kSkipPoint, "point skip accepted", now_ns);
}

}  // namespace dyx3_mission
