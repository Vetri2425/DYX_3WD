// mission_fsm — explicit mission lifecycle state machine. See docs/contracts/dyx3_mission.md.
//
// Pure C++ (no ROS). Named states, an explicit event set, every transition and every refusal logged
// with a reason. The legal transitions are exactly the table in the contract; anything else is
// refused (never silently ignored) and logged.
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>

namespace dyx3_mission {

/// Frozen ABI values (dyx3_interfaces/msg/MissionState.msg).
enum class State : std::uint8_t {
  kIdle = 0,
  kLoading = 1,
  kReady = 2,
  kRunning = 3,
  kPaused = 4,
  kCompleted = 5,
  kAborted = 6,
  kError = 7,
};

/// MissionState.REASON_* values.
enum Reason : std::uint8_t {
  kReasonNone = 0,
  kReasonOperator = 1,
  kReasonSafety = 2,
  kReasonRtk = 3,
  kReasonPathError = 4,
  kReasonInternalError = 5,
};

enum class Event : std::uint8_t {
  kStart,
  kArtifactLoaded,
  kRppAck,
  kPause,
  kResume,
  kAbort,
  kRppComplete,
  kRppError,
  kGateLost,
  kEstop,
  kSkipPoint,
  kRppStale,
  kRppAckTimeout,
};

/// Why a request was refused. Mapped to the service REASON_* codes by the node.
enum class Reject : std::uint8_t {
  kNone = 0,
  kBusy,
  kSafetyGate,
  kNotRunning,
  kNotPaused,
  kNotActive,
  kInvalidArtifact,
  kNoActivePoint,
  kIllegal,  // event not defined in this state: refused and logged
};

/// MotionSetpointStatus.REASON_RTK_GATE (guard reason codes mapped to mission reasons).
constexpr std::uint8_t kGuardReasonRtkGate = 6;

/// Guard reason -> mission reason: the RTK gate keeps its own code, every other gate is SAFETY.
constexpr std::uint8_t map_guard_reason(std::uint8_t guard_reason) {
  return guard_reason == kGuardReasonRtkGate ? kReasonRtk : kReasonSafety;
}

const char* to_string(State s);
const char* to_string(Event e);
const char* to_string(Reject r);

struct Transition {
  std::uint64_t seq = 0;
  State from = State::kIdle;
  State to = State::kIdle;
  Event event = Event::kStart;
  std::uint8_t reason = kReasonNone;
  bool refused = false;  ///< true: the event did NOT change state (from == to)
  Reject reject = Reject::kNone;
  std::string detail;
  std::int64_t stamp_ns = 0;
};

struct Result {
  bool accepted = false;
  Reject reject = Reject::kNone;
  bool transitioned = false;
  State state = State::kIdle;  ///< state after the call
};

class MissionFsm {
public:
  using Observer = std::function<void(const Transition&)>;
  static constexpr std::size_t kLogCapacity = 256;

  /// Called for every logged transition or refusal (the node writes them to /rosout).
  void set_observer(Observer o) { observer_ = std::move(o); }

  Result start(bool gate_ok, std::int64_t now_ns);
  Result artifact_loaded(bool valid, std::int64_t now_ns);
  /// RPP acknowledged this mission's artifact. `gate_ok` is re-checked: a gate lost while waiting
  /// aborts rather than driving.
  Result rpp_ack(bool gate_ok, std::uint8_t guard_reason, std::int64_t now_ns);
  Result pause(std::int64_t now_ns);
  Result resume(bool gate_ok, std::int64_t now_ns);
  Result abort(std::uint8_t reason, std::int64_t now_ns);
  Result rpp_complete(std::int64_t now_ns);
  Result rpp_error(std::int64_t now_ns);
  Result gate_lost(std::uint8_t guard_reason, std::int64_t now_ns);
  Result estop(std::int64_t now_ns);
  /// READY and RPP never acknowledged the artifact in time: ERROR (REASON_INTERNAL_ERROR).
  Result rpp_ack_timeout(std::int64_t now_ns);
  /// RPP stopped reporting for the running mission: automatic pause (REASON_SAFETY), never an
  /// automatic resume.
  Result rpp_stale(std::int64_t now_ns);
  Result skip_point(bool has_active_point, std::int64_t now_ns);

  State state() const { return state_; }
  std::uint8_t reason() const { return reason_; }
  /// Incremented on every ACCEPTED start (0 until the first mission).
  std::uint32_t mission_id() const { return mission_id_; }
  const std::deque<Transition>& log() const { return log_; }
  std::uint64_t transitions_total() const { return seq_; }

  /// True while a mission occupies the system (LOADING, READY, RUNNING, PAUSED).
  bool active() const {
    return state_ == State::kLoading || state_ == State::kReady || state_ == State::kRunning ||
           state_ == State::kPaused;
  }
  bool terminal() const {
    return state_ == State::kCompleted || state_ == State::kAborted || state_ == State::kError;
  }

private:
  Result go(State to, Event ev, std::uint8_t reason, const char* detail, std::int64_t now_ns);
  Result refuse(Event ev, Reject why, bool illegal, const char* detail, std::int64_t now_ns);
  Result ok_no_change(Event ev, const char* detail, std::int64_t now_ns);
  void record(const Transition& t);

  State state_ = State::kIdle;
  std::uint8_t reason_ = kReasonNone;
  std::uint32_t mission_id_ = 0;
  std::uint64_t seq_ = 0;
  std::deque<Transition> log_;
  Observer observer_;
};

}  // namespace dyx3_mission
