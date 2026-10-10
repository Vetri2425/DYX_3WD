// mission_fsm — explicit mission lifecycle state machine. See docs/contracts/dyx3_mission.md.
//
// Pure C++ (no ROS). Named states, an explicit event set, every transition and every refusal logged
// with a reason. The legal transitions are exactly the table in the contract; anything else is
// refused (never silently ignored) and logged. The FSM decides states only: the PX4 calls that
// arm, engage and release the vehicle are sequenced by Px4Sequencer, the frame by place_artifact.
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>

namespace dyx3_mission {

/// Frozen ABI values (dyx3_interfaces/msg/MissionState.msg). 8..10 appended in interfaces 0.15.0.
enum class State : std::uint8_t {
  kIdle = 0,
  kLoading = 1,
  kReady = 2,
  kRunning = 3,
  kPaused = 4,
  kCompleted = 5,
  kAborted = 6,
  kError = 7,
  kPlacing = 8,
  kArming = 9,
  kEngaging = 10,
};

/// MissionState.REASON_* values.
enum Reason : std::uint8_t {
  kReasonNone = 0,
  kReasonOperator = 1,
  kReasonSafety = 2,
  kReasonRtk = 3,
  kReasonPathError = 4,
  kReasonInternalError = 5,
  kReasonEkfReset = 6,
  kReasonEkfReferenceInvalid = 7,
  kReasonPlacementOutOfBounds = 8,
  kReasonNoPlacementFrame = 9,
  kReasonArmRefused = 10,
  kReasonArmTimeout = 11,
  kReasonOffboardRefused = 12,
  kReasonOffboardTimeout = 13,
  kReasonRppAckTimeout = 14,
  kReasonEstop = 15,
  kReasonRppError = 16,
  kReasonRppStale = 17,
  kReasonRppPivotTimeout = 18,  ///< interfaces 0.17.0
};

enum class Event : std::uint8_t {
  kStart,
  kArtifactLoaded,
  kPlaced,
  kArmed,
  kEngaged,
  kRppAck,
  kPause,
  kResume,
  kAbort,
  kRppComplete,
  kRppError,
  kGateLost,
  kEstop,
  kEkfReset,
  kSkipPoint,
  kRppStale,
  kRppAckTimeout,
  kReengage,         ///< PAUSED -> ARMING (resume of a vehicle that lost arm / OFFBOARD)
  kReengaged,        ///< ENGAGING (re-engage, OFFBOARD confirmed) -> RUNNING with the full gate
  kReengageTimeout,  ///< the full gate never passed after the re-engage's OFFBOARD confirmation
  kRppPivotTimeout,  ///< RPP's pivot watchdog expired while RUNNING
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
  std::int64_t stamp_ns = 0;  ///< the `now_ns` the caller passed (the node's steady clock)
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

  /// Called for every logged transition or refusal (the node logs it and publishes MissionState
  /// on every transition).
  void set_observer(Observer o) { observer_ = std::move(o); }

  /// Admission: needs the guard's PRE-ARM gate (not armed / OFFBOARD: the vehicle is not yet).
  Result start(bool pre_arm_ok, std::int64_t now_ns);
  /// LOADING: the artifact was read and verified by its hash (valid) or not.
  Result artifact_loaded(bool valid, const std::string& detail, std::int64_t now_ns);
  /// PLACING: the execution artifact exists (ok) or placement failed with `reason`.
  Result placed(bool ok, std::uint8_t reason, const std::string& detail, std::int64_t now_ns);
  /// ARMING: px4_link confirmed the arm (ok) or refused / timed out (`reason`).
  Result armed(bool ok, std::uint8_t reason, const std::string& detail, std::int64_t now_ns);
  /// ENGAGING: px4_link confirmed OFFBOARD (ok) or refused / timed out (`reason`). A start goes to
  /// READY; a re-engage stays in ENGAGING until reengage_gate() (the guard's verdict may lag the
  /// confirmation, and nothing reaches RUNNING without the full gate).
  Result engaged(bool ok, std::uint8_t reason, const std::string& detail, std::int64_t now_ns);
  /// RPP holds this execution's path. READY -> RUNNING only with the FULL gate ok; otherwise the
  /// acknowledgement is held (the guard's verdict may lag the OFFBOARD confirmation) until the
  /// gate passes or rpp_ack_timeout fires.
  Result rpp_ack(bool gate_ok, std::int64_t now_ns);
  Result pause(std::int64_t now_ns);
  /// PAUSED -> RUNNING with every resume condition met. During a re-engage: accepted, no change
  /// (a retry of the resume that started it).
  Result resume(bool gate_ok, std::int64_t now_ns);
  /// Resume of a PAUSED execution whose vehicle is no longer armed / in OFFBOARD (the node checks
  /// that only the arming gate fails and the pre-arm gate is ok): PAUSED -> ARMING -> ENGAGING ->
  /// RUNNING, with the same steps, timeouts and failure reasons as a start. RPP keeps the execution
  /// loaded and paused, so there is no READY handshake; the node publishes PAUSED until RUNNING.
  Result reengage(std::int64_t now_ns);
  /// Re-engage, OFFBOARD confirmed: -> RUNNING when the full gate is ok, else held (no change).
  Result reengage_gate(bool gate_ok, std::int64_t now_ns);
  /// Re-engage, OFFBOARD confirmed, and the full gate never passed in time: ERROR with the guard
  /// reason (SAFETY / RTK), as rpp_ack_timeout does for READY.
  Result reengage_timeout(std::uint8_t guard_reason, std::int64_t now_ns);
  Result abort(std::uint8_t reason, std::int64_t now_ns);
  Result rpp_complete(std::int64_t now_ns);
  Result rpp_error(std::int64_t now_ns);
  /// A guard gate failed: the pre-arm gate before RUNNING (-> ERROR), the full gate while RUNNING
  /// (-> PAUSED). Never an automatic resume.
  Result gate_lost(std::uint8_t guard_reason, std::int64_t now_ns);
  /// E-stop asserted: every active state -> ABORTED(ESTOP) (the node disarms).
  Result estop(std::int64_t now_ns);
  /// The EKF xy reset counter or global reference changed after placement.
  Result ekf_reset(const std::string& detail, std::int64_t now_ns);
  /// READY and RPP never acknowledged the artifact in time: ERROR(RPP_ACK_TIMEOUT), or the guard
  /// reason when the full gate was the one not passing.
  Result rpp_ack_timeout(bool gate_ok, std::uint8_t guard_reason, std::int64_t now_ns);
  /// RPP stopped reporting for the running mission: automatic pause (REASON_RPP_STALE), never an
  /// automatic resume.
  Result rpp_stale(std::int64_t now_ns);
  /// RPP reports its pivot watchdog expired (heading not reached) while RUNNING: automatic pause
  /// (REASON_RPP_PIVOT_TIMEOUT), never an automatic resume.
  Result rpp_pivot_timeout(std::int64_t now_ns);
  Result skip_point(bool has_active_point, std::int64_t now_ns);

  State state() const { return state_; }
  std::uint8_t reason() const { return reason_; }
  /// The guard gate behind REASON_SAFETY / REASON_RTK (MotionSetpointStatus.REASON_*), else 0.
  std::uint8_t gate_reason() const { return gate_reason_; }
  /// Detail of the transition into the current state.
  const std::string& detail() const { return detail_; }
  /// Time of the transition into the current state on the clock the caller passes (the node's
  /// steady clock; NOT ROS time: the node keeps the ROS time for MissionState.state_entered).
  /// 0 before the first one.
  std::int64_t state_entered_ns() const { return entered_ns_; }
  /// Incremented on every ACCEPTED start (0 until the first mission): the execution id.
  std::uint32_t mission_id() const { return mission_id_; }
  const std::deque<Transition>& log() const { return log_; }
  /// ARMING / ENGAGING of a re-engage from PAUSED (not of a start).
  bool reengaging() const { return reengaging_; }
  /// A re-engage whose OFFBOARD is confirmed, waiting for the full gate (reengage_gate()).
  bool awaiting_reengage_gate() const { return reengaging_ && reengage_confirmed_; }
  std::uint64_t transitions_total() const { return seq_; }
  /// Number of state changes (refusals and no-change events excluded). A self-transition (PAUSED
  /// -> PAUSED on an EKF reset) counts.
  std::uint64_t changes() const { return changes_; }

  /// True while an execution occupies the system (every non-IDLE, non-terminal state).
  bool active() const { return state_ != State::kIdle && !terminal(); }
  /// The steps before motion is allowed: LOADING, PLACING, ARMING, ENGAGING, READY.
  bool before_running() const {
    return state_ == State::kLoading || state_ == State::kPlacing || state_ == State::kArming ||
           state_ == State::kEngaging || state_ == State::kReady;
  }
  bool terminal() const {
    return state_ == State::kCompleted || state_ == State::kAborted || state_ == State::kError;
  }

private:
  Result go(State to, Event ev, std::uint8_t reason, const std::string& detail, std::int64_t now_ns,
            std::uint8_t gate_reason = 0);
  Result refuse(Event ev, Reject why, bool illegal, const char* detail, std::int64_t now_ns);
  Result ok_no_change(Event ev, const char* detail, std::int64_t now_ns);
  Result step(State expected, State next, Event ev, bool ok, std::uint8_t reason,
              const std::string& detail, std::int64_t now_ns);
  void record(const Transition& t);

  State state_ = State::kIdle;
  std::uint8_t reason_ = kReasonNone;
  std::uint8_t gate_reason_ = 0;
  std::string detail_;
  std::int64_t entered_ns_ = 0;
  std::uint32_t mission_id_ = 0;
  std::uint64_t seq_ = 0;
  std::uint64_t changes_ = 0;
  bool reengaging_ = false;
  bool reengage_confirmed_ = false;
  std::deque<Transition> log_;
  Observer observer_;
};

}  // namespace dyx3_mission
