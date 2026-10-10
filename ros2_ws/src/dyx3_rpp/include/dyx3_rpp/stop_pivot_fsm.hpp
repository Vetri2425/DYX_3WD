// stop_pivot_fsm — corner / run-boundary / completion stop and pivot as an EXPLICIT state machine.
// Contract: docs/contracts/rpp_stop_pivot_fsm.md. Pure C++ on plain structs, no ROS.
//
// The prototype scattered this logic over ~10 booleans and timestamps; every closed bug listed in
// the contract landed in that flag soup. Here every state is named, every transition is recorded
// with a reason, and every state's output ends at zero on a failure. Invariants carried from the
// field:
//   I1  a brake is exactly +/- along the body axis, capped, zero when velocity is stale / braking
//   is off /
//       the rover is already below the stop threshold / motion is lateral-dominant;
//   I3  "stopped" = measured speed AND yaw rate below their thresholds continuously for the dwell;
//   a FRESH
//       velocity above the threshold NEVER times out into a pivot, only a STALE one does (after 2.0
//       s of continuous staleness inside the hold; XR-RPP-011).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace dyx3_rpp {

// ---------------------------------------------------------------------------------------------------
// Times are int64 nanoseconds on the node clock (replay-safe: honours use_sim_time), exactly like
// the prototype's (now - t0).nanoseconds * 1e-9.
//
// Telemetry for the stop logic: from the measured velocity, never the commanded one.
struct StopTelemetry {
  bool vel_fresh{false};  // /velocity < 0.3 s old (the node decides)
  double speed{0.0};      // |v| ground speed, m/s
  double v_forward{0.0};  // v . nose (signed), m/s
  double yaw_rate{0.0};   // rad/s, clockwise positive
};

struct StopPivotParams {
  double stop_speed_threshold{0.02};          // segment_stop_speed_threshold
  double stop_yaw_rate_threshold{0.05};       // segment_stop_yaw_rate_threshold
  double stop_dwell_s{0.30};                  // segment_stop_dwell_s
  double brake_velocity_cap{0.08};            // segment_brake_velocity_cap_m_s
  double align_speed_threshold{0.02};         // segment_align_speed_threshold
  double align_settle_s{0.20};                // segment_align_settle_s
  double heading_tolerance_rad{0.0349066};    // segment_heading_tolerance_deg (2 deg)
  double timeout_heading_tol_rad{0.0523599};  // segment_timeout_heading_tolerance_deg (3 deg)
  double release_max_rad{0.0523599};          // segment_pivot_release_max_deg (3 deg)
  double spinup_margin_s{1.0};                // segment_pivot_spinup_margin_s
  double nominal_pivot_rate{0.40};            // segment_nominal_pivot_rate_rad_s
  double turn_timeout_s{5.0};                 // segment_turn_timeout_s (legacy floor)
  double pivot_timeout_max_s{9.0};            // segment_pivot_timeout_max_s (clamp, > 0)
  double corner_threshold_deg{45.0};          // segment_corner_threshold_deg
  double corner_speed{0.12};     // max(0.05, segment_min_corner_speed) memory during the pivot
  double stale_vel_hold_s{2.0};  // _CORNER_STOP_MAX_HOLD_S (a constant in the prototype)
};

// Signed longitudinal brake command along the nose (+ forward, - reverse). I1.
double brake_speed(const StopTelemetry& tel, const StopPivotParams& p);

// ---------------------------------------------------------------------------------------------------
// Stop confirmation (I3). The first call latches the entry time; `reset()` clears it.
class StopConfirm {
public:
  bool satisfied(int64_t now_ns, const StopTelemetry& tel, const StopPivotParams& p);
  void reset() {
    entered_ = false;
    settle_ = false;
    stale_ = false;
  }
  bool entered() const { return entered_; }

private:
  bool entered_{false};
  int64_t entered_ns_{0};
  bool settle_{false};
  int64_t settle_ns_{0};
  bool stale_{false};  // the velocity has been stale since stale_ns_ (XR-RPP-011)
  int64_t stale_ns_{0};
};

// Angle-aware pivot watchdog. The turn angle is captured on the first call.
class PivotWatchdog {
public:
  double budget_s(const StopPivotParams& p) const;
  // True once the pivot has run longer than the budget. The first call returns false and starts the
  // clock.
  bool timed_out(int64_t now_ns, double turn_angle_rad, const StopPivotParams& p);
  bool warned() const { return warned_; }
  void reset() {
    started_ = false;
    warned_ = false;
    angle_ = 0.0;
  }

private:
  bool started_{false};
  int64_t started_ns_{0};
  bool warned_{false};
  double angle_{0.0};
};

// ---------------------------------------------------------------------------------------------------
enum class FsmState : uint8_t {
  Tracking = 0,
  Brake,          // CORNER_STOP: stop not yet confirmed
  Pivot,          // CORNER_ALIGN: in-place turn toward the exit heading
  ReleaseSettle,  // heading inside the release band: brake and settle before advancing
  Advance,        // one-tick event: the caller advances the segment / run
  Done
};
const char* to_string(FsmState s);

struct Transition {
  uint64_t seq;
  FsmState from;
  FsmState to;
  const char* reason;  // a string literal: recording a transition never allocates (XR-RPP-009)
  int64_t t_ns;
};

enum class CornerAction : uint8_t { Brake, Pivot, SettleBrake, Advance };

struct CornerOutput {
  CornerAction action{CornerAction::Brake};
  FsmState state{FsmState::Brake};
  double brake_speed{0.0};  // Brake / SettleBrake: signed along the nose
  double heading_err{0.0};  // Pivot: wrapped target - yaw (the pivot-rate law is motion_output's)
  bool collinear{false};    // Advance without a stop: momentum is kept
  bool zero_speed_memory{false};  // Advance across a real corner: the caller zeroes last_speed_cmd
  bool set_speed_memory{false};   // Pivot: the caller sets last_speed_cmd = speed_memory
  double speed_memory{0.0};
  // Pivot only: the pivot watchdog has expired and the heading is still outside the (widened)
  // release band. False on every other action (brake, settle brake, advance).
  bool pivot_timed_out{false};
};

struct CornerInput {
  int64_t now_ns;
  double heading_err_rad;  // wrapped (target - yaw)
  double corner_deg;       // |path corner| in degrees
  StopTelemetry tel;
  // Pivot watchdog angle when it is known in radians (a run-entry alignment carries its own); NaN
  // derives it from corner_deg, exactly as the prototype's corner block does.
  double turn_angle_rad{std::numeric_limits<double>::quiet_NaN()};
};

// The hard-corner flow at a segment vertex (the corner block of the segment profile):
// TRACKING -> BRAKE -> PIVOT -> RELEASE_SETTLE -> ADVANCE, with the collinear shortcut.
class CornerFsm {
public:
  // `shared_stop`: the prototype keeps ONE stop confirmation for the corner flow, the run-boundary
  // hold, the completion hold and the endpoint precise stop (a hold that ends in the completion
  // hold reuses the dwell the precise stop already served). Pass the shared object to keep that.
  explicit CornerFsm(const StopPivotParams& p, StopConfirm* shared_stop = nullptr)
      : p_(p), stop_(shared_stop != nullptr ? shared_stop : &own_stop_) {}
  CornerFsm(const CornerFsm&) = delete;
  CornerFsm& operator=(const CornerFsm&) = delete;
  void set_params(const StopPivotParams& p) { p_ = p; }
  CornerOutput step(const CornerInput& in);
  void reset();                                          // after an advance or a new run
  void carry_stop_complete() { stop_complete_ = true; }  // run boundary: the stop was already done
  FsmState state() const { return state_; }
  bool stop_complete() const { return stop_complete_; }
  // The most recent transitions, oldest first, in a fixed ring (no allocation in the tick).
  static constexpr size_t kLogCapacity = 256;
  size_t log_size() const { return log_count_; }
  const Transition& log_entry(size_t i) const {
    return log_[(log_head_ + kLogCapacity - log_count_ + i) % kLogCapacity];
  }

private:
  void go(FsmState to, const char* reason, int64_t now_ns);
  void record(FsmState from, FsmState to, const char* reason, int64_t now_ns);
  StopPivotParams p_;
  StopConfirm own_stop_;
  StopConfirm* stop_;
  PivotWatchdog pivot_;
  bool stop_complete_{false};
  bool settle_active_{false};
  int64_t settle_since_ns_{0};
  FsmState state_{FsmState::Tracking};
  std::array<Transition, kLogCapacity> log_{};
  size_t log_head_{0};
  size_t log_count_{0};
  uint64_t seq_{0};
};

// Brake to a confirmed physical stop, then report Done (completion hold D3, and the stop before a
// run boundary). Checked before the goal test every tick by the caller.
enum class HoldPhase : uint8_t { Idle, Braking, Stopped };

struct HoldOutput {
  HoldPhase phase{HoldPhase::Idle};
  double brake_speed{0.0};
  bool stopped{false};  // the tick on which the stop is confirmed
};

class StopHold {
public:
  explicit StopHold(const StopPivotParams& p, StopConfirm* shared_stop = nullptr)
      : p_(p), stop_(shared_stop != nullptr ? shared_stop : &own_stop_) {}
  StopHold(const StopHold&) = delete;
  StopHold& operator=(const StopHold&) = delete;
  void set_params(const StopPivotParams& p) { p_ = p; }
  HoldOutput step(int64_t now_ns, const StopTelemetry& tel);
  bool latched() const { return latched_; }
  void reset() {
    latched_ = false;
    stop_->reset();
  }

private:
  StopPivotParams p_;
  StopConfirm own_stop_;
  StopConfirm* stop_;
  bool latched_{false};
};

}  // namespace dyx3_rpp
