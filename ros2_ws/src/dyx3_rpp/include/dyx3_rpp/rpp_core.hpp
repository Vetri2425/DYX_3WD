// rpp_core — the 50 Hz control-tick orchestrator. Contract: docs/contracts/rpp_orchestrator.md.
// Pure C++, no ROS, no allocation in tick(). Port of _control_loop / _control_loop_impl /
// _control_segment_profile (steady tracking) of the carried prototype, proven tick by tick against
// the verbatim Python (test/orchestrator_equivalence_test.cpp,
// tools/gate4/gen_orchestrator_vectors.py).
//
// SCOPE: the gates in front of the controller (pose staleness and extrapolation, RTK gate with
// recovery hold, EKF jump guard with reset compensation), projection, the goal test, the smooth and
// segment tracking laws, the corner stop-and-pivot, the run-boundary hold, the run-entry alignment
// pivot, the endpoint precise stop and the completion hold (the explicit machines of
// stop_pivot_fsm, sharing ONE stop confirmation as the prototype does), and the state each tick
// leaves behind. NOT ported: the point hold (any value of point_hold_enabled), the point handshake,
// progress publication. A tick that would enter the point hold publishes ZERO and says so
// (TickOutput::handoff). Fail to zero, never to a guess.
#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "dyx3_geometry/point.hpp"
#include "dyx3_geometry/project_onto_path.hpp"
#include "dyx3_rpp/path_conditioner.hpp"
#include "dyx3_rpp/rpp_params.hpp"
#include "dyx3_rpp/stop_pivot_fsm.hpp"

namespace dyx3_rpp {

// /rpp/debug[7]
enum class StateCode : int {
  Stale = -1,
  Idle = 0,
  Tracking = 1,
  Approach = 2,
  Done = 3,
  RtkWait = 4,
  JumpSkip = 5
};

// /rpp/segment_debug[1]
enum class SegState : int {
  Inactive = 0,
  TrackSegment = 1,
  PreCornerSlowdown = 2,
  CornerAlign = 3,
  Done = 4,
  CornerStop = 5
};

// A feature this orchestrator does not run. None means the tick was fully handled here.
enum class Handoff : uint8_t {
  None = 0,
  PointHold,  // point_hold_enabled (default OFF in the prototype)
};
const char* to_string(Handoff h);

// What the tick asks the vehicle to do, in the controller's own vocabulary (the legacy velocity
// vector below is the prototype's encoding of the same decision; the node maps this to
// MotionSetpoint modes).
enum class CmdKind : uint8_t {
  Stop = 0,  // zero speed, zero rate
  Track,     // follow: the NED velocity vector (v_n, v_e) with the body yaw rate
  Brake,     // active body-axis brake: signed speed along the nose (brake_speed)
  Pivot,     // in-place turn toward the exit heading (pivot_heading_err, wrapped target - yaw)
  Creep,     // endpoint precise stop: a small signed speed along the nose (creep_speed), no turn;
             // (v_n, v_e) carries the correction direction, which the command layer steers toward
             // when it is off the nose (XR-RPP-001, rpp_command.hpp)
};
const char* to_string(CmdKind k);

// Why the RTK gate refused (the prototype returned a free-text reason).
enum class RtkReason : uint8_t {
  Ok = 0,
  Unavailable,
  Stale,
  FixBelowMin,
  NotRoverFix,
  AccuracyUnknown,
  AccuracyTooLarge,
  Recovering
};

// One /rpp/debug row (indices 0-10 and 47-50 of the prototype's array; the parameter snapshot is
// the node's).
struct DebugRow {
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  double cross_track{kNaN}, heading_err{kNaN}, lookahead{kNaN}, speed{0}, kappa{kNaN},
      dist_goal{kNaN}, pose_age_ms{kNaN};
  int state{0};
  double l_d_raw{kNaN}, kappa_speed{kNaN}, yaw_rate{0};
  bool spray_active{false};  // Planner MARK/TRANSIT diagnostic; never an actuator verdict
  double speed_raw{kNaN}, v_lat_limit{kNaN}, accel_scale{kNaN}, speed_mode{kNaN};
};

// /rpp/segment_debug[1..8]
struct SegmentDebugRow {
  int state{0};
  int seg_idx{0};
  double dist_to_segment_end{0}, dist_to_corner{0}, corner_angle_deg{0}, target_heading_ned{0},
      heading_error_rad{0}, yaw_rate_body{0};
};

struct TickOutput {
  // The NED velocity vector and body yaw rate the prototype published (the LAST publish of the
  // tick).
  double v_n{0}, v_e{0}, yaw_rate{0};
  StateCode state{StateCode::Idle};
  CmdKind cmd{CmdKind::Stop};
  double brake_speed{0.0};  // CmdKind::Brake: signed speed along the nose (+ forward, - reverse)
  double pivot_heading_err{0.0};  // CmdKind::Pivot: wrapped (target heading - yaw), rad
  double pivot_speed_memory{
      0.0};  // CmdKind::Pivot: the prototype's corner speed (the vector magnitude)
  // CmdKind::Pivot (corner pivot or run-entry alignment pivot): the pivot watchdog has expired
  // and the heading is still outside the release band (RppStatus.pivot_timed_out). False on
  // every other tick. Reported only: the core keeps pivoting; the mission decides to pause.
  bool pivot_timed_out{false};
  double creep_speed{0.0};  // CmdKind::Creep: signed speed along the nose (- reverse)
  double track_heading_ned{
      0.0};             // CmdKind::Track: heading target (frozen below 1 cm/s: no North snap)
  double yaw_ned{0.0};  // the heading this tick used (the nose, for a held-heading command)
  Handoff handoff{Handoff::None};
  RtkReason rtk_reason{RtkReason::Ok};
  bool velocity_published{
      false};  // the prototype returned without publishing a command (run switch)
  bool debug_valid{false};
  DebugRow debug;
  // Cross-track for RppStatus.cross_track_right_m: metres, RIGHT of the directed path positive
  // (frames.md). Equal to debug.cross_track, except in the endpoint precise stop, whose legacy
  // debug value is left-positive (kept for the equivalence with the prototype; XR-RPP-005).
  double cross_track_right{std::numeric_limits<double>::quiet_NaN()};
  bool segment_debug_valid{false};
  SegmentDebugRow segment_debug;
  int segment_debug_publishes{0};
};

// A pose already in local NED (north, east, yaw clockwise from North), as the node derives it.
struct NedPose {
  double n{0}, e{0}, yaw_ned{0};
};

// ENU quaternion -> NED yaw, exactly the prototype's _enu_pose_to_ned (the same float operations).
double yaw_ned_from_enu_quaternion(double w, double x, double y, double z);

// Plain snapshot of the state a tick leaves behind (compared by the equivalence test).
struct CoreState {
  double last_speed_cmd, last_yaw_cmd, path_travel_m, tick_dt;
  int segment_idx, run_idx, hint_seg;
  bool hint_valid, kappa_hard_latched, stop_latched, path_done, run_align_pending;
  double ekf_offset_n, ekf_offset_e;
  int ekf_reset_count;
  bool have_last_pos;
  double last_pos_n, last_pos_e;
  int segment_state;
  bool rtk_recovering;
  bool run_boundary_stop_pending, completion_stop_pending, endpoint_stop_active,
      corner_stop_complete;
  double run_align_turn_rad;
};

class RppCore {
public:
  static constexpr int kControlHz = 50;
  static constexpr size_t kPoseGaps = 64;

  // `params` must outlive the core; LIVE parameter changes are read at the next tick.
  explicit RppCore(const ParamSet& params);

  // Mission lifecycle (mirror of _install_mission + _apply_run(0)). Allocates: call from the node
  // thread, never mid-tick. Takes ownership of the runs.
  void install_mission(std::vector<ConditionedRun> runs);
  // Begin the installed mission at run `idx` instead of run 0 (MissionState.start_run_index: a
  // resumed execution). The per-run reset of run `idx` with the run-boundary stop already
  // confirmed, exactly as when run `idx` is reached after the stop at a hard run boundary
  // (apply_run(idx, pre_stopped = true)): a pending entry alignment pivots without a second stop.
  // idx 0 is the plain start (identical to install_mission). Call after install_mission and
  // before the first tick; returns false and changes nothing when idx is not a run of the
  // installed mission.
  bool start_at_run(size_t idx);

  // Inputs (each stamps its own arrival time). All times are int64 ns on one monotonic clock.
  void on_pose(const NedPose& pose, int64_t now_ns);
  void on_velocity(double v_north, double v_east, double yaw_rate_ned, int64_t now_ns);
  void on_gps(int fix_type, double h_acc_m /*NaN = unknown*/, int64_t now_ns);

  // One control tick. The returned reference is valid until the next call.
  const TickOutput& tick(int64_t now_ns);

  CoreState snapshot() const;

  // The part of _apply_run that clears the pending alignment pivot: ONLY for tests and for the node
  // once the alignment machine is ported. Production behaviour without it is a handoff.
  void mark_alignment_done() { run_align_pending_ = false; }
  // Test hook: a warm start (the commanded-speed memory of a rover that is already moving).
  void test_set_last_speed_cmd(double v) { last_speed_cmd_ = v; }

  // Not running (paused, waiting): forget the motion memory exactly as a zero publish does, and
  // restart the stop confirmation. The node calls this instead of ticking. Also forgets what makes
  // a resume differ from a fresh start of the same run (XR-RPP-008): the jump-guard position, the
  // tick period, the projection hint (open runs), the precise-stop engagement and its timer, and
  // the stop latch.
  void pause();
  bool path_done() const { return path_done_; }
  bool profile_segment() const { return profile_segment_; }
  size_t run_index() const { return run_idx_; }
  bool loaded() const { return run_ != nullptr; }

  size_t run_count() const { return runs_.size(); }
  int ekf_reset_count() const { return ekf_reset_count_; }

private:
  struct Recv {
    bool has{false};
    int64_t ns{0};
  };

  // ---- front end ----
  bool vel_is_fresh(int64_t now_ns) const;
  double measured_speed(int64_t now_ns) const;
  bool rtk_gate(int64_t now_ns, RtkReason* reason);
  void apply_run(int idx, bool pre_stopped = false);
  bool advance_run(bool pre_stopped = false);
  StopTelemetry telemetry(int64_t now_ns, double yaw_ned) const;
  StopPivotParams stop_params() const;
  void reset_corner_pivot_state() { corner_fsm_.reset(); }
  double debug_xtrack(double pos_n, double pos_e) const;
  double next_run_turn() const;
  bool next_run_requires_alignment() const;
  void publish_brake(double yaw_ned, const StopTelemetry& tel, double* speed_out);
  bool run_alignment_hold(double pos_n, double pos_e, double yaw_ned, double pose_age_s,
                          int64_t now_ns);
  void hold_before_run_advance(double pos_n, double pos_e, double yaw_ned, double pose_age_s,
                               double dist_to_goal, int64_t now_ns);
  void hold_at_completion(double pos_n, double pos_e, double yaw_ned, double pose_age_s,
                          double dist_to_goal, int64_t now_ns);
  bool precise_stop_tick(double pos_n, double pos_e, double yaw_ned, double pose_age_s,
                         double dist_to_goal, int64_t now_ns);

  // ---- publishes ----
  void publish_velocity(double v_n, double v_e);
  void publish_yaw_rate(double yr);
  void publish_zero(StateCode state, double pose_age_ms,
                    double dist_to_goal = std::numeric_limits<double>::quiet_NaN());
  void publish_debug(const DebugRow& row);
  void publish_segment_debug(SegState s, int seg_idx, double dist_end, double dist_corner,
                             double corner_angle, double target_heading, double heading_err,
                             double yaw_rate_body);
  void handoff(Handoff h);

  // ---- tracking ----
  void control_loop_impl(int64_t now_ns);
  void control_smooth(double pos_n, double pos_e, double yaw_ned, double pose_age_s,
                      double dist_to_goal, dyx3_geometry::PathProjection proj, int64_t now_ns);
  void control_segment(double pos_n, double pos_e, double yaw_ned, double pose_age_s,
                       double dist_to_goal, int64_t now_ns);
  double stop_latch_filter(double speed, double stop_dist, int64_t now_ns);
  void clamp_to_forward_cone(double& v_n, double& v_e, double yaw_ned, double speed) const;
  bool segment_spray_active(int seg_idx) const;
  double goal_tol_effective(double goal_tol) const;
  std::optional<double> run_remaining_along() const;
  bool endpoint_capture_recovered(double pos_n, double pos_e, double dist_to_goal) const;
  double run_min_travel() const;
  void update_path_progress(int seg_idx, double t);

  const ParamSet& params_;
  TickOutput out_;

  // the stop machines: ONE stop confirmation shared by the corner flow and every hold (as in the
  // prototype)
  StopPivotParams sp_;  // refreshed from the ParamSet at the top of every tick
  StopConfirm stop_confirm_;
  CornerFsm corner_fsm_;
  StopHold boundary_hold_;
  StopHold completion_hold_;

  // mission
  std::vector<ConditionedRun> runs_;
  std::vector<double> run_tail_transit_;
  size_t run_idx_{0};
  const ConditionedRun* run_{nullptr};
  bool profile_segment_{false};  // _active_tracking_profile == "segment" (default "smooth")
  bool run_align_pending_{false};
  bool path_done_{false};
  bool run_boundary_stop_pending_{false};
  bool completion_stop_pending_{false};
  bool segment_endpoint_stop_active_{false};
  bool endpoint_brake_hold_{false};  // precise stop: braking inside the arrival / hold band
  bool endpoint_stop_started_{false};
  int64_t endpoint_stop_start_ns_{0};
  double run_align_turn_rad_{0.0};
  int segment_idx_{0};
  SegState segment_state_{SegState::Inactive};
  bool stop_latched_{false};
  double path_travel_m_{0.0};
  double run_tail_transit_m_{0.0};

  // guidance state
  dyx3_geometry::ProjectionHint hint_;
  double last_speed_cmd_{0.0};
  bool kappa_hard_latched_{false};
  double last_yaw_cmd_{0.0};
  int64_t last_tick_ns_{0};
  bool have_last_tick_{false};
  double tick_dt_{1.0 / kControlHz};

  // sensors
  bool have_pose_{false};
  NedPose pose_;
  int64_t pose_recv_ns_{0};
  std::array<double, kPoseGaps> gaps_{};
  size_t gaps_count_{0}, gaps_head_{0};
  double vel_n_{0}, vel_e_{0}, yaw_rate_ned_{0};
  Recv vel_;
  int gps_fix_{0};
  double gps_h_acc_{0};
  bool gps_h_acc_known_{false};
  Recv gps_;
  Recv rtk_recover_since_;

  // jump guard
  bool have_last_pos_{false};
  double last_pos_n_{0}, last_pos_e_{0};
  double ekf_off_n_{0}, ekf_off_e_{0};
  int ekf_reset_count_{0};
};

}  // namespace dyx3_rpp
