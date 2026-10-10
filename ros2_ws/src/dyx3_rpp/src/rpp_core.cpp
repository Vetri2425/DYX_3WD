#include "dyx3_rpp/rpp_core.hpp"

#include <algorithm>
#include <cmath>

#include "dyx3_geometry/angle_wrap.hpp"
#include "dyx3_geometry/curvature.hpp"
#include "dyx3_geometry/heading_delta.hpp"
#include "dyx3_geometry/project_onto_segment.hpp"
#include "dyx3_rpp/guidance.hpp"
#include "dyx3_rpp/speed_profile.hpp"
#include "dyx3_rpp/terminal.hpp"

namespace dyx3_rpp {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kPi = M_PI;
// XR-RPP-009: upper bound of the smooth-profile curvature preview count (= the parameter bound of
// preview_curvature_n). DERIVED — NOT FROM V1 SPEC.
constexpr int kMaxPreviewN = 64;
// _CORNER_MAX_BEARING_OFFSET_RAD = math.radians(75.0)
constexpr double kMaxBearingOffsetRad = 75.0 * (M_PI / 180.0);

double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
double dist(double ax, double ay, double bx, double by) { return std::hypot(ax - bx, ay - by); }
double ns_to_s(int64_t ns) { return static_cast<double>(ns) * 1e-9; }

// Python float %: the result takes the sign of the divisor.
double py_mod(double x, double y) {
  double m = std::fmod(x, y);
  if (m != 0.0) {
    if ((y < 0.0) != (m < 0.0)) m += y;
  } else {
    m = std::copysign(0.0, y);
  }
  return m;
}

}  // namespace

const char* to_string(Handoff h) {
  switch (h) {
    case Handoff::None:
      return "NONE";
    case Handoff::PointHold:
      return "POINT_HOLD";
  }
  return "?";
}

const char* to_string(CmdKind k) {
  switch (k) {
    case CmdKind::Stop:
      return "STOP";
    case CmdKind::Track:
      return "TRACK";
    case CmdKind::Brake:
      return "BRAKE";
    case CmdKind::Pivot:
      return "PIVOT";
    case CmdKind::Creep:
      return "CREEP";
  }
  return "?";
}

double yaw_ned_from_enu_quaternion(double w, double x, double y, double z) {
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  const double yaw_enu = std::atan2(siny_cosp, cosy_cosp);
  double yaw_ned = kPi / 2.0 - yaw_enu;
  yaw_ned = py_mod(yaw_ned + kPi, 2.0 * kPi) - kPi;
  return yaw_ned;
}

RppCore::RppCore(const ParamSet& params)
    : params_(params),
      corner_fsm_(StopPivotParams{}, &stop_confirm_),
      boundary_hold_(StopPivotParams{}, &stop_confirm_),
      completion_hold_(StopPivotParams{}, &stop_confirm_) {}

// ------------------------------------------------------------------------------------------------
// mission lifecycle
// ------------------------------------------------------------------------------------------------
void RppCore::install_mission(std::vector<ConditionedRun> runs) {
  runs_ = std::move(runs);
  run_tail_transit_.assign(runs_.size(), 0.0);
  for (size_t i = 0; i < runs_.size(); ++i) {
    std::vector<bool> fl(runs_[i].flags.begin(), runs_[i].flags.end());
    run_tail_transit_[i] = measure_tail_transit_m(PathView(runs_[i].pts), fl);
  }
  last_speed_cmd_ = 0.0;
  kappa_hard_latched_ = false;
  stop_latched_ = false;
  segment_endpoint_stop_active_ = false;
  endpoint_brake_hold_ = false;
  have_last_tick_ = false;
  have_last_pos_ = false;
  ekf_off_n_ = ekf_off_e_ = 0.0;
  ekf_reset_count_ = 0;
  run_ = nullptr;
  if (runs_.empty()) return;
  apply_run(0);
}

void RppCore::apply_run(int idx, bool pre_stopped) {
  const ConditionedRun* prev = idx > 0 ? &runs_[static_cast<size_t>(idx) - 1] : nullptr;
  const ConditionedRun& run = runs_[static_cast<size_t>(idx)];
  run_align_pending_ = false;
  run_align_turn_rad_ = 0.0;
  if (prev != nullptr && prev->pts.size() > 1 && run.pts.size() > 1) {
    const Point p0 = prev->pts[prev->pts.size() - 2];
    const Point p1 = prev->pts[prev->pts.size() - 1];
    const double h0 = std::atan2(p1.e - p0.e, p1.n - p0.n);
    const Point n0 = run.pts[0];
    const Point n1 = run.pts[1];
    const double h1 = std::atan2(n1.e - n0.e, n1.n - n0.n);
    const double threshold = params_.num(P::segment_corner_threshold_deg) * (kPi / 180.0);
    const double turn = std::fabs(dyx3_geometry::heading_delta(h0, h1));
    if (turn >= threshold) {
      run_align_pending_ = true;
      run_align_turn_rad_ = turn;  // angle-aware pivot budget
    }
  } else if (idx == 0 && run.pts.size() > 1 &&
             (params_.flag(P::entry_prealign_enabled) ||
              (!run.flags.empty() && run.flags[0] != 0))) {
    run_align_pending_ = true;
    run_align_turn_rad_ = kPi;  // worst case: the entry pose is unknown here
  }
  reset_corner_pivot_state();
  // A hard run boundary is stopped before the advance: carry that stop so the entry pivot starts
  // directly.
  if (pre_stopped && run_align_pending_) corner_fsm_.carry_stop_complete();
  run_boundary_stop_pending_ = false;
  run_idx_ = static_cast<size_t>(idx);
  run_ = &run;
  profile_segment_ = run.profile == Profile::Segment;
  segment_idx_ = 0;
  segment_state_ =
      (profile_segment_ && run.pts.size() >= 2) ? SegState::TrackSegment : SegState::Inactive;
  path_done_ = false;
  completion_stop_pending_ = false;
  segment_endpoint_stop_active_ = false;
  endpoint_brake_hold_ = false;
  path_travel_m_ = 0.0;
  kappa_hard_latched_ = false;
  run_tail_transit_m_ = run_tail_transit_[static_cast<size_t>(idx)];
  hint_.seg = 0;
  hint_.valid = run.closed;
}

bool RppCore::start_at_run(size_t idx) {
  if (idx >= runs_.size()) return false;
  // DERIVED — NOT FROM V1 SPEC: a resumed execution starts with the rover standing still (the
  // mission arms and engages before RUNNING), so run `idx` begins in the state the run-boundary
  // hold leaves behind (stop confirmed, pre_stopped), not in the state of a run-0 start.
  apply_run(static_cast<int>(idx), idx > 0);
  // DERIVED — NOT FROM V1 SPEC: the entry pose of a resumed run is unknown (the rover was paused,
  // moved or re-placed), so the alignment watchdog budgets the worst case, exactly as run 0 does,
  // instead of the run-boundary turn. Measured on the stand-in: a 163 deg alignment at the
  // default pivot rate takes about 7 s, past the 5.0 s budget of a 90 deg boundary, and would be
  // reported as a pivot timeout (and pause the mission) while it is turning correctly.
  if (idx > 0 && run_align_pending_) run_align_turn_rad_ = kPi;
  return true;
}

bool RppCore::advance_run(bool pre_stopped) {
  if (run_idx_ + 1 >= runs_.size()) return false;
  apply_run(static_cast<int>(run_idx_) + 1, pre_stopped);
  return true;
}

// ------------------------------------------------------------------------------------------------
// inputs
// ------------------------------------------------------------------------------------------------
void RppCore::on_pose(const NedPose& pose, int64_t now_ns) {
  // RPP-002: a non-finite sample is not a pose; the last good one ages out into STALE.
  if (!std::isfinite(pose.n) || !std::isfinite(pose.e) || !std::isfinite(pose.yaw_ned)) return;
  if (have_pose_) {
    const double gap_s = ns_to_s(now_ns - pose_recv_ns_);
    if (gap_s > 0.0 && gap_s < 1.0) {
      gaps_[gaps_head_] = gap_s;
      gaps_head_ = (gaps_head_ + 1) % kPoseGaps;
      if (gaps_count_ < kPoseGaps) ++gaps_count_;
    }
  }
  pose_ = pose;
  have_pose_ = true;
  pose_recv_ns_ = now_ns;
}

void RppCore::on_velocity(double v_north, double v_east, double yaw_rate_ned, int64_t now_ns) {
  // RPP-002: a non-finite velocity is not fed (a NaN with a fresh stamp made brake_speed return
  // +cap forward); the last good sample goes stale instead.
  if (!std::isfinite(v_north) || !std::isfinite(v_east) || !std::isfinite(yaw_rate_ned)) return;
  vel_n_ = v_north;
  vel_e_ = v_east;
  yaw_rate_ned_ = yaw_rate_ned;
  vel_.has = true;
  vel_.ns = now_ns;
}

void RppCore::on_gps(int fix_type, double h_acc_m, int64_t now_ns) {
  gps_fix_ = fix_type;
  gps_.has = true;
  gps_.ns = now_ns;
  gps_h_acc_known_ = std::isfinite(h_acc_m);
  gps_h_acc_ = gps_h_acc_known_ ? h_acc_m : 0.0;
}

bool RppCore::vel_is_fresh(int64_t now_ns) const {
  if (!vel_.has) return false;
  return ns_to_s(now_ns - vel_.ns) < 0.3;
}

double RppCore::measured_speed(int64_t now_ns) const {
  if (!vel_.has) return 0.0;
  const double age = ns_to_s(now_ns - vel_.ns);
  if (age >= 0.5) return 0.0;
  return std::hypot(vel_n_, vel_e_);
}

// evaluate_rtk_quality (min_fix_type = 6, as the prototype calls it) plus the recovery hold.
bool RppCore::rtk_gate(int64_t now_ns, RtkReason* reason) {
  *reason = RtkReason::Ok;
  const double timeout_s = std::max(0.0, params_.num(P::rtk_fix_timeout_s));
  auto fail = [&](RtkReason r) {
    rtk_recover_since_.has = false;
    *reason = r;
    return false;
  };
  if (!gps_.has) return fail(RtkReason::Unavailable);
  const double age_s = ns_to_s(now_ns - gps_.ns);
  if (!std::isfinite(age_s)) return fail(RtkReason::Unavailable);
  if (age_s < 0.0 || age_s > timeout_s) return fail(RtkReason::Stale);
  if (gps_fix_ < 6) return fail(RtkReason::FixBelowMin);
  if (gps_fix_ != 5 && gps_fix_ != 6) return fail(RtkReason::NotRoverFix);
  bool have_acc = false;
  double accuracy = 0.0;
  if (gps_h_acc_known_ && std::isfinite(gps_h_acc_) && gps_h_acc_ >= 0.0) {
    have_acc = true;
    accuracy = gps_h_acc_;
  }
  if (!have_acc) {
    if (params_.flag(P::rtk_require_accuracy)) return fail(RtkReason::AccuracyUnknown);
  } else {
    const double max_h = params_.num(P::rtk_max_hrms_m);
    if (max_h > 0.0 && accuracy > max_h) return fail(RtkReason::AccuracyTooLarge);
  }
  const double hold_s = std::max(0.0, params_.num(P::rtk_recover_hold_s));
  if (!rtk_recover_since_.has) {
    rtk_recover_since_.has = true;
    rtk_recover_since_.ns = now_ns;
  }
  double held_s = ns_to_s(now_ns - rtk_recover_since_.ns);
  if (held_s < 0.0) {
    rtk_recover_since_.ns = now_ns;
    held_s = 0.0;
  }
  if (held_s < hold_s) {
    *reason = RtkReason::Recovering;
    return false;
  }
  return true;
}

// ------------------------------------------------------------------------------------------------
// publishes (the LAST publish of a stream in a tick is what the tick reports)
// ------------------------------------------------------------------------------------------------
void RppCore::publish_velocity(double v_n, double v_e) {
  out_.v_n = v_n;
  out_.v_e = v_e;
  out_.velocity_published = true;
  out_.cmd = CmdKind::Track;
}

void RppCore::publish_yaw_rate(double yr) { out_.yaw_rate = yr; }

void RppCore::publish_debug(const DebugRow& row) {
  out_.debug = row;
  out_.cross_track_right = row.cross_track;
  out_.state = static_cast<StateCode>(row.state);
  out_.debug_valid = true;
}

void RppCore::publish_segment_debug(SegState s, int seg_idx, double dist_end, double dist_corner,
                                    double corner_angle, double target_heading, double heading_err,
                                    double yaw_rate_body) {
  out_.segment_debug = {static_cast<int>(s), seg_idx,        dist_end,    dist_corner,
                        corner_angle,        target_heading, heading_err, yaw_rate_body};
  out_.segment_debug_valid = true;
  ++out_.segment_debug_publishes;
}

void RppCore::publish_zero(StateCode state, double pose_age_ms, double dist_to_goal) {
  publish_velocity(0.0, 0.0);
  publish_yaw_rate(0.0);
  out_.cmd = CmdKind::Stop;
  if (state != StateCode::JumpSkip) {
    last_speed_cmd_ = 0.0;
    kappa_hard_latched_ = false;
  }
  DebugRow row;
  row.speed = 0.0;
  row.dist_goal = dist_to_goal;
  row.pose_age_ms = pose_age_ms;
  row.state = static_cast<int>(state);
  row.yaw_rate = 0.0;
  row.spray_active = false;
  publish_debug(row);
  if (profile_segment_) {
    const PathView path = run_ ? PathView(run_->pts) : PathView();
    const int n = static_cast<int>(path.n);
    const int seg_idx = std::max(0, std::min(segment_idx_, std::max(0, n - 2)));
    SegState dbg_state = segment_state_;
    if (state == StateCode::Done) dbg_state = SegState::Done;
    publish_segment_debug(dbg_state, seg_idx, kNaN, dist_to_goal,
                          n >= 3 ? segment_angle_deg(path, seg_idx) : kNaN, kNaN, kNaN, 0.0);
  }
}

// A stop/pivot machine this slice does not run: fail to zero and say which one.
void RppCore::handoff(Handoff h) {
  out_.handoff = h;
  publish_velocity(0.0, 0.0);
  publish_yaw_rate(0.0);
  out_.cmd = CmdKind::Stop;
  last_speed_cmd_ = 0.0;
  DebugRow row;
  row.speed = 0.0;
  row.state = static_cast<int>(StateCode::Idle);
  row.yaw_rate = 0.0;
  publish_debug(row);
}

// ------------------------------------------------------------------------------------------------
// small helpers carried from the prototype
// ------------------------------------------------------------------------------------------------
bool RppCore::segment_spray_active(int seg_idx) const {
  if (run_ == nullptr || path_done_ || run_->pts.size() < 2) return false;
  if (run_->flags.size() != run_->pts.size()) return false;
  const int n = static_cast<int>(run_->pts.size());
  const int seg = std::max(0, std::min(seg_idx, n - 2));
  return run_->flags[static_cast<size_t>(seg)] != 0 &&
         run_->flags[static_cast<size_t>(seg) + 1] != 0;
}

double RppCore::run_min_travel() const {
  RunInfo info;
  if (run_ != nullptr) {
    info.valid = true;
    info.length = run_->length;
    info.closed = run_->closed;
  }
  return dyx3_rpp::run_min_travel(info, params_.num(P::min_goal_travel_m),
                                  params_.num(P::closed_loop_min_travel_frac));
}

void RppCore::update_path_progress(int seg_idx, double t) {
  if (run_ == nullptr) return;
  path_travel_m_ =
      std::max(path_travel_m_, path_progress_at(run_->cum_s, run_->pts.size(), seg_idx, t));
}

std::optional<double> RppCore::run_remaining_along() const {
  RunInfo info;
  if (run_ != nullptr) {
    info.valid = true;
    info.length = run_->length;
    info.closed = run_->closed;
  }
  static const std::vector<double> kEmpty;
  return dyx3_rpp::run_remaining_along(info, run_ ? run_->cum_s : kEmpty,
                                       run_ ? run_->pts.size() : 0, path_travel_m_);
}

double RppCore::goal_tol_effective(double goal_tol) const {
  return dyx3_rpp::goal_tol_effective(goal_tol, run_tail_transit_m_,
                                      params_.num(P::transit_runout_goal_tolerance_m));
}

bool RppCore::endpoint_capture_recovered(double pos_n, double pos_e,
                                         double /*dist_to_goal*/) const {
  if (run_ == nullptr) return false;
  CaptureParams cp;
  cp.enabled = params_.flag(P::endpoint_capture_recover_enabled);
  cp.goal_tol = params_.num(P::xy_goal_tolerance);
  cp.tail_transit_m = run_tail_transit_m_;
  cp.transit_runout_goal_tolerance_m = params_.num(P::transit_runout_goal_tolerance_m);
  cp.past_m = params_.num(P::endpoint_capture_past_m);
  cp.max_miss_m = params_.num(P::endpoint_capture_max_miss_m);
  return dyx3_rpp::endpoint_capture_recovered(PathView(run_->pts), Point{pos_n, pos_e},
                                              run_remaining_along(),
                                              cp) == CaptureVerdict::Recovered;
}

// [STOP-LATCH] cmd in {0} U [min_actuatable, vmax] on the segment DRIVE command.
double RppCore::stop_latch_filter(double speed, double stop_dist, int64_t now_ns) {
  if (!params_.flag(P::stop_latch_enabled)) return speed;
  const double th = params_.num(P::stop_latch_min_actuatable_m_s);
  const double cap = params_.num(P::stop_latch_capture_dist_m);
  const double rel = params_.num(P::stop_latch_release_dist_m);
  if (stop_latched_) {
    if (speed > 0.0 && stop_dist > rel) {
      stop_latched_ = false;
      return std::max(speed, th);
    }
    if (speed > 0.0 && stop_dist > cap && measured_speed(now_ns) < 0.02) return th;
    return 0.0;
  }
  if (speed <= 0.0 || speed >= th) return speed;
  if (stop_dist <= cap) {
    stop_latched_ = true;
    return 0.0;
  }
  return th;
}

void RppCore::clamp_to_forward_cone(double& v_n, double& v_e, double yaw_ned, double speed) const {
  if (speed <= 1e-6) return;
  const double mag = std::hypot(v_n, v_e);
  if (mag <= 1e-9) return;
  const double bearing = std::atan2(v_e, v_n);
  const double heading_err = dyx3_geometry::angle_wrap(bearing - yaw_ned);
  if (std::fabs(heading_err) <= kMaxBearingOffsetRad) return;
  const double step = clampd(heading_err, -kMaxBearingOffsetRad, kMaxBearingOffsetRad);
  const double cmd_bearing = yaw_ned + step;
  v_n = speed * std::cos(cmd_bearing);
  v_e = speed * std::sin(cmd_bearing);
}

// ------------------------------------------------------------------------------------------------
// the stop machines
// ------------------------------------------------------------------------------------------------
StopPivotParams RppCore::stop_params() const {
  StopPivotParams p;
  p.stop_speed_threshold = params_.num(P::segment_stop_speed_threshold);
  p.stop_yaw_rate_threshold = params_.num(P::segment_stop_yaw_rate_threshold);
  p.stop_dwell_s = params_.num(P::segment_stop_dwell_s);
  p.brake_velocity_cap = params_.num(P::segment_brake_velocity_cap_m_s);
  p.align_speed_threshold = params_.num(P::segment_align_speed_threshold);
  p.align_settle_s = params_.num(P::segment_align_settle_s);
  p.heading_tolerance_rad = params_.num(P::segment_heading_tolerance_deg) * (kPi / 180.0);
  p.timeout_heading_tol_rad = params_.num(P::segment_timeout_heading_tolerance_deg) * (kPi / 180.0);
  p.release_max_rad = params_.num(P::segment_pivot_release_max_deg) * (kPi / 180.0);
  p.spinup_margin_s = params_.num(P::segment_pivot_spinup_margin_s);
  p.nominal_pivot_rate = params_.num(P::segment_nominal_pivot_rate_rad_s);
  p.turn_timeout_s = params_.num(P::segment_turn_timeout_s);
  p.pivot_timeout_max_s = params_.num(P::segment_pivot_timeout_max_s);
  p.corner_threshold_deg = params_.num(P::segment_corner_threshold_deg);
  p.corner_speed = std::max(0.05, params_.num(P::segment_min_corner_speed));
  p.stale_vel_hold_s = 2.0;  // _CORNER_STOP_MAX_HOLD_S: a constant in the prototype
  return p;
}

StopTelemetry RppCore::telemetry(int64_t now_ns, double yaw_ned) const {
  StopTelemetry t;
  t.vel_fresh = vel_is_fresh(now_ns);
  t.speed = std::hypot(vel_n_, vel_e_);
  t.v_forward = vel_n_ * std::cos(yaw_ned) + vel_e_ * std::sin(yaw_ned);
  t.yaw_rate = yaw_rate_ned_;
  return t;
}

// D7: the real signed cross-track for the debug emits inside holds (debug only, never feeds
// control).
double RppCore::debug_xtrack(double pos_n, double pos_e) const {
  if (run_ == nullptr || run_->pts.size() < 2) return 0.0;
  return dyx3_geometry::project_onto_segment(Point{pos_n, pos_e}, PathView(run_->pts), segment_idx_)
      .signed_cross;
}

double RppCore::next_run_turn() const {
  if (run_idx_ + 1 >= runs_.size()) return 0.0;
  const auto& cur = runs_[run_idx_].pts;
  const auto& nxt = runs_[run_idx_ + 1].pts;
  if (cur.size() < 2 || nxt.size() < 2) return 0.0;
  const Point a0 = cur[cur.size() - 2], a1 = cur[cur.size() - 1];
  const Point b0 = nxt[0], b1 = nxt[1];
  const double h0 = std::atan2(a1.e - a0.e, a1.n - a0.n);
  const double h1 = std::atan2(b1.e - b0.e, b1.n - b0.n);
  return std::fabs(dyx3_geometry::angle_wrap(h1 - h0));
}

bool RppCore::next_run_requires_alignment() const {
  return next_run_turn() >= params_.num(P::segment_corner_threshold_deg) * (kPi / 180.0);
}

// Active body-axis brake (invariant I1): exactly forward or reverse along the nose.
void RppCore::publish_brake(double yaw_ned, const StopTelemetry& tel, double* speed_out) {
  const double b = brake_speed(tel, sp_);
  const double bn = b * std::cos(yaw_ned);
  const double be = b * std::sin(yaw_ned);
  publish_velocity(bn, be);
  publish_yaw_rate(0.0);
  out_.cmd = CmdKind::Brake;
  out_.brake_speed = b;
  *speed_out = std::hypot(bn, be);
}

namespace {
DebugRow hold_row(double cross_track, double heading_err, double lookahead, double speed,
                  double dist_goal, double pose_age_ms, bool spray) {
  DebugRow row;
  row.cross_track = cross_track;
  row.heading_err = heading_err;
  row.lookahead = lookahead;
  row.speed = speed;
  row.kappa = 0.0;
  row.dist_goal = dist_goal;
  row.pose_age_ms = pose_age_ms;
  row.state = static_cast<int>(StateCode::Tracking);
  row.kappa_speed = 0.0;
  row.yaw_rate = 0.0;
  row.spray_active = spray;
  return row;
}
}  // namespace

// _run_alignment_hold: pivot in place toward the new run's first leg. True while stopping, pivoting
// or settling; false once aligned (or not applicable) so the tick continues with normal tracking.
bool RppCore::run_alignment_hold(double pos_n, double pos_e, double yaw_ned, double pose_age_s,
                                 int64_t now_ns) {
  if (!run_align_pending_) return false;
  const PathView path(run_->pts);
  if (path.n < 2) {
    run_align_pending_ = false;
    return false;
  }
  const Point a = path[0], b = path[1];
  if (dist(a.n, a.e, b.n, b.e) < 1e-6) {
    run_align_pending_ = false;
    return false;
  }
  const double leg_heading = std::atan2(b.e - a.e, b.n - a.n);
  const double target_heading = pivot_intercept_heading(Point{pos_n, pos_e}, a, b, leg_heading,
                                                        params_.flag(P::pivot_to_intercept_enabled),
                                                        params_.num(P::pivot_intercept_dist_m));
  const double heading_err = dyx3_geometry::angle_wrap(target_heading - yaw_ned);
  const StopTelemetry tel = telemetry(now_ns, yaw_ned);
  CornerInput in;
  in.now_ns = now_ns;
  in.heading_err_rad = heading_err;
  in.corner_deg =
      std::numeric_limits<double>::infinity();  // an entry pivot never takes the tangent shortcut
  in.tel = tel;
  in.turn_angle_rad = run_align_turn_rad_;
  const CornerOutput co = corner_fsm_.step(in);
  if (co.action == CornerAction::Advance) {  // settled
    run_align_pending_ = false;
    last_speed_cmd_ = 0.0;
    return false;
  }
  const Point fin = path[path.n - 1];
  const double dist_to_goal = dist(pos_n, pos_e, fin.n, fin.e);
  const double xt = debug_xtrack(pos_n, pos_e);
  const double age_ms = pose_age_s * 1000.0;
  double hv = 0.0;
  if (co.action == CornerAction::SettleBrake || co.action == CornerAction::Brake) {
    last_speed_cmd_ = 0.0;
    publish_brake(yaw_ned, tel, &hv);
    DebugRow row = hold_row(xt, heading_err, kNaN, hv, dist_to_goal, age_ms, false);
    publish_debug(row);
    publish_segment_debug(
        co.action == CornerAction::Brake ? SegState::CornerStop : SegState::CornerAlign, 0, kNaN,
        kNaN, kNaN, target_heading, heading_err, 0.0);
    return true;
  }
  // Pivot: the prototype commands a small vector at the exit heading, kept inside the forward cone.
  const double corner_speed = std::max(0.05, params_.num(P::segment_min_corner_speed));
  const double step = clampd(heading_err, -kMaxBearingOffsetRad, kMaxBearingOffsetRad);
  const double cmd_bearing = yaw_ned + step;
  last_speed_cmd_ = corner_speed;
  publish_velocity(corner_speed * std::cos(cmd_bearing), corner_speed * std::sin(cmd_bearing));
  publish_yaw_rate(0.0);
  out_.cmd = CmdKind::Pivot;
  out_.pivot_heading_err = heading_err;
  out_.pivot_speed_memory = corner_speed;
  out_.pivot_timed_out = co.pivot_timed_out;
  publish_debug(hold_row(xt, heading_err, kNaN, corner_speed, dist_to_goal, age_ms, false));
  publish_segment_debug(SegState::CornerAlign, 0, kNaN, kNaN, kNaN, target_heading, heading_err,
                        0.0);
  return true;
}

// _hold_before_run_advance: physically stop at a hard run boundary, then advance exactly once.
void RppCore::hold_before_run_advance(double pos_n, double pos_e, double yaw_ned, double pose_age_s,
                                      double dist_to_goal, int64_t now_ns) {
  if (run_idx_ + 1 >= runs_.size()) return;
  if (!next_run_requires_alignment()) {
    advance_run();  // a collinear transition: no stop, nothing published this tick
    return;
  }
  if (!run_boundary_stop_pending_) {
    reset_corner_pivot_state();
    run_boundary_stop_pending_ = true;
  }
  const auto& nxt = runs_[run_idx_ + 1].pts;
  const Point n0 = nxt[0], n1 = nxt[1];
  const double target_heading = std::atan2(n1.e - n0.e, n1.n - n0.n);
  const double heading_err = dyx3_geometry::angle_wrap(target_heading - yaw_ned);
  const StopTelemetry tel = telemetry(now_ns, yaw_ned);
  const HoldOutput h = boundary_hold_.step(now_ns, tel);
  if (h.stopped) {
    run_boundary_stop_pending_ = false;
    advance_run(true);
    return;
  }
  segment_state_ = SegState::CornerStop;
  last_speed_cmd_ = 0.0;
  double hv = 0.0;
  publish_brake(yaw_ned, tel, &hv);
  publish_debug(hold_row(debug_xtrack(pos_n, pos_e), heading_err, dist_to_goal, hv, dist_to_goal,
                         pose_age_s * 1000.0, false));
  publish_segment_debug(SegState::CornerStop, std::max(0, static_cast<int>(run_->pts.size()) - 2),
                        0.0, dist_to_goal, next_run_turn() * (180.0 / kPi), target_heading,
                        heading_err, 0.0);
}

// _hold_at_completion (D3): brake to a confirmed physical stop at the final waypoint, then DONE.
void RppCore::hold_at_completion(double pos_n, double pos_e, double yaw_ned, double pose_age_s,
                                 double dist_to_goal, int64_t now_ns) {
  if (!completion_stop_pending_) {
    reset_corner_pivot_state();
    completion_stop_pending_ = true;
  }
  const StopTelemetry tel = telemetry(now_ns, yaw_ned);
  const HoldOutput h = completion_hold_.step(now_ns, tel);
  if (h.stopped) {
    path_done_ = true;
    segment_state_ = SegState::Done;
    publish_zero(StateCode::Done, pose_age_s * 1000.0, dist_to_goal);
    return;
  }
  segment_state_ = SegState::CornerStop;
  last_speed_cmd_ = 0.0;
  double hv = 0.0;
  publish_brake(yaw_ned, tel, &hv);
  publish_debug(hold_row(debug_xtrack(pos_n, pos_e), 0.0, dist_to_goal, hv, dist_to_goal,
                         pose_age_s * 1000.0, false));
  publish_segment_debug(SegState::CornerStop, std::max(0, static_cast<int>(run_->pts.size()) - 2),
                        0.0, dist_to_goal, kNaN, kNaN, kNaN, 0.0);
}

// _segment_endpoint_precise_stop_tick: final-run endpoint overlay (ON by default in the prototype).
// True when this tick published a stop/correction command.
bool RppCore::precise_stop_tick(double pos_n, double pos_e, double yaw_ned, double pose_age_s,
                                double dist_to_goal, int64_t now_ns) {
  if (!params_.flag(P::segment_precise_endpoint_stop_enabled)) return false;
  if (run_idx_ + 1 < runs_.size()) return false;
  if (run_ == nullptr || run_->pts.size() < 2) return false;
  const Point a = run_->pts[run_->pts.size() - 2];
  const Point b = run_->pts[run_->pts.size() - 1];
  double un = b.n - a.n, ue = b.e - a.e;
  const double seg_len = std::hypot(un, ue);
  if (seg_len < 1e-6) return false;
  un /= seg_len;
  ue /= seg_len;

  // residual: + endpoint ahead on the final segment, - overshot. cross: the prototype's sign,
  // LEFT of the final segment positive (only |cross| feeds control; the status reports -cross).
  const double dn = b.n - pos_n;
  const double de = b.e - pos_e;
  const double residual = dn * un + de * ue;
  const double cross = (pos_n - b.n) * ue - (pos_e - b.e) * un;
  const double radial = std::hypot(dn, de);

  const double along_tol = params_.num(P::segment_endpoint_arrival_tolerance_m);
  const double cross_tol = params_.num(P::segment_endpoint_cross_tolerance_m);
  const double correction_limit = params_.num(P::segment_endpoint_max_correction_m);
  const double capture_past_m = params_.num(P::endpoint_capture_past_m);
  const double speed = measured_speed(now_ns);
  const double decel = params_.num(P::segment_endpoint_precise_decel_m_s2);
  // precise_stop.feedforward_trigger_distance(speed, decel, along_tol)
  const double sp0 = std::max(0.0, speed);
  const double floor_d = std::max(0.0, along_tol);
  const double ff = decel <= 0.0 ? floor_d : std::max(floor_d, (sp0 * sp0) / (2.0 * decel));
  const double trigger = ff + params_.num(P::segment_endpoint_trigger_margin_m);

  // A negative residual is already past the end plane, so it must engage.
  if (!segment_endpoint_stop_active_ && residual > trigger) return false;
  if (!segment_endpoint_stop_active_) {
    reset_corner_pivot_state();
    segment_endpoint_stop_active_ = true;
    endpoint_brake_hold_ = false;
    endpoint_stop_started_ = true;
    endpoint_stop_start_ns_ = now_ns;
  }

  const StopTelemetry tel = telemetry(now_ns, yaw_ned);
  const bool stopped = stop_confirm_.satisfied(now_ns, tel, sp_);
  auto finish = [&]() {
    segment_endpoint_stop_active_ = false;
    endpoint_stop_started_ = false;
    endpoint_brake_hold_ = false;
    completion_stop_pending_ = true;
    hold_at_completion(pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal, now_ns);
  };
  if (std::fabs(residual) <= along_tol && std::fabs(cross) <= cross_tol && stopped) {
    finish();
    return true;
  }
  const double max_s = params_.num(P::segment_endpoint_precise_max_s);
  const bool timed_out = max_s > 0.0 && endpoint_stop_started_ &&
                         static_cast<double>(now_ns - endpoint_stop_start_ns_) * 1e-9 >= max_s;
  if (timed_out && stopped) {
    finish();  // timeout: accept the best position, only once stopped
    return true;
  }

  segment_state_ = SegState::CornerStop;
  last_speed_cmd_ = 0.0;
  const double age_ms = pose_age_s * 1000.0;

  if (timed_out) {
    // XR-RPP-001 (BEHAVIOUR CHANGE, not in the prototype): past the timeout the correction is over.
    // The prototype kept creeping until a stop happened to be confirmed, which a rover rocking
    // through the end plane never reaches. Brake to a confirmed stop instead; the shared stop
    // confirmation then finishes on a later tick (a stale velocity confirms after its 2 s cap), so
    // the endpoint always completes and reports the miss it was left with.
    double hv = 0.0;
    publish_brake(yaw_ned, tel, &hv);
    publish_debug(hold_row(cross, 0.0, dist_to_goal, hv, dist_to_goal, age_ms, false));
    out_.cross_track_right = -cross;  // XR-RPP-005: `cross` is left-positive
    publish_segment_debug(SegState::CornerStop, std::max(0, static_cast<int>(run_->pts.size()) - 2),
                          std::max(0.0, residual), dist_to_goal, kNaN, kNaN, kNaN, 0.0);
    return true;
  }

  if (radial > correction_limit && std::fabs(cross) > cross_tol) {
    // lateral miss outside the correction envelope: brake, no aggressive diagonal chase
    double hv = 0.0;
    publish_brake(yaw_ned, tel, &hv);
    publish_debug(hold_row(cross, 0.0, dist_to_goal, hv, dist_to_goal, age_ms, false));
    out_.cross_track_right = -cross;  // XR-RPP-005: `cross` is left-positive
    return true;
  }

  // BEHAVIOUR CHANGE, not in the prototype (2026-10-10, mission 0001 run 3: 14 forward/reverse
  // reversals in 8.6 s, finished by the timeout 7 mm from the point). Inside the arrival band the
  // finish geometry is met and only the stop confirmation is missing, so the correct command is a
  // brake: the creep law below drives toward residual = 0 with a sign flip at the plane, and a
  // vehicle with speed-loop latency overshoots it and never settles.
  const bool in_finish_geometry = std::fabs(residual) <= along_tol && std::fabs(cross) <= cross_tol;
  // DERIVED — NOT FROM V1 SPEC: `endpoint_capture_past_m` (the prototype's "capture past"
  // allowance) reused as the brake-hold band beyond the arrival band, so a few millimetres of
  // coast after the brake does not abandon it; no new number.
  const bool in_hold_geometry =
      std::fabs(residual) <= along_tol + capture_past_m && std::fabs(cross) <= cross_tol;
  if (!stopped && (in_finish_geometry || (endpoint_brake_hold_ && in_hold_geometry))) {
    endpoint_brake_hold_ = true;
    double hv = 0.0;
    publish_brake(yaw_ned, tel, &hv);
    publish_debug(hold_row(cross, 0.0, dist_to_goal, hv, dist_to_goal, age_ms, false));
    out_.cross_track_right = -cross;  // XR-RPP-005: `cross` is left-positive
    publish_segment_debug(SegState::CornerStop, std::max(0, static_cast<int>(run_->pts.size()) - 2),
                          std::max(0.0, residual), dist_to_goal, kNaN, kNaN, kNaN, 0.0);
    return true;
  }
  // Released: a real overshoot (beyond the hold band), or stopped off the mark (the creep nudge
  // below takes over; the next arrival in the band brakes again).
  endpoint_brake_hold_ = false;

  const bool needs_lateral_correction = std::fabs(cross) > cross_tol && radial <= correction_limit;
  const double profile_dist = needs_lateral_correction ? radial : std::fabs(residual);
  const double creep = params_.num(P::segment_endpoint_creep_speed);
  double speed_mag;
  if (stopped && radial > std::max(along_tol, cross_tol)) {
    speed_mag = creep;
  } else {
    const double cap = std::max(speed, creep);
    // precise_stop.feedforward_brake_speed(max(0, profile_dist), decel, cap): evaluated to the
    // plane, as the prototype does (review 2026-10-10: an evaluation to the band edge aims the stop
    // short of the point; contract section 3.6, stop-position distribution). The brake inside the
    // arrival band above is what removes the rocking.
    const double rem = std::max(0.0, profile_dist);
    const double capc = std::max(0.0, cap);
    speed_mag = (rem <= 0.0 || decel <= 0.0) ? 0.0 : std::min(std::sqrt(2.0 * decel * rem), capc);
  }
  double dir_n, dir_e;
  if (radial < 1e-6 || radial > std::max(correction_limit, std::max(along_tol, cross_tol))) {
    const double sign = residual >= 0.0 ? 1.0 : -1.0;
    dir_n = sign * un;
    dir_e = sign * ue;
  } else if (needs_lateral_correction) {
    dir_n = dn / radial;
    dir_e = de / radial;
  } else {
    const double sign = residual >= 0.0 ? 1.0 : -1.0;
    dir_n = sign * un;
    dir_e = sign * ue;
  }
  const double v_n = speed_mag * dir_n;
  const double v_e = speed_mag * dir_e;
  publish_velocity(v_n, v_e);
  publish_yaw_rate(0.0);
  out_.cmd = CmdKind::Creep;  // a creep / profile speed along the final leg, not an active brake
  out_.creep_speed =
      std::copysign(std::hypot(v_n, v_e), v_n * std::cos(yaw_ned) + v_e * std::sin(yaw_ned));
  publish_debug(
      hold_row(cross, 0.0, dist_to_goal, std::hypot(v_n, v_e), dist_to_goal, age_ms, false));
  out_.cross_track_right = -cross;  // XR-RPP-005: `cross` is left-positive
  publish_segment_debug(SegState::CornerStop, std::max(0, static_cast<int>(run_->pts.size()) - 2),
                        std::max(0.0, residual), dist_to_goal, kNaN, kNaN, kNaN, 0.0);
  return true;
}

// ------------------------------------------------------------------------------------------------
// tick
// ------------------------------------------------------------------------------------------------
const TickOutput& RppCore::tick(int64_t now_ns) {
  out_ = TickOutput{};
  if (!have_last_tick_) {
    tick_dt_ = 1.0 / kControlHz;
  } else {
    tick_dt_ = std::max(0.0, std::min(0.1, ns_to_s(now_ns - last_tick_ns_)));
  }
  last_tick_ns_ = now_ns;
  have_last_tick_ = true;
  sp_ = stop_params();
  corner_fsm_.set_params(sp_);
  boundary_hold_.set_params(sp_);
  completion_hold_.set_params(sp_);
  control_loop_impl(now_ns);
  out_.yaw_ned = pose_.yaw_ned;
  return out_;
}

void RppCore::control_loop_impl(int64_t now_ns) {
  const double hw_max_v = params_.num(P::max_linear_vel);
  const double mission_v = params_.num(P::mission_speed);
  const double max_v = std::min(hw_max_v, mission_v);
  const double max_age_s = params_.num(P::pose_max_age_s);
  const bool req_rtk = params_.flag(P::require_rtk_fix);

  double pose_gap_worst;
  if (gaps_count_ > 0) {
    double mx = gaps_[0];
    for (size_t i = 1; i < gaps_count_; ++i) mx = std::max(mx, gaps_[i]);
    pose_gap_worst = std::min(mx, 0.3);
  } else {
    pose_gap_worst = 1.0 / kControlHz;
  }
  const double v_meas = vel_is_fresh(now_ns) ? std::hypot(vel_n_, vel_e_) : 0.0;
  const double jump_thr =
      std::max(params_.num(P::ekf_jump_threshold_m),
               std::max(max_v, v_meas) * std::max(pose_gap_worst, 1.0 / kControlHz) + 0.03);

  if (!have_pose_) {
    publish_zero(StateCode::Idle, kNaN);
    return;
  }

  const bool use_extrap = params_.flag(P::use_imu_extrapolation);
  const double extrap_horizon = params_.num(P::imu_max_extrap_age_s);
  const double effective_max_age = max_age_s + (use_extrap ? extrap_horizon : 0.0);

  const double pose_age_s = ns_to_s(now_ns - pose_recv_ns_);
  // RPP-004: a negative (a pose stamped after this tick) or non-finite age is not fresh.
  if (!std::isfinite(pose_age_s) || pose_age_s < 0.0 || pose_age_s > effective_max_age) {
    publish_zero(StateCode::Stale, pose_age_s * 1000);
    return;
  }

  double use_n = pose_.n, use_e = pose_.e;
  if (use_extrap && vel_.has && std::isfinite(vel_n_) && std::isfinite(vel_e_)) {
    const double vel_age_s = ns_to_s(now_ns - vel_.ns);
    if (vel_age_s >= 0.0 && vel_age_s < extrap_horizon) {
      const double dt = pose_age_s + std::max(0.0, params_.num(P::pose_latency_bias_s));
      const double d_n = vel_n_ * dt;
      const double d_e = vel_e_ * dt;
      use_n = pose_.n + d_n;
      use_e = pose_.e + d_e;
    }
  }

  // ---- P0.3 RTK gate ----
  bool rtk_ok;
  RtkReason reason = RtkReason::Ok;
  if (req_rtk) {
    rtk_ok = rtk_gate(now_ns, &reason);
  } else {
    rtk_recover_since_.has = false;
    rtk_ok = true;
  }
  out_.rtk_reason = reason;
  if (!rtk_ok) {
    publish_zero(StateCode::RtkWait, pose_age_s * 1000);
    return;
  }

  if (run_ == nullptr || run_->pts.empty()) {
    publish_zero(StateCode::Idle, pose_age_s * 1000);
    return;
  }
  if (path_done_) {
    publish_zero(StateCode::Done, pose_age_s * 1000);
    return;
  }

  double pos_n = use_n;
  double pos_e = use_e;
  const double yaw_ned = pose_.yaw_ned;
  const PathView path(run_->pts);

  // ---- P0.2 / A3 EKF / position jump ----
  const bool comp_enabled = params_.flag(P::ekf_reset_compensation);
  const double max_absorb = params_.num(P::ekf_reset_max_absorb_m);
  if (have_last_pos_) {
    const double d_n = pos_n - last_pos_n_;
    const double d_e = pos_e - last_pos_e_;
    const double jump_m = std::hypot(d_n, d_e);
    if (jump_m > jump_thr) {
      if (comp_enabled && jump_m <= max_absorb) {
        ekf_off_n_ = ekf_off_n_ + d_n;
        ekf_off_e_ = ekf_off_e_ + d_e;
        ++ekf_reset_count_;
      } else {
        last_pos_n_ = pos_n;
        last_pos_e_ = pos_e;
        have_last_pos_ = true;
        hint_.seg = 0;
        hint_.valid = false;
        publish_zero(StateCode::JumpSkip, pose_age_s * 1000);
        return;
      }
    }
  }
  last_pos_n_ = pos_n;
  last_pos_e_ = pos_e;
  have_last_pos_ = true;

  pos_n -= ekf_off_n_;
  pos_e -= ekf_off_e_;

  // ---- run-transition alignment: pivot toward the new run's first leg before tracking it ----
  if (run_alignment_hold(pos_n, pos_e, yaw_ned, pose_age_s, now_ns)) return;

  dyx3_geometry::PathProjection smooth_proj;
  if (!profile_segment_) {
    smooth_proj = dyx3_geometry::project_onto_path(Point{pos_n, pos_e}, path, hint_);
    // GEO-002 (consumer side): no valid projection, no guidance. Fail to zero.
    if (!smooth_proj.valid) {
      hint_.seg = 0;
      hint_.valid = false;
      publish_zero(StateCode::Idle, pose_age_s * 1000);
      return;
    }
    update_path_progress(smooth_proj.seg_idx, smooth_proj.t);
  }

  // ---- goal check ----
  const double min_travel = run_min_travel();
  const Point final_pt = path[path.n - 1];
  const double dist_to_goal = dist(pos_n, pos_e, final_pt.n, final_pt.e);
  if (params_.flag(P::point_hold_enabled)) {
    handoff(Handoff::PointHold);
    return;
  }
  if (run_boundary_stop_pending_) {
    hold_before_run_advance(pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal, now_ns);
    return;
  }
  // D3: once the final-waypoint stop is latched, hold it BEFORE the goal test.
  if (completion_stop_pending_) {
    hold_at_completion(pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal, now_ns);
    return;
  }
  if (profile_segment_ && run_idx_ + 1 >= runs_.size() && path_travel_m_ >= min_travel) {
    if (precise_stop_tick(pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal, now_ns)) return;
  }
  const double goal_tol = params_.num(P::xy_goal_tolerance);
  if (path_travel_m_ >= min_travel && (dist_to_goal <= goal_tol_effective(goal_tol) ||
                                       endpoint_capture_recovered(pos_n, pos_e, dist_to_goal))) {
    if (run_idx_ + 1 < runs_.size()) {
      hold_before_run_advance(pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal, now_ns);
    } else {
      hold_at_completion(pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal, now_ns);
    }
    return;
  }

  if (profile_segment_) {
    control_segment(pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal, now_ns);
    return;
  }
  control_smooth(pos_n, pos_e, yaw_ned, pose_age_s, dist_to_goal, smooth_proj, now_ns);
}

// ------------------------------------------------------------------------------------------------
// smooth profile (steps 1-8 of _control_loop_impl)
// ------------------------------------------------------------------------------------------------
void RppCore::control_smooth(double pos_n, double pos_e, double yaw_ned, double pose_age_s,
                             double dist_to_goal, dyx3_geometry::PathProjection proj,
                             int64_t /*now_ns*/) {
  const PathView path(run_->pts);
  const double max_v = std::min(params_.num(P::max_linear_vel), params_.num(P::mission_speed));
  const double min_v = params_.num(P::min_linear_vel);
  const double l_min = params_.num(P::min_lookahead_dist);
  const double l_max = params_.num(P::max_lookahead_dist);
  const double a_lat_max = params_.num(P::a_lat_max);
  const double min_curv_v = params_.num(P::regulated_linear_scaling_min_speed);
  const double approach_v = params_.num(P::min_approach_linear_velocity);
  const double p4_floor = params_.num(P::p4_zero_vel_threshold);
  const int n_preview = params_.integer(P::preview_curvature_n);
  const double max_decel = params_.num(P::max_linear_decel);
  const double approach_d =
      approach_distance(params_.num(P::approach_velocity_scaling_dist), max_v, max_decel);

  const int seg_idx = proj.seg_idx;
  const Point foot = proj.foot;
  const double signed_xtrack = proj.signed_cross;
  const bool spray_active = segment_spray_active(seg_idx);

  LookaheadParams lp{min_v,
                     max_v,
                     l_min,
                     l_max,
                     params_.num(P::lookahead_time),
                     params_.num(P::xtrack_lookahead_gain)};
  const LookaheadDistance ld = lookahead_distance(lp, last_speed_cmd_, signed_xtrack);
  double l_d = ld.l_d;
  const double l_d_raw = ld.raw;

  const double kappa_path =
      dyx3_geometry::curvature_at(path, seg_idx, params_.num(P::curvature_baseline_m));
  l_d = apply_arc_cap(
      l_d, kappa_path,
      ArcCapParams{params_.num(P::smooth_max_arc_cut_m), params_.num(P::smooth_min_arc_ld_m),
                   params_.num(P::smooth_curvature_ld_coeff)});

  LookaheadPoint lh = smooth_lookahead_point(path, seg_idx, foot, l_d);
  double dn = lh.p.n - pos_n;
  double de = lh.p.e - pos_e;
  Steering st = steering_geometry(dn, de, yaw_ned);
  if (st.degenerate) {
    lh = smooth_lookahead_point(path, seg_idx, foot, l_min);
    dn = lh.p.n - pos_n;
    de = lh.p.e - pos_e;
    st = steering_geometry(dn, de, yaw_ned);
    if (st.degenerate) {
      publish_zero(StateCode::Idle, pose_age_s * 1000, dist_to_goal);
      return;
    }
  }
  const double kappa = st.kappa;
  const double theta_e = st.theta_e;
  const double l_actual = st.l_actual;

  int n_eff = n_preview;
  const double preview_dist_m = params_.num(P::preview_curvature_distance_m);
  if (preview_dist_m > 0.0 && l_d > 1e-9) {
    // XR-RPP-009: bounded per-tick work. The distance / lookahead ratio is computed in double and
    // capped before the int conversion (a large distance over a small lookahead overflowed).
    const double want =
        std::min(std::ceil(preview_dist_m / l_d), static_cast<double>(kMaxPreviewN));
    n_eff = std::max(n_preview, static_cast<int>(want));
  }
  n_eff = std::min(n_eff, kMaxPreviewN);
  const double kappa_speed =
      n_eff > 1 ? dyx3_geometry::max_preview_curvature(path, seg_idx, foot, l_d, n_eff)
                : std::fabs(kappa);

  const LateralLimit lat = lateral_speed_limit(kappa_speed, a_lat_max, min_curv_v, max_v);
  double speed = lat.speed;
  const double v_lat_limit = lat.v_lat_limit;

  StateCode state_code = StateCode::Tracking;
  const ApproachResult ar = smooth_approach_scaling(
      speed, run_->closed, run_->length, path_travel_m_, dist_to_goal, approach_d, approach_v);
  speed = ar.speed;
  if (ar.approach_active) state_code = StateCode::Approach;

  const double speed_raw = speed;
  const double speed_before_accel = speed_raw;
  const double max_accel = params_.num(P::max_linear_accel);
  const double kappa_now = std::max(std::fabs(kappa), std::fabs(kappa_speed));
  const double accel_scale = alignment_accel_scale(
      theta_e, kappa_now, params_.num(P::accel_gate_heading_full_deg),
      params_.num(P::accel_gate_heading_none_deg), params_.num(P::accel_gate_curv_full),
      params_.num(P::accel_gate_curv_none));
  kappa_hard_latched_ =
      update_kappa_hard_latch(kappa_hard_latched_, kappa_now, params_.num(P::kappa_hard_enter),
                              params_.num(P::kappa_hard_exit));
  const bool approach_active = state_code == StateCode::Approach;
  const SlewResult sl = apply_smooth_speed_slew(
      speed_raw, last_speed_cmd_, tick_dt_, kappa_hard_latched_,
      params_.num(P::speed_cmd_decel_m_s2), max_accel, accel_scale, approach_active, p4_floor);
  speed = sl.speed;
  int speed_mode = sl.mode;

  speed = apply_p4_floor(speed, speed_before_accel, last_speed_cmd_, p4_floor, &speed_mode);
  last_speed_cmd_ = speed;

  double yaw_rate_body;
  if (params_.flag(P::use_feedforward_yaw_rate)) {
    const double yaw_rate_ff = kappa * speed;
    const double yaw_rate_fb = params_.num(P::yaw_rate_feedback_gain) * theta_e;
    yaw_rate_body = yaw_rate_ff + yaw_rate_fb;
    const double max_yr = params_.num(P::max_yaw_rate_body);
    if (max_yr > 0.0) yaw_rate_body = clampd(yaw_rate_body, -max_yr, max_yr);
  } else {
    yaw_rate_body = 0.0;
  }

  const double unit_n = l_actual > 1e-9 ? dn / l_actual : 0.0;
  const double unit_e = l_actual > 1e-9 ? de / l_actual : 0.0;
  double v_n = speed * unit_n;
  double v_e = speed * unit_e;

  const double lat_gain = params_.num(P::smooth_lateral_gain);
  if (lat_gain > 0.0 && speed > 1e-6) {
    const double max_corr = params_.num(P::smooth_lateral_max_deg) * (kPi / 180.0);
    const double delta = clampd(-lat_gain * signed_xtrack, -max_corr, max_corr);
    const double bearing = std::atan2(v_e, v_n) + delta;
    v_n = speed * std::cos(bearing);
    v_e = speed * std::sin(bearing);
  }

  clamp_to_forward_cone(v_n, v_e, yaw_ned, speed);

  const double speed_mag = std::hypot(v_n, v_e);
  double yaw_target_ned;
  if (speed_mag > 0.01) {
    yaw_target_ned = std::atan2(v_e, v_n);
  } else {
    yaw_target_ned = last_yaw_cmd_;
  }
  last_yaw_cmd_ = yaw_target_ned;
  out_.track_heading_ned = yaw_target_ned;

  publish_velocity(v_n, v_e);
  publish_yaw_rate(yaw_rate_body);

  DebugRow row;
  row.cross_track = signed_xtrack;
  row.heading_err = theta_e;
  row.lookahead = l_actual;
  row.speed = speed;
  row.kappa = kappa;
  row.dist_goal = dist_to_goal;
  row.pose_age_ms = pose_age_s * 1000;
  row.state = static_cast<int>(state_code);
  row.l_d_raw = l_d_raw;
  row.kappa_speed = kappa_speed;
  row.yaw_rate = yaw_rate_body;
  row.spray_active = spray_active;
  row.speed_raw = speed_raw;
  row.v_lat_limit = v_lat_limit;
  row.accel_scale = accel_scale;
  row.speed_mode = speed_mode;
  publish_debug(row);

  publish_segment_debug(SegState::TrackSegment, seg_idx, dist_to_goal, dist_to_goal, 0.0,
                        yaw_target_ned, theta_e, yaw_rate_body);
}

// ------------------------------------------------------------------------------------------------
// segment profile (_control_segment_profile; the corner stop/pivot and endpoint machines are handed
// off)
// ------------------------------------------------------------------------------------------------
void RppCore::control_segment(double pos_n, double pos_e, double yaw_ned, double pose_age_s,
                              double dist_to_goal, int64_t now_ns) {
  const PathView path(run_->pts);
  const int n_pts = static_cast<int>(path.n);

  for (;;) {  // the prototype recurses after crossing a collinear vertex; same state, so a loop
    if (n_pts < 2) {
      if (advance_run()) {
        out_.velocity_published = false;  // the prototype returns without publishing this tick
        return;
      }
      segment_state_ = SegState::Done;
      path_done_ = true;
      publish_zero(StateCode::Done, pose_age_s * 1000.0, dist_to_goal);
      publish_segment_debug(segment_state_, 0, kNaN, dist_to_goal, kNaN, kNaN, kNaN, 0.0);
      return;
    }

    while (segment_idx_ < n_pts - 2) {
      const Point a = path[static_cast<size_t>(segment_idx_)];
      const Point b = path[static_cast<size_t>(segment_idx_) + 1];
      if (dist(a.n, a.e, b.n, b.e) >= 1e-6) break;
      ++segment_idx_;
    }
    segment_idx_ = std::max(0, std::min(segment_idx_, n_pts - 2));
    const int seg_idx = segment_idx_;
    const Point a = path[static_cast<size_t>(seg_idx)];
    const Point b = path[static_cast<size_t>(seg_idx) + 1];
    const double seg_len = dist(a.n, a.e, b.n, b.e);
    if (seg_len < 1e-6) {
      publish_zero(StateCode::Idle, pose_age_s * 1000.0, dist_to_goal);
      return;
    }

    const bool final_segment = seg_idx >= n_pts - 2;
    const auto sp = dyx3_geometry::project_onto_segment(Point{pos_n, pos_e}, path, seg_idx);
    if (!sp.valid) {  // GEO-002 (consumer side): no valid projection, no guidance
      publish_zero(StateCode::Idle, pose_age_s * 1000.0, dist_to_goal);
      return;
    }
    const double signed_xtrack = sp.signed_cross;
    const double dist_to_end_along = sp.dist_to_end_along;
    update_path_progress(seg_idx, sp.t);
    const double dist_to_corner = dist(pos_n, pos_e, b.n, b.e);
    const double corner_angle = segment_angle_deg(path, seg_idx);
    const bool spray_active = segment_spray_active(seg_idx);

    const double goal_tol = params_.num(P::xy_goal_tolerance);
    const double goal_tol_eff = goal_tol_effective(goal_tol);
    const double min_travel = run_min_travel();
    if (final_segment && path_travel_m_ >= min_travel) {
      if (precise_stop_tick(pos_n, pos_e, yaw_ned, pose_age_s, dist_to_corner, now_ns)) return;
    }
    if (final_segment && path_travel_m_ >= min_travel &&
        (dist_to_corner <= goal_tol_eff ||
         endpoint_capture_recovered(pos_n, pos_e, dist_to_corner))) {
      // stop before switching across a real heading change; the final run goes through the single
      // completion handler
      if (run_idx_ + 1 < runs_.size()) {
        hold_before_run_advance(pos_n, pos_e, yaw_ned, pose_age_s, dist_to_corner, now_ns);
      } else {
        hold_at_completion(pos_n, pos_e, yaw_ned, pose_age_s, dist_to_corner, now_ns);
      }
      return;
    }

    const double acceptance = params_.num(P::segment_corner_acceptance_radius);
    const double yaw_gain = params_.num(P::segment_yaw_rate_gain);
    const bool use_ff_yaw_rate = params_.flag(P::use_feedforward_yaw_rate);
    const double max_yr = params_.num(P::max_yaw_rate_body);

    if (!final_segment && dist_to_corner <= acceptance) {
      const double path_corner_deg = std::fabs(segment_angle_deg(path, seg_idx));
      const Point c = path[static_cast<size_t>(seg_idx) + 2];
      const double leg_heading = std::atan2(c.e - b.e, c.n - b.n);
      const double target_heading = pivot_intercept_heading(
          Point{pos_n, pos_e}, b, c, leg_heading, params_.flag(P::pivot_to_intercept_enabled),
          params_.num(P::pivot_intercept_dist_m));
      const double heading_err = dyx3_geometry::angle_wrap(target_heading - yaw_ned);
      const StopTelemetry tel = telemetry(now_ns, yaw_ned);
      CornerInput in;
      in.now_ns = now_ns;
      in.heading_err_rad = heading_err;
      in.corner_deg = path_corner_deg;
      in.tel = tel;
      const CornerOutput co = corner_fsm_.step(in);
      const double age_ms = pose_age_s * 1000.0;
      if (co.action == CornerAction::Advance) {
        // tangent junctions keep momentum; a settled hard corner starts the next leg from rest
        ++segment_idx_;
        segment_state_ = SegState::TrackSegment;
        if (co.zero_speed_memory) last_speed_cmd_ = 0.0;
        publish_segment_debug(segment_state_, segment_idx_, kNaN, dist_to_corner, corner_angle,
                              target_heading, heading_err, 0.0);
        continue;
      }
      if (co.action == CornerAction::SettleBrake) {
        last_speed_cmd_ = 0.0;
        double hv = 0.0;
        publish_brake(yaw_ned, tel, &hv);
        publish_debug(hold_row(signed_xtrack, heading_err, dist_to_corner, hv, dist_to_goal, age_ms,
                               spray_active));
        publish_segment_debug(SegState::CornerAlign, seg_idx, dist_to_end_along, dist_to_corner,
                              corner_angle, target_heading, heading_err, 0.0);
        return;
      }
      if (co.action == CornerAction::Brake) {
        segment_state_ = SegState::CornerStop;
        last_speed_cmd_ = 0.0;
        double hv = 0.0;
        publish_brake(yaw_ned, tel, &hv);
        publish_debug(hold_row(signed_xtrack, heading_err, dist_to_corner, hv, dist_to_goal, age_ms,
                               spray_active));
        publish_segment_debug(segment_state_, seg_idx, dist_to_end_along, dist_to_corner,
                              corner_angle, target_heading, heading_err, 0.0);
        return;
      }
      // Pivot: a small vector at the exit heading, kept inside the forward cone
      segment_state_ = SegState::CornerAlign;
      const double corner_speed = std::max(0.05, params_.num(P::segment_min_corner_speed));
      const double step = clampd(heading_err, -kMaxBearingOffsetRad, kMaxBearingOffsetRad);
      const double cmd_bearing = yaw_ned + step;
      last_speed_cmd_ = corner_speed;
      publish_velocity(corner_speed * std::cos(cmd_bearing), corner_speed * std::sin(cmd_bearing));
      publish_yaw_rate(0.0);
      out_.cmd = CmdKind::Pivot;
      out_.pivot_heading_err = heading_err;
      out_.pivot_speed_memory = corner_speed;
      out_.pivot_timed_out = co.pivot_timed_out;
      publish_debug(hold_row(signed_xtrack, heading_err, dist_to_corner, corner_speed, dist_to_goal,
                             age_ms, spray_active));
      publish_segment_debug(segment_state_, seg_idx, dist_to_end_along, dist_to_corner,
                            corner_angle, target_heading, heading_err, 0.0);
      return;
    }

    const double max_v = std::min(params_.num(P::max_linear_vel), params_.num(P::mission_speed));
    LookaheadParams lp{params_.num(P::min_linear_vel),     max_v,
                       params_.num(P::min_lookahead_dist), params_.num(P::max_lookahead_dist),
                       params_.num(P::lookahead_time),     params_.num(P::xtrack_lookahead_gain)};
    const LookaheadDistance ld = lookahead_distance(lp, last_speed_cmd_, signed_xtrack);
    const double l_d = ld.l_d;
    const double l_d_raw = ld.raw;

    Point lh = segment_lookahead_point(path, seg_idx, sp.foot, l_d,
                                       params_.num(P::segment_lookahead_cross_collinear_deg),
                                       params_.flag(P::segment_endpoint_lookahead_extend),
                                       params_.flag(P::segment_corner_lookahead_extend));
    double dn = lh.n - pos_n;
    double de = lh.e - pos_e;
    Steering st = steering_geometry(dn, de, yaw_ned);
    if (st.degenerate) {
      dn = b.n - pos_n;
      de = b.e - pos_e;
      st = steering_geometry(dn, de, yaw_ned);
      if (st.degenerate) {
        publish_zero(StateCode::Idle, pose_age_s * 1000.0, dist_to_goal);
        return;
      }
    }
    const double l_actual = st.l_actual;
    const double theta_e = st.theta_e;
    double speed = max_v;
    const double slowdown = params_.num(P::segment_slowdown_dist);
    const double min_corner_speed = params_.num(P::segment_min_corner_speed);
    segment_state_ = SegState::TrackSegment;
    const double corner_threshold_deg = params_.num(P::segment_corner_threshold_deg);
    if (!final_segment && slowdown > 1e-6 && dist_to_corner < slowdown &&
        std::isfinite(corner_angle) && std::fabs(corner_angle) >= corner_threshold_deg) {
      const double scale = clampd(dist_to_corner / slowdown, 0.0, 1.0);
      speed = std::max(min_corner_speed, max_v * scale);
      segment_state_ = SegState::PreCornerSlowdown;
    }

    const double max_decel = params_.num(P::max_linear_decel);
    const double approach_v = params_.num(P::segment_endpoint_approach_speed);
    const double approach_d = std::max(params_.num(P::approach_velocity_scaling_dist),
                                       (max_v * max_v) / (2.0 * max_decel) + 0.10);
    double approach_ref = final_segment ? dist_to_corner : std::numeric_limits<double>::infinity();
    if (params_.flag(P::endpoint_approach_run_remaining)) {
      const std::optional<double> remaining_along = run_remaining_along();
      if (remaining_along.has_value()) approach_ref = std::min(approach_ref, *remaining_along);
    }
    if (approach_ref < approach_d) {
      const double scale = clampd(approach_ref / approach_d, 0.0, 1.0);
      speed = std::min(speed, std::max(approach_v, max_v * scale));
      const double runout_min = params_.num(P::transit_runout_min_speed_m_s);
      if (runout_min > 0.0 && run_tail_transit_m_ > 0.0 && 0.0 < speed && speed < runout_min &&
          dist_to_corner > goal_tol_eff) {
        speed = runout_min;
      }
      segment_state_ = SegState::PreCornerSlowdown;
    }

    const double max_accel = params_.num(P::max_linear_accel);
    const double speed_before_accel = speed;
    if (max_accel > 0.0 && speed > last_speed_cmd_) {
      const double accel_scale = alignment_accel_scale(
          theta_e, 0.0, params_.num(P::accel_gate_heading_full_deg),
          params_.num(P::accel_gate_heading_none_deg), params_.num(P::accel_gate_curv_full),
          params_.num(P::accel_gate_curv_none));
      speed = std::min(speed, last_speed_cmd_ + max_accel * accel_scale * tick_dt_);
    }

    const double p4_floor = params_.num(P::p4_zero_vel_threshold);
    if (speed < p4_floor && speed_before_accel < p4_floor && last_speed_cmd_ > 0.0) speed = 0.0;
    const double latch_ref = (final_segment || segment_state_ == SegState::PreCornerSlowdown)
                                 ? dist_to_corner
                                 : std::numeric_limits<double>::infinity();
    speed = stop_latch_filter(speed, latch_ref, now_ns);
    last_speed_cmd_ = speed;

    // B3: when segment_command_mode=rate, RPP owns the heading-error -> yaw-rate law even if the
    // legacy feed-forward toggle is off. Default command mode remains heading, so landing B3 does
    // not change the shipped steering behavior.
    const bool segment_rate_command = params_.str(P::segment_command_mode) == "rate";
    double yaw_rate_body = (use_ff_yaw_rate || segment_rate_command) ? yaw_gain * theta_e : 0.0;
    if (max_yr > 0.0) yaw_rate_body = clampd(yaw_rate_body, -max_yr, max_yr);

    const double unit_n = dn / l_actual;
    const double unit_e = de / l_actual;
    double v_n = speed * unit_n;
    double v_e = speed * unit_e;
    clamp_to_forward_cone(v_n, v_e, yaw_ned, speed);
    const double speed_mag = std::hypot(v_n, v_e);
    if (speed_mag > 0.01) last_yaw_cmd_ = std::atan2(v_e, v_n);
    out_.track_heading_ned = last_yaw_cmd_;

    publish_velocity(v_n, v_e);
    publish_yaw_rate(yaw_rate_body);
    DebugRow row;
    row.cross_track = signed_xtrack;
    row.heading_err = theta_e;
    row.lookahead = l_actual;
    row.speed = speed;
    row.kappa = 0.0;
    row.dist_goal = dist_to_goal;
    row.pose_age_ms = pose_age_s * 1000.0;
    row.state = static_cast<int>(StateCode::Tracking);
    row.l_d_raw = l_d_raw;
    row.kappa_speed = 0.0;
    row.yaw_rate = yaw_rate_body;
    row.spray_active = spray_active;
    publish_debug(row);
    publish_segment_debug(segment_state_, seg_idx, dist_to_end_along, dist_to_corner, corner_angle,
                          std::atan2(b.e - a.e, b.n - a.n), theta_e, yaw_rate_body);
    return;
  }
}

void RppCore::pause() {
  last_speed_cmd_ = 0.0;
  kappa_hard_latched_ = false;
  reset_corner_pivot_state();
  // XR-RPP-008: a resume is a fresh start of the same run from wherever the rover came to rest.
  // - jump guard: the rover may coast while paused (no command), so the first pose after a resume
  //   is not a jump. Without this the coast was a JumpSkip (a STOP tick and a lost hint) or, with
  //   ekf_reset_compensation, a PERMANENT EKF offset equal to the coast.
  have_last_pos_ = false;
  // - tick period: the first tick after the pause uses the nominal period, not the pause length
  //   (clamped to 0.1 s) for the speed slew.
  have_last_tick_ = false;
  // - projection hint: the coast can leave the hint window; search the whole open run once, as
  //   after a JumpSkip. A closed run keeps its hint: a full scan near the closure point can tie
  //   with segment 0 or the last segment, and the windowed search follows a coast within a few
  //   ticks.
  if (run_ != nullptr && !run_->closed) {
    hint_.seg = 0;
    hint_.valid = false;
  }
  // - endpoint precise stop: its timeout counted through the pause. Disengage it; it re-engages
  //   from the trigger test with a fresh start time.
  segment_endpoint_stop_active_ = false;
  endpoint_stop_started_ = false;
  endpoint_brake_hold_ = false;
  // - stop latch: re-evaluated from the rest position.
  stop_latched_ = false;
}

CoreState RppCore::snapshot() const {
  CoreState s{};
  s.last_speed_cmd = last_speed_cmd_;
  s.last_yaw_cmd = last_yaw_cmd_;
  s.path_travel_m = path_travel_m_;
  s.tick_dt = tick_dt_;
  s.segment_idx = segment_idx_;
  s.run_idx = static_cast<int>(run_idx_);
  s.hint_seg = hint_.seg;
  s.hint_valid = hint_.valid;
  s.kappa_hard_latched = kappa_hard_latched_;
  s.stop_latched = stop_latched_;
  s.path_done = path_done_;
  s.run_align_pending = run_align_pending_;
  s.ekf_offset_n = ekf_off_n_;
  s.ekf_offset_e = ekf_off_e_;
  s.ekf_reset_count = ekf_reset_count_;
  s.have_last_pos = have_last_pos_;
  s.last_pos_n = last_pos_n_;
  s.last_pos_e = last_pos_e_;
  s.segment_state = static_cast<int>(segment_state_);
  s.rtk_recovering = rtk_recover_since_.has;
  s.run_boundary_stop_pending = run_boundary_stop_pending_;
  s.completion_stop_pending = completion_stop_pending_;
  s.endpoint_stop_active = segment_endpoint_stop_active_;
  s.corner_stop_complete = corner_fsm_.stop_complete();
  s.run_align_turn_rad = run_align_turn_rad_;
  return s;
}

}  // namespace dyx3_rpp
