#include "dyx3_spray/spray_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace dyx3_spray {

namespace {
// DERIVED — NOT FROM V1 SPEC: the E-stop state is consumed with the contract's rule "absence is
// asserted"; 0.5 s matches the other 0.5 s freshness bounds in the guard (EmergencyStopState is
// published at 10 Hz).
constexpr double kEstopMaxAgeS = 0.5;
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

SprayController::SprayController(const ParamSet* params) : p_(params) {}

void SprayController::load_path(std::shared_ptr<const PathModel> model) {
  model_ = std::move(model);
  // B5: any new path resets the "run has started" evidence; the next projection must acquire
  // globally.
  tracking_seen_ = false;
  heading_entry_hold_ = true;
  dstate_ = DecisionState{};
  xtrack_trip_s_.reset();
  last_decision_.reset();
  fsm_.note_event_reset(last_tick_s_);
}

void SprayController::note_vehicle(const VehicleSnapshot& v, double now_s) {
  veh_ = v;
  have_veh_ = true;
  veh_recv_s_ = now_s;
}

void SprayController::note_rtk(const RtkSnapshot& r, double now_s) {
  rtk_ = r;
  have_rtk_ = true;
  rtk_recv_s_ = now_s;
}

void SprayController::note_rpp(uint8_t state, uint32_t mission_id, uint32_t run_index,
                               double heading_error_rad, double path_travel_m,
                               bool heading_evidence_valid, double now_s) {
  if (mission_id != rpp_mission_id_ || run_index != rpp_run_index_) heading_entry_hold_ = true;
  rpp_known_ = true;
  rpp_state_ = state;
  rpp_mission_id_ = mission_id;
  rpp_run_index_ = run_index;
  rpp_heading_error_rad_ = heading_error_rad;
  rpp_path_travel_m_ = path_travel_m;
  rpp_heading_evidence_valid_ =
      heading_evidence_valid && std::isfinite(heading_error_rad) && std::isfinite(path_travel_m);
  rpp_recv_s_ = now_s;
  // Tracking evidence (B5) counts only for the mission that is RUNNING now (review C1): it was a
  // permanent latch, cleared only on path load.
  if (static_cast<RppState>(state) == RppState::Tracking && mission_running_ &&
      mission_id == mission_id_)
    tracking_seen_ = true;
  pivot_gate_.note_state(static_cast<RppState>(state) == RppState::Pivoting, now_s);
}

void SprayController::set_mission(bool running, uint32_t mission_id) {
  if (!running || mission_id != mission_id_) {
    tracking_seen_ = false;
    heading_entry_hold_ = true;
  }
  mission_running_ = running;
  mission_id_ = mission_id;
}

GateResult SprayController::ownership(double now_s) const {
  OwnershipInputs o;
  o.mission_running = mission_running_;
  o.mission_id = mission_id_;
  o.rpp_known = rpp_known_;
  o.rpp_age_s = now_s - rpp_recv_s_;
  o.rpp_timeout_s = p_->num(P::rpp_timeout_s);
  o.rpp_state = rpp_state_;
  o.rpp_mission_id = rpp_mission_id_;
  return ownership_status(o);
}

void SprayController::note_estop(bool asserted, double now_s) {
  estop_known_ = true;
  estop_asserted_ = asserted;
  estop_recv_s_ = now_s;
}

void SprayController::note_watchdog(bool alive, bool off_authority_ready, double now_s) {
  wd_known_ = true;
  wd_alive_ = alive;
  wd_ready_ = alive && off_authority_ready;
  wd_recv_s_ = now_s;
}

bool SprayController::vehicle_fresh(double now_s) const {
  return have_veh_ && (now_s - veh_recv_s_) <= std::max(0.0, p_->num(P::pose_timeout_s));
}

bool SprayController::pose_fresh(double now_s) const {
  return vehicle_fresh(now_s) && veh_.position_valid && veh_.attitude_valid;
}

bool SprayController::velocity_fresh(double now_s) const {
  return have_veh_ && (now_s - veh_recv_s_) <= std::max(0.0, p_->num(P::velocity_timeout_s)) &&
         veh_.velocity_valid;
}

bool SprayController::watchdog_ok(std::string* reason, double now_s) const {
  if (!p_->flag(P::spray_watchdog_required)) return true;
  if (!wd_known_) {
    if (reason) *reason = "spray safety watchdog heartbeat missing";
    return false;
  }
  const double timeout = std::max(0.1, p_->num(P::spray_watchdog_timeout_s));
  const double age = now_s - wd_recv_s_;
  if (age > timeout) {
    if (reason) {
      char b[96];
      std::snprintf(b, sizeof b, "spray safety watchdog stale (%.2fs)", age);
      *reason = b;
    }
    return false;
  }
  if (!wd_ready_) {
    if (reason) *reason = "spray safety watchdog has not confirmed OFF authority";
    return false;
  }
  return true;
}

bool SprayController::safety_allows_on(double now_s) const {
  if (!p_->flag(P::spray_enabled)) return false;
  if (!(estop_known_ && !estop_asserted_ && (now_s - estop_recv_s_) <= kEstopMaxAgeS)) return false;
  if (!(vehicle_fresh(now_s) && veh_.armed)) return false;
  if (!watchdog_ok(nullptr, now_s)) return false;
  if (manual_active_)
    return true;  // bench: armed is sufficient, OFFBOARD is an auto-spray constraint
  if (!ownership(now_s).ok) return false;
  if (p_->flag(P::require_offboard) && !veh_.offboard) return false;
  return true;
}

std::pair<bool, std::string> SprayController::fsm_safety_ok(double now_s) const {
  if (!(estop_known_ && !estop_asserted_ && (now_s - estop_recv_s_) <= kEstopMaxAgeS)) {
    return {false, "emergency stop asserted or unknown"};
  }
  if (!(vehicle_fresh(now_s) && veh_.armed)) return {false, "disarmed"};
  std::string why;
  if (!watchdog_ok(&why, now_s)) return {false, why};
  if (manual_active_) return {true, ""};
  if (last_decision_) return {last_decision_->safety_ok, last_decision_->safety_reason};
  return {false, "distance-aware safety not yet evaluated"};
}

DecisionParams SprayController::decision_params() const {
  DecisionParams d;
  d.solenoid_open_delay_s = std::max(0.0, p_->num(P::solenoid_open_delay_s));
  d.solenoid_close_delay_s = std::max(0.0, p_->num(P::solenoid_close_delay_s));
  d.on_overspray_margin_m = std::max(0.0, p_->num(P::on_overspray_margin_m));
  d.off_overspray_margin_m = std::max(0.0, p_->num(P::off_overspray_margin_m));
  d.max_xtrack_error_m = std::max(0.0, p_->num(P::max_xtrack_error_m));
  d.max_xtrack_from_mission = false;
  d.xtrack_trip_error_m = std::max(0.0, p_->num(P::xtrack_trip_error_m));
  d.xtrack_gate_min_off_s = std::max(0.0, p_->num(P::xtrack_gate_min_off_s));
  d.terminal_off_epsilon_m = std::max(0.0, p_->num(P::terminal_off_epsilon_m));
  d.terminal_off_speed_mps = std::max(0.0, p_->num(P::terminal_off_speed_mps));
  d.projection_window_back_m = p_->num(P::projection_window_back_m);
  d.projection_window_fwd_m = p_->num(P::projection_window_fwd_m);
  d.projection_reacquire_dist_m = p_->num(P::projection_reacquire_dist_m);
  d.projection_direction_gate_cos = direction_gate_cos(p_->num(P::projection_direction_gate_deg));
  return d;
}

void SprayController::update_flow(double speed, double dt) {
  const bool commanded = fsm_.commanded();
  bool rising = commanded && !prev_commanded_;
  prev_commanded_ = commanded;
  const bool enabled = p_->flag(P::flow_modulation_enabled) &&
                       p_->str(P::actuator_backend) == "mavlink_actuator" && !manual_active_;
  if (!enabled) {
    flow_.reset();
    commanded_flow_.reset();
    return;
  }
  if (!flow_) {
    flow_ = std::make_unique<FlowModulator>(p_->num(P::min_flow_value), p_->num(P::on_value),
                                            p_->num(P::rated_marking_speed_mps),
                                            p_->num(P::max_flow_slew_per_s));
    rising = true;  // seed the slew filter to the floor on first build
  }
  if (rising) flow_->reset();
  if (!commanded) {
    commanded_flow_.reset();
    return;
  }
  commanded_flow_ = flow_->update(speed, dt);
}

ManualResult SprayController::set_manual(bool on, double now_s) {
  if (!on) {
    manual_active_ = false;
    return ManualResult::Ok;
  }
  if (!p_->flag(P::spray_enabled)) {
    manual_active_ = false;
    return ManualResult::Disabled;
  }
  if (!(vehicle_fresh(now_s) && veh_.armed)) {
    manual_active_ = false;
    return ManualResult::Disarmed;
  }
  if (!watchdog_ok(nullptr, now_s)) {
    manual_active_ = false;
    return ManualResult::WatchdogNotReady;
  }
  manual_active_ = true;
  manual_deadline_s_ = now_s + std::max(0.5, p_->num(P::manual_override_timeout_s));
  return ManualResult::Ok;
}

std::optional<SprayCommand> SprayController::drive_fsm(double now_s) {
  const auto [sok, why] = fsm_safety_ok(now_s);
  (void)why;
  return fsm_.tick(effective_desired(), sok, p_->flag(P::spray_enabled), now_s);
}

std::optional<SprayCommand> SprayController::tick(double now_s) {
  // Manual override: hard expiry, and fail-safes outrank it.
  if (manual_active_ && (now_s >= manual_deadline_s_ || !safety_allows_on(now_s)))
    manual_active_ = false;

  const bool pf = pose_fresh(now_s);
  const bool vf = velocity_fresh(now_s);
  const double speed = vf ? std::hypot(veh_.vel_n_mps, veh_.vel_e_mps) : 0.0;
  std::optional<double> nn, ne;
  if (pf) {
    double n, e;
    nozzle_position_ned(veh_.north_m, veh_.east_m, veh_.heading_rad,
                        p_->num(P::nozzle_forward_offset_m), p_->num(P::nozzle_lateral_offset_m),
                        &n, &e);
    nn = n;
    ne = e;
  }

  RtkGateConfig rc;
  rc.require_rtk_fix = p_->flag(P::spray_require_rtk_fix);
  rc.min_fix_type = p_->integer(P::spray_min_fix_type);
  rc.max_h_acc_m = p_->num(P::spray_max_hrms_m);
  rc.require_accuracy = p_->flag(P::spray_require_accuracy);
  rc.fix_timeout_s = p_->num(P::gps_fix_timeout_s);
  rc.recover_hold_s = p_->num(P::gps_recover_hold_s);
  std::optional<double> rtk_age;
  if (have_rtk_) rtk_age = now_s - rtk_recv_s_;
  const GateResult rtk = rtk_gate_.evaluate(rc, rtk_.fix_type, rtk_.h_acc_m, rtk_age, now_s);

  GateInputs gi;
  gi.ownership = ownership(now_s);
  const double rpp_age = now_s - rpp_recv_s_;
  const bool fresh_heading =
      rpp_heading_evidence_valid_ && rpp_age >= 0.0 && rpp_age <= p_->num(P::rpp_timeout_s);
  const double heading_deg =
      fresh_heading ? std::fabs(rpp_heading_error_rad_) * (180.0 / 3.14159265358979323846) : 0.0;
  const double cut_deg = p_->num(P::spray_heading_cut_deg);
  gi.heading_evidence = {
      fresh_heading && !(cut_deg > 0.0 && heading_deg >= cut_deg),
      !fresh_heading ? "rpp heading evidence stale or unavailable" : "heading exceeds spray cut"};
  gi.armed = vehicle_fresh(now_s) && veh_.armed;
  gi.offboard = vehicle_fresh(now_s) && veh_.offboard;
  gi.require_offboard = p_->flag(P::require_offboard);
  gi.path_loaded = static_cast<bool>(model_);
  gi.pose_fresh = pf;
  gi.velocity_fresh = vf;
  gi.rtk = rtk;
  gi.tracking_seen = tracking_seen_;
  gi.pivoting = pivot_gate_.active(p_->flag(P::spray_off_during_pivot),
                                   p_->num(P::segment_state_timeout_s), now_s);
  gi.estop_clear = estop_known_ && !estop_asserted_ && (now_s - estop_recv_s_) <= kEstopMaxAgeS;
  const GateResult safety = auto_safety_status(gi);

  const double dt = have_last_tick_ ? std::max(0.0, now_s - last_tick_s_) : 0.0;
  have_last_tick_ = true;
  last_tick_s_ = now_s;
  update_flow(speed, dt);

  DecisionInput in;
  in.model = model_.get();
  in.nozzle_n = nn;
  in.nozzle_e = ne;
  in.speed_mps = speed;
  in.yaw_rad = pf ? veh_.heading_rad : 0.0;
  in.safety_ok = safety.ok;
  in.safety_reason = safety.reason;
  dstate_.xtrack_tripped_elapsed_s = xtrack_trip_s_ ? (now_s - *xtrack_trip_s_) : 1e300;
  Decision d = make_decision(in, decision_params(), dstate_);
  if (!manual_active_ && d.geometry_desired && d.safety_ok) {
    const double entry_deg = p_->num(P::spray_entry_max_heading_deg);
    const double release_travel = p_->num(P::spray_entry_release_travel_m);
    if (heading_entry_hold_) {
      if (entry_deg <= 0.0 || heading_deg <= entry_deg || rpp_path_travel_m_ >= release_travel)
        heading_entry_hold_ = false;
      else
        d.desired = false;
    }
  }
  if (d.projection)
    dstate_.prev_projection_s =
        d.projection->s;  // carry the station so the next search stays on this leg
  if (d.xtrack_tripped && !dstate_.xtrack_tripped) xtrack_trip_s_ = now_s;
  dstate_.xtrack_tripped = d.xtrack_tripped;
  last_event_ = d.event;
  last_decision_ = std::move(d);

  // Debounce: N consecutive identical samples before the debounced desire follows.
  desired_raw_ = last_decision_->desired;
  if (!have_candidate_ || candidate_ != desired_raw_) {
    candidate_ = desired_raw_;
    candidate_count_ = 1;
    have_candidate_ = true;
  } else {
    ++candidate_count_;
  }
  const int deb = std::max(0, p_->integer(P::debounce_samples));
  if (candidate_count_ >= std::max(1, deb)) desired_debounced_ = candidate_;

  return drive_fsm(now_s);
}

std::optional<SprayCommand> SprayController::on_ack(uint32_t seq, bool success, double now_s) {
  return fsm_.on_ack(seq, success, now_s);
}

std::optional<SprayCommand> SprayController::shutdown(double now_s) {
  desired_raw_ = false;
  desired_debounced_ = false;
  have_candidate_ = false;
  candidate_count_ = 0;
  manual_active_ = false;
  shut_down_ = true;
  fsm_.note_event_reset(now_s);
  return fsm_.tick(false, false, false, now_s);
}

ActuatorWire SprayController::wire_for(bool on) const {
  ActuatorWire w;
  w.backend =
      p_->str(P::actuator_backend) == "mavlink_actuator" ? kBackendActuator : kBackendServoPwm;
  w.on = on;
  w.actuator_set_index = p_->integer(P::actuator_set_index);
  w.servo_instance = p_->integer(P::servo_instance);
  if (on) {
    w.value = commanded_flow_ ? *commanded_flow_ : p_->num(P::on_value);
    w.pwm_us = p_->integer(P::on_pwm_us);
  } else {
    w.value = p_->num(P::off_value);
    w.pwm_us = p_->integer(P::off_pwm_us);
  }
  return w;
}

bool SprayController::reassert_due(double now_s) const {
  return !shut_down_ && effective_desired() && fsm_.commanded() && safety_allows_on(now_s);
}

Lease SprayController::lease(double now_s) const {
  Lease l;
  const auto [sok, why] = fsm_safety_ok(now_s);
  (void)why;
  l.backend =
      p_->str(P::actuator_backend) == "mavlink_actuator" ? kBackendActuator : kBackendServoPwm;
  l.command_seq = fsm_.cmd_seq();
  l.actuator_set_index = p_->integer(P::actuator_set_index);
  l.off_value = p_->num(P::off_value);
  l.servo_instance = p_->integer(P::servo_instance);
  l.off_pwm_us = p_->integer(P::off_pwm_us);
  l.allow_on =
      !shut_down_ && effective_desired() && sok && p_->flag(P::spray_enabled) && fsm_.commanded();
  return l;
}

ControllerStatus SprayController::status(double now_s) const {
  ControllerStatus s;
  s.fsm_state = fsm_.state();
  s.spraying = fsm_.spraying();
  s.desired = effective_desired();
  s.manual_active = manual_active_;
  s.cmd_seq = fsm_.cmd_seq();
  s.enabled = p_->flag(P::spray_enabled);
  s.watchdog_ok = watchdog_ok(nullptr, now_s);
  s.commanded_flow = commanded_flow_ ? *commanded_flow_ : 0.0;
  s.distance_to_boundary_m = std::numeric_limits<double>::quiet_NaN();
  const auto [sok, why] = fsm_safety_ok(now_s);
  s.safety_ok = sok;
  s.safety_reason = why;
  if (last_decision_) {
    const Decision& d = *last_decision_;
    s.geometry_desired = d.geometry_desired;
    s.xtrack_tripped = d.xtrack_tripped;
    s.event = d.event;
    if (d.projection) {
      s.projection_valid = true;
      s.projection_s_m = d.projection->s;
      s.xtrack_error_m = d.projection->xtrack_error_m;
    }
    if (std::isfinite(d.distance_to_boundary_m) && d.distance_to_boundary_m != kInf) {
      s.distance_to_boundary_m = d.distance_to_boundary_m;
    }
  }
  return s;
}

}  // namespace dyx3_spray
