// spray_controller — the spray decision pipeline as one pure, clock-injected object. Contract:
// docs/contracts/dyx3_spray.md sections 2-7. No ROS: spray_node feeds it snapshots and publishes
// what it returns. Pipeline per tick (carried from the prototype's _watchdog_tick ->
// _distance_aware_tick -> _apply_debounce -> _drive_fsm_tick): manual expiry, freshness, nozzle
// position, gate stack, flow, decision, debounce, FSM, lease.
//
// The controller never talks to the FCU. A command it returns is turned into a SprayActuatorCommand
// by the node, carried to the valve by dyx3_px4_link, and acknowledged back through on_ack().
#pragma once

#include <memory>
#include <optional>
#include <string>

#include "dyx3_spray/boundary_projection.hpp"
#include "dyx3_spray/flow_model.hpp"
#include "dyx3_spray/safety_lease.hpp"
#include "dyx3_spray/spray_fsm.hpp"
#include "dyx3_spray/spray_gates.hpp"
#include "dyx3_spray/spray_params.hpp"

namespace dyx3_spray {

struct VehicleSnapshot {
  bool armed{false};
  bool offboard{false};
  bool position_valid{false};
  bool attitude_valid{false};
  bool velocity_valid{false};
  double north_m{0.0}, east_m{0.0}, heading_rad{0.0};
  double vel_n_mps{0.0}, vel_e_mps{0.0};
};

struct RtkSnapshot {
  int fix_type{0};
  std::optional<double> h_acc_m;  // nullopt: unknown (sentinel 0)
  bool corrections_fresh{false};
};

// What to put on the wire for a dispatched command.
struct ActuatorWire {
  int backend{kBackendActuator};
  bool on{false};
  int actuator_set_index{1};
  double value{0.0};
  int servo_instance{1};
  int pwm_us{0};
};

enum class ManualResult : uint8_t { Ok = 0, Disabled = 1, Disarmed = 2, WatchdogNotReady = 3 };

struct ControllerStatus {
  SprayState fsm_state{SprayState::OffUnconfirmed};
  bool spraying{false};
  bool desired{false};
  bool geometry_desired{false};
  bool safety_ok{false};
  std::string safety_reason;
  bool manual_active{false};
  bool projection_valid{false};
  double projection_s_m{0.0};
  double xtrack_error_m{0.0};
  bool xtrack_tripped{false};
  double distance_to_boundary_m;  // NaN when none
  LeadEvent event{LeadEvent::None};
  double commanded_flow{0.0};
  uint32_t cmd_seq{0};
  bool enabled{true};
  bool watchdog_ok{false};
};

class SprayController {
public:
  // tick_period_s: the real control period (1 / tick_hz) the caller ticks at. It converts
  // debounce_samples into the delay the boundary lead compensates; throws std::invalid_argument
  // unless finite and > 0.
  SprayController(const ParamSet* params, double tick_period_s);

  // ---- inputs (all stamps are caller-supplied monotonic seconds) ----
  void load_path(std::shared_ptr<const PathModel> model);  // nullptr clears
  void note_vehicle(const VehicleSnapshot& v, double now_s);
  void note_rtk(const RtkSnapshot& r, double now_s);
  // state: RppStatus.state (RppState values); mission_id: RppStatus.mission_id.
  void note_rpp(uint8_t state, uint32_t mission_id, uint32_t run_index, double heading_error_rad,
                double path_travel_m, bool heading_evidence_valid, double now_s);
  void note_estop(bool asserted, double now_s);
  void note_watchdog(bool alive, bool off_authority_ready, double now_s);
  // MissionState: any non-RUNNING state or a different mission id clears the tracking evidence.
  void set_mission(bool running, uint32_t mission_id);

  ManualResult set_manual(bool on, double now_s);

  // ---- outputs ----
  std::optional<SprayCommand> tick(double now_s);
  std::optional<SprayCommand> on_ack(uint32_t seq, bool success, double now_s);
  // Shutdown: revoke the lease and force an immediate OFF (RECOVERY backoff reset).
  std::optional<SprayCommand> shutdown(double now_s);

  ActuatorWire wire_for(bool on) const;  // the value/pwm to send for the FSM's command
  bool reassert_due(
      double now_s) const;  // ON desired, commanded and still allowed: re-affirm the ON on the wire
  Lease lease(double now_s) const;  // the lease to publish this tick
  ControllerStatus status(double now_s) const;
  bool path_loaded() const { return static_cast<bool>(model_); }
  const SpraySafetyStateMachine& fsm() const { return fsm_; }

private:
  bool watchdog_ok(std::string* reason, double now_s) const;
  bool safety_allows_on(double now_s) const;
  std::pair<bool, std::string> fsm_safety_ok(double now_s) const;
  GateResult ownership(double now_s) const;
  bool vehicle_fresh(double now_s) const;
  bool pose_fresh(double now_s) const;
  bool velocity_fresh(double now_s) const;
  void update_flow(double speed, double dt);
  std::optional<SprayCommand> drive_fsm(double now_s);
  bool effective_desired() const { return manual_active_ ? true : desired_debounced_; }
  DecisionParams decision_params() const;

  const ParamSet* p_;
  double tick_period_s_;
  SpraySafetyStateMachine fsm_;
  std::unique_ptr<FlowModulator> flow_;
  bool prev_commanded_{false};
  std::optional<double> commanded_flow_;

  std::shared_ptr<const PathModel> model_;
  DecisionState dstate_;
  std::optional<double> xtrack_trip_s_;
  std::optional<Decision> last_decision_;
  LeadEvent last_event_{LeadEvent::None};

  VehicleSnapshot veh_;
  bool have_veh_{false};
  double veh_recv_s_{0.0};
  RtkSnapshot rtk_;
  bool have_rtk_{false};
  double rtk_recv_s_{0.0};
  RtkGate rtk_gate_;
  PivotGate pivot_gate_;
  bool tracking_seen_{false};
  bool estop_known_{false};
  bool estop_asserted_{true};
  double estop_recv_s_{0.0};
  bool wd_known_{false};
  bool wd_alive_{false};
  bool wd_ready_{false};
  double wd_recv_s_{0.0};
  bool mission_running_{false};
  uint32_t mission_id_{0};
  bool rpp_known_{false};
  uint8_t rpp_state_{0};
  uint32_t rpp_mission_id_{0};
  uint32_t rpp_run_index_{0};
  double rpp_heading_error_rad_{0.0};
  double rpp_path_travel_m_{0.0};
  bool rpp_heading_evidence_valid_{false};
  double rpp_recv_s_{0.0};
  bool heading_entry_hold_{true};

  bool manual_active_{false};
  double manual_deadline_s_{0.0};
  bool desired_raw_{false};
  bool desired_debounced_{false};
  bool have_candidate_{false};
  bool candidate_{false};
  int candidate_count_{0};
  bool shut_down_{false};
  bool have_last_tick_{false};
  double last_tick_s_{0.0};
};

}  // namespace dyx3_spray
