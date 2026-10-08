// spray_node — see docs/contracts/dyx3_spray.md
#include "dyx3_spray/spray_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "dyx3_mission/path_artifact.hpp"

namespace dyx3_spray {
namespace {

using dyx3_interfaces::msg::SprayActuatorAck;
using dyx3_interfaces::msg::SprayActuatorCommand;
using dyx3_interfaces::msg::SprayStatus;

// The FSM and event enums are the frozen message ABI.
static_assert(static_cast<uint8_t>(SprayState::OffUnconfirmed) == SprayStatus::FSM_OFF_UNCONFIRMED);
static_assert(static_cast<uint8_t>(SprayState::OffConfirmed) == SprayStatus::FSM_OFF_CONFIRMED);
static_assert(static_cast<uint8_t>(SprayState::OnPending) == SprayStatus::FSM_ON_PENDING);
static_assert(static_cast<uint8_t>(SprayState::OnConfirmed) == SprayStatus::FSM_ON_CONFIRMED);
static_assert(static_cast<uint8_t>(SprayState::OffPending) == SprayStatus::FSM_OFF_PENDING);
static_assert(static_cast<uint8_t>(SprayState::Recovery) == SprayStatus::FSM_RECOVERY);
static_assert(static_cast<uint8_t>(SprayState::Disabled) == SprayStatus::FSM_DISABLED);
static_assert(static_cast<uint8_t>(LeadEvent::OnEarly) == SprayStatus::EVENT_ON_EARLY);
static_assert(static_cast<uint8_t>(LeadEvent::OffEarly) == SprayStatus::EVENT_OFF_EARLY);
static_assert(static_cast<uint8_t>(LeadEvent::TerminalOff) == SprayStatus::EVENT_TERMINAL_OFF);
static_assert(kBackendActuator == SprayActuatorCommand::BACKEND_ACTUATOR);
static_assert(kBackendServoPwm == SprayActuatorCommand::BACKEND_SERVO_PWM);

constexpr uint8_t kArmed = 2;      // px4 vehicle_status ARMING_STATE_ARMED
constexpr uint8_t kOffboard = 14;  // px4 vehicle_status NAVIGATION_STATE_OFFBOARD

double steady_now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void require(bool ok, const std::string& what) {
  if (!ok) throw std::invalid_argument("spray parameter invalid: " + what);
}

}  // namespace

SprayNode::SprayNode(const rclcpp::NodeOptions& options, ClockFn clock, bool create_timer)
    : rclcpp::Node("spray", options), clock_(clock ? std::move(clock) : ClockFn(steady_now_s)) {
  declare_and_validate_params();
  ctl_ = std::make_unique<SprayController>(&params_);

  // Commands must not be dropped and must keep order: reliable. The lease is depth 1 (only the
  // newest matters).
  const auto rel1 = rclcpp::QoS(1).reliable();
  const auto rel10 = rclcpp::QoS(10).reliable();
  pub_cmd_ = create_publisher<SprayActuatorCommand>("/dyx3/spray/actuator_command", rel10);
  pub_lease_ = create_publisher<dyx3_interfaces::msg::SprayLease>("/dyx3/spray/lease", rel1);
  pub_state_ = create_publisher<dyx3_interfaces::msg::SprayState>("/dyx3/spray/state", rel10);
  pub_status_ = create_publisher<SprayStatus>("/dyx3/spray/status", rel10);

  sub_vehicle_ = create_subscription<dyx3_interfaces::msg::VehicleState>(
      "/dyx3/vehicle_state", rel1, [this](dyx3_interfaces::msg::VehicleState::ConstSharedPtr m) {
        VehicleSnapshot v;
        v.armed = m->arming_state == kArmed;
        v.offboard = m->nav_state == kOffboard && !m->failsafe;
        v.position_valid = m->position_valid;
        v.attitude_valid = m->attitude_valid;
        v.velocity_valid = m->velocity_valid;
        v.north_m = m->north_m;
        v.east_m = m->east_m;
        v.heading_rad = m->heading_rad;
        v.vel_n_mps = m->velocity_north_mps;
        v.vel_e_mps = m->velocity_east_mps;
        ctl_->note_vehicle(v, clock_());
      });
  sub_rtk_ = create_subscription<dyx3_interfaces::msg::RtkStatus>(
      "/dyx3/rtk_status", rel1, [this](dyx3_interfaces::msg::RtkStatus::ConstSharedPtr m) {
        RtkSnapshot r;
        r.fix_type = m->fix_type;
        r.corrections_fresh = m->corrections_fresh;
        // horizontal_accuracy_m == 0 is the "unknown" sentinel (A14), never a perfect fix.
        if (m->horizontal_accuracy_m > 0.0F && std::isfinite(m->horizontal_accuracy_m)) {
          r.h_acc_m = static_cast<double>(m->horizontal_accuracy_m);
        }
        ctl_->note_rtk(r, clock_());
      });
  sub_rpp_ = create_subscription<dyx3_interfaces::msg::RppStatus>(
      "/dyx3/rpp/status", rel1, [this](dyx3_interfaces::msg::RppStatus::ConstSharedPtr m) {
        rpp_mission_id_ = m->mission_id;
        rpp_conditioned_sha_ = m->conditioned_execution_sha256;
        if (mission_running_ && mission_id_ == m->mission_id && rpp_mission_id_ == mission_id_ &&
            !rpp_conditioned_sha_.empty() && loaded_sha_ != rpp_conditioned_sha_)
          load_artifact(rpp_conditioned_sha_, mission_source_sha_);
        // Tracking evidence (B5): TRACKING of the RUNNING mission only (fail closed: it delays,
        // never advances, the first mark). Pivot gate: PIVOTING only (CORNER_ALIGN), never STOPPING
        // (CORNER_STOP still lays the last 2 cm of the leg). Ownership gate (C1): see spray_gates.
        ctl_->note_rpp(m->state, m->mission_id, m->run_index, m->heading_error_rad,
                       m->path_travel_m, m->heading_evidence_valid, clock_());
      });
  sub_mission_ = create_subscription<dyx3_interfaces::msg::MissionState>(
      "/dyx3/mission/state", rel1,
      [this](dyx3_interfaces::msg::MissionState::ConstSharedPtr m) { on_mission_state(*m); });
  sub_estop_ = create_subscription<dyx3_interfaces::msg::EmergencyStopState>(
      "/dyx3/emergency_stop_state", rel1,
      [this](dyx3_interfaces::msg::EmergencyStopState::ConstSharedPtr m) {
        ctl_->note_estop(m->asserted, clock_());
      });
  sub_wd_ = create_subscription<dyx3_interfaces::msg::SprayWatchdogStatus>(
      "/dyx3/spray/watchdog_status", rel1,
      [this](dyx3_interfaces::msg::SprayWatchdogStatus::ConstSharedPtr m) {
        ctl_->note_watchdog(m->watchdog_alive, m->off_authority_ready, clock_());
      });
  sub_ack_ = create_subscription<SprayActuatorAck>(
      "/dyx3/spray/actuator_ack", rel10, [this](SprayActuatorAck::ConstSharedPtr m) {
        if (m->source != SprayActuatorCommand::SOURCE_CONTROLLER)
          return;  // the watchdog's acks are not ours
        const double now = clock_();
        if (const auto next = ctl_->on_ack(m->seq, m->success, now))
          publish_command(next->seq, next->on);
      });
  srv_manual_ = create_service<dyx3_interfaces::srv::SetSprayManual>(
      "/dyx3/spray/set_manual",
      [this](const std::shared_ptr<dyx3_interfaces::srv::SetSprayManual::Request> req,
             std::shared_ptr<dyx3_interfaces::srv::SetSprayManual::Response> res) {
        using R = dyx3_interfaces::srv::SetSprayManual::Response;
        const ManualResult r = ctl_->set_manual(req->on, clock_());
        res->accepted = r == ManualResult::Ok;
        res->reason_code =
            static_cast<uint8_t>(r == ManualResult::Ok         ? R::REASON_OK
                                 : r == ManualResult::Disabled ? R::REASON_DISABLED
                                 : r == ManualResult::Disarmed ? R::REASON_DISARMED
                                                               : R::REASON_WATCHDOG_NOT_READY);
        step(clock_());  // act at once, do not wait for the next tick
      });

  param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& ps) {
    rcl_interfaces::msg::SetParametersResult res;
    std::vector<Item> items;
    for (const auto& p : ps) {
      if (p.get_name() == "use_sim_time") continue;
      Item it;
      it.name = p.get_name();
      const int idx = find_index(it.name);
      if (idx < 0) {
        res.successful = false;
        res.reason = "unknown parameter " + it.name;
        return res;
      }
      switch (descriptors()[idx].kind) {
        case Kind::Bool:
          it.num = p.as_bool() ? 1.0 : 0.0;
          break;
        case Kind::Int:
          it.num = static_cast<double>(p.as_int());
          break;
        case Kind::Float:
          it.num = p.as_double();
          break;
        case Kind::String:
          it.str = p.as_string();
          break;
      }
      items.push_back(std::move(it));
    }
    SetContext ctx;
    ctx.mission_running = mission_running_;
    ctx.source = "ros";
    const SetResult r = params_.set_many(items, ctx);
    res.successful = r.ok;
    res.reason = r.reason;
    return res;
  });

  if (create_timer) {
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tick_hz_),
                               [this]() { step(clock_()); });
  }
  RCLCPP_INFO(get_logger(), "spray up: %.0f Hz tick, backend %s, artifacts from %s", tick_hz_,
              params_.str(P::actuator_backend).c_str(), artifact_dir_.c_str());
}

void SprayNode::declare_and_validate_params() {
  tick_hz_ =
      declare_parameter<double>("tick_hz", 50.0);  // DERIVED: the prototype's 20 ms control timer
  require(std::isfinite(tick_hz_) && tick_hz_ >= 10.0, "tick_hz must be >= 10");
  artifact_dir_ = declare_parameter<std::string>("artifact_dir", "/var/lib/dyx3/missions");
  std::vector<Item> items;
  for (size_t i = 0; i < kParamCount; ++i) {
    const Descriptor& d = descriptors()[i];
    Item it;
    it.name = d.name;
    switch (d.kind) {
      case Kind::Bool:
        it.num = declare_parameter<bool>(d.name, d.dflt != 0.0) ? 1.0 : 0.0;
        break;
      case Kind::Int:
        it.num =
            static_cast<double>(declare_parameter<int64_t>(d.name, static_cast<int64_t>(d.dflt)));
        break;
      case Kind::Float:
        it.num = declare_parameter<double>(d.name, d.dflt);
        break;
      case Kind::String:
        it.str = declare_parameter<std::string>(d.name, d.sdflt);
        break;
    }
    items.push_back(std::move(it));
  }
  const SetResult r = params_.init_many(items);
  require(r.ok, r.reason);  // fail loud: never fall back to a default
}

void SprayNode::publish_command(uint32_t seq, bool on) {
  const ActuatorWire w = ctl_->wire_for(on);
  SprayActuatorCommand c;
  c.stamp = ros_now();
  c.seq = seq;
  c.source = SprayActuatorCommand::SOURCE_CONTROLLER;
  c.backend = static_cast<uint8_t>(w.backend);
  c.on = on;
  c.actuator_set_index = static_cast<uint8_t>(w.actuator_set_index);
  c.value = static_cast<float>(w.value);
  c.servo_instance = static_cast<uint8_t>(w.servo_instance);
  c.pwm_us = static_cast<uint16_t>(w.pwm_us);
  pub_cmd_->publish(c);
}

void SprayNode::publish_lease(double now_s) {
  const Lease l = ctl_->lease(now_s);
  dyx3_interfaces::msg::SprayLease m;
  m.stamp = ros_now();
  m.allow_on = l.allow_on;
  m.command_seq = static_cast<uint32_t>(l.command_seq);
  m.backend = static_cast<uint8_t>(l.backend);
  m.actuator_set_index = static_cast<uint8_t>(l.actuator_set_index);
  m.off_value = static_cast<float>(l.off_value);
  m.servo_instance = static_cast<uint8_t>(l.servo_instance);
  m.off_pwm_us = static_cast<uint16_t>(l.off_pwm_us);
  pub_lease_->publish(m);
}

void SprayNode::publish_status(double now_s) {
  const ControllerStatus s = ctl_->status(now_s);
  SprayStatus m;
  m.stamp = ros_now();
  m.fsm_state = static_cast<uint8_t>(s.fsm_state);
  m.spraying = s.spraying;
  m.desired = s.desired;
  m.geometry_desired = s.geometry_desired;
  m.safety_ok = s.safety_ok;
  m.safety_reason = s.safety_reason;
  m.manual_active = s.manual_active;
  m.projection_valid = s.projection_valid;
  m.projection_s_m = static_cast<float>(s.projection_s_m);
  m.xtrack_error_m = static_cast<float>(s.xtrack_error_m);
  m.xtrack_tripped = s.xtrack_tripped;
  m.distance_to_boundary_m = static_cast<float>(s.distance_to_boundary_m);
  m.event = static_cast<uint8_t>(s.event);
  m.commanded_flow = static_cast<float>(s.commanded_flow);
  m.cmd_seq = s.cmd_seq;
  pub_status_->publish(m);

  dyx3_interfaces::msg::SprayState st;  // OFF/enabled=false defaults are safe
  st.stamp = ros_now();
  st.state = s.spraying                            ? dyx3_interfaces::msg::SprayState::STATE_ON
             : s.fsm_state == SprayState::Recovery ? dyx3_interfaces::msg::SprayState::STATE_FAULT
                                                   : dyx3_interfaces::msg::SprayState::STATE_OFF;
  st.enabled = s.enabled;
  st.watchdog_healthy = s.watchdog_ok;
  st.commanded_flow_normalized = static_cast<float>(s.commanded_flow);
  pub_state_->publish(st);
}

void SprayNode::on_mission_state(const dyx3_interfaces::msg::MissionState& m) {
  mission_running_ = m.state == dyx3_interfaces::msg::MissionState::STATE_RUNNING;
  mission_id_ = m.mission_id;
  ctl_->set_mission(mission_running_, m.mission_id);
  mission_source_sha_ = m.path_artifact_sha256;
  if (m.path_artifact_sha256.empty()) {
    loaded_sha_.clear();
    ctl_->load_path(nullptr);
    return;
  }
  if (rpp_mission_id_ != mission_id_ || rpp_conditioned_sha_.empty()) {
    loaded_sha_.clear();
    ctl_->load_path(nullptr);
    return;
  }
  if (rpp_conditioned_sha_ != loaded_sha_) load_artifact(rpp_conditioned_sha_, mission_source_sha_);
}

// Geometry and flags come only from the RPP-produced conditioned artifact. Source SHA binding is
// checked before creating the PathModel; the original mission file is never a fallback.
void SprayNode::load_artifact(const std::string& sha, const std::string& source_sha) {
  loaded_sha_ = sha;
  if (sha.empty()) {
    ctl_->load_path(nullptr);
    RCLCPP_INFO(get_logger(), "spray path cleared");
    return;
  }
  const auto r = dyx3_mission::load_conditioned_artifact(artifact_dir_, sha);
  if (!r.ok) {
    loaded_sha_.clear();
    ctl_->load_path(nullptr);  // "path not loaded" gate: spray stays OFF
    RCLCPP_ERROR(get_logger(), "spray path %s rejected: %s", sha.c_str(), r.error.c_str());
    return;
  }
  if (r.artifact.source_sha256 != source_sha) {
    loaded_sha_.clear();
    ctl_->load_path(nullptr);
    RCLCPP_ERROR(get_logger(), "conditioned path %s belongs to a different source artifact",
                 sha.c_str());
    return;
  }
  std::vector<double> n, e;
  std::vector<bool> f;
  std::vector<uint32_t> run_ids;
  for (size_t ri = 0; ri < r.artifact.runs.size(); ++ri) {
    const auto& run = r.artifact.runs[ri];
    for (size_t i = 0; i < run.points.size(); ++i) {
      n.push_back(run.points[i].north_m);
      e.push_back(run.points[i].east_m);
      f.push_back(run.flags[i] != 0);
      run_ids.push_back(static_cast<uint32_t>(ri));
    }
  }
  auto model = std::make_shared<PathModel>();
  if (!build_path_model(n, e, f, run_ids, model.get())) {
    ctl_->load_path(nullptr);
    RCLCPP_ERROR(get_logger(), "spray path %s: inconsistent geometry", sha.c_str());
    return;
  }
  const size_t boundaries = model->boundaries.size();
  ctl_->load_path(std::move(model));
  RCLCPP_INFO(get_logger(), "spray path loaded: %zu points, %zu boundaries", n.size(), boundaries);
}

void SprayNode::step(double now_s) {
  if (const auto cmd = ctl_->tick(now_s)) publish_command(cmd->seq, cmd->on);

  // Re-assert an already-commanded ON on the wire (no FSM transition, no seq bump): a heartbeat, as
  // in the prototype.
  const double hz = params_.num(P::reassert_hz);
  if (hz > 0.0 && now_s >= next_reassert_s_ && ctl_->reassert_due(now_s)) {
    next_reassert_s_ = now_s + 1.0 / hz;
    publish_command(ctl_->fsm().cmd_seq(), true);
  }
  publish_lease(now_s);

  const ControllerStatus s = ctl_->status(now_s);
  const bool changed = s.fsm_state != last_fsm_state_ || s.event != last_event_;
  if (changed || now_s - last_status_pub_s_ >= 0.1) {
    last_status_pub_s_ = now_s;
    publish_status(now_s);
  }
  if (s.event != last_event_ && s.event != LeadEvent::None) {
    RCLCPP_INFO(get_logger(), "spray %s", to_string(s.event));
  }
  last_event_ = s.event;
  if (s.fsm_state != last_fsm_state_) {
    RCLCPP_INFO(get_logger(), "spray fsm %s -> %s", to_string(last_fsm_state_),
                to_string(s.fsm_state));
    last_fsm_state_ = s.fsm_state;
  }
  if (s.geometry_desired && !s.safety_ok && s.safety_reason != last_block_reason_) {
    RCLCPP_WARN(get_logger(), "safety blocked spray: %s", s.safety_reason.c_str());
  }
  last_block_reason_ = s.safety_ok ? std::string() : s.safety_reason;
}

void SprayNode::shutdown_off() {
  const double now = clock_();
  if (const auto cmd = ctl_->shutdown(now)) publish_command(cmd->seq, cmd->on);
  publish_lease(now);
  publish_status(now);
}

bool SprayNode::off_confirmed() const {
  const SprayState s = ctl_->fsm().state();
  return s == SprayState::OffConfirmed || s == SprayState::Disabled;
}

}  // namespace dyx3_spray
