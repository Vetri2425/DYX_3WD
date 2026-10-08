#include "dyx3_motion_guard/motion_guard_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dyx3_motion_guard {
namespace {

using dyx3_interfaces::msg::MotionSetpoint;
using dyx3_interfaces::msg::MotionSetpointStatus;

// The decision code's reason enum is the frozen message ABI.
static_assert(static_cast<uint8_t>(Reason::InvalidMessage) ==
              MotionSetpointStatus::REASON_INVALID_MESSAGE);
static_assert(static_cast<uint8_t>(Reason::Stale) == MotionSetpointStatus::REASON_STALE);
static_assert(static_cast<uint8_t>(Reason::Sequence) == MotionSetpointStatus::REASON_SEQUENCE);
static_assert(static_cast<uint8_t>(Reason::MissionGate) ==
              MotionSetpointStatus::REASON_MISSION_GATE);
static_assert(static_cast<uint8_t>(Reason::Estop) == MotionSetpointStatus::REASON_ESTOP);
static_assert(static_cast<uint8_t>(Reason::RtkGate) == MotionSetpointStatus::REASON_RTK_GATE);
static_assert(static_cast<uint8_t>(Reason::LimitClamped) ==
              MotionSetpointStatus::REASON_LIMIT_CLAMPED);
static_assert(static_cast<uint8_t>(Reason::Px4LinkUnhealthy) ==
              MotionSetpointStatus::REASON_PX4_LINK_UNHEALTHY);
static_assert(static_cast<uint8_t>(Reason::HeadingUnhealthy) ==
              MotionSetpointStatus::REASON_HEADING_UNHEALTHY);
static_assert(static_cast<uint8_t>(Reason::OperatorLinkLost) ==
              MotionSetpointStatus::REASON_OPERATOR_LINK_LOST);
static_assert(static_cast<uint8_t>(Reason::ArmingGate) == MotionSetpointStatus::REASON_ARMING_GATE);
static_assert(static_cast<uint8_t>(Reason::EstimatorUnhealthy) ==
              MotionSetpointStatus::REASON_ESTIMATOR_UNHEALTHY);

double steady_now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void require(bool ok, const std::string& what) {
  if (!ok) throw std::invalid_argument("motion_guard parameter invalid: " + what);
}

}  // namespace

MotionGuardNode::MotionGuardNode(const rclcpp::NodeOptions& options, ClockFn clock,
                                 bool create_timer)
    : rclcpp::Node("motion_guard", options),
      clock_(clock ? std::move(clock) : ClockFn(steady_now_s)) {
  declare_and_validate_params();
  DecisionConfig dc;
  dc.command_max_age_s = age_.command;
  dc.gates = gate_cfg_;
  dc.limits = limits_;
  core_ = std::make_unique<GuardCore>(accept_count_, dc);

  const auto rel1 = rclcpp::QoS(1).reliable();
  const auto rel10 = rclcpp::QoS(10).reliable();
  pub_cmd_ = create_publisher<MotionSetpoint>("/dyx3/motion_guard/command", rel1);
  pub_status_ = create_publisher<MotionSetpointStatus>("/dyx3/motion_guard/status", rel10);
  pub_gate_ = create_publisher<dyx3_interfaces::msg::SafetyGateStatus>("/dyx3/safety_gate", rel1);
  pub_estop_ = create_publisher<dyx3_interfaces::msg::EmergencyStopState>(
      "/dyx3/emergency_stop_state", rel1);

  sub_cmd_ = create_subscription<MotionSetpoint>("/dyx3/rpp/motion_setpoint", rel1,
                                                 [this](MotionSetpoint::ConstSharedPtr m) {
                                                   Command c;
                                                   c.seq = m->seq;
                                                   c.mode = m->mode;
                                                   c.speed_body_x = m->speed_body_x;
                                                   c.yaw_setpoint = m->yaw_setpoint;
                                                   c.yaw_rate_setpoint = m->yaw_rate_setpoint;
                                                   c.valid = m->valid;
                                                   const double now = clock_();
                                                   core_->on_command(c, now);
                                                   w_cmd_.touch(now);
                                                 });
  sub_mission_ = create_subscription<dyx3_interfaces::msg::MissionState>(
      "/dyx3/mission/state", rel1, [this](dyx3_interfaces::msg::MissionState::ConstSharedPtr m) {
        mission_.state = m->state;
        w_mission_.touch(clock_());
      });
  sub_vehicle_ = create_subscription<dyx3_interfaces::msg::VehicleState>(
      "/dyx3/vehicle_state", rel1, [this](dyx3_interfaces::msg::VehicleState::ConstSharedPtr m) {
        veh_.arming_state = m->arming_state;
        veh_.nav_state = m->nav_state;
        veh_.failsafe = m->failsafe;
        veh_.position_valid = m->position_valid;
        veh_.velocity_valid = m->velocity_valid;
        veh_.attitude_valid = m->attitude_valid;
        w_veh_.touch(clock_());
      });
  sub_est_ = create_subscription<dyx3_interfaces::msg::EstimatorHealth>(
      "/dyx3/estimator_health", rel1,
      [this](dyx3_interfaces::msg::EstimatorHealth::ConstSharedPtr m) {
        est_.flags_valid = m->flags_valid;
        est_.gnss_yaw_fusion_intended = m->gnss_yaw_fusion_intended;
        est_.gnss_yaw_fault = m->gnss_yaw_fault;
        est_.reject_yaw = m->reject_yaw;
        est_.reject_hor_pos = m->reject_hor_pos;
        est_.reject_hor_vel = m->reject_hor_vel;
        est_.inertial_dead_reckoning = m->inertial_dead_reckoning;
        w_est_.touch(clock_());
      });
  sub_rtk_ = create_subscription<dyx3_interfaces::msg::RtkStatus>(
      "/dyx3/rtk_status", rel1, [this](dyx3_interfaces::msg::RtkStatus::ConstSharedPtr m) {
        rtk_.fix_type = m->fix_type;
        rtk_.corrections_fresh = m->corrections_fresh;
        rtk_.horizontal_accuracy_m = m->horizontal_accuracy_m;
        w_rtk_.touch(clock_());
      });
  sub_op_ = create_subscription<dyx3_interfaces::msg::OperatorLinkStatus>(
      "/dyx3/operator_link", rel1,
      [this](dyx3_interfaces::msg::OperatorLinkStatus::ConstSharedPtr m) {
        op_.alive = m->alive;
        w_op_.touch(clock_());
      });
  sub_link_ = create_subscription<dyx3_interfaces::msg::Px4LinkStatus>(
      "/dyx3/px4_link/status", rel1, [this](dyx3_interfaces::msg::Px4LinkStatus::ConstSharedPtr m) {
        link_.session_alive = m->session_alive;
        link_.handshake_ok = m->handshake_ok;
        link_.stale_topics_mask = m->stale_topics_mask;
        w_link_.touch(clock_());
      });

  srv_estop_ = create_service<dyx3_interfaces::srv::SetEmergencyStop>(
      "/dyx3/motion_guard/set_emergency_stop",
      [this](const std::shared_ptr<dyx3_interfaces::srv::SetEmergencyStop::Request> req,
             std::shared_ptr<dyx3_interfaces::srv::SetEmergencyStop::Response> res) {
        const bool ok = estop_.request(req->asserted, req->source);
        res->accepted = ok;
        res->reason_code =
            ok ? dyx3_interfaces::srv::SetEmergencyStop::Response::REASON_OK
               : dyx3_interfaces::srv::SetEmergencyStop::Response::REASON_INVALID_SOURCE;
        if (ok) {
          RCLCPP_WARN(get_logger(), "emergency stop %s by %s",
                      req->asserted ? "ASSERTED" : "cleared", req->source.c_str());
          // Latched at once: the next decision tick outputs STOP; do not wait for it to be
          // published.
          step(clock_());
        }
      });

  if (create_timer) {
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_hz_),
                               [this]() { step(clock_()); });
  }
  RCLCPP_INFO(get_logger(),
              "motion_guard up: %.0f Hz, command_max_age %.3f s, session_accept_count %u",
              publish_rate_hz_, age_.command, accept_count_);
}

void MotionGuardNode::declare_and_validate_params() {
  const auto d = [this](const char* n, double v) { return declare_parameter<double>(n, v); };
  publish_rate_hz_ = d("publish_rate_hz", 50.0);
  require(std::isfinite(publish_rate_hz_) && publish_rate_hz_ >= 10.0,
          "publish_rate_hz must be >= 10");
  const int acc = static_cast<int>(declare_parameter<int>("session_accept_count", 3));
  require(acc >= 1 && acc <= 100, "session_accept_count in [1, 100]");
  accept_count_ = static_cast<uint32_t>(acc);
  age_.command = d("command_max_age_s", 0.2);
  age_.vehicle = d("vehicle_state_max_age_s", 0.5);
  age_.rtk = d("rtk_status_max_age_s", 0.5);
  age_.estimator = d("estimator_health_max_age_s", 0.5);
  age_.operator_link = d("operator_link_max_age_s", 0.5);
  age_.px4_link = d("px4_link_max_age_s", 0.5);
  age_.mission = d("mission_state_max_age_s", 0.5);
  for (const double v : {age_.command, age_.vehicle, age_.rtk, age_.estimator, age_.operator_link,
                         age_.px4_link, age_.mission}) {
    require(std::isfinite(v) && v > 0.0, "every *_max_age_s must be finite and > 0");
  }
  const int fix = static_cast<int>(declare_parameter<int>("rtk_min_fix_type", 6));
  require(fix == 5 || fix == 6, "rtk_min_fix_type must be 5 or 6");
  gate_cfg_.rtk.min_fix_type = static_cast<uint8_t>(fix);
  gate_cfg_.rtk.max_hrms_m = static_cast<float>(d("rtk_max_hrms_m", 0.10));
  require(std::isfinite(gate_cfg_.rtk.max_hrms_m) && gate_cfg_.rtk.max_hrms_m > 0.0F,
          "rtk_max_hrms_m must be > 0");
  gate_cfg_.require_gnss_yaw_fusion = declare_parameter<bool>("require_gnss_yaw_fusion", true);
  limits_.max_forward_speed_mps = static_cast<float>(d("max_forward_speed_mps", 1.0));
  limits_.max_reverse_speed_mps = static_cast<float>(d("max_reverse_speed_mps", 0.10));
  limits_.max_yaw_rate_radps = static_cast<float>(d("max_yaw_rate_radps", 0.45));
  // B2 / review H5: accel/decel/jerk/yaw-accel are owned by RPP and are intentionally not ROS
  // parameters here. profile_shaping_test_mode remains false in the production node.
  require(limits_valid(limits_), "hard limits (forward/yaw > 0; reverse >= 0)");
}

GateInputs MotionGuardNode::gather(double now_s) const {
  GateInputs g;
  g.estop = estop_.asserted();
  g.link = link_;
  g.link.fresh = w_link_.fresh(now_s, age_.px4_link);
  g.op = op_;
  g.op.fresh = w_op_.fresh(now_s, age_.operator_link);
  g.vehicle = veh_;
  g.vehicle.fresh = w_veh_.fresh(now_s, age_.vehicle);
  g.rtk = rtk_;
  g.rtk.fresh = w_rtk_.fresh(now_s, age_.rtk);
  g.est = est_;
  g.est.fresh = w_est_.fresh(now_s, age_.estimator);
  g.mission = mission_;
  g.mission.fresh = w_mission_.fresh(now_s, age_.mission);
  // A source that was never heard keeps its fail-safe defaults (struct defaults are all "failing").
  if (!w_link_.seen) g.link = Px4LinkIn{};
  if (!w_op_.seen) g.op = OperatorIn{};
  if (!w_veh_.seen) g.vehicle = VehicleIn{};
  if (!w_rtk_.seen) g.rtk = RtkIn{};
  if (!w_est_.seen) g.est = EstimatorIn{};
  if (!w_mission_.seen) g.mission = MissionIn{};
  return g;
}

void MotionGuardNode::step(double now_s) {
  const double nominal = 1.0 / publish_rate_hz_;
  double dt = last_step_s_ < 0.0 ? nominal : now_s - last_step_s_;
  dt = std::clamp(dt, 0.0, 0.1);  // a stalled tick must not widen the ramp step
  last_step_s_ = now_s;

  const GateInputs gates = gather(now_s);
  const Decision d = core_->decide(now_s, dt, gates);

  MotionSetpoint out;
  out.stamp = ros_now();
  out.seq = out_seq_++;
  out.mode = static_cast<uint8_t>(d.out.mode);
  out.speed_body_x = d.out.speed_body_x;
  out.yaw_setpoint = d.out.yaw_setpoint;
  out.yaw_rate_setpoint = d.out.yaw_rate_setpoint;
  out.valid = true;  // always a contract-conforming command: forwarded or the canonical STOP
  pub_cmd_->publish(out);

  const bool reason_changed = d.reason != last_reason_;
  if (reason_changed || d.input_seq != last_status_input_seq_ ||
      now_s - last_status_pub_s_ >= 0.1) {
    last_status_input_seq_ = d.input_seq;
    last_status_pub_s_ = now_s;
    MotionSetpointStatus s;
    s.stamp = ros_now();
    s.input_seq = d.input_seq;
    s.reason_code = static_cast<uint8_t>(d.reason);
    s.accepted = d.accepted;
    s.clamped = d.clamped;
    s.mode = out.mode;
    s.speed_body_x = out.speed_body_x;
    s.yaw_setpoint = out.yaw_setpoint;
    s.yaw_rate_setpoint = out.yaw_rate_setpoint;
    s.valid = true;
    pub_status_->publish(s);
  }
  if (reason_changed) {
    RCLCPP_WARN(get_logger(), "decision reason %u -> %u", static_cast<unsigned>(last_reason_),
                static_cast<unsigned>(d.reason));
  }
  last_reason_ = d.reason;

  if (now_s - last_gate_pub_s_ >= 0.1 - 1e-9) {
    last_gate_pub_s_ = now_s;
    const Reason g = first_failing_safety_gate(gates, gate_cfg_);
    dyx3_interfaces::msg::SafetyGateStatus gs;  // ok=false default
    gs.stamp = ros_now();
    gs.ok = g == Reason::Ok;
    gs.reason_code = static_cast<uint8_t>(g);
    pub_gate_->publish(gs);
    dyx3_interfaces::msg::EmergencyStopState es;
    es.stamp = ros_now();
    es.asserted = estop_.asserted();
    es.source = estop_.source();
    pub_estop_->publish(es);
  }
}

}  // namespace dyx3_motion_guard
