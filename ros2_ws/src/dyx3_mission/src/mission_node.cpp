// mission_node — see docs/contracts/dyx3_mission.md
#include "dyx3_mission/mission_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <utility>

#include "dyx3_interfaces/msg/motion_setpoint_status.hpp"

namespace dyx3_mission {

namespace {
using dyx3_interfaces::msg::MotionSetpointStatus;
using dyx3_interfaces::msg::RppStatus;
using StartSrv = dyx3_interfaces::srv::StartMission;

bool is_lower_hex64(const std::string& s) {
  if (s.size() != 64) return false;
  for (char c : s) {
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  }
  return true;
}
}  // namespace

MissionNode::MissionNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("dyx3_mission", options) {
  // Parameter classes (spec section 9): missions_dir RESTART, the rest IDLE_ONLY.
  // DERIVED — NOT FROM V1 SPEC: defaults and their sources are in docs/contracts/dyx3_mission.md
  // section 8; rpp_ack_timeout_s has no source either, so its default is conservative and must
  // exceed the largest mission's RPP conditioning time.
  missions_dir_ = declare_parameter<std::string>("missions_dir", "/var/lib/dyx3/missions");
  state_publish_hz_ = declare_parameter<double>("state_publish_hz", 10.0);
  gate_max_age_s_ = declare_parameter<double>("gate_status_max_age_s", 0.5);
  point_capture_radius_m_ = declare_parameter<double>("point_capture_radius_m", 0.10);
  rpp_ack_timeout_s_ = declare_parameter<double>("rpp_ack_timeout_s", 30.0);
  rpp_status_max_age_s_ = declare_parameter<double>("rpp_status_max_age_s", 0.5);
  if (!(state_publish_hz_ > 0.0 && state_publish_hz_ <= 100.0) || !(gate_max_age_s_ > 0.0) ||
      !(point_capture_radius_m_ > 0.0) || !(rpp_ack_timeout_s_ > 0.0) ||
      !(rpp_status_max_age_s_ > 0.0) ||
      !std::isfinite(state_publish_hz_ + gate_max_age_s_ + point_capture_radius_m_ +
                     rpp_ack_timeout_s_ + rpp_status_max_age_s_)) {
    throw std::invalid_argument("dyx3_mission: invalid parameter value");
  }
  param_cb_ = add_on_set_parameters_callback(
      std::bind(&MissionNode::on_parameters, this, std::placeholders::_1));

  fsm_.set_observer([this](const Transition& t) {
    RCLCPP_INFO(get_logger(), "mission %s: %s -> %s on %s reason=%u%s%s",
                t.refused ? "REFUSED" : "transition", to_string(t.from), to_string(t.to),
                to_string(t.event), static_cast<unsigned>(t.reason), t.detail.empty() ? "" : " : ",
                t.detail.c_str());
  });

  const auto status_qos = rclcpp::QoS(1).reliable();
  const auto sensor_qos = rclcpp::QoS(1).best_effort();
  state_pub_ =
      create_publisher<dyx3_interfaces::msg::MissionState>("/dyx3/mission/state", status_qos);
  point_pub_ = create_publisher<dyx3_interfaces::msg::PointResult>("/dyx3/mission/point_result",
                                                                   rclcpp::QoS(100).reliable());
  gate_sub_ = create_subscription<dyx3_interfaces::msg::SafetyGateStatus>(
      "/dyx3/safety_gate", sensor_qos,
      [this](const dyx3_interfaces::msg::SafetyGateStatus& m) { on_gate(m); });
  rpp_sub_ = create_subscription<RppStatus>("/dyx3/rpp/status", sensor_qos,
                                            [this](const RppStatus& m) { on_rpp(m); });
  vehicle_sub_ = create_subscription<dyx3_interfaces::msg::VehicleState>(
      "/dyx3/vehicle_state", sensor_qos,
      [this](const dyx3_interfaces::msg::VehicleState& m) { on_vehicle(m); });

  start_srv_ = create_service<StartSrv>("/dyx3/mission/start",
                                        [this](const std::shared_ptr<StartSrv::Request> req,
                                               std::shared_ptr<StartSrv::Response> res) {
                                          const auto o = begin_mission(req->path_artifact_sha256);
                                          res->accepted = o.accepted;
                                          res->reason_code = o.reason;
                                          res->mission_id = o.mission_id;
                                        });
  pause_srv_ = create_service<dyx3_interfaces::srv::PauseMission>(
      "/dyx3/mission/pause",
      [this](const std::shared_ptr<dyx3_interfaces::srv::PauseMission::Request>,
             std::shared_ptr<dyx3_interfaces::srv::PauseMission::Response> res) {
        const Result r = fsm_.pause(now_ns());
        res->accepted = r.accepted;
        res->reason_code = r.accepted ? res->REASON_OK : res->REASON_NOT_RUNNING;
        publish_state();
      });
  resume_srv_ = create_service<dyx3_interfaces::srv::ResumeMission>(
      "/dyx3/mission/resume",
      [this](const std::shared_ptr<dyx3_interfaces::srv::ResumeMission::Request>,
             std::shared_ptr<dyx3_interfaces::srv::ResumeMission::Response> res) {
        // Resume also needs a fresh RppStatus: otherwise it would run until the next timer tick.
        const Result r = fsm_.resume(gate_ok() && rpp_status_fresh(), now_ns());
        res->accepted = r.accepted;
        res->reason_code = r.accepted                       ? res->REASON_OK
                           : r.reject == Reject::kNotPaused ? res->REASON_NOT_PAUSED
                                                            : res->REASON_SAFETY_GATE;
        publish_state();
      });
  abort_srv_ = create_service<dyx3_interfaces::srv::AbortMission>(
      "/dyx3/mission/abort",
      [this](const std::shared_ptr<dyx3_interfaces::srv::AbortMission::Request> req,
             std::shared_ptr<dyx3_interfaces::srv::AbortMission::Response> res) {
        const Result r = fsm_.abort(req->reason_code, now_ns());
        res->accepted = r.accepted;
        res->reason_code = r.accepted ? res->REASON_OK : res->REASON_NOT_ACTIVE;
        if (r.accepted) {
          publish_points(journal_ ? journal_->finish() : std::vector<PointEvent>{});
          finish_goal_if_terminal();
        }
        publish_state();
      });
  skip_srv_ = create_service<dyx3_interfaces::srv::SkipPoint>(
      "/dyx3/mission/skip_point",
      [this](const std::shared_ptr<dyx3_interfaces::srv::SkipPoint::Request>,
             std::shared_ptr<dyx3_interfaces::srv::SkipPoint::Response> res) {
        const bool has_point = journal_ && journal_->active_point().has_value();
        const Result r = fsm_.skip_point(has_point, now_ns());
        res->accepted = r.accepted;
        if (r.accepted) {
          const auto ev = journal_->skip();
          res->reason_code = res->REASON_OK;
          res->skipped_point_index = ev ? ev->point_index : 0U;
          if (ev) publish_point(*ev);
        } else {
          res->reason_code = r.reject == Reject::kNotRunning ? res->REASON_NOT_RUNNING
                                                             : res->REASON_NO_ACTIVE_POINT;
        }
        publish_state();
      });

  action_ = rclcpp_action::create_server<ExecuteMission>(
      this, "/dyx3/mission/execute",
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteMission::Goal> goal) {
        // One mission at a time; begin_mission() is the single start path.
        const auto o = begin_mission(goal->path_artifact_sha256);
        return o.accepted ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                          : rclcpp_action::GoalResponse::REJECT;
      },
      [this](const std::shared_ptr<GoalHandle>) {
        // Cancel == AbortMission with REASON_OPERATOR. The goal is not CANCELING yet inside this
        // callback (finishing here would report abort, not canceled): on_timer finalises it
        // (review H3 / fix plan A3).
        fsm_.abort(kReasonOperator, now_ns());
        publish_points(journal_ ? journal_->finish() : std::vector<PointEvent>{});
        publish_state();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> gh) {
        goal_ = gh;
        finish_goal_if_terminal();
      });

  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / state_publish_hz_),
                             [this]() { on_timer(); });
  RCLCPP_INFO(get_logger(), "dyx3_mission up (missions_dir=%s)", missions_dir_.c_str());
}

rcl_interfaces::msg::SetParametersResult MissionNode::on_parameters(
    const std::vector<rclcpp::Parameter>& ps) {
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  double next_hz = state_publish_hz_;
  double next_gate_age = gate_max_age_s_;
  double next_capture_radius = point_capture_radius_m_;
  double next_ack_timeout = rpp_ack_timeout_s_;
  double next_rpp_age = rpp_status_max_age_s_;
  for (const auto& p : ps) {
    const std::string& n = p.get_name();
    if (n == "missions_dir") {
      res.successful = false;
      res.reason = "missions_dir is RESTART-class";
      return res;
    }
    if (n == "state_publish_hz" || n == "gate_status_max_age_s" || n == "point_capture_radius_m" ||
        n == "rpp_ack_timeout_s" || n == "rpp_status_max_age_s") {
      if (fsm_.state() != State::kIdle) {
        res.successful = false;
        res.reason = n + " is IDLE_ONLY: mission must be IDLE";
        return res;
      }
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        res.successful = false;
        res.reason = n + " must be a double";
        return res;
      }
      const double v = p.as_double();
      const bool bad = !std::isfinite(v) || v <= 0.0 || (n == "state_publish_hz" && v > 100.0);
      if (bad) {
        res.successful = false;
        res.reason = n + " out of range";
        return res;
      }
      if (n == "state_publish_hz") next_hz = v;
      if (n == "gate_status_max_age_s") next_gate_age = v;
      if (n == "point_capture_radius_m") next_capture_radius = v;
      if (n == "rpp_ack_timeout_s") next_ack_timeout = v;
      if (n == "rpp_status_max_age_s") next_rpp_age = v;
    }
  }
  // Humble has no post-set callback. Validate the entire atomic request before changing any
  // effective value. Production and tests use a SingleThreadedExecutor, so the timer and parameter
  // callbacks cannot overlap. Construct a replacement timer before committing cached values.
  if (next_hz != state_publish_hz_) {
    auto next_timer =
        create_wall_timer(std::chrono::duration<double>(1.0 / next_hz), [this]() { on_timer(); });
    timer_->cancel();
    timer_ = std::move(next_timer);
  }
  state_publish_hz_ = next_hz;
  gate_max_age_s_ = next_gate_age;
  point_capture_radius_m_ = next_capture_radius;
  rpp_ack_timeout_s_ = next_ack_timeout;
  rpp_status_max_age_s_ = next_rpp_age;
  return res;
}

bool MissionNode::gate_ok(std::uint8_t* reason) {
  std::uint8_t why = MotionSetpointStatus::REASON_STALE;
  bool ok = false;
  if (gate_stamp_) {
    const double age = (get_clock()->now() - *gate_stamp_).seconds();
    if (age <= gate_max_age_s_ && age >= -gate_max_age_s_) {
      ok = gate_flag_;
      why = gate_flag_ ? static_cast<std::uint8_t>(MotionSetpointStatus::REASON_OK) : gate_reason_;
    }
  }
  if (reason != nullptr) *reason = why;
  return ok;
}

bool MissionNode::rpp_status_fresh() {
  if (!rpp_stamp_ns_) return false;
  const double age = static_cast<double>(now_ns() - *rpp_stamp_ns_) * 1e-9;
  return age >= 0.0 && age <= rpp_status_max_age_s_;  // a clock step back is not fresh either
}

void MissionNode::on_gate(const dyx3_interfaces::msg::SafetyGateStatus& m) {
  gate_stamp_ = get_clock()->now();  // freshness is measured on OUR clock at receipt
  gate_flag_ = m.ok;
  gate_reason_ = m.reason_code;
  evaluate_gate();
}

void MissionNode::evaluate_gate() {
  if (!(fsm_.state() == State::kLoading || fsm_.state() == State::kReady ||
        fsm_.state() == State::kRunning)) {
    return;
  }
  std::uint8_t why = 0;
  if (gate_ok(&why)) return;
  if (why == MotionSetpointStatus::REASON_ESTOP) {
    fsm_.estop(now_ns());  // E-stop aborts (contract: DERIVED decision)
  } else {
    fsm_.gate_lost(why, now_ns());
  }
  if (fsm_.terminal()) {
    publish_points(journal_ ? journal_->finish() : std::vector<PointEvent>{});
    finish_goal_if_terminal();
  }
  publish_state();
}

void MissionNode::on_rpp(const RppStatus& m) {
  if (m.mission_id != fsm_.mission_id()) return;  // only the current mission's RPP status counts
  // Freshness is measured on OUR clock at receipt.
  rpp_stamp_ns_ = now_ns();
  run_.run_index = m.run_index;
  // Review H4 / fix plan A2: ERROR and COMPLETE are evaluated before the READY acknowledgement;
  // only a state that shows RPP holds this mission's path acknowledges it.
  const bool rpp_holds_path =
      m.state == RppStatus::STATE_LOADED || m.state == RppStatus::STATE_TRACKING ||
      m.state == RppStatus::STATE_STOPPING || m.state == RppStatus::STATE_PIVOTING ||
      m.state == RppStatus::STATE_CREEPING;
  if (m.state == RppStatus::STATE_ERROR) {
    if (fsm_.rpp_error(now_ns()).transitioned) {
      ready_since_ns_.reset();
      publish_points(journal_ ? journal_->finish() : std::vector<PointEvent>{});
      finish_goal_if_terminal();
      publish_state();
    }
  } else if (fsm_.state() == State::kReady && rpp_holds_path) {
    std::uint8_t why = 0;
    const bool ok = gate_ok(&why);
    fsm_.rpp_ack(ok, why, now_ns());
    ready_since_ns_.reset();
    publish_state();
  } else if (m.state == RppStatus::STATE_COMPLETE && fsm_.state() != State::kReady) {
    if (fsm_.rpp_complete(now_ns()).transitioned) {
      publish_points(journal_ ? journal_->finish() : std::vector<PointEvent>{});
      finish_goal_if_terminal();
      publish_state();
    }
  }
}

void MissionNode::on_vehicle(const dyx3_interfaces::msg::VehicleState& m) {
  if (fsm_.state() != State::kRunning || !journal_ || !m.position_valid) return;
  publish_points(journal_->update(m.north_m, m.east_m));
  const auto active = journal_->active_point();
  run_.point_index = active ? *active : static_cast<std::uint32_t>(journal_->point_count());
}

void MissionNode::on_timer() {
  evaluate_gate();
  if (fsm_.state() == State::kRunning && !rpp_status_fresh()) {
    // RPP is silent while the mission says RUNNING (the guard has already stopped the rover on its
    // own command age). Pause; resuming is an explicit operator action, never automatic.
    RCLCPP_WARN(get_logger(), "RppStatus older than %.3f s while RUNNING: pausing",
                rpp_status_max_age_s_);
    fsm_.rpp_stale(now_ns());
  }
  if (goal_ && goal_->is_active() && goal_->is_canceling()) finish_goal_if_terminal();
  if (fsm_.state() == State::kReady && ready_since_ns_) {
    if (static_cast<double>(now_ns() - *ready_since_ns_) * 1e-9 >= rpp_ack_timeout_s_) {
      RCLCPP_ERROR(get_logger(), "RPP did not acknowledge the artifact within %.1f s",
                   rpp_ack_timeout_s_);
      fsm_.rpp_ack_timeout(now_ns());  // ERROR(INTERNAL): RPP never acknowledged the artifact
      ready_since_ns_.reset();
      finish_goal_if_terminal();
    }
  }
  publish_state();
}

void MissionNode::publish_state() {
  dyx3_interfaces::msg::MissionState m;
  m.stamp = get_clock()->now();
  m.state = static_cast<std::uint8_t>(fsm_.state());
  m.mission_id = fsm_.mission_id();
  m.run_index = run_.run_index;
  m.point_index = run_.point_index;
  m.reason_code = fsm_.reason();
  m.path_artifact_sha256 = run_.path_artifact_sha256;
  state_pub_->publish(m);
  if (goal_ && goal_->is_active()) {
    auto fb = std::make_shared<ExecuteMission::Feedback>();
    fb->run_index = run_.run_index;
    fb->point_index = run_.point_index;
    const double total = journal_ && journal_->point_count() > 0
                             ? static_cast<double>(journal_->point_count())
                             : 0.0;
    fb->progress_fraction =
        total > 0.0 ? static_cast<float>(std::min(1.0, run_.point_index / total)) : 0.0F;
    fb->mission_state = m.state;
    goal_->publish_feedback(fb);
  }
}

void MissionNode::publish_point(const PointEvent& ev) {
  dyx3_interfaces::msg::PointResult m;
  m.stamp = get_clock()->now();
  m.mission_id = fsm_.mission_id();
  m.point_index = ev.point_index;
  m.result_code = static_cast<std::uint8_t>(ev.outcome);
  m.north_m = static_cast<float>(ev.north_m);
  m.east_m = static_cast<float>(ev.east_m);
  m.error_m = static_cast<float>(ev.error_m);
  point_pub_->publish(m);
}

void MissionNode::publish_points(const std::vector<PointEvent>& evs) {
  for (const auto& e : evs) publish_point(e);
}

void MissionNode::finish_goal_if_terminal() {
  if (!goal_ || !goal_->is_active() || !fsm_.terminal()) return;
  auto result = std::make_shared<ExecuteMission::Result>();
  result->completed_runs = fsm_.state() == State::kCompleted ? run_.run_index + 1U : run_.run_index;
  switch (fsm_.state()) {
    case State::kCompleted:
      result->result_code = ExecuteMission::Result::RESULT_COMPLETED;
      goal_->succeed(result);
      break;
    case State::kAborted:
      result->result_code = ExecuteMission::Result::RESULT_ABORTED;
      if (goal_->is_canceling())
        goal_->canceled(result);
      else
        goal_->abort(result);
      break;
    default:
      result->result_code = ExecuteMission::Result::RESULT_ERROR;
      goal_->abort(result);
      break;
  }
  goal_.reset();
}

MissionNode::StartOutcome MissionNode::begin_mission(const std::string& sha) {
  StartOutcome out;
  std::uint8_t why = 0;
  const bool ok_gate = gate_ok(&why);
  const Result r = fsm_.start(ok_gate, now_ns());
  if (!r.accepted) {
    out.reason = r.reject == Reject::kBusy ? StartSrv::Response::REASON_BUSY
                                           : StartSrv::Response::REASON_SAFETY_GATE;
    out.mission_id = fsm_.mission_id();
    return out;
  }
  // From here the mission exists (LOADING) and has an id.
  out.mission_id = fsm_.mission_id();
  run_.clear();
  run_.mission_id = out.mission_id;
  artifact_.reset();
  journal_.reset();
  rpp_stamp_ns_.reset();

  ArtifactResult art;
  if (is_lower_hex64(sha)) {
    art = load_artifact(missions_dir_, sha);
  } else {
    art.error = "artifact id must be 64 lowercase hex characters";
  }
  if (!art.ok) {
    RCLCPP_ERROR(get_logger(), "artifact refused: %s", art.error.c_str());
    fsm_.artifact_loaded(false, now_ns());
    out.reason = StartSrv::Response::REASON_INVALID_ARTIFACT;
    publish_state();
    return out;
  }
  try {
    journal_ = std::make_unique<PointJournal>(art.artifact.points, point_capture_radius_m_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "point journal refused: %s", e.what());
    fsm_.artifact_loaded(false, now_ns());
    out.reason = StartSrv::Response::REASON_INVALID_ARTIFACT;
    publish_state();
    return out;
  }
  artifact_ = std::move(art.artifact);
  run_.path_artifact_sha256 = artifact_->sha256;
  const auto active = journal_->active_point();
  run_.point_index = active ? *active : 0U;
  fsm_.artifact_loaded(true, now_ns());
  ready_since_ns_ = now_ns();
  out.accepted = true;
  out.reason = StartSrv::Response::REASON_OK;
  publish_state();
  return out;
}

}  // namespace dyx3_mission
