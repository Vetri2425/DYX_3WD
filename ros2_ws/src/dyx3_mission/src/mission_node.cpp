// mission_node — see docs/contracts/dyx3_mission.md
#include "dyx3_mission/mission_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <utility>

#include "dyx3_interfaces/msg/motion_setpoint_status.hpp"

namespace dyx3_mission {

namespace {
using dyx3_interfaces::msg::MissionState;
using dyx3_interfaces::msg::MotionSetpointStatus;
using dyx3_interfaces::msg::RppStatus;
using StartSrv = dyx3_interfaces::srv::StartMission;
using ResumeSrv = dyx3_interfaces::srv::ResumeMission;

static_assert(kArmReasonTimeout == dyx3_interfaces::srv::ArmDisarm::Response::REASON_TIMEOUT);
static_assert(static_cast<std::uint8_t>(State::kPlacing) == MissionState::STATE_PLACING);
static_assert(static_cast<std::uint8_t>(State::kArming) == MissionState::STATE_ARMING);
static_assert(static_cast<std::uint8_t>(State::kEngaging) == MissionState::STATE_ENGAGING);
static_assert(kReasonRppStale == MissionState::REASON_RPP_STALE);
static_assert(kReasonNoPlacementFrame == MissionState::REASON_NO_PLACEMENT_FRAME);
static_assert(kReasonEstop == MissionState::REASON_ESTOP);
static_assert(kReasonRppPivotTimeout == MissionState::REASON_RPP_PIVOT_TIMEOUT);

// px4_link's own confirmation windows (docs/contracts/dyx3_px4_link.md): arm_confirm_timeout_s
// 2.0 s; set_offboard answers by prestream 0.5 s + confirm 2.0 s + 1.0 s at the latest. The
// mission's timeouts must be longer, so px4_link's definitive answer always arrives first.
constexpr double kPx4LinkArmConfirmS = 2.0;
constexpr double kPx4LinkOffboardAnswerS = 3.5;
constexpr std::size_t kMaxRequestIdLength = 64;

bool is_lower_hex64(const std::string& s) {
  if (s.size() != 64) return false;
  for (char c : s) {
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  }
  return true;
}

bool valid_request_id(const std::string& s) {
  if (s.size() > kMaxRequestIdLength) return false;
  for (char c : s) {
    const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
                    c == '.' || c == '_' || c == ':' || c == '-';
    if (!ok) return false;
  }
  return true;
}

// The one validation of every IDLE_ONLY double parameter (construction and runtime alike).
bool valid_value(const std::string& n, double v) {
  if (!std::isfinite(v) || v <= 0.0) return false;
  if (n == "state_publish_hz") return v <= 100.0;
  if (n == "arm_timeout_s") return v > kPx4LinkArmConfirmS && v <= 60.0;
  if (n == "offboard_timeout_s") return v > kPx4LinkOffboardAnswerS && v <= 60.0;
  if (n == "placement_max_distance_m") return v <= 1000.0;
  return true;
}

const std::vector<std::string>& idle_only_params() {
  static const std::vector<std::string> names = {
      "state_publish_hz",   "gate_status_max_age_s",    "point_capture_radius_m",
      "rpp_ack_timeout_s",  "rpp_status_max_age_s",     "arm_timeout_s",
      "offboard_timeout_s", "placement_max_distance_m", "vehicle_state_max_age_s"};
  return names;
}

std::uint8_t placement_reason(PlacementError e) {
  switch (e) {
    case PlacementError::kNoPlacementFrame:
      return kReasonNoPlacementFrame;
    case PlacementError::kReferenceInvalid:
      return kReasonEkfReferenceInvalid;
    case PlacementError::kOutOfBounds:
      return kReasonPlacementOutOfBounds;
    default:
      return kReasonInternalError;
  }
}

std::int64_t steady_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

builtin_interfaces::msg::Time to_msg_time(std::int64_t ns) {
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<std::int32_t>(ns / 1'000'000'000LL);
  t.nanosec = static_cast<std::uint32_t>(ns % 1'000'000'000LL);
  return t;
}
}  // namespace

MissionNode::MissionNode(const rclcpp::NodeOptions& options, ClockFn clock)
    : rclcpp::Node("dyx3_mission", options),
      clock_(clock ? std::move(clock) : ClockFn(steady_now_ns)) {
  // Parameter classes (spec section 9): missions_dir RESTART, the rest IDLE_ONLY. Defaults and
  // their sources: docs/contracts/dyx3_mission.md section 9.
  missions_dir_ = declare_parameter<std::string>("missions_dir", "/var/lib/dyx3/missions");
  state_publish_hz_ = declare_parameter<double>("state_publish_hz", 10.0);
  gate_max_age_s_ = declare_parameter<double>("gate_status_max_age_s", 0.5);
  point_capture_radius_m_ = declare_parameter<double>("point_capture_radius_m", 0.10);
  rpp_ack_timeout_s_ = declare_parameter<double>("rpp_ack_timeout_s", 30.0);
  rpp_status_max_age_s_ = declare_parameter<double>("rpp_status_max_age_s", 0.5);
  arm_timeout_s_ = declare_parameter<double>("arm_timeout_s", 4.0);
  offboard_timeout_s_ = declare_parameter<double>("offboard_timeout_s", 5.0);
  placement_max_distance_m_ = declare_parameter<double>("placement_max_distance_m", 1000.0);
  vehicle_state_max_age_s_ = declare_parameter<double>("vehicle_state_max_age_s", 0.5);
  for (const auto& n : idle_only_params()) {
    if (!valid_value(n, get_parameter(n).as_double())) {
      throw std::invalid_argument("dyx3_mission: invalid parameter value: " + n);
    }
  }
  px4_.set_timeouts({arm_timeout_s_, offboard_timeout_s_});
  // Persisted progress (contract section 9a). DERIVED — NOT FROM V1 SPEC: read once here, before
  // the executor runs, so StartMission(resume) decides from memory (no file I/O in the callback).
  {
    std::vector<std::string> warnings;
    progress_ = load_progress_dir(missions_dir_, &warnings);
    for (const auto& w : warnings)
      RCLCPP_WARN(get_logger(), "progress record ignored: %s", w.c_str());
    if (!progress_.empty()) {
      RCLCPP_INFO(get_logger(), "%zu persisted mission progress record(s)", progress_.size());
    }
  }
  param_cb_ = add_on_set_parameters_callback(
      std::bind(&MissionNode::on_parameters, this, std::placeholders::_1));

  // Every transition is logged and published at once (the gateway pushes it to the tablet).
  fsm_.set_observer([this](const Transition& t) {
    // The FSM times its transitions on the steady clock; the message's state_entered is ROS time.
    // A state change (a PAUSED self-transition on an EKF reset included) is the one thing that
    // moves fsm_.changes(): record the ROS time of it here.
    if (fsm_.changes() != stamped_changes_) {
      stamped_changes_ = fsm_.changes();
      state_entered_ros_ns_ = get_clock()->now().nanoseconds();
    }
    RCLCPP_INFO(get_logger(), "mission %u %s: %s -> %s on %s reason=%u%s%s", fsm_.mission_id(),
                t.refused ? "REFUSED" : "transition", to_string(t.from), to_string(t.to),
                to_string(t.event), static_cast<unsigned>(t.reason), t.detail.empty() ? "" : " : ",
                t.detail.c_str());
    if (t.refused || (t.from == t.to && t.event != Event::kEkfReset)) return;
    // A terminal state's entry actions never transition, so they run here, before the message:
    // the tablet sees ERROR with the execution already unloaded and the release already queued.
    if (fsm_.terminal()) on_terminal_entry();
    publish_state();
  });

  const auto sensor_qos = rclcpp::QoS(1).best_effort();
  // Depth 20: every transition is its own message (several can follow each other within one
  // callback); a subscriber that wants them all (the gateway) must keep a depth > 1 too.
  state_pub_ = create_publisher<MissionState>("/dyx3/mission/state", rclcpp::QoS(20).reliable());
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
  arm_cli_ = create_client<ArmSrv>("/dyx3/px4_link/arm");
  offboard_cli_ = create_client<OffboardSrv>("/dyx3/px4_link/set_offboard");

  start_srv_ = create_service<StartSrv>(
      "/dyx3/mission/start", [this](const std::shared_ptr<StartSrv::Request> req,
                                    std::shared_ptr<StartSrv::Response> res) {
        const auto o = begin_mission(req->path_artifact_sha256, req->request_id, req->resume);
        res->accepted = o.accepted;
        res->reason_code = o.reason;
        res->mission_id = o.mission_id;
        res->duplicate = o.duplicate;
        res->gate_reason_code = o.gate_reason;
        res->resumed_run_index = o.resumed_run_index;
      });
  pause_srv_ = create_service<dyx3_interfaces::srv::PauseMission>(
      "/dyx3/mission/pause",
      [this](const std::shared_ptr<dyx3_interfaces::srv::PauseMission::Request>,
             std::shared_ptr<dyx3_interfaces::srv::PauseMission::Response> res) {
        // PAUSED keeps the vehicle armed and in OFFBOARD; RPP and the guard stream STOP.
        const Result r = fsm_.pause(steady_ns());
        res->accepted = r.accepted;
        res->reason_code = r.accepted ? res->REASON_OK : res->REASON_NOT_RUNNING;
        advance();
      });
  resume_srv_ = create_service<ResumeSrv>(
      "/dyx3/mission/resume",
      [this](const std::shared_ptr<ResumeSrv::Request>, std::shared_ptr<ResumeSrv::Response> res) {
        // Resume needs the FULL gate (armed + OFFBOARD included), a fresh RppStatus (otherwise it
        // would pause again on the next tick) and the EKF reference the path was placed with.
        // When ONLY the arming gate fails (the vehicle lost arm / OFFBOARD while paused) and the
        // pre-arm gate is ok, the resume re-engages: ARMING -> ENGAGING -> RUNNING, exactly the
        // steps of a start (contract section 3a). Never automatic: only this operator call.
        std::uint8_t why = 0;
        const bool paused = fsm_.state() == State::kPaused;
        const bool reference_ok = !paused || reference_matches_placement();
        const bool full = gate_ok(&why);
        const bool pre = pre_arm_ok();
        const bool rpp_ok = rpp_status_fresh();
        const bool reengage = paused && reference_ok && rpp_ok && !full && pre &&
                              why == MotionSetpointStatus::REASON_ARMING_GATE;
        const Result r = reengage ? fsm_.reengage(steady_ns())
                                  : fsm_.resume(reference_ok && full && rpp_ok, steady_ns());
        res->accepted = r.accepted;
        if (r.accepted) {
          res->reason_code = ResumeSrv::Response::REASON_OK;
          if (r.transitioned) {
            // The operator saw the EKF reset and resumes on the same reference: re-baseline.
            if (placement_ && vehicle_) placement_->xy_reset_counter = vehicle_->xy_reset_counter;
            ekf_reset_reported_ = false;
          }
        } else if (r.reject == Reject::kNotPaused) {
          res->reason_code = ResumeSrv::Response::REASON_NOT_PAUSED;
        } else if (!reference_ok) {
          res->reason_code = ResumeSrv::Response::REASON_EKF_REFERENCE_CHANGED;
        } else if (!full && why == MotionSetpointStatus::REASON_ARMING_GATE) {
          res->reason_code = ResumeSrv::Response::REASON_NOT_ARMED_OR_OFFBOARD;
        } else {
          res->reason_code = ResumeSrv::Response::REASON_SAFETY_GATE;
        }
        advance();
      });
  abort_srv_ = create_service<dyx3_interfaces::srv::AbortMission>(
      "/dyx3/mission/abort",
      [this](const std::shared_ptr<dyx3_interfaces::srv::AbortMission::Request> req,
             std::shared_ptr<dyx3_interfaces::srv::AbortMission::Response> res) {
        const Result r = fsm_.abort(req->reason_code, steady_ns());
        res->accepted = r.accepted;
        res->reason_code = r.accepted ? res->REASON_OK : res->REASON_NOT_ACTIVE;
        advance();
      });
  skip_srv_ = create_service<dyx3_interfaces::srv::SkipPoint>(
      "/dyx3/mission/skip_point",
      [this](const std::shared_ptr<dyx3_interfaces::srv::SkipPoint::Request>,
             std::shared_ptr<dyx3_interfaces::srv::SkipPoint::Response> res) {
        const bool has_point = journal_ && journal_->active_point().has_value();
        const Result r = fsm_.skip_point(has_point, steady_ns());
        res->accepted = r.accepted;
        if (r.accepted) {
          const auto ev = journal_->skip();
          res->reason_code = res->REASON_OK;
          res->skipped_point_index = ev ? ev->point_index : 0U;
          if (ev) {
            publish_point(*ev);
            const auto active = journal_->active_point();
            run_.point_index =
                active ? *active : static_cast<std::uint32_t>(journal_->point_count());
            record_resolved({*ev});
          }
        } else {
          res->reason_code = r.reject == Reject::kNotRunning ? res->REASON_NOT_RUNNING
                                                             : res->REASON_NO_ACTIVE_POINT;
        }
        publish_state();
      });

  action_ = rclcpp_action::create_server<ExecuteMission>(
      this, "/dyx3/mission/execute",
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteMission::Goal> goal) {
        // One mission at a time; begin_mission() is the single start path (no request id: an
        // action goal has its own UUID).
        const auto o = begin_mission(goal->path_artifact_sha256, "", false);
        return o.accepted ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                          : rclcpp_action::GoalResponse::REJECT;
      },
      [this](const std::shared_ptr<GoalHandle>) {
        // Cancel == AbortMission with REASON_OPERATOR. The goal is not CANCELING yet inside this
        // callback (finishing here would report abort, not canceled): on_timer finalises it
        // (review H3 / fix plan A3).
        cancel_pending_ = true;
        fsm_.abort(kReasonOperator, steady_ns());
        advance();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> gh) {
        goal_ = gh;
        cancel_pending_ = false;
        finish_goal_if_terminal();
      });

  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / state_publish_hz_),
                             [this]() { on_timer(); });
  // Polls the file jobs while one runs; cancelled otherwise (an idle node does not wake up).
  work_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() { on_work_timer(); });
  work_timer_->cancel();
  RCLCPP_INFO(get_logger(), "dyx3_mission up (missions_dir=%s)", missions_dir_.c_str());
}

MissionNode::~MissionNode() {
  // std::future from std::async joins in its destructor: a running file job finishes first.
  if (load_job_.valid()) load_job_.wait();
  if (place_job_.valid()) place_job_.wait();
  // The newest progress of every artifact reaches the disk on a clean shutdown (a destructor, not a
  // callback: writing here cannot stall the executor).
  if (progress_job_.valid()) {
    for (const auto& e : progress_job_.get()) {
      RCLCPP_ERROR(get_logger(), "progress not persisted: %s", e.c_str());
    }
  }
  for (const auto& kv : progress_queue_) {
    std::string e;
    if (!write_progress(missions_dir_, kv.second, &e)) {
      RCLCPP_ERROR(get_logger(), "progress not persisted: %s", e.c_str());
    }
  }
}

rcl_interfaces::msg::SetParametersResult MissionNode::on_parameters(
    const std::vector<rclcpp::Parameter>& ps) {
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  std::map<std::string, double> next;
  for (const auto& n : idle_only_params()) next[n] = get_parameter(n).as_double();
  for (const auto& p : ps) {
    const std::string& n = p.get_name();
    if (n == "missions_dir") {
      res.successful = false;
      res.reason = "missions_dir is RESTART-class";
      return res;
    }
    if (next.count(n) == 0) continue;
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
    if (!valid_value(n, p.as_double())) {
      res.successful = false;
      res.reason = n + " out of range";
      return res;
    }
    next[n] = p.as_double();
  }
  // Humble has no post-set callback. Validate the entire atomic request before changing any
  // effective value. Production and tests use a SingleThreadedExecutor, so the timer and parameter
  // callbacks cannot overlap. Construct a replacement timer before committing cached values.
  if (next["state_publish_hz"] != state_publish_hz_) {
    auto next_timer = create_wall_timer(
        std::chrono::duration<double>(1.0 / next["state_publish_hz"]), [this]() { on_timer(); });
    timer_->cancel();
    timer_ = std::move(next_timer);
  }
  state_publish_hz_ = next["state_publish_hz"];
  gate_max_age_s_ = next["gate_status_max_age_s"];
  point_capture_radius_m_ = next["point_capture_radius_m"];
  rpp_ack_timeout_s_ = next["rpp_ack_timeout_s"];
  rpp_status_max_age_s_ = next["rpp_status_max_age_s"];
  arm_timeout_s_ = next["arm_timeout_s"];
  offboard_timeout_s_ = next["offboard_timeout_s"];
  placement_max_distance_m_ = next["placement_max_distance_m"];
  vehicle_state_max_age_s_ = next["vehicle_state_max_age_s"];
  px4_.set_timeouts({arm_timeout_s_, offboard_timeout_s_});
  return res;
}

// ------------------------------------------------------------------------------------------------
// inputs
// ------------------------------------------------------------------------------------------------
bool MissionNode::gate_ok(std::uint8_t* reason) {
  std::uint8_t why = MotionSetpointStatus::REASON_STALE;
  bool ok = false;
  if (gate_stamp_ns_) {
    const double age = static_cast<double>(steady_ns() - *gate_stamp_ns_) * 1e-9;
    if (age <= gate_max_age_s_ && age >= -gate_max_age_s_) {
      ok = gate_flag_;
      why = gate_flag_ ? static_cast<std::uint8_t>(MotionSetpointStatus::REASON_OK) : gate_reason_;
    }
  }
  if (reason != nullptr) *reason = why;
  return ok;
}

bool MissionNode::pre_arm_ok(std::uint8_t* reason) {
  std::uint8_t why = MotionSetpointStatus::REASON_STALE;
  bool ok = false;
  if (gate_stamp_ns_) {
    const double age = static_cast<double>(steady_ns() - *gate_stamp_ns_) * 1e-9;
    if (age <= gate_max_age_s_ && age >= -gate_max_age_s_) {
      ok = pre_arm_flag_;
      why = pre_arm_flag_ ? static_cast<std::uint8_t>(MotionSetpointStatus::REASON_OK)
                          : pre_arm_reason_;
    }
  }
  if (reason != nullptr) *reason = why;
  return ok;
}

bool MissionNode::vehicle_fresh() {
  if (!vehicle_) return false;
  const double age = static_cast<double>(steady_ns() - vehicle_stamp_ns_) * 1e-9;
  return age >= 0.0 && age <= vehicle_state_max_age_s_;
}

bool MissionNode::rpp_status_fresh() {
  if (!rpp_stamp_ns_) return false;
  const double age = static_cast<double>(steady_ns() - *rpp_stamp_ns_) * 1e-9;
  return age >= 0.0 && age <= rpp_status_max_age_s_;  // a clock step back is not fresh either
}

bool MissionNode::reference_matches_placement() {
  if (!placement_ || !vehicle_fresh()) return false;
  if (!placement_->anchored) return true;
  return vehicle_->global_reference_valid &&
         vehicle_->reference_latitude_deg == placement_->ref_lat_deg &&
         vehicle_->reference_longitude_deg == placement_->ref_lon_deg;
}

void MissionNode::on_gate(const dyx3_interfaces::msg::SafetyGateStatus& m) {
  gate_stamp_ns_ = steady_ns();  // freshness: steady clock at receipt, not the message stamp
  gate_flag_ = m.ok;
  gate_reason_ = m.reason_code;
  pre_arm_flag_ = m.pre_arm_ok;
  pre_arm_reason_ = m.pre_arm_reason_code;
  evaluate_gate();
  advance();
}

void MissionNode::evaluate_gate() {
  if (!fsm_.active()) return;
  std::uint8_t pre_why = 0;
  std::uint8_t full_why = 0;
  const bool pre = pre_arm_ok(&pre_why);
  const bool full = gate_ok(&full_why);
  if (pre_why == MotionSetpointStatus::REASON_ESTOP ||
      full_why == MotionSetpointStatus::REASON_ESTOP) {
    fsm_.estop(steady_ns());  // E-stop aborts and disarms (owner decision), in every active state
    return;
  }
  if (fsm_.before_running()) {
    // Before motion the vehicle may be disarmed: the pre-arm gate is the one that must hold.
    if (!pre) {
      fsm_.gate_lost(pre_why, steady_ns());
    } else if (full && fsm_.awaiting_reengage_gate()) {
      fsm_.reengage_gate(true, steady_ns());  // re-engaged and the guard sees armed + OFFBOARD
    }
  } else if (fsm_.state() == State::kRunning) {
    if (!full) fsm_.gate_lost(full_why, steady_ns());
  }
}

void MissionNode::on_rpp(const RppStatus& m) {
  if (m.mission_id != fsm_.mission_id()) return;  // only the current mission's RPP status counts
  // Freshness: steady clock at receipt, not the message stamp.
  rpp_stamp_ns_ = steady_ns();
  run_.run_index = m.run_index;
  // Review H4 / fix plan A2: ERROR and COMPLETE are evaluated before the READY acknowledgement;
  // only a state that shows RPP holds this mission's path acknowledges it.
  const bool rpp_holds_path =
      m.state == RppStatus::STATE_LOADED || m.state == RppStatus::STATE_TRACKING ||
      m.state == RppStatus::STATE_STOPPING || m.state == RppStatus::STATE_PIVOTING ||
      m.state == RppStatus::STATE_CREEPING;
  // The run RPP reports is the one in progress (runs are driven in order): every earlier run is
  // complete, which is what a resume restarts from (contract section 9a).
  const bool driven =
      fsm_.state() == State::kRunning || fsm_.state() == State::kPaused || fsm_.reengaging();
  if (driven && rpp_holds_path && m.run_index > progress_run_) {
    progress_run_ = m.run_index;
    note_progress();
  }
  if (m.state == RppStatus::STATE_ERROR) {
    fsm_.rpp_error(steady_ns());
  } else if (fsm_.state() == State::kReady && rpp_holds_path) {
    // Nothing reaches RUNNING without the full gate (armed + OFFBOARD included).
    fsm_.rpp_ack(gate_ok(), steady_ns());
  } else if (fsm_.state() == State::kRunning && m.pivot_timed_out && !pivot_timeout_latched_) {
    // DERIVED — NOT FROM V1 SPEC: one pause per watchdog expiry. RPP keeps the flag up while the
    // heading is still outside the release band; once the operator resumed after this pause, the
    // same expiry does not pause again (only a new one, after RPP reported false, does).
    pivot_timeout_latched_ = true;
    RCLCPP_WARN(get_logger(), "rpp pivot watchdog expired while RUNNING (run %u): pausing",
                m.run_index);
    fsm_.rpp_pivot_timeout(steady_ns());
  } else if (m.state == RppStatus::STATE_COMPLETE && fsm_.state() != State::kReady) {
    fsm_.rpp_complete(steady_ns());
  }
  if (!m.pivot_timed_out) pivot_timeout_latched_ = false;
  advance();
}

void MissionNode::on_vehicle(const dyx3_interfaces::msg::VehicleState& m) {
  vehicle_ = m;
  vehicle_stamp_ns_ = steady_ns();
  check_ekf_reset();
  if (fsm_.state() == State::kRunning && journal_ && m.position_valid) {
    const auto evs = journal_->update(m.north_m, m.east_m);
    publish_points(evs);
    const auto active = journal_->active_point();
    run_.point_index = active ? *active : static_cast<std::uint32_t>(journal_->point_count());
    record_resolved(evs);
  }
  advance();
}

void MissionNode::check_ekf_reset() {
  const State s = fsm_.state();
  const bool placed_state = s == State::kArming || s == State::kEngaging || s == State::kReady ||
                            s == State::kRunning || s == State::kPaused;
  if (!placement_ || !vehicle_ || !placed_state || ekf_reset_reported_) return;
  const auto& v = *vehicle_;
  std::string why;
  if (v.xy_reset_counter != placement_->xy_reset_counter) {
    why = "EKF xy reset (counter " + std::to_string(placement_->xy_reset_counter) + " -> " +
          std::to_string(v.xy_reset_counter) + ")";
  } else if (placement_->anchored &&
             (!v.global_reference_valid || v.reference_latitude_deg != placement_->ref_lat_deg ||
              v.reference_longitude_deg != placement_->ref_lon_deg)) {
    why = "EKF global reference changed or became invalid since placement";
  }
  if (why.empty()) return;
  ekf_reset_reported_ = true;  // reported once; a resume re-baselines
  RCLCPP_WARN(get_logger(), "%s", why.c_str());
  fsm_.ekf_reset(why, steady_ns());
}

void MissionNode::on_timer() {
  evaluate_gate();
  if (fsm_.state() == State::kRunning && !rpp_status_fresh()) {
    // RPP is silent while the mission says RUNNING (the guard has already stopped the rover on its
    // own command age). Pause; resuming is an explicit operator action, never automatic.
    RCLCPP_WARN(get_logger(), "RppStatus older than %.3f s while RUNNING: pausing",
                rpp_status_max_age_s_);
    fsm_.rpp_stale(steady_ns());
  }
  if (fsm_.state() == State::kReady && ready_since_ns_ &&
      static_cast<double>(steady_ns() - *ready_since_ns_) * 1e-9 >= rpp_ack_timeout_s_) {
    std::uint8_t why = 0;
    const bool full = gate_ok(&why);
    RCLCPP_ERROR(get_logger(), "READY for %.1f s without RUNNING (rpp ack seen: %s, gate ok: %s)",
                 rpp_ack_timeout_s_, rpp_stamp_ns_ ? "yes" : "no", full ? "yes" : "no");
    fsm_.rpp_ack_timeout(full, why, steady_ns());
  }
  if (fsm_.awaiting_reengage_gate() && reengage_confirmed_ns_ &&
      static_cast<double>(steady_ns() - *reengage_confirmed_ns_) * 1e-9 >= offboard_timeout_s_) {
    // DERIVED — NOT FROM V1 SPEC: the hold after the re-engage's OFFBOARD confirmation is bounded
    // by the engage step's own timeout (offboard_timeout_s); the outcome mirrors READY's
    // rpp_ack_timeout with a full gate that never passed.
    std::uint8_t why = 0;
    gate_ok(&why);
    RCLCPP_ERROR(get_logger(),
                 "re-engaged but the full gate did not pass within %.1f s (reason %u)",
                 offboard_timeout_s_, static_cast<unsigned>(why));
    fsm_.reengage_timeout(why, steady_ns());
  }
  if (goal_ && goal_->is_active() && goal_->is_canceling()) finish_goal_if_terminal();
  advance();
  publish_state();
}

void MissionNode::on_work_timer() {
  advance();
  if (!jobs_pending() && !progress_job_.valid()) work_timer_->cancel();
}

// ------------------------------------------------------------------------------------------------
// admission
// ------------------------------------------------------------------------------------------------
MissionNode::StartOutcome MissionNode::begin_mission(const std::string& sha,
                                                     const std::string& request_id, bool resume) {
  StartOutcome out;
  out.mission_id = fsm_.mission_id();
  if (!valid_request_id(request_id)) {
    out.reason = StartSrv::Response::REASON_INVALID_REQUEST;
    RCLCPP_WARN(get_logger(), "start refused: invalid request_id");
    return out;
  }
  // Idempotency: the request that created the most recent execution gets that execution back.
  if (!request_id.empty() && fsm_.mission_id() != 0 && request_id == run_.request_id) {
    out.accepted = true;
    out.duplicate = true;
    out.reason = StartSrv::Response::REASON_OK;
    out.resumed_run_index = run_.start_run_index;
    RCLCPP_INFO(get_logger(), "start request %s is a duplicate of mission %u", request_id.c_str(),
                out.mission_id);
    return out;
  }
  if (!is_lower_hex64(sha)) {
    out.reason = StartSrv::Response::REASON_INVALID_ARTIFACT;
    RCLCPP_WARN(get_logger(), "start refused: artifact id must be 64 lowercase hex characters");
    return out;
  }
  if (!fsm_.active() && (px4_.busy() || jobs_pending())) {
    // The previous execution is still releasing OFFBOARD / disarming (or a file job of it runs).
    out.reason = StartSrv::Response::REASON_BUSY;
    RCLCPP_WARN(get_logger(), "start refused: the previous execution is still being released");
    return out;
  }
  std::uint8_t why = 0;
  const bool pre = pre_arm_ok(&why);
  // Resume: the persisted progress of this artifact, unless it is complete (from memory, no I/O).
  std::uint32_t start_run = 0;
  std::vector<std::uint32_t> restored;
  if (resume) {
    const auto it = progress_.find(sha);
    if (it != progress_.end() && !it->second.complete()) {
      start_run = it->second.run_index;
      restored = it->second.completed_points;
    }
  }
  // The new execution's identity must be in place before the transition publishes it.
  const RunState previous = run_;
  run_.clear();
  run_.mission_id = fsm_.mission_id() + 1;
  run_.source_artifact_sha256 = sha;
  run_.request_id = request_id;
  run_.start_run_index = start_run;  // MissionState.start_run_index from LOADING on
  run_.run_index = start_run;
  run_.point_index = static_cast<std::uint32_t>(restored.size());  // the first unresolved point
  const Result r = fsm_.start(pre, steady_ns());
  if (!r.accepted) {
    run_ = previous;
    out.reason = r.reject == Reject::kBusy ? StartSrv::Response::REASON_BUSY
                                           : StartSrv::Response::REASON_SAFETY_GATE;
    if (r.reject == Reject::kSafetyGate) out.gate_reason = why;
    return out;
  }
  out.mission_id = fsm_.mission_id();
  artifact_.reset();
  journal_.reset();
  placement_.reset();
  ekf_reset_reported_ = false;
  release_note_.clear();
  rpp_stamp_ns_.reset();
  ready_since_ns_.reset();
  reengage_confirmed_ns_.reset();
  pivot_timeout_latched_ = false;
  px4_.begin_execution();
  // Progress is written from PLACING on (the artifact verified); resume=false overwrites it then.
  progress_tracking_ = false;
  progress_run_ = start_run;
  resume_points_ = restored;
  completed_ = std::move(restored);
  out.accepted = true;
  out.reason = StartSrv::Response::REASON_OK;
  out.resumed_run_index = start_run;
  if (resume) {
    RCLCPP_INFO(get_logger(), "mission %u: resume of %s from run %u (%zu points already resolved)",
                out.mission_id, sha.c_str(), start_run, resume_points_.size());
  }
  advance();  // LOADING's entry action only launches the read: no file I/O here
  return out;
}

// ------------------------------------------------------------------------------------------------
// lifecycle
// ------------------------------------------------------------------------------------------------
void MissionNode::advance() {
  poll_jobs();
  for (const auto& o : px4_.on_tick(steady_ns())) {
    const auto it = px4_pending_.find(o.id);
    if (it != px4_pending_.end()) {
      if (it->second.arm_client) {
        arm_cli_->remove_pending_request(it->second.client_request_id);
      } else {
        offboard_cli_->remove_pending_request(it->second.client_request_id);
      }
      px4_pending_.erase(it);
    }
    handle_px4(o);
  }
  // Entry actions, once per state change, including changes they cause themselves.
  bool changed = false;
  while (entered_changes_ != fsm_.changes()) {
    entered_changes_ = fsm_.changes();
    enter(fsm_.state(), steady_ns());
    changed = true;
  }
  if (changed) note_progress();  // every state change of a tracked execution is persisted
  while (const auto req = px4_.next(steady_ns())) send_px4(*req);
  if (dirty_) publish_state();
}

void MissionNode::enter(State s, std::int64_t now) {
  dirty_ = true;
  switch (s) {
    case State::kLoading: {
      // Read + hash off the executor thread: a large artifact cannot stall the node.
      job_mission_id_ = fsm_.mission_id();
      load_job_ = std::async(std::launch::async,
                             [dir = missions_dir_, sha = run_.source_artifact_sha256]() {
                               return load_artifact(dir, sha);
                             });
      work_timer_->reset();
      break;
    }
    case State::kPlacing: {
      if (!artifact_) {
        fsm_.placed(false, kReasonInternalError, "no loaded artifact", now);
        break;
      }
      if (!vehicle_fresh()) {
        fsm_.placed(false, kReasonEkfReferenceInvalid, "no fresh VehicleState to place against",
                    now);
        break;
      }
      EkfReference ref;
      ref.global_reference_valid = vehicle_->global_reference_valid;
      ref.lat_deg = vehicle_->reference_latitude_deg;
      ref.lon_deg = vehicle_->reference_longitude_deg;
      PlacementRecord rec;
      rec.ref_lat_deg = ref.lat_deg;
      rec.ref_lon_deg = ref.lon_deg;
      rec.xy_reset_counter = vehicle_->xy_reset_counter;
      placement_ = rec;  // anchored is filled in when the placement completes
      job_mission_id_ = fsm_.mission_id();
      place_job_ = std::async(std::launch::async, [src = *artifact_, ref, dir = missions_dir_,
                                                   max = placement_max_distance_m_]() {
        PlaceResult out;
        out.placement = place_artifact(src, ref, max);
        if (out.placement.ok) {
          out.stored = store_execution_artifact(dir, out.placement, &out.store_error);
        }
        return out;
      });
      work_timer_->reset();
      break;
    }
    case State::kArming:
    case State::kEngaging: {
      // Never arm (or engage) unless every pre-arm gate is ok at this instant.
      std::uint8_t why = 0;
      if (!pre_arm_ok(&why)) {
        if (why == MotionSetpointStatus::REASON_ESTOP) {
          fsm_.estop(now);
        } else {
          fsm_.gate_lost(why, now);
        }
        break;
      }
      px4_.engage(s == State::kArming ? Px4Op::kArm : Px4Op::kOffboardOn);
      break;
    }
    case State::kReady:
      ready_since_ns_ = now;
      rpp_stamp_ns_.reset();
      break;
    case State::kRunning:
    case State::kPaused:
    case State::kIdle:
    case State::kCompleted:  // terminal entry actions: on_terminal_entry(), from the observer
    case State::kAborted:
    case State::kError:
      break;
  }
}

void MissionNode::on_terminal_entry() {
  dirty_ = true;
  publish_points(journal_ ? journal_->finish() : std::vector<PointEvent>{});
  ready_since_ns_.reset();
  if (fsm_.state() == State::kError) {
    // Unload the execution: nothing can drive it later (RPP unloads on a non-active state and the
    // id is withdrawn from MissionState).
    run_.path_artifact_sha256.clear();
    artifact_.reset();
    journal_.reset();
  }
  // COMPLETED / ABORTED: set_offboard(false) then arm(false). ERROR: release what this execution
  // engaged / armed. E-stop: always disarm. The requests go out in advance().
  px4_.release(fsm_.reason() == kReasonEstop);
  finish_goal_if_terminal();
}

void MissionNode::poll_jobs() {
  using namespace std::chrono_literals;
  if (progress_job_.valid() && progress_job_.wait_for(0s) == std::future_status::ready) {
    for (const auto& e : progress_job_.get()) {
      RCLCPP_ERROR(get_logger(), "progress not persisted: %s", e.c_str());
    }
    launch_progress_job();  // the records queued meanwhile (newest per artifact)
  }
  if (load_job_.valid() && load_job_.wait_for(0s) == std::future_status::ready) {
    ArtifactResult r = load_job_.get();
    if (job_mission_id_ == fsm_.mission_id() && fsm_.state() == State::kLoading) {
      if (!r.ok) {
        RCLCPP_ERROR(get_logger(), "artifact refused: %s", r.error.c_str());
        fsm_.artifact_loaded(false, r.error, steady_ns());
      } else {
        artifact_ = std::move(r.artifact);
        progress_tracking_ = true;  // a verified artifact: its progress is persisted from now on
        fsm_.artifact_loaded(true, "artifact verified", steady_ns());
      }
    }
  }
  if (place_job_.valid() && place_job_.wait_for(0s) == std::future_status::ready) {
    PlaceResult r = place_job_.get();
    if (job_mission_id_ != fsm_.mission_id() || fsm_.state() != State::kPlacing) return;
    Placement& p = r.placement;
    if (!p.ok) {
      RCLCPP_ERROR(get_logger(), "placement refused: %s", p.detail.c_str());
      fsm_.placed(false, placement_reason(p.error), p.detail, steady_ns());
      return;
    }
    if (!r.stored) {
      RCLCPP_ERROR(get_logger(), "execution artifact not stored: %s", r.store_error.c_str());
      fsm_.placed(false, kReasonInternalError, r.store_error, steady_ns());
      return;
    }
    try {
      journal_ = std::make_unique<PointJournal>(p.execution.points, point_capture_radius_m_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "point journal refused: %s", e.what());
      fsm_.placed(false, kReasonPathError, e.what(), steady_ns());
      return;
    }
    // Resume: the restored points were resolved by the earlier execution (placement keeps the
    // point order and flags, so a must-hit rank names the same vertex in source and execution).
    if (!resume_points_.empty() && !journal_->restore_resolved(resume_points_.size())) {
      const std::string why = "persisted progress (" + std::to_string(resume_points_.size()) +
                              " points) does not fit the execution artifact (" +
                              std::to_string(journal_->point_count()) + " must-hit points)";
      RCLCPP_ERROR(get_logger(), "%s", why.c_str());
      journal_.reset();
      fsm_.placed(false, kReasonInternalError, why, steady_ns());
      return;
    }
    placement_->anchored = p.transformed;
    run_.path_artifact_sha256 = p.execution.sha256;
    const auto active = journal_->active_point();
    run_.point_index = active ? *active : 0U;
    RCLCPP_INFO(get_logger(), "placed %s -> execution %s (%s)", run_.source_artifact_sha256.c_str(),
                p.execution.sha256.c_str(), p.detail.c_str());
    fsm_.placed(true, kReasonNone, p.detail, steady_ns());
  }
}

void MissionNode::send_px4(const Px4Request& r) {
  const std::uint64_t id = r.id;
  PendingPx4 pending;
  if (r.op == Px4Op::kArm || r.op == Px4Op::kDisarm) {
    auto req = std::make_shared<ArmSrv::Request>();
    req->arm = r.op == Px4Op::kArm;
    pending.arm_client = true;
    pending.client_request_id =
        arm_cli_
            ->async_send_request(req,
                                 [this, id](rclcpp::Client<ArmSrv>::SharedFuture f) {
                                   const auto res = f.get();
                                   on_px4_reply(id, res->accepted, res->reason_code);
                                 })
            .request_id;
  } else {
    auto req = std::make_shared<OffboardSrv::Request>();
    req->enable = r.op == Px4Op::kOffboardOn;
    pending.arm_client = false;
    pending.client_request_id =
        offboard_cli_
            ->async_send_request(req,
                                 [this, id](rclcpp::Client<OffboardSrv>::SharedFuture f) {
                                   const auto res = f.get();
                                   on_px4_reply(id, res->accepted, res->reason_code);
                                 })
            .request_id;
  }
  px4_pending_[id] = pending;
  dirty_ = true;
  RCLCPP_INFO(get_logger(), "mission %u: px4_link %s requested", fsm_.mission_id(),
              to_string(r.op));
}

void MissionNode::on_px4_reply(std::uint64_t id, bool accepted, std::uint8_t reason) {
  px4_pending_.erase(id);
  if (const auto o = px4_.on_response(id, accepted, reason, steady_ns())) handle_px4(*o);
  advance();
}

void MissionNode::handle_px4(const Px4Outcome& o) {
  dirty_ = true;
  const std::string what = std::string(to_string(o.op)) + " " + to_string(o.result) +
                           (o.result == Px4Result::kRefused
                                ? " by px4_link (reason " + std::to_string(o.reason_code) + ")"
                                : "");
  if (o.result == Px4Result::kOk) {
    RCLCPP_INFO(get_logger(), "mission %u: %s", fsm_.mission_id(), what.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "mission %u: %s", fsm_.mission_id(), what.c_str());
  }
  if (o.abandoned) return;  // overtaken by a release: it no longer drives the lifecycle
  const bool ok = o.result == Px4Result::kOk;
  const bool timeout = o.result == Px4Result::kTimeout ||
                       (o.op == Px4Op::kArm && o.reason_code == kArmReasonTimeout);
  switch (o.op) {
    case Px4Op::kArm:
      if (fsm_.state() == State::kArming) {
        fsm_.armed(ok, timeout ? kReasonArmTimeout : kReasonArmRefused, what, steady_ns());
      }
      break;
    case Px4Op::kOffboardOn:
      if (fsm_.state() == State::kEngaging) {
        fsm_.engaged(ok, timeout ? kReasonOffboardTimeout : kReasonOffboardRefused, what,
                     steady_ns());
        if (fsm_.awaiting_reengage_gate()) {
          // Re-engage: RUNNING once the guard also sees armed + OFFBOARD (evaluate_gate), bounded
          // by offboard_timeout_s (on_timer).
          reengage_confirmed_ns_ = steady_ns();
          if (gate_ok()) fsm_.reengage_gate(true, steady_ns());
        }
      }
      break;
    case Px4Op::kOffboardOff:
    case Px4Op::kDisarm:
      if (!ok) release_note_ += (release_note_.empty() ? "" : "; ") + what;
      break;
  }
}

std::uint8_t MissionNode::waiting_on() const {
  if (const auto op = px4_.release_pending()) {
    return *op == Px4Op::kOffboardOff ? MissionState::WAIT_OFFBOARD_RELEASE
                                      : MissionState::WAIT_DISARM;
  }
  switch (fsm_.state()) {
    case State::kLoading:
      return MissionState::WAIT_ARTIFACT;
    case State::kPlacing:
      return MissionState::WAIT_PLACEMENT;
    case State::kArming:
      return MissionState::WAIT_ARM;
    case State::kEngaging:
      return MissionState::WAIT_OFFBOARD;
    case State::kReady:
      return MissionState::WAIT_RPP_ACK;
    case State::kPaused:
      return MissionState::WAIT_OPERATOR;
    default:
      return MissionState::WAIT_NONE;
  }
}

std::uint8_t MissionNode::published_state() const {
  if (fsm_.reengaging()) return MissionState::STATE_PAUSED;
  return static_cast<std::uint8_t>(fsm_.state());
}

// ------------------------------------------------------------------------------------------------
// persisted progress (contract section 9a)
// ------------------------------------------------------------------------------------------------
void MissionNode::record_resolved(const std::vector<PointEvent>& evs) {
  if (evs.empty()) return;
  // The journal resolves points in path order, so this list stays 0..k-1.
  for (const auto& e : evs) completed_.push_back(e.point_index);
  note_progress();
}

void MissionNode::note_progress() {
  if (!progress_tracking_) return;
  MissionProgress p;
  p.path_artifact_sha256 = run_.source_artifact_sha256;
  p.mission_id = fsm_.mission_id();
  p.run_index = progress_run_;
  p.point_index = run_.point_index;
  p.completed_points = completed_;
  p.state = to_string(fsm_.state());
  p.updated_utc = utc_now_iso8601();
  if (fsm_.terminal()) progress_tracking_ = false;  // the last record of this execution
  progress_[p.path_artifact_sha256] = p;  // what a resume decides from, before the disk has it
  progress_queue_[p.path_artifact_sha256] = std::move(p);  // at most one pending, newest wins
  launch_progress_job();
}

void MissionNode::launch_progress_job() {
  if (progress_job_.valid() || progress_queue_.empty()) return;
  std::vector<MissionProgress> batch;
  for (auto& kv : progress_queue_) batch.push_back(std::move(kv.second));
  progress_queue_.clear();
  progress_job_ = std::async(std::launch::async, [dir = missions_dir_, batch = std::move(batch)]() {
    std::vector<std::string> errors;
    for (const auto& rec : batch) {
      std::string e;
      if (!write_progress(dir, rec, &e)) errors.push_back(e);
    }
    return errors;
  });
  work_timer_->reset();
}

// ------------------------------------------------------------------------------------------------
// outputs
// ------------------------------------------------------------------------------------------------
void MissionNode::publish_state() {
  dirty_ = false;
  MissionState m;
  m.stamp = get_clock()->now();  // message stamp: ROS time (never used for an age)
  m.state = published_state();
  m.mission_id = fsm_.mission_id();
  m.run_index = run_.run_index;
  m.point_index = run_.point_index;
  m.reason_code = fsm_.reason();
  m.path_artifact_sha256 = run_.path_artifact_sha256;
  m.source_artifact_sha256 = run_.source_artifact_sha256;
  m.request_id = run_.request_id;
  m.reason_detail = fsm_.detail();
  if (!release_note_.empty()) m.reason_detail += "; release: " + release_note_;
  m.gate_reason_code = fsm_.gate_reason();
  m.waiting_on = waiting_on();
  m.state_entered = to_msg_time(state_entered_ros_ns_);  // ROS time, see MissionState.msg
  m.start_run_index = run_.start_run_index;
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
  m.stamp = get_clock()->now();  // message stamp: ROS time
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
  // Inside the cancel callback the goal is not CANCELING yet: on_timer finalises it then.
  if (cancel_pending_ && !goal_->is_canceling()) return;
  cancel_pending_ = false;
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

}  // namespace dyx3_mission
