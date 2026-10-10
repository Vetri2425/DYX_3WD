// gateway_node — see docs/contracts/dyx3_system_gateway.md
#include "dyx3_system_gateway/gateway_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "dyx3_system_gateway/json.hpp"

namespace dyx3_gateway {
namespace {

double steady_now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void require(bool ok, const std::string& what) {
  if (!ok) throw std::invalid_argument("gateway parameter invalid: " + what);
}
const rclcpp::QoS kRel1 = rclcpp::QoS(1).reliable();
// The sources of pushed events keep a history: with depth 1 a transition followed by another one
// before the executor runs would be overwritten in DDS and never become an event.
const rclcpp::QoS kEventSrc = rclcpp::QoS(10).reliable();

// Processing order inside one batch: E-stop strictly first, then the heartbeat, then the rest.
int batch_rank(CmdKind k) {
  if (k == CmdKind::Estop) return 0;
  if (k == CmdKind::Heartbeat) return 1;
  return 2;
}

// A telemetry callback with a concrete (non-generic) signature, as rclcpp's traits require.
template <class Msg, class F>
std::function<void(typename Msg::ConstSharedPtr)> upd(TelemetrySnapshot& snap, const ClockFn& clock,
                                                      const char* name, F build) {
  return [&snap, &clock, name, build](typename Msg::ConstSharedPtr m) {
    snap.update(name, build(*m), clock());
  };
}

// The same, and the message also feeds a pushed event: `key` names the transition, `event` builds
// the event's data object (contract section 1.3).
template <class Msg, class F, class K, class E>
std::function<void(typename Msg::ConstSharedPtr)> upd_event(TelemetrySnapshot& snap,
                                                            const ClockFn& clock,
                                                            EventStream* events, const char* name,
                                                            F build, const char* kind, K key,
                                                            E event) {
  return [&snap, &clock, events, name, build, kind, key, event](typename Msg::ConstSharedPtr m) {
    const double now = clock();
    snap.update(name, build(*m), now);
    events->offer(kind, key(*m), event(*m), now);
  };
}

// Reply data for a reply without a downstream answer: carries the request_id when there is one.
std::string echo_data(const std::string& request_id) {
  return request_id.empty() ? std::string("{}") : JsonLine().str("request_id", request_id).dump();
}

double stamp_s(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

// MissionState as the tablet sees it (snapshot `mission` and the `mission_state` event data): every
// field of interfaces 0.17.0. `mission_id` is the execution id; `path_artifact_sha256` the
// execution artifact RPP loads, `source_artifact_sha256` the artifact the operator started.
JsonLine mission_fields(const dyx3_interfaces::msg::MissionState& m) {
  JsonLine j;
  j.integer("state", m.state)
      .integer("mission_id", m.mission_id)
      .integer("run_index", m.run_index)
      .integer("point_index", m.point_index)
      .integer("reason_code", m.reason_code)
      .str("path_artifact_sha256", m.path_artifact_sha256)
      .str("source_artifact_sha256", m.source_artifact_sha256)
      .str("request_id", m.request_id)
      .str("reason_detail", m.reason_detail)
      .integer("gate_reason_code", m.gate_reason_code)
      .integer("waiting_on", m.waiting_on)
      .raw("state_entered", json_dbl(stamp_s(m.state_entered)))
      .integer("start_run_index", m.start_run_index);
  return j;
}

// The transition key of the `mission_state` event: a change of state, reason or of the step being
// waited on is a transition, even when the other two stay.
std::string mission_key(const dyx3_interfaces::msg::MissionState& m) {
  return std::to_string(m.state) + "/" + std::to_string(m.reason_code) + "/" +
         std::to_string(m.waiting_on);
}

// The gateway code of a refused downstream answer. A refusal is `rejected` with the mission's
// `reason_code` verbatim in `data`, except a request id the mission node refused: that is a
// malformed command, the same typed error the gateway itself gives for a bad id.
template <class Res>
const char* refusal_code(const Res& /*res*/) {
  return "rejected";
}
const char* refusal_code(const dyx3_interfaces::srv::StartMission::Response& res) {
  return res.reason_code == dyx3_interfaces::srv::StartMission::Response::REASON_INVALID_REQUEST
             ? "invalid_command"
             : "rejected";
}

}  // namespace

// Wakes the executor as soon as the IPC thread queues a command: the command is dispatched within
// the executor's reaction time instead of on the next 10 ms tick. Triggering is thread safe.
class WakeWaitable : public rclcpp::Waitable {
public:
  WakeWaitable(rclcpp::Context::SharedPtr ctx, std::function<void()> on_wake)
      : gc_(std::move(ctx)), on_wake_(std::move(on_wake)) {}
  void trigger() { gc_.trigger(); }
  size_t get_number_of_ready_guard_conditions() override { return 1; }
  void add_to_wait_set(rcl_wait_set_t* ws) override {
    if (rcl_wait_set_add_guard_condition(ws, &gc_.get_rcl_guard_condition(), nullptr) !=
        RCL_RET_OK) {
      throw std::runtime_error("gateway: cannot add the wake guard condition to the wait set");
    }
  }
  bool is_ready(rcl_wait_set_t* ws) override {
    for (size_t i = 0; i < ws->size_of_guard_conditions; ++i) {
      if (ws->guard_conditions[i] == &gc_.get_rcl_guard_condition()) return true;
    }
    return false;
  }
  std::shared_ptr<void> take_data() override { return nullptr; }
  void execute(std::shared_ptr<void>& /*data*/) override { on_wake_(); }

private:
  rclcpp::GuardCondition gc_;
  std::function<void()> on_wake_;
};

GatewayNode::GatewayNode(const rclcpp::NodeOptions& options, ClockFn clock, bool create_timer)
    : rclcpp::Node("system_gateway", options),
      clock_(clock ? std::move(clock) : ClockFn(steady_now_s)) {
  declare_params();
  link_ = OperatorLink(operator_link_timeout_s_);
  snap_ = TelemetrySnapshot(snapshot_fresh_s_);
  events_ = std::make_unique<EventStream>(event_coalesce_s_,
                                          [this](const std::string& l) { ipc_.broadcast(l); });
  EventStream* const ev = events_.get();
  pub_link_ =
      create_publisher<dyx3_interfaces::msg::OperatorLinkStatus>("/dyx3/operator_link", kRel1);

  cli_start_ = create_client<dyx3_interfaces::srv::StartMission>("/dyx3/mission/start");
  cli_abort_ = create_client<dyx3_interfaces::srv::AbortMission>("/dyx3/mission/abort");
  cli_pause_ = create_client<dyx3_interfaces::srv::PauseMission>("/dyx3/mission/pause");
  cli_resume_ = create_client<dyx3_interfaces::srv::ResumeMission>("/dyx3/mission/resume");
  cli_skip_ = create_client<dyx3_interfaces::srv::SkipPoint>("/dyx3/mission/skip_point");
  cli_estop_ = create_client<dyx3_interfaces::srv::SetEmergencyStop>(
      "/dyx3/motion_guard/set_emergency_stop");
  cli_arm_ = create_client<dyx3_interfaces::srv::ArmDisarm>("/dyx3/px4_link/arm");
  cli_offboard_ = create_client<dyx3_interfaces::srv::SetOffboard>("/dyx3/px4_link/set_offboard");
  cli_spray_ = create_client<dyx3_interfaces::srv::SetSprayManual>("/dyx3/spray/set_manual");

  using namespace dyx3_interfaces::msg;
  subs_.push_back(create_subscription<VehicleState>(
      "/dyx3/vehicle_state", kRel1,
      upd<VehicleState>(snap_, clock_, "vehicle_state", [](const VehicleState& m) {
        return JsonLine()
            .boolean("position_valid", m.position_valid)
            .boolean("velocity_valid", m.velocity_valid)
            .boolean("attitude_valid", m.attitude_valid)
            .num("north_m", m.north_m)
            .num("east_m", m.east_m)
            .num("down_m", m.down_m)
            .num("velocity_north_mps", m.velocity_north_mps)
            .num("velocity_east_mps", m.velocity_east_mps)
            .num("velocity_down_mps", m.velocity_down_mps)
            // Derived for the tablet: ground speed, and its component along the heading (+
            // forward).
            .num("ground_speed_mps", std::hypot(m.velocity_north_mps, m.velocity_east_mps))
            .num("forward_speed_mps", m.velocity_north_mps * std::cos(m.heading_rad) +
                                          m.velocity_east_mps * std::sin(m.heading_rad))
            .num("heading_rad", m.heading_rad)
            .num("yaw_rate_radps", m.yaw_rate_radps)
            .integer("arming_state", m.arming_state)
            .integer("nav_state", m.nav_state)
            .boolean("failsafe", m.failsafe)
            .boolean("global_reference_valid", m.global_reference_valid)
            // The EKF origin, so a client can turn north_m/east_m into lat/lon with PX4's own
            // projection (azimuthal equidistant, R 6371000 m) instead of assuming a plan origin.
            .raw("reference_latitude_deg", json_dbl(m.reference_latitude_deg))
            .raw("reference_longitude_deg", json_dbl(m.reference_longitude_deg))
            .integer("xy_reset_counter", m.xy_reset_counter)
            .boolean("battery_valid", m.battery_valid)
            .num("battery_voltage_v", m.battery_voltage_v)
            .num("battery_current_a", m.battery_current_a)
            .num("battery_remaining", m.battery_remaining)
            // 0.17.0: the RC link as PX4 sees it; rc_link_ok is meaningful only while valid.
            .boolean("rc_link_valid", m.rc_link_valid)
            .boolean("rc_link_ok", m.rc_link_ok)
            .dump();
      })));
  subs_.push_back(create_subscription<EstimatorHealth>(
      "/dyx3/estimator_health", kRel1,
      upd<EstimatorHealth>(snap_, clock_, "estimator_health", [](const EstimatorHealth& m) {
        return JsonLine()
            .boolean("flags_valid", m.flags_valid)
            .boolean("gnss_yaw_fusion_intended", m.gnss_yaw_fusion_intended)
            .boolean("gnss_yaw_fault", m.gnss_yaw_fault)
            .boolean("reject_yaw", m.reject_yaw)
            .boolean("reject_hor_pos", m.reject_hor_pos)
            .boolean("reject_hor_vel", m.reject_hor_vel)
            .boolean("inertial_dead_reckoning", m.inertial_dead_reckoning)
            .dump();
      })));
  subs_.push_back(create_subscription<RtkStatus>(
      "/dyx3/rtk_status", kRel1,
      upd<RtkStatus>(snap_, clock_, "rtk_status", [](const RtkStatus& m) {
        return JsonLine()
            .integer("fix_type", m.fix_type)
            .boolean("corrections_fresh", m.corrections_fresh)
            .num("correction_age_s", m.correction_age_s)
            .num("horizontal_accuracy_m", m.horizontal_accuracy_m)
            .integer("satellites_used", m.satellites_used)
            .dump();
      })));
  subs_.push_back(create_subscription<GnssReport>(
      "/dyx3/gnss_report", rclcpp::QoS(5),
      upd<GnssReport>(snap_, clock_, "gnss_report", [](const GnssReport& m) {
        return JsonLine()
            .boolean("valid", m.valid)
            .integer("fix_type", m.fix_type)
            .num("horizontal_accuracy_m", m.horizontal_accuracy_m)
            .num("vertical_accuracy_m", m.vertical_accuracy_m)
            .integer("satellites_used", m.satellites_used)
            .num("heading_rad", m.heading_rad)
            .num("heading_accuracy_rad", m.heading_accuracy_rad)
            .raw("latitude_deg", json_dbl(m.latitude_deg))
            .raw("longitude_deg", json_dbl(m.longitude_deg))
            .num("altitude_msl_m", m.altitude_msl_m)
            .num("hdop", m.hdop)
            .dump();
      })));
  subs_.push_back(create_subscription<NtripStatus>(
      "/dyx3/ntrip_status", kRel1,
      upd<NtripStatus>(snap_, clock_, "ntrip_status", [](const NtripStatus& m) {
        return JsonLine()
            .integer("state", m.state)
            .boolean("connected", m.connected)
            .boolean("streaming", m.streaming)
            .num("correction_age_s", m.correction_age_s)
            .num("correction_rate_hz", m.correction_rate_hz)
            .integer("reconnect_count", m.reconnect_count)
            .integer("source_bytes_received", m.source_bytes_received)
            .integer("valid_rtcm_frames", m.valid_rtcm_frames)
            .integer("chunks_handed_off", m.chunks_handed_off)
            .integer("security", m.security)
            .boolean("tls_verified", m.tls_verified)
            .boolean("tls_verification_failed", m.tls_verification_failed)
            .boolean("plaintext_credentials_warning", m.plaintext_credentials_warning)
            .str("last_error", m.last_error)
            .integer("fix_transitions", m.fix_transitions)
            .dump();
      })));
  subs_.push_back(create_subscription<Px4LinkStatus>(
      "/dyx3/px4_link/status", kEventSrc,
      upd_event<Px4LinkStatus>(
          snap_, clock_, ev, "px4_link",
          [](const Px4LinkStatus& m) {
            return JsonLine()
                .boolean("session_alive", m.session_alive)
                .boolean("handshake_ok", m.handshake_ok)
                .boolean("offboard_heartbeat_active", m.offboard_heartbeat_active)
                .boolean("failing_to_zero", m.failing_to_zero)
                .integer("fault", m.fault)
                .integer("stale_topics_mask", m.stale_topics_mask)
                .num("worst_topic_age_s", m.worst_topic_age_s)
                .integer("session_resets", m.session_resets)
                .boolean("timesync_valid", m.timesync_valid)
                .integer("timesync_offset_us", m.timesync_offset_us)
                .integer("timesync_round_trip_us", m.timesync_round_trip_us)
                .integer("rtcm_chunks_accepted", m.rtcm_chunks_accepted)
                .integer("rtcm_chunks_dropped", m.rtcm_chunks_dropped)
                .dump();
          },
          "fcu_link",
          [](const Px4LinkStatus& m) {
            return std::string(m.session_alive ? "1" : "0") + (m.handshake_ok ? "1" : "0");
          },
          [](const Px4LinkStatus& m) {
            return JsonLine()
                .boolean("fresh", true)
                .boolean("session_alive", m.session_alive)
                .boolean("handshake_ok", m.handshake_ok)
                .integer("fault", m.fault)
                .integer("session_resets", m.session_resets)
                .dump();
          })));
  subs_.push_back(create_subscription<SafetyGateStatus>(
      "/dyx3/safety_gate", kRel1,
      upd<SafetyGateStatus>(snap_, clock_, "safety_gate", [](const SafetyGateStatus& m) {
        return JsonLine()
            .boolean("ok", m.ok)
            .integer("reason_code", m.reason_code)
            .boolean("pre_arm_ok", m.pre_arm_ok)
            .integer("pre_arm_reason_code", m.pre_arm_reason_code)
            .dump();
      })));
  subs_.push_back(create_subscription<EmergencyStopState>(
      "/dyx3/emergency_stop_state", kEventSrc,
      upd_event<EmergencyStopState>(
          snap_, clock_, ev, "emergency_stop",
          [](const EmergencyStopState& m) {
            return JsonLine().boolean("asserted", m.asserted).str("source", m.source).dump();
          },
          "estop",
          [](const EmergencyStopState& m) {
            return std::string(m.asserted ? "1:" : "0:") + std::string(m.source);
          },
          [](const EmergencyStopState& m) {
            return JsonLine()
                .boolean("fresh", true)
                .boolean("asserted", m.asserted)
                .str("source", m.source)
                .dump();
          })));
  subs_.push_back(create_subscription<MotionSetpointStatus>(
      "/dyx3/motion_guard/status", rclcpp::QoS(10).reliable(),
      upd<MotionSetpointStatus>(snap_, clock_, "motion_guard", [](const MotionSetpointStatus& m) {
        return JsonLine()
            .integer("reason_code", m.reason_code)
            .boolean("accepted", m.accepted)
            .boolean("clamped", m.clamped)
            .integer("mode", m.mode)
            .num("speed_body_x", m.speed_body_x)
            .dump();
      })));
  subs_.push_back(create_subscription<RppStatus>(
      "/dyx3/rpp/status", kRel1, upd<RppStatus>(snap_, clock_, "rpp", [](const RppStatus& m) {
        return JsonLine()
            .integer("state", m.state)
            .integer("mission_id", m.mission_id)
            .integer("run_index", m.run_index)
            .num("cross_track_right_m", m.cross_track_right_m)
            .num("heading_error_rad", m.heading_error_rad)
            .num("dist_to_goal_m", m.dist_to_goal_m)
            .num("path_travel_m", m.path_travel_m)
            .num("commanded_speed_mps", m.commanded_speed_mps)
            .num("commanded_yaw_rate_radps", m.commanded_yaw_rate_radps)
            .integer("tick_state", m.tick_state)
            .integer("segment_state", m.segment_state)
            .integer("rtk_reason", m.rtk_reason)
            .boolean("spray_request", m.spray_request)
            .boolean("pivot_timed_out", m.pivot_timed_out)
            .num("loop_jitter_max_us", m.loop_jitter_max_us)
            .integer("loop_overrun_count", static_cast<int64_t>(m.loop_overrun_count))
            .dump();
      })));
  subs_.push_back(create_subscription<MissionState>(
      "/dyx3/mission/state", kEventSrc,
      upd_event<MissionState>(
          snap_, clock_, ev, "mission",
          [](const MissionState& m) { return mission_fields(m).dump(); }, "mission_state",
          [](const MissionState& m) { return mission_key(m); },
          [](const MissionState& m) {
            JsonLine j = mission_fields(m);
            j.boolean("fresh", true).raw("stamp_s", json_dbl(stamp_s(m.stamp)));
            return j.dump();
          })));
  subs_.push_back(create_subscription<PointResult>(
      "/dyx3/mission/point_result", rclcpp::QoS(10).reliable(),
      upd<PointResult>(snap_, clock_, "last_point_result", [](const PointResult& m) {
        return JsonLine()
            .integer("mission_id", m.mission_id)
            .integer("point_index", m.point_index)
            .integer("result_code", m.result_code)
            .num("north_m", m.north_m)
            .num("east_m", m.east_m)
            .num("error_m", m.error_m)
            .dump();
      })));
  subs_.push_back(create_subscription<SprayStatus>(
      "/dyx3/spray/status", rclcpp::QoS(10).reliable(),
      upd<SprayStatus>(snap_, clock_, "spray", [](const SprayStatus& m) {
        return JsonLine()
            .integer("fsm_state", m.fsm_state)
            .boolean("spraying", m.spraying)
            .boolean("desired", m.desired)
            .boolean("safety_ok", m.safety_ok)
            .str("safety_reason", m.safety_reason)
            .boolean("manual_active", m.manual_active)
            .boolean("xtrack_tripped", m.xtrack_tripped)
            .num("xtrack_error_m", m.xtrack_error_m)
            .dump();
      })));
  subs_.push_back(create_subscription<RecorderStatus>(
      "/dyx3/recorder/status", kRel1,
      upd<RecorderStatus>(snap_, clock_, "recorder", [](const RecorderStatus& m) {
        return JsonLine()
            .integer("state", m.state)
            .boolean("bag_healthy", m.bag_healthy)
            .integer("bytes_written", static_cast<int64_t>(m.bytes_written))
            .integer("free_bytes", static_cast<int64_t>(m.free_bytes))
            .dump();
      })));

  if (create_timer) {
    // Before the IPC thread starts: on_line() reads wake_ from that thread.
    wake_ = std::make_shared<WakeWaitable>(get_node_base_interface()->get_context(),
                                           [this]() { step(clock_()); });
    get_node_waitables_interface()->add_waitable(wake_, nullptr);
  }
  IpcServer::Config c;
  c.path = socket_path_;
  c.max_clients = max_clients_;
  std::string err;
  if (!ipc_.start(
          c, [this](int cl, const std::string& l) { on_line(cl, l); }, &err,
          [this](int cl) { on_connect(cl); })) {
    throw std::runtime_error("gateway cannot open its socket: " + err);
  }
  if (create_timer) {
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { step(clock_()); });
  }
  RCLCPP_INFO(get_logger(), "gateway up: %s, operator-link timeout %.2f s", socket_path_.c_str(),
              operator_link_timeout_s_);
}

GatewayNode::~GatewayNode() {
  ipc_.stop();  // first: no IPC callback (on_line, on_connect) runs past this point
  if (wake_) get_node_waitables_interface()->remove_waitable(wake_, nullptr);
}

void GatewayNode::declare_params() {
  socket_path_ = declare_parameter<std::string>("socket_path", "/run/dyx3/gateway.sock");
  max_clients_ = static_cast<int>(declare_parameter<int>("max_clients", 4));
  telemetry_hz_ = declare_parameter<double>("telemetry_hz", 10.0);
  operator_link_timeout_s_ =
      declare_parameter<double>("operator_link_timeout_s", 2.0);  // DERIVED / OPEN, see contract
  service_timeout_s_ = declare_parameter<double>("service_timeout_s", 2.0);
  estop_timeout_s_ = declare_parameter<double>("estop_timeout_s", 1.0);
  arm_timeout_s_ = declare_parameter<double>("arm_timeout_s", 4.0);
  offboard_timeout_s_ = declare_parameter<double>("offboard_timeout_s", 5.0);
  snapshot_fresh_s_ = declare_parameter<double>("snapshot_fresh_s", 1.0);
  operator_link_hz_ = declare_parameter<double>("operator_link_hz", 10.0);
  event_coalesce_s_ = declare_parameter<double>("event_coalesce_s", 0.01);
  for (const double v :
       {telemetry_hz_, operator_link_timeout_s_, service_timeout_s_, estop_timeout_s_,
        arm_timeout_s_, offboard_timeout_s_, snapshot_fresh_s_, operator_link_hz_}) {
    require(std::isfinite(v) && v > 0.0, "rates and timeouts must be finite and > 0");
  }
  // A backend waits for every answer: no command may hold it longer than kMaxCommandTimeoutS.
  constexpr double kMaxCommandTimeoutS = 30.0;
  for (const double v : {service_timeout_s_, estop_timeout_s_, arm_timeout_s_, offboard_timeout_s_})
    require(v <= kMaxCommandTimeoutS, "command timeouts must be <= 30 s");
  // The E-stop verdict is the first to come back: a failed E-stop must reach the operator first.
  require(estop_timeout_s_ <= std::min({service_timeout_s_, arm_timeout_s_, offboard_timeout_s_}),
          "estop_timeout_s must not exceed any other command timeout");
  require(std::isfinite(event_coalesce_s_) && event_coalesce_s_ >= 0.0 && event_coalesce_s_ <= 1.0,
          "event_coalesce_s in [0, 1]");
  require(!socket_path_.empty(), "socket_path must not be empty");
  require(max_clients_ >= 1 && max_clients_ <= 64, "max_clients in [1, 64]");
}

void GatewayNode::on_line(int client, const std::string& line) {
  ParseResult pr = parse_command(line);
  if (!pr.ok) {
    JsonLine r;
    r.integer("v", kProtocolVersion);
    if (pr.has_id) r.integer("id", pr.id);
    r.boolean("ok", false).str("code", pr.code).str("reason", pr.reason).raw("data", "{}");
    ipc_.send(client, r.dump());
    return;
  }
  std::lock_guard<std::mutex> lk(inbox_mu_);
  // Only an E-stop may exceed the cap (it is never refused); heartbeats count against it like
  // every other command, so a heartbeat flood cannot grow the inbox without bound (GW-005).
  if (inbox_.size() >= kInboxCap && pr.cmd.kind != CmdKind::Estop) {
    ++inbox_refused_;
    JsonLine r;
    r.integer("v", kProtocolVersion);
    if (pr.has_id) r.integer("id", pr.id);
    r.boolean("ok", false)
        .str("code", "busy")
        .str("reason", "gateway inbox full")
        .raw("data", "{}");
    ipc_.send(client, r.dump());
    return;
  }
  inbox_.push_back(Inbound{client, std::move(pr), steady_now_s()});
  if (wake_) wake_->trigger();
}

void GatewayNode::on_connect(int client) {
  // The newcomer gets the latest event of each kind at once (marked replay), so it never polls for
  // the current state after a (re)connect.
  events_->replay([this, client](const std::string& l) { ipc_.send(client, l); });
}

double GatewayNode::timeout_for(CmdKind k) const {
  switch (k) {
    case CmdKind::Estop:
      return estop_timeout_s_;
    case CmdKind::Arm:
      return arm_timeout_s_;
    case CmdKind::Offboard:
      return offboard_timeout_s_;
    default:
      return service_timeout_s_;
  }
}

void GatewayNode::note_dispatch(const Inbound& in) {
  const double us = (steady_now_s() - in.rx_steady_s) * 1e6;
  ++dispatch_.count;
  dispatch_.last_us = us;
  dispatch_.max_us = std::max(dispatch_.max_us, us);
  dispatch_.sum_us += us;
  if (in.pr.cmd.kind == CmdKind::Estop) dispatch_.estop_last_us = us;
}

void GatewayNode::reply(int client, bool has_id, int64_t id, bool ok, const std::string& code,
                        const std::string& reason, const std::string& data_json) {
  JsonLine r;
  r.integer("v", kProtocolVersion);
  if (has_id) r.integer("id", id);
  r.boolean("ok", ok).str("code", code).str("reason", reason).raw("data", data_json);
  ipc_.send(client, r.dump());
}

OperatorLinkState GatewayNode::link_state(double now_s) const {
  return link_.state(now_s, ipc_.connected(link_.client()));
}

std::string GatewayNode::gateway_json(double now_s) const {
  const OperatorLinkState s = link_state(now_s);
  return JsonLine()
      .integer("schema", kProtocolVersion)
      .boolean("operator_alive", s.alive)
      .num("operator_age_s", s.age_s)
      .integer("clients", ipc_.clients())
      .raw("ipc", JsonLine()
                      .integer("dropped_slow", static_cast<int64_t>(ipc_.dropped_slow()))
                      .integer("overflows", static_cast<int64_t>(ipc_.overflows()))
                      .integer("rejected_full", static_cast<int64_t>(ipc_.rejected_full()))
                      .integer("event_seq", static_cast<int64_t>(events_->seq()))
                      .num("dispatch_last_us", dispatch_.last_us)
                      .num("dispatch_max_us", dispatch_.max_us)
                      .dump())
      .dump();
}

template <typename Srv, typename Fill, typename Render>
void GatewayNode::call(const Inbound& in, double now_s, typename rclcpp::Client<Srv>::SharedPtr cli,
                       const char* name, Fill fill, Render render) {
  const Command& c = in.pr.cmd;
  const std::string what =
      c.kind == CmdKind::Estop
          ? std::string("E-stop ") + (c.flag ? "assert" : "clear") + " (source=" + c.source + ")"
          : std::string(to_string(c.kind));
  if (!cli->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "%s from client %d: %s is not available", what.c_str(), in.client,
                name);
    reply(in.client, in.pr.has_id, in.pr.id, false, "service_unavailable",
          std::string(name) + " is not available", echo_data(c.request_id));
    return;
  }
  // Bounded: an E-stop is never refused, everything else waits for room.
  if (pending_.size() >= kPendingCap && c.kind != CmdKind::Estop) {
    ++inbox_refused_;
    reply(in.client, in.pr.has_id, in.pr.id, false, "busy", "too many commands awaiting an answer",
          echo_data(c.request_id));
    return;
  }
  auto req = std::make_shared<typename Srv::Request>();
  fill(*req);
  const uint64_t token = next_token_++;
  const double timeout_s = timeout_for(c.kind);
  pending_[token] = Pending{in.client, in.pr.has_id, in.pr.id, c.kind, now_s + timeout_s,
                            timeout_s, c.request_id, what,     {}};
  const auto sent = cli->async_send_request(
      req, [this, token, render](typename rclcpp::Client<Srv>::SharedFuture f) {
        const auto it = pending_.find(token);
        if (it == pending_.end()) return;  // already answered with "timeout"
        const Pending p = it->second;
        pending_.erase(it);
        const auto res = f.get();
        bool accepted = false;
        JsonLine data = render(*res, &accepted);
        if (!p.request_id.empty()) data.str("request_id", p.request_id);
        const std::string data_json = data.dump();
        if (p.kind == CmdKind::Estop) {
          RCLCPP_WARN(get_logger(), "%s from client %d: %s by motion_guard %s", p.what.c_str(),
                      p.client, accepted ? "ACCEPTED" : "REJECTED", data_json.c_str());
        }
        const char* const refused = refusal_code(*res);
        reply(p.client, p.has_id, p.id, accepted, accepted ? "ok" : refused,
              accepted ? "" : "refused by the target (see data.reason_code)", data_json);
      });
  note_dispatch(in);
  const int64_t request_id = sent.request_id;
  pending_[token].forget = [cli, request_id]() { cli->remove_pending_request(request_id); };
}

size_t GatewayNode::prune_rclcpp_pending_requests() {
  return cli_start_->prune_pending_requests() + cli_abort_->prune_pending_requests() +
         cli_pause_->prune_pending_requests() + cli_resume_->prune_pending_requests() +
         cli_skip_->prune_pending_requests() + cli_estop_->prune_pending_requests() +
         cli_arm_->prune_pending_requests() + cli_offboard_->prune_pending_requests() +
         cli_spray_->prune_pending_requests();
}

size_t GatewayNode::inbox_depth() const {
  std::lock_guard<std::mutex> lk(inbox_mu_);
  return inbox_.size();
}

std::vector<std::string> GatewayNode::unavailable_services() const {
  const rclcpp::ClientBase* clients[] = {cli_start_.get(),  cli_abort_.get(),    cli_pause_.get(),
                                         cli_resume_.get(), cli_skip_.get(),     cli_estop_.get(),
                                         cli_arm_.get(),    cli_offboard_.get(), cli_spray_.get()};
  std::vector<std::string> out;
  for (const auto* c : clients) {
    if (!c->service_is_ready()) out.emplace_back(c->get_service_name());
  }
  return out;
}

void GatewayNode::process(const Inbound& in, double now_s) {
  const Command& c = in.pr.cmd;
  const auto base = [](auto& r, bool* acc) {
    *acc = r.accepted;
    JsonLine j;
    j.boolean("accepted", r.accepted).integer("reason_code", r.reason_code);
    return j;
  };
  switch (c.kind) {
    case CmdKind::Heartbeat:
      link_.note_heartbeat(now_s, in.client);
      reply(in.client, in.pr.has_id, in.pr.id, true, "ok", "");
      return;
    case CmdKind::GetSnapshot:
      reply(in.client, in.pr.has_id, in.pr.id, true, "ok", "",
            snap_.to_json(now_s, gateway_json(now_s)));
      return;
    case CmdKind::StartMission:
      call<dyx3_interfaces::srv::StartMission>(
          in, now_s, cli_start_, "mission start service",
          [&](auto& rq) {
            rq.path_artifact_sha256 = c.sha256;
            rq.request_id = c.request_id;
            rq.resume = c.resume;
          },
          [base](auto& r, bool* a) {
            JsonLine j = base(r, a);
            j.integer("mission_id", r.mission_id)
                .boolean("duplicate", r.duplicate)
                .integer("gate_reason_code", r.gate_reason_code)
                .integer("resumed_run_index", r.resumed_run_index);
            return j;
          });
      return;
    case CmdKind::AbortMission:
      call<dyx3_interfaces::srv::AbortMission>(
          in, now_s, cli_abort_, "mission abort service",
          [&](auto& rq) { rq.reason_code = c.abort_reason; },
          [base](auto& r, bool* a) { return base(r, a); });
      return;
    case CmdKind::PauseMission:
      call<dyx3_interfaces::srv::PauseMission>(
          in, now_s, cli_pause_, "mission pause service", [](auto&) {},
          [base](auto& r, bool* a) { return base(r, a); });
      return;
    case CmdKind::ResumeMission:
      call<dyx3_interfaces::srv::ResumeMission>(
          in, now_s, cli_resume_, "mission resume service", [](auto&) {},
          [base](auto& r, bool* a) { return base(r, a); });
      return;
    case CmdKind::SkipPoint:
      call<dyx3_interfaces::srv::SkipPoint>(
          in, now_s, cli_skip_, "mission skip service", [](auto&) {},
          [base](auto& r, bool* a) {
            JsonLine j = base(r, a);
            j.integer("skipped_point_index", r.skipped_point_index);
            return j;
          });
      return;
    case CmdKind::Estop:
      RCLCPP_WARN(get_logger(), "E-stop %s requested (source=%s, client %d)",
                  c.flag ? "assert" : "clear", c.source.c_str(), in.client);
      call<dyx3_interfaces::srv::SetEmergencyStop>(
          in, now_s, cli_estop_, "motion_guard emergency-stop service",
          [&](auto& rq) {
            rq.asserted = c.flag;
            rq.source = c.source;
          },
          [base](auto& r, bool* a) { return base(r, a); });
      return;
    case CmdKind::Arm:
      call<dyx3_interfaces::srv::ArmDisarm>(
          in, now_s, cli_arm_, "px4_link arm service", [&](auto& rq) { rq.arm = c.flag; },
          [base](auto& r, bool* a) { return base(r, a); });
      return;
    case CmdKind::Offboard:
      call<dyx3_interfaces::srv::SetOffboard>(
          in, now_s, cli_offboard_, "px4_link offboard service",
          [&](auto& rq) { rq.enable = c.flag; }, [base](auto& r, bool* a) { return base(r, a); });
      return;
    case CmdKind::SprayManual:
      call<dyx3_interfaces::srv::SetSprayManual>(
          in, now_s, cli_spray_, "spray manual service", [&](auto& rq) { rq.on = c.flag; },
          [base](auto& r, bool* a) { return base(r, a); });
      return;
  }
}

// Evaluated every step (not only at the 10 Hz publish), so the operator_link event and the audit
// line follow a heartbeat or a closed connection within one step.
void GatewayNode::track_operator_link(double now_s) {
  const OperatorLinkState s = link_state(now_s);
  if (s.alive == link_alive_ && !events_->last_key("operator_link").empty()) return;
  const bool changed = s.alive != link_alive_;
  link_alive_ = s.alive;
  std::string cause = "heartbeat";
  if (!s.alive) {
    if (link_.client() < 0) {
      cause = "never";
    } else {
      cause = ipc_.connected(link_.client()) ? "timeout" : "connection_closed";
    }
  }
  if (changed) {
    if (s.alive) {
      RCLCPP_WARN(get_logger(), "operator link ALIVE (heartbeat from client %d)", link_.client());
    } else {
      RCLCPP_WARN(get_logger(), "operator link LOST: %s (client %d, heartbeat age %.2f s)",
                  cause == "timeout" ? "heartbeat timeout" : "heartbeating connection closed",
                  link_.client(), s.age_s);
    }
  }
  events_->offer(
      "operator_link", s.alive ? "1" : "0",
      JsonLine().boolean("alive", s.alive).num("age_s", s.age_s).str("cause", cause).dump(), now_s);
}

// A status source that stops publishing is a transition too: its event says "fresh": false (the
// state is unknown, which a client must never read as the last value).
void GatewayNode::track_stale_sources(double now_s) {
  static const std::pair<const char*, const char*> kWatched[] = {
      {"mission_state", "mission"}, {"fcu_link", "px4_link"}, {"estop", "emergency_stop"}};
  for (const auto& [kind, source] : kWatched) {
    const std::string key = events_->last_key(kind);
    if (key.empty() || key == "stale" || snap_.fresh(source, now_s)) continue;
    events_->offer(kind, "stale", JsonLine().boolean("fresh", false).dump(), now_s);
  }
}

void GatewayNode::publish_operator_link(double now_s) {
  const OperatorLinkState s = link_state(now_s);
  dyx3_interfaces::msg::OperatorLinkStatus m;
  m.stamp = get_clock()->now();
  m.alive = s.alive;
  m.age_s = static_cast<float>(s.age_s);
  pub_link_->publish(m);
}

void GatewayNode::audit_ipc(double now_s) {
  // Checked at most once a second, so a reconnect loop cannot flood the log.
  if (now_s - last_audit_s_ < 1.0) return;
  last_audit_s_ = now_s;
  const int cl = ipc_.clients();
  if (cl != audit_clients_) {
    RCLCPP_INFO(get_logger(), "IPC clients: %d (was %d)", cl, audit_clients_);
    audit_clients_ = cl;
  }
  const auto delta = [this](uint64_t now_v, uint64_t* seen, const char* what) {
    if (now_v == *seen) return;
    RCLCPP_WARN(get_logger(), "IPC: %llu %s (total %llu)",
                static_cast<unsigned long long>(now_v - *seen), what,
                static_cast<unsigned long long>(now_v));
    *seen = now_v;
  };
  delta(ipc_.dropped_slow(), &audit_dropped_, "client(s) dropped as slow consumers");
  delta(ipc_.overflows(), &audit_overflows_, "client(s) closed for an oversize line");
  delta(ipc_.rejected_full(), &audit_rejected_, "connection(s) refused at max_clients");
  delta(inbox_refused_.load(), &audit_busy_,
        "command(s) refused as busy (inbox or pending table full)");
}

void GatewayNode::step(double now_s) {
  std::deque<Inbound> work;
  {
    std::lock_guard<std::mutex> lk(inbox_mu_);
    work.swap(inbox_);
  }
  // E-stops first, then heartbeats, then everything else, each in arrival order (GW-005).
  std::stable_sort(work.begin(), work.end(), [](const Inbound& a, const Inbound& b) {
    return batch_rank(a.pr.cmd.kind) < batch_rank(b.pr.cmd.kind);
  });
  // Heartbeats are coalesced: only the most recent one in the batch refreshes (and binds) the
  // operator link; the earlier ones are acknowledged without further effect.
  const Inbound* last_hb = nullptr;
  for (const auto& in : work)
    if (in.pr.cmd.kind == CmdKind::Heartbeat) last_hb = &in;
  if (!work.empty()) last_batch_.clear();
  for (const auto& in : work) {
    if (in.pr.cmd.kind == CmdKind::Heartbeat && &in != last_hb) {
      reply(in.client, in.pr.has_id, in.pr.id, true, "ok", "");
      continue;
    }
    last_batch_.push_back(in.pr.cmd.kind);
    process(in, now_s);
  }

  for (auto it = pending_.begin(); it != pending_.end();) {
    if (now_s >= it->second.deadline_s) {
      const Pending p = it->second;
      RCLCPP_WARN(get_logger(), "%s from client %d: no answer within %.1f s, reported as timeout",
                  p.what.c_str(), p.client, p.timeout_s);
      // A late answer is no longer wanted: drop it from the client as well, or every request that
      // is never answered stays in rclcpp's pending map for the life of the node.
      if (p.forget) p.forget();
      reply(p.client, p.has_id, p.id, false, "timeout",
            std::string(to_string(p.kind)) + " was not answered in time", echo_data(p.request_id));
      it = pending_.erase(it);
    } else {
      ++it;
    }
  }
  track_operator_link(now_s);
  track_stale_sources(now_s);
  events_->flush(now_s);
  audit_ipc(now_s);
  if (now_s - last_link_pub_s_ >= 1.0 / operator_link_hz_ - 1e-9) {
    last_link_pub_s_ = now_s;
    publish_operator_link(now_s);
  }
  if (now_s - last_tel_s_ >= 1.0 / telemetry_hz_ - 1e-9) {
    last_tel_s_ = now_s;
    if (ipc_.clients() > 0) {
      // seq counts the frames pushed (1, 2, ...) so a consumer can see a dropped or reordered
      // frame; t_mono_s is the same clock and format as an event's t_mono_s.
      const uint64_t seq = ++telemetry_seq_;
      ipc_.broadcast(JsonLine()
                         .integer("v", kProtocolVersion)
                         .str("type", "telemetry")
                         .integer("seq", static_cast<int64_t>(seq))
                         .raw("t_mono_s", json_dbl(now_s))
                         .raw("snapshot", snap_.to_json(now_s, gateway_json(now_s)))
                         .dump());
    }
  }
}

}  // namespace dyx3_gateway
