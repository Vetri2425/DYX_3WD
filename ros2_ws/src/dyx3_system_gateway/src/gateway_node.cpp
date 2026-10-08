// gateway_node — see docs/contracts/dyx3_system_gateway.md
#include "dyx3_system_gateway/gateway_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

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

// A telemetry callback with a concrete (non-generic) signature, as rclcpp's traits require.
template <class Msg, class F>
std::function<void(typename Msg::ConstSharedPtr)> upd(TelemetrySnapshot& snap, const ClockFn& clock,
                                                      const char* name, F build) {
  return [&snap, &clock, name, build](typename Msg::ConstSharedPtr m) {
    snap.update(name, build(*m), clock());
  };
}

}  // namespace

GatewayNode::GatewayNode(const rclcpp::NodeOptions& options, ClockFn clock, bool create_timer)
    : rclcpp::Node("system_gateway", options),
      clock_(clock ? std::move(clock) : ClockFn(steady_now_s)) {
  declare_params();
  link_ = OperatorLink(operator_link_timeout_s_);
  snap_ = TelemetrySnapshot(snapshot_fresh_s_);
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
            .num("heading_rad", m.heading_rad)
            .num("yaw_rate_radps", m.yaw_rate_radps)
            .integer("arming_state", m.arming_state)
            .integer("nav_state", m.nav_state)
            .boolean("failsafe", m.failsafe)
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
            .integer("satellites_used", m.satellites_used)
            .num("heading_rad", m.heading_rad)
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
      "/dyx3/px4_link/status", kRel1,
      upd<Px4LinkStatus>(snap_, clock_, "px4_link", [](const Px4LinkStatus& m) {
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
      })));
  subs_.push_back(create_subscription<SafetyGateStatus>(
      "/dyx3/safety_gate", kRel1,
      upd<SafetyGateStatus>(snap_, clock_, "safety_gate", [](const SafetyGateStatus& m) {
        return JsonLine().boolean("ok", m.ok).integer("reason_code", m.reason_code).dump();
      })));
  subs_.push_back(create_subscription<EmergencyStopState>(
      "/dyx3/emergency_stop_state", kRel1,
      upd<EmergencyStopState>(snap_, clock_, "emergency_stop", [](const EmergencyStopState& m) {
        return JsonLine().boolean("asserted", m.asserted).str("source", m.source).dump();
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
            .num("commanded_speed_mps", m.commanded_speed_mps)
            .num("loop_jitter_max_us", m.loop_jitter_max_us)
            .integer("loop_overrun_count", static_cast<int64_t>(m.loop_overrun_count))
            .dump();
      })));
  subs_.push_back(create_subscription<MissionState>(
      "/dyx3/mission/state", kRel1,
      upd<MissionState>(snap_, clock_, "mission", [](const MissionState& m) {
        return JsonLine()
            .integer("state", m.state)
            .integer("mission_id", m.mission_id)
            .integer("run_index", m.run_index)
            .integer("point_index", m.point_index)
            .integer("reason_code", m.reason_code)
            .str("path_artifact_sha256", m.path_artifact_sha256)
            .dump();
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

  IpcServer::Config c;
  c.path = socket_path_;
  c.max_clients = max_clients_;
  std::string err;
  if (!ipc_.start(c, [this](int cl, const std::string& l) { on_line(cl, l); }, &err)) {
    throw std::runtime_error("gateway cannot open its socket: " + err);
  }
  if (create_timer) {
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { step(clock_()); });
  }
  RCLCPP_INFO(get_logger(), "gateway up: %s, operator-link timeout %.2f s", socket_path_.c_str(),
              operator_link_timeout_s_);
}

GatewayNode::~GatewayNode() { ipc_.stop(); }

void GatewayNode::declare_params() {
  socket_path_ = declare_parameter<std::string>("socket_path", "/run/dyx3/gateway.sock");
  max_clients_ = static_cast<int>(declare_parameter<int>("max_clients", 4));
  telemetry_hz_ = declare_parameter<double>("telemetry_hz", 5.0);
  operator_link_timeout_s_ =
      declare_parameter<double>("operator_link_timeout_s", 2.0);  // DERIVED / OPEN, see contract
  service_timeout_s_ = declare_parameter<double>("service_timeout_s", 2.0);
  snapshot_fresh_s_ = declare_parameter<double>("snapshot_fresh_s", 1.0);
  operator_link_hz_ = declare_parameter<double>("operator_link_hz", 10.0);
  for (const double v : {telemetry_hz_, operator_link_timeout_s_, service_timeout_s_,
                         snapshot_fresh_s_, operator_link_hz_}) {
    require(std::isfinite(v) && v > 0.0, "rates and timeouts must be finite and > 0");
  }
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
  if (inbox_.size() >= 256 && !is_priority(pr.cmd.kind)) {
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
  inbox_.push_back(Inbound{client, std::move(pr)});
}

void GatewayNode::reply(int client, bool has_id, int64_t id, bool ok, const std::string& code,
                        const std::string& reason, const std::string& data_json) {
  JsonLine r;
  r.integer("v", kProtocolVersion);
  if (has_id) r.integer("id", id);
  r.boolean("ok", ok).str("code", code).str("reason", reason).raw("data", data_json);
  ipc_.send(client, r.dump());
}

std::string GatewayNode::gateway_json(double now_s) const {
  const OperatorLinkState s = link_.state(now_s, ipc_.clients());
  return JsonLine()
      .integer("schema", kProtocolVersion)
      .boolean("operator_alive", s.alive)
      .num("operator_age_s", s.age_s)
      .integer("clients", ipc_.clients())
      .dump();
}

template <typename Srv, typename Fill, typename Render>
void GatewayNode::call(const Inbound& in, double now_s, typename rclcpp::Client<Srv>::SharedPtr cli,
                       const char* name, Fill fill, Render render) {
  if (!cli->service_is_ready()) {
    reply(in.client, in.pr.has_id, in.pr.id, false, "service_unavailable",
          std::string(name) + " is not available");
    return;
  }
  auto req = std::make_shared<typename Srv::Request>();
  fill(*req);
  const uint64_t token = next_token_++;
  pending_[token] =
      Pending{in.client, in.pr.has_id, in.pr.id, in.pr.cmd.kind, now_s + service_timeout_s_};
  cli->async_send_request(req, [this, token, render](typename rclcpp::Client<Srv>::SharedFuture f) {
    const auto it = pending_.find(token);
    if (it == pending_.end()) return;  // already answered with "timeout"
    const Pending p = it->second;
    pending_.erase(it);
    const auto res = f.get();
    bool accepted = false;
    const std::string data = render(*res, &accepted);
    reply(p.client, p.has_id, p.id, accepted, accepted ? "ok" : "rejected",
          accepted ? "" : "refused by the target (see data.reason_code)", data);
  });
}

void GatewayNode::process(const Inbound& in, double now_s) {
  const Command& c = in.pr.cmd;
  const auto base = [](auto& r, bool* acc) {
    *acc = r.accepted;
    return JsonLine().boolean("accepted", r.accepted).integer("reason_code", r.reason_code);
  };
  switch (c.kind) {
    case CmdKind::Heartbeat:
      link_.note_heartbeat(now_s);
      reply(in.client, in.pr.has_id, in.pr.id, true, "ok", "");
      return;
    case CmdKind::GetSnapshot:
      reply(in.client, in.pr.has_id, in.pr.id, true, "ok", "",
            snap_.to_json(now_s, gateway_json(now_s)));
      return;
    case CmdKind::StartMission:
      call<dyx3_interfaces::srv::StartMission>(
          in, now_s, cli_start_, "mission start service",
          [&](auto& rq) { rq.path_artifact_sha256 = c.sha256; },
          [base](auto& r, bool* a) {
            return base(r, a).integer("mission_id", r.mission_id).dump();
          });
      return;
    case CmdKind::AbortMission:
      call<dyx3_interfaces::srv::AbortMission>(
          in, now_s, cli_abort_, "mission abort service",
          [&](auto& rq) { rq.reason_code = c.abort_reason; },
          [base](auto& r, bool* a) { return base(r, a).dump(); });
      return;
    case CmdKind::PauseMission:
      call<dyx3_interfaces::srv::PauseMission>(
          in, now_s, cli_pause_, "mission pause service", [](auto&) {},
          [base](auto& r, bool* a) { return base(r, a).dump(); });
      return;
    case CmdKind::ResumeMission:
      call<dyx3_interfaces::srv::ResumeMission>(
          in, now_s, cli_resume_, "mission resume service", [](auto&) {},
          [base](auto& r, bool* a) { return base(r, a).dump(); });
      return;
    case CmdKind::SkipPoint:
      call<dyx3_interfaces::srv::SkipPoint>(
          in, now_s, cli_skip_, "mission skip service", [](auto&) {},
          [base](auto& r, bool* a) {
            return base(r, a).integer("skipped_point_index", r.skipped_point_index).dump();
          });
      return;
    case CmdKind::Estop:
      call<dyx3_interfaces::srv::SetEmergencyStop>(
          in, now_s, cli_estop_, "motion_guard emergency-stop service",
          [&](auto& rq) {
            rq.asserted = c.flag;
            rq.source = c.source;
          },
          [base](auto& r, bool* a) { return base(r, a).dump(); });
      return;
    case CmdKind::Arm:
      call<dyx3_interfaces::srv::ArmDisarm>(
          in, now_s, cli_arm_, "px4_link arm service", [&](auto& rq) { rq.arm = c.flag; },
          [base](auto& r, bool* a) { return base(r, a).dump(); });
      return;
    case CmdKind::Offboard:
      call<dyx3_interfaces::srv::SetOffboard>(
          in, now_s, cli_offboard_, "px4_link offboard service",
          [&](auto& rq) { rq.enable = c.flag; },
          [base](auto& r, bool* a) { return base(r, a).dump(); });
      return;
    case CmdKind::SprayManual:
      call<dyx3_interfaces::srv::SetSprayManual>(
          in, now_s, cli_spray_, "spray manual service", [&](auto& rq) { rq.on = c.flag; },
          [base](auto& r, bool* a) { return base(r, a).dump(); });
      return;
  }
}

void GatewayNode::publish_operator_link(double now_s) {
  const OperatorLinkState s = link_.state(now_s, ipc_.clients());
  dyx3_interfaces::msg::OperatorLinkStatus m;
  m.stamp = get_clock()->now();
  m.alive = s.alive;
  m.age_s = static_cast<float>(s.age_s);
  pub_link_->publish(m);
}

void GatewayNode::step(double now_s) {
  std::deque<Inbound> work;
  {
    std::lock_guard<std::mutex> lk(inbox_mu_);
    work.swap(inbox_);
  }
  // E-stop and heartbeats first, then everything else in arrival order.
  std::stable_partition(work.begin(), work.end(),
                        [](const Inbound& i) { return is_priority(i.pr.cmd.kind); });
  if (!work.empty()) {
    last_batch_.clear();
    for (const auto& in : work) last_batch_.push_back(in.pr.cmd.kind);
  }
  for (const auto& in : work) process(in, now_s);

  for (auto it = pending_.begin(); it != pending_.end();) {
    if (now_s >= it->second.deadline_s) {
      const Pending p = it->second;
      reply(p.client, p.has_id, p.id, false, "timeout",
            std::string(to_string(p.kind)) + " was not answered in time");
      it = pending_.erase(it);
    } else {
      ++it;
    }
  }
  if (now_s - last_link_pub_s_ >= 1.0 / operator_link_hz_ - 1e-9) {
    last_link_pub_s_ = now_s;
    publish_operator_link(now_s);
  }
  if (now_s - last_tel_s_ >= 1.0 / telemetry_hz_ - 1e-9) {
    last_tel_s_ = now_s;
    if (ipc_.clients() > 0) {
      ipc_.broadcast(JsonLine()
                         .integer("v", kProtocolVersion)
                         .str("type", "telemetry")
                         .raw("snapshot", snap_.to_json(now_s, gateway_json(now_s)))
                         .dump());
    }
  }
}

}  // namespace dyx3_gateway
