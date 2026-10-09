// rpp_node — see docs/contracts/rpp_node.md
#include "dyx3_rpp/rpp_node.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

#include "dyx3_mission/path_artifact.hpp"
#include "dyx3_mission/sha256.hpp"

namespace dyx3_rpp {
namespace {

using dyx3_interfaces::msg::MissionState;
using dyx3_interfaces::msg::MotionSetpoint;
using dyx3_interfaces::msg::RppStatus;

// The motion modes and the diagnostic codes are the frozen message ABI.
static_assert(static_cast<uint8_t>(MotionMode::Stop) == MotionSetpoint::MODE_STOP);
static_assert(static_cast<uint8_t>(MotionMode::TrackHeading) == MotionSetpoint::MODE_TRACK_HEADING);
static_assert(static_cast<uint8_t>(MotionMode::TrackRate) == MotionSetpoint::MODE_TRACK_RATE);
static_assert(static_cast<uint8_t>(MotionMode::Pivot) == MotionSetpoint::MODE_PIVOT);
static_assert(static_cast<uint8_t>(MotionMode::Creep) == MotionSetpoint::MODE_CREEP);

int64_t steady_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void require(bool ok, const std::string& what) {
  if (!ok) throw std::invalid_argument("rpp parameter invalid: " + what);
}

float finite_or_zero(double v) { return std::isfinite(v) ? static_cast<float>(v) : 0.0F; }

}  // namespace

RppNode::RppNode(const rclcpp::NodeOptions& options, ClockFn clock, bool create_timer)
    : rclcpp::Node("rpp", options),
      clock_(clock ? std::move(clock) : ClockFn(steady_now_ns)),
      core_(params_),
      timer_stats_(1e6 / 50.0) {
  declare_and_validate_params();
  timer_stats_ = LoopTimer(1e6 / tick_hz_);

  const auto rel1 = rclcpp::QoS(1).reliable();
  pub_motion_ = create_publisher<MotionSetpoint>("/dyx3/rpp/motion_setpoint", rel1);
  pub_status_ = create_publisher<RppStatus>("/dyx3/rpp/status", rel1);

  sub_vehicle_ = create_subscription<dyx3_interfaces::msg::VehicleState>(
      "/dyx3/vehicle_state", rel1, [this](dyx3_interfaces::msg::VehicleState::ConstSharedPtr m) {
        const int64_t now = clock_();
        // An invalid measurement is NOT fed: the pose then ages out and the core stops (STALE),
        // which is the fail-safe behaviour; a stale-but-plausible pose would be worse.
        if (m->position_valid && m->attitude_valid) {
          NedPose p;
          p.n = m->north_m;
          p.e = m->east_m;
          p.yaw_ned = m->heading_rad;
          core_.on_pose(p, now);
        }
        if (m->velocity_valid) {
          core_.on_velocity(m->velocity_north_mps, m->velocity_east_mps, m->yaw_rate_radps, now);
        }
      });
  sub_rtk_ = create_subscription<dyx3_interfaces::msg::RtkStatus>(
      "/dyx3/rtk_status", rel1, [this](dyx3_interfaces::msg::RtkStatus::ConstSharedPtr m) {
        // horizontal_accuracy_m == 0 is the "unknown" sentinel (A14), never a perfect fix.
        const double acc =
            (m->horizontal_accuracy_m > 0.0F && std::isfinite(m->horizontal_accuracy_m))
                ? static_cast<double>(m->horizontal_accuracy_m)
                : std::numeric_limits<double>::quiet_NaN();
        core_.on_gps(m->fix_type, acc, clock_());
      });
  sub_mission_ = create_subscription<MissionState>(
      "/dyx3/mission/state", rel1,
      [this](MissionState::ConstSharedPtr m) { on_mission_state(*m); });

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
    // RPP-006: IDLE_ONLY is refused for the whole life of a mission, not only while RUNNING.
    ctx.mission_running = mission_active_ || loaded_ || load_failed_;
    ctx.source = "ros";
    const SetResult r =
        params_.set_many(items, ctx);  // atomic; class rules; recorded in the journal
    res.successful = r.ok;
    res.reason = r.reason;
    return res;
  });

  if (create_timer) {
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tick_hz_),
                               [this]() { step(clock_()); });
  }
  RCLCPP_INFO(get_logger(), "rpp up: %.0f Hz tick, artifacts from %s", tick_hz_,
              artifact_dir_.c_str());
}

void RppNode::declare_and_validate_params() {
  // DERIVED — NOT FROM V1 SPEC: the prototype's CONTROL_HZ = 50.
  tick_hz_ = declare_parameter<double>("tick_hz", 50.0);
  require(std::isfinite(tick_hz_) && tick_hz_ >= 20.0 && tick_hz_ <= 100.0,
          "tick_hz must be in [20, 100]");
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

ConditionParams RppNode::condition_params() const {
  ConditionParams c;
  c.tracking_profile = params_.str(P::tracking_profile);
  c.segment_corner_threshold_deg = params_.num(P::segment_corner_threshold_deg);
  c.connector_absorb_m = params_.num(P::connector_absorb_m);
  c.connector_min_corner_deg = params_.num(P::connector_min_corner_deg);
  c.transit_merge_max_len_m = params_.num(P::transit_merge_max_len_m);
  c.segment_simplify_max_offset_m = params_.num(P::segment_simplify_max_offset_m);
  c.corner_smooth_radius_m = params_.num(P::corner_smooth_radius_m);
  c.corner_smooth_arc_pts = params_.integer(P::corner_smooth_arc_pts);
  c.path_resample_spacing_m = params_.num(P::path_resample_spacing_m);
  c.close_loop_threshold_m = params_.num(P::close_loop_threshold_m);
  c.close_loop_min_len_m = params_.num(P::close_loop_min_len_m);
  return c;
}

// ------------------------------------------------------------------------------------------------
// mission
// ------------------------------------------------------------------------------------------------
void RppNode::on_mission_state(const MissionState& m) {
  const bool was_running = mission_running_;
  mission_running_ = m.state == MissionState::STATE_RUNNING;
  mission_active_ = m.state == MissionState::STATE_LOADING ||
                    m.state == MissionState::STATE_READY ||
                    m.state == MissionState::STATE_RUNNING || m.state == MissionState::STATE_PAUSED;
  const bool active = mission_active_ && !m.path_artifact_sha256.empty();
  wants_mission_ = active;
  pending_mission_id_ = m.mission_id;
  pending_sha_ = m.path_artifact_sha256;
  if (!active) {
    if (loaded_ || load_failed_) unload_mission();
    return;
  }
  // A new mission id loads again even when the artifact is the same file: the run starts from
  // scratch.
  if (!(loaded_ || load_failed_) || mission_id_ != m.mission_id || sha_ != m.path_artifact_sha256) {
    load_mission(m.mission_id, m.path_artifact_sha256);
  }
  if (was_running && !mission_running_ && loaded_)
    core_.pause();  // paused: forget the motion memory
}

void RppNode::unload_mission() {
  core_.install_mission({});
  loaded_ = false;
  load_failed_ = false;
  mission_id_ = 0;
  sha_.clear();
  conditioned_sha_.clear();
  RCLCPP_INFO(get_logger(), "rpp path cleared");
}

void RppNode::load_mission(uint32_t mission_id, const std::string& sha) {
  mission_id_ = mission_id;
  sha_ = sha;
  loaded_ = false;
  repeat_available_ = false;
  load_failed_ = true;  // until proven otherwise; step() retries
  retry_load_at_ns_ = clock_() + 1'000'000'000;
  const auto r = dyx3_mission::load_artifact(artifact_dir_, sha);
  if (!r.ok) {
    RCLCPP_ERROR(get_logger(), "rpp path %s rejected: %s", sha.c_str(), r.error.c_str());
    return;
  }
  std::vector<RawPoint> raw;
  raw.reserve(r.artifact.points.size());
  for (const auto& p : r.artifact.points)
    raw.push_back({p.north_m, p.east_m, static_cast<int>(p.flags)});
  int dropped = 0;
  auto runs = condition_path(raw, condition_params(), nullptr, &dropped);
  if (runs.empty()) {
    RCLCPP_ERROR(get_logger(), "rpp path %s: conditioning produced no usable run", sha.c_str());
    return;
  }
  std::vector<dyx3_mission::ConditionedRunArtifact> artifact_runs;
  artifact_runs.reserve(runs.size());
  for (const auto& run : runs) {
    dyx3_mission::ConditionedRunArtifact a;
    a.profile = static_cast<uint8_t>(run.profile);
    for (size_t i = 0; i < run.pts.size(); ++i) {
      a.points.push_back({run.pts[i].n, run.pts[i].e});
      a.flags.push_back(i < run.flags.size() ? run.flags[i] : 0);
      a.must_hit.push_back(i < run.must_hit.size() ? run.must_hit[i] : 0);
    }
    artifact_runs.push_back(std::move(a));
  }
  const ConditionParams cp = condition_params();
  std::ostringstream config;
  config.imbue(std::locale::classic());
  config << std::setprecision(std::numeric_limits<double>::max_digits10)
         << "tracking_profile=" << cp.tracking_profile
         << ";segment_corner_threshold_deg=" << cp.segment_corner_threshold_deg
         << ";connector_absorb_m=" << cp.connector_absorb_m
         << ";connector_min_corner_deg=" << cp.connector_min_corner_deg
         << ";transit_merge_max_len_m=" << cp.transit_merge_max_len_m
         << ";segment_simplify_max_offset_m=" << cp.segment_simplify_max_offset_m
         << ";corner_smooth_radius_m=" << cp.corner_smooth_radius_m
         << ";corner_smooth_arc_pts=" << cp.corner_smooth_arc_pts
         << ";path_resample_spacing_m=" << cp.path_resample_spacing_m
         << ";close_loop_threshold_m=" << cp.close_loop_threshold_m
         << ";close_loop_min_len_m=" << cp.close_loop_min_len_m;
  const std::string conditioned_bytes =
      dyx3_mission::serialize_conditioned_artifact(sha, config.str(), artifact_runs);
  if (conditioned_bytes.empty()) {
    RCLCPP_ERROR(get_logger(), "conditioned artifact serialization failed");
    return;
  }
  conditioned_sha_ = dyx3_mission::sha256_hex(conditioned_bytes);
  std::error_code dir_ec;
  std::filesystem::create_directories(artifact_dir_, dir_ec);
  if (dir_ec) {
    conditioned_sha_.clear();
    RCLCPP_ERROR(get_logger(), "cannot create artifact directory: %s", dir_ec.message().c_str());
    return;
  }
  const auto conditioned_path =
      std::filesystem::path(artifact_dir_) / (conditioned_sha_ + ".dyx3cond");
  bool valid_existing = false;
  if (std::filesystem::exists(conditioned_path)) {
    const auto existing = dyx3_mission::load_conditioned_artifact(artifact_dir_, conditioned_sha_);
    valid_existing = existing.ok && existing.artifact.source_sha256 == sha;
  }
  if (!valid_existing) {
    const auto temp_path = conditioned_path.string() + ".tmp";
    std::ofstream out(temp_path, std::ios::binary | std::ios::trunc);
    out.write(conditioned_bytes.data(), static_cast<std::streamsize>(conditioned_bytes.size()));
    out.close();
    if (!out) {
      conditioned_sha_.clear();
      RCLCPP_ERROR(get_logger(), "cannot store conditioned artifact");
      return;
    }
    std::error_code ec;
    std::filesystem::rename(temp_path, conditioned_path, ec);
    if (ec) {
      std::filesystem::remove(temp_path);
      conditioned_sha_.clear();
      RCLCPP_ERROR(get_logger(), "cannot publish conditioned artifact: %s", ec.message().c_str());
      return;
    }
  }
  const size_t n_runs = runs.size();
  core_.install_mission(std::move(runs));
  loaded_ = true;
  load_failed_ = false;
  RCLCPP_INFO(get_logger(),
              "rpp path loaded: mission %u, %zu points -> %zu runs (%d slivers dropped)",
              mission_id, r.artifact.points.size(), n_runs, dropped);
}

// ------------------------------------------------------------------------------------------------
// the tick
// ------------------------------------------------------------------------------------------------
void RppNode::step(int64_t now_ns) {
  if (stopped_for_shutdown_) return;
  timer_stats_.note(now_ns);

  if (wants_mission_ && load_failed_ && now_ns >= retry_load_at_ns_)
    load_mission(pending_mission_id_, pending_sha_);

  MotionCommand cmd = make_stop();
  // Only the command of the previous RUNNING tick of the same mission may be repeated.
  if (!(wants_mission_ && loaded_ && mission_running_)) repeat_available_ = false;
  if (!wants_mission_ || !(loaded_ || load_failed_)) {
    publish_motion(cmd);
    publish_status(RppStatus::STATE_IDLE, nullptr, cmd);
    return;
  }
  if (load_failed_) {
    publish_motion(cmd);
    publish_status(RppStatus::STATE_ERROR, nullptr, cmd);
    return;
  }
  if (!mission_running_) {
    publish_motion(cmd);  // loaded and waiting (READY), or paused: STOP
    publish_status(RppStatus::STATE_LOADED, nullptr, cmd);
    return;
  }

  const TickOutput& out = core_.tick(now_ns);
  const bool segment_rate_command = params_.str(P::segment_command_mode) == "rate";
  cmd = command_from_tick(out, core_.profile_segment(), params_.num(P::max_yaw_rate_body),
                          segment_rate_command);

  uint8_t state = RppStatus::STATE_STOPPING;
  if (out.handoff != Handoff::None) {
    state = RppStatus::STATE_ERROR;  // an unported feature is enabled: refuse to drive and say so
    cmd = make_stop();
  } else if (core_.path_done()) {
    state = RppStatus::STATE_COMPLETE;
  } else if (!out.velocity_published) {
    // XR-RPP-002: the core switched runs without publishing (out_ defaults to STOP). Repeat the
    // previous running command once rather than drop to STOP for one tick while driving.
    if (repeat_available_) {
      cmd = last_running_cmd_;
      state = last_running_state_;
    } else {
      cmd = make_stop();
      state = RppStatus::STATE_STOPPING;
    }
    repeat_available_ = false;
  } else {
    switch (out.cmd) {
      case CmdKind::Track:
        state = (out.state == StateCode::Tracking || out.state == StateCode::Approach)
                    ? RppStatus::STATE_TRACKING
                    : RppStatus::STATE_STOPPING;
        break;
      case CmdKind::Pivot:
        state = RppStatus::STATE_PIVOTING;
        break;
      case CmdKind::Creep:
        state = RppStatus::STATE_CREEPING;
        break;
      case CmdKind::Brake:
      case CmdKind::Stop:
        state = RppStatus::STATE_STOPPING;
        break;
    }
  }
  if (out.velocity_published && out.handoff == Handoff::None && !core_.path_done()) {
    repeat_available_ = true;
    last_running_cmd_ = cmd;
    last_running_state_ = state;
  }
  publish_motion(cmd);
  publish_status(state, &out, cmd);
  if (state != last_state_) {  // transitions only: nothing is logged per tick
    RCLCPP_INFO(get_logger(), "rpp state %u -> %u (run %zu, tick state %d)",
                static_cast<unsigned>(last_state_), static_cast<unsigned>(state), core_.run_index(),
                static_cast<int>(out.state));
    last_state_ = state;
  }
}

void RppNode::publish_motion(const MotionCommand& c) {
  MotionSetpoint m;
  m.stamp = ros_now();
  m.seq = seq_++;
  m.mode = static_cast<uint8_t>(c.mode);
  m.speed_body_x = c.speed_body_x;
  m.yaw_setpoint = c.yaw_setpoint;
  m.yaw_rate_setpoint = c.yaw_rate_setpoint;
  m.valid = true;
  pub_motion_->publish(m);
}

void RppNode::publish_status(uint8_t state, const TickOutput* out, const MotionCommand& cmd) {
  RppStatus s;
  s.stamp = ros_now();
  s.state = state;
  s.mission_id = (loaded_ || load_failed_) ? mission_id_ : 0U;
  s.conditioned_execution_sha256 = loaded_ ? conditioned_sha_ : std::string();
  s.run_index = loaded_ ? static_cast<uint32_t>(core_.run_index()) : 0U;
  s.commanded_speed_mps = finite_or_zero(cmd.speed_body_x);
  s.commanded_yaw_rate_radps = finite_or_zero(cmd.yaw_rate_setpoint);
  s.loop_jitter_us = static_cast<float>(timer_stats_.jitter_us());
  s.loop_jitter_max_us = static_cast<float>(timer_stats_.max_abs_jitter_us());
  s.loop_overrun_count = timer_stats_.overruns();
  if (loaded_) s.path_travel_m = finite_or_zero(core_.snapshot().path_travel_m);
  if (out != nullptr) {
    s.cross_track_right_m = finite_or_zero(out->cross_track_right);  // right-positive
    s.heading_error_rad = finite_or_zero(out->debug.heading_err);
    s.tick_state = static_cast<int8_t>(out->state);
    s.segment_state = static_cast<uint8_t>(out->segment_debug_valid ? out->segment_debug.state : 0);
    s.spray_request = out->debug.spray_active;
    s.handoff = static_cast<uint8_t>(out->handoff);
    s.rtk_reason = static_cast<uint8_t>(out->rtk_reason);
    const bool spray_state =
        state == RppStatus::STATE_TRACKING || state == RppStatus::STATE_STOPPING ||
        state == RppStatus::STATE_PIVOTING || state == RppStatus::STATE_CREEPING;
    s.heading_evidence_valid =
        out->debug_valid && std::isfinite(out->debug.heading_err) && loaded_ && spray_state;
  }
  pub_status_->publish(s);
}

void RppNode::shutdown_stop() {
  publish_motion(make_stop());
  publish_status(RppStatus::STATE_IDLE, nullptr, make_stop());
  stopped_for_shutdown_ = true;
}

}  // namespace dyx3_rpp
