// recorder_node — see docs/contracts/dyx3_recorder.md
#include "dyx3_recorder/recorder_node.hpp"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "dyx3_recorder/run_store.hpp"

namespace dyx3_recorder {
namespace {

namespace fs = std::filesystem;
using dyx3_interfaces::msg::RecorderStatus;

double steady_now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
time_t wall_now() { return std::time(nullptr); }

std::string read_file(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  std::ostringstream o;
  o << f.rdbuf();
  return o.str();
}

std::string hostname() {
  char b[256] = {0};
  if (gethostname(b, sizeof b - 1) != 0) return "unknown";
  return b;
}

}  // namespace

std::string param_value_text(const rclcpp::ParameterValue& v) {
  // REC-020: rclcpp's to_string formats doubles with std::to_string (6 fixed decimals: 1e-7 ->
  // "0.000000"). %.17g round-trips every double exactly (PC-8 precision).
  const auto g17 = [](double d) {
    if (std::isnan(d)) return std::string("nan");
    if (std::isinf(d)) return std::string(d > 0 ? "inf" : "-inf");
    char b[40];
    std::snprintf(b, sizeof b, "%.17g", d);
    return std::string(b);
  };
  switch (v.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return g17(v.get<double>());
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
      std::string o = "[";
      const auto& a = v.get<std::vector<double>>();
      for (size_t i = 0; i < a.size(); ++i) o += (i ? ", " : "") + g17(a[i]);
      return o + "]";
    }
    default:
      return rclcpp::to_string(v);
  }
}

std::vector<std::string> default_param_nodes() {
  // Every node of dyx3_bringup/launch/control_graph.launch.py (GRAPH) plus the nodes that run as
  // their own services (dyx3-rtk, dyx3-spray-watchdog, dyx3-recorder). recorder_node_test checks
  // this list against the launch file, so a node added to the graph cannot be silently left out.
  return {"dyx3_mission", "gnss_rtk", "motion_guard",   "px4_link",      "recorder",
          "rpp",          "spray",    "spray_watchdog", "system_gateway"};
}

std::vector<NodeParams> collect_ros_params(const std::vector<std::string>& nodes, double timeout_s,
                                           rclcpp::Context::SharedPtr context) {
  std::vector<NodeParams> out;
  const auto timeout = std::chrono::duration<double>(timeout_s);
  rclcpp::Node::SharedPtr helper;
  rclcpp::Executor::SharedPtr exec;
  try {
    rclcpp::NodeOptions o;
    if (context) o.context(context);
    o.start_parameter_services(false).start_parameter_event_publisher(false);
    helper = std::make_shared<rclcpp::Node>("dyx3_recorder_params_" + std::to_string(getpid()), o);
    rclcpp::ExecutorOptions eo;
    eo.context = helper->get_node_base_interface()->get_context();
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(eo);
  } catch (const std::exception& e) {
    for (const auto& name : nodes) {
      NodeParams np;
      np.node = name;
      np.reachable = false;
      np.note = std::string("parameter client could not be created: ") + e.what();
      out.push_back(np);
    }
    return out;
  }
  for (const auto& name : nodes) {
    NodeParams np;
    np.node = name;
    // REC-018: SyncParametersClient::list_parameters throws on a timeout (rclcpp humble), e.g. a
    // node that is discovered but not spinning. Nothing may escape: this runs at run start, at run
    // stop and from the destructor.
    try {
      rclcpp::SyncParametersClient cli(exec, helper, name);
      if (!cli.wait_for_service(timeout)) {
        np.reachable = false;
        np.note = "parameter service not available";
      } else {
        const auto names = cli.list_parameters({}, 10, timeout).names;
        const auto values = cli.get_parameters(names, timeout);
        if (values.size() != names.size()) {
          // REC-019: get_parameters returns an empty vector on a timeout; never record that as a
          // reachable node without parameters.
          np.reachable = false;
          np.note = "get_parameters returned " + std::to_string(values.size()) + " of " +
                    std::to_string(names.size()) + " values";
        } else {
          for (const auto& p : values) {
            np.params.push_back(ParamEntry{name, p.get_name(), p.get_type_name(),
                                           param_value_text(p.get_parameter_value())});
          }
        }
      }
    } catch (const std::exception& e) {
      np.reachable = false;
      np.params.clear();
      np.note = std::string("parameter request failed: ") + e.what();
    } catch (...) {
      np.reachable = false;
      np.params.clear();
      np.note = "parameter request failed: unknown exception";
    }
    out.push_back(np);
  }
  return out;
}

RecorderNode::RecorderNode(const rclcpp::NodeOptions& options, ClockFn clock, WallFn wall,
                           ParamCollector collector, bool create_timer)
    : rclcpp::Node("recorder", options),
      clock_(clock ? std::move(clock) : ClockFn(steady_now_s)),
      wall_(wall ? std::move(wall) : WallFn(wall_now)),
      collector_(collector ? std::move(collector)
                           : ParamCollector([ctx = options.context()](
                                                const std::vector<std::string>& n, double t) {
                               return collect_ros_params(n, t, ctx);
                             })),
      free_fn_(fs_free_bytes) {
  declare_params();
  // REC-009: before any new run, mark what a previous recorder left open.
  for (const auto& n : mark_interrupted_runs(runs_dir_, iso_utc(wall_())))
    RCLCPP_WARN(get_logger(), "run %s had no summary.json: marked INTERRUPTED", n.c_str());
  cb_mission_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_ulog_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_status_ =
      create_publisher<RecorderStatus>("/dyx3/recorder/status", rclcpp::QoS(1).reliable());
  rclcpp::SubscriptionOptions mo, uo;
  mo.callback_group = cb_mission_;
  uo.callback_group = cb_ulog_;
  sub_mission_ = create_subscription<dyx3_interfaces::msg::MissionState>(
      "/dyx3/mission/state", rclcpp::QoS(1).reliable(),
      [this](dyx3_interfaces::msg::MissionState::ConstSharedPtr m) { on_mission(*m); }, mo);
  sub_ulog_ = create_subscription<dyx3_interfaces::msg::UlogChunk>(
      "/dyx3/ulog_chunk", rclcpp::QoS(64).reliable(),
      [this](dyx3_interfaces::msg::UlogChunk::ConstSharedPtr m) {
        // Every chunk, run or not: the ULog header and subscriptions are cached from the start of
        // the stream so each run's file can begin with them (REC-005).
        std::lock_guard<std::mutex> lk(mu_);
        ulog_.on_chunk(m->msg_sequence, m->first_message_offset, m->data.data(), m->data.size());
      },
      uo);
  sub_link_ = create_subscription<dyx3_interfaces::msg::Px4LinkStatus>(
      "/dyx3/px4_link/status", rclcpp::QoS(1).reliable(),
      [this](dyx3_interfaces::msg::Px4LinkStatus::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mu_);
        ts_valid_ = m->timesync_valid;
        ts_offset_us_ = m->timesync_offset_us;
        ts_rtt_us_ = m->timesync_round_trip_us;
        ts_stamp_s_ = clock_();
      });
  // REC-016: the id of the conditioned geometry RPP actually executes (X-001 / PC-2), so a run
  // proves which geometry ran, not only which artifact was loaded.
  sub_rpp_ = create_subscription<dyx3_interfaces::msg::RppStatus>(
      "/dyx3/rpp/status", rclcpp::QoS(1).reliable(),
      [this](dyx3_interfaces::msg::RppStatus::ConstSharedPtr m) {
        if (m->conditioned_execution_sha256.empty()) return;
        std::lock_guard<std::mutex> lk(mu_);
        rpp_sha_ = m->conditioned_execution_sha256;
        rpp_sha_mission_ = m->mission_id;
        if (run_dir_.empty() || finalizing_ || m->mission_id != info_.mission_id) return;
        std::string& have = summary_.conditioned_execution_sha256;
        if (have.empty()) {
          have = m->conditioned_execution_sha256;
        } else if (have != m->conditioned_execution_sha256) {
          summary_.notes.push_back("conditioned_execution_sha256 changed during the run: " + have +
                                   " -> " + m->conditioned_execution_sha256);
          have = m->conditioned_execution_sha256;
        }
      });
  if (create_timer) {
    timer_ = create_wall_timer(
        std::chrono::duration<double>(0.5 / status_hz_), [this]() { step(clock_()); },
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
  }
  RCLCPP_INFO(get_logger(), "recorder up: runs in %s, %zu topics", runs_dir_.c_str(),
              topics_.size());
}

RecorderNode::~RecorderNode() {
  std::lock_guard<std::mutex> tl(run_mu_);
  if (lifecycle_.recording()) stop_run("RECORDER_SHUTDOWN");
  join_param_job();
}

void RecorderNode::declare_params() {
  runs_dir_ = declare_parameter<std::string>("runs_dir", "/var/lib/dyx3/runs");
  versions_file_ = declare_parameter<std::string>("versions_file", "/etc/dyx3/versions.json");
  // REC-007: the directory the control graph actually reads its <node>.yaml from
  // (control_graph.launch.py config_dir). Secrets there (*.env, tokens, auth stores) are excluded
  // by is_secret_name.
  config_dir_ = declare_parameter<std::string>("config_dir", "/etc/dyx3");
  vehicle_id_ = declare_parameter<std::string>("vehicle_id", "unknown");
  operator_ = declare_parameter<std::string>("operator", "unknown");
  // DERIVED — NOT FROM V1 SPEC: the /dyx3/** set that exists today. Raw /fmu/out/** and the
  // high-rate /dyx3/rtcm are not recorded (OPEN, see the contract).
  topics_ = declare_parameter<std::vector<std::string>>(
      "topics",
      {"/dyx3/vehicle_state", "/dyx3/estimator_health", "/dyx3/rtk_status", "/dyx3/gnss_report",
       "/dyx3/ntrip_status", "/dyx3/px4_link/status", "/dyx3/rpp/motion_setpoint",
       "/dyx3/rpp/status", "/dyx3/motion_guard/command", "/dyx3/motion_guard/status",
       "/dyx3/safety_gate", "/dyx3/emergency_stop_state", "/dyx3/operator_link",
       "/dyx3/mission/state", "/dyx3/mission/point_result", "/dyx3/spray/state",
       "/dyx3/spray/status", "/dyx3/spray/lease", "/dyx3/spray/actuator_command",
       "/dyx3/spray/actuator_ack", "/dyx3/spray/watchdog_status", "/dyx3/recorder/status",
       // REC-003: every LIVE parameter
       // change of every node, and
       // every node's log, mid-run
       "/parameter_events", "/rosout"});
  param_nodes_ = declare_parameter<std::vector<std::string>>("param_nodes", default_param_nodes());
  bag_command_ = declare_parameter<std::vector<std::string>>(
      "bag_command", {"ros2", "bag", "record", "-o", "{dir}"});
  // Bag format (owner requirement: runs in MB, not GB; REC-021: survive a power cut). sqlite3 with
  // the "resilient" preset (WAL + synchronous=NORMAL instead of journal in memory +
  // synchronous=OFF), zstd FILE compression of each closed split, and a split every
  // bag_max_duration_s so that after a power cut only the last split is uncompressed (and still a
  // valid WAL database).
  bag_storage_ = declare_parameter<std::string>("bag_storage", "sqlite3");
  bag_storage_preset_ = declare_parameter<std::string>("bag_storage_preset", "resilient");
  bag_compression_mode_ = declare_parameter<std::string>("bag_compression_mode", "file");
  bag_compression_format_ = declare_parameter<std::string>("bag_compression_format", "zstd");
  bag_compression_threads_ = declare_parameter<int64_t>("bag_compression_threads", 1);
  bag_max_duration_s_ = declare_parameter<int64_t>("bag_max_duration_s", 300);
  bag_finalize_timeout_s_ = declare_parameter<double>("bag_finalize_timeout_s", 10.0);
  param_timeout_s_ = declare_parameter<double>("param_timeout_s", 2.0);
  status_hz_ = declare_parameter<double>("status_hz", 2.0);
  // REC-008: dyx3_mission publishes its state at 10 Hz. If it falls silent while a run is open
  // (the graph died and was not restarted), the run is closed as MISSION_STATE_LOST instead of
  // recording forever.
  mission_silence_s_ = declare_parameter<double>("mission_silence_s", 3.0);
  max_bag_restarts_ = declare_parameter<int64_t>("max_bag_restarts", 1);
  if (max_bag_restarts_ < 0)
    throw std::invalid_argument("recorder parameter invalid: max_bag_restarts");
  // REC-001: the runs share /var/lib/dyx3 with the missions, the RTK state and the spray-ACK
  // ledger. Below min_free_bytes no run starts and a running bag is stopped (checked at every
  // step); retention keeps the complete runs under max_runs_bytes. 0 disables either.
  const int64_t min_free = declare_parameter<int64_t>("min_free_bytes", int64_t{2} << 30);
  const int64_t max_runs = declare_parameter<int64_t>("max_runs_bytes", int64_t{20} << 30);
  if (min_free < 0 || max_runs < 0)
    throw std::invalid_argument("recorder parameter invalid: min_free_bytes / max_runs_bytes < 0");
  min_free_bytes_ = static_cast<uint64_t>(min_free);
  max_runs_bytes_ = static_cast<uint64_t>(max_runs);
  const auto bad = [](double v) { return !std::isfinite(v) || v <= 0.0; };
  if (bad(bag_finalize_timeout_s_) || bad(param_timeout_s_) || bad(status_hz_) ||
      bad(mission_silence_s_)) {
    throw std::invalid_argument(
        "recorder parameter invalid: timeouts and rates must be finite and > 0");
  }
  if (runs_dir_.empty() || bag_command_.empty() || bag_storage_.empty())
    throw std::invalid_argument("recorder parameter invalid: empty");
  if (bag_compression_mode_ != "none" && bag_compression_mode_ != "file" &&
      bag_compression_mode_ != "message")
    throw std::invalid_argument("recorder parameter invalid: bag_compression_mode");
  if (bag_compression_mode_ != "none" && bag_compression_format_.empty())
    throw std::invalid_argument("recorder parameter invalid: bag_compression_format");
  if (bag_max_duration_s_ < 0 || bag_compression_threads_ < 0)
    throw std::invalid_argument("recorder parameter invalid: negative bag setting");
}

std::vector<std::string> RecorderNode::bag_argv(const std::string& bag_dir) const {
  std::vector<std::string> argv;
  for (auto a : bag_command_) {
    for (size_t p; (p = a.find("{dir}")) != std::string::npos;) a.replace(p, 5, bag_dir);
    argv.push_back(a);
  }
  for (const auto& a : bag_options()) argv.push_back(a);
  for (const auto& t : topics_) argv.push_back(t);
  return argv;
}

std::vector<std::string> RecorderNode::bag_options() const {
  std::vector<std::string> o{"--storage", bag_storage_};
  if (!bag_storage_preset_.empty()) {
    o.push_back("--storage-preset-profile");
    o.push_back(bag_storage_preset_);
  }
  if (bag_compression_mode_ != "none") {
    o.insert(o.end(), {"--compression-mode", bag_compression_mode_, "--compression-format",
                       bag_compression_format_});
    if (bag_compression_threads_ > 0) {
      o.push_back("--compression-threads");
      o.push_back(std::to_string(bag_compression_threads_));
    }
  }
  if (bag_max_duration_s_ > 0) {
    o.push_back("--max-bag-duration");
    o.push_back(std::to_string(bag_max_duration_s_));
  }
  return o;
}

// REC-011: every evidence file write is checked; a failure is a note and a provenance gap.
static bool write_evidence(const std::string& path, const std::string& content,
                           RunSummary& summary) {
  if (write_file_atomic(path, content)) return true;
  summary.notes.push_back(fs::path(path).filename().string() + " could not be written");
  summary.provenance_complete = false;
  return false;
}

std::vector<NodeParams> RecorderNode::collect_params() const {
  // An injected collector is as untrusted as the default one: nothing escapes into the node.
  try {
    return collector_(param_nodes_, param_timeout_s_);
  } catch (const std::exception& e) {
    std::vector<NodeParams> out;
    for (const auto& n : param_nodes_)
      out.push_back(
          NodeParams{n, false, {}, std::string("parameter collector failed: ") + e.what()});
    return out;
  } catch (...) {
    std::vector<NodeParams> out;
    for (const auto& n : param_nodes_)
      out.push_back(NodeParams{n, false, {}, "parameter collector failed"});
    return out;
  }
}

void RecorderNode::set_free_space_source(FreeSpaceFn f) {
  std::lock_guard<std::mutex> lk(mu_);
  free_fn_ = f ? std::move(f) : FreeSpaceFn(fs_free_bytes);
}

uint64_t RecorderNode::free_space() const {
  FreeSpaceFn f;
  {
    std::lock_guard<std::mutex> lk(mu_);
    f = free_fn_;
  }
  return f(runs_dir_);
}

bool RecorderNode::recording() const {
  std::lock_guard<std::mutex> lk(mu_);
  return lifecycle_.recording();
}

std::string RecorderNode::current_run_dir() const {
  std::lock_guard<std::mutex> lk(mu_);
  return run_dir_;
}

void RecorderNode::on_mission(const dyx3_interfaces::msg::MissionState& m) {
  std::lock_guard<std::mutex> tl(run_mu_);
  LifecycleAction a;
  {
    std::lock_guard<std::mutex> lk(mu_);
    last_mission_s_ = clock_();
    if (!lifecycle_.recording() && last_mission_s_ < retry_after_s_) return;  // REC-017 backoff
    a = lifecycle_.on_mission(m.state, m.mission_id, m.run_index);
  }
  if (a.stop) {
    if (!a.stop_note.empty()) {
      std::lock_guard<std::mutex> lk(mu_);
      summary_.notes.push_back(a.stop_note);
    }
    stop_run(a.final_state);
  }
  if (a.start) start_run(m.mission_id, m.run_index, m.path_artifact_sha256, a.start_running);
  if (a.running) mark_running();
}

void RecorderNode::mark_running() {
  std::lock_guard<std::mutex> lk(mu_);
  if (run_dir_.empty()) return;
  summary_.running_utc = iso_utc(wall_());
  summary_.preroll_s = clock_() - run_start_s_;
  RCLCPP_INFO(get_logger(), "mission RUNNING after %.1f s of pre-roll", summary_.preroll_s);
}

void RecorderNode::join_param_job() {
  if (param_thread_.joinable()) param_thread_.join();
}

void RecorderNode::start_run(uint32_t mission_id, uint32_t run_index, const std::string& sha,
                             bool running) {
  join_param_job();  // normally already joined by stop_run
  std::error_code ec;
  const time_t now = wall_();
  RunInfo info;
  info.mission_id = mission_id;
  info.run_index = run_index;
  info.path_artifact_sha256 = sha;
  info.start_utc = iso_utc(now);
  info.start_state = running ? "RUNNING" : "READY";
  info.vehicle_id = vehicle_id_;
  info.operator_name = operator_;
  info.hostname = hostname();
  {
    std::lock_guard<std::mutex> lk(mu_);
    // A sample older than 1 s (the link publishes at 10 Hz) is not a measurement of "now".
    const bool fresh = ts_valid_ && (clock_() - ts_stamp_s_) <= 1.0;
    info.timesync_valid = fresh;
    info.timesync_offset_us = fresh ? ts_offset_us_ : 0;
    info.timesync_round_trip_us = fresh ? ts_rtt_us_ : 0U;
    if (rpp_sha_mission_ == mission_id) info.conditioned_execution_sha256 = rpp_sha_;
  }
  fs::create_directories(runs_dir_, ec);
  RunSummary summary;
  summary.conditioned_execution_sha256 = info.conditioned_execution_sha256;
  // Retention first (no run is open here: stop_run came before): it may free the space we need.
  if (max_runs_bytes_ > 0) {
    const PruneResult pr = prune_runs(runs_dir_, max_runs_bytes_);
    if (!pr.removed.empty()) {
      summary.notes.push_back("retention removed " + std::to_string(pr.removed.size()) +
                              " old complete run(s), " + std::to_string(pr.bytes_removed) +
                              " bytes (max_runs_bytes " + std::to_string(max_runs_bytes_) + ")");
      RCLCPP_WARN(get_logger(), "retention removed %zu old run(s), oldest %s", pr.removed.size(),
                  pr.removed.front().c_str());
    }
    for (const auto& e : pr.errors) summary.notes.push_back("retention: " + e);
    if (pr.bytes_after > max_runs_bytes_)
      summary.notes.push_back("runs exceed max_runs_bytes and nothing more may be pruned");
  }
  const uint64_t free_now = free_space();
  const bool disk_low = min_free_bytes_ > 0 && free_now < min_free_bytes_;
  const std::string dir = unique_run_path(runs_dir_, run_dir_name(now, mission_id, run_index));
  info.run_id = fs::path(dir).filename().string();
  bool dir_ok = fs::create_directories(dir, ec) && !ec;

  if (!dir_ok) {
    // REC-017: forget the run so a later MissionState retries, after a growing backoff (the
    // mission publishes at 10 Hz).
    std::lock_guard<std::mutex> lk(mu_);
    error_ = true;
    lifecycle_.reset();
    retry_after_s_ = clock_() + dir_backoff_s_;
    RCLCPP_ERROR(get_logger(), "cannot create run directory %s (%s): retry in %.0f s", dir.c_str(),
                 ec.message().c_str(), dir_backoff_s_);
    dir_backoff_s_ = std::min(2.0 * dir_backoff_s_, 30.0);
    return;
  }
  {
    std::lock_guard<std::mutex> lk(mu_);
    dir_backoff_s_ = 1.0;
  }
  write_evidence(dir + "/manifest.json", manifest_json(info), summary);
  if (disk_low) {
    // Only the manifest (and later summary.json) are written: never fill the shared disk.
    summary.notes.push_back("not recorded: free space " + std::to_string(free_now) +
                            " below min_free_bytes " + std::to_string(min_free_bytes_));
    summary.bag_healthy_throughout = false;
    summary.provenance_complete = false;
    std::lock_guard<std::mutex> lk(mu_);
    run_dir_ = dir;
    info_ = info;
    summary_ = summary;
    params_start_.clear();
    run_start_s_ = clock_();
    error_ = true;
    disk_stopped_ = true;
    bag_died_ = false;
    finalizing_ = false;
    RCLCPP_ERROR(get_logger(), "run %s not recorded: free space %llu below min_free_bytes %llu",
                 info.run_id.c_str(), static_cast<unsigned long long>(free_now),
                 static_cast<unsigned long long>(min_free_bytes_));
    return;
  }
  // versions.json
  const std::string versions =
      fs::exists(versions_file_, ec) ? read_file(versions_file_) : std::string();
  if (versions.empty()) {
    summary.notes.push_back("versions file missing: " + versions_file_);
    summary.provenance_complete = false;
    write_evidence(dir + "/versions.json",
                   unavailable_json("versions", "versions file missing: " + versions_file_),
                   summary);
  } else {
    write_evidence(dir + "/versions.json", versions, summary);
  }
  // config snapshot (secrets excluded)
  const CopyResult cr = copy_config_tree(config_dir_, dir + "/config_snapshot");
  if (!cr.ok) {
    for (const auto& e : cr.errors) summary.notes.push_back(e);
    summary.provenance_complete = false;
  }
  write_evidence(
      dir + "/params_fcu.json",
      unavailable_json("fcu_parameters", "no FCU parameter read path in this stack yet (OPEN)"),
      summary);
  summary.notes.push_back("params_fcu.json: unavailable (no FCU parameter read path yet)");
  summary.provenance_complete = false;
  if (running) {
    summary.running_utc = info.start_utc;
    summary.preroll_s = 0.0;
    summary.notes.push_back("run opened at RUNNING: no pre-roll");
  }

  // ulog + bag
  fs::create_directories(dir + "/ulog", ec);
  bool bag_ok = false;
  const std::vector<std::string> argv = bag_argv(dir + "/rosbag2");

  {
    std::lock_guard<std::mutex> lk(mu_);
    run_dir_ = dir;
    info_ = info;
    summary_ = summary;
    params_start_.clear();
    run_start_s_ = clock_();
    error_ = false;
    bag_died_ = false;
    disk_stopped_ = false;
    finalizing_ = false;
    bag_restarts_ = 0;
    bag_bytes_prev_ = 0;
    if (!ulog_.open(dir + "/ulog/stream.ulg")) {
      summary_.notes.push_back("ulog file could not be created");
      summary_.provenance_complete = false;
    }
    bag_ok = bag_.start(argv, dir + "/rosbag2");
    if (!bag_ok) {
      error_ = true;
      summary_.notes.push_back("bag process could not be started");
      summary_.bag_healthy_throughout = false;
      RCLCPP_ERROR(get_logger(), "bag process could not be started (%s)", argv.front().c_str());
    }
  }
  RCLCPP_INFO(get_logger(), "run started: %s (%s)", info.run_id.c_str(), info.start_state.c_str());

  // ROS parameters (start): after the bag is running, on their own thread. Up to
  // |param_nodes| x 3 RPCs x param_timeout_s must never delay the first second of the bag.
  const std::string stamp = iso_utc(now);
  param_thread_ = std::thread([this, dir, stamp]() {
    const auto nodes = collect_params();
    const std::string snapshot = params_ros_snapshot_json(stamp, nodes);
    const bool written =
        write_file_atomic(dir + "/params_ros.json", params_ros_file_json(snapshot, ""));
    std::lock_guard<std::mutex> lk(mu_);
    if (run_dir_ != dir) return;
    params_start_ = snapshot;
    if (!written) {
      summary_.notes.push_back("params_ros.json (start) could not be written");
      summary_.provenance_complete = false;
    }
    for (const auto& n : nodes) {
      if (!n.reachable) {
        summary_.notes.push_back("parameters unreachable at start: " + n.node +
                                 (n.note.empty() ? "" : " (" + n.note + ")"));
        summary_.provenance_complete = false;
      }
    }
  });
}

void RecorderNode::stop_run(const std::string& final_state) {
  join_param_job();  // the start snapshot (and its notes) belongs to this run
  std::string dir, params_start, gaps;
  RunSummary summary;
  RunInfo info;
  double start_s;
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (run_dir_.empty()) return;
    finalizing_ = true;
    dir = run_dir_;
    params_start = params_start_;
    summary = summary_;
    info = info_;
    start_s = run_start_s_;
  }
  // The bag first (it finalises its database on SIGINT), outside the lock so ULog chunks keep being
  // appended.
  const bool was_running = bag_.running();
  const int esc = bag_.stop(bag_finalize_timeout_s_, 2.0);
  if (esc > 0) {
    // REC-014: SIGTERM/SIGKILL means rosbag2 did not finalise on SIGINT within the timeout.
    summary.notes.push_back("bag needed escalation step " + std::to_string(esc) + " to stop");
    summary.bag_healthy_throughout = false;
  }
  // REC-014: a finalised rosbag2 directory has metadata.yaml; without it the bag needs a reindex.
  {
    std::error_code ec;
    std::vector<std::string> bag_dirs;
    for (auto it = fs::directory_iterator(dir, ec); !ec && it != fs::directory_iterator();
         it.increment(ec)) {
      const std::string n = it->path().filename().string();
      std::error_code e2;
      if (n.rfind("rosbag2", 0) == 0 && it->is_directory(e2)) bag_dirs.push_back(n);
    }
    std::sort(bag_dirs.begin(), bag_dirs.end());
    for (const auto& n : bag_dirs) {
      std::error_code e2;
      if (!fs::exists(dir + "/" + n + "/metadata.yaml", e2)) {
        summary.notes.push_back("rosbag2 metadata.yaml missing in " + n +
                                ": the bag was not finalised (ros2 bag reindex may recover it)");
        summary.bag_healthy_throughout = false;
      }
    }
  }
  if (!was_running && bag_.exited_abnormally()) {
    summary.notes.push_back("bag process exited with code " +
                            std::to_string(bag_.last_exit_code()));
  }
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (bag_died_) summary.bag_healthy_throughout = false;
    const bool fresh = ts_valid_ && (clock_() - ts_stamp_s_) <= 1.0;
    summary.timesync_valid_end = fresh;
    summary.timesync_offset_us_end = fresh ? ts_offset_us_ : 0;
    summary.timesync_round_trip_us_end = fresh ? ts_rtt_us_ : 0U;
    if (!info.timesync_valid || !fresh)
      summary.notes.push_back("FCU timesync not available at the start or the end of the run");
    summary.bag_bytes = bag_bytes_prev_ + bag_.bytes();
    summary.ulog_bytes = ulog_.bytes();
    summary.ulog_gaps = ulog_.gaps().size();
    summary.ulog_header = ulog_.header_status();
    if (summary.conditioned_execution_sha256.empty()) {
      summary.notes.push_back(
          "conditioned_execution_sha256 not reported by RPP for this mission during the run");
      summary.provenance_complete = false;
    }
    if (!ulog_.run_header_complete())
      summary.notes.push_back("ulog " + ulog_.header_status() +
                              ": stream.ulg cannot be decoded alone");
    if (ulog_.segments() > 1)
      summary.notes.push_back(
          "ulog stream restarted during the run: " + std::to_string(ulog_.segments()) + " files");
    ulog_.close();
    if (ulog_.write_failed()) {
      summary.notes.push_back("ulog write failed");
      summary.provenance_complete = false;
    }
    gaps = ulog_.gaps_json();
  }
  // fsync'ed writes happen outside mu_: the ULog callback and the status never wait on the disk.
  write_evidence(dir + "/ulog/gaps.json", gaps, summary);
  bool disk_stopped;
  {
    std::lock_guard<std::mutex> lk(mu_);
    disk_stopped = disk_stopped_;
  }
  if (disk_stopped) {
    summary.notes.push_back("end parameter snapshot skipped: free space below min_free_bytes");
  } else {
    const auto nodes = collect_params();
    for (const auto& n : nodes) {
      if (!n.reachable) {
        summary.notes.push_back("parameters unreachable at end: " + n.node +
                                (n.note.empty() ? "" : " (" + n.note + ")"));
      }
    }
    write_evidence(
        dir + "/params_ros.json",
        params_ros_file_json(params_start, params_ros_snapshot_json(iso_utc(wall_()), nodes)),
        summary);
  }
  summary.end_utc = iso_utc(wall_());
  summary.final_state = final_state;
  summary.duration_s = clock_() - start_s;
  if (!write_file_atomic(dir + "/summary.json", summary_json(summary))) {
    // Nothing left to note it in: the log, and the next start marks the run INTERRUPTED.
    RCLCPP_ERROR(get_logger(), "summary.json could not be written for %s", dir.c_str());
  }
  RCLCPP_INFO(get_logger(), "run finished: %s (%s)", info.run_id.c_str(), final_state.c_str());
  std::lock_guard<std::mutex> lk(mu_);
  run_dir_.clear();
  finalizing_ = false;
  error_ = false;
  disk_stopped_ = false;
}

void RecorderNode::check_bag(double) {
  // Called from step() with run_mu_ held. REC-012: a bag child that dies mid-run is restarted into
  // a new directory (rosbag2 refuses an existing one), at most max_bag_restarts times per run.
  std::lock_guard<std::mutex> lk(mu_);
  if (!lifecycle_.recording() || run_dir_.empty() || finalizing_ || error_ || disk_stopped_ ||
      bag_died_ || bag_.running())
    return;
  const int code = bag_.last_exit_code();
  summary_.bag_healthy_throughout = false;
  if (bag_restarts_ >= max_bag_restarts_) {
    bag_died_ = true;
    error_ = true;
    summary_.notes.push_back(
        "bag process died during the run (exit code " + std::to_string(code) + ")" +
        (bag_restarts_ > 0 ? ", restart limit " + std::to_string(max_bag_restarts_) + " reached"
                           : std::string()));
    RCLCPP_ERROR(get_logger(), "bag process died during the run (exit code %d)", code);
    return;
  }
  ++bag_restarts_;
  bag_bytes_prev_ += bag_.bytes();
  const std::string name = "rosbag2_" + std::to_string(bag_restarts_ + 1);
  const bool ok = bag_.start(bag_argv(run_dir_ + "/" + name), run_dir_ + "/" + name);
  summary_.notes.push_back("bag process died during the run (exit code " + std::to_string(code) +
                           "): " + (ok ? "restarted into " + name : "restart failed"));
  RCLCPP_ERROR(get_logger(), "bag process died during the run (exit code %d): %s %s", code,
               ok ? "restarted into" : "restart failed for", name.c_str());
  if (!ok) {
    bag_died_ = true;
    error_ = true;
  }
}

void RecorderNode::check_mission_silence(double now_s) {
  // Called from step() with run_mu_ held.
  double silent_s;
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!lifecycle_.recording()) return;
    silent_s = now_s - last_mission_s_;
    if (silent_s <= mission_silence_s_) return;
    lifecycle_.reset();
    char b[96];
    std::snprintf(b, sizeof b, "no MissionState for %.1f s (mission_silence_s %.1f)", silent_s,
                  mission_silence_s_);
    summary_.notes.push_back(b);
  }
  RCLCPP_ERROR(get_logger(), "MissionState silent for %.1f s: closing the run", silent_s);
  stop_run("MISSION_STATE_LOST");
}

void RecorderNode::check_disk(double) {
  // Called from step() with run_mu_ held. The bag is stopped (never killed mid-write without
  // SIGINT first) and the ULog file closed; the run stays open until the mission ends so that
  // summary.json still records how it ended.
  if (min_free_bytes_ == 0) return;
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!lifecycle_.recording() || run_dir_.empty() || disk_stopped_ || finalizing_) return;
  }
  const uint64_t free_now = free_space();
  if (free_now >= min_free_bytes_) return;
  RCLCPP_ERROR(get_logger(), "free space %llu below min_free_bytes %llu: stopping the bag",
               static_cast<unsigned long long>(free_now),
               static_cast<unsigned long long>(min_free_bytes_));
  {
    std::lock_guard<std::mutex> lk(mu_);
    disk_stopped_ = true;  // from here on step() does not report the bag as died
    error_ = true;
  }
  const int esc = bag_.stop(bag_finalize_timeout_s_, 2.0);
  std::lock_guard<std::mutex> lk(mu_);
  ulog_.close();
  summary_.bag_healthy_throughout = false;
  summary_.provenance_complete = false;
  summary_.notes.push_back(
      "recording stopped: free space " + std::to_string(free_now) + " below min_free_bytes " +
      std::to_string(min_free_bytes_) +
      (esc > 0 ? " (bag needed escalation step " + std::to_string(esc) + ")" : std::string()));
}

void RecorderNode::step(double now_s) {
  {
    std::unique_lock<std::mutex> tl(run_mu_, std::try_to_lock);
    if (tl.owns_lock()) {
      check_disk(now_s);
      check_bag(now_s);
      check_mission_silence(now_s);
    }
  }
  if (now_s - last_status_s_ >= 1.0 / status_hz_ - 1e-9) {
    last_status_s_ = now_s;
    publish_status(now_s);
  }
}

void RecorderNode::publish_status(double) {
  RecorderStatus s;
  s.stamp = get_clock()->now();
  std::lock_guard<std::mutex> lk(mu_);
  if (error_) {
    s.state = RecorderStatus::STATE_ERROR;
  } else if (finalizing_) {
    s.state = RecorderStatus::STATE_FINALIZING;
  } else if (lifecycle_.recording()) {
    s.state = RecorderStatus::STATE_RECORDING;
  } else {
    s.state = RecorderStatus::STATE_IDLE;
  }
  s.bag_healthy = lifecycle_.recording() && !error_ && !bag_died_ && bag_.running();
  s.bytes_written = lifecycle_.recording() ? bag_bytes_prev_ + bag_.bytes() + ulog_.bytes() : 0;
  s.free_bytes = free_fn_(runs_dir_);
  pub_status_->publish(s);
}

}  // namespace dyx3_recorder
