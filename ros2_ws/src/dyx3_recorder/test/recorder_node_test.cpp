// In-process tests of the recorder node: a fake world publishes MissionState and ULog chunks, the
// bag command is a fake child, the clock is injected, DDS runs on a private domain.
#include "dyx3_recorder/recorder_node.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <thread>

using namespace dyx3_recorder;
using namespace std::chrono_literals;
namespace fs = std::filesystem;
using dyx3_interfaces::msg::MissionState;
using dyx3_interfaces::msg::RecorderStatus;
using dyx3_interfaces::msg::UlogChunk;

namespace {

std::string slurp(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  std::ostringstream o;
  o << f.rdbuf();
  return o.str();
}

const char* kGoodBag =
    "trap 'exit 0' INT; mkdir -p \"$0\"; while :; do echo xxxxxxxxxx >> \"$0/data\"; sleep 0.05; "
    "done";

struct Rig {
  std::shared_ptr<rclcpp::Context> ctx;
  std::string root;
  double now{100.0};
  time_t wall{1788617730};
  std::shared_ptr<RecorderNode> rec;
  std::shared_ptr<rclcpp::Node> world;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
  rclcpp::Publisher<MissionState>::SharedPtr p_mission;
  rclcpp::Publisher<UlogChunk>::SharedPtr p_ulog;
  rclcpp::Publisher<dyx3_interfaces::msg::Px4LinkStatus>::SharedPtr p_link;
  rclcpp::Subscription<RecorderStatus>::SharedPtr s_status;
  RecorderStatus status;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> keep;

  explicit Rig(const std::string& bag_script = kGoodBag, bool with_versions = true,
               const std::string& bag_exe = "/bin/sh") {
    ctx = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions io;
    io.set_domain_id(120 + (getpid() % 100));
    ctx->init(0, nullptr, io);
    root = (fs::temp_directory_path() / ("dyx3_recnode_" + std::to_string(getpid()) + "_" +
                                         std::to_string(reinterpret_cast<uintptr_t>(this))))
               .string();
    fs::create_directories(root + "/config/rpp");
    std::ofstream(root + "/config/rpp/params.yaml") << "a: 1\n";
    std::ofstream(root + "/config/ntrip.env") << "NTRIP_PASSWORD=hunter2\n";
    if (with_versions) std::ofstream(root + "/versions.json") << "{\"stack_sha\": \"abc\"}\n";
    rclcpp::NodeOptions o;
    o.context(ctx);
    o.append_parameter_override("runs_dir", root + "/runs");
    o.append_parameter_override("versions_file", root + "/versions.json");
    o.append_parameter_override("config_dir", root + "/config");
    o.append_parameter_override("vehicle_id", std::string("3wd-test"));
    o.append_parameter_override("bag_command",
                                std::vector<std::string>{bag_exe, "-c", bag_script, "{dir}"});
    o.append_parameter_override("bag_finalize_timeout_s", 3.0);
    rec = std::make_shared<RecorderNode>(
        o, [this]() { return now; }, [this]() { return wall; },
        [](const std::vector<std::string>&, double) {
          return std::vector<NodeParams>{
              {"spray", true, {{"spray", "max_xtrack_error_m", "double", "0.05"}}}};
        },
        false);
    rclcpp::NodeOptions wo;
    wo.context(ctx);
    world = std::make_shared<rclcpp::Node>("world", wo);
    rclcpp::ExecutorOptions eo;
    eo.context = ctx;
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(eo);
    exec->add_node(rec);
    exec->add_node(world);
    p_mission =
        world->create_publisher<MissionState>("/dyx3/mission/state", rclcpp::QoS(1).reliable());
    p_ulog = world->create_publisher<UlogChunk>("/dyx3/ulog_chunk", rclcpp::QoS(64).reliable());
    p_link = world->create_publisher<dyx3_interfaces::msg::Px4LinkStatus>(
        "/dyx3/px4_link/status", rclcpp::QoS(1).reliable());
    s_status = world->create_subscription<RecorderStatus>(
        "/dyx3/recorder/status", rclcpp::QoS(1).reliable(),
        [this](RecorderStatus::ConstSharedPtr m) { status = *m; });
    const auto end = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < end) {
      exec->spin_some(5ms);
      if (world->count_subscribers("/dyx3/mission/state") > 0 &&
          world->count_subscribers("/dyx3/ulog_chunk") > 0 &&
          world->count_subscribers("/dyx3/px4_link/status") > 0 &&
          rec->count_subscribers("/dyx3/recorder/status") > 0) {
        return;
      }
    }
    ADD_FAILURE() << "DDS discovery did not complete";
  }
  ~Rig() {
    exec.reset();
    keep.clear();
    s_status.reset();
    rec.reset();
    world.reset();
    ctx->shutdown("test done");
    std::error_code ec;
    fs::remove_all(root, ec);
  }
  void pump(int ms = 60) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) exec->spin_some(2ms);
  }
  void mission(uint8_t state, uint32_t id = 42, uint32_t run = 0) {
    MissionState m;
    m.state = state;
    m.mission_id = id;
    m.run_index = run;
    m.path_artifact_sha256 = std::string(64, 'b');
    p_mission->publish(m);
    pump(150);
  }
  void chunk(uint16_t seq, std::vector<uint8_t> d) {
    UlogChunk c;
    c.msg_sequence = seq;
    c.data = std::move(d);
    p_ulog->publish(c);
    pump(30);
  }
  std::string run_dir() const {
    for (const auto& e : fs::directory_iterator(root + "/runs")) return e.path().string();
    return "";
  }
  size_t run_count() const {
    size_t n = 0;
    if (!fs::exists(root + "/runs")) return 0;
    for (const auto& e : fs::directory_iterator(root + "/runs")) {
      (void)e;
      ++n;
    }
    return n;
  }
};

}  // namespace

TEST(RecorderNode, FullRunProducesAnEvidenceDirectoryWithProvenance) {
  Rig r;
  EXPECT_EQ(r.run_count(), 0U);
  r.mission(MissionState::STATE_READY);
  EXPECT_EQ(r.run_count(), 0U);
  r.mission(MissionState::STATE_RUNNING);
  ASSERT_EQ(r.run_count(), 1U);
  const std::string d = r.run_dir();
  EXPECT_NE(d.find("2026-09-05_141530_mission_0042"), std::string::npos);
  for (const char* f : {"manifest.json", "versions.json", "params_ros.json", "params_fcu.json",
                        "config_snapshot/rpp/params.yaml"}) {
    EXPECT_TRUE(fs::exists(d + "/" + f)) << f;
  }
  EXPECT_FALSE(fs::exists(d + "/config_snapshot/ntrip.env"));  // secrets never copied
  EXPECT_NE(slurp(d + "/manifest.json").find("3wd-test"), std::string::npos);
  EXPECT_NE(slurp(d + "/manifest.json").find(std::string(64, 'b')), std::string::npos);
  EXPECT_NE(slurp(d + "/versions.json").find("abc"), std::string::npos);
  EXPECT_NE(slurp(d + "/params_fcu.json").find("unavailable"), std::string::npos);
  EXPECT_NE(slurp(d + "/params_ros.json").find("max_xtrack_error_m"), std::string::npos);

  r.chunk(10, {1, 2, 3});
  r.chunk(11, {4, 5});
  r.chunk(14, {6});  // 12 and 13 lost
  r.rec->step(r.now += 1.0);
  r.pump(100);
  EXPECT_EQ(r.status.state, RecorderStatus::STATE_RECORDING);
  EXPECT_TRUE(r.status.bag_healthy);
  EXPECT_GT(r.status.bytes_written, 6U);
  EXPECT_GT(r.status.free_bytes, 0U);

  r.now += 30.0;
  r.wall += 30;
  r.mission(MissionState::STATE_COMPLETED);
  EXPECT_FALSE(r.rec->recording());
  const std::string summary = slurp(d + "/summary.json");
  EXPECT_NE(summary.find("\"final_state\": \"COMPLETED\""), std::string::npos);
  EXPECT_NE(summary.find("\"ulog_bytes\": 6"), std::string::npos);
  EXPECT_NE(summary.find("\"ulog_gaps\": 1"), std::string::npos);
  EXPECT_NE(summary.find("\"bag_healthy_throughout\": true"), std::string::npos);
  EXPECT_NE(summary.find("\"duration_s\": 31"), std::string::npos);
  EXPECT_NE(summary.find("\"end_utc\": \"2026-09-05T14:16:00Z\""), std::string::npos);
  EXPECT_NE(slurp(d + "/ulog/gaps.json").find("\"missing_chunks\": 2"), std::string::npos);
  EXPECT_EQ(slurp(d + "/ulog/stream.ulg"), std::string("\x01\x02\x03\x04\x05\x06", 6));
  EXPECT_NE(slurp(d + "/params_ros.json").find("\"end\": {"),
            std::string::npos);  // end snapshot recorded
  EXPECT_TRUE(fs::exists(d + "/rosbag2/data"));
  // provenance is honestly incomplete (no FCU parameter read path yet)
  EXPECT_NE(summary.find("\"provenance_complete\": false"), std::string::npos);
  r.rec->step(r.now += 1.0);
  r.pump(100);
  EXPECT_EQ(r.status.state, RecorderStatus::STATE_IDLE);
}

TEST(RecorderNode, TimesyncAtStartAndEndIsRecordedAndAStaleSampleIsNotPassedOffAsCurrent) {
  Rig r;
  auto send = [&](bool valid, int64_t off, uint32_t rtt) {
    dyx3_interfaces::msg::Px4LinkStatus s;
    s.timesync_valid = valid;
    s.timesync_offset_us = off;
    s.timesync_round_trip_us = rtt;
    r.p_link->publish(s);
    r.pump(150);
  };
  send(true, -40000, 900);  // the #28519 symptom at the start of the run
  r.mission(MissionState::STATE_RUNNING);
  const std::string d = r.run_dir();
  EXPECT_NE(slurp(d + "/manifest.json").find("\"timesync_offset_us\": -40000"), std::string::npos);
  EXPECT_NE(slurp(d + "/manifest.json").find("\"timesync_valid\": true"), std::string::npos);
  r.now += 30.0;
  send(true, 4100, 800);  // converged by the end
  r.mission(MissionState::STATE_COMPLETED);
  const std::string summary = slurp(d + "/summary.json");
  EXPECT_NE(summary.find("\"timesync_offset_us_end\": 4100"), std::string::npos);
  EXPECT_EQ(summary.find("timesync not available"), std::string::npos);

  // no fresh sample at the start or the end: recorded as not valid, with a note, never as a number
  r.now += 10.0;
  r.wall += 100;
  r.mission(MissionState::STATE_RUNNING, 43);
  r.now += 5.0;  // the last status is now 5 s old
  r.mission(MissionState::STATE_COMPLETED, 43);
  std::string second;
  for (const auto& e : fs::directory_iterator(r.root + "/runs")) {
    if (e.path().string().find("mission_0043") != std::string::npos) second = e.path().string();
  }
  ASSERT_FALSE(second.empty());
  EXPECT_NE(slurp(second + "/manifest.json").find("\"timesync_valid\": false"), std::string::npos);
  EXPECT_NE(slurp(second + "/summary.json").find("timesync not available"), std::string::npos);
}

TEST(RecorderNode, PauseKeepsRecordingAndANewMissionSplitsTheRun) {
  Rig r;
  r.mission(MissionState::STATE_RUNNING, 7);
  r.mission(MissionState::STATE_PAUSED, 7);
  EXPECT_TRUE(r.rec->recording());
  r.mission(MissionState::STATE_RUNNING, 7);
  EXPECT_EQ(r.run_count(), 1U);
  r.wall += 5;
  r.mission(MissionState::STATE_RUNNING, 8);
  EXPECT_EQ(r.run_count(), 2U);
  r.mission(MissionState::STATE_ABORTED, 8);
  size_t aborted = 0, superseded = 0;
  for (const auto& e : fs::directory_iterator(r.root + "/runs")) {
    const std::string s = slurp(e.path().string() + "/summary.json");
    aborted += s.find("\"final_state\": \"ABORTED\"") != std::string::npos;
    superseded += s.find("\"final_state\": \"SUPERSEDED\"") != std::string::npos;
  }
  EXPECT_EQ(aborted, 1U);
  EXPECT_EQ(superseded, 1U);
}

TEST(RecorderNode, ABagThatDiesMidRunIsReportedAndRecorded) {
  Rig r("mkdir -p \"$0\"; echo x > \"$0/data\"; sleep 0.3; exit 5");
  r.mission(MissionState::STATE_RUNNING);
  r.pump(600);
  r.rec->step(r.now += 1.0);
  r.pump(100);
  EXPECT_EQ(r.status.state, RecorderStatus::STATE_ERROR);
  EXPECT_FALSE(r.status.bag_healthy);
  r.mission(MissionState::STATE_COMPLETED);
  const std::string summary = slurp(r.run_dir() + "/summary.json");
  EXPECT_NE(summary.find("\"bag_healthy_throughout\": false"), std::string::npos);
  EXPECT_NE(summary.find("bag process died"), std::string::npos);
}

TEST(RecorderNode, AnUnstartableBagIsAnErrorButTheRunDirectoryAndMissionAreUntouched) {
  Rig r("", true, "/nonexistent/ros2");
  r.mission(MissionState::STATE_RUNNING);
  r.rec->step(r.now += 1.0);
  r.pump(100);
  EXPECT_EQ(r.status.state, RecorderStatus::STATE_ERROR);
  const std::string d = r.run_dir();
  EXPECT_TRUE(fs::exists(d + "/manifest.json"));  // the provenance that could be written, was
  r.mission(MissionState::STATE_COMPLETED);
  EXPECT_NE(slurp(d + "/summary.json").find("bag process could not be started"), std::string::npos);
}

TEST(RecorderNode, MissingVersionsFileIsRecordedAsMissingNotSilentlyAbsent) {
  Rig r(kGoodBag, /*with_versions=*/false);
  r.mission(MissionState::STATE_RUNNING);
  const std::string d = r.run_dir();
  EXPECT_NE(slurp(d + "/versions.json").find("unavailable"), std::string::npos);
  r.mission(MissionState::STATE_COMPLETED);
  const std::string summary = slurp(d + "/summary.json");
  EXPECT_NE(summary.find("versions file missing"), std::string::npos);
  EXPECT_NE(summary.find("\"provenance_complete\": false"), std::string::npos);
}

TEST(RecorderNode, ShutdownFinalisesAnOpenRun) {
  std::string d;
  std::string root;
  {
    Rig r;
    r.mission(MissionState::STATE_RUNNING);
    d = r.run_dir();
    root = r.root;
    r.rec.reset();  // the destructor path used by main on SIGTERM
    EXPECT_NE(slurp(d + "/summary.json").find("RECORDER_SHUTDOWN"), std::string::npos);
  }
}

// REC-002: every node of the production control graph (and the separately run services) has its
// parameters snapshotted by default. The graph is parsed from the launch file in the source tree so
// a node added there without updating the recorder fails here.
TEST(RecorderDefaults, ParamNodesCoverTheWholeLaunchGraph) {
  const std::string launch = slurp(DYX3_CONTROL_GRAPH_LAUNCH);
  ASSERT_FALSE(launch.empty()) << DYX3_CONTROL_GRAPH_LAUNCH;
  const size_t g = launch.find("GRAPH = (");
  ASSERT_NE(g, std::string::npos);
  const std::string graph = launch.substr(g, launch.find("\n)\n", g) - g);
  const std::regex row(R"re(\(\s*"([^"]+)",\s*"([^"]+)",\s*"([^"]+)",\s*"([^"]+)"\s*\))re");
  std::set<std::string> in_graph;
  for (auto it = std::sregex_iterator(graph.begin(), graph.end(), row); it != std::sregex_iterator();
       ++it)
    in_graph.insert((*it)[3].str());
  EXPECT_GE(in_graph.size(), 6U);
  const auto defaults = default_param_nodes();
  const std::set<std::string> have(defaults.begin(), defaults.end());
  for (const auto& n : in_graph) EXPECT_TRUE(have.count(n)) << "param_nodes misses graph node " << n;
  for (const char* n : {"gnss_rtk", "spray_watchdog", "recorder", "rpp", "system_gateway"})
    EXPECT_TRUE(have.count(n)) << n;
}

// REC-018 / REC-019: a node that is discovered (its parameter services exist) but never spins makes
// SyncParametersClient::list_parameters throw on its timeout. The collector must turn that into
// reachable=false with a note, never an exception, and a missing node is unreachable too.
TEST(ParamCollector, ANodeThatNeverSpinsIsUnreachableNotACrash) {
  auto ctx = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions io;
  io.set_domain_id(20 + (getpid() % 100));
  ctx->init(0, nullptr, io);
  rclcpp::NodeOptions o;
  o.context(ctx);
  auto silent = std::make_shared<rclcpp::Node>("silent_node", o);  // created, never spun
  std::vector<NodeParams> got;
  EXPECT_NO_THROW(got = collect_ros_params({"silent_node", "no_such_node"}, 0.5, ctx));
  ASSERT_EQ(got.size(), 2U);
  EXPECT_EQ(got[0].node, "silent_node");
  EXPECT_FALSE(got[0].reachable);
  EXPECT_TRUE(got[0].params.empty());
  EXPECT_FALSE(got[0].note.empty());
  EXPECT_FALSE(got[1].reachable);
  EXPECT_EQ(got[1].note, "parameter service not available");
  const std::string j = params_ros_snapshot_json("t", got);
  EXPECT_NE(j.find("\"note\": "), std::string::npos);
  silent.reset();
  ctx->shutdown("done");
}

TEST(RecorderNode, AThrowingCollectorNeverEscapesTheNode) {
  auto ctx = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions io;
  io.set_domain_id(20 + (getpid() % 100));
  ctx->init(0, nullptr, io);
  const std::string root = (fs::temp_directory_path() / ("dyx3_recthrow_" + std::to_string(getpid()))).string();
  rclcpp::NodeOptions o;
  o.context(ctx);
  o.append_parameter_override("runs_dir", root + "/runs");
  o.append_parameter_override("versions_file", root + "/versions.json");
  o.append_parameter_override("config_dir", root + "/config");
  o.append_parameter_override("bag_command",
                              std::vector<std::string>{"/bin/sh", "-c", kGoodBag, "{dir}"});
  double now = 10.0;
  {
    auto rec = std::make_shared<RecorderNode>(
        o, [&now]() { return now; }, []() { return time_t{1788617730}; },
        [](const std::vector<std::string>&, double) -> std::vector<NodeParams> {
          throw std::runtime_error("Unable to get result of list parameters service call.");
        },
        false);
    auto world = std::make_shared<rclcpp::Node>("world", o);
    rclcpp::ExecutorOptions eo;
    eo.context = ctx;
    rclcpp::executors::SingleThreadedExecutor ex(eo);
    ex.add_node(rec);
    ex.add_node(world);
    auto pub = world->create_publisher<MissionState>("/dyx3/mission/state", rclcpp::QoS(1).reliable());
    const auto until = std::chrono::steady_clock::now() + 10s;
    while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < until)
      ex.spin_some(5ms);
    MissionState m;
    m.state = MissionState::STATE_RUNNING;
    m.mission_id = 5;
    pub->publish(m);
    const auto end = std::chrono::steady_clock::now() + 300ms;
    while (std::chrono::steady_clock::now() < end) EXPECT_NO_THROW(ex.spin_some(5ms));
    ASSERT_TRUE(rec->recording());
    const std::string d = rec->current_run_dir();
    EXPECT_NO_THROW(rec.reset());  // stop from the destructor collects the end snapshot too
    const std::string summary = slurp(d + "/summary.json");
    EXPECT_NE(summary.find("parameter collector failed"), std::string::npos);
    EXPECT_NE(summary.find("\"provenance_complete\": false"), std::string::npos);
  }
  ctx->shutdown("done");
  std::error_code ec;
  fs::remove_all(root, ec);
}
