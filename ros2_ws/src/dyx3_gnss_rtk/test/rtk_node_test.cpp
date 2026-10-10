// RtkNode in a private DDS domain with an injected clock: status mapping, fail-safe defaults, chunk
// publication.
#include "dyx3_gnss_rtk/rtk_node.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <thread>

#include "dds_test_support.hpp"
#include "dyx3_gnss_rtk/rtcm_parser.hpp"

using namespace dyx3_gnss_rtk;
using namespace std::chrono_literals;

namespace {

struct Rig {
  std::shared_ptr<rclcpp::Context> ctx;
  double now{10.0};
  std::shared_ptr<RtkNode> node;
  std::shared_ptr<rclcpp::Node> world;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
  rclcpp::Publisher<dyx3_interfaces::msg::GnssReport>::SharedPtr p_report;
  rclcpp::Publisher<dyx3_interfaces::msg::MissionState>::SharedPtr p_mission;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> keep;
  dyx3_interfaces::msg::RtkStatus rtk;
  dyx3_interfaces::msg::NtripStatus ntrip;
  std::vector<dyx3_interfaces::msg::RtcmData> chunks;
  std::string config_dir;

  Rig() {
    char pattern[] = "/tmp/dyx3-node-config-XXXXXX";
    config_dir = ::mkdtemp(pattern);
    ::setenv("DYX3_RTK_STATE_DIR", config_dir.c_str(), 1);
    ctx = std::make_shared<rclcpp::Context>();
    dyx3_test::init_isolated(ctx);
    rclcpp::NodeOptions no;
    no.context(ctx);
    auto config = RtkConfigStore::initial_from_environment();
    config["transport"] = "PX4_DDS";
    node = std::make_shared<RtkNode>(no, [this]() { return now; }, false, false, config);
    rclcpp::NodeOptions wo;
    wo.context(ctx);
    world = std::make_shared<rclcpp::Node>("world", wo);
    rclcpp::ExecutorOptions eo;
    eo.context = ctx;
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(eo);
    exec->add_node(node);
    exec->add_node(world);
    p_report = world->create_publisher<dyx3_interfaces::msg::GnssReport>("/dyx3/gnss_report",
                                                                         rclcpp::QoS(5));
    p_mission = world->create_publisher<dyx3_interfaces::msg::MissionState>(
        "/dyx3/mission/state", rclcpp::QoS(1).reliable());
    keep.push_back(world->create_subscription<dyx3_interfaces::msg::RtkStatus>(
        "/dyx3/rtk_status", rclcpp::QoS(1).reliable(),
        [this](dyx3_interfaces::msg::RtkStatus::ConstSharedPtr m) { rtk = *m; }));
    keep.push_back(world->create_subscription<dyx3_interfaces::msg::NtripStatus>(
        "/dyx3/ntrip_status", rclcpp::QoS(1).reliable(),
        [this](dyx3_interfaces::msg::NtripStatus::ConstSharedPtr m) { ntrip = *m; }));
    keep.push_back(world->create_subscription<dyx3_interfaces::msg::RtcmData>(
        "/dyx3/rtcm", rclcpp::QoS(32).reliable(),
        [this](dyx3_interfaces::msg::RtcmData::ConstSharedPtr m) { chunks.push_back(*m); }));
    const auto end = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < end) {
      exec->spin_some(5ms);
      if (node->count_subscribers("/dyx3/rtk_status") > 0 &&
          node->count_subscribers("/dyx3/rtcm") > 0 &&
          node->count_subscribers("/dyx3/ntrip_status") > 0 &&
          world->count_subscribers("/dyx3/gnss_report") > 0 &&
          world->count_subscribers("/dyx3/mission/state") > 0) {
        return;
      }
    }
    ADD_FAILURE() << "DDS discovery did not complete";
  }
  ~Rig() {
    exec.reset();
    keep.clear();
    node.reset();
    world.reset();
    ctx->shutdown("test done");
    ::unsetenv("DYX3_RTK_STATE_DIR");
    std::filesystem::remove_all(config_dir);
  }
  void pump(int ms = 60) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) exec->spin_some(2ms);
  }
  void report(uint8_t fix, float acc = 0.02F) {
    dyx3_interfaces::msg::GnssReport r;
    r.valid = true;
    r.fix_type = fix;
    r.horizontal_accuracy_m = acc;
    r.satellites_used = 22;
    r.latitude_deg = 48.0;
    r.longitude_deg = 11.0;
    p_report->publish(r);
    pump();
  }
  void mission(uint8_t state) {
    dyx3_interfaces::msg::MissionState m;
    m.state = state;
    p_mission->publish(m);
    pump();
  }
  std::vector<uint8_t> frame(size_t n) {
    std::vector<uint8_t> f(n, 0x11);
    f[0] = 0xD3;
    const size_t payload = n - 6;
    f[1] = static_cast<uint8_t>((payload >> 8) & 0x03);
    f[2] = static_cast<uint8_t>(payload);
    const uint32_t crc = crc24q(f.data(), n - 3);
    f[n - 3] = static_cast<uint8_t>(crc >> 16);
    f[n - 2] = static_cast<uint8_t>(crc >> 8);
    f[n - 1] = static_cast<uint8_t>(crc);
    return f;
  }
};

}  // namespace

TEST(RtkNode, FailSafeDefaultsWithNothingReceived) {
  Rig r;
  r.node->publish_status(r.now);
  r.node->publish_ntrip_status(r.now);
  r.pump();
  EXPECT_EQ(r.rtk.fix_type, dyx3_interfaces::msg::RtkStatus::FIX_UNKNOWN);
  EXPECT_FALSE(r.rtk.corrections_fresh);
  EXPECT_GT(r.rtk.correction_age_s, 1e6F);
  EXPECT_FALSE(r.ntrip.connected);
  EXPECT_FALSE(r.ntrip.streaming);
  EXPECT_EQ(r.ntrip.state,
            dyx3_interfaces::msg::NtripStatus::STATE_ERROR);  // not configured: loud, not silent
  EXPECT_NE(r.ntrip.last_error.find("not configured"), std::string::npos);
}

TEST(RtkNode, FixAndAccuracyPassThroughOnlyWhileTheReportIsFresh) {
  Rig r;
  r.report(6, 0.012F);
  r.now += 0.1;
  r.node->publish_status(r.now);
  r.pump();
  EXPECT_EQ(r.rtk.fix_type, 6);
  EXPECT_NEAR(r.rtk.horizontal_accuracy_m, 0.012F, 1e-6);
  EXPECT_EQ(r.rtk.satellites_used, 22);
  EXPECT_FALSE(r.rtk.corrections_fresh);  // no RTCM frame yet: never assumed fresh
  r.now += 2.0;                           // the FCU report goes stale (limit 1 s)
  r.node->publish_status(r.now);
  r.pump();
  EXPECT_EQ(r.rtk.fix_type, dyx3_interfaces::msg::RtkStatus::FIX_UNKNOWN);
  EXPECT_EQ(r.rtk.horizontal_accuracy_m, 0.0F);
}

TEST(RtkNode, CorrectionsAreFreshOnlyWithBothFramesAndAFreshReport) {
  Rig r;
  r.report(6);
  r.node->on_frame(
      r.frame(40));  // a frame arrives... but the stream is "disconnected" (client not started)
  r.now += 0.1;
  r.node->publish_status(r.now);
  r.pump();
  EXPECT_FALSE(r.rtk.corrections_fresh);  // CorrectionHealth requires a connected stream
}

TEST(RtkNode, FramesBecomeChunksWithFlags) {
  Rig r;
  r.node->on_frame(r.frame(100));
  r.node->on_frame(r.frame(700));  // 2 chunks (300 + 300 + 100 -> 3 chunks of a 700-byte frame)
  r.pump(200);
  ASSERT_EQ(r.chunks.size(), 4U);
  EXPECT_EQ(r.chunks[0].data.size(), 100U);
  EXPECT_EQ(r.chunks[0].flags, 0U);
  EXPECT_EQ(r.chunks[1].flags & 1U, 1U);
  EXPECT_EQ((r.chunks[1].flags >> 3), 1U);
  EXPECT_EQ((r.chunks[2].flags >> 1) & 3U, 1U);
  EXPECT_EQ((r.chunks[3].flags >> 1) & 3U, 2U);
  r.now += 0.1;
  r.node->publish_ntrip_status(r.now);
  r.pump();
  EXPECT_EQ(r.ntrip.frames_total, 2U);
  EXPECT_EQ(r.ntrip.chunks_handed_off, 4U);
  EXPECT_EQ(r.ntrip.source_bytes_received, 0U);  // direct test injection bypassed NTRIP
  EXPECT_EQ(r.ntrip.valid_rtcm_frames, 0U);
}

TEST(RtkNode, OversizeFrameIsNeverTruncated) {
  Rig r;
  r.node->on_frame(std::vector<uint8_t>(2000, 0xD3));
  r.pump(150);
  EXPECT_TRUE(r.chunks.empty());
}

TEST(RtkNode, FixTransitionsAreCounted) {
  Rig r;
  r.report(3);
  r.now += 0.1;
  r.node->publish_status(r.now);
  r.report(6);
  r.now += 0.1;
  r.node->publish_status(r.now);
  r.now += 0.1;
  r.node->publish_ntrip_status(r.now);
  r.pump();
  EXPECT_EQ(r.ntrip.fix_transitions, 1U);
  EXPECT_EQ(r.ntrip.fix_type, 6);
  EXPECT_EQ(r.ntrip.chunks_handed_off, 0U);
}

TEST(RtkNode, InvalidConfigLeavesDdsActiveAndUsbSwitchStopsDdsPublication) {
  Rig r;
  const auto before = r.node->handle_control({{"v", 1}, {"cmd", "GET_CONFIG"}})["data"];
  auto invalid = before;
  invalid["source"] = "UNKNOWN";
  EXPECT_THROW(r.node->handle_control({{"v", 1}, {"cmd", "SET_CONFIG"}, {"config", invalid}}),
               ConfigError);
  r.node->on_frame(r.frame(40));
  r.pump();
  ASSERT_EQ(r.chunks.size(), 1U);
  auto usb = before;
  usb["transport"] = "USB_DIRECT";  // no by-id path: fail closed, never publish DDS
  const auto reply = r.node->handle_control({{"v", 1}, {"cmd", "SET_CONFIG"}, {"config", usb}});
  EXPECT_EQ(reply["data"]["transport"], "USB_DIRECT");
  r.node->on_frame(r.frame(40));
  r.pump();
  EXPECT_EQ(r.chunks.size(), 1U);
  EXPECT_EQ(r.node->status_json(r.now)["transport"]["selected"], "USB_DIRECT");
}

TEST(RtkNodeStartup, InvalidPersistedConfigStartsStoppedInsteadOfCrashing) {
  char pattern[] = "/tmp/dyx3-node-bad-config-XXXXXX";
  const std::string dir = ::mkdtemp(pattern);
  {
    std::ofstream bad(dir + "/config.json");
    bad << "{\"schema\": 1, \"source\": \"NTRIP\"";  // truncated write
  }
  ::setenv("DYX3_RTK_STATE_DIR", dir.c_str(), 1);
  auto ctx = std::make_shared<rclcpp::Context>();
  dyx3_test::init_isolated(ctx);
  rclcpp::NodeOptions no;
  no.context(ctx);
  double now = 10.0;
  std::shared_ptr<RtkNode> node;
  ASSERT_NO_THROW(node = std::make_shared<RtkNode>(no, [&now]() { return now; }, false, false));
  const auto status = node->status_json(now);
  EXPECT_EQ(status["worker_state"], "ERROR");
  EXPECT_EQ(status["desired_state"], "STOPPED");
  // The operator can replace the configuration through the control interface.
  auto config = node->handle_control({{"v", 1}, {"cmd", "GET_CONFIG"}})["data"];
  config["transport"] = "PX4_DDS";
  const auto reply = node->handle_control({{"v", 1}, {"cmd", "SET_CONFIG"}, {"config", config}});
  EXPECT_EQ(reply["data"]["transport"], "PX4_DDS");
  EXPECT_TRUE(RtkConfigStore(dir).load().has_value());
  node.reset();
  ctx->shutdown("test done");
  ::unsetenv("DYX3_RTK_STATE_DIR");
  std::filesystem::remove_all(dir);
}

TEST(RtkNodeConfigLock, FreshActiveMissionRefusesEveryWorkerRestartingCommand) {
  using dyx3_interfaces::msg::MissionState;
  for (const uint8_t state :
       {MissionState::STATE_LOADING, MissionState::STATE_PLACING, MissionState::STATE_ARMING,
        MissionState::STATE_ENGAGING, MissionState::STATE_READY, MissionState::STATE_RUNNING,
        MissionState::STATE_PAUSED}) {
    Rig r;
    const auto before = r.node->handle_control({{"v", 1}, {"cmd", "GET_CONFIG"}})["data"];
    const auto status_before = r.node->status_json(r.now);
    r.mission(state);
    r.now += 0.1;  // fresh: well inside the 1 s limit
    auto changed = before;
    changed["transport"] = "USB_DIRECT";
    const Json set = {{"v", 1}, {"cmd", "SET_CONFIG"}, {"config", changed}};
    for (const Json& request :
         {set, Json{{"v", 1}, {"cmd", "START"}}, Json{{"v", 1}, {"cmd", "STOP"}}}) {
      const auto reply = r.node->handle_control(request);
      EXPECT_FALSE(reply["ok"].get<bool>()) << "state " << static_cast<int>(state);
      EXPECT_EQ(reply["code"], "conflict");
      EXPECT_EQ(reply["reason"], "mission active: configuration is locked");
    }
    // Config unchanged and the worker untouched: same revision, transport, worker state and
    // desired state, and an RTCM frame still reaches the DDS transport.
    const auto after = r.node->handle_control({{"v", 1}, {"cmd", "GET_CONFIG"}})["data"];
    EXPECT_EQ(after, before);
    const auto status_after = r.node->status_json(r.now);
    EXPECT_EQ(status_after["config_revision"], status_before["config_revision"]);
    EXPECT_EQ(status_after["worker_state"], status_before["worker_state"]);
    EXPECT_EQ(status_after["desired_state"], status_before["desired_state"]);
    EXPECT_EQ(status_after["transport"]["selected"], "PX4_DDS");
    EXPECT_EQ(status_after["events"], status_before["events"]);  // no RECONFIGURING transition
    r.node->on_frame(r.frame(40));
    r.pump();
    EXPECT_EQ(r.chunks.size(), 1U);
    // Reads stay allowed while the lock is held.
    EXPECT_TRUE(r.node->handle_control({{"v", 1}, {"cmd", "GET_STATUS"}})["ok"].get<bool>());
    EXPECT_TRUE(r.node->handle_control({{"v", 1}, {"cmd", "GET_CONFIG"}})["ok"].get<bool>());
  }
}

TEST(RtkNodeConfigLock, NoMissionStateIdleTerminalOrStaleStateDoNotLock) {
  using dyx3_interfaces::msg::MissionState;
  auto set_usb = [](Rig& r) {
    auto config = r.node->handle_control({{"v", 1}, {"cmd", "GET_CONFIG"}})["data"];
    config["transport"] = config["transport"] == "USB_DIRECT" ? "PX4_DDS" : "USB_DIRECT";
    return r.node->handle_control({{"v", 1}, {"cmd", "SET_CONFIG"}, {"config", config}});
  };
  {  // the RTK service starts before the graph: no state ever seen
    Rig r;
    EXPECT_TRUE(set_usb(r)["ok"].get<bool>());
  }
  for (const uint8_t state : {MissionState::STATE_IDLE, MissionState::STATE_COMPLETED,
                              MissionState::STATE_ABORTED, MissionState::STATE_ERROR}) {
    Rig r;
    r.mission(state);
    r.now += 0.1;
    const auto reply = set_usb(r);
    EXPECT_TRUE(reply["ok"].get<bool>()) << "state " << static_cast<int>(state);
    EXPECT_EQ(reply["data"]["revision"].get<uint64_t>(), 2U);
  }
  {  // RUNNING, but the last MissionState is older than the freshness limit (1 s)
    Rig r;
    r.mission(MissionState::STATE_RUNNING);
    r.now += 2.0;
    EXPECT_TRUE(set_usb(r)["ok"].get<bool>());
  }
  {  // the lock lifts as soon as the mission ends
    Rig r;
    r.mission(MissionState::STATE_RUNNING);
    r.now += 0.1;
    EXPECT_FALSE(set_usb(r)["ok"].get<bool>());
    r.mission(MissionState::STATE_COMPLETED);
    r.now += 0.1;
    EXPECT_TRUE(set_usb(r)["ok"].get<bool>());
  }
}
