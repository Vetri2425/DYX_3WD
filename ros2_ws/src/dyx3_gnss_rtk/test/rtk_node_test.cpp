// RtkNode in a private DDS domain with an injected clock: status mapping, fail-safe defaults, chunk
// publication.
#include "dyx3_gnss_rtk/rtk_node.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <thread>

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
  std::vector<rclcpp::SubscriptionBase::SharedPtr> keep;
  dyx3_interfaces::msg::RtkStatus rtk;
  dyx3_interfaces::msg::NtripStatus ntrip;
  std::vector<dyx3_interfaces::msg::RtcmData> chunks;

  Rig() {
    ctx = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions io;
    io.set_domain_id(150 + (getpid() % 80));
    ctx->init(0, nullptr, io);
    rclcpp::NodeOptions no;
    no.context(ctx);
    node = std::make_shared<RtkNode>(no, [this]() { return now; }, false, false);
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
          world->count_subscribers("/dyx3/gnss_report") > 0) {
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
  std::vector<uint8_t> frame(size_t n) {
    std::vector<uint8_t> f(n, 0x11);
    f[0] = 0xD3;
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
