// MissionNode integration: the ROS wiring (services, action, clients, QoS, freshness) and the v2
// lifecycle, exercised in-process against a helper node that plays the guard (SafetyGateStatus with
// the pre-arm verdict), px4_link (arm / set_offboard services that confirm, refuse or stay silent,
// and the armed / OFFBOARD state the full gate follows), RPP (acknowledges the execution artifact
// it can load) and the vehicle (EKF reference, reset counter, position). Waits are condition waits;
// the process runs in a private locked DDS domain (dds_test_support.hpp).
#include "dyx3_mission/mission_node.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "dds_test_support.hpp"
#include "dyx3_interfaces/action/execute_mission.hpp"
#include "dyx3_interfaces/msg/mission_state.hpp"
#include "dyx3_interfaces/msg/motion_setpoint_status.hpp"
#include "dyx3_interfaces/msg/point_result.hpp"
#include "dyx3_interfaces/msg/rpp_status.hpp"
#include "dyx3_interfaces/msg/safety_gate_status.hpp"
#include "dyx3_interfaces/msg/vehicle_state.hpp"
#include "dyx3_interfaces/srv/abort_mission.hpp"
#include "dyx3_interfaces/srv/arm_disarm.hpp"
#include "dyx3_interfaces/srv/pause_mission.hpp"
#include "dyx3_interfaces/srv/resume_mission.hpp"
#include "dyx3_interfaces/srv/set_offboard.hpp"
#include "dyx3_interfaces/srv/skip_point.hpp"
#include "dyx3_interfaces/srv/start_mission.hpp"
#include "dyx3_mission/frame_placement.hpp"
#include "dyx3_mission/sha256.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
namespace di = dyx3_interfaces;
namespace fs = std::filesystem;
using MS = di::msg::MissionState;
using GuardReason = di::msg::MotionSetpointStatus;

namespace {

constexpr double kRefLat = 45.0;
constexpr double kRefLon = 7.0;
constexpr double kAnchorLat = 45.0002;  // ~22.24 m north of the EKF origin
constexpr double kAnchorLon = 7.0;

std::string square_sha() {
  std::ifstream in(std::string(DYX3_FIXTURES) + "/manifest.txt");
  std::string name, sha;
  in >> name >> sha;
  return sha;
}

// Writes `meta` + the square fixture's points as a new source artifact in `dir`; returns its sha.
std::string write_source(const fs::path& dir, const std::string& meta) {
  const auto fx = dyx3_mission::load_artifact(DYX3_FIXTURES, square_sha());
  EXPECT_TRUE(fx.ok);
  const std::string bytes =
      dyx3_mission::serialize_artifact(fx.artifact.engine_id, meta, fx.artifact.points);
  EXPECT_FALSE(bytes.empty());
  const std::string sha = dyx3_mission::sha256_hex(bytes);
  std::ofstream(dir / (sha + ".dyx3path"), std::ios::binary) << bytes;
  return sha;
}

enum class Reply { kOk, kRefuse, kSilent };

struct Call {
  std::string op;  // arm, disarm, offboard_on, offboard_off
  std::int64_t t_ns;
};

class MissionNodeTest : public ::testing::Test {
protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("dyx3_mission_node_test_" + std::to_string(getpid()) + "_" + std::to_string(++seq_));
    fs::remove_all(dir_);
    fs::create_directories(dir_);
    const std::string anchor =
        "{\"anchor\":{\"alt\":null,\"lat\":" + dyx3_mission::python_repr(kAnchorLat) +
        ",\"lon\":" + dyx3_mission::python_repr(kAnchorLon) +
        "},\"densified_steps\":0,\"frame\":\"local_ned\","
        "\"max_boundary_snap_m\":0.0}";
    anchored_sha_ = write_source(dir_, anchor);
    ekf_sha_ = write_source(dir_,
                            "{\"anchor\":null,\"densified_steps\":0,\"frame\":"
                            "\"ekf_local_ned\",\"max_boundary_snap_m\":0.0}");
    fs::copy_file(std::string(DYX3_FIXTURES) + "/" + square_sha() + ".dyx3path",
                  dir_ / (square_sha() + ".dyx3path"));  // an old artifact: no "frame"

    make_node();
    helper_ = std::make_shared<rclcpp::Node>("helper");
    exec_.add_node(helper_);
    const auto be = rclcpp::QoS(1).best_effort();
    gate_pub_ = helper_->create_publisher<di::msg::SafetyGateStatus>("/dyx3/safety_gate", be);
    rpp_pub_ = helper_->create_publisher<di::msg::RppStatus>("/dyx3/rpp/status", be);
    veh_pub_ = helper_->create_publisher<di::msg::VehicleState>("/dyx3/vehicle_state", be);
    state_sub_ = helper_->create_subscription<MS>("/dyx3/mission/state",
                                                  rclcpp::QoS(100).reliable(), [this](const MS& m) {
                                                    last_state_ = m;
                                                    have_state_ = true;
                                                    states_.push_back(m);
                                                    ++state_count_;
                                                  });
    point_sub_ = helper_->create_subscription<di::msg::PointResult>(
        "/dyx3/mission/point_result", rclcpp::QoS(100).reliable(),
        [this](const di::msg::PointResult& m) { points_.push_back(m); });
    arm_srv_ = helper_->create_service<di::srv::ArmDisarm>(
        "/dyx3/px4_link/arm", [this](const std::shared_ptr<rmw_request_id_t> hdr,
                                     const std::shared_ptr<di::srv::ArmDisarm::Request> req) {
          calls_.push_back({req->arm ? "arm" : "disarm", now_ns()});
          di::srv::ArmDisarm::Response r;
          const Reply reply = req->arm ? arm_reply_ : disarm_reply_;
          if (reply != Reply::kOk) {
            if (reply == Reply::kSilent) return;
            r.accepted = false;
            r.reason_code = arm_refuse_reason_;
          } else {
            r.accepted = true;
            armed_ = req->arm;
            if (!req->arm) offboard_ = false;
          }
          arm_srv_->send_response(*hdr, r);
        });
    off_srv_ = helper_->create_service<di::srv::SetOffboard>(
        "/dyx3/px4_link/set_offboard",
        [this](const std::shared_ptr<rmw_request_id_t> hdr,
               const std::shared_ptr<di::srv::SetOffboard::Request> req) {
          calls_.push_back({req->enable ? "offboard_on" : "offboard_off", now_ns()});
          di::srv::SetOffboard::Response r;
          if (req->enable && offboard_reply_ != Reply::kOk) {
            if (offboard_reply_ == Reply::kSilent) return;
            r.accepted = false;
            r.reason_code = di::srv::SetOffboard::Response::REASON_NOT_ARMED_OR_REJECTED;
          } else {
            r.accepted = true;
            offboard_ = req->enable && armed_;
          }
          off_srv_->send_response(*hdr, r);
        });
    start_ = helper_->create_client<di::srv::StartMission>("/dyx3/mission/start");
    pause_ = helper_->create_client<di::srv::PauseMission>("/dyx3/mission/pause");
    resume_ = helper_->create_client<di::srv::ResumeMission>("/dyx3/mission/resume");
    abort_ = helper_->create_client<di::srv::AbortMission>("/dyx3/mission/abort");
    skip_ = helper_->create_client<di::srv::SkipPoint>("/dyx3/mission/skip_point");
    action_ = rclcpp_action::create_client<dyx3_mission::MissionNode::ExecuteMission>(
        helper_, "/dyx3/mission/execute");
    world_on_ = false;  // nothing is published until a test asks for it (pump / spin)
    ASSERT_TRUE(start_->wait_for_service(5s));
    // The fake px4_link services are discovered (same participant as the mission's clients).
    auto arm_probe = helper_->create_client<di::srv::ArmDisarm>("/dyx3/px4_link/arm");
    auto off_probe = helper_->create_client<di::srv::SetOffboard>("/dyx3/px4_link/set_offboard");
    ASSERT_TRUE(arm_probe->wait_for_service(5s));
    ASSERT_TRUE(off_probe->wait_for_service(5s));
    ASSERT_TRUE(action_->wait_for_action_server(5s));
    ASSERT_TRUE(spin_until([this] {
      return node_->count_subscribers("/dyx3/mission/state") > 0 &&
             helper_->count_subscribers("/dyx3/safety_gate") > 0 &&
             helper_->count_subscribers("/dyx3/vehicle_state") > 0 &&
             helper_->count_subscribers("/dyx3/rpp/status") > 0 &&
             node_->count_publishers("/dyx3/rpp/status") > 0;
    }));
    world_on_ = true;
  }
  void TearDown() override {
    exec_.remove_node(node_);
    exec_.remove_node(helper_);
    node_.reset();
    fs::remove_all(dir_);
  }

  void make_node(std::vector<rclcpp::Parameter> extra = {}) {
    if (node_) exec_.remove_node(node_);
    node_.reset();
    rclcpp::NodeOptions o;
    // The arm / OFFBOARD timeouts are the smallest valid values, so the timeout cases stay short.
    std::vector<rclcpp::Parameter> ps = {{"missions_dir", dir_.string()},
                                         {"rpp_status_max_age_s", 5.0},
                                         {"arm_timeout_s", 2.1},
                                         {"offboard_timeout_s", 3.6}};
    for (auto& p : extra) ps.push_back(p);
    o.parameter_overrides(ps);
    node_ = std::make_shared<dyx3_mission::MissionNode>(o);
    exec_.add_node(node_);
  }

  std::int64_t now_ns() { return helper_->get_clock()->now().nanoseconds(); }

  // ---- the simulated world, published every 20 ms while the rig spins
  void publish_world() {
    if (!world_on_) return;
    di::msg::SafetyGateStatus g;
    g.stamp = helper_->get_clock()->now();
    g.pre_arm_ok = pre_arm_ok_;
    g.pre_arm_reason_code = pre_arm_ok_ ? 0 : pre_arm_reason_;
    const bool engaged = armed_ && offboard_;
    g.ok = pre_arm_ok_ && engaged;
    g.reason_code = !pre_arm_ok_ ? pre_arm_reason_ : engaged ? 0 : GuardReason::REASON_ARMING_GATE;
    gate_pub_->publish(g);
    if (vehicle_on_) {
      di::msg::VehicleState v;
      v.position_valid = true;
      v.north_m = static_cast<float>(veh_n_);
      v.east_m = static_cast<float>(veh_e_);
      v.global_reference_valid = global_ref_;
      v.reference_latitude_deg = ref_lat_;
      v.reference_longitude_deg = ref_lon_;
      v.xy_reset_counter = xy_counter_;
      v.arming_state = armed_ ? 2 : 1;
      v.nav_state = offboard_ ? 14 : 4;
      veh_pub_->publish(v);
    }
    // RPP: acknowledges an execution artifact it can load, while the mission holds one.
    if (rpp_auto_ && have_state_ && !last_state_.path_artifact_sha256.empty()) {
      const auto s = last_state_.state;
      if (s == MS::STATE_READY || s == MS::STATE_RUNNING || s == MS::STATE_PAUSED) {
        if (dyx3_mission::load_artifact(dir_.string(), last_state_.path_artifact_sha256).ok) {
          rpp(s == MS::STATE_RUNNING ? di::msg::RppStatus::STATE_TRACKING
                                     : di::msg::RppStatus::STATE_LOADED,
              last_state_.mission_id);
        }
      }
    }
  }
  bool spin_until(const std::function<bool()>& pred, std::chrono::milliseconds limit = 6000ms) {
    const auto end = std::chrono::steady_clock::now() + limit;
    auto next_world = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < end) {
      if (std::chrono::steady_clock::now() >= next_world) {
        publish_world();
        next_world += 20ms;
      }
      exec_.spin_once(5ms);
      if (pred()) return true;
    }
    return pred();
  }
  void pump(std::chrono::milliseconds d) {
    spin_until([] { return false; }, d);
  }
  void rpp(std::uint8_t state, std::uint32_t mission_id, std::uint32_t run = 0) {
    di::msg::RppStatus r;
    r.state = state;
    r.mission_id = mission_id;
    r.run_index = run;
    rpp_pub_->publish(r);
  }
  template <typename Client, typename Req>
  typename Client::SharedResponse call(const std::shared_ptr<Client>& c, std::shared_ptr<Req> req) {
    auto fut = c->async_send_request(req);
    EXPECT_TRUE(spin_until([&] { return fut.wait_for(0s) == std::future_status::ready; }));
    return fut.get();
  }
  di::srv::StartMission::Response::SharedPtr start(const std::string& sha,
                                                   const std::string& request_id = "") {
    auto req = std::make_shared<di::srv::StartMission::Request>();
    req->path_artifact_sha256 = sha;
    req->request_id = request_id;
    return call(start_, req);
  }
  bool wait_state(std::uint8_t s, std::chrono::milliseconds limit = 6000ms) {
    return spin_until([&] { return have_state_ && last_state_.state == s; }, limit);
  }
  std::uint8_t state() { return last_state_.state; }
  std::vector<std::string> ops() const {
    std::vector<std::string> out;
    for (const auto& c : calls_) out.push_back(c.op);
    return out;
  }
  bool called(const std::string& op) const {
    const auto o = ops();
    return std::find(o.begin(), o.end(), op) != o.end();
  }
  // The distinct consecutive states published so far.
  std::vector<std::uint8_t> state_path() const {
    std::vector<std::uint8_t> out;
    for (const auto& m : states_) {
      if (out.empty() || out.back() != m.state) out.push_back(m.state);
    }
    return out;
  }
  // Brings a mission (ekf_local_ned unless given) to RUNNING through the whole lifecycle.
  void start_running(const std::string& sha = "") {
    pump(150ms);
    const auto res = start(sha.empty() ? ekf_sha_ : sha);
    ASSERT_TRUE(res->accepted);
    ASSERT_TRUE(wait_state(MS::STATE_RUNNING));
  }
  void finish_release() {
    ASSERT_TRUE(spin_until([&] { return last_state_.waiting_on == MS::WAIT_NONE; }));
  }

  static inline int seq_ = 0;
  fs::path dir_;
  std::string anchored_sha_, ekf_sha_;
  std::shared_ptr<dyx3_mission::MissionNode> node_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp::Publisher<di::msg::SafetyGateStatus>::SharedPtr gate_pub_;
  rclcpp::Publisher<di::msg::RppStatus>::SharedPtr rpp_pub_;
  rclcpp::Publisher<di::msg::VehicleState>::SharedPtr veh_pub_;
  rclcpp::Subscription<MS>::SharedPtr state_sub_;
  rclcpp::Subscription<di::msg::PointResult>::SharedPtr point_sub_;
  rclcpp::Service<di::srv::ArmDisarm>::SharedPtr arm_srv_;
  rclcpp::Service<di::srv::SetOffboard>::SharedPtr off_srv_;
  rclcpp::Client<di::srv::StartMission>::SharedPtr start_;
  rclcpp::Client<di::srv::PauseMission>::SharedPtr pause_;
  rclcpp::Client<di::srv::ResumeMission>::SharedPtr resume_;
  rclcpp::Client<di::srv::AbortMission>::SharedPtr abort_;
  rclcpp::Client<di::srv::SkipPoint>::SharedPtr skip_;
  rclcpp_action::Client<dyx3_mission::MissionNode::ExecuteMission>::SharedPtr action_;

  // world
  bool world_on_ = true;
  bool pre_arm_ok_ = true;
  std::uint8_t pre_arm_reason_ = 0;
  bool vehicle_on_ = true;
  bool global_ref_ = true;
  double ref_lat_ = kRefLat, ref_lon_ = kRefLon;
  std::uint8_t xy_counter_ = 0;
  double veh_n_ = 0.0, veh_e_ = 0.0;
  bool rpp_auto_ = true;
  bool armed_ = false, offboard_ = false;
  Reply arm_reply_ = Reply::kOk, disarm_reply_ = Reply::kOk, offboard_reply_ = Reply::kOk;
  std::uint8_t arm_refuse_reason_ = di::srv::ArmDisarm::Response::REASON_LINK_UNHEALTHY;
  std::vector<Call> calls_;

  MS last_state_;
  bool have_state_ = false;
  std::size_t state_count_ = 0;
  std::vector<MS> states_;
  std::vector<di::msg::PointResult> points_;
};

}  // namespace

// ------------------------------------------------------------------------------------------------
// lifecycle
// ------------------------------------------------------------------------------------------------
TEST_F(MissionNodeTest, HappyPathRunsEveryStepInOrderAndPlacesTheAnchoredTrajectory) {
  pump(150ms);
  const auto res = start(anchored_sha_, "req-1");
  ASSERT_TRUE(res->accepted);
  EXPECT_EQ(res->mission_id, 1U);
  EXPECT_FALSE(res->duplicate);
  ASSERT_TRUE(wait_state(MS::STATE_RUNNING));
  const std::vector<std::uint8_t> expected = {
      MS::STATE_IDLE,     MS::STATE_LOADING, MS::STATE_PLACING, MS::STATE_ARMING,
      MS::STATE_ENGAGING, MS::STATE_READY,   MS::STATE_RUNNING};
  EXPECT_EQ(state_path(), expected);
  // One message per transition, entered in order, each with its own entry time and wait step.
  std::int64_t last_entered = 0;
  for (const auto& m : states_) {
    const std::int64_t t =
        static_cast<std::int64_t>(m.state_entered.sec) * 1'000'000'000LL + m.state_entered.nanosec;
    if (m.state != MS::STATE_IDLE) EXPECT_GE(t, last_entered);
    if (m.state != MS::STATE_IDLE) last_entered = t;
    if (m.state == MS::STATE_ARMING) EXPECT_EQ(m.waiting_on, MS::WAIT_ARM);
    if (m.state == MS::STATE_ENGAGING) EXPECT_EQ(m.waiting_on, MS::WAIT_OFFBOARD);
    if (m.state == MS::STATE_READY) EXPECT_EQ(m.waiting_on, MS::WAIT_RPP_ACK);
    if (m.state != MS::STATE_IDLE) {
      EXPECT_EQ(m.request_id, "req-1");
      EXPECT_EQ(m.source_artifact_sha256, anchored_sha_);
      EXPECT_EQ(m.mission_id, 1U);
    }
  }
  EXPECT_EQ(ops(), (std::vector<std::string>{"arm", "offboard_on"}));
  EXPECT_LT(calls_[0].t_ns, calls_[1].t_ns);
  // The execution artifact is a different, content-addressed file RPP can load, placed with the
  // EKF reference: point (n, e) ground metres -> WGS84 lat/lon at the anchor -> PX4 projection.
  const std::string exec_sha = last_state_.path_artifact_sha256;
  ASSERT_FALSE(exec_sha.empty());
  EXPECT_NE(exec_sha, anchored_sha_);
  const auto exec = dyx3_mission::load_artifact(dir_.string(), exec_sha);
  ASSERT_TRUE(exec.ok) << exec.error;
  const auto src = dyx3_mission::load_artifact(dir_.string(), anchored_sha_);
  ASSERT_EQ(exec.artifact.points.size(), src.artifact.points.size());
  const auto a = dyx3_mission::project_to_ekf({kRefLat, kRefLon}, {kAnchorLat, kAnchorLon});
  EXPECT_NEAR(a.north_m, 22.238985, 1e-5);  // 0.0002 deg of the 6371 km sphere
  for (std::size_t i = 0; i < exec.artifact.points.size(); ++i) {
    const auto& sp = src.artifact.points[i];
    const auto want = dyx3_mission::place_point({kRefLat, kRefLon}, {kAnchorLat, kAnchorLon},
                                                {sp.north_m, sp.east_m});
    EXPECT_NEAR(exec.artifact.points[i].north_m, want.north_m, 1e-9);
    EXPECT_NEAR(exec.artifact.points[i].east_m, want.east_m, 1e-9);
    // Not a translation (unless the point is the anchor): 45 deg N, R/M = 1.000568.
    if (sp.north_m != 0.0) {
      EXPECT_GT(std::fabs(exec.artifact.points[i].north_m - (a.north_m + sp.north_m)),
                1e-4 * std::fabs(sp.north_m));
    }
    EXPECT_EQ(exec.artifact.points[i].flags, src.artifact.points[i].flags);
  }
}

TEST_F(MissionNodeTest, StartIsAnAdmissionThatReturnsAtOnce) {
  pump(150ms);
  const auto t0 = std::chrono::steady_clock::now();
  const auto o = node_->begin_mission(anchored_sha_, "");
  const auto dt = std::chrono::steady_clock::now() - t0;
  EXPECT_TRUE(o.accepted);
  EXPECT_EQ(o.mission_id, 1U);
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(), 50);
  // Nothing beyond admission happened inside the call: the artifact is read afterwards.
  EXPECT_EQ(node_->fsm().state(), dyx3_mission::State::kLoading);
  EXPECT_TRUE(calls_.empty());
  EXPECT_TRUE(wait_state(MS::STATE_RUNNING));
  // Over the service the reply also precedes the lifecycle.
  auto ab = std::make_shared<di::srv::AbortMission::Request>();
  ASSERT_TRUE(call(abort_, ab)->accepted);
  finish_release();
  const std::size_t before = states_.size();
  const auto res = start(ekf_sha_);
  ASSERT_TRUE(res->accepted);
  EXPECT_EQ(res->mission_id, 2U);
  bool running_before_reply = false;
  for (std::size_t i = before; i < states_.size(); ++i)
    running_before_reply |= states_[i].mission_id == 2U && states_[i].state == MS::STATE_RUNNING;
  EXPECT_FALSE(running_before_reply);
}

TEST_F(MissionNodeTest, PreArmGateNotOkRefusesTheStartAndNeverArms) {
  world_on_ = false;
  auto res = start(ekf_sha_);  // the guard has never reported: no data == not safe
  EXPECT_FALSE(res->accepted);
  EXPECT_EQ(res->reason_code, di::srv::StartMission::Response::REASON_SAFETY_GATE);
  EXPECT_EQ(res->gate_reason_code, GuardReason::REASON_STALE);
  world_on_ = true;
  pre_arm_ok_ = false;
  pre_arm_reason_ = GuardReason::REASON_RTK_GATE;
  pump(150ms);
  res = start(ekf_sha_);
  EXPECT_FALSE(res->accepted);
  EXPECT_EQ(res->reason_code, di::srv::StartMission::Response::REASON_SAFETY_GATE);
  EXPECT_EQ(res->gate_reason_code, GuardReason::REASON_RTK_GATE);
  EXPECT_EQ(node_->fsm().state(), dyx3_mission::State::kIdle);
  pump(300ms);
  EXPECT_TRUE(calls_.empty());
}

TEST_F(MissionNodeTest, PreArmGateLostBeforeMotionIsAnErrorThatDisarmsWhatItArmed) {
  arm_reply_ = Reply::kSilent;  // hold the lifecycle in ARMING
  pump(150ms);
  ASSERT_TRUE(start(ekf_sha_)->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_ARMING));
  ASSERT_TRUE(spin_until([&] { return called("arm"); }));
  pre_arm_ok_ = false;
  pre_arm_reason_ = GuardReason::REASON_HEADING_UNHEALTHY;
  ASSERT_TRUE(wait_state(MS::STATE_ERROR));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_SAFETY);
  EXPECT_EQ(last_state_.gate_reason_code, GuardReason::REASON_HEADING_UNHEALTHY);
  // The arm was in flight: it may have armed, so the release disarms; nothing engages.
  EXPECT_TRUE(spin_until([&] { return called("disarm"); }));
  EXPECT_FALSE(called("offboard_on"));
  EXPECT_TRUE(last_state_.path_artifact_sha256.empty());  // the execution is unloaded
}

TEST_F(MissionNodeTest, ArmRefusedIsAnErrorWithNoOffboardAndNoDisarm) {
  arm_reply_ = Reply::kRefuse;
  pump(150ms);
  ASSERT_TRUE(start(ekf_sha_)->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_ERROR));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_ARM_REFUSED);
  EXPECT_NE(last_state_.reason_detail.find("refused"), std::string::npos);
  pump(300ms);
  EXPECT_EQ(ops(), (std::vector<std::string>{"arm"}));
  EXPECT_TRUE(last_state_.path_artifact_sha256.empty());
}

TEST_F(MissionNodeTest, ArmTimeoutIsAnErrorWithNoOffboardAndADisarmOnDoubt) {
  arm_reply_ = Reply::kSilent;
  pump(150ms);
  ASSERT_TRUE(start(ekf_sha_)->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_ERROR, 5000ms));  // arm_timeout_s 2.1
  EXPECT_EQ(last_state_.reason_code, MS::REASON_ARM_TIMEOUT);
  const auto t_arm = calls_.at(0).t_ns;
  EXPECT_TRUE(spin_until([&] { return called("disarm"); }));
  EXPECT_FALSE(called("offboard_on"));
  // ERROR came no earlier than the configured timeout.
  for (const auto& m : states_) {
    if (m.state == MS::STATE_ERROR) {
      const std::int64_t t = static_cast<std::int64_t>(m.state_entered.sec) * 1'000'000'000LL +
                             m.state_entered.nanosec;
      EXPECT_GE(t - t_arm, 2'000'000'000LL);
      break;
    }
  }
}

TEST_F(MissionNodeTest, OffboardRefusedIsAnErrorThatReleasesAndDisarms) {
  offboard_reply_ = Reply::kRefuse;
  pump(150ms);
  ASSERT_TRUE(start(ekf_sha_)->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_ERROR));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_OFFBOARD_REFUSED);
  finish_release();
  EXPECT_EQ(ops(), (std::vector<std::string>{"arm", "offboard_on", "offboard_off", "disarm"}));
  EXPECT_FALSE(armed_);
}

TEST_F(MissionNodeTest, OffboardTimeoutIsAnErrorThatReleasesAndDisarms) {
  offboard_reply_ = Reply::kSilent;
  pump(150ms);
  ASSERT_TRUE(start(ekf_sha_)->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_ERROR, 7000ms));  // offboard_timeout_s 3.6
  EXPECT_EQ(last_state_.reason_code, MS::REASON_OFFBOARD_TIMEOUT);
  finish_release();
  EXPECT_EQ(ops(), (std::vector<std::string>{"arm", "offboard_on", "offboard_off", "disarm"}));
}

TEST_F(MissionNodeTest, RppAckTimeoutIsAnErrorThatReleasesDisarmsAndUnloads) {
  ASSERT_TRUE(
      node_->set_parameters_atomically({rclcpp::Parameter("rpp_ack_timeout_s", 0.4)}).successful);
  rpp_auto_ = false;  // RPP never acknowledges
  pump(150ms);
  ASSERT_TRUE(start(ekf_sha_)->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_READY));
  ASSERT_TRUE(wait_state(MS::STATE_ERROR));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_RPP_ACK_TIMEOUT);
  EXPECT_EQ(node_->fsm().log().back().event, dyx3_mission::Event::kRppAckTimeout);
  finish_release();
  EXPECT_EQ(ops(), (std::vector<std::string>{"arm", "offboard_on", "offboard_off", "disarm"}));
  EXPECT_TRUE(last_state_.path_artifact_sha256.empty());
}

TEST_F(MissionNodeTest, ReadyWaitsForTheFullGateEvenWithTheRppAck) {
  ASSERT_TRUE(
      node_->set_parameters_atomically({rclcpp::Parameter("rpp_ack_timeout_s", 0.8)}).successful);
  rpp_auto_ = false;
  pump(150ms);
  ASSERT_TRUE(start(ekf_sha_)->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_READY));
  // The vehicle drops out of OFFBOARD: RPP acknowledges, the full gate does not pass.
  offboard_ = false;
  rpp_auto_ = true;
  pump(300ms);
  EXPECT_EQ(state(), MS::STATE_READY);
  ASSERT_TRUE(wait_state(MS::STATE_ERROR));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_SAFETY);
  EXPECT_EQ(last_state_.gate_reason_code, GuardReason::REASON_ARMING_GATE);
  for (const auto& m : states_) EXPECT_NE(m.state, MS::STATE_RUNNING);
}

// ------------------------------------------------------------------------------------------------
// placement
// ------------------------------------------------------------------------------------------------
TEST_F(MissionNodeTest, EkfLocalNedIsDrivenUnchanged) {
  start_running(ekf_sha_);
  EXPECT_EQ(last_state_.path_artifact_sha256, ekf_sha_);
  EXPECT_EQ(last_state_.source_artifact_sha256, ekf_sha_);
}

TEST_F(MissionNodeTest, InvalidEkfReferenceIsAPlacementErrorWithNoArm) {
  global_ref_ = false;  // (the fake guard's pre-arm verdict is set independently)
  pump(150ms);
  ASSERT_TRUE(start(anchored_sha_)->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_ERROR));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_EKF_REFERENCE_INVALID);
  pump(200ms);
  EXPECT_TRUE(calls_.empty());
}

TEST_F(MissionNodeTest, NoFreshVehicleStateIsAPlacementError) {
  vehicle_on_ = false;
  pump(700ms);  // nothing ever received
  ASSERT_TRUE(start(ekf_sha_)->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_ERROR));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_EKF_REFERENCE_INVALID);
  EXPECT_TRUE(calls_.empty());
}

TEST_F(MissionNodeTest, AnArtifactWithoutAFrameIsRefusedAtPlacement) {
  pump(150ms);
  ASSERT_TRUE(start(square_sha())->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_ERROR));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_NO_PLACEMENT_FRAME);
  pump(200ms);
  EXPECT_TRUE(calls_.empty());
}

TEST_F(MissionNodeTest, AnchorOutOfBoundsIsAPlacementError) {
  ref_lat_ = kRefLat - 0.01;  // the EKF origin ~1.1 km south of the anchor
  pump(150ms);
  ASSERT_TRUE(start(anchored_sha_)->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_ERROR));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_PLACEMENT_OUT_OF_BOUNDS);
  EXPECT_TRUE(calls_.empty());
}

TEST_F(MissionNodeTest, MissingOrMalformedArtifact) {
  pump(150ms);
  auto res = start(std::string(64, 'a'));  // well-formed id, no such file: accepted, then ERROR
  EXPECT_TRUE(res->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_ERROR));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_PATH_ERROR);
  // A malformed id is refused at admission: no execution is created.
  res = start("not-a-hash");
  EXPECT_FALSE(res->accepted);
  EXPECT_EQ(res->reason_code, di::srv::StartMission::Response::REASON_INVALID_ARTIFACT);
  EXPECT_EQ(node_->fsm().mission_id(), 1U);
  EXPECT_TRUE(calls_.empty());
}

TEST_F(MissionNodeTest, EkfResetWhileRunningPausesAndNeverAutoResumes) {
  start_running(anchored_sha_);
  ++xy_counter_;
  ASSERT_TRUE(wait_state(MS::STATE_PAUSED));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_EKF_RESET);
  pump(400ms);
  EXPECT_EQ(state(), MS::STATE_PAUSED);
  EXPECT_FALSE(called("disarm"));  // paused, still armed and in OFFBOARD
  // Same EKF reference: the operator may resume (the counter is re-baselined).
  auto rr = call(resume_, std::make_shared<di::srv::ResumeMission::Request>());
  EXPECT_TRUE(rr->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_RUNNING));
  pump(300ms);
  EXPECT_EQ(state(), MS::STATE_RUNNING);
  // The reference itself moves: paused, and resume is refused (the placed path is wrong now).
  ref_lat_ += 0.00001;
  ASSERT_TRUE(wait_state(MS::STATE_PAUSED));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_EKF_RESET);
  rr = call(resume_, std::make_shared<di::srv::ResumeMission::Request>());
  EXPECT_FALSE(rr->accepted);
  EXPECT_EQ(rr->reason_code, di::srv::ResumeMission::Response::REASON_EKF_REFERENCE_CHANGED);
}

// ------------------------------------------------------------------------------------------------
// terminal handling, pause, E-stop, idempotency
// ------------------------------------------------------------------------------------------------
TEST_F(MissionNodeTest, CompleteReleasesOffboardThenDisarms) {
  start_running();
  rpp_auto_ = false;
  rpp(di::msg::RppStatus::STATE_COMPLETE, 1);
  ASSERT_TRUE(wait_state(MS::STATE_COMPLETED));
  finish_release();
  EXPECT_EQ(ops(), (std::vector<std::string>{"arm", "offboard_on", "offboard_off", "disarm"}));
  bool saw_release_wait = false;
  for (const auto& m : states_)
    saw_release_wait |=
        m.state == MS::STATE_COMPLETED &&
        (m.waiting_on == MS::WAIT_OFFBOARD_RELEASE || m.waiting_on == MS::WAIT_DISARM);
  EXPECT_TRUE(saw_release_wait);
  EXPECT_FALSE(armed_);
}

TEST_F(MissionNodeTest, AbortReleasesAndDisarms) {
  start_running();
  auto ab = std::make_shared<di::srv::AbortMission::Request>();
  ab->reason_code = di::srv::AbortMission::Request::REASON_OPERATOR;
  ASSERT_TRUE(call(abort_, ab)->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_ABORTED));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_OPERATOR);
  finish_release();
  EXPECT_EQ(ops(), (std::vector<std::string>{"arm", "offboard_on", "offboard_off", "disarm"}));
  EXPECT_FALSE(call(abort_, ab)->accepted);  // nothing active any more
}

TEST_F(MissionNodeTest, EmergencyStopAbortsAndDisarms) {
  start_running();
  pre_arm_ok_ = false;
  pre_arm_reason_ = GuardReason::REASON_ESTOP;
  ASSERT_TRUE(wait_state(MS::STATE_ABORTED));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_ESTOP);
  EXPECT_TRUE(spin_until([&] { return called("disarm"); }));
  EXPECT_FALSE(armed_);
}

TEST_F(MissionNodeTest, EmergencyStopWhileArmingAbortsAndDisarms) {
  arm_reply_ = Reply::kSilent;  // PX4 still confirming the arm
  pump(150ms);
  ASSERT_TRUE(start(ekf_sha_)->accepted);
  ASSERT_TRUE(spin_until([&] { return called("arm"); }));
  pre_arm_ok_ = false;
  pre_arm_reason_ = GuardReason::REASON_ESTOP;
  ASSERT_TRUE(wait_state(MS::STATE_ABORTED));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_ESTOP);
  EXPECT_TRUE(spin_until([&] { return called("disarm"); }));  // sent at once, not after the arm
  EXPECT_FALSE(called("offboard_on"));
}

TEST_F(MissionNodeTest, PauseKeepsTheVehicleArmedAndResumeNeedsOffboard) {
  start_running();
  ASSERT_TRUE(call(pause_, std::make_shared<di::srv::PauseMission::Request>())->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_PAUSED));
  EXPECT_EQ(last_state_.waiting_on, MS::WAIT_OPERATOR);
  pump(400ms);
  EXPECT_EQ(ops(), (std::vector<std::string>{"arm", "offboard_on"}));  // no release on pause
  EXPECT_TRUE(armed_);
  EXPECT_TRUE(call(resume_, std::make_shared<di::srv::ResumeMission::Request>())->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_RUNNING));
  // OFFBOARD lost: the guard pauses the mission; resume is refused and nothing re-engages.
  offboard_ = false;
  ASSERT_TRUE(wait_state(MS::STATE_PAUSED));
  EXPECT_EQ(last_state_.gate_reason_code, GuardReason::REASON_ARMING_GATE);
  const auto rr = call(resume_, std::make_shared<di::srv::ResumeMission::Request>());
  EXPECT_FALSE(rr->accepted);
  EXPECT_EQ(rr->reason_code, di::srv::ResumeMission::Response::REASON_NOT_ARMED_OR_OFFBOARD);
  pump(300ms);
  EXPECT_EQ(state(), MS::STATE_PAUSED);
  EXPECT_EQ(ops(), (std::vector<std::string>{"arm", "offboard_on"}));
}

TEST_F(MissionNodeTest, DuplicateRequestIdReturnsTheSameExecution) {
  pump(150ms);
  const auto a = start(ekf_sha_, "tablet-42");
  ASSERT_TRUE(a->accepted);
  const auto b = start(ekf_sha_, "tablet-42");
  EXPECT_TRUE(b->accepted);
  EXPECT_TRUE(b->duplicate);
  EXPECT_EQ(b->mission_id, a->mission_id);
  ASSERT_TRUE(wait_state(MS::STATE_RUNNING));
  const auto c = start(ekf_sha_, "tablet-42");  // still the same execution while it runs
  EXPECT_TRUE(c->duplicate);
  EXPECT_EQ(c->mission_id, a->mission_id);
  EXPECT_EQ(node_->fsm().mission_id(), 1U);
  EXPECT_EQ(ops(), (std::vector<std::string>{"arm", "offboard_on"}));  // armed once
  const auto d = start(ekf_sha_, "tablet-43");  // another request while active
  EXPECT_FALSE(d->accepted);
  EXPECT_EQ(d->reason_code, di::srv::StartMission::Response::REASON_BUSY);
  EXPECT_FALSE(start(ekf_sha_, "bad id!")->accepted);
  EXPECT_EQ(start(ekf_sha_, std::string(65, 'x'))->reason_code,
            di::srv::StartMission::Response::REASON_INVALID_REQUEST);
}

TEST_F(MissionNodeTest, ANewStartWaitsForThePreviousRelease) {
  start_running();
  disarm_reply_ = Reply::kSilent;  // the disarm will hang until its timeout
  auto ab = std::make_shared<di::srv::AbortMission::Request>();
  ASSERT_TRUE(call(abort_, ab)->accepted);
  ASSERT_TRUE(spin_until([&] { return called("disarm"); }));
  EXPECT_EQ(last_state_.waiting_on, MS::WAIT_DISARM);
  EXPECT_EQ(start(ekf_sha_)->reason_code, di::srv::StartMission::Response::REASON_BUSY);
  ASSERT_TRUE(spin_until([&] { return last_state_.waiting_on == MS::WAIT_NONE; }, 5000ms));
  EXPECT_NE(last_state_.reason_detail.find("disarm timeout"), std::string::npos);
}

// ------------------------------------------------------------------------------------------------
// RPP hand-off, journal, action (unchanged behaviour on the new lifecycle)
// ------------------------------------------------------------------------------------------------
TEST_F(MissionNodeTest, AnRppStatusOfAnotherMissionDoesNotAcknowledge) {
  rpp_auto_ = false;
  pump(150ms);
  ASSERT_TRUE(start(ekf_sha_)->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_READY));
  for (int i = 0; i < 4; ++i) {
    rpp(di::msg::RppStatus::STATE_LOADED, 99);
    pump(30ms);
  }
  EXPECT_EQ(state(), MS::STATE_READY);
  for (int i = 0; i < 4; ++i) {
    rpp(di::msg::RppStatus::STATE_LOADED, 1);
    pump(30ms);
  }
  EXPECT_TRUE(wait_state(MS::STATE_RUNNING));
}

TEST_F(MissionNodeTest, RppErrorInReadyIsAnErrorAndCompleteIsNotAnAcknowledgement) {
  rpp_auto_ = false;
  pump(150ms);
  ASSERT_TRUE(start(ekf_sha_)->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_READY));
  for (int i = 0; i < 4; ++i) {
    rpp(di::msg::RppStatus::STATE_COMPLETE, 1);
    pump(30ms);
  }
  EXPECT_EQ(state(), MS::STATE_READY);
  rpp(di::msg::RppStatus::STATE_ERROR, 1);
  ASSERT_TRUE(wait_state(MS::STATE_ERROR));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_RPP_ERROR);
  finish_release();
  EXPECT_FALSE(armed_);
}

TEST_F(MissionNodeTest, GateLossWhileRunningPausesWithTheGuardReasonAndNeverAutoResumes) {
  start_running();
  pre_arm_ok_ = false;
  pre_arm_reason_ = GuardReason::REASON_RTK_GATE;
  ASSERT_TRUE(wait_state(MS::STATE_PAUSED));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_RTK);
  EXPECT_EQ(last_state_.gate_reason_code, GuardReason::REASON_RTK_GATE);
  auto rr = call(resume_, std::make_shared<di::srv::ResumeMission::Request>());
  EXPECT_FALSE(rr->accepted);
  EXPECT_EQ(rr->reason_code, di::srv::ResumeMission::Response::REASON_SAFETY_GATE);
  pre_arm_ok_ = true;
  pump(400ms);
  EXPECT_EQ(state(), MS::STATE_PAUSED);  // the gate recovering does not resume
  EXPECT_TRUE(call(resume_, std::make_shared<di::srv::ResumeMission::Request>())->accepted);
  EXPECT_TRUE(wait_state(MS::STATE_RUNNING));
}

TEST_F(MissionNodeTest, AStaleGateStatusPausesARunningMission) {
  start_running();
  world_on_ = false;  // the guard goes silent: the verdict ages out (0.5 s)
  ASSERT_TRUE(wait_state(MS::STATE_PAUSED, 3000ms));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_SAFETY);
  EXPECT_EQ(last_state_.gate_reason_code, GuardReason::REASON_STALE);
}

TEST_F(MissionNodeTest, SilentRppPausesARunningMissionAndNeverAutoResumes) {
  ASSERT_TRUE(node_->set_parameters_atomically({rclcpp::Parameter("rpp_status_max_age_s", 0.4)})
                  .successful);
  start_running();
  pump(800ms);
  EXPECT_EQ(state(), MS::STATE_RUNNING);
  rpp_auto_ = false;
  ASSERT_TRUE(wait_state(MS::STATE_PAUSED, 2000ms));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_RPP_STALE);
  auto rr = call(resume_, std::make_shared<di::srv::ResumeMission::Request>());
  EXPECT_FALSE(rr->accepted);  // RPP still silent
  EXPECT_EQ(rr->reason_code, di::srv::ResumeMission::Response::REASON_SAFETY_GATE);
  rpp_auto_ = true;
  pump(400ms);
  EXPECT_EQ(state(), MS::STATE_PAUSED);
  EXPECT_TRUE(call(resume_, std::make_shared<di::srv::ResumeMission::Request>())->accepted);
  EXPECT_TRUE(wait_state(MS::STATE_RUNNING));
}

TEST_F(MissionNodeTest, PointResultsAreJournalledFromVehicleState) {
  start_running();  // ekf_local_ned: the execution points are the fixture's points
  const auto art = dyx3_mission::load_artifact(dir_.string(), ekf_sha_);
  ASSERT_TRUE(art.ok);
  for (const auto& p : art.artifact.points) {
    veh_n_ = p.north_m;
    veh_e_ = p.east_m + 0.02;
    pump(25ms);
  }
  rpp_auto_ = false;
  rpp(di::msg::RppStatus::STATE_COMPLETE, 1, 0);
  EXPECT_TRUE(wait_state(MS::STATE_COMPLETED));
  EXPECT_TRUE(spin_until([&] { return points_.size() >= 5; }));
  ASSERT_GE(points_.size(), 5U);
  for (std::size_t k = 0; k < 5; ++k) {
    EXPECT_EQ(points_[k].point_index, k);
    EXPECT_EQ(points_[k].result_code, di::msg::PointResult::RESULT_COMPLETED);
    EXPECT_LE(points_[k].error_m, 0.05F);
  }
}

TEST_F(MissionNodeTest, SkipPointIsAcceptedOnlyWithAnActivePointWhileRunning) {
  EXPECT_FALSE(call(skip_, std::make_shared<di::srv::SkipPoint::Request>())->accepted);
  start_running();
  auto res = call(skip_, std::make_shared<di::srv::SkipPoint::Request>());
  EXPECT_TRUE(res->accepted);
  EXPECT_EQ(res->skipped_point_index, 0U);
  EXPECT_TRUE(spin_until([&] { return !points_.empty(); }));
  EXPECT_EQ(points_.front().result_code, di::msg::PointResult::RESULT_SKIPPED);
  auto ab = std::make_shared<di::srv::AbortMission::Request>();
  EXPECT_TRUE(call(abort_, ab)->accepted);
  EXPECT_FALSE(call(skip_, std::make_shared<di::srv::SkipPoint::Request>())->accepted);
}

TEST_F(MissionNodeTest, ExecuteMissionActionRunsThroughTheSameStartPath) {
  using Exec = dyx3_mission::MissionNode::ExecuteMission;
  Exec::Goal goal;
  goal.path_artifact_sha256 = ekf_sha_;
  world_on_ = false;
  auto gf = action_->async_send_goal(goal);
  ASSERT_TRUE(spin_until([&] { return gf.wait_for(0s) == std::future_status::ready; }));
  EXPECT_EQ(gf.get(), nullptr);  // no gate verdict: rejected

  world_on_ = true;
  pump(150ms);
  bool result_seen = false;
  std::uint8_t result_code = 255;
  auto opts = rclcpp_action::Client<Exec>::SendGoalOptions();
  opts.result_callback = [&](const rclcpp_action::ClientGoalHandle<Exec>::WrappedResult& r) {
    result_seen = true;
    result_code = r.result->result_code;
  };
  gf = action_->async_send_goal(goal, opts);
  ASSERT_TRUE(spin_until([&] { return gf.wait_for(0s) == std::future_status::ready; }));
  ASSERT_TRUE(gf.get() != nullptr) << "goal rejected";
  ASSERT_TRUE(wait_state(MS::STATE_RUNNING));
  rpp_auto_ = false;
  rpp(di::msg::RppStatus::STATE_COMPLETE, 1, 0);
  EXPECT_TRUE(spin_until([&] { return result_seen; }));
  EXPECT_EQ(result_code, Exec::Result::RESULT_COMPLETED);
}

TEST_F(MissionNodeTest, ActionCancelAbortsAsCanceled) {
  using Exec = dyx3_mission::MissionNode::ExecuteMission;
  pump(150ms);
  Exec::Goal goal;
  goal.path_artifact_sha256 = ekf_sha_;
  auto gf = action_->async_send_goal(goal);
  ASSERT_TRUE(spin_until([&] { return gf.wait_for(0s) == std::future_status::ready; }));
  auto gh = gf.get();
  ASSERT_TRUE(gh != nullptr);
  auto rf = action_->async_get_result(gh);
  auto cf = action_->async_cancel_goal(gh);
  ASSERT_TRUE(spin_until([&] { return cf.wait_for(0s) == std::future_status::ready; }));
  EXPECT_TRUE(wait_state(MS::STATE_ABORTED));
  EXPECT_EQ(last_state_.reason_code, MS::REASON_OPERATOR);
  // H3: the goal is finalised as CANCELED (not left hanging, not reported as an abort)
  ASSERT_TRUE(spin_until([&] { return rf.wait_for(0s) == std::future_status::ready; }));
  const auto wr = rf.get();
  EXPECT_EQ(wr.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_TRUE(wr.result != nullptr);
  EXPECT_EQ(wr.result->result_code, Exec::Result::RESULT_ABORTED);
}

// ------------------------------------------------------------------------------------------------
// parameters, idle behaviour
// ------------------------------------------------------------------------------------------------
TEST_F(MissionNodeTest, IdleExecutorWaitsBetweenTenHertzStateCallbacks) {
  world_on_ = false;
  const std::size_t before = state_count_;
  const std::clock_t cpu_before = std::clock();
  std::thread spin_thread([this] { exec_.spin(); });
  std::this_thread::sleep_for(1500ms);
  exec_.cancel();
  spin_thread.join();
  const double cpu_s = static_cast<double>(std::clock() - cpu_before) / CLOCKS_PER_SEC;
  EXPECT_GE(state_count_ - before, 5U);
  EXPECT_LE(state_count_ - before, 30U);
  EXPECT_LT(cpu_s, 0.5) << "an idle mission process consumed too much CPU";
}

TEST_F(MissionNodeTest, IdleParameterBatchAppliesAndChangesPublicationPeriod) {
  world_on_ = false;
  pump(350ms);
  const auto before = state_count_;
  pump(350ms);
  const auto slow_count = state_count_ - before;
  const auto result = node_->set_parameters_atomically(
      {rclcpp::Parameter("state_publish_hz", 50.0),
       rclcpp::Parameter("gate_status_max_age_s", 0.05),
       rclcpp::Parameter("point_capture_radius_m", 0.20),
       rclcpp::Parameter("rpp_ack_timeout_s", 0.25), rclcpp::Parameter("arm_timeout_s", 3.0),
       rclcpp::Parameter("offboard_timeout_s", 4.0),
       rclcpp::Parameter("placement_max_distance_m", 500.0),
       rclcpp::Parameter("vehicle_state_max_age_s", 0.3)});
  ASSERT_TRUE(result.successful) << result.reason;
  EXPECT_DOUBLE_EQ(node_->get_parameter("state_publish_hz").as_double(), 50.0);
  EXPECT_DOUBLE_EQ(node_->get_parameter("arm_timeout_s").as_double(), 3.0);
  EXPECT_DOUBLE_EQ(node_->get_parameter("placement_max_distance_m").as_double(), 500.0);
  const auto fast_before = state_count_;
  pump(350ms);
  EXPECT_GT(state_count_ - fast_before, slow_count * 2);
  // The effective freshness cache changes too: a formerly fresh gate cannot start a mission.
  world_on_ = true;
  publish_world();
  world_on_ = false;
  pump(120ms);
  EXPECT_FALSE(start(ekf_sha_)->accepted);
  EXPECT_EQ(node_->fsm().state(), dyx3_mission::State::kIdle);
}

TEST_F(MissionNodeTest, InvalidParametersAreRefusedAtomicallyAndAtConstruction) {
  const auto result =
      node_->set_parameters_atomically({rclcpp::Parameter("state_publish_hz", 50.0),
                                        rclcpp::Parameter("gate_status_max_age_s", -1.0)});
  EXPECT_FALSE(result.successful);
  EXPECT_DOUBLE_EQ(node_->get_parameter("state_publish_hz").as_double(), 10.0);
  EXPECT_DOUBLE_EQ(node_->get_parameter("gate_status_max_age_s").as_double(), 0.5);
  EXPECT_FALSE(node_
                   ->set_parameters_atomically(
                       {rclcpp::Parameter("point_capture_radius_m", std::string("bad"))})
                   .successful);
  // Timeouts must exceed px4_link's own confirmation windows; placement is bounded to 1 km.
  for (const auto& bad :
       {rclcpp::Parameter("arm_timeout_s", 2.0), rclcpp::Parameter("offboard_timeout_s", 3.5),
        rclcpp::Parameter("placement_max_distance_m", 1000.5),
        rclcpp::Parameter("rpp_ack_timeout_s", 0.0),
        rclcpp::Parameter("rpp_status_max_age_s", std::numeric_limits<double>::quiet_NaN()),
        rclcpp::Parameter("vehicle_state_max_age_s", -1.0)}) {
    EXPECT_FALSE(node_->set_parameters_atomically({bad}).successful) << bad.get_name();
    rclcpp::NodeOptions bo;
    bo.parameter_overrides({bad});
    EXPECT_THROW(std::make_shared<dyx3_mission::MissionNode>(bo), std::invalid_argument)
        << bad.get_name();
  }
  EXPECT_FALSE(
      node_->set_parameters_atomically({rclcpp::Parameter("missions_dir", "/tmp")}).successful);
}

TEST_F(MissionNodeTest, DefaultsAreTheContractValues) {
  rclcpp::NodeOptions o;
  auto n = std::make_shared<dyx3_mission::MissionNode>(o);
  EXPECT_DOUBLE_EQ(n->get_parameter("rpp_ack_timeout_s").as_double(), 30.0);
  EXPECT_DOUBLE_EQ(n->get_parameter("rpp_status_max_age_s").as_double(), 0.5);
  EXPECT_DOUBLE_EQ(n->get_parameter("arm_timeout_s").as_double(), 4.0);
  EXPECT_DOUBLE_EQ(n->get_parameter("offboard_timeout_s").as_double(), 5.0);
  EXPECT_DOUBLE_EQ(n->get_parameter("placement_max_distance_m").as_double(), 1000.0);
  EXPECT_DOUBLE_EQ(n->get_parameter("vehicle_state_max_age_s").as_double(), 0.5);
}

TEST_F(MissionNodeTest, ActiveAndTerminalStatesRejectIdleOnlyUpdates) {
  start_running();
  auto result = node_->set_parameters_atomically({rclcpp::Parameter("state_publish_hz", 50.0)});
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("IDLE_ONLY"), std::string::npos);
  auto ab = std::make_shared<di::srv::AbortMission::Request>();
  ASSERT_TRUE(call(abort_, ab)->accepted);
  ASSERT_TRUE(wait_state(MS::STATE_ABORTED));
  result = node_->set_parameters_atomically({rclcpp::Parameter("arm_timeout_s", 3.0)});
  EXPECT_FALSE(result.successful);
  EXPECT_DOUBLE_EQ(node_->get_parameter("arm_timeout_s").as_double(), 2.1);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  // A private DDS domain (an exclusive per-process lock, dds_test_support.hpp): the suite cannot
  // talk to other test processes or a live graph.
  dyx3_test::use_synchronous_delivery();
  rclcpp::InitOptions io;
  io.set_domain_id(static_cast<size_t>(dyx3_test::isolated_domain_id()));
  rclcpp::init(argc, argv, io);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
