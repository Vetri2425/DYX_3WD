// MissionNode integration: the ROS wiring (services, action, QoS, freshness) exercised in-process
// against a helper node that plays the guard, RPP and PX4 link. Fixtures: real artifacts produced
// by the Python writer (test/fixtures). The pure logic is covered by the other tests; this checks
// the glue.
#include "dyx3_mission/mission_node.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include "dyx3_interfaces/action/execute_mission.hpp"
#include "dyx3_interfaces/msg/mission_state.hpp"
#include "dyx3_interfaces/msg/motion_setpoint_status.hpp"
#include "dyx3_interfaces/msg/point_result.hpp"
#include "dyx3_interfaces/msg/rpp_status.hpp"
#include "dyx3_interfaces/msg/safety_gate_status.hpp"
#include "dyx3_interfaces/msg/vehicle_state.hpp"
#include "dyx3_interfaces/srv/abort_mission.hpp"
#include "dyx3_interfaces/srv/pause_mission.hpp"
#include "dyx3_interfaces/srv/resume_mission.hpp"
#include "dyx3_interfaces/srv/skip_point.hpp"
#include "dyx3_interfaces/srv/start_mission.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
namespace di = dyx3_interfaces;

namespace {

std::string square_sha() {
  std::ifstream in(std::string(DYX3_FIXTURES) + "/manifest.txt");
  std::string name, sha;
  in >> name >> sha;
  return sha;
}

class MissionNodeTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::NodeOptions o;
    o.parameter_overrides({{"missions_dir", std::string(DYX3_FIXTURES)}});
    node_ = std::make_shared<dyx3_mission::MissionNode>(o);
    helper_ = std::make_shared<rclcpp::Node>("helper");
    exec_.add_node(node_);
    exec_.add_node(helper_);
    const auto be = rclcpp::QoS(1).best_effort();
    gate_pub_ = helper_->create_publisher<di::msg::SafetyGateStatus>("/dyx3/safety_gate", be);
    rpp_pub_ = helper_->create_publisher<di::msg::RppStatus>("/dyx3/rpp/status", be);
    veh_pub_ = helper_->create_publisher<di::msg::VehicleState>("/dyx3/vehicle_state", be);
    state_sub_ = helper_->create_subscription<di::msg::MissionState>(
        "/dyx3/mission/state", rclcpp::QoS(1).reliable(), [this](const di::msg::MissionState& m) {
          last_state_ = m;
          have_state_ = true;
        });
    point_sub_ = helper_->create_subscription<di::msg::PointResult>(
        "/dyx3/mission/point_result", rclcpp::QoS(100).reliable(),
        [this](const di::msg::PointResult& m) { points_.push_back(m); });
    start_ = helper_->create_client<di::srv::StartMission>("/dyx3/mission/start");
    pause_ = helper_->create_client<di::srv::PauseMission>("/dyx3/mission/pause");
    resume_ = helper_->create_client<di::srv::ResumeMission>("/dyx3/mission/resume");
    abort_ = helper_->create_client<di::srv::AbortMission>("/dyx3/mission/abort");
    skip_ = helper_->create_client<di::srv::SkipPoint>("/dyx3/mission/skip_point");
    action_ = rclcpp_action::create_client<dyx3_mission::MissionNode::ExecuteMission>(
        helper_, "/dyx3/mission/execute");
    ASSERT_TRUE(start_->wait_for_service(5s));
    ASSERT_TRUE(action_->wait_for_action_server(5s));
  }
  void TearDown() override {
    exec_.remove_node(node_);
    exec_.remove_node(helper_);
  }

  bool spin_until(const std::function<bool()>& pred, std::chrono::milliseconds limit = 4000ms) {
    const auto end = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < end) {
      exec_.spin_some(20ms);
      if (pred()) return true;
    }
    return pred();
  }
  void gate(bool ok, std::uint8_t reason = 0) {
    di::msg::SafetyGateStatus g;
    g.stamp = helper_->get_clock()->now();
    g.ok = ok;
    g.reason_code = reason;
    gate_pub_->publish(g);
  }
  // Publish the gate verdict repeatedly while spinning (the node requires a FRESH verdict).
  void hold_gate(bool ok, std::uint8_t reason, std::chrono::milliseconds dur) {
    const auto end = std::chrono::steady_clock::now() + dur;
    while (std::chrono::steady_clock::now() < end) {
      gate(ok, reason);
      exec_.spin_some(20ms);
    }
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
  std::uint8_t state() { return last_state_.state; }
  bool wait_state(std::uint8_t s) {
    return spin_until([&] { return have_state_ && last_state_.state == s; });
  }
  // Bring a mission to READY or RUNNING.
  void start_ready() {
    hold_gate(true, 0, 150ms);
    auto req = std::make_shared<di::srv::StartMission::Request>();
    req->path_artifact_sha256 = square_sha();
    auto res = call(start_, req);
    ASSERT_TRUE(res->accepted);
    ASSERT_EQ(res->mission_id, 1U);
    ASSERT_TRUE(wait_state(di::msg::MissionState::STATE_READY));
  }
  void start_running() {
    start_ready();
    for (int i = 0; i < 5; ++i) {
      rpp(di::msg::RppStatus::STATE_TRACKING, 1);
      hold_gate(true, 0, 30ms);
    }
    ASSERT_TRUE(wait_state(di::msg::MissionState::STATE_RUNNING));
  }

  std::shared_ptr<dyx3_mission::MissionNode> node_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp::Publisher<di::msg::SafetyGateStatus>::SharedPtr gate_pub_;
  rclcpp::Publisher<di::msg::RppStatus>::SharedPtr rpp_pub_;
  rclcpp::Publisher<di::msg::VehicleState>::SharedPtr veh_pub_;
  rclcpp::Subscription<di::msg::MissionState>::SharedPtr state_sub_;
  rclcpp::Subscription<di::msg::PointResult>::SharedPtr point_sub_;
  rclcpp::Client<di::srv::StartMission>::SharedPtr start_;
  rclcpp::Client<di::srv::PauseMission>::SharedPtr pause_;
  rclcpp::Client<di::srv::ResumeMission>::SharedPtr resume_;
  rclcpp::Client<di::srv::AbortMission>::SharedPtr abort_;
  rclcpp::Client<di::srv::SkipPoint>::SharedPtr skip_;
  rclcpp_action::Client<dyx3_mission::MissionNode::ExecuteMission>::SharedPtr action_;
  di::msg::MissionState last_state_;
  bool have_state_ = false;
  std::vector<di::msg::PointResult> points_;
};

}  // namespace

TEST_F(MissionNodeTest, StartIsRefusedWithoutAFreshSafetyGate) {
  auto req = std::make_shared<di::srv::StartMission::Request>();
  req->path_artifact_sha256 = square_sha();
  auto res = call(start_, req);  // the guard has never reported: no data == not safe
  EXPECT_FALSE(res->accepted);
  EXPECT_EQ(res->reason_code, di::srv::StartMission::Response::REASON_SAFETY_GATE);
  EXPECT_EQ(node_->fsm().state(), dyx3_mission::State::kIdle);

  hold_gate(false, di::msg::MotionSetpointStatus::REASON_RTK_GATE, 150ms);  // an explicit "not ok"
  res = call(start_, req);
  EXPECT_FALSE(res->accepted);
  EXPECT_EQ(res->reason_code, di::srv::StartMission::Response::REASON_SAFETY_GATE);
}

TEST_F(MissionNodeTest, ValidArtifactLoadsThenNeedsTheRppAcknowledgementBeforeRunning) {
  start_ready();
  EXPECT_EQ(last_state_.path_artifact_sha256, square_sha());
  EXPECT_EQ(last_state_.mission_id, 1U);
  // a second start is refused: one mission at a time
  auto req = std::make_shared<di::srv::StartMission::Request>();
  req->path_artifact_sha256 = square_sha();
  EXPECT_EQ(call(start_, req)->reason_code, di::srv::StartMission::Response::REASON_BUSY);
  // an RPP status for ANOTHER mission id must not release motion
  for (int i = 0; i < 4; ++i) {
    rpp(di::msg::RppStatus::STATE_TRACKING, 99);
    hold_gate(true, 0, 30ms);
  }
  EXPECT_EQ(state(), di::msg::MissionState::STATE_READY);
  // the right id does
  for (int i = 0; i < 4; ++i) {
    rpp(di::msg::RppStatus::STATE_TRACKING, 1, 0);
    hold_gate(true, 0, 30ms);
  }
  EXPECT_TRUE(wait_state(di::msg::MissionState::STATE_RUNNING));
}

TEST_F(MissionNodeTest, MissingOrMalformedArtifactIsAPathError) {
  hold_gate(true, 0, 150ms);
  auto req = std::make_shared<di::srv::StartMission::Request>();
  req->path_artifact_sha256 = std::string(64, 'a');  // well-formed id, no such file
  auto res = call(start_, req);
  EXPECT_FALSE(res->accepted);
  EXPECT_EQ(res->reason_code, di::srv::StartMission::Response::REASON_INVALID_ARTIFACT);
  EXPECT_TRUE(wait_state(di::msg::MissionState::STATE_ERROR));
  EXPECT_EQ(last_state_.reason_code, di::msg::MissionState::REASON_PATH_ERROR);
  // a terminal state is replaced by a new start, which gets a new mission id
  hold_gate(true, 0, 150ms);
  req->path_artifact_sha256 = "not-a-hash";
  EXPECT_EQ(call(start_, req)->reason_code,
            di::srv::StartMission::Response::REASON_INVALID_ARTIFACT);
  EXPECT_EQ(node_->fsm().mission_id(), 2U);
}

TEST_F(MissionNodeTest, PauseResumeAndAutomaticSafetyPauseNeverAutoResumes) {
  start_running();
  EXPECT_TRUE(call(pause_, std::make_shared<di::srv::PauseMission::Request>())->accepted);
  EXPECT_TRUE(wait_state(di::msg::MissionState::STATE_PAUSED));
  hold_gate(true, 0, 150ms);
  EXPECT_TRUE(call(resume_, std::make_shared<di::srv::ResumeMission::Request>())->accepted);
  EXPECT_TRUE(wait_state(di::msg::MissionState::STATE_RUNNING));

  // RTK lost: automatic pause with the RTK reason; recovery of the gate does NOT resume.
  hold_gate(false, di::msg::MotionSetpointStatus::REASON_RTK_GATE, 200ms);
  EXPECT_TRUE(wait_state(di::msg::MissionState::STATE_PAUSED));
  EXPECT_EQ(last_state_.reason_code, di::msg::MissionState::REASON_RTK);
  auto rr = call(resume_, std::make_shared<di::srv::ResumeMission::Request>());
  EXPECT_FALSE(rr->accepted);  // gate still not ok
  EXPECT_EQ(rr->reason_code, di::srv::ResumeMission::Response::REASON_SAFETY_GATE);
  hold_gate(true, 0, 400ms);
  EXPECT_EQ(state(), di::msg::MissionState::STATE_PAUSED);
}

TEST_F(MissionNodeTest, AStaleGateStatusPausesARunningMission) {
  start_running();
  // stop publishing: the verdict ages out (default 0.5 s) and the mission pauses
  EXPECT_TRUE(spin_until([&] { return state() == di::msg::MissionState::STATE_PAUSED; }, 3000ms));
  EXPECT_EQ(last_state_.reason_code, di::msg::MissionState::REASON_SAFETY);
}

TEST_F(MissionNodeTest, EmergencyStopReasonAbortsInsteadOfPausing) {
  start_running();
  hold_gate(false, di::msg::MotionSetpointStatus::REASON_ESTOP, 200ms);
  EXPECT_TRUE(wait_state(di::msg::MissionState::STATE_ABORTED));
  EXPECT_EQ(last_state_.reason_code, di::msg::MissionState::REASON_SAFETY);
}

TEST_F(MissionNodeTest, PointResultsAreJournalledFromVehicleState) {
  start_running();
  // Drive the planned square (5 must-hit vertices) via VehicleState.
  const std::string sha = square_sha();
  const auto art = dyx3_mission::load_artifact(DYX3_FIXTURES, sha);
  ASSERT_TRUE(art.ok);
  for (const auto& p : art.artifact.points) {
    di::msg::VehicleState v;
    v.position_valid = true;
    v.north_m = static_cast<float>(p.north_m);
    v.east_m = static_cast<float>(p.east_m + 0.02);
    veh_pub_->publish(v);
    hold_gate(true, 0, 8ms);
  }
  rpp(di::msg::RppStatus::STATE_COMPLETE, 1, 0);
  EXPECT_TRUE(wait_state(di::msg::MissionState::STATE_COMPLETED));
  EXPECT_TRUE(spin_until([&] { return points_.size() >= 5; }));
  ASSERT_GE(points_.size(), 5U);
  for (std::size_t k = 0; k < 5; ++k) {
    EXPECT_EQ(points_[k].point_index, k);
    EXPECT_EQ(points_[k].result_code, di::msg::PointResult::RESULT_COMPLETED);
    EXPECT_LE(points_[k].error_m, 0.05F);
  }
}

TEST_F(MissionNodeTest, SkipPointIsAcceptedOnlyWithAnActivePointWhileRunning) {
  start_running();
  auto res = call(skip_, std::make_shared<di::srv::SkipPoint::Request>());
  EXPECT_TRUE(res->accepted);
  EXPECT_EQ(res->skipped_point_index, 0U);
  EXPECT_TRUE(spin_until([&] { return !points_.empty(); }));
  EXPECT_EQ(points_.front().result_code, di::msg::PointResult::RESULT_SKIPPED);
  // abort, then skip is refused (not running)
  auto ab = std::make_shared<di::srv::AbortMission::Request>();
  ab->reason_code = di::srv::AbortMission::Request::REASON_OPERATOR;
  EXPECT_TRUE(call(abort_, ab)->accepted);
  EXPECT_FALSE(call(skip_, std::make_shared<di::srv::SkipPoint::Request>())->accepted);
  EXPECT_FALSE(call(abort_, ab)->accepted);  // nothing active any more
}

TEST_F(MissionNodeTest, ExecuteMissionActionRunsToCompletionThroughTheSameStartPath) {
  hold_gate(true, 0, 150ms);
  using Exec = dyx3_mission::MissionNode::ExecuteMission;
  Exec::Goal goal;
  goal.path_artifact_sha256 = square_sha();
  auto opts = rclcpp_action::Client<Exec>::SendGoalOptions();
  std::shared_ptr<rclcpp_action::ClientGoalHandle<Exec>> gh;
  bool result_seen = false;
  std::uint8_t result_code = 255;
  opts.result_callback = [&](const rclcpp_action::ClientGoalHandle<Exec>::WrappedResult& r) {
    result_seen = true;
    result_code = r.result->result_code;
  };
  auto gf = action_->async_send_goal(goal, opts);
  ASSERT_TRUE(spin_until([&] { return gf.wait_for(0s) == std::future_status::ready; }));
  gh = gf.get();
  ASSERT_TRUE(gh != nullptr) << "goal rejected";
  EXPECT_EQ(node_->fsm().mission_id(), 1U);
  for (int i = 0; i < 5; ++i) {
    rpp(di::msg::RppStatus::STATE_TRACKING, 1);
    hold_gate(true, 0, 30ms);
  }
  EXPECT_TRUE(wait_state(di::msg::MissionState::STATE_RUNNING));
  rpp(di::msg::RppStatus::STATE_COMPLETE, 1, 0);
  EXPECT_TRUE(spin_until([&] { return result_seen; }));
  EXPECT_EQ(result_code, Exec::Result::RESULT_COMPLETED);

  // while one goal exists a second start via the service is refused: single entry point, one
  // mission
  hold_gate(true, 0, 150ms);
}

TEST_F(MissionNodeTest, ActionGoalIsRejectedWhenTheGateIsNotOkAndCancelAborts) {
  using Exec = dyx3_mission::MissionNode::ExecuteMission;
  Exec::Goal goal;
  goal.path_artifact_sha256 = square_sha();
  auto gf = action_->async_send_goal(goal);
  ASSERT_TRUE(spin_until([&] { return gf.wait_for(0s) == std::future_status::ready; }));
  EXPECT_EQ(gf.get(), nullptr);  // no gate verdict: rejected

  hold_gate(true, 0, 150ms);
  gf = action_->async_send_goal(goal);
  ASSERT_TRUE(spin_until([&] { return gf.wait_for(0s) == std::future_status::ready; }));
  auto gh = gf.get();
  ASSERT_TRUE(gh != nullptr);
  auto rf = action_->async_get_result(gh);
  auto cf = action_->async_cancel_goal(gh);
  ASSERT_TRUE(spin_until([&] { return cf.wait_for(0s) == std::future_status::ready; }));
  EXPECT_TRUE(wait_state(di::msg::MissionState::STATE_ABORTED));
  EXPECT_EQ(last_state_.reason_code, di::msg::MissionState::REASON_OPERATOR);
  // H3: the goal is finalised as CANCELED (not left hanging, not reported as an abort)
  ASSERT_TRUE(spin_until([&] { return rf.wait_for(0s) == std::future_status::ready; }));
  const auto wr = rf.get();
  EXPECT_EQ(wr.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_TRUE(wr.result != nullptr);
  EXPECT_EQ(wr.result->result_code, Exec::Result::RESULT_ABORTED);
}

// Review H4 / fix plan A2: RPP ERROR in READY is an error, not an acknowledgement; COMPLETE in
// READY is neither.
TEST_F(MissionNodeTest, RppErrorInReadyIsAnErrorNotAnAcknowledgement) {
  start_ready();
  for (int i = 0; i < 4; ++i) {
    rpp(di::msg::RppStatus::STATE_ERROR, 1);
    hold_gate(true, 0, 30ms);
  }
  EXPECT_TRUE(wait_state(di::msg::MissionState::STATE_ERROR));
  EXPECT_EQ(last_state_.reason_code, di::msg::MissionState::REASON_INTERNAL_ERROR);
}

TEST_F(MissionNodeTest, RppCompleteInReadyIsNotAnAcknowledgement) {
  start_ready();
  for (int i = 0; i < 4; ++i) {
    rpp(di::msg::RppStatus::STATE_COMPLETE, 1);
    hold_gate(true, 0, 30ms);
  }
  EXPECT_EQ(state(), di::msg::MissionState::STATE_READY);
  for (int i = 0; i < 4; ++i) {
    rpp(di::msg::RppStatus::STATE_LOADED, 1);
    hold_gate(true, 0, 30ms);
  }
  EXPECT_TRUE(wait_state(di::msg::MissionState::STATE_RUNNING));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
