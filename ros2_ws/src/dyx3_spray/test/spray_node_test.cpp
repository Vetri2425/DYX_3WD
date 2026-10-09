// In-process tests of the spray controller node and the independent watchdog node: a fake world and
// a fake FCU link (it records every valve command and answers with SprayActuatorAck), injected
// clocks, a private DDS domain.
#include "dyx3_spray/spray_node.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <thread>

#include "dyx3_mission/path_artifact.hpp"
#include "dyx3_mission/sha256.hpp"
#include "dyx3_spray/safety_watchdog_node.hpp"

using namespace dyx3_spray;
using namespace std::chrono_literals;
using dyx3_interfaces::msg::SprayActuatorAck;
using dyx3_interfaces::msg::SprayActuatorCommand;

namespace {

// A 10 m north line, TRANSIT 0..2, MARK 2..9 (vertices 2..8 are MARK), TRANSIT 9..10, as a
// content-addressed DYX3PATH artifact.
std::string write_artifact(const std::string& dir) {
  std::string body = "DYX3PATH 1\nframe local_ned\nengine 0123456789abcdef\nmeta {}\npoints 11\n";
  for (int i = 0; i <= 10; ++i)
    body += std::to_string(i) + ".0 0.0 " + ((i >= 2 && i <= 8) ? "1" : "0") + "\n";
  body += "end 11\n";
  const std::string sha = dyx3_mission::sha256_hex(body);
  std::filesystem::create_directories(dir);
  std::ofstream(dir + "/" + sha + ".dyx3path", std::ios::binary) << body;
  return sha;
}

std::string write_conditioned_artifact(const std::string& dir, const std::string& source_sha,
                                       bool wrong_source = false) {
  dyx3_mission::ConditionedRunArtifact run;
  run.profile = 0;
  for (int i = 0; i <= 10; ++i) {
    run.points.push_back({static_cast<double>(i), 0.0});
    run.flags.push_back(i >= 2 && i <= 8 ? 1 : 0);
    run.must_hit.push_back(0);
  }
  const std::string bytes = dyx3_mission::serialize_conditioned_artifact(
      wrong_source ? std::string(64, 'b') : source_sha, "tracking_profile=segment", {run});
  const std::string sha = dyx3_mission::sha256_hex(bytes);
  std::ofstream(dir + "/" + sha + ".dyx3cond", std::ios::binary) << bytes;
  return sha;
}

struct Rig {
  std::shared_ptr<rclcpp::Context> ctx;
  double now{200.0};
  std::string dir;
  std::string sha;
  std::string conditioned_sha;
  std::shared_ptr<SprayNode> spray;
  std::shared_ptr<SafetyWatchdogNode> wd;
  std::shared_ptr<rclcpp::Node> world;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
  rclcpp::Publisher<dyx3_interfaces::msg::VehicleState>::SharedPtr p_veh;
  rclcpp::Publisher<dyx3_interfaces::msg::RtkStatus>::SharedPtr p_rtk;
  rclcpp::Publisher<dyx3_interfaces::msg::RppStatus>::SharedPtr p_rpp;
  rclcpp::Publisher<dyx3_interfaces::msg::MissionState>::SharedPtr p_mission;
  rclcpp::Publisher<dyx3_interfaces::msg::EmergencyStopState>::SharedPtr p_estop;
  rclcpp::Publisher<SprayActuatorAck>::SharedPtr p_ack;
  rclcpp::Client<dyx3_interfaces::srv::SetSprayManual>::SharedPtr cli_manual;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> keep;
  std::vector<SprayActuatorCommand> cmds;
  dyx3_interfaces::msg::SprayStatus status;
  dyx3_interfaces::msg::SprayLease lease;
  dyx3_interfaces::msg::SprayWatchdogStatus wd_status;
  bool fcu_acks{true};
  bool fcu_ok{true};
  bool armed{true};
  bool estop{false};
  bool publish_mission{true};
  bool corrections_fresh{true};
  bool heading_evidence_valid{true};
  bool legacy_spray_request{false};
  double heading_error_rad{0.0};
  uint8_t mission_state{dyx3_interfaces::msg::MissionState::STATE_RUNNING};
  int rpp_state{dyx3_interfaces::msg::RppStatus::STATE_TRACKING};  // -1: RPP is dead (silent)
  std::string mission_sha;
  double north{0.0};
  double speed{0.35};
  size_t acked{0};

  explicit Rig(bool with_artifact = true, const std::vector<rclcpp::Parameter>& spray_params = {},
               bool wrong_conditioned_source = false) {
    ctx = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions io;
    io.set_domain_id(120 + (getpid() % 100));
    ctx->init(0, nullptr, io);
    dir = (std::filesystem::temp_directory_path() /
           ("dyx3_spray_test_" + std::to_string(getpid()) + "_" +
            std::to_string(reinterpret_cast<uintptr_t>(this))))
              .string();
    sha = with_artifact ? write_artifact(dir) : std::string(64, 'f');
    conditioned_sha = with_artifact ? write_conditioned_artifact(dir, sha, wrong_conditioned_source)
                                    : std::string();
    mission_sha = sha;
    rclcpp::NodeOptions so;
    so.context(ctx);
    so.append_parameter_override("artifact_dir", dir);
    so.append_parameter_override("gps_recover_hold_s", 0.0);
    for (const auto& p : spray_params)
      so.append_parameter_override(p.get_name(), p.get_parameter_value());
    spray = std::make_shared<SprayNode>(so, [this]() { return now; }, false);
    rclcpp::NodeOptions wo;
    wo.context(ctx);
    wd = std::make_shared<SafetyWatchdogNode>(wo, [this]() { return now; }, false);
    rclcpp::NodeOptions wr;
    wr.context(ctx);
    world = std::make_shared<rclcpp::Node>("world", wr);
    rclcpp::ExecutorOptions eo;
    eo.context = ctx;
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(eo);
    exec->add_node(spray);
    exec->add_node(wd);
    exec->add_node(world);
    const auto r1 = rclcpp::QoS(1).reliable();
    const auto r10 = rclcpp::QoS(10).reliable();
    p_veh = world->create_publisher<dyx3_interfaces::msg::VehicleState>("/dyx3/vehicle_state", r1);
    p_rtk = world->create_publisher<dyx3_interfaces::msg::RtkStatus>("/dyx3/rtk_status", r1);
    p_rpp = world->create_publisher<dyx3_interfaces::msg::RppStatus>("/dyx3/rpp/status", r1);
    p_mission =
        world->create_publisher<dyx3_interfaces::msg::MissionState>("/dyx3/mission/state", r1);
    p_estop = world->create_publisher<dyx3_interfaces::msg::EmergencyStopState>(
        "/dyx3/emergency_stop_state", r1);
    p_ack = world->create_publisher<SprayActuatorAck>("/dyx3/spray/actuator_ack", r10);
    // The fake FCU link: record every valve command, answer with an ack.
    keep.push_back(world->create_subscription<SprayActuatorCommand>(
        "/dyx3/spray/actuator_command", r10, [this](SprayActuatorCommand::ConstSharedPtr m) {
          cmds.push_back(*m);
          if (!fcu_acks) return;
          SprayActuatorAck a;
          a.seq = m->seq;
          a.source = m->source;
          a.success = fcu_ok;
          a.result = fcu_ok ? 0 : SprayActuatorAck::RESULT_LINK_REFUSED;
          p_ack->publish(a);
          ++acked;
        }));
    keep.push_back(world->create_subscription<dyx3_interfaces::msg::SprayStatus>(
        "/dyx3/spray/status", r10,
        [this](dyx3_interfaces::msg::SprayStatus::ConstSharedPtr m) { status = *m; }));
    keep.push_back(world->create_subscription<dyx3_interfaces::msg::SprayLease>(
        "/dyx3/spray/lease", r1,
        [this](dyx3_interfaces::msg::SprayLease::ConstSharedPtr m) { lease = *m; }));
    keep.push_back(world->create_subscription<dyx3_interfaces::msg::SprayWatchdogStatus>(
        "/dyx3/spray/watchdog_status", r1,
        [this](dyx3_interfaces::msg::SprayWatchdogStatus::ConstSharedPtr m) { wd_status = *m; }));
    cli_manual =
        world->create_client<dyx3_interfaces::srv::SetSprayManual>("/dyx3/spray/set_manual");
    const auto end = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < end) {
      exec->spin_some(5ms);
      if (spray->count_subscribers("/dyx3/spray/actuator_command") > 0 &&
          world->count_subscribers("/dyx3/vehicle_state") > 0 &&
          world->count_subscribers("/dyx3/rtk_status") > 0 &&
          world->count_subscribers("/dyx3/rpp/status") > 0 &&
          world->count_subscribers("/dyx3/mission/state") > 0 &&
          world->count_subscribers("/dyx3/emergency_stop_state") > 0 &&
          world->count_subscribers("/dyx3/spray/actuator_ack") >= 2 &&
          wd->count_subscribers("/dyx3/spray/watchdog_status") > 0 &&
          world->count_subscribers("/dyx3/spray/lease") >= 1 && cli_manual->service_is_ready()) {
        return;
      }
    }
    ADD_FAILURE() << "DDS discovery did not complete";
  }
  ~Rig() {
    exec.reset();
    keep.clear();
    spray.reset();
    wd.reset();
    world.reset();
    ctx->shutdown("test done");
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
  }

  void pump(int ms = 10) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) exec->spin_some(2ms);
  }

  void publish_world() {
    dyx3_interfaces::msg::VehicleState v;
    v.arming_state = armed ? 2 : 1;
    v.nav_state = 14;
    v.position_valid = v.velocity_valid = v.attitude_valid = true;
    v.north_m = static_cast<float>(north);
    v.east_m = 0.0F;
    v.heading_rad = 0.0F;
    v.velocity_north_mps = static_cast<float>(speed);
    p_veh->publish(v);
    dyx3_interfaces::msg::RtkStatus r;
    r.fix_type = 6;
    r.corrections_fresh = corrections_fresh;
    r.horizontal_accuracy_m = 0.02F;
    p_rtk->publish(r);
    if (rpp_state >= 0) {
      dyx3_interfaces::msg::RppStatus p;
      p.state = static_cast<uint8_t>(rpp_state);
      p.conditioned_execution_sha256 = conditioned_sha;
      p.heading_evidence_valid = heading_evidence_valid;
      p.heading_error_rad = static_cast<float>(heading_error_rad);
      p.path_travel_m = static_cast<float>(north);
      p.spray_request = legacy_spray_request;
      p_rpp->publish(p);
    }
    if (publish_mission) {
      dyx3_interfaces::msg::MissionState m;
      m.state = mission_state;
      m.path_artifact_sha256 = mission_sha;
      p_mission->publish(m);
    }
    dyx3_interfaces::msg::EmergencyStopState e;
    e.asserted = estop;
    p_estop->publish(e);
  }

  // One 20 ms cycle: world -> deliver -> both nodes step (spray optional: "the controller died") ->
  // deliver.
  void cycle(bool spray_alive = true) {
    now += 0.02;
    publish_world();
    pump(6);
    if (spray_alive) spray->step(now);
    wd->step(now);
    pump(6);
  }

  // Startup: the watchdog's very first OFF can be lost to DDS discovery and is retried after
  // the 1.0 s ack timeout.
  void settle() { run(1.6); }

  void run(double seconds, bool spray_alive = true) {
    for (double t = 0.0; t < seconds; t += 0.02) cycle(spray_alive);
  }

  int count(bool on, uint8_t source) const {
    int n = 0;
    for (const auto& c : cmds) n += (c.on == on && c.source == source) ? 1 : 0;
    return n;
  }
};

}  // namespace

TEST(SprayNode, RejectsAnInvalidParameterAtStartup) {
  auto ctx = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions io;
  io.set_domain_id(120 + (getpid() % 100));
  ctx->init(0, nullptr, io);
  rclcpp::NodeOptions o;
  o.context(ctx);
  o.append_parameter_override("max_xtrack_error_m", -1.0);
  EXPECT_THROW(SprayNode(o, []() { return 1.0; }, false), std::invalid_argument);
  rclcpp::NodeOptions w;
  w.context(ctx);
  w.append_parameter_override("off_value", 2.0);
  EXPECT_THROW(SafetyWatchdogNode(w, []() { return 1.0; }, false), std::invalid_argument);
  ctx->shutdown("test done");
}

TEST(SprayNode, FullRunOpensEarlyOnTheMarkAndClosesAtItsEnd) {
  Rig r;
  r.settle();  // the watchdog proves it can close the valve, the controller proves OFF
  ASSERT_TRUE(r.wd_status.off_authority_ready);
  EXPECT_FALSE(r.status.spraying);
  double on_at = -1.0, off_at = -1.0;
  bool prev = false;
  for (r.north = 0.0; r.north < 9.9; r.north += 0.007) {
    r.cycle();
    if (r.status.spraying && !prev && on_at < 0) on_at = r.north;
    if (!r.status.spraying && prev) off_at = r.north;
    prev = r.status.spraying;
  }
  EXPECT_NEAR(on_at, 2.0 - 0.083, 0.08);
  EXPECT_NEAR(off_at, 9.0 - 0.0175,
              0.08);  // see the controller test: the debounce is led (SP-002)
  EXPECT_GE(r.count(true, SprayActuatorCommand::SOURCE_CONTROLLER), 1);
  const auto& last_ctl = [&]() -> const SprayActuatorCommand& {
    for (auto it = r.cmds.rbegin(); it != r.cmds.rend(); ++it) {
      if (it->source == SprayActuatorCommand::SOURCE_CONTROLLER) return *it;
    }
    return r.cmds.front();
  }();
  EXPECT_FALSE(last_ctl.on);
  // every ON carried the configured actuator slot and the ON value; every OFF the OFF value
  for (const auto& c : r.cmds) {
    EXPECT_EQ(c.backend, SprayActuatorCommand::BACKEND_ACTUATOR);
    EXPECT_EQ(c.actuator_set_index, 1);
    EXPECT_FLOAT_EQ(c.value, c.on ? 1.0F : -1.0F);
  }
}

TEST(SprayNode, StaleCorrectionsBlockValveEvenWithFixedSolutionAndGoodAccuracy) {
  Rig r;
  r.settle();
  r.corrections_fresh = false;
  for (r.north = 0.0; r.north < 4.0; r.north += 0.007) r.cycle();
  EXPECT_EQ(r.count(true, SprayActuatorCommand::SOURCE_CONTROLLER), 0);
  EXPECT_FALSE(r.status.safety_ok);
  EXPECT_EQ(r.status.safety_reason, "RTK corrections stale");
  EXPECT_FALSE(r.lease.allow_on);
}

TEST(SprayNode, ControllerDeathIsClosedByTheIndependentWatchdog) {
  Rig r;
  r.settle();
  for (r.north = 0.0; r.north < 4.0; r.north += 0.007) r.cycle();
  ASSERT_TRUE(r.status.spraying);
  ASSERT_TRUE(r.lease.allow_on);
  const int wd_offs_before = r.count(false, SprayActuatorCommand::SOURCE_WATCHDOG);
  r.run(0.6, /*spray_alive=*/false);  // the controller froze: no more leases
  EXPECT_FALSE(r.wd_status.allow_on);
  EXPECT_NE(r.wd_status.off_reason.find("stale"), std::string::npos);
  EXPECT_GT(r.count(false, SprayActuatorCommand::SOURCE_WATCHDOG), wd_offs_before);
  // the OFF the watchdog sends carries the mapping from the last valid lease
  const auto& c = r.cmds.back();
  EXPECT_EQ(c.source, SprayActuatorCommand::SOURCE_WATCHDOG);
  EXPECT_FALSE(c.on);
  EXPECT_FLOAT_EQ(c.value, -1.0F);
}

TEST(SprayNode, ALinkThatRefusesEveryCommandNeverLatchesOn) {
  Rig r;
  r.fcu_ok =
      false;  // dyx3_px4_link answers RESULT_LINK_REFUSED (handshake not proven / session dead)
  r.settle();
  EXPECT_FALSE(
      r.wd_status
          .off_authority_ready);  // the watchdog cannot prove OFF: the controller must not open
  for (r.north = 0.0; r.north < 5.0; r.north += 0.007) r.cycle();
  EXPECT_FALSE(r.status.spraying);
  EXPECT_EQ(r.count(true, SprayActuatorCommand::SOURCE_CONTROLLER), 0);
  EXPECT_FALSE(r.lease.allow_on);
}

TEST(SprayNode, NoWatchdogHeartbeatNoSpray) {
  Rig r;
  r.settle();
  // silence the watchdog: its heartbeat goes stale (spray_watchdog_timeout_s = 1.0)
  for (r.north = 0.0; r.north < 1.9; r.north += 0.007) {
    r.now += 0.02;
    r.publish_world();
    r.pump(6);
    r.spray->step(r.now);
    r.pump(6);
    if (r.now > 205.0) break;
  }
  for (int i = 0; i < 300; ++i) {  // 6 s without the watchdog stepping
    r.now += 0.02;
    r.north = 3.0;
    r.publish_world();
    r.pump(6);
    r.spray->step(r.now);
    r.pump(6);
  }
  EXPECT_FALSE(r.status.spraying);
  EXPECT_FALSE(r.status.safety_ok);
  EXPECT_NE(r.status.safety_reason.find("watchdog"), std::string::npos);
}

TEST(SprayNode, EmergencyStopClosesTheValveAndRevokesTheLease) {
  Rig r;
  r.settle();
  for (r.north = 0.0; r.north < 4.0; r.north += 0.007) r.cycle();
  ASSERT_TRUE(r.status.spraying);
  r.estop = true;
  r.run(0.2);
  EXPECT_FALSE(r.status.spraying);
  EXPECT_FALSE(r.lease.allow_on);
  EXPECT_FALSE(r.cmds.back().on);
}

// Review C1 / fix plan A1: pause, abort, completion, mission error, RPP error and RPP death all
// close a valve that is ON (OFF command sent and acknowledged), and revoke the lease.
TEST(SprayNode, MissionOrRppLeavingTheRunClosesTheValve) {
  using M = dyx3_interfaces::msg::MissionState;
  using S = dyx3_interfaces::msg::RppStatus;
  struct Case {
    const char* name;
    int mission;
    int rpp;
  };
  const Case cases[] = {
      {"PauseMission", M::STATE_PAUSED, S::STATE_TRACKING},
      {"AbortMission", M::STATE_ABORTED, S::STATE_TRACKING},
      {"mission COMPLETED", M::STATE_COMPLETED, S::STATE_COMPLETE},
      {"mission ERROR", M::STATE_ERROR, S::STATE_TRACKING},
      {"RppStatus ERROR", M::STATE_RUNNING, S::STATE_ERROR},
      {"RPP process killed", M::STATE_RUNNING, -1},
  };
  for (const auto& k : cases) {
    Rig r;
    r.settle();
    for (r.north = 0.0; r.north < 4.0; r.north += 0.007) r.cycle();
    ASSERT_TRUE(r.status.spraying) << k.name;
    ASSERT_TRUE(r.lease.allow_on) << k.name;
    const int offs = r.count(false, SprayActuatorCommand::SOURCE_CONTROLLER);
    const size_t acked = r.acked;
    r.mission_state = static_cast<uint8_t>(k.mission);
    r.rpp_state = k.rpp;
    r.run(0.6);  // > rpp_timeout_s 0.5 for the silent case
    EXPECT_FALSE(r.status.spraying) << k.name;
    EXPECT_FALSE(r.lease.allow_on) << k.name;
    EXPECT_GT(r.count(false, SprayActuatorCommand::SOURCE_CONTROLLER), offs) << k.name;
    EXPECT_GT(r.acked, acked) << k.name;
    EXPECT_FALSE(r.cmds.back().on) << k.name;
  }
}

TEST(SprayNode, UnknownPathMeansPathNotLoaded) {
  Rig r(/*with_artifact=*/false);
  r.settle();
  r.north = 3.0;
  r.run(0.5);
  EXPECT_FALSE(r.status.spraying);
  EXPECT_EQ(r.status.safety_reason, "path not loaded");
}

TEST(SprayNode, ConditionedArtifactWithWrongSourceShaIsRefused) {
  Rig r(/*with_artifact=*/true, {}, /*wrong_conditioned_source=*/true);
  r.settle();
  r.north = 3.0;
  r.run(0.5);
  EXPECT_FALSE(r.status.spraying);
  EXPECT_EQ(r.status.safety_reason, "path not loaded");
}

TEST(SprayNode, HeadingVerdictUsesFreshRppEvidenceAndIgnoresLegacySprayRequest) {
  Rig r;
  r.settle();
  r.north = 3.0;
  r.run(0.4);
  ASSERT_TRUE(r.status.spraying);

  r.legacy_spray_request = true;
  r.run(0.2);
  EXPECT_TRUE(r.status.spraying);
  r.legacy_spray_request = false;
  r.run(0.2);
  EXPECT_TRUE(r.status.spraying);

  r.heading_error_rad = 35.0 * 3.14159265358979323846 / 180.0;
  r.run(0.2);
  EXPECT_FALSE(r.status.spraying);
  EXPECT_FALSE(r.lease.allow_on);

  r.heading_error_rad = 0.0;
  r.heading_evidence_valid = false;
  r.run(0.2);
  EXPECT_FALSE(r.status.spraying);
  EXPECT_EQ(r.status.safety_reason, "rpp heading evidence stale or unavailable");
}

TEST(SprayNode, TransitStaysOffAndStoppingStillLaysTheCurrentMark) {
  Rig transit;
  transit.settle();
  transit.north = 0.5;
  transit.run(0.4);
  EXPECT_FALSE(transit.status.spraying);

  Rig stopping;
  stopping.settle();
  stopping.north = 3.0;
  stopping.rpp_state = dyx3_interfaces::msg::RppStatus::STATE_STOPPING;
  stopping.run(0.4);
  EXPECT_TRUE(stopping.status.spraying);
}

TEST(SprayNode, HeadingEntryHoldReleasesFromFreshRppHeadingOrProgress) {
  const std::vector<rclcpp::Parameter> params{
      rclcpp::Parameter("spray_entry_release_travel_m", 100.0)};
  Rig r(true, params);
  r.settle();
  r.north = 3.0;
  r.heading_error_rad = 20.0 * 3.14159265358979323846 / 180.0;
  r.run(0.4);
  EXPECT_FALSE(r.status.spraying);

  r.heading_error_rad = 2.0 * 3.14159265358979323846 / 180.0;
  r.run(0.4);
  EXPECT_TRUE(r.status.spraying);
}

TEST(SprayNode, ManualServiceHonoursTheFailSafes) {
  Rig r;
  r.mission_state = dyx3_interfaces::msg::MissionState::STATE_IDLE;  // bench: no mission
  r.settle();
  r.north = 0.5;
  auto call = [&](bool on) {
    auto req = std::make_shared<dyx3_interfaces::srv::SetSprayManual::Request>();
    req->on = on;
    auto fut = r.cli_manual->async_send_request(req);
    for (int i = 0; i < 400 && fut.wait_for(0ms) != std::future_status::ready; ++i) {
      r.cycle();
    }
    return fut.get();
  };
  const auto ok = call(true);
  EXPECT_TRUE(ok->accepted);
  r.run(0.3);
  EXPECT_TRUE(r.status.spraying);  // bench ON works anywhere on the path while armed
  EXPECT_TRUE(r.status.manual_active);
  r.armed = false;
  r.run(0.2);
  EXPECT_FALSE(r.status.spraying);
  EXPECT_FALSE(r.status.manual_active);
  const auto refused = call(true);
  EXPECT_FALSE(refused->accepted);
  EXPECT_EQ(refused->reason_code, dyx3_interfaces::srv::SetSprayManual::Response::REASON_DISARMED);

  // SP-001: a running mission owns the valve; manual ON is refused (reported as DISABLED).
  r.armed = true;
  r.mission_state = dyx3_interfaces::msg::MissionState::STATE_RUNNING;
  r.run(0.2);
  const auto locked = call(true);
  EXPECT_FALSE(locked->accepted);
  EXPECT_EQ(locked->reason_code, dyx3_interfaces::srv::SetSprayManual::Response::REASON_DISABLED);
  r.run(0.2);
  EXPECT_FALSE(r.status.manual_active);
  EXPECT_FALSE(r.status.spraying);
}

TEST(SprayNode, RuntimeParameterChangesObeyTheirClass) {
  Rig r;
  auto cli = std::make_shared<rclcpp::AsyncParametersClient>(r.world, "spray");
  ASSERT_TRUE(cli->wait_for_service(5s));
  const auto set = [&](const rclcpp::Parameter& p) {
    auto fut = cli->set_parameters({p});
    for (int i = 0; i < 500 && fut.wait_for(0ms) != std::future_status::ready; ++i) r.pump(5);
    return fut.get()[0];
  };
  EXPECT_TRUE(set(rclcpp::Parameter("max_xtrack_error_m", 0.04))
                  .successful);  // IDLE_ONLY while no mission runs
  EXPECT_FALSE(set(rclcpp::Parameter("max_xtrack_error_m", -1.0)).successful);  // invalid value
  EXPECT_FALSE(set(rclcpp::Parameter("actuator_backend", std::string("mavlink_servo_pwm")))
                   .successful);  // RESTART
}

TEST(WatchdogNode, StartsFailClosedAndPublishesItsHeartbeatAndProofOfOffAuthority) {
  Rig r;
  r.run(1.6, /*spray_alive=*/false);
  EXPECT_GE(r.count(false, SprayActuatorCommand::SOURCE_WATCHDOG), 1);
  EXPECT_EQ(r.count(true, SprayActuatorCommand::SOURCE_WATCHDOG),
            0);  // the watchdog only ever asks for OFF
  EXPECT_TRUE(r.wd_status.watchdog_alive);
  EXPECT_TRUE(r.wd_status.off_authority_ready);
  EXPECT_FALSE(r.wd_status.allow_on);
  EXPECT_EQ(r.wd_status.off_reason, "no controller lease");
}

TEST(WatchdogNode, ShutdownSendsOff) {
  Rig r;
  r.run(1.6, false);
  r.cmds.clear();
  r.wd->shutdown_off();
  r.pump(30);
  ASSERT_FALSE(r.cmds.empty());
  EXPECT_FALSE(r.cmds.back().on);
  EXPECT_EQ(r.cmds.back().source, SprayActuatorCommand::SOURCE_WATCHDOG);
}
