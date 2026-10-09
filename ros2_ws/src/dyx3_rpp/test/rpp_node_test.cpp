// In-process tests of the RPP node: a fake world (vehicle state, RTK, mission) publishes over a
// private DDS domain, the node runs on an injected monotonic clock, and the test records the
// MotionSetpoint stream and RppStatus.
#include "dyx3_rpp/rpp_node.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <vector>

#include "dyx3_mission/path_artifact.hpp"
#include "dyx3_mission/sha256.hpp"

using namespace dyx3_rpp;
using namespace std::chrono_literals;
using dyx3_interfaces::msg::MissionState;
using dyx3_interfaces::msg::MotionSetpoint;
using dyx3_interfaces::msg::RppStatus;

namespace {

struct ArtPoint {
  double n, e;
  int flag;  // 1 MARK, 0 TRANSIT
};

// A 6 m north line: TRANSIT 0..1, MARK 1..5, TRANSIT 5..6.
std::vector<ArtPoint> north_line() {
  std::vector<ArtPoint> pts;
  for (int i = 0; i <= 6; ++i)
    pts.push_back({static_cast<double>(i), 0.0, (i >= 1 && i <= 5) ? 1 : 0});
  return pts;
}

// The points as a content-addressed DYX3PATH artifact.
std::string write_artifact(const std::string& dir, const std::vector<ArtPoint>& pts) {
  std::ostringstream body;
  body.imbue(std::locale::classic());
  body << "DYX3PATH 1\nframe local_ned\nengine 0123456789abcdef\nmeta {}\npoints " << pts.size()
       << "\n";
  body << std::setprecision(17);
  for (const auto& p : pts) body << p.n << " " << p.e << " " << p.flag << "\n";
  body << "end " << pts.size() << "\n";
  const std::string text = body.str();
  const std::string sha = dyx3_mission::sha256_hex(text);
  std::filesystem::create_directories(dir);
  std::ofstream(dir + "/" + sha + ".dyx3path", std::ios::binary) << text;
  return sha;
}

struct Rig {
  std::shared_ptr<rclcpp::Context> ctx;
  int64_t now{200'000'000'000};
  std::string dir;
  std::string sha;
  std::shared_ptr<RppNode> rpp;
  std::shared_ptr<rclcpp::Node> world;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
  rclcpp::Publisher<dyx3_interfaces::msg::VehicleState>::SharedPtr p_veh;
  rclcpp::Publisher<dyx3_interfaces::msg::RtkStatus>::SharedPtr p_rtk;
  rclcpp::Publisher<MissionState>::SharedPtr p_mission;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> keep;
  std::vector<MotionSetpoint> motion;
  RppStatus status;
  // the world
  uint8_t mission_state{MissionState::STATE_READY};
  uint32_t mission_id{7};
  std::string mission_sha;
  bool publish_vehicle{true};
  double north{0.0}, east{0.0}, heading{0.0}, speed{0.0};
  uint8_t fix{6};

  explicit Rig(const std::vector<rclcpp::Parameter>& params = {}, bool with_artifact = true,
               const std::vector<ArtPoint>& path = north_line()) {
    ctx = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions io;
    io.set_domain_id(120 + (getpid() % 100));
    ctx->init(0, nullptr, io);
    dir = (std::filesystem::temp_directory_path() /
           ("dyx3_rpp_test_" + std::to_string(getpid()) + "_" +
            std::to_string(reinterpret_cast<uintptr_t>(this))))
              .string();
    sha = with_artifact ? write_artifact(dir, path) : std::string(64, 'e');
    mission_sha = sha;
    rclcpp::NodeOptions no;
    no.context(ctx);
    no.append_parameter_override("artifact_dir", dir);
    no.append_parameter_override("rtk_recover_hold_s", 0.0);
    no.append_parameter_override("entry_prealign_enabled", false);
    for (const auto& p : params)
      no.append_parameter_override(p.get_name(), p.get_parameter_value());
    rpp = std::make_shared<RppNode>(no, [this]() { return now; }, false);
    rclcpp::NodeOptions wo;
    wo.context(ctx);
    world = std::make_shared<rclcpp::Node>("world", wo);
    rclcpp::ExecutorOptions eo;
    eo.context = ctx;
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(eo);
    exec->add_node(rpp);
    exec->add_node(world);
    const auto r1 = rclcpp::QoS(1).reliable();
    p_veh = world->create_publisher<dyx3_interfaces::msg::VehicleState>("/dyx3/vehicle_state", r1);
    p_rtk = world->create_publisher<dyx3_interfaces::msg::RtkStatus>("/dyx3/rtk_status", r1);
    p_mission = world->create_publisher<MissionState>("/dyx3/mission/state", r1);
    keep.push_back(world->create_subscription<MotionSetpoint>(
        "/dyx3/rpp/motion_setpoint", r1,
        [this](MotionSetpoint::ConstSharedPtr m) { motion.push_back(*m); }));
    keep.push_back(world->create_subscription<RppStatus>(
        "/dyx3/rpp/status", r1, [this](RppStatus::ConstSharedPtr m) { status = *m; }));
    const auto end = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < end) {
      exec->spin_some(5ms);
      if (world->count_subscribers("/dyx3/vehicle_state") > 0 &&
          world->count_subscribers("/dyx3/rtk_status") > 0 &&
          world->count_subscribers("/dyx3/mission/state") > 0 &&
          rpp->count_subscribers("/dyx3/rpp/motion_setpoint") > 0 &&
          rpp->count_subscribers("/dyx3/rpp/status") > 0)
        return;
    }
    ADD_FAILURE() << "DDS discovery did not complete";
  }
  ~Rig() {
    exec.reset();
    keep.clear();
    rpp.reset();
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
    if (publish_vehicle) {
      dyx3_interfaces::msg::VehicleState v;
      v.position_valid = v.velocity_valid = v.attitude_valid = true;
      v.north_m = static_cast<float>(north);
      v.east_m = static_cast<float>(east);
      v.heading_rad = static_cast<float>(heading);
      v.velocity_north_mps = static_cast<float>(speed * std::cos(heading));
      v.velocity_east_mps = static_cast<float>(speed * std::sin(heading));
      p_veh->publish(v);
    }
    dyx3_interfaces::msg::RtkStatus r;
    r.fix_type = fix;
    r.corrections_fresh = true;
    r.horizontal_accuracy_m = 0.02F;
    p_rtk->publish(r);
    MissionState m;
    m.state = mission_state;
    m.mission_id = mission_id;
    m.path_artifact_sha256 = mission_sha;
    p_mission->publish(m);
  }

  // A kinematic stand-in for the vehicle: it does exactly what the last MotionSetpoint asks
  // (heading follows the heading target at once, speed follows the signed body speed, a pivot turns
  // in place at the commanded rate).
  bool auto_drive{false};
  void integrate(double dt) {
    if (!auto_drive || motion.empty()) return;
    const MotionSetpoint& m = motion.back();
    switch (m.mode) {
      case MotionSetpoint::MODE_TRACK_HEADING:
        heading = m.yaw_setpoint;
        speed = m.speed_body_x;
        break;
      case MotionSetpoint::MODE_TRACK_RATE:
      case MotionSetpoint::MODE_CREEP:
        heading += m.yaw_rate_setpoint * dt;
        speed = m.speed_body_x;
        break;
      case MotionSetpoint::MODE_PIVOT:
        heading += m.yaw_rate_setpoint * dt;
        speed = 0.0;
        break;
      default:
        speed = 0.0;
    }
    north += speed * std::cos(heading) * dt;
    east += speed * std::sin(heading) * dt;
  }

  void cycle() {
    now += 20'000'000;
    publish_world();
    pump(6);
    rpp->step(now);
    pump(6);
    integrate(0.02);
  }
  void run(double seconds) {
    for (double t = 0.0; t < seconds; t += 0.02) cycle();
  }
};

}  // namespace

TEST(RppNode, RejectsAnInvalidParameterAtStartup) {
  auto ctx = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions io;
  io.set_domain_id(120 + (getpid() % 100));
  ctx->init(0, nullptr, io);
  rclcpp::NodeOptions o;
  o.context(ctx);
  o.append_parameter_override("max_linear_vel", -1.0);
  EXPECT_THROW(RppNode(o, []() { return int64_t{1}; }, false), std::invalid_argument);
  rclcpp::NodeOptions t;
  t.context(ctx);
  t.append_parameter_override("tick_hz", 5.0);
  EXPECT_THROW(RppNode(t, []() { return int64_t{1}; }, false), std::invalid_argument);
  ctx->shutdown("test done");
}

TEST(RppNode, LoadsTheArtifactByIdAndAcknowledgesWithStop) {
  Rig r;
  r.run(0.5);
  EXPECT_EQ(r.status.state, RppStatus::STATE_LOADED);
  EXPECT_EQ(r.status.mission_id, 7U);  // the acknowledgement dyx3_mission waits for
  ASSERT_FALSE(r.status.conditioned_execution_sha256.empty());
  const auto conditioned =
      dyx3_mission::load_conditioned_artifact(r.dir, r.status.conditioned_execution_sha256);
  ASSERT_TRUE(conditioned.ok) << conditioned.error;
  EXPECT_EQ(conditioned.artifact.source_sha256, r.sha);
  ASSERT_GT(r.motion.size(), 10U);
  for (size_t i = 0; i < r.motion.size(); ++i) {
    EXPECT_EQ(r.motion[i].mode, MotionSetpoint::MODE_STOP) << i;
    EXPECT_TRUE(r.motion[i].valid);
    EXPECT_EQ(r.motion[i].seq,
              i == 0 ? r.motion[0].seq : r.motion[i - 1].seq + 1);  // monotonic, no gaps
  }
}

TEST(RppNode, PublishesFreshHeadingAndProgressEvidenceOnlyForActiveTracking) {
  Rig r;
  r.mission_state = MissionState::STATE_RUNNING;
  // Let the fake vehicle follow RPP's commands. A stationary vehicle can leave the node in a
  // legitimate STOPPING/PIVOTING transition, making an instantaneous state assertion flaky.
  r.auto_drive = true;
  for (int i = 0; i < 100 && r.status.state != RppStatus::STATE_TRACKING; ++i) r.run(0.02);
  ASSERT_EQ(r.status.state, RppStatus::STATE_TRACKING);
  EXPECT_TRUE(r.status.heading_evidence_valid);
  EXPECT_TRUE(std::isfinite(r.status.heading_error_rad));
  EXPECT_TRUE(std::isfinite(r.status.path_travel_m));
  EXPECT_EQ(r.status.run_index, 0U);

  r.mission_state = MissionState::STATE_PAUSED;
  r.run(0.1);
  EXPECT_FALSE(r.status.heading_evidence_valid);
}

TEST(RppNode, AMissingArtifactIsAnErrorAndStopsNeverGuesses) {
  Rig r({}, /*with_artifact=*/false);
  r.run(0.5);
  EXPECT_EQ(r.status.state, RppStatus::STATE_ERROR);
  EXPECT_EQ(r.status.mission_id, 7U);
  EXPECT_EQ(r.motion.back().mode, MotionSetpoint::MODE_STOP);
}

TEST(RppNode, DrivesTheWholeMissionMarksTheLineAndCompletes) {
  Rig r;
  r.auto_drive = true;
  r.mission_state = MissionState::STATE_RUNNING;
  bool saw_track = false, saw_request = false, request_on_transit = false, saw_stop_at_end = false;
  double max_speed = 0.0;
  for (int i = 0; i < 1500 && r.status.state != RppStatus::STATE_COMPLETE; ++i) {
    r.cycle();
    const MotionSetpoint& m = r.motion.back();
    if (m.mode == MotionSetpoint::MODE_TRACK_HEADING && m.speed_body_x > 0.0F) {
      saw_track = true;
      max_speed = std::max(max_speed, static_cast<double>(m.speed_body_x));
      EXPECT_NEAR(m.yaw_setpoint, 0.0F, 0.2F);  // along the line
      EXPECT_TRUE(std::isnan(m.yaw_rate_setpoint));
    }
    if (r.status.spray_request) {
      saw_request = true;
      // The request follows the CONDITIONED run, which fuses a short unpainted lead into the mark
      // (the prototype's a54fd2d: no double stop at every mark start), so it is true from the start
      // here; the tail stays unpainted. The valve itself is dyx3_spray's boundary projection on the
      // artifact, not this request.
      if (r.north > 5.3) {
        request_on_transit = true;
        ADD_FAILURE() << "request at north " << r.north;
      }
    }
  }
  EXPECT_TRUE(saw_track);
  EXPECT_LE(max_speed, 1.0);
  EXPECT_TRUE(saw_request) << "the planner's MARK flag must reach the request";
  EXPECT_FALSE(request_on_transit) << "no request outside the MARK";
  EXPECT_EQ(r.status.state, RppStatus::STATE_COMPLETE);
  saw_stop_at_end = r.motion.back().mode == MotionSetpoint::MODE_STOP;
  EXPECT_TRUE(saw_stop_at_end);
  EXPECT_NEAR(r.north, 6.0, 0.06) << "stopped on the final point";
  EXPECT_NEAR(r.east, 0.0, 0.03);
}

TEST(RppNode, SegmentRateCommandModePublishesTrackRate) {
  Rig r({rclcpp::Parameter("segment_command_mode", "rate")});
  r.mission_state = MissionState::STATE_RUNNING;
  bool saw_rate = false;
  for (int i = 0; i < 100; ++i) {
    r.cycle();
    const auto& m = r.motion.back();
    if (m.mode == MotionSetpoint::MODE_TRACK_RATE && m.speed_body_x > 0.0F) {
      saw_rate = true;
      EXPECT_TRUE(std::isnan(m.yaw_setpoint));
      EXPECT_TRUE(std::isfinite(m.yaw_rate_setpoint));
      break;
    }
  }
  EXPECT_TRUE(saw_rate);
}

TEST(RppNode, AStalePoseStops) {
  Rig r;
  r.mission_state = MissionState::STATE_RUNNING;
  r.run(0.3);
  r.publish_vehicle = false;  // the estimate stops arriving
  r.run(0.8);                 // pose_max_age_s default 0.5
  EXPECT_EQ(r.motion.back().mode, MotionSetpoint::MODE_STOP);
  EXPECT_EQ(r.status.state, RppStatus::STATE_STOPPING);
  EXPECT_EQ(r.status.tick_state, -1);
}

TEST(RppNode, ARtkDropStopsWithTheReason) {
  Rig r;
  r.mission_state = MissionState::STATE_RUNNING;
  r.run(0.3);
  r.fix = 5;  // RTK_FLOAT: the prototype's gate refuses it
  r.run(0.2);
  EXPECT_EQ(r.motion.back().mode, MotionSetpoint::MODE_STOP);
  EXPECT_EQ(r.status.tick_state, 4);
  EXPECT_EQ(r.status.rtk_reason, 3U);  // fix below the minimum
}

TEST(RppNode, APausedMissionIsStopAndResumesFromRest) {
  Rig r;
  r.auto_drive = true;
  r.mission_state = MissionState::STATE_RUNNING;
  for (int i = 0; i < 300 && !(r.status.state == RppStatus::STATE_TRACKING && r.north > 1.5); ++i)
    r.cycle();
  ASSERT_EQ(r.status.state, RppStatus::STATE_TRACKING);
  r.mission_state = MissionState::STATE_PAUSED;
  r.run(0.2);
  EXPECT_EQ(r.motion.back().mode, MotionSetpoint::MODE_STOP);
  EXPECT_EQ(r.status.state, RppStatus::STATE_LOADED);
  r.run(0.5);  // the vehicle comes to rest
  r.mission_state = MissionState::STATE_RUNNING;
  const size_t from = r.motion.size();
  r.run(0.5);
  // the speed memory was cleared: the first commands after a resume ramp up from rest
  float first_speed = -1.0F;
  for (size_t i = from; i < r.motion.size(); ++i) {
    if (r.motion[i].mode == MotionSetpoint::MODE_TRACK_HEADING) {
      first_speed = r.motion[i].speed_body_x;
      break;
    }
  }
  EXPECT_GE(first_speed, 0.0F);
  EXPECT_LT(first_speed, 0.1F);
}

TEST(RppNode, AnEntryPivotIsPivotingAndSpraysNothing) {
  Rig r({rclcpp::Parameter("entry_prealign_enabled", true)});
  r.mission_state = MissionState::STATE_RUNNING;
  r.heading = 1.0;  // the rover points 57 degrees off the first leg
  r.run(0.6);
  bool saw_pivot = false;
  for (const auto& m : r.motion) {
    if (m.mode == MotionSetpoint::MODE_PIVOT) {
      saw_pivot = true;
      EXPECT_EQ(m.speed_body_x, 0.0F);
      EXPECT_LT(m.yaw_rate_setpoint, 0.0F);  // toward north: counter-clockwise
    }
  }
  EXPECT_TRUE(saw_pivot);
  EXPECT_EQ(r.status.state, RppStatus::STATE_PIVOTING);
  EXPECT_FALSE(r.status.spray_request);
}

TEST(RppNode, ParameterRulesAreEnforcedAndRecorded) {
  Rig r;
  // LIVE: accepted, applied and recorded
  EXPECT_TRUE(r.rpp->set_parameter(rclcpp::Parameter("mission_speed", 0.5)).successful);
  EXPECT_DOUBLE_EQ(r.rpp->params().num(P::mission_speed), 0.5);
  EXPECT_FALSE(r.rpp->params().journal().empty());
  // out of range: refused, value unchanged
  EXPECT_FALSE(r.rpp->set_parameter(rclcpp::Parameter("mission_speed", -1.0)).successful);
  EXPECT_DOUBLE_EQ(r.rpp->params().num(P::mission_speed), 0.5);
  // unknown: refused
  EXPECT_THROW(r.rpp->set_parameter(rclcpp::Parameter("not_a_parameter", 1.0)),
               rclcpp::exceptions::ParameterNotDeclaredException);
  // IDLE_ONLY while a mission runs: refused
  r.mission_state = MissionState::STATE_RUNNING;
  r.run(0.2);
  const double before = r.rpp->params().num(P::segment_corner_threshold_deg);
  EXPECT_FALSE(r.rpp->set_parameter(rclcpp::Parameter("segment_corner_threshold_deg", before + 5.0))
                   .successful);
  EXPECT_DOUBLE_EQ(r.rpp->params().num(P::segment_corner_threshold_deg), before);
}

TEST(RppNode, TheUnportedPointHoldRefusesToDriveAndSaysSo) {
  Rig r({rclcpp::Parameter("point_hold_enabled", true)});
  r.auto_drive = true;
  r.mission_state = MissionState::STATE_RUNNING;
  r.run(2.0);  // the entry alignment (the run starts on a MARK) completes first, then the unported
               // feature is reached
  EXPECT_EQ(r.status.state, RppStatus::STATE_ERROR)
      << "tick_state " << int(r.status.tick_state) << " rtk " << int(r.status.rtk_reason) << " seg "
      << int(r.status.segment_state);
  EXPECT_EQ(r.status.handoff, 1U);
  EXPECT_EQ(r.motion.back().mode, MotionSetpoint::MODE_STOP);
}

TEST(RppNode, TheCommandStreamNeverCarriesANonFiniteValueInTheWrongField) {
  Rig r({rclcpp::Parameter("entry_prealign_enabled", true)});
  r.auto_drive = true;
  r.heading = 0.4;
  r.mission_state = MissionState::STATE_RUNNING;
  r.run(8.0);  // pivot, brake, track, approach, stop: every mode the node can emit
  for (const auto& m : r.motion) {
    switch (m.mode) {
      case MotionSetpoint::MODE_STOP:
        EXPECT_EQ(m.speed_body_x, 0.0F);
        EXPECT_TRUE(std::isnan(m.yaw_setpoint));
        EXPECT_EQ(m.yaw_rate_setpoint, 0.0F);
        break;
      case MotionSetpoint::MODE_TRACK_HEADING:
        EXPECT_TRUE(std::isfinite(m.speed_body_x) && std::isfinite(m.yaw_setpoint) &&
                    std::isnan(m.yaw_rate_setpoint));
        break;
      case MotionSetpoint::MODE_TRACK_RATE:
      case MotionSetpoint::MODE_CREEP:
        EXPECT_TRUE(std::isfinite(m.speed_body_x) && std::isfinite(m.yaw_rate_setpoint) &&
                    std::isnan(m.yaw_setpoint));
        break;
      case MotionSetpoint::MODE_PIVOT:
        EXPECT_EQ(m.speed_body_x, 0.0F);
        EXPECT_TRUE(std::isfinite(m.yaw_rate_setpoint) && std::isnan(m.yaw_setpoint));
        break;
      default:
        ADD_FAILURE() << "unknown mode " << static_cast<int>(m.mode);
    }
  }
}

namespace {
// Sign changes of the commanded body speed, counting only commands at or above the stop speed
// threshold (segment_stop_speed_threshold, 0.02 m/s): below it the rover is stopped by definition.
int speed_sign_changes(const std::vector<MotionSetpoint>& ms, size_t from) {
  int changes = 0, last = 0;
  for (size_t i = from; i < ms.size(); ++i) {
    const float v = ms[i].speed_body_x;
    if (std::fabs(v) < 0.02F) continue;
    const int s = v > 0.0F ? 1 : -1;
    if (last != 0 && s != last) ++changes;
    last = s;
  }
  return changes;
}
}  // namespace

// XR-RPP-001: the final approach ends 3 cm to the side of the endpoint. The precise stop aims
// diagonally; the published command must carry that direction, so the rover removes the lateral
// miss and completes instead of rocking through the end plane along its nose.
TEST(RppNode, AnEndpointWithALateralMissCompletesWithoutRocking) {
  // pivot_to_intercept off: the entry alignment holds the leg heading, so the 3 cm miss is intact
  // when the precise stop engages (it is a final approach, not a line acquisition).
  Rig r({rclcpp::Parameter("pivot_to_intercept_enabled", false)});
  r.auto_drive = true;
  r.north = 5.92;
  r.east = 0.03;
  r.heading = 0.0;
  r.mission_state = MissionState::STATE_RUNNING;
  const size_t from = r.motion.size();
  bool saw_creeping = false, saw_steer = false;
  int ticks = 0;
  for (; ticks < 500 && r.status.state != RppStatus::STATE_COMPLETE; ++ticks) {  // 10 s
    r.cycle();
    if (r.status.state == RppStatus::STATE_CREEPING) {
      saw_creeping = true;
      const MotionSetpoint& m = r.motion.back();
      if (m.mode == MotionSetpoint::MODE_TRACK_HEADING && std::fabs(m.yaw_setpoint) > 0.05F)
        saw_steer = true;  // toward the endpoint, off the line heading
    }
  }
  EXPECT_TRUE(saw_creeping) << "the precise stop never engaged";
  EXPECT_TRUE(saw_steer) << "the lateral correction never reached the command";
  EXPECT_EQ(r.status.state, RppStatus::STATE_COMPLETE)
      << "not complete after " << ticks * 0.02 << " s at n " << r.north << " e " << r.east;
  EXPECT_LE(speed_sign_changes(r.motion, from), 2);
  EXPECT_NEAR(r.north, 6.0, 0.02 + 1e-3);
  EXPECT_NEAR(r.east, 0.0, 0.02 + 1e-3);
  EXPECT_EQ(r.motion.back().mode, MotionSetpoint::MODE_STOP);
}
