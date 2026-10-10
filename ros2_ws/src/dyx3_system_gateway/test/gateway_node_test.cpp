// In-process tests of the gateway node over a REAL Unix socket: fake services stand in for the
// mission node, motion_guard, px4_link and spray; an injected clock; a private DDS domain.
// No fixed wall-clock waits: DDS delivery is drained (dds_test_support.hpp) and anything owned by
// the gateway's IPC thread (the inbox, client connections, socket replies) is waited on as a
// condition with a bound that only limits a failure.
#include "dyx3_system_gateway/gateway_node.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include "dds_test_support.hpp"
#include "dyx3_system_gateway/json.hpp"
#include "sock_client.hpp"

using namespace dyx3_gateway;
using namespace std::chrono_literals;
namespace srv = dyx3_interfaces::srv;

namespace {

double steady_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Rig {
  std::shared_ptr<rclcpp::Context> ctx;
  double now{50.0};
  std::string sock;
  std::shared_ptr<GatewayNode> gw;
  std::shared_ptr<rclcpp::Node> world;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
  rclcpp::Subscription<dyx3_interfaces::msg::OperatorLinkStatus>::SharedPtr s_link;
  rclcpp::Publisher<dyx3_interfaces::msg::RtkStatus>::SharedPtr p_rtk;
  rclcpp::Publisher<dyx3_interfaces::msg::MissionState>::SharedPtr p_mission;
  rclcpp::Publisher<dyx3_interfaces::msg::EmergencyStopState>::SharedPtr p_estop;
  rclcpp::Publisher<dyx3_interfaces::msg::SafetyGateStatus>::SharedPtr p_safety;
  rclcpp::Publisher<dyx3_interfaces::msg::Px4LinkStatus>::SharedPtr p_px4;
  rclcpp::Publisher<dyx3_interfaces::msg::VehicleState>::SharedPtr p_vehicle;
  rclcpp::Publisher<dyx3_interfaces::msg::RppStatus>::SharedPtr p_rpp;
  dyx3_interfaces::msg::OperatorLinkStatus link;
  bool link_seen{false};
  // fake services; the *_answers flags false = the request is received but never answered
  std::atomic<bool> estop_answers{true}, estop_accepts{true}, start_accepts{true},
      start_answers{true}, offboard_answers{true}, start_duplicate{false};
  // the fake mission node's refusal (used when start_accepts is false) and the guard gate it names
  std::atomic<uint8_t> start_refusal{srv::StartMission::Response::REASON_BUSY};
  std::atomic<uint8_t> start_gate_reason{0};
  // Guarded by mu: an autonomous rig runs the fake services on its spin thread.
  std::mutex mu;
  std::vector<std::string> calls;
  std::string last_start_request_id;
  bool last_start_resume{false};
  // the fake mission node's resumed_run_index for a start with resume = true (0 without resume)
  std::atomic<uint32_t> start_resumed_run{4};
  std::vector<double> estop_rx_s;  // steady time each E-stop request reached the guard
  rclcpp::ServiceBase::SharedPtr svc_keep[9];
  // Autonomous mode: the gateway runs on the real clock with its own timer and wake-up, and the
  // executor spins on a background thread, as in production.
  bool autonomous{false};
  std::thread spinner;

  explicit Rig(bool autonomous_mode = false) : autonomous(autonomous_mode) {
    ctx = std::make_shared<rclcpp::Context>();
    dyx3_test::init_isolated(ctx);
    sock = tmp_sock();
    rclcpp::NodeOptions go;
    go.context(ctx);
    go.append_parameter_override("socket_path", sock);
    go.append_parameter_override("service_timeout_s", 1.0);
    if (autonomous) {
      gw = std::make_shared<GatewayNode>(go, nullptr, true);
    } else {
      gw = std::make_shared<GatewayNode>(go, [this]() { return now; }, false);
    }
    rclcpp::NodeOptions wo;
    wo.context(ctx);
    world = std::make_shared<rclcpp::Node>("world", wo);
    rclcpp::ExecutorOptions eo;
    eo.context = ctx;
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(eo);
    exec->add_node(gw);
    exec->add_node(world);

    svc_keep[0] = world->create_service<srv::StartMission>(
        "/dyx3/mission/start", [this](std::shared_ptr<rclcpp::Service<srv::StartMission>> svc,
                                      std::shared_ptr<rmw_request_id_t> hdr,
                                      const std::shared_ptr<srv::StartMission::Request> rq) {
          {
            std::lock_guard<std::mutex> lk(mu);
            calls.push_back("start:" + rq->path_artifact_sha256.substr(0, 4));
            last_start_request_id = rq->request_id;
            last_start_resume = rq->resume;
          }
          if (!start_answers) return;
          srv::StartMission::Response rs;
          rs.accepted = start_accepts;
          rs.reason_code = start_accepts ? 0 : start_refusal.load();
          rs.mission_id = 42;
          rs.duplicate = start_duplicate;
          rs.gate_reason_code = start_gate_reason;
          rs.resumed_run_index = rq->resume ? start_resumed_run.load() : 0U;
          svc->send_response(*hdr, rs);
        });
    svc_keep[1] = world->create_service<srv::AbortMission>(
        "/dyx3/mission/abort", [this](const std::shared_ptr<srv::AbortMission::Request> rq,
                                      std::shared_ptr<srv::AbortMission::Response> rs) {
          std::lock_guard<std::mutex> lk(mu);
          calls.push_back("abort:" + std::to_string(rq->reason_code));
          rs->accepted = true;
        });
    svc_keep[2] = world->create_service<srv::PauseMission>(
        "/dyx3/mission/pause", [this](const std::shared_ptr<srv::PauseMission::Request>,
                                      std::shared_ptr<srv::PauseMission::Response> rs) {
          std::lock_guard<std::mutex> lk(mu);
          calls.push_back("pause");
          rs->accepted = true;
        });
    svc_keep[3] = world->create_service<srv::ResumeMission>(
        "/dyx3/mission/resume", [this](const std::shared_ptr<srv::ResumeMission::Request>,
                                       std::shared_ptr<srv::ResumeMission::Response> rs) {
          std::lock_guard<std::mutex> lk(mu);
          calls.push_back("resume");
          rs->accepted = true;
        });
    svc_keep[4] = world->create_service<srv::SkipPoint>(
        "/dyx3/mission/skip_point", [this](const std::shared_ptr<srv::SkipPoint::Request>,
                                           std::shared_ptr<srv::SkipPoint::Response> rs) {
          std::lock_guard<std::mutex> lk(mu);
          calls.push_back("skip");
          rs->accepted = true;
          rs->skipped_point_index = 5;
        });
    // Deferred response: with estop_answers false the guard receives the request but never
    // answers (a hung guard), which is different from the service being absent.
    svc_keep[5] = world->create_service<srv::SetEmergencyStop>(
        "/dyx3/motion_guard/set_emergency_stop",
        [this](std::shared_ptr<rclcpp::Service<srv::SetEmergencyStop>> svc,
               std::shared_ptr<rmw_request_id_t> hdr,
               const std::shared_ptr<srv::SetEmergencyStop::Request> rq) {
          {
            std::lock_guard<std::mutex> lk(mu);
            calls.push_back(std::string("estop:") + (rq->asserted ? "1" : "0") + ":" + rq->source);
            estop_rx_s.push_back(steady_s());
          }
          if (!estop_answers) return;
          srv::SetEmergencyStop::Response rs;
          rs.accepted = estop_accepts;
          svc->send_response(*hdr, rs);
        });
    svc_keep[6] = world->create_service<srv::ArmDisarm>(
        "/dyx3/px4_link/arm", [this](const std::shared_ptr<srv::ArmDisarm::Request> rq,
                                     std::shared_ptr<srv::ArmDisarm::Response> rs) {
          std::lock_guard<std::mutex> lk(mu);
          calls.push_back(std::string("arm:") + (rq->arm ? "1" : "0"));
          rs->accepted = false;
          rs->reason_code = srv::ArmDisarm::Response::REASON_LINK_UNHEALTHY;
        });
    // Deferred like the E-stop: with offboard_answers false px4_link is "still confirming".
    svc_keep[7] = world->create_service<srv::SetOffboard>(
        "/dyx3/px4_link/set_offboard",
        [this](std::shared_ptr<rclcpp::Service<srv::SetOffboard>> svc,
               std::shared_ptr<rmw_request_id_t> hdr,
               const std::shared_ptr<srv::SetOffboard::Request>) {
          {
            std::lock_guard<std::mutex> lk(mu);
            calls.push_back("offboard");
          }
          if (!offboard_answers) return;
          srv::SetOffboard::Response rs;
          rs.accepted = true;
          svc->send_response(*hdr, rs);
        });
    svc_keep[8] = world->create_service<srv::SetSprayManual>(
        "/dyx3/spray/set_manual", [this](const std::shared_ptr<srv::SetSprayManual::Request> rq,
                                         std::shared_ptr<srv::SetSprayManual::Response> rs) {
          std::lock_guard<std::mutex> lk(mu);
          calls.push_back(std::string("spray:") + (rq->on ? "1" : "0"));
          rs->accepted = true;
        });
    s_link = world->create_subscription<dyx3_interfaces::msg::OperatorLinkStatus>(
        "/dyx3/operator_link", rclcpp::QoS(1).reliable(),
        [this](dyx3_interfaces::msg::OperatorLinkStatus::ConstSharedPtr m) {
          link = *m;
          link_seen = true;
        });
    p_rtk = world->create_publisher<dyx3_interfaces::msg::RtkStatus>("/dyx3/rtk_status",
                                                                     rclcpp::QoS(1).reliable());
    p_mission = world->create_publisher<dyx3_interfaces::msg::MissionState>(
        "/dyx3/mission/state", rclcpp::QoS(1).reliable());
    p_estop = world->create_publisher<dyx3_interfaces::msg::EmergencyStopState>(
        "/dyx3/emergency_stop_state", rclcpp::QoS(1).reliable());
    p_safety = world->create_publisher<dyx3_interfaces::msg::SafetyGateStatus>(
        "/dyx3/safety_gate", rclcpp::QoS(1).reliable());
    p_px4 = world->create_publisher<dyx3_interfaces::msg::Px4LinkStatus>("/dyx3/px4_link/status",
                                                                         rclcpp::QoS(1).reliable());
    p_vehicle = world->create_publisher<dyx3_interfaces::msg::VehicleState>(
        "/dyx3/vehicle_state", rclcpp::QoS(1).reliable());
    p_rpp = world->create_publisher<dyx3_interfaces::msg::RppStatus>("/dyx3/rpp/status",
                                                                     rclcpp::QoS(1).reliable());
    // Discovery: every topic matched and every one of the gateway's nine clients reaches its
    // service (a client that does not would answer "service_unavailable").
    const bool discovered = pump_until([this] {
      return gw->count_subscribers("/dyx3/operator_link") > 0 &&
             world->count_subscribers("/dyx3/rtk_status") > 0 &&
             world->count_subscribers("/dyx3/mission/state") > 0 &&
             world->count_subscribers("/dyx3/emergency_stop_state") > 0 &&
             world->count_subscribers("/dyx3/px4_link/status") > 0 &&
             world->count_subscribers("/dyx3/vehicle_state") > 0 &&
             world->count_subscribers("/dyx3/rpp/status") > 0 && gw->unavailable_services().empty();
    });
    if (!discovered) {
      std::string missing;
      for (const auto& n : gw->unavailable_services()) missing += " " + n;
      ADD_FAILURE() << "DDS discovery did not complete; unreachable:" << missing;
    }
    // every later deliver() relies on synchronous delivery: prove it
    EXPECT_TRUE(dyx3_test::delivery_is_synchronous(ctx));
    if (autonomous) spinner = std::thread([this] { exec->spin(); });
  }
  ~Rig() {
    if (spinner.joinable()) {
      exec->cancel();
      spinner.join();
    }
    exec.reset();
    s_link.reset();
    gw.reset();
    world.reset();
    ctx->shutdown("test done");
  }
  // Delivers everything published so far (topics, service requests and responses) and runs every
  // callback it causes, then returns.
  void deliver() { dyx3_test::drain(*exec); }
  // Drive the node until `done` holds. The limit only bounds a failure; no result depends on it.
  bool pump_until(const std::function<bool()>& done, int limit_ms = 10000) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit_ms);
    while (std::chrono::steady_clock::now() < end) {
      deliver();
      if (done()) return true;
      exec->spin_once(1ms);  // blocks only while nothing is ready (or the IPC thread is working)
    }
    deliver();
    return done();
  }
  // Send one line, drive the node until a reply with this id arrives. `advance` of link time is
  // spread over 300 steps, counted from the step that takes the command: the IPC thread queues the
  // line on wall time, and the clock must not run before the command is in, or a timeout case
  // would depend on how fast that thread was scheduled. Past those steps the clock stays put and
  // only a wall-clock bound remains, which limits a failure (a reply that never comes) and decides
  // nothing. A line the IPC thread answers itself (never queued) is read like any other reply.
  JsonValue ask(const Sock& s, const std::string& line, int64_t id, double advance = 0.0) {
    s.write_all(line + "\n");
    JsonValue v;
    const auto end = std::chrono::steady_clock::now() + 15s;
    bool taken = false;
    for (int i = 0; (taken && i < 300) || std::chrono::steady_clock::now() < end;) {
      deliver();
      taken = taken || gw->inbox_depth() > 0;  // this step takes the queued command
      if (taken && i < 300) {
        now += advance / 300.0;
        ++i;
      }
      gw->step(now);
      JsonValue found;
      for (const auto& l : s.read_lines(1, 5)) {
        JsonValue j;
        std::string e;
        if (found.type == JsonValue::Type::Null && parse_json(l, &j, &e) && j.get("id") &&
            j.get("id")->is_int && j.get("id")->i == id) {
          found = j;  // the lines after it in the same read are kept too
        } else {
          s.stash.push_back(l);
        }
      }
      if (found.type != JsonValue::Type::Null) return found;
    }
    ADD_FAILURE() << "no reply to " << line;
    return v;
  }
};

// Autonomous rig: the reply with this id, or a null value after `timeout_ms`.
JsonValue await_reply(const Sock& s, int64_t id, int timeout_ms = 5000) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < end) {
    for (const auto& l : s.read_lines(1, 5)) {
      JsonValue j;
      std::string e;
      if (parse_json(l, &j, &e) && j.get("id") && j.get("id")->is_int && j.get("id")->i == id)
        return j;
    }
  }
  return JsonValue{};
}

// Every event line read until `done` holds over the events read so far (bounded).
std::vector<JsonValue> read_events(Rig& r, const Sock& s,
                                   const std::function<bool(const std::vector<JsonValue>&)>& done,
                                   int limit_ms = 3000) {
  std::vector<JsonValue> ev;
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit_ms);
  std::vector<std::string> lines;
  lines.swap(s.stash);
  while (!done(ev) && std::chrono::steady_clock::now() < end) {
    if (!r.autonomous) r.deliver();
    if (lines.empty()) lines = s.read_lines(1, 5);
    for (const auto& l : lines) {
      JsonValue j;
      std::string e;
      if (parse_json(l, &j, &e) && j.get("type") && j.get("type")->s == "event") ev.push_back(j);
    }
    lines.clear();
  }
  return ev;
}

std::vector<JsonValue> of_kind(const std::vector<JsonValue>& ev, const std::string& kind) {
  std::vector<JsonValue> out;
  for (const auto& e : ev)
    if (e.get("event")->s == kind) out.push_back(e);
  return out;
}

dyx3_interfaces::msg::MissionState mission(uint8_t state, uint8_t reason = 0, uint32_t point = 0) {
  dyx3_interfaces::msg::MissionState m;
  m.state = state;
  m.reason_code = reason;
  m.mission_id = 42;
  m.point_index = point;
  m.path_artifact_sha256 = std::string(64, 'c');
  return m;
}

// A MissionState with every interfaces-0.17.0 field set to a value no other field shares, so a
// field that is dropped or mixed up with another is seen.
dyx3_interfaces::msg::MissionState mission_v2() {
  using M = dyx3_interfaces::msg::MissionState;
  M m = mission(M::STATE_PAUSED, M::REASON_EKF_RESET, 5);
  m.run_index = 3;
  m.path_artifact_sha256 = std::string(64, 'e');  // the execution artifact
  m.source_artifact_sha256 = std::string(64, 'a');
  m.request_id = "tab-1:9f2c";
  m.reason_detail = "EKF reset (xy_reset_counter 4 -> 5); release: disarm \"timeout\"";
  m.gate_reason_code = 7;
  m.waiting_on = M::WAIT_OPERATOR;
  m.state_entered.sec = 1791624580;
  m.state_entered.nanosec = 500000000;
  m.start_run_index = 2;  // 0.17.0
  return m;
}

std::string describe(const std::vector<JsonValue>& ev) {
  std::string out;
  for (const auto& e : ev) {
    out += " [" + e.get("event")->s + " seq " + std::to_string(e.get("seq")->i);
    for (const auto& kv : e.get("data")->o) {
      out += " " + kv.first + "=";
      if (kv.second.type == JsonValue::Type::Bool) out += kv.second.b ? "true" : "false";
      if (kv.second.type == JsonValue::Type::String) out += kv.second.s;
      if (kv.second.type == JsonValue::Type::Number) out += std::to_string(kv.second.n);
    }
    out += "]";
  }
  return out;
}

double percentile(std::vector<double> v, double p) {
  std::sort(v.begin(), v.end());
  return v[static_cast<size_t>(p * static_cast<double>(v.size() - 1))];
}

bool ok_of(const JsonValue& v) { return v.get("ok") && v.get("ok")->b; }
std::string code_of(const JsonValue& v) { return v.get("code") ? v.get("code")->s : "?"; }

}  // namespace

TEST(GatewayNode, CommandsReachTheRightServicesAndAnswersCarryTheDownstreamVerdict) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  auto v = r.ask(c,
                 R"({"v":1,"id":1,"cmd":"start_mission","args":{"path_artifact_sha256":")" +
                     std::string(64, 'c') + R"("}})",
                 1);
  EXPECT_TRUE(ok_of(v));
  EXPECT_EQ(v.get("data")->get("mission_id")->i, 42);
  r.start_accepts = false;
  v = r.ask(c,
            R"({"v":1,"id":2,"cmd":"start_mission","args":{"path_artifact_sha256":")" +
                std::string(64, 'c') + R"("}})",
            2);
  EXPECT_FALSE(ok_of(v));
  EXPECT_EQ(code_of(v), "rejected");
  EXPECT_EQ(v.get("data")->get("reason_code")->i, srv::StartMission::Response::REASON_BUSY);
  v = r.ask(c, R"({"v":1,"id":3,"cmd":"arm","args":{"arm":true}})", 3);
  EXPECT_FALSE(ok_of(v));  // px4_link refused: its reason is passed through, not reinterpreted
  EXPECT_EQ(v.get("data")->get("reason_code")->i, srv::ArmDisarm::Response::REASON_LINK_UNHEALTHY);
  EXPECT_TRUE(ok_of(r.ask(c, R"({"v":1,"id":4,"cmd":"pause_mission"})", 4)));
  EXPECT_TRUE(ok_of(r.ask(c, R"({"v":1,"id":5,"cmd":"resume_mission"})", 5)));
  v = r.ask(c, R"({"v":1,"id":6,"cmd":"skip_point"})", 6);
  EXPECT_EQ(v.get("data")->get("skipped_point_index")->i, 5);
  EXPECT_TRUE(
      ok_of(r.ask(c, R"({"v":1,"id":7,"cmd":"abort_mission","args":{"reason":"operator"}})", 7)));
  EXPECT_TRUE(ok_of(r.ask(c, R"({"v":1,"id":8,"cmd":"offboard","args":{"enable":true}})", 8)));
  EXPECT_TRUE(ok_of(r.ask(c, R"({"v":1,"id":9,"cmd":"spray_manual","args":{"on":true}})", 9)));
  const std::vector<std::string> expect{"start:cccc", "start:cccc", "arm:1",    "pause",  "resume",
                                        "skip",       "abort:1",    "offboard", "spray:1"};
  EXPECT_EQ(r.calls, expect);
}

TEST(GatewayNode, BadInputNeverReachesRosAndIsAnsweredWithTheId) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  auto v = r.ask(c, R"({"v":1,"id":11,"cmd":"reboot"})", 11);
  EXPECT_FALSE(ok_of(v));
  EXPECT_EQ(code_of(v), "invalid_command");
  v = r.ask(c, R"({"v":1,"id":12,"cmd":"estop","args":{"asserted":true,"source":"root"}})", 12);
  EXPECT_EQ(code_of(v), "invalid_command");
  c.write_all("garbage\n");
  r.deliver();
  const auto lines = c.read_lines(1);  // waits for the reply (bounded)
  ASSERT_EQ(lines.size(), 1U);
  EXPECT_NE(lines[0].find("bad_message"), std::string::npos);
  // invalid UTF-8 is refused and the reply carries none of it (GW-008)
  c.write_all("{\"v\":1,\"id\":13,\"cmd\":\"\xC0\xAF\xFF\"}\n");
  r.deliver();
  const auto bad = c.read_lines(1);  // waits for the reply (bounded)
  ASSERT_EQ(bad.size(), 1U);
  EXPECT_NE(bad[0].find("bad_message"), std::string::npos);
  EXPECT_TRUE(is_valid_utf8(bad[0])) << bad[0];
  EXPECT_TRUE(r.calls.empty());
}

TEST(GatewayNode, AnEstopThatCannotBeDeliveredIsNeverReportedAsAccepted) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  const std::string assert_line = R"(,"cmd":"estop","args":{"asserted":true,"source":"tablet"}})";
  // 1. delivered and accepted
  auto v = r.ask(c, R"({"v":1,"id":21)" + assert_line, 21);
  EXPECT_TRUE(ok_of(v));
  EXPECT_EQ(code_of(v), "ok");
  EXPECT_EQ(r.calls.back(), "estop:1:tablet");
  // 2. delivered and refused by the guard: the refusal is passed through
  r.estop_accepts = false;
  v = r.ask(c, R"({"v":1,"id":22)" + assert_line, 22);
  EXPECT_FALSE(ok_of(v));
  EXPECT_EQ(code_of(v), "rejected");
  r.estop_accepts = true;
  // 3. the guard's service exists and receives the request but never answers: exactly "timeout"
  // once service_timeout_s (1.0 s here) has passed, never "ok"
  r.estop_answers = false;
  const size_t n_calls = r.calls.size();
  v = r.ask(c, R"({"v":1,"id":23)" + assert_line, 23, 3.0);
  EXPECT_FALSE(ok_of(v));
  EXPECT_EQ(code_of(v), "timeout");
  EXPECT_EQ(r.calls.size(), n_calls + 1);  // it was delivered, the answer never came
  // 4. the guard's service disappears: exactly "service_unavailable", at once
  r.svc_keep[5].reset();
  // (by node: the plain service list also shows the name while the gateway's client exists)
  const auto gone = [&r]() {
    const auto names = r.gw->get_service_names_and_types_by_node("world", "/");
    return names.find("/dyx3/motion_guard/set_emergency_stop") == names.end();
  };
  ASSERT_TRUE(r.pump_until(gone, 5000));
  // ... and the gateway's own client has lost it (not a bet on how long that takes)
  ASSERT_TRUE(r.pump_until(
      [&r] {
        const auto down = r.gw->unavailable_services();
        return std::find(down.begin(), down.end(), "/dyx3/motion_guard/set_emergency_stop") !=
               down.end();
      },
      5000));
  v = r.ask(c, R"({"v":1,"id":24)" + assert_line, 24);
  EXPECT_FALSE(ok_of(v));
  EXPECT_EQ(code_of(v), "service_unavailable");
  EXPECT_EQ(r.calls.size(), n_calls + 1);
}

TEST(GatewayNode, TimedOutRequestsAreRemovedFromTheRosClient) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  r.estop_answers = false;  // the guard's service exists but never answers
  for (int i = 0; i < 3; ++i) {
    const auto v = r.ask(c,
                         R"({"v":1,"id":)" + std::to_string(70 + i) +
                             R"(,"cmd":"estop","args":{"asserted":true,"source":"tablet"}})",
                         70 + i, 3.0);
    EXPECT_EQ(code_of(v), "timeout");
  }
  EXPECT_EQ(r.calls.size(), 3U);  // every request reached the guard
  // nothing the gateway already answered with "timeout" is left pending inside rclcpp
  EXPECT_EQ(r.gw->prune_rclcpp_pending_requests(), 0U);
}

TEST(GatewayNode, EstopIsProcessedBeforeOtherQueuedCommands) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  // Three commands in one burst; the E-stop is last on the wire but must be called first.
  c.write_all(R"({"v":1,"id":31,"cmd":"pause_mission"})"
              "\n"
              R"({"v":1,"id":32,"cmd":"resume_mission"})"
              "\n"
              R"({"v":1,"id":33,"cmd":"estop","args":{"asserted":true,"source":"backend"}})"
              "\n");
  // One step must see the whole burst: wait until the IPC thread has queued all three.
  ASSERT_TRUE(r.pump_until([&r] { return r.gw->inbox_depth() == 3U; }))
      << "inbox depth " << r.gw->inbox_depth();
  r.gw->step(r.now);
  r.pump_until([&r] { return r.calls.size() >= 3U; });  // the three fake handlers ran
  // Service endpoints are separate DDS entities, so the executor may run the fake handlers in any
  // order: the guarantee under test is the order in which the GATEWAY processed the batch.
  const auto& batch = r.gw->last_batch();
  ASSERT_EQ(batch.size(), 3U);
  EXPECT_EQ(batch[0], CmdKind::Estop);
  EXPECT_EQ(batch[1], CmdKind::PauseMission);
  EXPECT_EQ(batch[2], CmdKind::ResumeMission);
  EXPECT_EQ(r.calls.size(), 3U);
}

TEST(GatewayNode, AnEstopBehindAHeartbeatFloodIsDispatchedFirstAndTheInboxStaysBounded) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  const int n = 1000;
  std::string burst;
  for (int i = 0; i < n; ++i)
    burst += R"({"v":1,"id":)" + std::to_string(1000 + i) + R"(,"cmd":"heartbeat"})" + "\n";
  burst += R"({"v":1,"id":99,"cmd":"estop","args":{"asserted":true,"source":"tablet"}})"
           "\n";
  c.write_all(burst);
  // No step runs until the IPC thread has handled the whole burst: the inbox then holds exactly
  // kInboxCap heartbeats plus the E-stop (the last line on the wire, never refused), and every
  // other heartbeat has been refused "busy". Waiting on that state, not on a fixed time, is what
  // puts the E-stop and the heartbeats queued with it in ONE batch.
  ASSERT_TRUE(r.pump_until([&r] { return r.gw->inbox_depth() == GatewayNode::kInboxCap + 1; }))
      << "inbox depth " << r.gw->inbox_depth();
  r.gw->step(r.now);
  // Collect replies until every heartbeat and the E-stop are answered. Telemetry is interleaved
  // with the replies, so the number of lines is not fixed; a reply is matched by its id.
  std::map<int64_t, int> answers;  // id -> replies received
  int hb_ok = 0, hb_busy = 0;
  bool estop_ok = false;
  const auto all_answered = [&] { return answers.count(99) && hb_ok + hb_busy >= n; };
  r.pump_until([&] {
    for (const auto& l : c.read_lines(n + 2, 5)) {
      JsonValue j;
      std::string e;
      EXPECT_TRUE(parse_json(l, &j, &e)) << e << ": " << l;
      if (!j.get("id")) continue;  // telemetry
      const int64_t id = j.get("id")->i;
      ++answers[id];
      if (id == 99) {
        estop_ok = ok_of(j);
      } else if (ok_of(j)) {
        ++hb_ok;
      } else {
        EXPECT_EQ(code_of(j), "busy");
        ++hb_busy;
      }
    }
    return all_answered();
  });
  ASSERT_TRUE(all_answered()) << "E-stop answered " << answers.count(99) << ", heartbeats answered "
                              << hb_ok + hb_busy << "/" << n;
  // The E-stop was dispatched first, then ONE coalesced heartbeat.
  const auto& batch = r.gw->last_batch();
  ASSERT_EQ(batch.size(), 2U);
  EXPECT_EQ(batch[0], CmdKind::Estop);
  EXPECT_EQ(batch[1], CmdKind::Heartbeat);
  ASSERT_FALSE(r.calls.empty());
  EXPECT_EQ(r.calls.front(), "estop:1:tablet");
  EXPECT_TRUE(estop_ok);
  // Every heartbeat is answered exactly once, and the inbox never exceeded its cap: exactly
  // kInboxCap of them were queued (ok), the rest refused busy.
  EXPECT_EQ(answers.size(), static_cast<size_t>(n) + 1);
  for (const auto& [id, count] : answers) EXPECT_EQ(count, 1) << "id " << id;
  EXPECT_EQ(hb_ok, static_cast<int>(GatewayNode::kInboxCap));
  EXPECT_EQ(hb_busy, n - static_cast<int>(GatewayNode::kInboxCap));
  r.link_seen = false;
  r.gw->step(r.now + 0.1);
  ASSERT_TRUE(r.pump_until([&r] { return r.link_seen; }));
  EXPECT_TRUE(r.link.alive);  // the coalesced heartbeat still counts
}

TEST(GatewayNode, OperatorLinkFollowsTheHeartbeatAndFailsSafe) {
  Rig r;
  r.gw->step(r.now);
  r.deliver();
  ASSERT_TRUE(r.link_seen);
  EXPECT_FALSE(r.link.alive);  // boot: nobody has spoken
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  EXPECT_TRUE(ok_of(r.ask(c, R"({"v":1,"id":41,"cmd":"heartbeat"})", 41)));
  r.now += 0.2;
  r.gw->step(r.now);
  r.deliver();
  EXPECT_TRUE(r.link.alive);
  r.now += 1.0;  // within the 2 s timeout
  r.gw->step(r.now);
  r.deliver();
  EXPECT_TRUE(r.link.alive);
  r.now += 1.5;  // timeout
  r.gw->step(r.now);
  r.deliver();
  EXPECT_FALSE(r.link.alive);
  EXPECT_GT(r.link.age_s, 2.0F);
  EXPECT_TRUE(ok_of(r.ask(c, R"({"v":1,"id":42,"cmd":"heartbeat"})", 42)));
  r.now += 0.2;
  r.gw->step(r.now);
  r.deliver();
  EXPECT_TRUE(r.link.alive);
  // the backend disappears: no client, so not alive even though the last heartbeat is fresh
  {
    Sock gone(r.sock);
  }
  c.~Sock();
  new (&c) Sock("/nonexistent");
  ASSERT_TRUE(r.pump_until([&r] { return r.gw->ipc().clients() == 0; }));  // both hang-ups seen
  r.now += 0.1;
  r.gw->step(r.now);
  r.deliver();
  EXPECT_FALSE(r.link.alive);
}

TEST(GatewayNode, TheOperatorLinkDiesWithTheConnectionThatHeartbeated) {
  Rig r;
  auto a = std::make_unique<Sock>(r.sock);
  Sock b(r.sock);  // a second client (e.g. a debug tool) that never heartbeats
  ASSERT_TRUE(a->ok() && b.ok());
  EXPECT_TRUE(ok_of(r.ask(*a, R"({"v":1,"id":61,"cmd":"heartbeat"})", 61)));
  r.now += 0.1;
  r.gw->step(r.now);
  r.deliver();
  EXPECT_TRUE(r.link.alive);
  // A (the backend) goes away while B stays connected and silent: the link must drop on the
  // next publish, not after the heartbeat timeout.
  a.reset();
  r.pump_until([&r] { return r.gw->ipc().clients() == 1; });  // the IPC thread sees the hang-up
  ASSERT_EQ(r.gw->ipc().clients(), 1);
  r.now += 0.1;  // one publish period; the last heartbeat is only 0.2 s old
  r.gw->step(r.now);
  r.deliver();
  EXPECT_FALSE(r.link.alive);
  EXPECT_LT(r.link.age_s, 1.0F);
  // B is not the operator until it heartbeats itself
  EXPECT_TRUE(ok_of(r.ask(b, R"({"v":1,"id":62,"cmd":"heartbeat"})", 62)));
  r.now += 0.1;
  r.gw->step(r.now);
  r.deliver();
  EXPECT_TRUE(r.link.alive);
  r.now += 1.0;
  r.gw->step(r.now);
  r.deliver();
  EXPECT_TRUE(r.link.alive);  // within the timeout, B still connected
}

TEST(GatewayNode, SnapshotAndTelemetryPushCarryAgeAndFreshness) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  dyx3_interfaces::msg::RtkStatus m;
  m.fix_type = 6;
  m.horizontal_accuracy_m = 0.02F;
  r.p_rtk->publish(m);
  r.deliver();
  auto v = r.ask(c, R"({"v":1,"id":51,"cmd":"get_snapshot"})", 51);
  ASSERT_TRUE(ok_of(v));
  const JsonValue* d = v.get("data");
  ASSERT_NE(d, nullptr);
  EXPECT_EQ(d->get("rtk_status")->get("data")->get("fix_type")->i, 6);
  EXPECT_TRUE(d->get("rtk_status")->get("fresh")->b);
  EXPECT_EQ(d->get("vehicle_state")->type,
            JsonValue::Type::Null);  // never received: null, not a stale zero
  EXPECT_FALSE(d->get("gateway")->get("operator_alive")->b);
  // the IPC counters are exported for the audit trail (XR-GW-001)
  const JsonValue* ipc = d->get("gateway")->get("ipc");
  ASSERT_NE(ipc, nullptr);
  for (const char* k : {"dropped_slow", "overflows", "rejected_full"}) {
    ASSERT_NE(ipc->get(k), nullptr) << k;
    EXPECT_TRUE(ipc->get(k)->is_int) << k;
    EXPECT_EQ(ipc->get(k)->i, 0) << k;
  }
  {
    std::vector<std::unique_ptr<Sock>> extra;  // max_clients is 4: the 5th connection is refused
    for (int i = 0; i < 4; ++i) extra.push_back(std::make_unique<Sock>(r.sock));
    r.pump_until([&r] { return r.gw->ipc().rejected_full() >= 1; });
  }
  v = r.ask(c, R"({"v":1,"id":52,"cmd":"get_snapshot"})", 52);
  EXPECT_EQ(v.get("data")->get("gateway")->get("ipc")->get("rejected_full")->i, 1);
  // a telemetry push arrives without being asked; once the data is old it says so
  r.now += 5.0;
  r.gw->step(r.now);
  r.deliver();
  bool saw_stale = false;
  r.pump_until([&] {  // read pushes until the stale one arrives (bounded)
    for (const auto& l : c.read_lines(1, 20)) {
      JsonValue j;
      std::string e;
      if (parse_json(l, &j, &e) && j.get("type") && j.get("type")->s == "telemetry") {
        saw_stale = saw_stale || !j.get("snapshot")->get("rtk_status")->get("fresh")->b;
      }
    }
    return saw_stale;
  });
  EXPECT_TRUE(saw_stale);
}

// The telemetry frame carries its own counter and the gateway's steady clock, so a consumer can
// see a dropped or reordered frame (contract section 1). Protocol version stays 1.
TEST(GatewayNode, TelemetryFramesCarryIncreasingSeqAndSteadyStamp) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  ASSERT_TRUE(r.pump_until([&r] { return r.gw->ipc().clients() >= 1; }));
  constexpr size_t kFrames = 5;
  for (size_t i = 0; i < kFrames; ++i) {
    r.now += 0.2;  // longer than one telemetry period: every step pushes exactly one frame
    r.gw->step(r.now);
    r.deliver();
  }
  std::vector<int64_t> seqs;
  std::vector<double> stamps;
  std::vector<std::string> lines;
  lines.swap(c.stash);
  const auto end = std::chrono::steady_clock::now() + 5s;
  while (seqs.size() < kFrames && std::chrono::steady_clock::now() < end) {
    if (lines.empty()) lines = c.read_lines(1, 50);
    for (const auto& l : lines) {
      JsonValue j;
      std::string e;
      if (!parse_json(l, &j, &e) || !j.get("type") || j.get("type")->s != "telemetry") continue;
      EXPECT_EQ(j.get("v")->i, 1);
      ASSERT_NE(j.get("snapshot"), nullptr);
      ASSERT_NE(j.get("seq"), nullptr);
      EXPECT_TRUE(j.get("seq")->is_int);
      ASSERT_NE(j.get("t_mono_s"), nullptr);
      EXPECT_EQ(j.get("t_mono_s")->type, JsonValue::Type::Number);
      seqs.push_back(j.get("seq")->i);
      stamps.push_back(j.get("t_mono_s")->n);
    }
    lines.clear();
  }
  ASSERT_EQ(seqs.size(), kFrames);
  EXPECT_EQ(seqs.front(), 1);  // a separate counter, starting at 1 per gateway process
  for (size_t i = 1; i < kFrames; ++i) {
    EXPECT_EQ(seqs[i], seqs[i - 1] + 1) << "frame " << i;
    EXPECT_GE(stamps[i], stamps[i - 1]) << "frame " << i;
  }
  EXPECT_DOUBLE_EQ(stamps.back(), r.now);  // the clock the node runs on, as for event t_mono_s
}

TEST(GatewayNode, InvalidParametersStopTheNodeAtStart) {
  auto ctx = std::make_shared<rclcpp::Context>();
  dyx3_test::init_isolated(ctx);
  rclcpp::NodeOptions o;
  o.context(ctx);
  o.append_parameter_override("socket_path", tmp_sock());
  o.append_parameter_override("operator_link_timeout_s", -1.0);
  EXPECT_THROW(GatewayNode(o, []() { return 1.0; }, false), std::invalid_argument);
  rclcpp::NodeOptions p;
  p.context(ctx);
  p.append_parameter_override("socket_path", std::string("/nonexistent_dir_dyx3/g.sock"));
  EXPECT_THROW(GatewayNode(p, []() { return 1.0; }, false), std::runtime_error);
  ctx->shutdown("test done");
}

// ---- per-command timeouts, request ids ---------------------------------------------------------

TEST(GatewayNode, EachCommandHasItsOwnAnswerDeadline) {
  Rig r;
  // service_timeout_s is 1.0 in the rig; the others are the defaults.
  EXPECT_DOUBLE_EQ(r.gw->timeout_for(CmdKind::StartMission), 1.0);
  EXPECT_DOUBLE_EQ(r.gw->timeout_for(CmdKind::PauseMission), 1.0);
  EXPECT_DOUBLE_EQ(r.gw->timeout_for(CmdKind::Estop), 1.0);
  EXPECT_DOUBLE_EQ(r.gw->timeout_for(CmdKind::Arm), 4.0);
  EXPECT_DOUBLE_EQ(r.gw->timeout_for(CmdKind::Offboard), 5.0);
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  // px4_link is still confirming OFFBOARD (it answers within its own 3.5 s): the gateway must not
  // report a timeout before offboard_timeout_s.
  r.offboard_answers = false;
  c.write_all(R"({"v":1,"id":81,"cmd":"offboard","args":{"enable":true}})"
              "\n");
  ASSERT_TRUE(r.pump_until([&r] { return r.gw->inbox_depth() == 1U; }));
  const double t0 = r.now;
  r.gw->step(r.now);
  ASSERT_TRUE(r.pump_until([&r] { return !r.calls.empty(); }));
  for (const double t : {1.0, 3.5, 4.99}) {  // the old shared 2.0 s would have fired here
    r.now = t0 + t;
    r.gw->step(r.now);
    r.deliver();
    for (const auto& l : c.read_lines(1, 5)) EXPECT_EQ(l.find("\"id\":81"), std::string::npos) << l;
  }
  r.now = t0 + 5.0;
  r.gw->step(r.now);
  bool timed_out = false;
  r.pump_until([&] {
    for (const auto& l : c.read_lines(1, 5)) {
      JsonValue j;
      std::string e;
      if (parse_json(l, &j, &e) && j.get("id") && j.get("id")->i == 81) {
        timed_out = code_of(j) == "timeout";
        return true;
      }
    }
    return false;
  });
  EXPECT_TRUE(timed_out);
}

TEST(GatewayNode, StartMissionPassesResumeAndEchoesTheResumedRun) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  const std::string sha(64, 'c');
  const std::string head = R"(,"cmd":"start_mission","args":{"path_artifact_sha256":")" + sha;
  // resume true reaches StartMission.resume; the run the mission resumes from is in the reply
  auto v = r.ask(c, R"({"v":1,"id":81)" + head + R"(","resume":true}})", 81);
  ASSERT_TRUE(ok_of(v));
  EXPECT_TRUE(r.last_start_resume);
  EXPECT_EQ(v.get("data")->get("resumed_run_index")->i, 4);
  EXPECT_EQ(v.get("data")->get("mission_id")->i, 42);
  // resume false, and resume absent (default false), start at run 0
  v = r.ask(c, R"({"v":1,"id":82)" + head + R"(","resume":false}})", 82);
  ASSERT_TRUE(ok_of(v));
  EXPECT_FALSE(r.last_start_resume);
  EXPECT_EQ(v.get("data")->get("resumed_run_index")->i, 0);
  r.last_start_resume = true;
  v = r.ask(c, R"({"v":1,"id":83)" + head + R"("}})", 83);
  ASSERT_TRUE(ok_of(v));
  EXPECT_FALSE(r.last_start_resume);
  EXPECT_EQ(v.get("data")->get("resumed_run_index")->i, 0);
  // a non-boolean resume is refused at the gateway: nothing reaches the mission node
  size_t starts = 0;
  {
    std::lock_guard<std::mutex> lk(r.mu);
    starts = r.calls.size();
  }
  v = r.ask(c, R"({"v":1,"id":84)" + head + R"(","resume":1}})", 84);
  EXPECT_FALSE(ok_of(v));
  EXPECT_EQ(code_of(v), "invalid_command");
  r.deliver();
  std::lock_guard<std::mutex> lk(r.mu);
  EXPECT_EQ(r.calls.size(), starts);
}

TEST(GatewayNode, SnapshotCarriesTheRcLinkAndThePivotTimeout) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  dyx3_interfaces::msg::VehicleState vs;
  vs.rc_link_valid = true;
  vs.rc_link_ok = false;
  r.p_vehicle->publish(vs);
  dyx3_interfaces::msg::RppStatus rp;
  rp.pivot_timed_out = true;
  r.p_rpp->publish(rp);
  r.deliver();
  auto v = r.ask(c, R"({"v":1,"id":85,"cmd":"get_snapshot"})", 85);
  ASSERT_TRUE(ok_of(v));
  const JsonValue* veh = v.get("data")->get("vehicle_state")->get("data");
  ASSERT_NE(veh, nullptr);
  EXPECT_TRUE(veh->get("rc_link_valid")->b);
  EXPECT_FALSE(veh->get("rc_link_ok")->b);
  const JsonValue* rpp = v.get("data")->get("rpp")->get("data");
  ASSERT_NE(rpp, nullptr);
  EXPECT_TRUE(rpp->get("pivot_timed_out")->b);
  vs.rc_link_valid = false;
  vs.rc_link_ok = true;
  r.p_vehicle->publish(vs);
  rp.pivot_timed_out = false;
  r.p_rpp->publish(rp);
  r.deliver();
  v = r.ask(c, R"({"v":1,"id":86,"cmd":"get_snapshot"})", 86);
  ASSERT_TRUE(ok_of(v));
  veh = v.get("data")->get("vehicle_state")->get("data");
  ASSERT_NE(veh, nullptr);
  EXPECT_FALSE(veh->get("rc_link_valid")->b);
  EXPECT_TRUE(veh->get("rc_link_ok")->b);
  rpp = v.get("data")->get("rpp")->get("data");
  ASSERT_NE(rpp, nullptr);
  EXPECT_FALSE(rpp->get("pivot_timed_out")->b);
}

TEST(GatewayNode, StartMissionCarriesAndEchoesTheRequestId) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  const std::string sha(64, 'c');
  auto v = r.ask(c,
                 R"({"v":1,"id":91,"cmd":"start_mission","args":{"path_artifact_sha256":")" + sha +
                     R"(","request_id":"req-7f3a"}})",
                 91);
  ASSERT_TRUE(ok_of(v));
  EXPECT_EQ(v.get("data")->get("request_id")->s, "req-7f3a");
  EXPECT_EQ(v.get("data")->get("mission_id")->i, 42);
  EXPECT_EQ(r.last_start_request_id, "req-7f3a");  // passed to StartMission
  EXPECT_FALSE(v.get("data")->get("duplicate")->b);
  EXPECT_EQ(v.get("data")->get("gate_reason_code")->i, 0);
  // without a request id the reply carries none
  v = r.ask(
      c,
      R"({"v":1,"id":92,"cmd":"start_mission","args":{"path_artifact_sha256":")" + sha + R"("}})",
      92);
  ASSERT_TRUE(ok_of(v));
  EXPECT_EQ(v.get("data")->get("request_id"), nullptr);
  // a start that is never answered: the typed timeout still carries the request id
  r.start_answers = false;
  v = r.ask(c,
            R"({"v":1,"id":93,"cmd":"start_mission","args":{"path_artifact_sha256":")" + sha +
                R"(","request_id":"req-8"}})",
            93, 3.0);
  EXPECT_FALSE(ok_of(v));
  EXPECT_EQ(code_of(v), "timeout");
  EXPECT_EQ(v.get("data")->get("request_id")->s, "req-8");
  // a malformed request id is a typed error and never reaches ROS
  const size_t n = r.calls.size();
  v = r.ask(c,
            R"({"v":1,"id":94,"cmd":"start_mission","args":{"path_artifact_sha256":")" + sha +
                R"(","request_id":"no spaces"}})",
            94);
  EXPECT_EQ(code_of(v), "invalid_command");
  EXPECT_EQ(r.calls.size(), n);
}

TEST(GatewayNode, RequestIdIsRefusedAtTheGatewayBeyondSixtyFourCharacters) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  const std::string sha(64, 'c');
  // 64 characters (the bound mission and the backend share) pass and reach StartMission verbatim
  const std::string ok_id(64, 'r');
  auto v = r.ask(c,
                 R"({"v":1,"id":95,"cmd":"start_mission","args":{"path_artifact_sha256":")" + sha +
                     R"(","request_id":")" + ok_id + R"("}})",
                 95);
  EXPECT_TRUE(ok_of(v));
  EXPECT_EQ(r.last_start_request_id, ok_id);
  // 65 characters: a typed error at the gateway, nothing reaches ROS
  const size_t n = r.calls.size();
  v = r.ask(c,
            R"({"v":1,"id":96,"cmd":"start_mission","args":{"path_artifact_sha256":")" + sha +
                R"(","request_id":")" + std::string(65, 'r') + R"("}})",
            96);
  EXPECT_FALSE(ok_of(v));
  EXPECT_EQ(code_of(v), "invalid_command");
  EXPECT_EQ(r.calls.size(), n);
}

TEST(GatewayNode, StartReplyCarriesTheExecutionDuplicateAndGateReasonAndTypesAnInvalidRequest) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  const std::string start = R"({"v":1,"id":)";
  const std::string args = R"(,"cmd":"start_mission","args":{"path_artifact_sha256":")" +
                           std::string(64, 'c') + R"(","request_id":"dup-1"}})";
  // a duplicate is an accepted start that names the existing execution and starts nothing new
  r.start_duplicate = true;
  auto v = r.ask(c, start + "97" + args, 97);
  ASSERT_TRUE(ok_of(v));
  EXPECT_EQ(code_of(v), "ok");
  EXPECT_EQ(v.get("data")->get("mission_id")->i, 42);
  EXPECT_TRUE(v.get("data")->get("duplicate")->b);
  EXPECT_EQ(v.get("data")->get("gate_reason_code")->i, 0);
  EXPECT_EQ(v.get("data")->get("request_id")->s, "dup-1");
  // a refusal by the pre-arm gate names the failing guard gate, verbatim
  r.start_duplicate = false;
  r.start_accepts = false;
  r.start_refusal = srv::StartMission::Response::REASON_SAFETY_GATE;
  r.start_gate_reason = 6;
  v = r.ask(c, start + "98" + args, 98);
  EXPECT_FALSE(ok_of(v));
  EXPECT_EQ(code_of(v), "rejected");
  EXPECT_EQ(v.get("data")->get("reason_code")->i, srv::StartMission::Response::REASON_SAFETY_GATE);
  EXPECT_EQ(v.get("data")->get("gate_reason_code")->i, 6);
  EXPECT_FALSE(v.get("data")->get("duplicate")->b);
  // the mission node refusing the request id is a typed gateway error, its reason code kept
  r.start_refusal = srv::StartMission::Response::REASON_INVALID_REQUEST;
  r.start_gate_reason = 0;
  v = r.ask(c, start + "99" + args, 99);
  EXPECT_FALSE(ok_of(v));
  EXPECT_EQ(code_of(v), "invalid_command");
  EXPECT_EQ(v.get("data")->get("reason_code")->i,
            srv::StartMission::Response::REASON_INVALID_REQUEST);
  EXPECT_EQ(v.get("data")->get("request_id")->s, "dup-1");
  // every other refusal stays `rejected`
  r.start_refusal = srv::StartMission::Response::REASON_INVALID_ARTIFACT;
  v = r.ask(c, start + "100" + args, 100);
  EXPECT_EQ(code_of(v), "rejected");
}

// ---- pushed events -----------------------------------------------------------------------------

TEST(GatewayNode, MissionStateTransitionsArePushedAsOrderedCoalescedEvents) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  // Accepted before the first push: otherwise the event may also arrive as a replay (same seq).
  ASSERT_TRUE(r.pump_until([&r] { return r.gw->ipc().clients() == 1; }));
  r.p_mission->publish(mission(1));  // LOADING
  auto ev =
      of_kind(read_events(r, c, [](const auto& e) { return !of_kind(e, "mission_state").empty(); }),
              "mission_state");
  ASSERT_EQ(ev.size(), 1U);
  const JsonValue& e = ev[0];
  EXPECT_EQ(e.get("v")->i, 1);
  EXPECT_FALSE(e.get("replay")->b);
  EXPECT_EQ(e.get("coalesced")->i, 0);
  EXPECT_DOUBLE_EQ(e.get("t_mono_s")->n, r.now);
  EXPECT_GT(e.get("t_wall_ms")->i, 1600000000000);
  const JsonValue* d = e.get("data");
  EXPECT_EQ(d->get("state")->i, 1);
  EXPECT_EQ(d->get("reason_code")->i, 0);
  EXPECT_EQ(d->get("mission_id")->i, 42);
  EXPECT_EQ(d->get("path_artifact_sha256")->s, std::string(64, 'c'));
  EXPECT_TRUE(d->get("fresh")->b);
  ASSERT_NE(d->get("stamp_s"), nullptr);
  const int64_t seq1 = e.get("seq")->i;
  // the same state again (the mission node republishes at 10 Hz): no event
  r.p_mission->publish(mission(1, 0, 3));
  r.deliver();
  EXPECT_EQ(static_cast<int64_t>(r.gw->event_seq()), seq1);
  // a burst inside 10 ms (same instant here): READY then RUNNING folds into ONE event that carries
  // the latest state, pushed when the window has passed
  r.now += 0.001;
  r.p_mission->publish(mission(2));
  r.p_mission->publish(mission(3));
  r.deliver();
  EXPECT_EQ(static_cast<int64_t>(r.gw->event_seq()), seq1);  // waiting for the window
  r.now += 0.009;
  r.gw->step(r.now);
  ev =
      of_kind(read_events(r, c, [](const auto& x) { return !of_kind(x, "mission_state").empty(); }),
              "mission_state");
  ASSERT_EQ(ev.size(), 1U);
  EXPECT_EQ(ev[0].get("data")->get("state")->i, 3);
  EXPECT_EQ(ev[0].get("coalesced")->i, 1);
  EXPECT_GT(ev[0].get("seq")->i, seq1);
  // a new reason with the same state is a transition too
  r.now += 0.05;
  r.p_mission->publish(mission(3, 2));
  ev =
      of_kind(read_events(r, c, [](const auto& x) { return !of_kind(x, "mission_state").empty(); }),
              "mission_state");
  ASSERT_EQ(ev.size(), 1U);
  EXPECT_EQ(ev[0].get("data")->get("reason_code")->i, 2);
  // the mission node goes silent: once its state is no longer fresh, that is pushed as well
  r.now += 1.5;
  r.gw->step(r.now);
  ev =
      of_kind(read_events(r, c, [](const auto& x) { return !of_kind(x, "mission_state").empty(); }),
              "mission_state");
  ASSERT_EQ(ev.size(), 1U);
  EXPECT_FALSE(ev[0].get("data")->get("fresh")->b);
}

TEST(GatewayNode, MissionStateEventAndSnapshotCarryEveryV2Field) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  ASSERT_TRUE(r.pump_until([&r] { return r.gw->ipc().clients() == 1; }));
  const auto src = mission_v2();
  r.p_mission->publish(src);
  auto ev =
      of_kind(read_events(r, c, [](const auto& e) { return !of_kind(e, "mission_state").empty(); }),
              "mission_state");
  ASSERT_EQ(ev.size(), 1U);
  const auto check = [&](const JsonValue* d, const char* where) {
    SCOPED_TRACE(where);
    ASSERT_NE(d, nullptr);
    EXPECT_EQ(d->get("state")->i, dyx3_interfaces::msg::MissionState::STATE_PAUSED);
    EXPECT_EQ(d->get("mission_id")->i, 42);  // the execution id
    EXPECT_EQ(d->get("run_index")->i, 3);
    EXPECT_EQ(d->get("point_index")->i, 5);
    EXPECT_EQ(d->get("reason_code")->i, dyx3_interfaces::msg::MissionState::REASON_EKF_RESET);
    EXPECT_EQ(d->get("path_artifact_sha256")->s, std::string(64, 'e'));  // execution sha
    EXPECT_EQ(d->get("source_artifact_sha256")->s, std::string(64, 'a'));
    EXPECT_EQ(d->get("request_id")->s, "tab-1:9f2c");
    EXPECT_EQ(d->get("reason_detail")->s, src.reason_detail);
    EXPECT_EQ(d->get("gate_reason_code")->i, 7);
    EXPECT_EQ(d->get("waiting_on")->i, dyx3_interfaces::msg::MissionState::WAIT_OPERATOR);
    EXPECT_DOUBLE_EQ(d->get("state_entered")->n, 1791624580.5);
    EXPECT_EQ(d->get("start_run_index")->i, 2);
  };
  check(ev[0].get("data"), "event");
  EXPECT_TRUE(ev[0].get("data")->get("fresh")->b);
  const auto snap = r.ask(c, R"({"v":1,"id":61,"cmd":"get_snapshot"})", 61);
  ASSERT_TRUE(ok_of(snap));
  check(snap.get("data")->get("mission")->get("data"), "snapshot");
  // a state that never carried an id or detail reports them as empty strings, not as absent
  r.now += 0.05;
  r.p_mission->publish(mission(1));
  ev =
      of_kind(read_events(r, c, [](const auto& e) { return !of_kind(e, "mission_state").empty(); }),
              "mission_state");
  ASSERT_EQ(ev.size(), 1U);
  EXPECT_EQ(ev[0].get("data")->get("request_id")->s, "");
  EXPECT_EQ(ev[0].get("data")->get("source_artifact_sha256")->s, "");
  EXPECT_EQ(ev[0].get("data")->get("reason_detail")->s, "");
  EXPECT_EQ(ev[0].get("data")->get("waiting_on")->i, 0);
  EXPECT_EQ(ev[0].get("data")->get("start_run_index")->i, 0);
}

TEST(GatewayNode, ChangeOfTheStepBeingWaitedOnIsATransitionEvenWithStateAndReasonUnchanged) {
  using M = dyx3_interfaces::msg::MissionState;
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  ASSERT_TRUE(r.pump_until([&r] { return r.gw->ipc().clients() == 1; }));
  const auto next_event = [&] {
    return of_kind(
        read_events(
            r, c, [](const auto& e) { return !of_kind(e, "mission_state").empty(); }, 500),
        "mission_state");
  };
  auto m = mission(M::STATE_ERROR, M::REASON_ARM_TIMEOUT);
  m.waiting_on = M::WAIT_OFFBOARD_RELEASE;  // a release step is pending
  r.p_mission->publish(m);
  auto ev = next_event();
  ASSERT_EQ(ev.size(), 1U);
  EXPECT_EQ(ev[0].get("data")->get("waiting_on")->i, M::WAIT_OFFBOARD_RELEASE);
  const int64_t seq1 = ev[0].get("seq")->i;
  // the same publication again (10 Hz republish): no event
  r.now += 0.05;
  r.p_mission->publish(m);
  r.deliver();
  EXPECT_EQ(static_cast<int64_t>(r.gw->event_seq()), seq1);
  // the release moves on to the disarm: same state, same reason, new step -> an event
  r.now += 0.05;
  m.waiting_on = M::WAIT_DISARM;
  r.p_mission->publish(m);
  ev = next_event();
  ASSERT_EQ(ev.size(), 1U);
  EXPECT_GT(ev[0].get("seq")->i, seq1);
  EXPECT_EQ(ev[0].get("data")->get("state")->i, M::STATE_ERROR);
  EXPECT_EQ(ev[0].get("data")->get("reason_code")->i, M::REASON_ARM_TIMEOUT);
  EXPECT_EQ(ev[0].get("data")->get("waiting_on")->i, M::WAIT_DISARM);
  // the release finished: waiting_on NONE is the last transition
  r.now += 0.05;
  m.waiting_on = M::WAIT_NONE;
  r.p_mission->publish(m);
  ev = next_event();
  ASSERT_EQ(ev.size(), 1U);
  EXPECT_EQ(ev[0].get("data")->get("waiting_on")->i, M::WAIT_NONE);
}

TEST(GatewayNode, SafetyGateSnapshotCarriesThePreArmVerdictAndItsReason) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  dyx3_interfaces::msg::SafetyGateStatus g;
  g.ok = false;
  g.reason_code = 9;
  g.pre_arm_ok = true;  // everything but armed / OFFBOARD is fine
  g.pre_arm_reason_code = 0;
  r.p_safety->publish(g);
  r.deliver();
  auto v = r.ask(c, R"({"v":1,"id":62,"cmd":"get_snapshot"})", 62);
  ASSERT_TRUE(ok_of(v));
  const JsonValue* d = v.get("data")->get("safety_gate")->get("data");
  ASSERT_NE(d, nullptr);
  EXPECT_FALSE(d->get("ok")->b);
  EXPECT_EQ(d->get("reason_code")->i, 9);
  EXPECT_TRUE(d->get("pre_arm_ok")->b);
  EXPECT_EQ(d->get("pre_arm_reason_code")->i, 0);
  g.pre_arm_ok = false;
  g.pre_arm_reason_code = 5;
  r.p_safety->publish(g);
  r.deliver();
  v = r.ask(c, R"({"v":1,"id":63,"cmd":"get_snapshot"})", 63);
  d = v.get("data")->get("safety_gate")->get("data");
  EXPECT_FALSE(d->get("pre_arm_ok")->b);
  EXPECT_EQ(d->get("pre_arm_reason_code")->i, 5);
}

TEST(GatewayNode, LinkAndEstopChangesArePushedAndANewClientGetsTheCurrentStateAtOnce) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  // Accepted before the first push: otherwise the event may also arrive as a replay (same seq).
  ASSERT_TRUE(r.pump_until([&r] { return r.gw->ipc().clients() == 1; }));
  // boot: nobody has heartbeated yet; the operator link starts as "never"
  r.gw->step(r.now);
  auto ev = read_events(r, c, [](const auto& e) { return !of_kind(e, "operator_link").empty(); });
  auto ol = of_kind(ev, "operator_link");
  ASSERT_EQ(ol.size(), 1U);
  EXPECT_FALSE(ol[0].get("data")->get("alive")->b);
  EXPECT_EQ(ol[0].get("data")->get("cause")->s, "never");
  // a heartbeat makes it alive in the same step
  r.now += 0.05;
  EXPECT_TRUE(ok_of(r.ask(c, R"({"v":1,"id":101,"cmd":"heartbeat"})", 101)));
  ol =
      of_kind(read_events(r, c, [](const auto& e) { return !of_kind(e, "operator_link").empty(); }),
              "operator_link");
  ASSERT_EQ(ol.size(), 1U) << describe(ol);
  EXPECT_TRUE(ol[0].get("data")->get("alive")->b);
  // E-stop asserted, FCU session up then lost
  dyx3_interfaces::msg::EmergencyStopState es;
  es.asserted = true;
  es.source = "ble";
  r.p_estop->publish(es);
  dyx3_interfaces::msg::Px4LinkStatus px;
  px.session_alive = true;
  px.handshake_ok = true;
  r.p_px4->publish(px);
  ev = read_events(r, c, [](const auto& e) {
    return !of_kind(e, "estop").empty() && !of_kind(e, "fcu_link").empty();
  });
  ASSERT_EQ(of_kind(ev, "estop").size(), 1U);
  EXPECT_TRUE(of_kind(ev, "estop")[0].get("data")->get("asserted")->b);
  EXPECT_EQ(of_kind(ev, "estop")[0].get("data")->get("source")->s, "ble");
  EXPECT_TRUE(of_kind(ev, "fcu_link")[0].get("data")->get("session_alive")->b);
  r.now += 0.05;
  px.session_alive = false;
  r.p_px4->publish(px);
  auto fcu = of_kind(
      read_events(r, c, [](const auto& e) { return !of_kind(e, "fcu_link").empty(); }), "fcu_link");
  ASSERT_EQ(fcu.size(), 1U);
  EXPECT_FALSE(fcu[0].get("data")->get("session_alive")->b);
  // the heartbeating connection closes: operator link lost, cause given
  r.p_mission->publish(mission(3));
  r.deliver();
  {
    Sock other(r.sock);  // a second client heartbeats, then goes away
    ASSERT_TRUE(other.ok());
    r.now += 0.05;
    EXPECT_TRUE(ok_of(r.ask(other, R"({"v":1,"id":102,"cmd":"heartbeat"})", 102)));
  }
  ASSERT_TRUE(r.pump_until([&r] { return r.gw->ipc().clients() == 1; }));
  r.now += 0.05;
  r.gw->step(r.now);
  ol = of_kind(read_events(r, c,
                           [](const auto& e) {
                             for (const auto& x : of_kind(e, "operator_link"))
                               if (!x.get("data")->get("alive")->b) return true;
                             return false;
                           }),
               "operator_link");
  ASSERT_FALSE(ol.empty());
  EXPECT_EQ(ol.back().get("data")->get("cause")->s, "connection_closed");
  const uint64_t seq_now = r.gw->event_seq();
  // a client that connects now gets the latest event of every kind at once, without polling
  Sock late(r.sock);
  ASSERT_TRUE(late.ok());
  const auto replay = read_events(r, late, [](const auto& e) { return e.size() >= 4U; });
  ASSERT_EQ(replay.size(), 4U);
  int64_t prev = 0;
  for (const auto& x : replay) {
    EXPECT_TRUE(x.get("replay")->b);
    EXPECT_GT(x.get("seq")->i, prev);  // in sequence order
    prev = x.get("seq")->i;
  }
  EXPECT_FALSE(of_kind(replay, "operator_link")[0].get("data")->get("alive")->b);
  EXPECT_TRUE(of_kind(replay, "estop")[0].get("data")->get("asserted")->b);
  EXPECT_FALSE(of_kind(replay, "fcu_link")[0].get("data")->get("session_alive")->b);
  EXPECT_EQ(of_kind(replay, "mission_state")[0].get("data")->get("state")->i, 3);
  EXPECT_EQ(r.gw->event_seq(), seq_now);  // a replay is not a new event
}

// ---- isolation, latency (autonomous rig: real clock, executor on its own thread) --------------

TEST(GatewayNode, AnEstopDuringAPendingOffboardWaitIsDispatchedAndAnsweredAtOnce) {
  Rig r(true);
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  r.offboard_answers = false;  // px4_link is still confirming: the gateway waits up to 5 s
  c.write_all(R"({"v":1,"id":111,"cmd":"offboard","args":{"enable":true}})"
              "\n");
  const auto end = std::chrono::steady_clock::now() + 5s;
  for (;;) {
    {
      std::lock_guard<std::mutex> lk(r.mu);
      if (!r.calls.empty()) break;
    }
    ASSERT_LT(std::chrono::steady_clock::now(), end) << "offboard never reached px4_link";
    std::this_thread::sleep_for(1ms);
  }
  const double t0 = steady_s();
  c.write_all(R"({"v":1,"id":112,"cmd":"estop","args":{"asserted":true,"source":"tablet"}})"
              "\n");
  const JsonValue v = await_reply(c, 112);
  const double t_reply = steady_s();
  ASSERT_NE(v.get("id"), nullptr) << "no E-stop reply";
  EXPECT_TRUE(ok_of(v));
  double rx;
  {
    std::lock_guard<std::mutex> lk(r.mu);
    ASSERT_EQ(r.estop_rx_s.size(), 1U);
    rx = r.estop_rx_s[0];
  }
  std::printf("[ INFO ] E-stop during pending offboard: line->guard %.3f ms, line->reply %.3f ms\n",
              (rx - t0) * 1e3, (t_reply - t0) * 1e3);
  // Far below the 5 s offboard wait; the bound only catches a dispatch that waited on it.
  EXPECT_LT(t_reply - t0, 0.5);
  // ... and the offboard request is still waiting for its own answer
  for (const auto& l : c.read_lines(1, 20)) EXPECT_EQ(l.find("\"id\":111"), std::string::npos) << l;
}

TEST(GatewayNode, LatencyCommandToServiceRequestAndMissionStateToEventLine) {
  Rig r(true);
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  constexpr int kN = 50;
  // 1. a command line -> the ROS service request (the gateway's own measurement)
  for (int i = 0; i < kN; ++i) {
    c.write_all(R"({"v":1,"id":)" + std::to_string(200 + i) + R"(,"cmd":"pause_mission"})" + "\n");
    ASSERT_TRUE(ok_of(await_reply(c, 200 + i))) << i;
  }
  // 2. MissionState publish -> its event line read on the socket
  std::vector<double> ev_ms;
  for (int i = 0; i < kN; ++i) {
    std::this_thread::sleep_for(
        12ms);  // past the 10 ms coalescing window: each one is a leading edge
    const double t0 = steady_s();
    r.p_mission->publish(mission(static_cast<uint8_t>(2 + i % 2)));
    bool got = false;
    const auto end = std::chrono::steady_clock::now() + 2s;
    while (!got && std::chrono::steady_clock::now() < end) {
      for (const auto& l : c.read_lines(1, 5)) {
        JsonValue j;
        std::string e;
        if (parse_json(l, &j, &e) && j.get("type") && j.get("type")->s == "event" &&
            j.get("event")->s == "mission_state") {
          got = true;
        }
      }
    }
    ASSERT_TRUE(got) << "no event for publish " << i;
    ev_ms.push_back((steady_s() - t0) * 1e3);
  }
  r.exec->cancel();  // stop the executor before reading the gateway's counters
  r.spinner.join();
  const auto& st = r.gw->dispatch_stats();
  ASSERT_GE(st.count, static_cast<uint64_t>(kN));
  const double dispatch_mean_ms = st.sum_us / static_cast<double>(st.count) / 1e3;
  std::printf(
      "[ INFO ] command line -> service request: mean %.3f ms, max %.3f ms (%llu commands)\n"
      "[ INFO ] MissionState publish -> event line: p50 %.3f ms, p95 %.3f ms, max %.3f ms\n",
      dispatch_mean_ms, st.max_us / 1e3, static_cast<unsigned long long>(st.count),
      percentile(ev_ms, 0.5), percentile(ev_ms, 0.95), percentile(ev_ms, 1.0));
  // Targets (contract section 7). The dispatch mean also proves the wake-up: on the 10 ms tick
  // alone it would average about 5 ms.
  EXPECT_LT(dispatch_mean_ms, 2.0);
  EXPECT_LT(percentile(ev_ms, 0.5), 5.0);
}

TEST(GatewayNode, ASlowReaderIsDroppedAndNeverHoldsUpEventsToTheOtherClient) {
  Rig r;
  Sock slow(r.sock);  // never reads
  Sock fast(r.sock);
  ASSERT_TRUE(slow.ok() && fast.ok());
  ASSERT_TRUE(r.pump_until([&r] { return r.gw->ipc().clients() == 2; }));
  // Telemetry at every step fills the slow client's socket and then its 1 MiB out buffer, while
  // mission transitions keep coming. The fast client must see every one of them, as they happen.
  int published = 0, received = 0;
  std::vector<double> ev_ms;
  for (int i = 0; i < 5000 && r.gw->ipc().dropped_slow() == 0; ++i) {
    r.now += 0.2;  // one telemetry period
    if (i % 25 == 0) {
      const double t0 = steady_s();
      r.p_mission->publish(mission(static_cast<uint8_t>(2 + published % 2)));
      ++published;
      r.deliver();
      const auto ev = read_events(r, fast, [](const auto& e) { return !e.empty(); }, 2000);
      ASSERT_EQ(of_kind(ev, "mission_state").size(), 1U) << "event " << published;
      ev_ms.push_back((steady_s() - t0) * 1e3);
      ++received;
    }
    r.gw->step(r.now);
    fast.read_lines(1000, 1);  // keeps up with the telemetry
  }
  EXPECT_GE(r.gw->ipc().dropped_slow(), 1U);
  ASSERT_TRUE(r.pump_until([&r] { return r.gw->ipc().clients() == 1; }));  // only the slow one went
  EXPECT_TRUE(slow.closed_by_peer());
  EXPECT_EQ(received, published);
  // still flowing to the fast client after the drop
  r.now += 0.2;
  r.p_mission->publish(mission(5));
  const auto ev =
      read_events(r, fast, [](const auto& e) { return !of_kind(e, "mission_state").empty(); });
  ASSERT_EQ(of_kind(ev, "mission_state").size(), 1U);
  EXPECT_EQ(of_kind(ev, "mission_state")[0].get("data")->get("state")->i, 5);
  std::printf(
      "[ INFO ] %d events to the fast client while the slow one filled up: p50 %.3f ms, "
      "max %.3f ms\n",
      published, percentile(ev_ms, 0.5), percentile(ev_ms, 1.0));
}

TEST(GatewayNode, InvalidTimeoutParametersStopTheNodeAtStart) {
  auto ctx = std::make_shared<rclcpp::Context>();
  dyx3_test::init_isolated(ctx);
  const auto build = [&](const char* name, double v) {
    rclcpp::NodeOptions o;
    o.context(ctx);
    o.append_parameter_override("socket_path", tmp_sock());
    o.append_parameter_override(name, v);
    return GatewayNode(o, []() { return 1.0; }, false);
  };
  EXPECT_THROW(build("arm_timeout_s", 0.0), std::invalid_argument);
  EXPECT_THROW(build("offboard_timeout_s", 31.0), std::invalid_argument);
  EXPECT_THROW(build("estop_timeout_s", 2.5), std::invalid_argument);  // slower than a fast accept
  EXPECT_THROW(build("event_coalesce_s", -0.01), std::invalid_argument);
  EXPECT_NO_THROW(build("offboard_timeout_s", 6.0));
  ctx->shutdown("test done");
}
