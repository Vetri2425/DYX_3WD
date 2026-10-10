// In-process tests of the gateway node over a REAL Unix socket: fake services stand in for the
// mission node, motion_guard, px4_link and spray; an injected clock; a private DDS domain.
// No fixed wall-clock waits: DDS delivery is drained (dds_test_support.hpp) and anything owned by
// the gateway's IPC thread (the inbox, client connections, socket replies) is waited on as a
// condition with a bound that only limits a failure.
#include "dyx3_system_gateway/gateway_node.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <thread>

#include "dds_test_support.hpp"
#include "dyx3_system_gateway/json.hpp"
#include "sock_client.hpp"

using namespace dyx3_gateway;
using namespace std::chrono_literals;
namespace srv = dyx3_interfaces::srv;

namespace {

struct Rig {
  std::shared_ptr<rclcpp::Context> ctx;
  double now{50.0};
  std::string sock;
  std::shared_ptr<GatewayNode> gw;
  std::shared_ptr<rclcpp::Node> world;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
  rclcpp::Subscription<dyx3_interfaces::msg::OperatorLinkStatus>::SharedPtr s_link;
  rclcpp::Publisher<dyx3_interfaces::msg::RtkStatus>::SharedPtr p_rtk;
  dyx3_interfaces::msg::OperatorLinkStatus link;
  bool link_seen{false};
  // fake services
  bool estop_answers{true}, estop_accepts{true}, start_accepts{true};
  std::vector<std::string> calls;
  rclcpp::ServiceBase::SharedPtr svc_keep[9];

  Rig() {
    ctx = std::make_shared<rclcpp::Context>();
    dyx3_test::init_isolated(ctx);
    sock = tmp_sock();
    rclcpp::NodeOptions go;
    go.context(ctx);
    go.append_parameter_override("socket_path", sock);
    go.append_parameter_override("service_timeout_s", 1.0);
    gw = std::make_shared<GatewayNode>(go, [this]() { return now; }, false);
    rclcpp::NodeOptions wo;
    wo.context(ctx);
    world = std::make_shared<rclcpp::Node>("world", wo);
    rclcpp::ExecutorOptions eo;
    eo.context = ctx;
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(eo);
    exec->add_node(gw);
    exec->add_node(world);

    svc_keep[0] = world->create_service<srv::StartMission>(
        "/dyx3/mission/start", [this](const std::shared_ptr<srv::StartMission::Request> rq,
                                      std::shared_ptr<srv::StartMission::Response> rs) {
          calls.push_back("start:" + rq->path_artifact_sha256.substr(0, 4));
          rs->accepted = start_accepts;
          rs->reason_code = start_accepts ? 0 : srv::StartMission::Response::REASON_BUSY;
          rs->mission_id = 42;
        });
    svc_keep[1] = world->create_service<srv::AbortMission>(
        "/dyx3/mission/abort", [this](const std::shared_ptr<srv::AbortMission::Request> rq,
                                      std::shared_ptr<srv::AbortMission::Response> rs) {
          calls.push_back("abort:" + std::to_string(rq->reason_code));
          rs->accepted = true;
        });
    svc_keep[2] = world->create_service<srv::PauseMission>(
        "/dyx3/mission/pause", [this](const std::shared_ptr<srv::PauseMission::Request>,
                                      std::shared_ptr<srv::PauseMission::Response> rs) {
          calls.push_back("pause");
          rs->accepted = true;
        });
    svc_keep[3] = world->create_service<srv::ResumeMission>(
        "/dyx3/mission/resume", [this](const std::shared_ptr<srv::ResumeMission::Request>,
                                       std::shared_ptr<srv::ResumeMission::Response> rs) {
          calls.push_back("resume");
          rs->accepted = true;
        });
    svc_keep[4] = world->create_service<srv::SkipPoint>(
        "/dyx3/mission/skip_point", [this](const std::shared_ptr<srv::SkipPoint::Request>,
                                           std::shared_ptr<srv::SkipPoint::Response> rs) {
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
          calls.push_back(std::string("estop:") + (rq->asserted ? "1" : "0") + ":" + rq->source);
          if (!estop_answers) return;
          srv::SetEmergencyStop::Response rs;
          rs.accepted = estop_accepts;
          svc->send_response(*hdr, rs);
        });
    svc_keep[6] = world->create_service<srv::ArmDisarm>(
        "/dyx3/px4_link/arm", [this](const std::shared_ptr<srv::ArmDisarm::Request> rq,
                                     std::shared_ptr<srv::ArmDisarm::Response> rs) {
          calls.push_back(std::string("arm:") + (rq->arm ? "1" : "0"));
          rs->accepted = false;
          rs->reason_code = srv::ArmDisarm::Response::REASON_LINK_UNHEALTHY;
        });
    svc_keep[7] = world->create_service<srv::SetOffboard>(
        "/dyx3/px4_link/set_offboard", [this](const std::shared_ptr<srv::SetOffboard::Request>,
                                              std::shared_ptr<srv::SetOffboard::Response> rs) {
          calls.push_back("offboard");
          rs->accepted = true;
        });
    svc_keep[8] = world->create_service<srv::SetSprayManual>(
        "/dyx3/spray/set_manual", [this](const std::shared_ptr<srv::SetSprayManual::Request> rq,
                                         std::shared_ptr<srv::SetSprayManual::Response> rs) {
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
    // Discovery: both topics matched and every one of the gateway's nine clients reaches its
    // service (a client that does not would answer "service_unavailable").
    const bool discovered = pump_until([this] {
      return gw->count_subscribers("/dyx3/operator_link") > 0 &&
             world->count_subscribers("/dyx3/rtk_status") > 0 && gw->unavailable_services().empty();
    });
    if (!discovered) {
      std::string missing;
      for (const auto& n : gw->unavailable_services()) missing += " " + n;
      ADD_FAILURE() << "DDS discovery did not complete; unreachable:" << missing;
    }
    // every later deliver() relies on synchronous delivery: prove it
    EXPECT_TRUE(dyx3_test::delivery_is_synchronous(ctx));
  }
  ~Rig() {
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
  // Send one line, drive the node until a reply with this id arrives. The first 300 steps spread
  // `advance` of link time; past them the clock stays put and only a wall-clock bound remains,
  // which limits a failure (a reply that never comes) and decides nothing.
  JsonValue ask(const Sock& s, const std::string& line, int64_t id, double advance = 0.0) {
    s.write_all(line + "\n");
    JsonValue v;
    const auto end = std::chrono::steady_clock::now() + 15s;
    for (int i = 0; i < 300 || std::chrono::steady_clock::now() < end; ++i) {
      deliver();
      if (i < 300) now += advance / 300.0;
      gw->step(now);
      for (const auto& l : s.read_lines(1, 5)) {
        JsonValue j;
        std::string e;
        if (parse_json(l, &j, &e) && j.get("id") && j.get("id")->is_int && j.get("id")->i == id)
          return j;
      }
    }
    ADD_FAILURE() << "no reply to " << line;
    return v;
  }
};

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
