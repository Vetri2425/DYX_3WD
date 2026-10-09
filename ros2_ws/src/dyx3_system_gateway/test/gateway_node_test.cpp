// In-process tests of the gateway node over a REAL Unix socket: fake services stand in for the
// mission node, motion_guard, px4_link and spray; an injected clock; a private DDS domain.
#include "dyx3_system_gateway/gateway_node.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <thread>

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
    rclcpp::InitOptions io;
    io.set_domain_id(120 + (getpid() % 100));
    ctx->init(0, nullptr, io);
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
    svc_keep[5] = world->create_service<srv::SetEmergencyStop>(
        "/dyx3/motion_guard/set_emergency_stop",
        [this](const std::shared_ptr<srv::SetEmergencyStop::Request> rq,
               std::shared_ptr<srv::SetEmergencyStop::Response> rs) {
          calls.push_back(std::string("estop:") + (rq->asserted ? "1" : "0") + ":" + rq->source);
          rs->accepted = estop_accepts;
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
    const auto end = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < end) {
      exec->spin_some(5ms);
      if (gw->count_subscribers("/dyx3/operator_link") > 0 &&
          world->count_subscribers("/dyx3/rtk_status") > 0) {
        // services discovered by the gateway's clients
        bool all = true;
        for (const char* n : {"/dyx3/mission/start", "/dyx3/motion_guard/set_emergency_stop",
                              "/dyx3/px4_link/arm", "/dyx3/spray/set_manual"}) {
          all = all && !gw->get_service_names_and_types_by_node("world", "/").empty();
          (void)n;
        }
        if (all) break;
      }
    }
    pump(800);  // let the clients see the services
  }
  ~Rig() {
    exec.reset();
    s_link.reset();
    gw.reset();
    world.reset();
    ctx->shutdown("test done");
  }
  void pump(int ms) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) exec->spin_some(2ms);
  }
  // Send one line, drive the node until a reply with this id arrives.
  JsonValue ask(const Sock& s, const std::string& line, int64_t id, double advance = 0.0) {
    s.write_all(line + "\n");
    std::vector<std::string> lines;
    JsonValue v;
    for (int i = 0; i < 300; ++i) {
      pump(10);
      now += advance / 300.0;
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
  r.pump(100);
  const auto lines = c.read_lines(1);
  ASSERT_EQ(lines.size(), 1U);
  EXPECT_NE(lines[0].find("bad_message"), std::string::npos);
  EXPECT_TRUE(r.calls.empty());
}

TEST(GatewayNode, AnEstopThatCannotBeDeliveredIsNeverReportedAsAccepted) {
  Rig r;
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  // 1. delivered and accepted
  auto v =
      r.ask(c, R"({"v":1,"id":21,"cmd":"estop","args":{"asserted":true,"source":"tablet"}})", 21);
  EXPECT_TRUE(ok_of(v));
  EXPECT_EQ(r.calls.back(), "estop:1:tablet");
  // 2. the service answers late/never: a timeout, reported as failed
  r.svc_keep[5].reset();  // the guard's service disappears
  r.pump(300);
  v = r.ask(c, R"({"v":1,"id":22,"cmd":"estop","args":{"asserted":true,"source":"tablet"}})", 22);
  EXPECT_FALSE(ok_of(v));
  EXPECT_TRUE(code_of(v) == "service_unavailable" || code_of(v) == "timeout") << code_of(v);
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
  r.pump(200);
  r.gw->step(r.now);
  r.pump(300);
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
  r.pump(500);  // the IPC thread queues the whole burst; no step runs meanwhile
  r.gw->step(r.now);
  r.pump(300);
  const auto& batch = r.gw->last_batch();
  ASSERT_EQ(batch.size(), 2U);  // the E-stop, then ONE coalesced heartbeat
  EXPECT_EQ(batch[0], CmdKind::Estop);
  EXPECT_EQ(batch[1], CmdKind::Heartbeat);
  ASSERT_FALSE(r.calls.empty());
  EXPECT_EQ(r.calls.front(), "estop:1:tablet");
  int hb_ok = 0, hb_busy = 0;
  bool estop_ok = false;
  for (const auto& l : c.read_lines(n + 1, 5000)) {
    JsonValue j;
    std::string e;
    ASSERT_TRUE(parse_json(l, &j, &e)) << e;
    if (!j.get("id")) continue;  // telemetry
    if (j.get("id")->i == 99) {
      estop_ok = ok_of(j);
    } else if (ok_of(j)) {
      ++hb_ok;
    } else {
      EXPECT_EQ(code_of(j), "busy");
      ++hb_busy;
    }
  }
  EXPECT_TRUE(estop_ok);
  EXPECT_EQ(hb_ok + hb_busy, n);                               // every heartbeat is answered
  EXPECT_LE(hb_ok, static_cast<int>(GatewayNode::kInboxCap));  // the inbox never exceeded its cap
  EXPECT_GT(hb_busy, 0);
  r.gw->step(r.now + 0.1);
  r.pump(200);
  EXPECT_TRUE(r.link.alive);  // the coalesced heartbeat still counts
}

TEST(GatewayNode, OperatorLinkFollowsTheHeartbeatAndFailsSafe) {
  Rig r;
  r.gw->step(r.now);
  r.pump(200);
  ASSERT_TRUE(r.link_seen);
  EXPECT_FALSE(r.link.alive);  // boot: nobody has spoken
  Sock c(r.sock);
  ASSERT_TRUE(c.ok());
  EXPECT_TRUE(ok_of(r.ask(c, R"({"v":1,"id":41,"cmd":"heartbeat"})", 41)));
  r.now += 0.2;
  r.gw->step(r.now);
  r.pump(200);
  EXPECT_TRUE(r.link.alive);
  r.now += 1.0;  // within the 2 s timeout
  r.gw->step(r.now);
  r.pump(200);
  EXPECT_TRUE(r.link.alive);
  r.now += 1.5;  // timeout
  r.gw->step(r.now);
  r.pump(200);
  EXPECT_FALSE(r.link.alive);
  EXPECT_GT(r.link.age_s, 2.0F);
  EXPECT_TRUE(ok_of(r.ask(c, R"({"v":1,"id":42,"cmd":"heartbeat"})", 42)));
  r.now += 0.2;
  r.gw->step(r.now);
  r.pump(200);
  EXPECT_TRUE(r.link.alive);
  // the backend disappears: no client, so not alive even though the last heartbeat is fresh
  {
    Sock gone(r.sock);
  }
  c.~Sock();
  new (&c) Sock("/nonexistent");
  r.pump(300);
  r.now += 0.1;
  r.gw->step(r.now);
  r.pump(200);
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
  r.pump(200);
  EXPECT_TRUE(r.link.alive);
  // A (the backend) goes away while B stays connected and silent: the link must drop on the
  // next publish, not after the heartbeat timeout.
  a.reset();
  r.pump(300);  // the IPC thread sees the hang-up
  ASSERT_EQ(r.gw->ipc().clients(), 1);
  r.now += 0.1;  // one publish period; the last heartbeat is only 0.2 s old
  r.gw->step(r.now);
  r.pump(200);
  EXPECT_FALSE(r.link.alive);
  EXPECT_LT(r.link.age_s, 1.0F);
  // B is not the operator until it heartbeats itself
  EXPECT_TRUE(ok_of(r.ask(b, R"({"v":1,"id":62,"cmd":"heartbeat"})", 62)));
  r.now += 0.1;
  r.gw->step(r.now);
  r.pump(200);
  EXPECT_TRUE(r.link.alive);
  r.now += 1.0;
  r.gw->step(r.now);
  r.pump(200);
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
  r.pump(300);
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
    for (int i = 0; i < 100 && r.gw->ipc().rejected_full() < 1; ++i) r.pump(10);
  }
  v = r.ask(c, R"({"v":1,"id":52,"cmd":"get_snapshot"})", 52);
  EXPECT_EQ(v.get("data")->get("gateway")->get("ipc")->get("rejected_full")->i, 1);
  // a telemetry push arrives without being asked; once the data is old it says so
  r.now += 5.0;
  r.gw->step(r.now);
  r.pump(100);
  bool saw_stale = false;
  for (const auto& l : c.read_lines(20, 300)) {
    JsonValue j;
    std::string e;
    if (parse_json(l, &j, &e) && j.get("type") && j.get("type")->s == "telemetry") {
      saw_stale = saw_stale || !j.get("snapshot")->get("rtk_status")->get("fresh")->b;
    }
  }
  EXPECT_TRUE(saw_stale);
}

TEST(GatewayNode, InvalidParametersStopTheNodeAtStart) {
  auto ctx = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions io;
  io.set_domain_id(120 + (getpid() % 100));
  ctx->init(0, nullptr, io);
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
