#include <gtest/gtest.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>

#include "dyx3_system_gateway/command_validator.hpp"
#include "dyx3_system_gateway/ipc_server.hpp"
#include "dyx3_system_gateway/json.hpp"
#include "dyx3_system_gateway/operator_link.hpp"
#include "dyx3_system_gateway/telemetry_snapshot.hpp"
#include "sock_client.hpp"

using namespace dyx3_gateway;

// ---- JSON parser
// ---------------------------------------------------------------------------------------------------
namespace {
bool parses(const std::string& s) {
  JsonValue v;
  std::string e;
  return parse_json(s, &v, &e);
}
}  // namespace

TEST(Json, AcceptsValidDocuments) {
  JsonValue v;
  std::string e;
  ASSERT_TRUE(parse_json(R"({"a":1,"b":[true,false,null,"x\u00e9\n"],"c":{"d":-2.5e3}})", &v, &e))
      << e;
  EXPECT_EQ(v.get("a")->i, 1);
  EXPECT_TRUE(v.get("a")->is_int);
  EXPECT_EQ(v.get("b")->a[3].s, "x\xC3\xA9\n");
  EXPECT_DOUBLE_EQ(v.get("c")->get("d")->n, -2500.0);
  EXPECT_FALSE(v.get("c")->get("d")->is_int);
  ASSERT_TRUE(parse_json(R"("\ud83d\ude00")", &v, &e));
  EXPECT_EQ(v.s, "\xF0\x9F\x98\x80");
  EXPECT_TRUE(parses(" {} "));
  EXPECT_TRUE(parses("[]"));
}

TEST(Json, RejectsEverythingMalformedOrAmbiguous) {
  for (const char* bad : {"",
                          "{",
                          "{\"a\":1,}",
                          "{\"a\" 1}",
                          "[1,]",
                          "{\"a\":1}{",
                          "{\"a\":1} x",
                          "{\"a\":1,\"a\":2}",
                          "01",
                          "1.",
                          "-",
                          ".5",
                          "+1",
                          "NaN",
                          "Infinity",
                          "\"a\nb\"",
                          "\"\\x\"",
                          "\"\\ud800\"",
                          "\"\\udc00\"",
                          "tru",
                          "nul",
                          "{'a':1}",
                          "[1 2]",
                          "1e",
                          "\"unterminated"}) {
    EXPECT_FALSE(parses(bad)) << bad;
  }
  std::string deep(kMaxDepth + 3, '[');
  deep += std::string(kMaxDepth + 3, ']');
  EXPECT_FALSE(parses(deep));
  std::string ok(kMaxDepth, '[');
  ok += std::string(kMaxDepth, ']');
  EXPECT_TRUE(parses(ok));
  EXPECT_FALSE(parses("1e999"));
}

TEST(Json, WritersEscapeAndRefuseNonFinite) {
  EXPECT_EQ(json_str("a\"\\\n\x01"), "\"a\\\"\\\\\\n\\u0001\"");
  EXPECT_EQ(json_num(NAN), "null");
  EXPECT_EQ(json_num(0.5), "0.5");
  EXPECT_EQ(std::stod(json_dbl(12.345678901234)), 12.345678901234);  // 17 digits round-trip exactly
  EXPECT_EQ(JsonLine().integer("a", 1).boolean("b", true).str("c", "x").raw("d", "[]").dump(),
            R"({"a":1,"b":true,"c":"x","d":[]})");
}

// ---- command validation
// ---------------------------------------------------------------------------------------------
namespace {
ParseResult P(const std::string& s) { return parse_command(s); }
const std::string kSha(64, 'a');
}  // namespace

TEST(Commands, ValidCommandsParseToTypedForms) {
  auto r = P(R"({"v":1,"id":7,"cmd":"heartbeat"})");
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.cmd.kind, CmdKind::Heartbeat);
  EXPECT_TRUE(r.has_id);
  EXPECT_EQ(r.id, 7);
  r = P(R"({"v":1,"cmd":"start_mission","args":{"path_artifact_sha256":")" + kSha + R"("}})");
  ASSERT_TRUE(r.ok);
  EXPECT_FALSE(r.has_id);
  EXPECT_EQ(r.cmd.sha256, kSha);
  r = P(R"({"v":1,"id":1,"cmd":"abort_mission","args":{"reason":"safety"}})");
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.cmd.abort_reason, 2);
  r = P(R"({"v":1,"id":1,"cmd":"abort_mission"})");
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.cmd.abort_reason, 0);
  r = P(R"({"v":1,"id":2,"cmd":"estop","args":{"asserted":true,"source":"tablet"}})");
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(r.cmd.flag);
  EXPECT_EQ(r.cmd.source, "tablet");
  EXPECT_TRUE(is_priority(r.cmd.kind));
  r = P(R"({"v":1,"id":3,"cmd":"arm","args":{"arm":false}})");
  ASSERT_TRUE(r.ok);
  EXPECT_FALSE(r.cmd.flag);
  for (const char* c : {"pause_mission", "resume_mission", "skip_point", "get_snapshot"}) {
    EXPECT_TRUE(P(std::string(R"({"v":1,"id":1,"cmd":")") + c + R"("})").ok) << c;
  }
  EXPECT_TRUE(P(R"({"v":1,"id":3,"cmd":"offboard","args":{"enable":true}})").ok);
  EXPECT_TRUE(P(R"({"v":1,"id":3,"cmd":"spray_manual","args":{"on":true}})").ok);
}

TEST(Commands, EveryDeviationIsRejectedAndTheIdIsStillRecovered) {
  struct Bad {
    const char* line;
    const char* code;
  };
  const Bad bad[] = {
      {"not json", "bad_message"},
      {"[1]", "bad_message"},
      {R"({"cmd":"heartbeat"})", "bad_message"},                         // no version
      {R"({"v":2,"id":9,"cmd":"heartbeat"})", "bad_message"},            // wrong version
      {R"({"v":1,"id":9,"cmd":"heartbeat","extra":1})", "bad_message"},  // unknown top-level field
      {R"({"v":1,"id":9.5,"cmd":"heartbeat"})", "bad_message"},          // id not an integer
      {R"({"v":1,"id":9,"cmd":5})", "bad_message"},
      {R"({"v":1,"id":9,"cmd":"reboot"})", "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"heartbeat","args":{"x":1}})", "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"heartbeat","args":[]})", "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"start_mission","args":{}})", "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"start_mission","args":{"path_artifact_sha256":"ABC"}})",
       "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"start_mission","args":{"path_artifact_sha256":"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"}})",
       "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"abort_mission","args":{"reason":"because"}})", "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"estop","args":{"asserted":true}})", "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"estop","args":{"asserted":"yes","source":"tablet"}})",
       "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"estop","args":{"asserted":true,"source":"root"}})",
       "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"arm","args":{"arm":1}})", "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"arm","args":{"arm":true,"force":true}})", "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"spray_manual","args":{}})", "invalid_command"},
  };
  for (const auto& b : bad) {
    const auto r = P(b.line);
    EXPECT_FALSE(r.ok) << b.line;
    EXPECT_EQ(r.code, b.code) << b.line;
  }
  const auto r = P(R"({"v":1,"id":9,"cmd":"reboot"})");
  EXPECT_TRUE(r.has_id);
  EXPECT_EQ(r.id, 9);
}

// ---- snapshot / operator link
// ------------------------------------------------------------------------------------------
TEST(Snapshot, MissingSourcesAreNullAndOldSourcesAreNotFresh) {
  TelemetrySnapshot s(1.0);
  s.update("rtk_status", R"({"fix_type":6})", 10.0);
  JsonValue v;
  std::string e;
  ASSERT_TRUE(parse_json(s.to_json(10.4, R"({"operator_alive":true})"), &v, &e)) << e;
  EXPECT_EQ(v.get("vehicle_state")->type, JsonValue::Type::Null);
  const JsonValue* r = v.get("rtk_status");
  ASSERT_NE(r, nullptr);
  EXPECT_TRUE(r->get("fresh")->b);
  EXPECT_NEAR(r->get("age_s")->n, 0.4, 1e-6);
  EXPECT_EQ(r->get("data")->get("fix_type")->i, 6);
  EXPECT_TRUE(v.get("gateway")->get("operator_alive")->b);
  ASSERT_TRUE(parse_json(s.to_json(12.0, "{}"), &v, &e));
  EXPECT_FALSE(v.get("rtk_status")->get("fresh")->b);  // a stale value is flagged, not hidden
  size_t n;
  TelemetrySnapshot::all_sources(&n);
  EXPECT_EQ(v.o.size(), n + 1);
}

TEST(OperatorLink, AliveOnlyWithAClientAndAFreshHeartbeat) {
  OperatorLink l(2.0);
  EXPECT_FALSE(l.state(1.0, 1).alive);  // never heard
  EXPECT_EQ(l.state(1.0, 1).age_s, 0.0);
  l.note_heartbeat(10.0);
  EXPECT_TRUE(l.state(11.9, 1).alive);
  EXPECT_FALSE(l.state(12.1, 1).alive);  // timeout
  EXPECT_NEAR(l.state(12.1, 1).age_s, 2.1, 1e-9);
  EXPECT_FALSE(
      l.state(10.5, 0).alive);  // no client connected (backend dead) is a dead operator link
  l.note_heartbeat(12.1);
  EXPECT_TRUE(l.state(12.2, 1).alive);
}

// ---- socket server
// -------------------------------------------------------------------------------------------------
TEST(IpcServer, FramingRepliesBroadcastAndModeBits) {
  IpcServer s;
  std::mutex mu;
  std::vector<std::pair<int, std::string>> got;
  IpcServer::Config c;
  c.path = tmp_sock();
  std::string err;
  ASSERT_TRUE(s.start(
      c,
      [&](int cl, const std::string& l) {
        std::lock_guard<std::mutex> lk(mu);
        got.emplace_back(cl, l);
        s.send(cl, "echo:" + l);
      },
      &err))
      << err;
  struct stat st{};
  ASSERT_EQ(stat(c.path.c_str(), &st), 0);
  EXPECT_EQ(st.st_mode & 0777, 0660U);
  Sock a(c.path), b(c.path);
  ASSERT_TRUE(a.ok() && b.ok());
  a.write_all("one\ntw");  // a line split across two writes
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  a.write_all("o\n");
  const auto r = a.read_lines(2);
  ASSERT_EQ(r.size(), 2U);
  EXPECT_EQ(r[0], "echo:one");
  EXPECT_EQ(r[1], "echo:two");
  for (int i = 0; i < 100 && s.clients() < 2; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(s.clients(), 2);
  s.broadcast("hello");
  EXPECT_EQ(a.read_lines(1).at(0), "hello");
  EXPECT_EQ(b.read_lines(1).at(0), "hello");
  s.stop();
  EXPECT_FALSE(std::filesystem::exists(c.path));
}

TEST(IpcServer, OversizeLineDropsTheClientAndMaxClientsIsEnforced) {
  IpcServer s;
  IpcServer::Config c;
  c.path = tmp_sock();
  c.max_clients = 2;
  c.max_line_bytes = 1024;
  std::string err;
  ASSERT_TRUE(s.start(c, [](int, const std::string&) {}, &err)) << err;
  Sock a(c.path), b(c.path), d(c.path);
  ASSERT_TRUE(a.ok() && b.ok() && d.ok());
  for (int i = 0; i < 100 && s.rejected_full() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(s.rejected_full(), 1U);
  EXPECT_TRUE(d.closed_by_peer());
  a.write_all(std::string(4096, 'x'));  // no newline, over the limit
  EXPECT_TRUE(a.closed_by_peer());
  EXPECT_GE(s.overflows(), 1U);
  // a well-behaved client is unaffected
  b.write_all("ok\n");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(b.closed_by_peer(200));
}

TEST(IpcServer, StaleSocketFileIsReplacedAndASlowConsumerIsDropped) {
  const std::string path = tmp_sock();
  {
    std::ofstream(path) << "stale";
  }
  IpcServer s;
  IpcServer::Config c;
  c.path = path;
  c.max_out_bytes = 64 * 1024;
  std::string err;
  ASSERT_TRUE(s.start(c, [](int, const std::string&) {}, &err)) << err;
  Sock a(path);  // never reads
  ASSERT_TRUE(a.ok());
  for (int i = 0; i < 100 && s.clients() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const std::string big(8192, 'y');
  for (int i = 0; i < 4000 && s.dropped_slow() == 0; ++i) {
    s.broadcast(big);
    if (i % 50 == 0) std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  for (int i = 0; i < 200 && s.dropped_slow() == 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_GE(s.dropped_slow(), 1U);
}

// GW-002: a reply queued for a peer that has already closed must not raise SIGPIPE. The server runs
// in a forked child with the DEFAULT SIGPIPE disposition (no SIG_IGN anywhere), so only the
// server's own send path decides whether the process survives. Each client sends an invalid line
// (answered at once, like the gateway's bad_message reply) and closes without reading.
TEST(IpcServer, APeerThatClosesBeforeItsReplyIsWrittenDoesNotKillTheProcess) {
  const std::string path = tmp_sock();
  int ready[2];
  ASSERT_EQ(pipe(ready), 0);
  const pid_t pid = fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    close(ready[0]);
    std::signal(SIGPIPE, SIG_DFL);
    std::atomic<bool> quit{false};
    IpcServer s;
    IpcServer::Config c;
    c.path = path;
    c.max_clients = 64;
    std::string err;
    const bool up = s.start(
        c,
        [&](int cl, const std::string& l) {
          if (l == "quit") {
            quit = true;
            return;
          }
          s.send(cl, R"({"v":1,"ok":false,"code":"bad_message","reason":")" +
                         std::string(2048, 'r') + R"(","data":{}})");
        },
        &err);
    const char b = up ? 1 : 0;
    (void)!write(ready[1], &b, 1);
    if (!up) _exit(2);
    for (int i = 0; i < 3000 && !quit; ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!quit) _exit(3);
    // Every closed peer must have been removed; only the control client is left.
    for (int i = 0; i < 300 && s.clients() != 1; ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const int left = s.clients();
    s.stop();
    _exit(left == 1 ? 0 : 4);
  }
  // The test process itself writes to the server; do not let a dead server kill the test runner.
  const auto old_pipe = std::signal(SIGPIPE, SIG_IGN);
  close(ready[1]);
  char b = 0;
  ASSERT_EQ(read(ready[0], &b, 1), 1);
  close(ready[0]);
  ASSERT_EQ(b, 1);
  int alive_after = -1;
  for (int i = 0; i < 500; ++i) {
    Sock a(path);
    if (!a.ok()) break;  // the server is gone
    a.write_all("not json\n");
    if (i % 50 == 0) std::this_thread::sleep_for(std::chrono::milliseconds(2));
    alive_after = i;
  }  // each Sock closes at once, without reading its reply
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  Sock ctl(path);
  if (ctl.ok()) ctl.write_all("quit\n");
  int st = 0;
  ASSERT_EQ(waitpid(pid, &st, 0), pid);  // always reaped, also when the server died
  std::signal(SIGPIPE, old_pipe);
  ASSERT_FALSE(WIFSIGNALED(st)) << "server killed by signal " << WTERMSIG(st) << " after "
                                << alive_after + 1 << " clients";
  ASSERT_TRUE(WIFEXITED(st));
  EXPECT_EQ(WEXITSTATUS(st), 0) << "2: start failed, 3: no quit, 4: closed peers not removed";
}
