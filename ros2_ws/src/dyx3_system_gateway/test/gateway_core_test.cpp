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
#include "dyx3_system_gateway/event_stream.hpp"
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

// GW-008: raw (unescaped) bytes inside strings must be strict UTF-8.
TEST(Json, RawStringBytesMustBeStrictUtf8) {
  struct Fixture {
    const char* bytes;
    const char* what;
  };
  const Fixture good[] = {
      {"\x24", "U+0024"},
      {"\xC2\x80", "U+0080, smallest 2-byte"},
      {"\xC3\xA9", "U+00E9"},
      {"\xDF\xBF", "U+07FF, largest 2-byte"},
      {"\xE0\xA0\x80", "U+0800, smallest 3-byte"},
      {"\xE2\x82\xAC", "U+20AC"},
      {"\xED\x9F\xBF", "U+D7FF, just below the surrogates"},
      {"\xEE\x80\x80", "U+E000, just above the surrogates"},
      {"\xEF\xBF\xBF", "U+FFFF"},
      {"\xF0\x90\x80\x80", "U+10000, smallest 4-byte"},
      {"\xF0\x9F\x98\x80", "U+1F600"},
      {"\xF4\x8F\xBF\xBF", "U+10FFFF, the last code point"},
  };
  const Fixture bad[] = {
      {"\x80", "lone continuation byte"},
      {"\xBF", "lone continuation byte"},
      {"\xC0\xAF", "overlong '/' (C0)"},
      {"\xC1\xBF", "overlong (C1)"},
      {"\xE0\x80\xAF", "overlong 3-byte '/'"},
      {"\xE0\x9F\xBF", "overlong 3-byte U+07FF"},
      {"\xF0\x80\x80\xAF", "overlong 4-byte '/'"},
      {"\xF0\x8F\xBF\xBF", "overlong 4-byte U+FFFF"},
      {"\xED\xA0\x80", "surrogate U+D800"},
      {"\xED\xBF\xBF", "surrogate U+DFFF"},
      {"\xF4\x90\x80\x80", "U+110000, above U+10FFFF"},
      {"\xF5\x80\x80\x80", "lead byte F5"},
      {"\xFF", "byte FF"},
      {"\xFE", "byte FE"},
      {"\xC3", "truncated 2-byte"},
      {"\xE2\x82", "truncated 3-byte"},
      {"\xF0\x9F\x98", "truncated 4-byte"},
      {"\xC3\x28", "bad continuation"},
      {"\xE2\x28\xA1", "bad continuation"},
  };
  for (const auto& f : good) {
    EXPECT_TRUE(is_valid_utf8(f.bytes)) << f.what;
    JsonValue v;
    std::string e;
    ASSERT_TRUE(parse_json(std::string("\"a") + f.bytes + "b\"", &v, &e)) << f.what << ": " << e;
    EXPECT_EQ(v.s, std::string("a") + f.bytes + "b") << f.what;
    EXPECT_EQ(json_escape(f.bytes), f.bytes) << f.what;  // valid text passes through untouched
  }
  for (const auto& f : bad) {
    EXPECT_FALSE(is_valid_utf8(f.bytes)) << f.what;
    EXPECT_FALSE(parses(std::string("\"a") + f.bytes + "b\"")) << f.what;
    EXPECT_FALSE(parses(std::string("{\"") + f.bytes + "\":1}")) << f.what << " in a key";
    EXPECT_TRUE(is_valid_utf8(json_escape(f.bytes))) << f.what;
  }
  EXPECT_EQ(json_escape("a\xC0\xAF"
                        "b"),
            "a\\ufffd\\ufffdb");
  EXPECT_EQ(json_escape("\xE2\x82"), "\\ufffd\\ufffd");
}

TEST(Json, InvalidUtf8IsRejectedAndNeverEchoed) {
  // A command whose name or argument carries invalid bytes is refused, and the reason text (which
  // the gateway sends back) contains no invalid UTF-8.
  // (ordinary string literals: the \x escapes must become raw bytes, not JSON escape text)
  const std::string lines[] = {
      "{\"v\":1,\"id\":5,\"cmd\":\"\xC0\xAF"
      "reboot\"}",
      "{\"v\":1,\"id\":5,\"cmd\":\"estop\",\"args\":{\"asserted\":true,\"source\":\"tab\xFF"
      "let\"}}",
      "{\"v\":1,\"id\":5,\"cmd\":\"heartbeat\",\"\xED\xA0\x80\":1}",
  };
  for (const std::string& line : lines) {
    ASSERT_EQ(line.find('\\'), std::string::npos);  // really raw bytes
    const auto r = parse_command(line);
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.code, "bad_message");
    EXPECT_NE(r.reason.find("invalid UTF-8"), std::string::npos) << r.reason;
    EXPECT_TRUE(is_valid_utf8(r.reason)) << r.reason;
    EXPECT_TRUE(is_valid_utf8(json_str(r.reason)));
  }
  // and an unknown command made of valid UTF-8 is still echoed faithfully
  const auto r = parse_command(
      "{\"v\":1,\"id\":5,\"cmd\":\"r\xC3\xA9"
      "boot\"}");
  EXPECT_EQ(r.code, "invalid_command");
  EXPECT_NE(r.reason.find("r\xC3\xA9"
                          "boot"),
            std::string::npos);
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

// The same bound as dyx3_mission (StartMission.request_id) and the backend: 1..64.
static_assert(kMaxRequestIdLen == 64, "request_id bound must match dyx3_mission and the backend");

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
  EXPECT_EQ(r.cmd.request_id, "");  // optional: absent is empty
  EXPECT_FALSE(r.cmd.resume);       // optional: absent is false
  r = P(R"({"v":1,"id":5,"cmd":"start_mission","args":{"path_artifact_sha256":")" + kSha +
        R"(","resume":true}})");
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(r.cmd.resume);
  r = P(R"({"v":1,"id":5,"cmd":"start_mission","args":{"path_artifact_sha256":")" + kSha +
        R"(","request_id":"r-1","resume":false}})");
  ASSERT_TRUE(r.ok);
  EXPECT_FALSE(r.cmd.resume);
  EXPECT_EQ(r.cmd.request_id, "r-1");
  r = P(R"({"v":1,"id":4,"cmd":"start_mission","args":{"path_artifact_sha256":")" + kSha +
        R"(","request_id":"5f0c-AB_9.x:1"}})");
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.cmd.request_id, "5f0c-AB_9.x:1");
  r = P(R"({"v":1,"id":4,"cmd":"start_mission","args":{"path_artifact_sha256":")" + kSha +
        R"(","request_id":")" + std::string(kMaxRequestIdLen, 'r') + R"("}})");
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.cmd.request_id.size(), kMaxRequestIdLen);
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
      // start_mission.request_id: a string of [A-Za-z0-9._:-], 1..kMaxRequestIdLen
      {R"({"v":1,"id":9,"cmd":"start_mission","args":{"path_artifact_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","request_id":7}})",
       "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"start_mission","args":{"path_artifact_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","request_id":""}})",
       "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"start_mission","args":{"path_artifact_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","request_id":"a b"}})",
       "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"start_mission","args":{"path_artifact_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","request_id":"x\n"}})",
       "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"start_mission","args":{"path_artifact_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","request_id":null}})",
       "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"pause_mission","args":{"request_id":"r1"}})", "invalid_command"},
      // start_mission.resume: a boolean only
      {R"({"v":1,"id":9,"cmd":"start_mission","args":{"path_artifact_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","resume":1}})",
       "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"start_mission","args":{"path_artifact_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","resume":"true"}})",
       "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"start_mission","args":{"path_artifact_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","resume":null}})",
       "invalid_command"},
      {R"({"v":1,"id":9,"cmd":"resume_mission","args":{"resume":true}})", "invalid_command"},
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
  const std::string too_long(kMaxRequestIdLen + 1, 'r');
  const auto tl = P(R"({"v":1,"id":9,"cmd":"start_mission","args":{"path_artifact_sha256":")" +
                    kSha + R"(","request_id":")" + too_long + R"("}})");
  EXPECT_FALSE(tl.ok);
  EXPECT_EQ(tl.code, "invalid_command");
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
  EXPECT_TRUE(s.fresh("rtk_status", 11.0));
  EXPECT_FALSE(s.fresh("rtk_status", 11.1));
  EXPECT_FALSE(s.fresh("vehicle_state", 10.0));  // never received is never fresh
}

// ---- pushed events
// -------------------------------------------------------------------------------------------------
namespace {
struct Events {
  std::vector<JsonValue> out;
  EventStream es;
  explicit Events(double coalesce_s = 0.01)
      : es(
            coalesce_s,
            [this](const std::string& l) {
              JsonValue v;
              std::string e;
              EXPECT_TRUE(parse_json(l, &v, &e)) << e << ": " << l;
              out.push_back(v);
            },
            [] { return int64_t{1760000000123}; }) {}
};
}  // namespace

TEST(EventStream, ATransitionIsPushedAtOnceWithItsSequenceAndStamps) {
  Events ev;
  ev.es.offer("mission_state", "2/0", R"({"state":2})", 10.0);
  ASSERT_EQ(ev.out.size(), 1U);
  const JsonValue& e = ev.out[0];
  EXPECT_EQ(e.get("v")->i, kProtocolVersion);
  EXPECT_EQ(e.get("type")->s, "event");
  EXPECT_EQ(e.get("event")->s, "mission_state");
  EXPECT_EQ(e.get("seq")->i, 1);
  EXPECT_DOUBLE_EQ(e.get("t_mono_s")->n, 10.0);
  EXPECT_EQ(e.get("t_wall_ms")->i, 1760000000123);
  EXPECT_EQ(e.get("coalesced")->i, 0);
  EXPECT_FALSE(e.get("replay")->b);
  EXPECT_EQ(e.get("data")->get("state")->i, 2);
  // the same key again is not a transition: nothing is pushed (the snapshot carries it)
  ev.es.offer("mission_state", "2/0", R"({"state":2,"point_index":7})", 10.5);
  EXPECT_EQ(ev.out.size(), 1U);
  EXPECT_FALSE(ev.es.pending("mission_state"));
  // another kind has its own window and shares the sequence
  ev.es.offer("estop", "1:tablet", R"({"asserted":true})", 10.501);
  ASSERT_EQ(ev.out.size(), 2U);
  EXPECT_EQ(ev.out[1].get("seq")->i, 2);
  EXPECT_EQ(ev.es.seq(), 2U);
  EXPECT_EQ(ev.es.last_key("estop"), "1:tablet");
  EXPECT_EQ(ev.es.last_key("fcu_link"), "");
}

TEST(EventStream, TransitionsWithinTheWindowAreFoldedIntoOneEventWithTheLatestData) {
  Events ev;
  ev.es.offer("mission_state", "1/0", R"({"state":1})", 20.000);  // leading edge: pushed
  ev.es.offer("mission_state", "2/0", R"({"state":2})", 20.002);  // inside the window: waits
  ev.es.offer("mission_state", "3/0", R"({"state":3})", 20.004);  // folded
  ev.es.offer("mission_state", "3/0", R"({"state":3,"point_index":1})", 20.006);  // latest data
  ASSERT_EQ(ev.out.size(), 1U);
  EXPECT_TRUE(ev.es.pending("mission_state"));
  ev.es.flush(20.009);  // the window (10 ms from the last push) has not passed
  ASSERT_EQ(ev.out.size(), 1U);
  ev.es.flush(20.010);
  ASSERT_EQ(ev.out.size(), 2U);
  const JsonValue& e = ev.out[1];
  EXPECT_EQ(e.get("seq")->i, 2);
  EXPECT_EQ(e.get("coalesced")->i, 1);  // two transitions in one event: one folded away
  EXPECT_EQ(e.get("data")->get("state")->i, 3);
  EXPECT_EQ(e.get("data")->get("point_index")->i, 1);
  EXPECT_DOUBLE_EQ(e.get("t_mono_s")->n, 20.006);  // when the carried value was observed
  // after a quiet window the next transition is pushed at once again
  ev.es.offer("mission_state", "4/0", R"({"state":4})", 20.030);
  ASSERT_EQ(ev.out.size(), 3U);
  EXPECT_EQ(ev.out[2].get("coalesced")->i, 0);
  // a window of 0 pushes every transition
  Events all(0.0);
  for (int i = 0; i < 5; ++i)
    all.es.offer("estop", std::to_string(i % 2), "{}", 1.0);  // same instant, five transitions
  EXPECT_EQ(all.out.size(), 5U);
}

TEST(EventStream, ReplayGivesTheLatestEventOfEachKindInSequenceOrder) {
  Events ev;
  std::vector<std::string> sent;
  ev.es.replay([&](const std::string& l) { sent.push_back(l); });
  EXPECT_TRUE(sent.empty());  // nothing pushed yet, nothing to replay
  ev.es.offer("estop", "0:", R"({"asserted":false})", 1.0);
  ev.es.offer("mission_state", "2/0", R"({"state":2})", 1.0);
  ev.es.offer("estop", "1:ble", R"({"asserted":true})", 1.5);
  ev.es.replay([&](const std::string& l) { sent.push_back(l); });
  ASSERT_EQ(sent.size(), 2U);
  JsonValue a, b;
  std::string e;
  ASSERT_TRUE(parse_json(sent[0], &a, &e)) << e;
  ASSERT_TRUE(parse_json(sent[1], &b, &e)) << e;
  EXPECT_EQ(a.get("event")->s, "mission_state");
  EXPECT_EQ(a.get("seq")->i, 2);
  EXPECT_EQ(b.get("event")->s, "estop");
  EXPECT_EQ(b.get("seq")->i, 3);  // the original sequence number: a client can drop a duplicate
  EXPECT_TRUE(a.get("replay")->b && b.get("replay")->b);
  EXPECT_TRUE(b.get("data")->get("asserted")->b);
  EXPECT_EQ(ev.es.seq(), 3U);  // a replay is not a new event
}

TEST(OperatorLink, AliveOnlyWithAClientAndAFreshHeartbeat) {
  OperatorLink l(2.0);
  EXPECT_EQ(l.client(), -1);
  EXPECT_FALSE(l.state(1.0, true).alive);  // never heard
  EXPECT_EQ(l.state(1.0, true).age_s, 0.0);
  l.note_heartbeat(10.0, 3);
  EXPECT_EQ(l.client(), 3);
  EXPECT_TRUE(l.state(11.9, true).alive);
  EXPECT_FALSE(l.state(12.1, true).alive);  // timeout
  EXPECT_NEAR(l.state(12.1, true).age_s, 2.1, 1e-9);
  // the heartbeating client is gone (backend dead): a dead operator link, however fresh
  EXPECT_FALSE(l.state(10.5, false).alive);
  l.note_heartbeat(12.1, 4);  // another connection takes over only by heartbeating itself
  EXPECT_EQ(l.client(), 4);
  EXPECT_TRUE(l.state(12.2, true).alive);
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
  int a_id = -1;
  {
    std::lock_guard<std::mutex> lk(mu);
    ASSERT_FALSE(got.empty());
    a_id = got[0].first;
  }
  EXPECT_TRUE(s.connected(a_id));
  EXPECT_FALSE(s.connected(a_id + 100));
  a.~Sock();
  new (&a) Sock("/nonexistent");
  for (int i = 0; i < 100 && s.connected(a_id); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_FALSE(s.connected(a_id));  // a closed connection is gone at once
  EXPECT_EQ(s.clients(), 1);
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

// XR-GW-003: a second instance on the same path must neither steal nor delete the live socket.
TEST(IpcServer, ASecondServerOnTheSamePathIsRefusedAndTheFirstKeepsServing) {
  const std::string path = tmp_sock();
  IpcServer a;
  IpcServer::Config c;
  c.path = path;
  std::string err;
  ASSERT_TRUE(a.start(c, [&](int cl, const std::string& l) { a.send(cl, "a:" + l); }, &err)) << err;
  struct stat before{};
  ASSERT_EQ(stat(path.c_str(), &before), 0);
  {
    IpcServer b;
    err.clear();
    EXPECT_FALSE(b.start(c, [](int, const std::string&) {}, &err));
    EXPECT_NE(err.find("another gateway"), std::string::npos) << err;
  }  // b's destructor (stop) must not unlink a's socket either
  struct stat after{};
  ASSERT_EQ(stat(path.c_str(), &after), 0);
  EXPECT_EQ(after.st_ino, before.st_ino);  // the same socket, not re-bound
  Sock s(path);
  ASSERT_TRUE(s.ok());
  s.write_all("ping\n");
  const auto r = s.read_lines(1);
  ASSERT_EQ(r.size(), 1U);
  EXPECT_EQ(r[0], "a:ping");
  a.stop();
  EXPECT_FALSE(std::filesystem::exists(path));
  // once released, the path can be served again
  IpcServer d;
  EXPECT_TRUE(d.start(c, [](int, const std::string&) {}, &err)) << err;
  d.stop();
  std::filesystem::remove(path + ".lock");
}

TEST(IpcServer, TheConnectHookSeesEachNewClientAndCanWriteToIt) {
  const std::string path = tmp_sock();
  IpcServer s;
  IpcServer::Config c;
  c.path = path;
  std::string err;
  std::mutex mu;
  std::vector<int> seen;
  ASSERT_TRUE(s.start(
      c, [](int, const std::string&) {}, &err,
      [&](int cl) {
        {
          std::lock_guard<std::mutex> lk(mu);
          seen.push_back(cl);
        }
        s.send(cl, "hello " + std::to_string(cl));
      }))
      << err;
  Sock a(path);
  ASSERT_TRUE(a.ok());
  auto r = a.read_lines(1);
  ASSERT_EQ(r.size(), 1U);
  EXPECT_EQ(r[0].rfind("hello ", 0), 0U);
  Sock b(path);
  ASSERT_TRUE(b.ok());
  r = b.read_lines(1);
  ASSERT_EQ(r.size(), 1U);
  std::lock_guard<std::mutex> lk(mu);
  ASSERT_EQ(seen.size(), 2U);
  EXPECT_NE(seen[0], seen[1]);
  EXPECT_EQ(r[0], "hello " + std::to_string(seen[1]));
}

TEST(IpcServer, StopLeavesASocketFileItDidNotCreate) {
  const std::string path = tmp_sock();
  IpcServer a;
  IpcServer::Config c;
  c.path = path;
  std::string err;
  ASSERT_TRUE(a.start(c, [](int, const std::string&) {}, &err)) << err;
  // the path is replaced behind the server's back (e.g. by an operator's manual instance)
  ASSERT_EQ(unlink(path.c_str()), 0);
  {
    std::ofstream(path) << "someone else";
  }
  a.stop();
  EXPECT_TRUE(std::filesystem::exists(path));
  std::filesystem::remove(path);
  std::filesystem::remove(path + ".lock");
}
