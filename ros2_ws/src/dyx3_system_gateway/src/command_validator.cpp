#include "dyx3_system_gateway/command_validator.hpp"

#include <set>

#include "dyx3_system_gateway/json.hpp"

namespace dyx3_gateway {
namespace {

ParseResult fail(ParseResult r, const char* code, const std::string& why) {
  r.ok = false;
  r.code = code;
  r.reason = why;
  return r;
}

bool only_keys(const JsonValue& o, const std::set<std::string>& allowed, std::string* bad) {
  for (const auto& kv : o.o) {
    if (!allowed.count(kv.first)) {
      *bad = kv.first;
      return false;
    }
  }
  return true;
}

const JsonValue* need(const JsonValue& args, const char* key, JsonValue::Type t) {
  const JsonValue* v = args.get(key);
  return (v != nullptr && v->type == t) ? v : nullptr;
}

bool is_hex64(const std::string& s) {
  if (s.size() != 64) return false;
  for (const char c : s)
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  return true;
}

}  // namespace

const char* to_string(CmdKind k) {
  switch (k) {
    case CmdKind::Heartbeat:
      return "heartbeat";
    case CmdKind::GetSnapshot:
      return "get_snapshot";
    case CmdKind::StartMission:
      return "start_mission";
    case CmdKind::AbortMission:
      return "abort_mission";
    case CmdKind::PauseMission:
      return "pause_mission";
    case CmdKind::ResumeMission:
      return "resume_mission";
    case CmdKind::SkipPoint:
      return "skip_point";
    case CmdKind::Estop:
      return "estop";
    case CmdKind::Arm:
      return "arm";
    case CmdKind::Offboard:
      return "offboard";
    case CmdKind::SprayManual:
      return "spray_manual";
  }
  return "?";
}

ParseResult parse_command(const std::string& line) {
  ParseResult r;
  JsonValue root;
  std::string err;
  if (!parse_json(line, &root, &err)) return fail(r, "bad_message", "invalid JSON: " + err);
  if (root.type != JsonValue::Type::Object)
    return fail(r, "bad_message", "message must be a JSON object");
  // Recover the id first so even a rejected message is answered with it.
  if (const JsonValue* id = root.get("id")) {
    if (id->type == JsonValue::Type::Number && id->is_int) {
      r.has_id = true;
      r.id = id->i;
    }
  }
  std::string bad;
  if (!only_keys(root, {"v", "id", "cmd", "args"}, &bad))
    return fail(r, "bad_message", "unknown field '" + bad + "'");
  const JsonValue* v = root.get("v");
  if (v == nullptr || v->type != JsonValue::Type::Number || !v->is_int ||
      v->i != kProtocolVersion) {
    return fail(r, "bad_message", "unsupported or missing protocol version");
  }
  if (const JsonValue* id = root.get("id")) {
    if (!(id->type == JsonValue::Type::Number && id->is_int))
      return fail(r, "bad_message", "id must be an integer");
  }
  const JsonValue* cmd = root.get("cmd");
  if (cmd == nullptr || cmd->type != JsonValue::Type::String)
    return fail(r, "bad_message", "cmd must be a string");
  JsonValue empty;
  empty.type = JsonValue::Type::Object;
  const JsonValue* args = root.get("args");
  if (args == nullptr) args = &empty;
  if (args->type != JsonValue::Type::Object)
    return fail(r, "invalid_command", "args must be an object");

  const std::string& c = cmd->s;
  Command out;
  const auto no_args = [&](CmdKind k) -> ParseResult {
    if (!args->o.empty()) return fail(r, "invalid_command", c + " takes no arguments");
    out.kind = k;
    r.ok = true;
    r.cmd = out;
    return r;
  };
  if (c == "heartbeat") return no_args(CmdKind::Heartbeat);
  if (c == "get_snapshot") return no_args(CmdKind::GetSnapshot);
  if (c == "pause_mission") return no_args(CmdKind::PauseMission);
  if (c == "resume_mission") return no_args(CmdKind::ResumeMission);
  if (c == "skip_point") return no_args(CmdKind::SkipPoint);

  const auto bool_arg = [&](CmdKind k, const char* key) -> ParseResult {
    if (!only_keys(*args, {key}, &bad))
      return fail(r, "invalid_command", "unknown argument '" + bad + "'");
    const JsonValue* f = need(*args, key, JsonValue::Type::Bool);
    if (f == nullptr)
      return fail(r, "invalid_command", std::string(key) + " (boolean) is required");
    out.kind = k;
    out.flag = f->b;
    r.ok = true;
    r.cmd = out;
    return r;
  };
  if (c == "arm") return bool_arg(CmdKind::Arm, "arm");
  if (c == "offboard") return bool_arg(CmdKind::Offboard, "enable");
  if (c == "spray_manual") return bool_arg(CmdKind::SprayManual, "on");

  if (c == "start_mission") {
    if (!only_keys(*args, {"path_artifact_sha256"}, &bad))
      return fail(r, "invalid_command", "unknown argument '" + bad + "'");
    const JsonValue* s = need(*args, "path_artifact_sha256", JsonValue::Type::String);
    if (s == nullptr || !is_hex64(s->s)) {
      return fail(r, "invalid_command", "path_artifact_sha256 must be 64 lowercase hex characters");
    }
    out.kind = CmdKind::StartMission;
    out.sha256 = s->s;
    r.ok = true;
    r.cmd = out;
    return r;
  }
  if (c == "abort_mission") {
    if (!only_keys(*args, {"reason"}, &bad))
      return fail(r, "invalid_command", "unknown argument '" + bad + "'");
    out.kind = CmdKind::AbortMission;
    if (const JsonValue* rs = args->get("reason")) {
      if (rs->type != JsonValue::Type::String)
        return fail(r, "invalid_command", "reason must be a string");
      if (rs->s == "operator")
        out.abort_reason = 1;
      else if (rs->s == "safety")
        out.abort_reason = 2;
      else if (rs->s == "unspecified")
        out.abort_reason = 0;
      else
        return fail(r, "invalid_command", "reason must be operator, safety or unspecified");
    }
    r.ok = true;
    r.cmd = out;
    return r;
  }
  if (c == "estop") {
    if (!only_keys(*args, {"asserted", "source"}, &bad))
      return fail(r, "invalid_command", "unknown argument '" + bad + "'");
    const JsonValue* a = need(*args, "asserted", JsonValue::Type::Bool);
    const JsonValue* s = need(*args, "source", JsonValue::Type::String);
    if (a == nullptr || s == nullptr)
      return fail(r, "invalid_command", "asserted (boolean) and source (string) are required");
    if (s->s != "tablet" && s->s != "backend" && s->s != "ble" && s->s != "physical") {
      return fail(r, "invalid_command", "source must be tablet, backend, ble or physical");
    }
    out.kind = CmdKind::Estop;
    out.flag = a->b;
    out.source = s->s;
    r.ok = true;
    r.cmd = out;
    return r;
  }
  return fail(r, "invalid_command", "unknown command '" + c + "'");
}

}  // namespace dyx3_gateway
