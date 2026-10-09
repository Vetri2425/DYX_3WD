// command_validator — strict parsing and validation of one client line into a typed Command.
// Contract: docs/contracts/dyx3_system_gateway.md sections 1 and 2. Pure C++. No policy: only
// syntax, types, ranges.
#pragma once

#include <cstdint>
#include <string>

namespace dyx3_gateway {

enum class CmdKind : uint8_t {
  Heartbeat,
  GetSnapshot,
  StartMission,
  AbortMission,
  PauseMission,
  ResumeMission,
  SkipPoint,
  Estop,
  Arm,
  Offboard,
  SprayManual
};

const char* to_string(CmdKind k);

struct Command {
  CmdKind kind{CmdKind::Heartbeat};
  std::string sha256;       // StartMission
  uint8_t abort_reason{0};  // AbortMission: 0 unspecified, 1 operator, 2 safety
  bool flag{false};         // Estop.asserted / Arm.arm / Offboard.enable / SprayManual.on
  std::string source;       // Estop
};

struct ParseResult {
  bool ok{false};
  std::string code;    // "bad_message" | "invalid_command" when !ok
  std::string reason;  // human text
  bool has_id{false};
  int64_t id{0};
  Command cmd;
};

constexpr int kProtocolVersion = 1;
ParseResult parse_command(const std::string& line);

// E-stop and heartbeat are processed ahead of other work; the gateway node orders E-stop strictly
// first, coalesces heartbeats and never refuses an E-stop.
inline bool is_priority(CmdKind k) { return k == CmdKind::Estop || k == CmdKind::Heartbeat; }

}  // namespace dyx3_gateway
