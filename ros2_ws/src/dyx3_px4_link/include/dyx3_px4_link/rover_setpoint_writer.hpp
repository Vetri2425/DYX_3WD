// rover_setpoint_writer — MotionSetpoint -> explicit PX4 control set, plus the fail-to-zero gate.
// See docs/contracts/dyx3_px4_link.md sections 3 and 4. Pure C++, no ROS.
#pragma once

#include <cstdint>

namespace dyx3_px4_link {

enum class Mode : uint8_t { Stop = 0, TrackHeading = 1, TrackRate = 2, Pivot = 3, Creep = 4 };

// Plain copy of dyx3_interfaces/MotionSetpoint (without the stamp).
struct Command {
  uint64_t seq{0};
  uint8_t mode{0};
  float speed_body_x{0.0F};
  float yaw_setpoint{0.0F};
  float yaw_rate_setpoint{0.0F};
  bool valid{false};
};

// The five-field explicit-control set. NaN is a value that is always sent, never omitted.
struct Setpoint {
  Mode mode{Mode::Stop};
  float speed_body_x{0.0F};
  float yaw_setpoint;       // NaN unless TrackHeading
  float yaw_rate_setpoint;  // NaN for TrackHeading
  Setpoint();
};

Setpoint stop_setpoint();

enum class Verdict : uint8_t { Ok = 0, NotValid, BadMode, NonFinite, ContractViolation };

// Checks the per-mode field contract of MotionSetpoint.msg. A STOP command is always Ok (whatever
// its other fields say, it maps to the canonical zero set).
Verdict validate(const Command& c);

// Maps a command that validate() accepted. Anything else yields stop_setpoint().
Setpoint to_setpoint(const Command& c);

enum class Reason : uint8_t {
  None = 0,        // a fresh, valid guard command is being forwarded
  NoCommand,       // nothing received (yet, or after a sequence reset)
  CommandStale,    // newest command older than command_max_age_s
  CommandInvalid,  // valid=false, bad mode, non-finite, or breaks the per-mode contract
  HandshakeNotOk,  // message formats not proven identical
  NoSession,       // uXRCE-DDS session not alive
  TopicStale,      // a monitored topic is stale
  SequenceReset,   // publisher restarted: the first command of the new session is discarded
  GuardStop        // the guard itself commanded STOP (forwarded, not a fault)
};

struct GateInputs {
  double now_s;  // link steady clock
  bool handshake_ok;
  bool session_alive;
  uint32_t stale_topics_mask;
};

struct GateOutput {
  Setpoint sp;
  Reason reason{Reason::NoCommand};
  bool failing_to_zero{true};  // true for every reason except None and GuardStop
  double command_age_s{1.0e9};
};

class CommandGate {
public:
  explicit CommandGate(double command_max_age_s) : max_age_(command_max_age_s) {}

  // Freshness is refreshed only by a sequence number strictly greater than the last one accepted:
  // a repeated command (same seq) must never look fresh. A smaller seq means the publisher
  // restarted: the command is discarded and the gate reports SequenceReset for one tick.
  void on_command(const Command& c, double now_s);

  GateOutput step(const GateInputs& in);

  uint64_t gap_events() const { return gap_events_; }

private:
  double max_age_;
  bool have_cmd_{false};
  bool have_seq_{false};
  uint64_t last_seq_{0};
  Command cmd_{};
  double received_s_{0.0};
  bool gap_counted_{false};
  bool reset_pending_{false};
  uint64_t gap_events_{0};
};

}  // namespace dyx3_px4_link
