// motion_types — plain command type and the per-mode field contract. See
// docs/contracts/dyx3_motion_guard.md. Pure C++, no ROS.
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace dyx3_motion_guard {

enum class Mode : uint8_t { Stop = 0, TrackHeading = 1, TrackRate = 2, Pivot = 3, Creep = 4 };

// Numeric values are the frozen MotionSetpointStatus.REASON_* ABI (checked by static_assert in the
// node).
enum class Reason : uint8_t {
  Ok = 0,
  InvalidMessage = 1,
  Stale = 2,
  Sequence = 3,
  MissionGate = 4,
  Estop = 5,
  RtkGate = 6,
  LimitClamped = 7,
  Px4LinkUnhealthy = 8,
  HeadingUnhealthy = 9,
  OperatorLinkLost = 10,
  ArmingGate = 11,
  EstimatorUnhealthy = 12
};

struct Motion {
  Mode mode{Mode::Stop};
  float speed_body_x{0.0F};
  float yaw_setpoint{std::numeric_limits<float>::quiet_NaN()};
  float yaw_rate_setpoint{0.0F};
};

inline Motion canonical_stop() { return Motion{}; }

struct Command {
  uint64_t seq{0};
  uint8_t mode{0};
  float speed_body_x{0.0F};
  float yaw_setpoint{0.0F};
  float yaw_rate_setpoint{0.0F};
  bool valid{false};
};

inline bool uses_rate(Mode m) {
  return m == Mode::TrackRate || m == Mode::Pivot || m == Mode::Creep;
}

// Per-mode contract of MotionSetpoint.msg. A STOP command is always acceptable: whatever its other
// fields say it means the canonical zero.
inline bool command_is_valid(const Command& c) {
  if (!c.valid || c.mode > static_cast<uint8_t>(Mode::Creep)) return false;
  const bool sp = std::isfinite(c.speed_body_x);
  const bool yf = std::isfinite(c.yaw_setpoint), yn = std::isnan(c.yaw_setpoint);
  const bool rf = std::isfinite(c.yaw_rate_setpoint), rn = std::isnan(c.yaw_rate_setpoint);
  switch (static_cast<Mode>(c.mode)) {
    case Mode::Stop:
      return true;
    case Mode::TrackHeading:
      return sp && yf && rn;
    case Mode::TrackRate:
    case Mode::Creep:
      return sp && rf && yn;
    case Mode::Pivot:
      return sp && c.speed_body_x == 0.0F && rf && yn;
  }
  return false;
}

}  // namespace dyx3_motion_guard
