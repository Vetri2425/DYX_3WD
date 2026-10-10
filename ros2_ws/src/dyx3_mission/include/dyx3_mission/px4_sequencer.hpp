// px4_sequencer — the PX4 service calls of one mission execution: arm, OFFBOARD, and their
// release. See docs/contracts/dyx3_mission.md section 5.
//
// Pure C++ (no ROS): the node sends what next() returns through its asynchronous clients of
// /dyx3/px4_link/arm and /dyx3/px4_link/set_offboard, and feeds back the responses and the clock.
// The sequencer owns the bookkeeping the safety rules depend on:
//  * one engage step (arm, then OFFBOARD) in flight at a time, each with its own timeout;
//  * what THIS execution armed / engaged, so the release disarms exactly that ("on any doubt,
//    disarm": an arm that timed out or is still in flight counts as armed);
//  * the release order: set_offboard(false) first, then arm(false), one at a time; a release is
//    sent at once even while an engage request is still in flight (that request is abandoned).
#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace dyx3_mission {

enum class Px4Op : std::uint8_t { kArm, kOffboardOn, kOffboardOff, kDisarm };
enum class Px4Result : std::uint8_t { kOk, kRefused, kTimeout };

const char* to_string(Px4Op op);
const char* to_string(Px4Result r);

/// ArmDisarm.Response.REASON_TIMEOUT: px4_link sent the command but PX4 did not confirm in time.
/// The arm may still have happened, so it is a doubt (the node static_asserts the value).
constexpr std::uint8_t kArmReasonTimeout = 3;

struct Px4Request {
  Px4Op op = Px4Op::kArm;
  std::uint64_t id = 0;
};

struct Px4Outcome {
  Px4Op op = Px4Op::kArm;
  Px4Result result = Px4Result::kOk;
  std::uint8_t reason_code = 0;  ///< the service's REASON_* (0 on a timeout)
  std::uint64_t id = 0;
  /// An engage request that a release overtook: its result no longer drives the lifecycle.
  bool abandoned = false;
};

class Px4Sequencer {
public:
  struct Timeouts {
    double arm_s = 4.0;       ///< arm and disarm
    double offboard_s = 5.0;  ///< set_offboard(true) and set_offboard(false)
  };

  void set_timeouts(const Timeouts& t) { timeouts_ = t; }
  const Timeouts& timeouts() const { return timeouts_; }

  /// Forget the previous execution's bookkeeping. Only legal while !busy().
  void begin_execution();
  /// Queue an engage step (kArm or kOffboardOn). Ignored while releasing.
  void engage(Px4Op op);
  /// Stop engaging and queue the release: set_offboard(false) if this execution requested OFFBOARD,
  /// then arm(false) if it armed (or may have armed), or always when `force_disarm` (E-stop).
  void release(bool force_disarm);

  /// The next request to send now, if any (it is then in flight until its response or timeout).
  std::optional<Px4Request> next(std::int64_t now_ns);
  /// A response to request `id`; nullopt for an unknown id (already timed out or never sent).
  std::optional<Px4Outcome> on_response(std::uint64_t id, bool accepted, std::uint8_t reason,
                                        std::int64_t now_ns);
  /// Requests whose timeout passed, as kTimeout outcomes (the node also drops their client entry).
  std::vector<Px4Outcome> on_tick(std::int64_t now_ns);

  /// A request is in flight or queued.
  bool busy() const { return !flights_.empty() || !queue_.empty(); }
  bool releasing() const { return releasing_; }
  /// The release step being waited on (kOffboardOff / kDisarm), if any.
  std::optional<Px4Op> release_pending() const;
  /// This execution armed the vehicle or may have (in flight, confirmed or timed out).
  bool arm_owned() const {
    return arm_ == Arm::kInFlight || arm_ == Arm::kConfirmed || arm_ == Arm::kDoubt;
  }
  bool offboard_requested() const { return offboard_requested_; }

private:
  enum class Arm : std::uint8_t { kNone, kInFlight, kConfirmed, kRefused, kDoubt };
  struct Flight {
    Px4Request req;
    std::int64_t deadline_ns = 0;
    bool abandoned = false;
  };
  static bool is_release(Px4Op op) { return op == Px4Op::kOffboardOff || op == Px4Op::kDisarm; }
  double timeout_s(Px4Op op) const;
  void note_arm_result(Px4Result r, std::uint8_t reason);

  Timeouts timeouts_;
  std::deque<Px4Op> queue_;
  std::vector<Flight> flights_;
  Arm arm_ = Arm::kNone;
  bool offboard_requested_ = false;
  bool releasing_ = false;
  std::uint64_t next_id_ = 1;
};

}  // namespace dyx3_mission
