// px4_sequencer — see docs/contracts/dyx3_mission.md section 5.
#include "dyx3_mission/px4_sequencer.hpp"

#include <algorithm>
#include <cmath>

namespace dyx3_mission {

const char* to_string(Px4Op op) {
  switch (op) {
    case Px4Op::kArm:
      return "arm";
    case Px4Op::kOffboardOn:
      return "set_offboard(true)";
    case Px4Op::kOffboardOff:
      return "set_offboard(false)";
    case Px4Op::kDisarm:
      return "disarm";
  }
  return "?";
}

const char* to_string(Px4Result r) {
  switch (r) {
    case Px4Result::kOk:
      return "ok";
    case Px4Result::kRefused:
      return "refused";
    case Px4Result::kTimeout:
      return "timeout";
  }
  return "?";
}

void Px4Sequencer::begin_execution() {
  queue_.clear();
  flights_.clear();
  arm_ = Arm::kNone;
  offboard_requested_ = false;
  releasing_ = false;
}

void Px4Sequencer::engage(Px4Op op) {
  if (releasing_ || is_release(op)) return;
  queue_.push_back(op);
}

void Px4Sequencer::release(bool force_disarm) {
  if (releasing_) {
    // Already releasing: an E-stop during the release still forces the disarm.
    if (force_disarm && std::find(queue_.begin(), queue_.end(), Px4Op::kDisarm) == queue_.end()) {
      const bool disarm_in_flight =
          std::any_of(flights_.begin(), flights_.end(),
                      [](const Flight& f) { return f.req.op == Px4Op::kDisarm; });
      if (!disarm_in_flight) queue_.push_back(Px4Op::kDisarm);
    }
    return;
  }
  releasing_ = true;
  queue_.clear();  // engage steps not yet sent are never sent
  for (auto& f : flights_) f.abandoned = true;
  if (offboard_requested_) queue_.push_back(Px4Op::kOffboardOff);
  if (force_disarm || arm_owned()) queue_.push_back(Px4Op::kDisarm);
}

std::optional<Px4Op> Px4Sequencer::release_pending() const {
  for (const auto& f : flights_) {
    if (is_release(f.req.op)) return f.req.op;
  }
  if (releasing_ && !queue_.empty()) return queue_.front();
  return std::nullopt;
}

double Px4Sequencer::timeout_s(Px4Op op) const {
  return (op == Px4Op::kArm || op == Px4Op::kDisarm) ? timeouts_.arm_s : timeouts_.offboard_s;
}

std::optional<Px4Request> Px4Sequencer::next(std::int64_t now_ns) {
  if (queue_.empty()) return std::nullopt;
  // One step at a time. Abandoned engage flights do not hold the release back.
  for (const auto& f : flights_) {
    if (!f.abandoned) return std::nullopt;
  }
  Flight f;
  f.req.op = queue_.front();
  f.req.id = next_id_++;
  f.deadline_ns = now_ns + static_cast<std::int64_t>(std::llround(timeout_s(f.req.op) * 1e9));
  queue_.pop_front();
  if (f.req.op == Px4Op::kArm) arm_ = Arm::kInFlight;
  if (f.req.op == Px4Op::kOffboardOn) offboard_requested_ = true;
  flights_.push_back(f);
  return f.req;
}

void Px4Sequencer::note_arm_result(Px4Result r, std::uint8_t reason) {
  if (r == Px4Result::kOk) {
    arm_ = Arm::kConfirmed;
  } else if (r == Px4Result::kTimeout || reason == kArmReasonTimeout) {
    arm_ = Arm::kDoubt;  // PX4 may still arm: the release disarms
  } else {
    arm_ = Arm::kRefused;  // px4_link refused before commanding: nothing to disarm
  }
}

std::optional<Px4Outcome> Px4Sequencer::on_response(std::uint64_t id, bool accepted,
                                                    std::uint8_t reason, std::int64_t) {
  const auto it = std::find_if(flights_.begin(), flights_.end(),
                               [id](const Flight& f) { return f.req.id == id; });
  if (it == flights_.end()) return std::nullopt;
  Px4Outcome o;
  o.op = it->req.op;
  o.id = id;
  o.result = accepted ? Px4Result::kOk : Px4Result::kRefused;
  o.reason_code = reason;
  o.abandoned = it->abandoned;
  flights_.erase(it);
  if (o.op == Px4Op::kArm) note_arm_result(o.result, reason);
  return o;
}

std::vector<Px4Outcome> Px4Sequencer::on_tick(std::int64_t now_ns) {
  std::vector<Px4Outcome> out;
  for (auto it = flights_.begin(); it != flights_.end();) {
    if (now_ns < it->deadline_ns) {
      ++it;
      continue;
    }
    Px4Outcome o;
    o.op = it->req.op;
    o.id = it->req.id;
    o.result = Px4Result::kTimeout;
    o.abandoned = it->abandoned;
    if (o.op == Px4Op::kArm) note_arm_result(o.result, 0);
    out.push_back(o);
    it = flights_.erase(it);
  }
  return out;
}

}  // namespace dyx3_mission
