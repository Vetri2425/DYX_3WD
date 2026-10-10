// event_stream — the gateway's pushed status events. Contract:
// docs/contracts/dyx3_system_gateway.md section 1.3. Pure C++ (std only), clock-injected.
//
// Each event kind (mission_state, operator_link, fcu_link, estop) is fed with every new value and a
// transition key. A value whose key differs from the previous one is a transition and is pushed at
// once, unless an event of the same kind went out less than `coalesce_s` ago: then it waits, and
// every further value until flush() folds into that one pending event (it carries the latest data
// and counts the folded transitions in `coalesced`). Every pushed event gets the next sequence
// number. replay() sends the latest event of each kind, marked "replay":true and with its original
// sequence number, so a client that (re)connects has the current state at once.
//
// Thread safety: offer() and flush() run on the producer thread (the ROS executor); replay() runs
// on the IPC thread. One mutex orders a push against a replay, so a client never receives a replay
// after a newer push of the same kind.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>

namespace dyx3_gateway {

class EventStream {
public:
  using Sink = std::function<void(const std::string& line)>;
  using WallMsFn = std::function<int64_t()>;

  // `broadcast` receives each event line (no trailing newline). `wall_ms` stamps t_wall_ms (Unix
  // milliseconds; system clock when null).
  EventStream(double coalesce_s, Sink broadcast, WallMsFn wall_ms = nullptr);

  // A new value of `kind`. `key` names the transition (equal keys = no transition); `data_json`
  // must be a JSON object, the full current value. Pushes at once when it is a transition and the
  // kind's coalescing window has passed.
  void offer(const std::string& kind, const std::string& key, const std::string& data_json,
             double now_s);
  // Pushes every pending event whose coalescing window has passed. Call it periodically.
  void flush(double now_s);
  // Sends the latest pushed event of each kind (in sequence order) through `send`.
  void replay(const Sink& send);

  // The key of the last value offered for `kind`, "" when none.
  std::string last_key(const std::string& kind) const;
  uint64_t seq() const;  // sequence number of the last pushed event, 0 = none
  bool pending(const std::string& kind) const;

private:
  struct Kind {
    bool seen{false};
    std::string key;  // key of the last value offered
    bool pending{false};
    uint64_t folded{0};      // transitions in the pending event, minus one
    std::string data;        // latest data (carried by the pending event)
    double observed_s{0.0};  // when the carried data was offered
    double last_push_s{-1e18};
    uint64_t last_seq{0};
    std::string replay_line;
  };
  void push_locked(const std::string& kind, Kind& k, double now_s);

  double coalesce_s_;
  Sink broadcast_;
  WallMsFn wall_ms_;
  mutable std::mutex mu_;
  std::map<std::string, Kind> kinds_;
  uint64_t seq_{0};
};

}  // namespace dyx3_gateway
