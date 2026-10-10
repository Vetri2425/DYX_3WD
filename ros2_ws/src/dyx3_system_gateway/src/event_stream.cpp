#include "dyx3_system_gateway/event_stream.hpp"

#include <algorithm>
#include <chrono>
#include <vector>

#include "dyx3_system_gateway/command_validator.hpp"
#include "dyx3_system_gateway/json.hpp"

namespace dyx3_gateway {
namespace {
int64_t system_wall_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}
}  // namespace

EventStream::EventStream(double coalesce_s, Sink broadcast, WallMsFn wall_ms)
    : coalesce_s_(coalesce_s),
      broadcast_(std::move(broadcast)),
      wall_ms_(wall_ms ? std::move(wall_ms) : WallMsFn(system_wall_ms)) {}

void EventStream::offer(const std::string& kind, const std::string& key,
                        const std::string& data_json, double now_s) {
  std::lock_guard<std::mutex> lk(mu_);
  Kind& k = kinds_[kind];
  const bool transition = !k.seen || key != k.key;
  k.seen = true;
  k.key = key;
  if (transition) {
    if (k.pending) ++k.folded;
    k.pending = true;
  }
  if (!k.pending) return;  // not a transition and nothing waiting: the snapshot carries it
  k.data = data_json;
  k.observed_s = now_s;
  if (now_s - k.last_push_s >= coalesce_s_ - 1e-9) push_locked(kind, k, now_s);
}

void EventStream::flush(double now_s) {
  std::lock_guard<std::mutex> lk(mu_);
  for (auto& [name, k] : kinds_) {
    if (k.pending && now_s - k.last_push_s >= coalesce_s_ - 1e-9) push_locked(name, k, now_s);
  }
}

void EventStream::push_locked(const std::string& kind, Kind& k, double now_s) {
  const uint64_t seq = ++seq_;
  k.pending = false;
  k.last_push_s = now_s;  // the window runs from the push, t_mono_s is the observation
  k.last_seq = seq;
  const int64_t wall = wall_ms_();
  const std::string common = JsonLine()
                                 .integer("v", kProtocolVersion)
                                 .str("type", "event")
                                 .str("event", kind)
                                 .integer("seq", static_cast<int64_t>(seq))
                                 .raw("t_mono_s", json_dbl(k.observed_s))
                                 .integer("t_wall_ms", wall)
                                 .integer("coalesced", static_cast<int64_t>(k.folded))
                                 .dump();
  // common is "{...}": both lines append the replay flag and the data to the same prefix.
  const std::string prefix = common.substr(0, common.size() - 1);
  k.replay_line = prefix + ",\"replay\":true,\"data\":" + k.data + "}";
  k.folded = 0;
  if (broadcast_) broadcast_(prefix + ",\"replay\":false,\"data\":" + k.data + "}");
}

void EventStream::replay(const Sink& send) {
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<const Kind*> order;
  for (const auto& [name, k] : kinds_)
    if (k.last_seq != 0) order.push_back(&k);
  std::sort(order.begin(), order.end(),
            [](const Kind* a, const Kind* b) { return a->last_seq < b->last_seq; });
  for (const Kind* k : order) send(k->replay_line);
}

std::string EventStream::last_key(const std::string& kind) const {
  std::lock_guard<std::mutex> lk(mu_);
  const auto it = kinds_.find(kind);
  return it == kinds_.end() ? std::string() : it->second.key;
}

uint64_t EventStream::seq() const {
  std::lock_guard<std::mutex> lk(mu_);
  return seq_;
}

bool EventStream::pending(const std::string& kind) const {
  std::lock_guard<std::mutex> lk(mu_);
  const auto it = kinds_.find(kind);
  return it != kinds_.end() && it->second.pending;
}

}  // namespace dyx3_gateway
