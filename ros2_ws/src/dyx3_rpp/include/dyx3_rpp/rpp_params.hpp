// rpp_params — the 117 motion/tracking parameters with class, structural validation and change
// recording. See docs/contracts/dyx3_rpp.md section on parameters. Pure C++, no ROS.
//
// Every tunable is classified LIVE / IDLE_ONLY / RESTART (spec section 9). A change is validated
// before it is applied, an IDLE_ONLY change while a mission is running and any RESTART change at
// runtime are REFUSED with a reason (never deferred), and every applied change is recorded.
// Defaults are the prototype's, verbatim, and must be re-validated at GATE 4.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace dyx3_rpp {

enum class Kind : uint8_t { Bool, Int, Float, String };
enum class ParamClass : uint8_t { Live, IdleOnly, Restart };

// Index of every numeric/bool/int parameter (and string ones, which also have a slot). Generated.
enum class P : uint16_t {
#include "dyx3_rpp/rpp_param_ids.inc"
  kCount
};

constexpr size_t kParamCount = static_cast<size_t>(P::kCount);

struct Descriptor {
  const char* name;
  Kind kind;
  ParamClass cls;
  double dflt;             // numeric default (bool as 0/1)
  const char* sdflt;       // string default
  double lo;               // structural lower bound (inclusive)
  double hi;               // structural upper bound (inclusive)
  bool positive;           // value must be > 0 (a divisor or period)
  bool nonempty;           // strings: must not be empty
  const char* allowed[4];  // strings: closed set when allowed[0] != nullptr
};

const Descriptor* descriptors();          // kParamCount entries, index == static_cast<size_t>(P)
int find_index(const std::string& name);  // -1 if unknown

struct Change {
  uint64_t seq;
  std::string name;
  std::string old_value;
  std::string new_value;
  std::string source;
};

struct SetContext {
  bool mission_running{false};  // IDLE_ONLY changes are refused while true
  std::string source;           // who asked ("tablet", "backend", "cli"): recorded with the change
};

struct SetResult {
  bool ok{false};
  std::string reason;
};

struct Item {
  std::string name;
  double num{0.0};  // bool as 0/1
  std::string str;  // string parameters
};

class ParamSet {
public:
  ParamSet();  // prototype defaults

  // Hot-path accessors: array index, no allocation, no hashing.
  double num(P id) const { return num_[static_cast<size_t>(id)]; }
  bool flag(P id) const { return num_[static_cast<size_t>(id)] != 0.0; }
  int integer(P id) const { return static_cast<int>(std::llround(num_[static_cast<size_t>(id)])); }
  const std::string& str(P id) const { return str_[static_cast<size_t>(id)]; }

  // Validate and apply one change / an atomic batch (all or nothing, cross-parameter relations
  // checked on the tentative result). Recorded in the journal on success.
  SetResult set(const Item& item, const SetContext& ctx);
  SetResult set_many(const std::vector<Item>& items, const SetContext& ctx);

  // Startup only: validates exactly like set_many (including the cross-parameter relations) but
  // applies to every class, since a RESTART value is by definition set while the process starts.
  // Atomic.
  SetResult init_many(const std::vector<Item>& items);

  // Structural validation of a value against its descriptor (no class rule, no relations).
  static SetResult validate(const Descriptor& d, double num, const std::string& str);

  // Cross-parameter relations on the whole set (also run on the defaults).
  SetResult check_relations() const;

  const std::vector<Change>& journal() const { return journal_; }

private:
  SetResult relations_of(const std::array<double, kParamCount>& v) const;
  std::array<double, kParamCount> num_{};
  std::array<std::string, kParamCount> str_{};
  std::vector<Change> journal_;
  uint64_t seq_{0};
};

}  // namespace dyx3_rpp
