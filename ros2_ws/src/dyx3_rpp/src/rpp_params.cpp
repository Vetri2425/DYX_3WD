#include "dyx3_rpp/rpp_params.hpp"

#include <cmath>
#include <cstring>
#include <sstream>

namespace dyx3_rpp {
namespace {

const Descriptor kTable[] = {
#include "dyx3_rpp/rpp_param_table.inc"
};
static_assert(sizeof(kTable) / sizeof(kTable[0]) == kParamCount, "descriptor table and id enum disagree");

std::string fmt(const Descriptor& d, double n, const std::string& s) {
  if (d.kind == Kind::String) return s;
  if (d.kind == Kind::Bool) return n != 0.0 ? "true" : "false";
  std::ostringstream o;
  o.precision(17);
  o << n;
  return o.str();
}

struct Relation {
  P lo;
  P hi;
  const char* why;
};
// DERIVED — NOT FROM V1 SPEC: structural orderings implied by how the prototype interpolates between
// the pairs (min/max bounds, full/none gate ends, hysteresis enter/exit). Not tuning values.
const Relation kRelations[] = {
    {P::min_linear_vel, P::max_linear_vel, "min_linear_vel must not exceed max_linear_vel"},
    {P::min_lookahead_dist, P::max_lookahead_dist, "min_lookahead_dist must not exceed max_lookahead_dist"},
    {P::accel_gate_heading_full_deg, P::accel_gate_heading_none_deg,
     "accel_gate_heading_full_deg must not exceed accel_gate_heading_none_deg"},
    {P::accel_gate_curv_full, P::accel_gate_curv_none, "accel_gate_curv_full must not exceed accel_gate_curv_none"},
    {P::kappa_hard_exit, P::kappa_hard_enter, "kappa_hard_exit must not exceed kappa_hard_enter"},
};

}  // namespace

const Descriptor* descriptors() { return kTable; }

int find_index(const std::string& name) {
  for (size_t i = 0; i < kParamCount; ++i) {
    if (name == kTable[i].name) return static_cast<int>(i);
  }
  return -1;
}

ParamSet::ParamSet() {
  for (size_t i = 0; i < kParamCount; ++i) {
    num_[i] = kTable[i].dflt;
    str_[i] = kTable[i].sdflt;
  }
}

SetResult ParamSet::validate(const Descriptor& d, double num, const std::string& str) {
  SetResult r;
  if (d.kind == Kind::String) {
    if (d.nonempty && str.empty()) {
      r.reason = std::string(d.name) + " must not be empty";
      return r;
    }
    if (d.allowed[0] != nullptr) {
      bool found = false;
      for (const char* a : d.allowed) found = found || (a != nullptr && str == a);
      if (!found) {
        r.reason = std::string(d.name) + ": '" + str + "' is not an allowed value";
        return r;
      }
    }
    r.ok = true;
    return r;
  }
  if (!std::isfinite(num)) {
    r.reason = std::string(d.name) + " must be finite";
    return r;
  }
  if (d.kind == Kind::Bool && num != 0.0 && num != 1.0) {
    r.reason = std::string(d.name) + " must be a boolean";
    return r;
  }
  if (d.kind == Kind::Int && num != std::floor(num)) {
    r.reason = std::string(d.name) + " must be an integer";
    return r;
  }
  if (d.kind != Kind::Bool) {
    if (num < d.lo || num > d.hi) {
      r.reason = std::string(d.name) + " out of range";
      return r;
    }
    if (d.positive && !(num > 0.0)) {
      r.reason = std::string(d.name) + " must be > 0";
      return r;
    }
  }
  r.ok = true;
  return r;
}

SetResult ParamSet::relations_of(const std::array<double, kParamCount>& v) const {
  SetResult r;
  for (const auto& rel : kRelations) {
    if (v[static_cast<size_t>(rel.lo)] > v[static_cast<size_t>(rel.hi)]) {
      r.reason = rel.why;
      return r;
    }
  }
  r.ok = true;
  return r;
}

SetResult ParamSet::check_relations() const { return relations_of(num_); }

SetResult ParamSet::set(const Item& item, const SetContext& ctx) { return set_many({item}, ctx); }

SetResult ParamSet::set_many(const std::vector<Item>& items, const SetContext& ctx) {
  SetResult fail;
  auto tentative_n = num_;
  auto tentative_s = str_;
  std::vector<size_t> touched;
  for (const auto& it : items) {
    const int idx = find_index(it.name);
    if (idx < 0) {
      fail.reason = "unknown parameter " + it.name;
      return fail;
    }
    const auto& d = kTable[idx];
    if (d.cls == ParamClass::Restart) {
      fail.reason = it.name + " is RESTART: it cannot change at runtime";
      return fail;
    }
    if (d.cls == ParamClass::IdleOnly && ctx.mission_running) {
      fail.reason = it.name + " is IDLE_ONLY: refused while a mission is running";
      return fail;
    }
    const auto v = validate(d, it.num, it.str);
    if (!v.ok) return v;
    tentative_n[static_cast<size_t>(idx)] = it.num;
    tentative_s[static_cast<size_t>(idx)] = it.str;
    touched.push_back(static_cast<size_t>(idx));
  }
  const auto rel = relations_of(tentative_n);
  if (!rel.ok) return rel;
  for (const size_t i : touched) {
    const auto& d = kTable[i];
    Change c;
    c.seq = ++seq_;
    c.name = d.name;
    c.old_value = fmt(d, num_[i], str_[i]);
    c.new_value = fmt(d, tentative_n[i], tentative_s[i]);
    c.source = ctx.source;
    journal_.push_back(std::move(c));
  }
  num_ = tentative_n;
  str_ = tentative_s;
  SetResult ok;
  ok.ok = true;
  return ok;
}

}  // namespace dyx3_rpp
