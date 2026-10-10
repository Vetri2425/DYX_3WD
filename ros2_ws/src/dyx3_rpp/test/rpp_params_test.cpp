#include "dyx3_rpp/rpp_params.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>

using namespace dyx3_rpp;

namespace {
struct Row {
  std::string type, dflt, cls;
};
// Parse the registry's dyx3_rpp rows independently of the generator.
std::map<std::string, Row> registry() {
  std::map<std::string, Row> m;
  std::ifstream f(DYX3_REGISTRY);
  std::string line;
  while (std::getline(f, line)) {
    if (line.rfind("| `", 0) != 0) continue;
    std::vector<std::string> c;
    std::stringstream ss(line.substr(1));
    std::string cell;
    while (std::getline(ss, cell, '|')) {
      const auto a = cell.find_first_not_of(' ');
      const auto b = cell.find_last_not_of(' ');
      c.push_back(a == std::string::npos ? "" : cell.substr(a, b - a + 1));
    }
    if (c.size() < 7 || c[6] != "dyx3_rpp") continue;
    auto strip = [](std::string s, char ch) {
      s.erase(std::remove(s.begin(), s.end(), ch), s.end());
      return s;
    };
    m[strip(c[0], '`')] = {c[1], strip(c[2], '`'), strip(c[4], '*')};
  }
  return m;
}
}  // namespace

TEST(RppParams, TableMatchesTheRegistry) {
  const auto reg = registry();
  ASSERT_EQ(reg.size(), 117U);
  ASSERT_EQ(kParamCount, 117U);
  const auto* d = descriptors();
  for (size_t i = 0; i < kParamCount; ++i) {
    const auto it = reg.find(d[i].name);
    ASSERT_NE(it, reg.end()) << d[i].name;
    const auto& r = it->second;
    const char* cls = d[i].cls == ParamClass::Live       ? "LIVE"
                      : d[i].cls == ParamClass::IdleOnly ? "IDLE_ONLY"
                                                         : "RESTART";
    EXPECT_EQ(r.cls, cls) << d[i].name;
    if (d[i].kind == Kind::String) {
      EXPECT_EQ(r.dflt, std::string("\"") + d[i].sdflt + "\"") << d[i].name;
    } else if (d[i].kind == Kind::Bool) {
      EXPECT_EQ(r.dflt, d[i].dflt != 0.0 ? "True" : "False") << d[i].name;
    } else {
      EXPECT_DOUBLE_EQ(std::stod(r.dflt), d[i].dflt) << d[i].name;
    }
  }
}

TEST(RppParams, EveryDefaultIsValidAndRelationsHold) {
  const auto* d = descriptors();
  ParamSet p;
  for (size_t i = 0; i < kParamCount; ++i) {
    EXPECT_TRUE(ParamSet::validate(d[i], d[i].dflt, d[i].sdflt).ok) << d[i].name;
  }
  EXPECT_TRUE(p.check_relations().ok);
  // Field-test speed defaults (not the prototype's 1.0): physical cap = PX4 RO_SPEED_LIM, start
  // 0.6.
  EXPECT_DOUBLE_EQ(p.num(P::max_linear_vel), 0.85);
  EXPECT_DOUBLE_EQ(p.num(P::mission_speed), 0.6);
  EXPECT_EQ(p.str(P::tracking_profile), "auto");
  EXPECT_EQ(p.str(P::segment_command_mode), "heading");
  EXPECT_TRUE(p.flag(P::require_rtk_fix));
  EXPECT_EQ(p.integer(P::preview_curvature_n), 4);
}

TEST(RppParams, ClassRulesAreEnforced) {
  ParamSet p;
  SetContext idle;
  SetContext running;
  running.mission_running = true;
  EXPECT_TRUE(p.set({"max_linear_vel", 0.4, ""}, running).ok);  // LIVE
  EXPECT_DOUBLE_EQ(p.num(P::max_linear_vel), 0.4);
  const auto r = p.set({"require_rtk_fix", 0.0, ""}, running);  // IDLE_ONLY while running
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.reason.find("IDLE_ONLY"), std::string::npos);
  EXPECT_TRUE(p.flag(P::require_rtk_fix));  // not applied, not deferred
  const auto mode_running = p.set({"segment_command_mode", 0.0, "rate"}, running);
  EXPECT_FALSE(mode_running.ok);
  EXPECT_NE(mode_running.reason.find("IDLE_ONLY"), std::string::npos);
  EXPECT_EQ(p.str(P::segment_command_mode), "heading");
  EXPECT_TRUE(p.set({"segment_command_mode", 0.0, "rate"}, idle).ok);
  EXPECT_EQ(p.str(P::segment_command_mode), "rate");
  EXPECT_TRUE(p.set({"require_rtk_fix", 0.0, ""}, idle).ok);
  EXPECT_FALSE(p.flag(P::require_rtk_fix));
  const auto rs = p.set({"path_frame_id", 0.0, "other"}, idle);  // RESTART never at runtime
  EXPECT_FALSE(rs.ok);
  EXPECT_NE(rs.reason.find("RESTART"), std::string::npos);
  EXPECT_EQ(p.str(P::path_frame_id), "local_ned");
  EXPECT_FALSE(p.set({"no_such_param", 1.0, ""}, idle).ok);
}

TEST(RppParams, StructuralValidation) {
  ParamSet p;
  SetContext c;
  const double nan = std::nan("");
  const double inf = HUGE_VAL;
  EXPECT_FALSE(p.set({"max_linear_vel", nan, ""}, c).ok);
  EXPECT_FALSE(p.set({"max_linear_vel", inf, ""}, c).ok);
  EXPECT_FALSE(p.set({"max_linear_vel", 0.0, ""}, c).ok);       // divisor: > 0
  EXPECT_FALSE(p.set({"min_linear_vel", -0.1, ""}, c).ok);      // negative
  EXPECT_TRUE(p.set({"min_linear_vel", 0.0, ""}, c).ok);        // zero allowed where not a divisor
  EXPECT_FALSE(p.set({"preview_curvature_n", 2.5, ""}, c).ok);  // int
  EXPECT_FALSE(p.set({"preview_curvature_n", 0.0, ""}, c).ok);
  EXPECT_FALSE(p.set({"closed_loop_min_travel_frac", 1.5, ""}, c).ok);
  EXPECT_FALSE(p.set({"tracking_profile", 0.0, "bogus"}, c).ok);
  EXPECT_FALSE(p.set({"tracking_profile", 0.0, ""}, c).ok);
  EXPECT_TRUE(p.set({"tracking_profile", 0.0, "segment"}, c).ok);
  EXPECT_FALSE(p.set({"segment_command_mode", 0.0, "bogus"}, c).ok);
  EXPECT_FALSE(p.set({"segment_command_mode", 0.0, ""}, c).ok);
  EXPECT_TRUE(p.set({"segment_command_mode", 0.0, "rate"}, c).ok);
  EXPECT_FALSE(p.set({"precise_stop_mode", 0.0, "x"}, c).ok);
  EXPECT_TRUE(p.set({"pose_latency_bias_s", -0.05, ""}, c).ok);  // the one signed parameter
  EXPECT_FALSE(p.set({"require_rtk_fix", 2.0, ""}, c).ok);
  EXPECT_EQ(p.str(P::tracking_profile), "segment");
  EXPECT_EQ(p.str(P::segment_command_mode), "rate");
}

TEST(RppParams, RelationsAreCheckedAndBatchesAreAtomic) {
  ParamSet p;
  SetContext c;
  EXPECT_FALSE(p.set({"min_linear_vel", 2.0, ""}, c).ok);  // above max_linear_vel
  EXPECT_DOUBLE_EQ(p.num(P::min_linear_vel), 0.15);
  // raising max first and min second works as one batch, in any order
  EXPECT_TRUE(p.set_many({{"min_linear_vel", 2.0, ""}, {"max_linear_vel", 3.0, ""}}, c).ok);
  // a batch with one bad item changes nothing
  const size_t before = p.journal().size();
  EXPECT_FALSE(p.set_many({{"max_linear_accel", 0.3, ""}, {"max_linear_decel", -1.0, ""}}, c).ok);
  EXPECT_DOUBLE_EQ(p.num(P::max_linear_accel), 0.20);
  EXPECT_EQ(p.journal().size(), before);
  EXPECT_FALSE(p.set({"kappa_hard_exit", 0.5, ""}, c).ok);  // above kappa_hard_enter
}

TEST(RppParams, ChangesAreRecorded) {
  ParamSet p;
  SetContext c;
  c.source = "tablet";
  ASSERT_TRUE(p.set({"segment_yaw_rate_gain", 2.0, ""}, c).ok);
  ASSERT_EQ(p.journal().size(), 1U);
  const auto& j = p.journal()[0];
  EXPECT_EQ(j.name, "segment_yaw_rate_gain");
  EXPECT_EQ(j.source, "tablet");
  EXPECT_EQ(j.old_value, "1.5");
  EXPECT_EQ(j.new_value, "2");
  EXPECT_EQ(j.seq, 1U);
  EXPECT_FALSE(p.set({"segment_yaw_rate_gain", -1.0, ""}, c).ok);
  EXPECT_EQ(p.journal().size(), 1U);  // a refusal leaves no trace in the journal
}

// XR-RPP-009: sane upper bounds on the freshness gates, the per-tick geometry walk and the loop
// counts; an Int never overflows int.
TEST(RppParams, UpperBoundsAndIntegerRange) {
  ParamSet p;
  SetContext c;
  EXPECT_TRUE(p.set({"pose_max_age_s", 2.0, ""}, c).ok);
  EXPECT_FALSE(p.set({"pose_max_age_s", 2.01, ""}, c).ok);
  EXPECT_DOUBLE_EQ(p.num(P::pose_max_age_s), 2.0);
  EXPECT_FALSE(p.set({"rtk_fix_timeout_s", 2.5, ""}, c).ok);
  EXPECT_FALSE(p.set({"curvature_baseline_m", 2.5, ""}, c).ok);
  EXPECT_TRUE(p.set({"curvature_baseline_m", 2.0, ""}, c).ok);
  EXPECT_TRUE(p.set({"preview_curvature_n", 64.0, ""}, c).ok);
  EXPECT_FALSE(p.set({"preview_curvature_n", 65.0, ""}, c).ok);
  EXPECT_FALSE(p.set({"corner_smooth_arc_pts", 1e12, ""}, c).ok);
  EXPECT_EQ(p.integer(P::preview_curvature_n), 64);
  // structural: an Int outside int is refused whatever its table bound says
  Descriptor wide = descriptors()[static_cast<size_t>(P::preview_curvature_n)];
  wide.hi = HUGE_VAL;
  EXPECT_TRUE(ParamSet::validate(wide, 1000.0, "").ok);
  EXPECT_FALSE(ParamSet::validate(wide, 1e300, "").ok);
  EXPECT_FALSE(ParamSet::validate(wide, 4294967296.0, "").ok);
}
