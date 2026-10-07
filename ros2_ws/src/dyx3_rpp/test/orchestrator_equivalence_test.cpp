// GATE 4 (control-tick orchestrator): RppCore::tick compared tick by tick with the VERBATIM PX4_DXP
// node (tools/gate4/gen_orchestrator_vectors.py). The C++ is fed the SAME recorded inputs (poses,
// velocities, GPS, clock) and must reproduce every published value, the segment-debug stream, the
// state the tick leaves behind, and WHICH unported stop/pivot machine the carried code entered.
// Values to 1e-9 relative (expected outputs are stored with 13 digits; libm differences).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "dyx3_rpp/rpp_core.hpp"

using namespace dyx3_rpp;

namespace {

std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> o;
  std::istringstream is(s);
  std::string t;
  while (is >> t) o.push_back(t);
  return o;
}
double num(const std::string& t) { return std::strtod(t.c_str(), nullptr); }
long long inum(const std::string& t) { return std::strtoll(t.c_str(), nullptr, 10); }

bool same(double a, double b) {
  if (std::isnan(a) || std::isnan(b)) return std::isnan(a) && std::isnan(b);
  if (std::isinf(a) || std::isinf(b)) return a == b;
  return std::fabs(a - b) <= 1e-9 * std::max(1e-3, std::max(std::fabs(a), std::fabs(b)));
}

struct Stats {
  int scenarios{0}, ticks{0}, compared{0}, handoffs{0}, failures{0};
  std::map<std::string, int> handoff_names;
  std::map<int, int> states;
  int shown{0};
};

// The arguments are evaluated ONCE (the expected value is often a parse expression with a side
// effect).
#define CHECK_NEAR(stats, what, exp, got)                                                  \
  do {                                                                                     \
    const double e_ = (exp);                                                               \
    const double g_ = (got);                                                               \
    if (!same(e_, g_)) {                                                                   \
      ++(stats).failures;                                                                  \
      if ((stats).shown++ < 40)                                                            \
        ADD_FAILURE() << scen << " tick " << tick_no << " " << what << ": expected " << e_ \
                      << " got " << g_;                                                    \
    }                                                                                      \
  } while (0)

#define CHECK_EQ(stats, what, exp, got)                                                    \
  do {                                                                                     \
    const auto e_ = (exp);                                                                 \
    const auto g_ = (got);                                                                 \
    if (!(e_ == g_)) {                                                                     \
      ++(stats).failures;                                                                  \
      if ((stats).shown++ < 40)                                                            \
        ADD_FAILURE() << scen << " tick " << tick_no << " " << what << ": expected " << e_ \
                      << " got " << g_;                                                    \
    }                                                                                      \
  } while (0)

}  // namespace

TEST(OrchestratorEquivalence, TickByTickAgainstTheCarriedNode) {
  std::ifstream f(std::string(DYX3_FIXTURES) + "/gate4_orchestrator_vectors.txt");
  ASSERT_TRUE(f.good()) << "fixture missing";
  std::vector<std::string> lines;
  for (std::string l; std::getline(f, l);) lines.push_back(l);
  ASSERT_GT(lines.size(), 1000u);
  ASSERT_EQ(lines[0], "GATE4ORCH 1");

  Stats st;
  size_t i = 1;
  while (i < lines.size()) {
    if (lines[i].rfind("SCEN ", 0) != 0) {
      ++i;
      continue;
    }
    auto h = split(lines[i++]);
    const std::string scen = h[1];
    const bool align_done = h[2] == "1";
    const int n_params = static_cast<int>(inum(h[3]));
    const int n_runs = static_cast<int>(inum(h[4]));
    ++st.scenarios;

    ParamSet params;
    std::vector<Item> items;
    for (int k = 0; k < n_params; ++k) {
      auto t = split(lines[i++]);
      ASSERT_EQ(t[0], "PARAM") << scen;
      Item it;
      it.name = t[1];
      it.num = num(t[2]);
      items.push_back(it);
    }
    SetContext ctx;
    ctx.mission_running = false;
    ctx.source = "test";
    const SetResult sr = params.set_many(items, ctx);
    ASSERT_TRUE(sr.ok) << scen << ": " << sr.reason;

    std::vector<ConditionedRun> runs;
    for (int k = 0; k < n_runs; ++k) {
      auto t = split(lines[i++]);
      ASSERT_EQ(t[0], "RUN") << scen;
      ConditionedRun r;
      r.profile = t[1] == "segment" ? Profile::Segment : Profile::Smooth;
      r.length = num(t[2]);
      r.closed = t[3] == "1";
      const int n = static_cast<int>(inum(t[4]));
      for (int p = 0; p < n; ++p) {
        auto q = split(lines[i++]);
        r.pts.push_back({num(q[0]), num(q[1])});
        r.flags.push_back(static_cast<unsigned char>(inum(q[2])));
        r.must_hit.push_back(static_cast<unsigned char>(inum(q[3])));
        r.cum_s.push_back(num(q[4]));
      }
      runs.push_back(std::move(r));
    }

    RppCore core(params);
    core.install_mission(std::move(runs));
    if (align_done) core.mark_alignment_done();

    int tick_no = 0;
    bool episode_over = false;
    while (i < lines.size() && lines[i] != "END") {
      auto t = split(lines[i++]);
      if (t.empty() || episode_over) continue;
      if (t[0] == "EV") {
        const int64_t ns = t.size() > 2 && t[1] != "INIT" ? static_cast<int64_t>(inum(t[2])) : 0;
        if (t[1] == "POSE") {
          NedPose p;
          p.n = num(t[3]);
          p.e = num(t[4]);
          p.yaw_ned = yaw_ned_from_enu_quaternion(num(t[5]), num(t[6]), num(t[7]), num(t[8]));
          core.on_pose(p, ns);
        } else if (t[1] == "VEL") {
          // ENU twist: linear.x = East, linear.y = North; angular.z is CCW-positive
          core.on_velocity(num(t[4]), num(t[3]), -num(t[5]), ns);
        } else if (t[1] == "GPS") {
          const long long mm = inum(t[4]);
          core.on_gps(static_cast<int>(inum(t[3])),
                      mm > 0 ? static_cast<double>(mm) * 1e-3 : std::nan(""), ns);
        } else if (t[1] == "INIT") {
          core.test_set_last_speed_cmd(num(t[2]));
        } else if (t[1] == "TICK") {
          const TickOutput& o = core.tick(ns);
          ++tick_no;
          ++st.ticks;
          // the next record is EXP
          auto e = split(lines[i++]);
          ASSERT_EQ(e[0], "EXP") << scen;
          const std::string ho = e.back();
          const std::string got_ho = to_string(o.handoff);
          const std::string want_ho = ho == "NONE" ? "NONE" : ho;
          CHECK_EQ(st, "handoff", want_ho, got_ho);
          if (ho != "NONE") {
            ++st.handoffs;
            ++st.handoff_names[ho];
            episode_over = true;
            continue;
          }
          size_t k = 1;
          const bool vel_pub = e[k++] == "1";
          const double vn = num(e[k++]), ve = num(e[k++]), yr = num(e[k++]);
          const bool dbg_valid = e[k++] == "1";
          double d[16];
          for (double& x : d) x = num(e[k++]);
          const bool seg_valid = e[k++] == "1";
          double sg[8];
          for (double& x : sg) x = num(e[k++]);
          const long long seg_n = inum(e[k++]);
          CHECK_EQ(st, "velocity_published", vel_pub, o.velocity_published);
          if (vel_pub) {
            CHECK_NEAR(st, "v_n", vn, o.v_n);
            CHECK_NEAR(st, "v_e", ve, o.v_e);
            CHECK_NEAR(st, "yaw_rate", yr, o.yaw_rate);
          }
          CHECK_EQ(st, "debug_valid", dbg_valid, o.debug_valid);
          if (dbg_valid) {
            const DebugRow& g = o.debug;
            CHECK_NEAR(st, "dbg.cross_track", d[0], g.cross_track);
            CHECK_NEAR(st, "dbg.heading_err", d[1], g.heading_err);
            CHECK_NEAR(st, "dbg.lookahead", d[2], g.lookahead);
            CHECK_NEAR(st, "dbg.speed", d[3], g.speed);
            CHECK_NEAR(st, "dbg.kappa", d[4], g.kappa);
            CHECK_NEAR(st, "dbg.dist_goal", d[5], g.dist_goal);
            CHECK_NEAR(st, "dbg.pose_age_ms", d[6], g.pose_age_ms);
            CHECK_NEAR(st, "dbg.state", d[7], static_cast<double>(g.state));
            CHECK_NEAR(st, "dbg.l_d_raw", d[8], g.l_d_raw);
            CHECK_NEAR(st, "dbg.kappa_speed", d[9], g.kappa_speed);
            CHECK_NEAR(st, "dbg.yaw_rate", d[10], g.yaw_rate);
            CHECK_NEAR(st, "dbg.spray", d[11], g.spray_active ? 1.0 : 0.0);
            CHECK_NEAR(st, "dbg.speed_raw", d[12], g.speed_raw);
            CHECK_NEAR(st, "dbg.v_lat_limit", d[13], g.v_lat_limit);
            CHECK_NEAR(st, "dbg.accel_scale", d[14], g.accel_scale);
            CHECK_NEAR(st, "dbg.speed_mode", d[15], g.speed_mode);
            ++st.states[g.state];
          }
          CHECK_EQ(st, "segment_debug_valid", seg_valid, o.segment_debug_valid);
          CHECK_EQ(st, "segment_debug_publishes", seg_n,
                   static_cast<long long>(o.segment_debug_publishes));
          if (seg_valid) {
            const SegmentDebugRow& g = o.segment_debug;
            CHECK_NEAR(st, "seg.state", sg[0], static_cast<double>(g.state));
            CHECK_NEAR(st, "seg.idx", sg[1], static_cast<double>(g.seg_idx));
            CHECK_NEAR(st, "seg.dist_end", sg[2], g.dist_to_segment_end);
            CHECK_NEAR(st, "seg.dist_corner", sg[3], g.dist_to_corner);
            CHECK_NEAR(st, "seg.corner_angle", sg[4], g.corner_angle_deg);
            CHECK_NEAR(st, "seg.target_heading", sg[5], g.target_heading_ned);
            CHECK_NEAR(st, "seg.heading_err", sg[6], g.heading_error_rad);
            CHECK_NEAR(st, "seg.yaw_rate", sg[7], g.yaw_rate_body);
          }
          ++st.compared;
          // optional state snapshot
          if (i < lines.size() && lines[i].rfind("ST ", 0) == 0) {
            auto s = split(lines[i++]);
            const CoreState c = core.snapshot();
            size_t q = 1;
            CHECK_NEAR(st, "st.last_speed_cmd", num(s[q++]), c.last_speed_cmd);
            CHECK_NEAR(st, "st.last_yaw_cmd", num(s[q++]), c.last_yaw_cmd);
            CHECK_NEAR(st, "st.path_travel_m", num(s[q++]), c.path_travel_m);
            CHECK_NEAR(st, "st.tick_dt", num(s[q++]), c.tick_dt);
            CHECK_EQ(st, "st.segment_idx", inum(s[q++]), static_cast<long long>(c.segment_idx));
            CHECK_EQ(st, "st.run_idx", inum(s[q++]), static_cast<long long>(c.run_idx));
            CHECK_EQ(st, "st.hint_seg", inum(s[q++]), static_cast<long long>(c.hint_seg));
            CHECK_EQ(st, "st.hint_valid", inum(s[q++]), static_cast<long long>(c.hint_valid));
            CHECK_EQ(st, "st.kappa_hard_latched", inum(s[q++]),
                     static_cast<long long>(c.kappa_hard_latched));
            CHECK_EQ(st, "st.stop_latched", inum(s[q++]), static_cast<long long>(c.stop_latched));
            CHECK_EQ(st, "st.entry_spray_hold", inum(s[q++]),
                     static_cast<long long>(c.entry_spray_hold));
            CHECK_EQ(st, "st.path_done", inum(s[q++]), static_cast<long long>(c.path_done));
            CHECK_EQ(st, "st.run_align_pending", inum(s[q++]),
                     static_cast<long long>(c.run_align_pending));
            CHECK_NEAR(st, "st.ekf_off_n", num(s[q++]), c.ekf_offset_n);
            CHECK_NEAR(st, "st.ekf_off_e", num(s[q++]), c.ekf_offset_e);
            CHECK_EQ(st, "st.ekf_reset_count", inum(s[q++]),
                     static_cast<long long>(c.ekf_reset_count));
            CHECK_EQ(st, "st.have_last_pos", inum(s[q++]), static_cast<long long>(c.have_last_pos));
            CHECK_NEAR(st, "st.last_pos_n", num(s[q++]), c.last_pos_n);
            CHECK_NEAR(st, "st.last_pos_e", num(s[q++]), c.last_pos_e);
            CHECK_EQ(st, "st.segment_state", inum(s[q++]), static_cast<long long>(c.segment_state));
            CHECK_EQ(st, "st.rtk_recovering", inum(s[q++]),
                     static_cast<long long>(c.rtk_recovering));
          }
        }
      }
    }
    while (i < lines.size() && lines[i] != "END")
      ++i;  // after a handoff: skip the rest of the episode records
    ++i;
  }

  std::printf(
      "orchestrator: %d scenarios, %d ticks (%d compared, %d ended at a handoff), %d mismatches\n",
      st.scenarios, st.ticks, st.compared, st.handoffs, st.failures);
  for (auto& kv : st.handoff_names) std::printf("  handoff %s x%d\n", kv.first.c_str(), kv.second);
  for (auto& kv : st.states) std::printf("  state %d x%d\n", kv.first, kv.second);
  EXPECT_EQ(st.failures, 0);
  EXPECT_GE(st.scenarios, 50);
  EXPECT_GE(st.compared, 6500);
  // the gates and every tracked state must actually have been exercised
  for (int s : {-1, 0, 1, 2, 4, 5}) EXPECT_GT(st.states[s], 0) << "state " << s << " never reached";
  for (const char* n : {"COMPLETION_HOLD", "RUN_BOUNDARY_HOLD", "ENDPOINT_PRECISE_STOP",
                        "CORNER_STOP_PIVOT", "RUN_ALIGNMENT"})
    EXPECT_GT(st.handoff_names[n], 0) << n << " never reached";
}
