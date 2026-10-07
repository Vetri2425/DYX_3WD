// GATE 4 (spray) equivalence: every pure module here is compared against vectors produced by
// running the VERBATIM PX4_DXP spray modules (tools/gate4/gen_spray_vectors.py). Fixture:
// test/fixtures/gate4_spray_vectors.txt.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "dyx3_spray/boundary_projection.hpp"
#include "dyx3_spray/flow_model.hpp"
#include "dyx3_spray/safety_lease.hpp"
#include "dyx3_spray/spray_fsm.hpp"
#include "dyx3_spray/spray_gates.hpp"

using namespace dyx3_spray;

namespace {

std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> out;
  std::istringstream is(s);
  std::string t;
  while (is >> t) out.push_back(t);
  return out;
}
double num(const std::string& t) { return std::strtod(t.c_str(), nullptr); }
bool flag(const std::string& t) { return t == "1"; }
std::optional<double> opt(const std::string& t) {
  if (t == "-") return std::nullopt;
  return num(t);
}
std::string txt(const std::string& s) {
  if (s.empty()) return "-";
  std::string o = s;
  for (char& c : o)
    if (c == ' ') c = '_';
  return o;
}
::testing::AssertionResult same(double a, double b) {
  if (std::isnan(a) && std::isnan(b)) return ::testing::AssertionSuccess();
  if (a == b) return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << a << " != " << b;
}

// CPython's math.hypot is its own implementation (not libm's): values derived from it agree to a
// few ulp, not bit for bit. Every DECISION (booleans, indices, reason text, event) must agree
// exactly; only the derived distances are compared with a relative tolerance of 1e-12.
::testing::AssertionResult near(double a, double b) {
  if (std::isnan(a) && std::isnan(b)) return ::testing::AssertionSuccess();
  if (std::fabs(a - b) <= 1e-12 * std::max(1.0, std::max(std::fabs(a), std::fabs(b))))
    return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << a << " != " << b;
}

struct Vectors {
  std::vector<std::string> lines;
  size_t i{0};
  bool next(std::string* l) {
    if (i >= lines.size()) return false;
    *l = lines[i++];
    return true;
  }
};

Vectors load() {
  Vectors v;
  std::ifstream f(std::string(DYX3_FIXTURES) + "/gate4_spray_vectors.txt");
  EXPECT_TRUE(f.good());
  std::string l;
  while (std::getline(f, l)) {
    if (l.empty() || l[0] == '#') continue;
    v.lines.push_back(l);
  }
  return v;
}

struct Counts {
  size_t fsm{0}, lease{0}, mon{0}, flow{0}, rtkq{0}, gates{0}, es{0}, gc{0}, dec{0};
} g_counts;

struct NamedPath {
  std::vector<double> n, e;
  std::vector<bool> f;
};

}  // namespace

TEST(SprayEquivalence, AllVectors) {
  Vectors v = load();
  std::map<std::string, NamedPath> paths;
  std::string line;
  ASSERT_TRUE(v.next(&line));
  ASSERT_EQ(line, "GATE4SPRAY 1");
  while (v.next(&line)) {
    const auto t = split(line);
    const std::string& k = t[0];
    if (k == "PATH") {
      NamedPath p;
      const int n = std::atoi(t[2].c_str());
      for (int j = 0; j < n; ++j) {
        v.next(&line);
        const auto q = split(line);
        p.n.push_back(num(q[0]));
        p.e.push_back(num(q[1]));
        p.f.push_back(flag(q[2]));
      }
      v.next(&line);
      ASSERT_EQ(line, "END");
      paths[t[1]] = p;
    } else if (k == "FSM") {
      SpraySafetyStateMachine fsm;
      const int n = std::atoi(t[1].c_str());
      for (int j = 0; j < n; ++j) {
        v.next(&line);
        const size_t arrow = line.find(" => ");
        const auto in = split(line.substr(0, arrow));
        const auto out = split(line.substr(arrow + 4));
        std::optional<SprayCommand> cmd;
        if (in[0] == "T") {
          cmd = fsm.tick(flag(in[2]), flag(in[3]), flag(in[4]), num(in[1]));
        } else if (in[0] == "A") {
          cmd =
              fsm.on_ack(static_cast<uint32_t>(std::atoi(in[2].c_str())), flag(in[3]), num(in[1]));
        } else {
          fsm.note_event_reset(num(in[1]));
        }
        size_t o = 0;
        if (out[o] == "N") {
          ASSERT_FALSE(cmd.has_value()) << line;
          ++o;
        } else {
          ASSERT_TRUE(cmd.has_value()) << line;
          EXPECT_EQ(cmd->on, flag(out[o + 1])) << line;
          EXPECT_EQ(cmd->seq, static_cast<uint32_t>(std::atoi(out[o + 2].c_str()))) << line;
          EXPECT_EQ(cmd->force, flag(out[o + 3])) << line;
          o += 4;
        }
        EXPECT_EQ(std::string(to_string(fsm.state())), out[o]) << line;
        EXPECT_EQ(fsm.spraying(), flag(out[o + 1])) << line;
        EXPECT_EQ(fsm.commanded(), flag(out[o + 2])) << line;
        EXPECT_EQ(fsm.cmd_seq(), static_cast<uint32_t>(std::atoi(out[o + 3].c_str()))) << line;
        ++g_counts.fsm;
      }
      v.next(&line);
    } else if (k == "V") {
      const size_t arrow = line.find(" => ");
      const auto in = split(line.substr(0, arrow));
      Lease l;
      l.allow_on = flag(in[1]);
      l.command_seq = std::atoll(in[2].c_str());
      l.backend = std::atoi(in[3].c_str());  // 2 = bogus
      l.actuator_set_index = std::atoi(in[4].c_str());
      l.off_value = num(in[5]);
      l.servo_instance = std::atoi(in[6].c_str());
      l.off_pwm_us = std::atoi(in[7].c_str());
      EXPECT_EQ(validate_lease(l).empty(), flag(split(line.substr(arrow + 4))[0])) << line;
      ++g_counts.lease;
    } else if (k == "MON") {
      LeaseMonitor mon(num(t[1]));
      while (v.next(&line) && line != "END") {
        const auto in = split(line.substr(0, line.find(" => ")));
        if (in[0] == "O") {
          Lease l;
          l.allow_on = flag(in[2]);
          l.command_seq = std::atoll(in[3].c_str());
          l.backend = std::atoi(in[4].c_str());
          l.actuator_set_index = std::atoi(in[5].c_str());
          l.off_value = num(in[6]);
          l.servo_instance = std::atoi(in[7].c_str());
          l.off_pwm_us = std::atoi(in[8].c_str());
          ASSERT_TRUE(mon.observe(l, num(in[1]))) << line;
        } else if (in[0] == "I") {
          Lease bad;
          bad.backend = 9;
          std::string err;
          ASSERT_FALSE(mon.observe(bad, num(in[1]), &err));
        } else {
          const OffReason r = mon.off_reason(num(in[1]));
          std::string got = r.required() ? txt(r.text) : "-";
          if (r.cause == OffCause::Invalidated) got = "INVALID";
          EXPECT_EQ(got, split(line.substr(line.find(" => ") + 4))[0]) << line;
          ++g_counts.mon;
        }
      }
    } else if (k == "FLOW") {
      FlowModulator fm(num(t[1]), num(t[2]), num(t[3]), num(t[4]));
      while (v.next(&line) && line != "END") {
        if (line == "R") {
          fm.reset();
          continue;
        }
        const size_t arrow = line.find(" => ");
        const auto in = split(line.substr(0, arrow));
        const auto out = split(line.substr(arrow + 4));
        const double val = fm.update(num(in[1]), num(in[2]));
        EXPECT_TRUE(same(val, num(out[0]))) << line;
        EXPECT_TRUE(same(fm.value(), num(out[1]))) << line;
        ++g_counts.flow;
      }
    } else if (k == "Q") {
      const size_t arrow = line.find(" => ");
      const auto in = split(line.substr(0, arrow));
      const auto out = split(line.substr(arrow + 4));
      const RtkQuality q =
          evaluate_rtk_quality(std::atoi(in[1].c_str()), opt(in[2]), opt(in[3]), num(in[4]),
                               std::atoi(in[5].c_str()), num(in[6]), flag(in[7]));
      EXPECT_EQ(q.fresh, flag(out[0])) << line;
      EXPECT_EQ(q.acceptable, flag(out[1])) << line;
      EXPECT_EQ(txt(q.fix_name), out[2]) << line;
      EXPECT_EQ(txt(q.reason), out[3]) << line;
      ++g_counts.rtkq;
    } else if (k == "GS") {
      RtkGateConfig rc;
      const bool require_offboard = flag(t[1]);
      rc.require_rtk_fix = flag(t[2]);
      rc.min_fix_type = std::atoi(t[3].c_str());
      rc.max_h_acc_m = num(t[4]);
      rc.require_accuracy = flag(t[5]);
      rc.fix_timeout_s = num(t[6]);
      rc.recover_hold_s = num(t[7]);
      const bool off_during_pivot = flag(t[8]);
      const double seg_timeout = num(t[9]);
      v.next(&line);
      const int n = std::atoi(line.c_str());
      RtkGate gate;
      PivotGate piv;
      for (int j = 0; j < n; ++j) {
        v.next(&line);
        const size_t arrow = line.find(" => ");
        const auto in = split(line.substr(0, arrow));
        const auto out = split(line.substr(arrow + 4));
        const double now = num(in[1]);
        if (in[5] != "-") piv.note_state(in[5] == "1", now);
        GateInputs gi;
        gi.armed = gi.offboard = gi.path_loaded = gi.pose_fresh = gi.velocity_fresh =
            gi.tracking_seen = true;
        gi.estop_clear = true;
        gi.require_offboard = require_offboard;
        gi.rtk = gate.evaluate(rc, std::atoi(in[2].c_str()), opt(in[3]), opt(in[4]), now);
        gi.pivoting = piv.active(off_during_pivot, seg_timeout, now);
        const GateResult res = auto_safety_status(gi);
        EXPECT_EQ(res.ok, flag(out[0])) << line;
        EXPECT_EQ(txt(res.reason), out[1]) << line;
        ++g_counts.gates;
      }
      v.next(&line);
    } else if (k == "E") {
      const size_t arrow = line.find(" => ");
      const auto in = split(line.substr(0, arrow));
      const auto out = split(line.substr(arrow + 4));
      GateInputs gi;
      gi.require_offboard = flag(in[1]);
      gi.armed = flag(in[2]);
      gi.offboard = flag(in[3]);
      gi.path_loaded = flag(in[4]);
      gi.pose_fresh = flag(in[5]);
      gi.velocity_fresh = flag(in[6]);
      gi.tracking_seen = true;
      gi.estop_clear = true;
      gi.rtk = {true, ""};
      const GateResult res = auto_safety_status(gi);
      EXPECT_EQ(res.ok, flag(out[0])) << line;
      EXPECT_EQ(txt(res.reason), out[1]) << line;
      ++g_counts.es;
    } else if (k == "GC") {
      const auto in = split(line.substr(0, line.find(" => ")));
      const auto out = split(line.substr(line.find(" => ") + 4));
      EXPECT_TRUE(same(direction_gate_cos(num(in[1])), num(out[0]))) << line;
      ++g_counts.gc;
    } else if (k == "DEC") {
      const NamedPath& np = paths.at(t[1]);
      PathModel model;
      ASSERT_TRUE(build_path_model(np.n, np.e, np.f, &model));
      DecisionParams p;
      p.solenoid_open_delay_s = num(t[2]);
      p.solenoid_close_delay_s = num(t[3]);
      p.on_overspray_margin_m = num(t[4]);
      p.off_overspray_margin_m = num(t[5]);
      p.max_xtrack_error_m = num(t[6]);
      p.xtrack_trip_error_m = num(t[7]);
      p.xtrack_gate_min_off_s = num(t[8]);
      p.terminal_off_epsilon_m = num(t[9]);
      p.terminal_off_speed_mps = num(t[10]);
      p.projection_window_back_m = num(t[11]);
      p.projection_window_fwd_m = num(t[12]);
      p.projection_reacquire_dist_m = num(t[13]);
      p.projection_direction_gate_cos = direction_gate_cos(num(t[14]));
      p.max_xtrack_from_mission = flag(t[15]);
      const int n = std::atoi(t[16].c_str());
      DecisionState st;
      std::optional<double> trip_t;
      for (int j = 0; j < n; ++j) {
        v.next(&line);
        const size_t arrow = line.find(" => ");
        const auto in = split(line.substr(0, arrow));
        const std::string rest = line.substr(arrow + 4);
        // rest: "<d> <g> <s> <reason> <tripped> <event> <dist> | <proj> | <boundary>"
        const size_t bar1 = rest.find(" | ");
        const size_t bar2 = rest.find(" | ", bar1 + 3);
        const auto head = split(rest.substr(0, bar1));
        const auto proj = split(rest.substr(bar1 + 3, bar2 - bar1 - 3));
        const auto nb = split(rest.substr(bar2 + 3));
        const double now = num(in[1]);
        DecisionInput di;
        di.model = &model;
        di.nozzle_n = opt(in[2]);
        di.nozzle_e = opt(in[3]);
        di.speed_mps = num(in[4]);
        di.yaw_rad = num(in[5]);
        di.safety_ok = flag(in[6]);
        di.safety_reason = in[7] == "-" ? "" : in[7];
        st.xtrack_tripped_elapsed_s = trip_t ? now - *trip_t : 1e300;
        const Decision d = make_decision(di, p, st);
        if (d.projection) st.prev_projection_s = d.projection->s;
        if (d.xtrack_tripped && !st.xtrack_tripped) trip_t = now;
        st.xtrack_tripped = d.xtrack_tripped;

        EXPECT_EQ(d.desired, flag(head[0])) << line;
        EXPECT_EQ(d.geometry_desired, flag(head[1])) << line;
        EXPECT_EQ(d.safety_ok, flag(head[2])) << line;
        EXPECT_EQ(txt(d.safety_reason), head[3]) << line;
        EXPECT_EQ(d.xtrack_tripped, flag(head[4])) << line;
        EXPECT_EQ(txt(to_string(d.event)), head[5]) << line;
        if (head[6] == "inf") {
          EXPECT_FALSE(std::isfinite(d.distance_to_boundary_m)) << line;
        } else {
          EXPECT_TRUE(near(d.distance_to_boundary_m, num(head[6]))) << line;
        }
        if (proj[0] == "-") {
          EXPECT_FALSE(d.projection.has_value()) << line;
        } else {
          ASSERT_TRUE(d.projection.has_value()) << line;
          EXPECT_TRUE(near(d.projection->s, num(proj[0]))) << line;
          EXPECT_TRUE(near(d.projection->xtrack_error_m, num(proj[1]))) << line;
          EXPECT_EQ(d.projection->current_flag, flag(proj[2])) << line;
          EXPECT_EQ(d.projection->segment_index, std::atoi(proj[3].c_str())) << line;
          EXPECT_TRUE(near(d.projection->t, num(proj[4]))) << line;
        }
        if (nb[0] == "-") {
          EXPECT_FALSE(d.next_boundary.has_value()) << line;
        } else {
          ASSERT_TRUE(d.next_boundary.has_value()) << line;
          EXPECT_EQ(d.next_boundary->kind == BoundaryKind::TransitToMark ? "T" : "M", nb[0])
              << line;
          EXPECT_TRUE(near(d.next_boundary->s, num(nb[1]))) << line;
        }
        ++g_counts.dec;
        if (::testing::Test::HasFailure())
          return;  // one failing line is enough to read; do not drown the log
      }
      v.next(&line);
    } else {
      FAIL() << "unknown record: " << line;
    }
    if (::testing::Test::HasFailure()) return;
  }
  // The vectors must actually exercise every module (a silently empty fixture proves nothing).
  EXPECT_GT(g_counts.fsm, 10000U);
  EXPECT_GT(g_counts.lease, 500U);
  EXPECT_GT(g_counts.mon, 500U);
  EXPECT_GT(g_counts.flow, 1000U);
  EXPECT_GT(g_counts.rtkq, 2000U);
  EXPECT_GT(g_counts.gates, 5000U);
  EXPECT_GT(g_counts.es, 300U);
  EXPECT_GT(g_counts.gc, 10U);
  EXPECT_GT(g_counts.dec, 4000U);
  std::printf(
      "spray equivalence: fsm %zu lease %zu monitor %zu flow %zu rtk %zu gates %zu early %zu "
      "gate_cos %zu decisions %zu\n",
      g_counts.fsm, g_counts.lease, g_counts.mon, g_counts.flow, g_counts.rtkq, g_counts.gates,
      g_counts.es, g_counts.gc, g_counts.dec);
}
