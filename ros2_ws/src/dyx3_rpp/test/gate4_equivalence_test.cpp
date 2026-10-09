// Module-level equivalence of the C++ against the VERBATIM PX4_DXP ancestors.
// Vectors: tools/gate4/gen_rpp_vectors.py (fixtures/gate4_rpp_vectors.txt). The expected values are
// produced by the carried controller module, not written by hand.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>

#include "dyx3_rpp/guidance.hpp"
#include "dyx3_rpp/speed_profile.hpp"
#include "dyx3_rpp/stop_pivot_fsm.hpp"
#include "dyx3_rpp/terminal.hpp"

using namespace dyx3_rpp;

namespace {

constexpr double kTol = 1e-9;  // relative + absolute; libm differs in the last ULPs between hosts

struct Stats {
  size_t cases{0};
  double worst{0.0};
};
std::map<std::string, Stats> g_stats;

void close(double got, double want, const std::string& what, const std::string& line) {
  const double err = std::fabs(got - want);
  const double lim = kTol * std::max(1.0, std::fabs(want));
  if (std::isnan(want)) {
    EXPECT_TRUE(std::isnan(got)) << what << "\n" << line;
    return;
  }
  auto& s = g_stats[what];
  s.worst = std::max(s.worst, err);
  EXPECT_LE(err, lim) << what << " got " << got << " want " << want << "\n" << line;
}

struct Tok {
  std::istringstream in;
  explicit Tok(const std::string& s) : in(s) {}
  double d() {
    std::string t;
    in >> t;
    return std::strtod(t.c_str(), nullptr);
  }
  int i() {
    std::string t;
    in >> t;
    return std::atoi(t.c_str());
  }
  bool b() { return i() != 0; }
  std::string s() {
    std::string t;
    in >> t;
    return t;
  }
};

using Path = std::vector<Point>;
std::map<std::string, Path> g_paths;

std::string trim_arrow(const std::string& line, std::string* rhs) {
  const auto p = line.find(" -> ");
  *rhs = line.substr(p + 4);
  return line.substr(0, p);
}

StopPivotParams stop_params(double thr, double ythr, double dwell) {
  StopPivotParams p;
  p.stop_speed_threshold = thr;
  p.stop_yaw_rate_threshold = ythr;
  p.stop_dwell_s = dwell;
  return p;
}

}  // namespace

TEST(Gate4Equivalence, AgainstVerbatimAncestors) {
  std::ifstream f(DYX3_FIXTURES "/gate4_rpp_vectors.txt");
  ASSERT_TRUE(f.good()) << "fixture missing";
  std::string line;
  std::getline(f, line);
  ASSERT_EQ(line, "GATE4 1");

  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    const std::string kind = line.substr(0, line.find(' '));
    if (kind == "PATH") {
      Tok t(line);
      t.s();
      const std::string name = t.s();
      const int n = t.i();
      Path p;
      for (int k = 0; k < n; ++k) {
        std::getline(f, line);
        Tok c(line);
        const double a = c.d();
        const double b = c.d();
        p.push_back({a, b});
      }
      std::getline(f, line);  // END
      g_paths[name] = std::move(p);
      continue;
    }
    if (kind == "SEQ") {
      Tok h(line);
      h.s();
      const std::string sk = h.s();
      if (sk == "STOP") {
        const double thr = h.d(), ythr = h.d(), dwell = h.d();
        const int n = h.i();
        StopConfirm sc;
        const auto p = stop_params(thr, ythr, dwell);
        // DOCUMENTED DEVIATION (XR-RPP-011): the ancestor counts the stale-velocity cap from the
        // hold entry; the C++ counts it from the first stale tick of the hold. The expected value
        // is the ancestor's, except that a stale tick confirms only after the velocity has been
        // stale for the cap. Every such tick is counted ("STOP seq XR-RPP-011"); no other
        // difference is allowed.
        bool stale_run = false;
        int64_t stale_since = 0;
        for (int k = 0; k < n; ++k) {
          std::getline(f, line);
          std::string rhs;
          Tok c(trim_arrow(line, &rhs));
          c.s();
          const double t = c.d(), age = c.d(), vn = c.d(), ve = c.d(), yr = c.d();
          StopTelemetry tel;
          tel.vel_fresh = age < 0.3;
          tel.speed = std::hypot(vn, ve);
          tel.yaw_rate = yr;
          const int64_t ns = std::llround(t * 1e9);
          // The Python freshness is (now - vel_time) from ns integers; recompute it the same way.
          const int64_t vns = std::llround((t - age) * 1e9);
          tel.vel_fresh = static_cast<double>(ns - vns) * 1e-9 < 0.3;
          const bool ancestor = Tok(rhs).b();
          bool want = ancestor;
          if (tel.vel_fresh) {
            stale_run = false;
          } else {
            if (!stale_run) {
              stale_run = true;
              stale_since = ns;
            }
            if (ancestor && static_cast<double>(ns - stale_since) * 1e-9 < p.stale_vel_hold_s) {
              want = false;
              ++g_stats["STOP seq XR-RPP-011"].cases;
            }
          }
          EXPECT_EQ(sc.satisfied(ns, tel, p), want) << "STOP seq line: " << line;
          ++g_stats["STOP seq"].cases;
        }
        std::getline(f, line);  // END
      } else if (sk == "PIVOT") {
        StopPivotParams p;
        p.turn_timeout_s = h.d();
        p.nominal_pivot_rate = h.d();
        p.spinup_margin_s = h.d();
        p.pivot_timeout_max_s = h.d();
        const int n = h.i();
        PivotWatchdog w;
        for (int k = 0; k < n; ++k) {
          std::getline(f, line);
          std::string rhs;
          Tok c(trim_arrow(line, &rhs));
          c.s();
          const double t = c.d(), ang = c.d();
          Tok r(rhs);
          const bool want = r.b();
          const double budget = r.d();
          EXPECT_EQ(w.timed_out(std::llround(t * 1e9), ang, p), want) << line;
          close(w.budget_s(p), budget, "pivot budget", line);
          ++g_stats["PIVOT seq"].cases;
        }
        std::getline(f, line);  // END
      }
      continue;
    }
    std::string rhs;
    Tok in(trim_arrow(line, &rhs));
    in.s();
    Tok out(rhs);
    ++g_stats[kind].cases;
    if (kind == "ACC") {
      const double he = in.d(), cv = in.d(), fh = in.d(), nh = in.d(), fc = in.d(), nc = in.d();
      close(alignment_accel_scale(he, cv, fh, nh, fc, nc), out.d(), kind, line);
    } else if (kind == "LATCH") {
      const bool lat = in.b();
      const double k = in.d(), en = in.d(), ex = in.d();
      EXPECT_EQ(update_kappa_hard_latch(lat, k, en, ex), out.b()) << line;
    } else if (kind == "SLEW") {
      const double raw = in.d(), last = in.d(), dt = in.d();
      const bool lat = in.b();
      const double dec = in.d(), acc = in.d(), sc = in.d();
      const bool ap = in.b();
      const double p4 = in.d();
      const auto r = apply_smooth_speed_slew(raw, last, dt, lat, dec, acc, sc, ap, p4);
      close(r.speed, out.d(), kind, line);
      EXPECT_EQ(r.mode, out.i()) << line;
    } else if (kind == "SLOOK") {
      const Path& p = g_paths.at(in.s().substr(1));
      const int seg = in.i();
      const double fn = in.d(), fe = in.d(), ld = in.d();
      const auto r = smooth_lookahead_point(p, seg, {fn, fe}, ld);
      close(r.p.n, out.d(), "SLOOK n", line);
      close(r.p.e, out.d(), "SLOOK e", line);
      EXPECT_EQ(r.hit_end, out.b()) << line;
    } else if (kind == "SEGLOOK") {
      const Path& p = g_paths.at(in.s().substr(1));
      const int seg = in.i();
      const double fn = in.d(), fe = in.d(), ld = in.d(), mj = in.d();
      const bool ee = in.b(), ec = in.b();
      const auto r = segment_lookahead_point(p, seg, {fn, fe}, ld, mj, ee, ec);
      close(r.n, out.d(), "SEGLOOK n", line);
      close(r.e, out.d(), "SEGLOOK e", line);
    } else if (kind == "PIVINT") {
      const double px = in.d(), py = in.d(), ax = in.d(), ay = in.d(), bx = in.d(), by = in.d(),
                   leg = in.d();
      const bool en = in.b();
      const double di = in.d();
      close(pivot_intercept_heading({px, py}, {ax, ay}, {bx, by}, leg, en, di), out.d(), kind,
            line);
    } else if (kind == "CLOSED") {
      const Path& p = g_paths.at(in.s().substr(1));
      const double thr = in.d(), ml = in.d();
      EXPECT_EQ(is_closed_run(p, thr, ml), out.b()) << line;
    } else if (kind == "TAIL") {
      const Path& p = g_paths.at(in.s().substr(1));
      const int n = in.i();
      std::vector<bool> fl;
      for (int k = 0; k < n; ++k) fl.push_back(in.b());
      close(measure_tail_transit_m(p, fl), out.d(), kind, line);
    } else if (kind == "PROGRESS") {
      const int n = in.i();
      std::vector<double> cum;
      for (int k = 0; k < n; ++k) cum.push_back(in.d());
      const int path_size = in.i();
      const int seg = in.i();
      const double t = in.d();
      close(path_progress_at(cum, static_cast<size_t>(path_size), seg, t), out.d(), kind, line);
    } else if (kind == "REMAIN") {
      RunInfo run;
      run.valid = in.b();
      run.length = in.d();
      const int n = in.i();
      std::vector<double> cum;
      for (int k = 0; k < n; ++k) cum.push_back(in.d());
      const int path_size = in.i();
      const double travel = in.d();
      const auto r = run_remaining_along(run, cum, static_cast<size_t>(path_size), travel);
      const bool has = out.b();
      EXPECT_EQ(r.has_value(), has) << line;
      const double want = out.d();
      if (r && has) close(*r, want, kind, line);
    } else if (kind == "MINTRAVEL") {
      RunInfo run;
      run.valid = in.b();
      run.length = in.d();
      run.closed = in.b();
      const double mg = in.d(), fr = in.d();
      close(run_min_travel(run, mg, fr), out.d(), kind, line);
    } else if (kind == "GOALTOL") {
      const double gt = in.d(), tail = in.d(), ro = in.d();
      close(goal_tol_effective(gt, tail, ro), out.d(), kind, line);
    } else if (kind == "CAPTURE") {
      const Path& p = g_paths.at(in.s().substr(1));
      const double px = in.d(), py = in.d();
      const bool has = in.b();
      const double rem = in.d();
      CaptureParams cp;
      cp.enabled = in.b();
      cp.goal_tol = in.d();
      cp.tail_transit_m = in.d();
      cp.transit_runout_goal_tolerance_m = in.d();
      cp.past_m = in.d();
      cp.max_miss_m = in.d();
      const auto v = endpoint_capture_recovered(
          p, {px, py}, has ? std::optional<double>(rem) : std::nullopt, cp);
      EXPECT_EQ(static_cast<int>(v), out.i()) << line;
    } else if (kind == "BRAKE") {
      const bool fresh = in.b();
      const double vn = in.d(), ve = in.d(), yaw = in.d(), cap = in.d(), thr = in.d();
      StopTelemetry tel;
      tel.vel_fresh = fresh;
      tel.speed = std::hypot(vn, ve);
      tel.v_forward = vn * std::cos(yaw) + ve * std::sin(yaw);
      StopPivotParams p;
      p.brake_velocity_cap = cap;
      p.stop_speed_threshold = thr;
      const double s = brake_speed(tel, p);
      close(s * std::cos(yaw), out.d(), "BRAKE n", line);
      close(s * std::sin(yaw), out.d(), "BRAKE e", line);
    } else {
      FAIL() << "unknown vector kind " << kind;
    }
  }
  size_t total = 0;
  for (const auto& kv : g_stats) {
    if (kv.first != "STOP seq XR-RPP-011") total += kv.second.cases;  // a subset of "STOP seq"
    std::printf("  %-14s cases %6zu  worst abs err %.3g\n", kv.first.c_str(), kv.second.cases,
                kv.second.worst);
  }
  EXPECT_GT(total, 15000U);
}
