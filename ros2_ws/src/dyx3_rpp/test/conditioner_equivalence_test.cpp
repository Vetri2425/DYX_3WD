// GATE 4 (path conditioner): every function and the whole run-building of _path_cb compared with
// vectors produced by the VERBATIM PX4_DXP code (tools/gate4/gen_conditioner_vectors.py). Selection
// logic (which points survive, run structure, profile, flags) must match exactly; computed
// coordinates and lengths to 1e-9 (libm vs CPython differences in hypot/acos/tan).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "dyx3_rpp/path_conditioner.hpp"

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

struct In {
  std::vector<Point> pts;
  Flags flags;
  std::vector<unsigned char> must;
  KeySet keys;
};

struct Reader {
  std::vector<std::string> lines;
  size_t i{0};
  bool next(std::string* l) {
    if (i >= lines.size()) return false;
    *l = lines[i++];
    return true;
  }
};

::testing::AssertionResult near(double a, double b) {
  if (std::fabs(a - b) <= 1e-9 * std::max(1.0, std::max(std::fabs(a), std::fabs(b))))
    return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << a << " != " << b;
}

void expect_points(Reader& rd, const std::vector<Point>& pts, const Flags& flags,
                   const std::string& what, bool exact) {
  std::string l;
  rd.next(&l);
  auto t = split(l);
  ASSERT_EQ(t[0], "=>") << what;
  ASSERT_EQ(static_cast<size_t>(std::atoi(t[1].c_str())), pts.size()) << what << ": point count";
  ASSERT_EQ(pts.size(), flags.size()) << what;
  for (size_t k = 0; k < pts.size(); ++k) {
    rd.next(&l);
    t = split(l);
    if (exact) {
      EXPECT_EQ(pts[k].n, num(t[0])) << what << " pt " << k;
      EXPECT_EQ(pts[k].e, num(t[1])) << what << " pt " << k;
    } else {
      EXPECT_TRUE(near(pts[k].n, num(t[0]))) << what << " pt " << k;
      EXPECT_TRUE(near(pts[k].e, num(t[1]))) << what << " pt " << k;
    }
    EXPECT_EQ(static_cast<int>(flags[k]), std::atoi(t[2].c_str())) << what << " flag " << k;
  }
}

void expect_runs(Reader& rd, const std::vector<PointRun>& runs, const std::string& what,
                 bool exact) {
  std::string l;
  rd.next(&l);
  auto t = split(l);
  ASSERT_EQ(t[0], "=>") << what;
  ASSERT_EQ(static_cast<size_t>(std::atoi(t[1].c_str())), runs.size()) << what << ": run count";
  for (size_t r = 0; r < runs.size(); ++r) {
    rd.next(&l);
    t = split(l);
    ASSERT_EQ(t[0], "RUN");
    ASSERT_EQ(static_cast<size_t>(std::atoi(t[1].c_str())), runs[r].pts.size())
        << what << " run " << r;
    for (size_t k = 0; k < runs[r].pts.size(); ++k) {
      rd.next(&l);
      t = split(l);
      if (exact) {
        EXPECT_EQ(runs[r].pts[k].n, num(t[0])) << what << " run " << r << " pt " << k;
        EXPECT_EQ(runs[r].pts[k].e, num(t[1])) << what;
      } else {
        EXPECT_TRUE(near(runs[r].pts[k].n, num(t[0]))) << what;
        EXPECT_TRUE(near(runs[r].pts[k].e, num(t[1]))) << what;
      }
      EXPECT_EQ(static_cast<int>(runs[r].flags[k]), std::atoi(t[2].c_str())) << what << " flag";
    }
  }
}

}  // namespace

TEST(ConditionerEquivalence, AllVectors) {
  Reader rd;
  {
    std::ifstream f(std::string(DYX3_FIXTURES) + "/gate4_conditioner_vectors.txt");
    ASSERT_TRUE(f.good());
    std::string l;
    while (std::getline(f, l)) {
      if (!l.empty() && l[0] != '#') rd.lines.push_back(l);
    }
  }
  std::map<std::string, In> inputs;
  std::string line;
  ASSERT_TRUE(rd.next(&line));
  ASSERT_EQ(line, "GATE4COND 1");
  size_t n_cases = 0, n_pathcb = 0, n_pts = 0;
  while (rd.next(&line)) {
    const auto t = split(line);
    if (t[0] == "PATH") {
      In in;
      const int n = std::atoi(t[2].c_str());
      for (int k = 0; k < n; ++k) {
        rd.next(&line);
        const auto q = split(line);
        in.pts.push_back({num(q[0]), num(q[1])});
        in.flags.push_back(static_cast<unsigned char>(std::atoi(q[2].c_str())));
        in.must.push_back(static_cast<unsigned char>(std::atoi(q[3].c_str())));
        if (q[3] == "1") in.keys.insert(pt_key(in.pts.back()));
      }
      rd.next(&line);
      ASSERT_EQ(line, "END");
      inputs[t[1]] = in;
      continue;
    }
    ASSERT_EQ(t[0], "CASE") << line;
    const std::string kind = t[1];
    const In& in = inputs.at(t.back().substr(1));
    ++n_cases;
    n_pts += in.pts.size();
    if (kind == "simplify") {
      const PointRun r = simplify_for_profile(in.pts, &in.flags, num(t[2]), num(t[3]), &in.keys);
      expect_points(rd, r.pts, r.flags, line, true);
    } else if (kind == "classify") {
      rd.next(&line);
      const auto o = split(line);
      EXPECT_EQ(std::string(to_string(classify_auto_profile(in.pts, num(t[2])))), o[1]) << t[2];
    } else if (kind == "split_flags") {
      expect_runs(rd, split_runs_by_flag(in.pts, in.flags), line, true);
    } else if (kind == "absorb") {
      const PointRun r = absorb_short_connectors(in.pts, in.flags, num(t[2]), num(t[3]), num(t[4]));
      expect_points(rd, r.pts, r.flags, line, false);
    } else if (kind == "split_corners") {
      expect_runs(rd, split_run_at_corners(in.pts, in.flags, num(t[2])), line, true);
    } else if (kind == "smooth") {
      const PointRun r = smooth_corners(in.pts, num(t[2]), std::atoi(t[3].c_str()), &in.flags);
      expect_points(rd, r.pts, r.flags, line, false);
    } else if (kind == "closed") {
      rd.next(&line);
      const auto o = split(line);
      EXPECT_EQ(is_closed_run(in.pts, num(t[2]), num(t[3])), o[1] == "1");
    } else if (kind == "merge") {
      expect_runs(rd,
                  merge_collinear_runs(split_runs_by_flag(in.pts, in.flags), num(t[2]), num(t[3])),
                  line, false);
    } else if (kind == "path_cb") {
      ++n_pathcb;
      ConditionParams p;
      p.tracking_profile = t[2];
      p.segment_corner_threshold_deg = num(t[3]);
      p.connector_absorb_m = num(t[4]);
      p.connector_min_corner_deg = num(t[5]);
      p.transit_merge_max_len_m = num(t[6]);
      p.segment_simplify_max_offset_m = num(t[7]);
      p.corner_smooth_radius_m = num(t[8]);
      p.corner_smooth_arc_pts = std::atoi(t[9].c_str());
      p.path_resample_spacing_m = num(t[10]);
      p.close_loop_threshold_m = num(t[11]);
      p.close_loop_min_len_m = num(t[12]);
      std::vector<RawPoint> raw;
      for (size_t k = 0; k < in.pts.size(); ++k)
        raw.push_back({in.pts[k].n, in.pts[k].e, (in.flags[k] ? 1 : 0) | (in.must[k] ? 2 : 0)});
      const auto runs = condition_path(raw, p);
      rd.next(&line);
      auto o = split(line);
      ASSERT_EQ(o[0], "=>");
      ASSERT_EQ(static_cast<size_t>(std::atoi(o[1].c_str())), runs.size())
          << "run count for " << t[2];
      for (size_t r = 0; r < runs.size(); ++r) {
        rd.next(&line);
        o = split(line);
        ASSERT_EQ(o[0], "RUN");
        EXPECT_EQ(std::string(to_string(runs[r].profile)), o[1]);
        EXPECT_TRUE(near(runs[r].length, num(o[2])));
        EXPECT_EQ(runs[r].closed, o[3] == "1");
        ASSERT_EQ(static_cast<size_t>(std::atoi(o[4].c_str())), runs[r].pts.size());
        for (size_t k = 0; k < runs[r].pts.size(); ++k) {
          rd.next(&line);
          o = split(line);
          EXPECT_TRUE(near(runs[r].pts[k].n, num(o[0])));
          EXPECT_TRUE(near(runs[r].pts[k].e, num(o[1])));
          EXPECT_EQ(static_cast<int>(runs[r].flags[k]), std::atoi(o[2].c_str()));
          EXPECT_EQ(static_cast<int>(runs[r].must_hit[k]), std::atoi(o[3].c_str()));
          EXPECT_TRUE(near(runs[r].cum_s[k], num(o[4])));
        }
      }
    } else {
      FAIL() << "unknown case " << kind;
    }
    rd.next(&line);
    ASSERT_EQ(line, "END");
    if (::testing::Test::HasFailure()) return;
  }
  EXPECT_GT(n_cases, 1500U);
  EXPECT_GT(n_pathcb, 150U);
  std::printf(
      "conditioner equivalence: %zu cases (%zu end-to-end _path_cb) over %zu input points\n",
      n_cases, n_pathcb, n_pts);
}
