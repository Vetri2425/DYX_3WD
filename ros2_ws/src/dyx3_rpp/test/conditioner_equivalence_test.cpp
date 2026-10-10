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

#include "dyx3_mission/path_artifact.hpp"
#include "dyx3_mission/sha256.hpp"
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

// DELIBERATE DIVERGENCE FROM THE PROTOTYPE (docs/contracts/rpp_path_conditioner.md, "Must-hit
// divergence"). The recorded end-to-end vectors encode the prototype's defect: it moved or deleted
// must-hit vertices in smooth runs and in absorbed connectors. A recorded `path_cb` case is
// EXCLUDED from the vector comparison iff its must-hit set can change the result, i.e. iff
//   (a) the profile is not forced to "segment" and, replaying the pipeline stages on the input,
//       absorb_short_connectors on some spray-flag run returns a different polyline with the case's
//       must-hit keys than without them (a must-hit vertex sits in a connector that the prototype
//       absorbs); or
//   (b) a run that is conditioned as SMOOTH (forced, or auto-classified) has a must-hit vertex that
//       is neither its first nor its last point, while smooth_corners or resample is enabled
//       (the prototype rounds it off or resamples it away).
// Every other case (no must-hit, must-hit only in segment runs, only at run endpoints, or only in
// connectors the prototype leaves alone) is still compared with the prototype, bit for bit as
// before. Excluded cases are not skipped silently: they must satisfy the new contract instead
// (every must-hit input vertex present at exactly its coordinates, flagged), and they are counted.
bool diverges_by_must_hit(const In& in, const ConditionParams& p) {
  if (in.keys.empty()) return false;
  const std::string requested = normalize_tracking_profile(p.tracking_profile);
  std::vector<PointRun> runs;
  if (requested == "auto") {
    for (const PointRun& run : split_runs_by_flag(in.pts, in.flags)) {
      const PointRun legacy =
          absorb_short_connectors(run.pts, run.flags, p.segment_corner_threshold_deg,
                                  p.connector_absorb_m, p.connector_min_corner_deg);
      const PointRun now =
          absorb_short_connectors(run.pts, run.flags, p.segment_corner_threshold_deg,
                                  p.connector_absorb_m, p.connector_min_corner_deg, &in.keys);
      if (legacy.pts.size() != now.pts.size()) return true;
      for (size_t k = 0; k < legacy.pts.size(); ++k) {
        if (legacy.pts[k].n != now.pts[k].n || legacy.pts[k].e != now.pts[k].e) return true;
      }
      for (PointRun& sub :
           split_run_at_corners(legacy.pts, legacy.flags, p.segment_corner_threshold_deg))
        runs.push_back(std::move(sub));
    }
    runs = merge_collinear_runs(runs, p.segment_corner_threshold_deg, p.transit_merge_max_len_m);
  } else {
    runs.push_back({in.pts, in.flags});
  }
  if (!(p.corner_smooth_radius_m > 0.0 || p.path_resample_spacing_m > 0.0)) return false;
  for (const PointRun& rr : runs) {
    const bool smooth =
        requested != "auto"
            ? requested == "smooth"
            : classify_auto_profile(rr.pts, p.segment_corner_threshold_deg) == Profile::Smooth;
    if (!smooth) continue;
    for (size_t k = 1; k + 1 < rr.pts.size(); ++k) {
      if (in.keys.count(pt_key(rr.pts[k])) > 0) return true;
    }
  }
  return false;
}

}  // namespace

TEST(ConditionedExecutionArtifact, RoundTripsTheExactRppRunGeometryAndFlags) {
  const std::vector<RawPoint> raw{{0.0, 0.0, 0}, {1.0, 0.0, 0}, {2.0, 0.0, 1}, {3.0, 0.0, 1}};
  ConditionParams params{"segment", 45.0, 0.2, 20.0, 2.0, 0.01, 0.5, 6, 0.08, 0.15, 1.0};
  const auto conditioned = condition_path(raw, params);
  ASSERT_FALSE(conditioned.empty());
  std::vector<dyx3_mission::ConditionedRunArtifact> artifact_runs;
  for (const auto& run : conditioned) {
    dyx3_mission::ConditionedRunArtifact encoded;
    encoded.profile = static_cast<uint8_t>(run.profile);
    for (size_t i = 0; i < run.pts.size(); ++i) {
      encoded.points.push_back({run.pts[i].n, run.pts[i].e});
      encoded.flags.push_back(run.flags[i]);
      encoded.must_hit.push_back(run.must_hit[i]);
    }
    artifact_runs.push_back(std::move(encoded));
  }
  const std::string source_sha(64, 'c');
  const auto bytes =
      dyx3_mission::serialize_conditioned_artifact(source_sha, "profile=segment", artifact_runs);
  const auto parsed =
      dyx3_mission::parse_conditioned_artifact(bytes, dyx3_mission::sha256_hex(bytes));
  ASSERT_TRUE(parsed.ok) << parsed.error;
  EXPECT_EQ(parsed.artifact.source_sha256, source_sha);
  ASSERT_EQ(parsed.artifact.runs.size(), conditioned.size());
  for (size_t r = 0; r < conditioned.size(); ++r) {
    ASSERT_EQ(parsed.artifact.runs[r].points.size(), conditioned[r].pts.size());
    EXPECT_EQ(parsed.artifact.runs[r].flags, conditioned[r].flags);
    EXPECT_EQ(parsed.artifact.runs[r].must_hit, conditioned[r].must_hit);
    for (size_t i = 0; i < conditioned[r].pts.size(); ++i) {
      EXPECT_EQ(parsed.artifact.runs[r].points[i].north_m, conditioned[r].pts[i].n);
      EXPECT_EQ(parsed.artifact.runs[r].points[i].east_m, conditioned[r].pts[i].e);
    }
  }
}

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
  size_t n_cases = 0, n_pathcb = 0, n_pts = 0, n_excluded = 0, n_excluded_musthit = 0;
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
      if (diverges_by_must_hit(in, p)) {
        // Deliberate divergence: skip the recorded expectation, hold the new contract instead.
        ++n_excluded;
        while (rd.next(&line) && line != "END") {
        }
        for (size_t k = 0; k < in.pts.size(); ++k) {
          if (!in.must[k]) continue;
          ++n_excluded_musthit;
          bool found = false;
          for (const auto& run : runs) {
            for (size_t q = 0; q < run.pts.size(); ++q)
              found = found || (run.pts[q].n == in.pts[k].n && run.pts[q].e == in.pts[k].e &&
                                run.must_hit[q] == 1);
          }
          EXPECT_TRUE(found) << "excluded case (" << t[2] << ", @" << t.back()
                             << ") lost must-hit vertex " << k;
        }
        if (::testing::Test::HasFailure()) return;
        continue;
      }
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
  EXPECT_GT(n_pathcb - n_excluded, 100U);  // still compared with the prototype
  std::printf(
      "conditioner equivalence: %zu cases (%zu end-to-end _path_cb) over %zu input points; "
      "%zu of the %zu _path_cb cases excluded as a deliberate must-hit divergence (%zu must-hit "
      "vertices checked against the new contract instead), %zu compared with the prototype\n",
      n_cases, n_pathcb, n_pts, n_excluded, n_pathcb, n_excluded_musthit, n_pathcb - n_excluded);
}
