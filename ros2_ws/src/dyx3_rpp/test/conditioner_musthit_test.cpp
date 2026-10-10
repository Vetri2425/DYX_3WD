// Must-hit preservation in the path conditioner (contract docs/contracts/rpp_path_conditioner.md,
// rules 2, 7, 11, 12). A must-hit vertex is a survey/CAD vertex the rover has to reach: the
// conditioner may drop densification fill, never a must-hit vertex, and never move one.
//
// The prototype lost them in two places, both fixed here and both DELIBERATE divergences from the
// prototype that the gate-4 equivalence vectors cannot cover (they would encode the defect):
//   * smooth runs: smooth_corners replaced every interior vertex by an arc and resample moved the
//     rest onto a uniform grid, so a must-hit vertex disappeared and the flag with it;
//   * absorb_short_connectors replaced b and c by the apex/midpoint without looking at must-hit.
// Expectations are exact coordinates (EXPECT_EQ on doubles): "preserved" means bit-identical.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "dyx3_rpp/path_conditioner.hpp"

using namespace dyx3_rpp;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr unsigned kSpray = 1;
constexpr unsigned kMust = 2;

// The production defaults (rpp_param_table.inc) of the eleven conditioning parameters.
ConditionParams defaults(const std::string& profile = "auto") {
  ConditionParams p;
  p.tracking_profile = profile;
  p.segment_corner_threshold_deg = 45.0;
  p.connector_absorb_m = 0.2;
  p.connector_min_corner_deg = 20.0;
  p.transit_merge_max_len_m = 2.0;
  p.segment_simplify_max_offset_m = 0.01;
  p.corner_smooth_radius_m = 0.5;
  p.corner_smooth_arc_pts = 6;
  p.path_resample_spacing_m = 0.08;
  p.close_loop_threshold_m = 0.15;
  p.close_loop_min_len_m = 1.0;
  return p;
}

RawPoint raw(double n, double e, unsigned z) { return {n, e, static_cast<int>(z)}; }

// Every (run, index) whose coordinates are bit-identical to `p`.
std::vector<std::pair<size_t, size_t>> find_exact(const std::vector<ConditionedRun>& runs, double n,
                                                  double e) {
  std::vector<std::pair<size_t, size_t>> hits;
  for (size_t r = 0; r < runs.size(); ++r) {
    for (size_t k = 0; k < runs[r].pts.size(); ++k) {
      if (runs[r].pts[k].n == n && runs[r].pts[k].e == e) hits.emplace_back(r, k);
    }
  }
  return hits;
}

// The input must-hit vertex appears in the output at exactly the same coordinates, with its flag.
::testing::AssertionResult kept_exact(const std::vector<ConditionedRun>& runs, double n, double e) {
  const auto hits = find_exact(runs, n, e);
  if (hits.empty())
    return ::testing::AssertionFailure() << "must-hit (" << n << ", " << e << ") is gone";
  for (const auto& [r, k] : hits) {
    if (runs[r].must_hit[k] == 0)
      return ::testing::AssertionFailure()
             << "must-hit (" << n << ", " << e << ") kept but unflagged in run " << r;
  }
  return ::testing::AssertionSuccess();
}

size_t count_flagged(const std::vector<ConditionedRun>& runs) {
  size_t c = 0;
  for (const auto& r : runs) {
    for (unsigned char m : r.must_hit) c += m != 0 ? 1U : 0U;
  }
  return c;
}

double max_gap(const ConditionedRun& r) {
  double g = 0.0;
  for (size_t k = 1; k < r.pts.size(); ++k)
    g = std::max(g, std::hypot(r.pts[k].n - r.pts[k - 1].n, r.pts[k].e - r.pts[k - 1].e));
  return g;
}

void expect_consistent(const std::vector<ConditionedRun>& runs) {
  for (const auto& r : runs) {
    ASSERT_EQ(r.pts.size(), r.flags.size());
    ASSERT_EQ(r.pts.size(), r.must_hit.size());
    ASSERT_EQ(r.pts.size(), r.cum_s.size());
  }
}

}  // namespace

// ---- smooth closed circle -----------------------------------------------------------------------
TEST(MustHitSmooth, ClosedCircleKeepsEndpointsAndQuadrantPoints) {
  const double R = 3.0;
  const int steps = 120;  // 3 degrees per vertex
  std::vector<RawPoint> in;
  std::vector<std::pair<double, double>> must;
  const double a0 = kPi / 4.0;  // start at 45 deg so the endpoints are not quadrant points
  for (int k = 0; k <= steps; ++k) {
    const double a = a0 + 2.0 * kPi * k / steps;
    double n = R * std::cos(a), e = R * std::sin(a);
    if (k == steps) {  // a closed loop: the last vertex IS the first
      n = in.front().n;
      e = in.front().e;
    }
    // Quadrant points: 90, 180, 270 and 360 deg, i.e. k = 15, 45, 75, 105.
    const bool quadrant = k == 15 || k == 45 || k == 75 || k == 105;
    const bool endpoint = k == 0 || k == steps;
    unsigned z = kSpray | ((quadrant || endpoint) ? kMust : 0U);
    in.push_back(raw(n, e, z));
    if (quadrant || endpoint) must.emplace_back(n, e);
  }
  ASSERT_EQ(must.size(), 6U);

  const auto runs = condition_path(in, defaults());
  ASSERT_EQ(runs.size(), 1U);
  expect_consistent(runs);
  EXPECT_EQ(runs[0].profile, Profile::Smooth) << "the test must exercise the smooth path";
  EXPECT_TRUE(runs[0].closed);
  for (const auto& [n, e] : must) EXPECT_TRUE(kept_exact(runs, n, e));
  // Endpoints are the first and last output sample, flagged.
  EXPECT_EQ(runs[0].pts.front().n, in.front().n);
  EXPECT_EQ(runs[0].pts.front().e, in.front().e);
  EXPECT_EQ(runs[0].pts.back().n, in.back().n);
  EXPECT_EQ(runs[0].pts.back().e, in.back().e);
  EXPECT_EQ(runs[0].must_hit.front(), 1);
  EXPECT_EQ(runs[0].must_hit.back(), 1);
  // Exactly the six flags (start, end, four quadrants) and no stray ones.
  EXPECT_EQ(count_flagged(runs), 6U);
  // Still a circle sampled at the target spacing: every gap within the spacing, every point on the
  // circle to a few millimetres (3 degree chords sag 1 mm; the 0.5 m fillets are inside that).
  EXPECT_LE(max_gap(runs[0]), 0.08 + 1e-9);
  for (const Point& q : runs[0].pts) EXPECT_NEAR(std::hypot(q.n, q.e), R, 3e-3);
  for (unsigned char f : runs[0].flags) EXPECT_EQ(f, 1);
}

// ---- filleted rectangle -------------------------------------------------------------------------
TEST(MustHitSmooth, FilletedRectangleKeepsTheFourTangentPoints) {
  // 4 x 2 m rectangle, 0.4 m fillets, arcs densified every 5 degrees, sides every 0.5 m. NED (n,
  // e).
  const double W = 4.0, H = 2.0, r = 0.4;
  struct Corner {
    double cn, ce, a_start;  // fillet centre, start angle (rad); arcs run +90 deg
  };
  // Counter-clockwise in (e, n): bottom side -> right fillet -> right side -> ...
  const Corner corners[4] = {
      {r, W - r, -kPi / 2.0}, {H - r, W - r, 0.0}, {H - r, r, kPi / 2.0}, {r, r, kPi}};
  std::vector<RawPoint> in;
  std::vector<std::pair<double, double>> tangents;
  auto add = [&](double n, double e, bool must) {
    in.push_back(raw(n, e, kSpray | (must ? kMust : 0U)));
    if (must) tangents.emplace_back(n, e);
  };
  auto side = [&](double n0, double e0, double n1, double e1) {  // exclusive of both ends
    const double len = std::hypot(n1 - n0, e1 - e0);
    const int m = static_cast<int>(std::floor(len / 0.5));
    for (int i = 1; i <= m; ++i) {
      const double t = 0.5 * i / len;
      if (t < 1.0 - 1e-9) add(n0 + t * (n1 - n0), e0 + t * (e1 - e0), false);
    }
  };
  add(0.0, W / 2.0, false);  // start mid-bottom (not a tangent point)
  double pn = 0.0, pe = W / 2.0;
  for (int c = 0; c < 4; ++c) {
    const Corner& k = corners[c];
    const double sn = k.cn + r * std::sin(k.a_start), se = k.ce + r * std::cos(k.a_start);
    side(pn, pe, sn, se);
    add(sn, se, true);  // the tangent point: side meets arc
    for (int s = 1; s <= 18; ++s) {
      const double a = k.a_start + (kPi / 2.0) * s / 18.0;
      add(k.cn + r * std::sin(a), k.ce + r * std::cos(a), false);
    }
    pn = in.back().n;
    pe = in.back().e;
  }
  side(pn, pe, 0.0, W / 2.0);
  add(0.0, W / 2.0, false);
  ASSERT_EQ(tangents.size(), 4U);

  const auto runs = condition_path(in, defaults());
  expect_consistent(runs);
  ASSERT_FALSE(runs.empty());
  for (const auto& run : runs) EXPECT_EQ(run.profile, Profile::Smooth);
  for (const auto& [n, e] : tangents) EXPECT_TRUE(kept_exact(runs, n, e));
  EXPECT_EQ(count_flagged(runs), 4U);
  for (const auto& run : runs) EXPECT_LE(max_gap(run), 0.08 + 1e-9);
}

// ---- connector absorb ---------------------------------------------------------------------------
namespace {
// A 0.15 m connector between two 90 degree bends: a, b -> c, d with b and c 0.15 m apart.
std::vector<RawPoint> connector_path(unsigned zb, unsigned zc, bool with_mid, unsigned zmid) {
  std::vector<RawPoint> in{raw(0.0, 0.0, 0), raw(2.0, 0.0, zb)};
  if (with_mid)
    in.push_back(raw(2.0, 0.05, zmid));  // collinear fill inside the connector (not the midpoint)
  in.push_back(raw(2.0, 0.15, zc));
  in.push_back(raw(0.0, 0.15, 0));
  return in;
}
}  // namespace

TEST(MustHitConnector, UnderThresholdConnectorIsAbsorbedWithoutMustHit) {
  // Control: proves the geometry below really is an absorb candidate.
  const auto runs = condition_path(connector_path(0, 0, false, 0), defaults());
  expect_consistent(runs);
  EXPECT_TRUE(find_exact(runs, 2.0, 0.0).empty());
  EXPECT_TRUE(find_exact(runs, 2.0, 0.15).empty());
  EXPECT_FALSE(find_exact(runs, 2.0, 0.075).empty());  // b and c became their midpoint
}

TEST(MustHitConnector, MustHitAtBIsNotAbsorbed) {
  const auto runs = condition_path(connector_path(kMust, 0, false, 0), defaults());
  expect_consistent(runs);
  EXPECT_TRUE(kept_exact(runs, 2.0, 0.0));
  EXPECT_FALSE(find_exact(runs, 2.0, 0.15).empty()) << "the connector survives whole";
  EXPECT_TRUE(find_exact(runs, 2.0, 0.075).empty());
}

TEST(MustHitConnector, MustHitAtCIsNotAbsorbed) {
  const auto runs = condition_path(connector_path(0, kMust, false, 0), defaults());
  expect_consistent(runs);
  EXPECT_TRUE(kept_exact(runs, 2.0, 0.15));
  EXPECT_FALSE(find_exact(runs, 2.0, 0.0).empty());
}

TEST(MustHitConnector, MustHitCollinearInsideTheConnectorIsNotErased) {
  // The absorb erases the whole raw range b..c, including collinear fill that the simplifier had
  // dropped: a must-hit vertex hiding there is protected too.
  const auto runs = condition_path(connector_path(0, 0, true, kMust), defaults());
  expect_consistent(runs);
  EXPECT_TRUE(kept_exact(runs, 2.0, 0.05));
}

TEST(MustHitConnector, DirectCallHonoursTheKeySet) {
  const std::vector<Point> pts{{0.0, 0.0}, {2.0, 0.0}, {2.0, 0.15}, {0.0, 0.15}};
  const Flags flags{0, 0, 0, 0};
  const PointRun plain = absorb_short_connectors(pts, flags, 45.0, 0.2, 20.0);
  ASSERT_EQ(plain.pts.size(), 3U);  // prototype behaviour with no key set: b, c -> one vertex
  KeySet keys{pt_key(pts[1])};
  const PointRun kept = absorb_short_connectors(pts, flags, 45.0, 0.2, 20.0, &keys);
  ASSERT_EQ(kept.pts.size(), 4U);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(kept.pts[i].n, pts[i].n);
    EXPECT_EQ(kept.pts[i].e, pts[i].e);
  }
  const KeySet unrelated{pt_key({50.0, 50.0})};  // a must-hit elsewhere does not block it
  EXPECT_EQ(absorb_short_connectors(pts, flags, 45.0, 0.2, 20.0, &unrelated).pts.size(), 3U);
}

// ---- smooth_corners / resample directly ---------------------------------------------------------
TEST(MustHitSmooth, SmoothCornersLeavesAMustHitVertexSharpAndUnflaggedNeighboursSmooth) {
  // Two 30 degree bends, 2 m legs; radius 0.5 m fits (d = 0.5 / tan(75 deg) = 0.13 m).
  const std::vector<Point> pts{{0.0, 0.0}, {2.0, 0.0}, {3.7320508075688772, 1.0}, {5.0, 3.0}};
  const Flags flags{1, 1, 1, 1};
  int skipped = -1;
  const PointRun plain = smooth_corners(pts, 0.5, 6, &flags, &skipped);
  EXPECT_EQ(skipped, 0);
  for (const Point& q : plain.pts) EXPECT_FALSE(q.n == pts[1].n && q.e == pts[1].e);

  const KeySet keys{pt_key(pts[1])};
  skipped = -1;
  const PointRun kept = smooth_corners(pts, 0.5, 6, &flags, &skipped, &keys);
  EXPECT_EQ(skipped, 0) << "a must-hit vertex is not a 'segments too short' skip";
  size_t at = kept.pts.size();
  for (size_t i = 0; i < kept.pts.size(); ++i) {
    if (kept.pts[i].n == pts[1].n && kept.pts[i].e == pts[1].e) at = i;
  }
  ASSERT_LT(at, kept.pts.size());
  // Sharp: its neighbours are the previous vertex and the untouched next arc, not arc points of its
  // own.
  EXPECT_EQ(kept.pts[at - 1].n, pts[0].n);
  EXPECT_EQ(kept.pts[at - 1].e, pts[0].e);
  // The other bend is still replaced by an arc (7 points instead of one vertex).
  EXPECT_EQ(kept.pts.size(), plain.pts.size() - 7 + 1);
  EXPECT_EQ(kept.flags.size(), kept.pts.size());
}

TEST(MustHitSmooth, MustHitVertexOnADegenerateSegmentSurvives) {
  // A repeated vertex used to be dropped by the l < 1e-9 guard, taking a must-hit coordinate with
  // it.
  const std::vector<Point> pts{{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {2.0, 1.0}};
  const KeySet keys{pt_key(pts[1])};
  const PointRun out = smooth_corners(pts, 0.5, 6, nullptr, nullptr, &keys);
  bool found = false;
  for (const Point& q : out.pts) found = found || (q.n == 1.0 && q.e == 0.0);
  EXPECT_TRUE(found);
}

TEST(MustHitSmooth, ForcedSmoothKeepsAKinkTheFilletWouldHaveReplaced) {
  // Two 2 m legs with a 25 degree kink: the 0.5 m fillet fits (d = 0.11 m < 0.45 * 2 m), so
  // without must-hit the kink is rounded off (control). Sparse vertices on purpose: with 0.1 m
  // densification d > 0.45 * segment and smooth_corners leaves every vertex sharp anyway.
  const double c = std::cos(25.0 * kPi / 180.0), s = std::sin(25.0 * kPi / 180.0);
  std::vector<RawPoint> in{raw(0.0, 0.0, kSpray), raw(2.0, 0.0, kSpray | kMust),
                           raw(2.0 + 2.0 * c, 2.0 * s, kSpray)};

  const auto kept = condition_path(in, defaults("smooth"));
  expect_consistent(kept);
  ASSERT_EQ(kept.size(), 1U);
  EXPECT_TRUE(kept_exact(kept, 2.0, 0.0));
  EXPECT_EQ(count_flagged(kept), 1U);
  EXPECT_LE(max_gap(kept[0]), 0.08 + 1e-9);

  in[1].z &= ~static_cast<int>(kMust);
  const auto control = condition_path(in, defaults("smooth"));
  ASSERT_EQ(control.size(), 1U);
  EXPECT_TRUE(find_exact(control, 2.0, 0.0).empty())
      << "without must-hit the corner is rounded off";
  EXPECT_EQ(count_flagged(control), 0U);
}

TEST(MustHitSmooth, SpacingIsHeldInsideEverySpan) {
  // Two must-hit vertices 0.1 m apart on a gentle arc: the span between them is shorter than
  // twice the spacing and must still be sampled at or below the target spacing.
  std::vector<RawPoint> in;
  for (int i = 0; i <= 60; ++i) {
    const double a = i * 0.02;
    in.push_back(raw(4.0 * std::sin(a), 4.0 * (1.0 - std::cos(a)), kSpray));
  }
  in[30].z |= static_cast<int>(kMust);
  in[32].z |= static_cast<int>(kMust);
  const auto runs = condition_path(in, defaults("smooth"));
  expect_consistent(runs);
  ASSERT_EQ(runs.size(), 1U);
  EXPECT_TRUE(kept_exact(runs, in[30].n, in[30].e));
  EXPECT_TRUE(kept_exact(runs, in[32].n, in[32].e));
  EXPECT_LE(max_gap(runs[0]), 0.08 + 1e-9);
}

// ---- segment runs are unchanged -----------------------------------------------------------------
TEST(MustHitSegment, SegmentRunBehaviourIsUnchanged) {
  // (0,0) (1,0)* (2,0) (3,0) corner (3,2): collinear fill is dropped, the must-hit (1,0) and the
  // corner (3,0) stay. Forced segment: one run, no smoothing, no resampling.
  const std::vector<RawPoint> in{raw(0, 0, 0), raw(1, 0, kMust), raw(2, 0, 0), raw(3, 0, 0),
                                 raw(3, 2, 0)};
  const auto runs = condition_path(in, defaults("segment"));
  expect_consistent(runs);
  ASSERT_EQ(runs.size(), 1U);
  EXPECT_EQ(runs[0].profile, Profile::Segment);
  const std::vector<Point> want{{0, 0}, {1, 0}, {3, 0}, {3, 2}};
  const std::vector<unsigned char> want_must{0, 1, 0, 0};
  ASSERT_EQ(runs[0].pts.size(), want.size());
  for (size_t i = 0; i < want.size(); ++i) {
    EXPECT_EQ(runs[0].pts[i].n, want[i].n);
    EXPECT_EQ(runs[0].pts[i].e, want[i].e);
    EXPECT_EQ(runs[0].must_hit[i], want_must[i]);
  }
}

TEST(MustHitSegment, AutoPicksSegmentAndAbsorbsExactlyAsBefore) {
  // Two legs and a 0.1 m connector with no must-hit anywhere: the prototype result.
  const std::vector<RawPoint> in{raw(0, 0, 0), raw(1, 0, 0), raw(2, 0, 0), raw(2, 0.1, 0),
                                 raw(0, 0.1, 0)};
  const auto runs = condition_path(in, defaults());
  expect_consistent(runs);
  ASSERT_FALSE(runs.empty());
  EXPECT_TRUE(find_exact(runs, 2.0, 0.0).empty());    // b absorbed
  EXPECT_TRUE(find_exact(runs, 2.0, 0.1).empty());    // c absorbed
  EXPECT_FALSE(find_exact(runs, 2.0, 0.05).empty());  // into the midpoint
  EXPECT_EQ(count_flagged(runs), 0U);
}

// ---- property: every input must-hit survives, whatever the profile ------------------------------
namespace {
struct Lcg {
  uint64_t s;
  double next() {  // [0, 1)
    s = s * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<double>(s >> 11) / 9007199254740992.0;
  }
};
}  // namespace

TEST(MustHitProperty, EveryInputMustHitSurvivesAtItsExactCoordinates) {
  Lcg rng{0x5EEDULL};
  size_t cases = 0, must_total = 0;
  for (const char* profile : {"auto", "segment", "smooth"}) {
    for (int c = 0; c < 60; ++c) {
      // A random walk with gentle curves, sharp turns and short connectors; spray toggles now and
      // then; ~12 percent of the vertices are must-hit.
      std::vector<RawPoint> in;
      double n = 0.0, e = 0.0, h = rng.next() * 2.0 * kPi;
      const int len = 25 + static_cast<int>(rng.next() * 60.0);
      unsigned spray = rng.next() < 0.5 ? 1U : 0U;
      for (int i = 0; i < len; ++i) {
        const unsigned z = spray | (rng.next() < 0.12 ? kMust : 0U);
        in.push_back(raw(n, e, z));
        const double r = rng.next();
        if (r < 0.10) {
          h += (rng.next() - 0.5) * 2.0 * kPi * 0.6;  // sharp turn
        } else if (r < 0.15) {
          h += (rng.next() > 0.5 ? 1.0 : -1.0) * 1.5;  // ~90 degree turn
        } else {
          h += (rng.next() - 0.5) * 0.08;  // gentle
        }
        if (rng.next() < 0.04) spray ^= 1U;
        const double step = (r > 0.15 && r < 0.25) ? 0.05 + 0.1 * rng.next()  // short connector
                                                   : 0.1 + 0.5 * rng.next();
        n += step * std::cos(h);
        e += step * std::sin(h);
      }
      // A path that is entirely a sliver is out of scope (the sliver rule drops <5 cm runs).
      const auto runs = condition_path(in, defaults(profile));
      expect_consistent(runs);
      ++cases;
      for (const RawPoint& p : in) {
        if ((p.z & static_cast<int>(kMust)) == 0) continue;
        ++must_total;
        EXPECT_TRUE(kept_exact(runs, p.n, p.e))
            << profile << " case " << c << " lost must-hit (" << p.n << ", " << p.e << ")";
      }
      if (::testing::Test::HasFailure()) return;
    }
  }
  std::printf("must-hit property: %zu paths, %zu must-hit vertices, all kept exactly\n", cases,
              must_total);
  EXPECT_GT(must_total, 500U);
}
