// Definitional tests for dyx3_geometry: frame conventions and geometric facts.
//
// Expectations here are mathematical definitions (NED: 0 = North, clockwise-positive; cross-track
// + = RIGHT; circle curvature = 1/R), NOT values copied from the implementation or the Python
// ancestors. Numeric equivalence with the ancestors is gate3_equivalence_test (GATE 3), and
// recorded-bag evidence is geometry_bag_replay_test.
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "check.hpp"
#include "dyx3_geometry/angle_wrap.hpp"
#include "dyx3_geometry/curvature.hpp"
#include "dyx3_geometry/distance.hpp"
#include "dyx3_geometry/heading_delta.hpp"
#include "dyx3_geometry/line_intersection.hpp"
#include "dyx3_geometry/path_length.hpp"
#include "dyx3_geometry/perpendicular_distance.hpp"
#include "dyx3_geometry/project_onto_path.hpp"
#include "dyx3_geometry/project_onto_segment.hpp"
#include "dyx3_geometry/resample.hpp"
#include "dyx3_geometry/segment_heading.hpp"

using namespace dyx3_geometry;  // NOLINT

namespace {
constexpr double kPi = 3.14159265358979323846;
double deg(double d) { return d * kPi / 180.0; }

void test_distance() {
  CHECK_NEAR(distance({0, 0}, {3, 4}), 5.0, 1e-15);
  CHECK(distance({1, 2}, {-4, 7}) == distance({-4, 7}, {1, 2}));
  CHECK(distance({1, 1}, {1, 1}) == 0.0);
}

void test_angle_wrap() {
  CHECK_NEAR(angle_wrap(kPi), -kPi, 1e-15);  // interval is [-pi, pi)
  CHECK_NEAR(angle_wrap(-kPi), -kPi, 1e-15);
  CHECK_NEAR(angle_wrap(0.0), 0.0, 1e-15);
  CHECK_NEAR(angle_wrap(2.0 * kPi), 0.0, 1e-12);
  CHECK_NEAR(angle_wrap(1.5 * kPi), -0.5 * kPi, 1e-12);
  CHECK_NEAR(angle_wrap(-kPi - 0.1), kPi - 0.1, 1e-12);
  CHECK_NEAR(angle_wrap(0.1 + 10.0 * 2.0 * kPi), 0.1, 1e-9);
  for (double a = -20.0; a <= 20.0; a += 0.37) {
    const double w = angle_wrap(a);
    CHECK(w >= -kPi && w < kPi);
    CHECK_NEAR(std::cos(w), std::cos(a), 1e-12);  // same direction
    CHECK_NEAR(std::sin(w), std::sin(a), 1e-12);
  }
}

void test_heading_delta() {
  CHECK_NEAR(heading_delta(deg(350), deg(10)), deg(20), 1e-12);  // shortest way through North
  CHECK_NEAR(heading_delta(deg(10), deg(350)), deg(20), 1e-12);
  CHECK_NEAR(heading_delta(0.0, 0.0), 0.0, 1e-15);
  CHECK_NEAR(heading_delta(deg(-90), deg(90)), kPi, 1e-12);
  for (double a = -3.0; a <= 3.0; a += 0.5) {
    for (double b = -3.0; b <= 3.0; b += 0.5) {
      const double d = heading_delta(a, b);
      CHECK(d >= 0.0 && d <= kPi + 1e-12);
      CHECK_NEAR(d, heading_delta(b, a), 1e-12);
    }
  }
}

void test_segment_heading() {
  // NED: 0 = North (+n), +90 deg = East (+e), clockwise-positive.
  CHECK_NEAR(segment_heading({0, 0}, {1, 0}), 0.0, 1e-15);
  CHECK_NEAR(segment_heading({0, 0}, {0, 1}), 0.5 * kPi, 1e-15);
  CHECK_NEAR(std::fabs(segment_heading({0, 0}, {-1, 0})), kPi, 1e-15);
  CHECK_NEAR(segment_heading({0, 0}, {0, -1}), -0.5 * kPi, 1e-15);
  CHECK_NEAR(segment_heading({2, 3}, {3, 4}), 0.25 * kPi, 1e-15);
  CHECK(segment_heading({1, 1}, {1, 1}) == 0.0);  // degenerate: atan2(0, 0)
}

void test_perpendicular_distance() {
  CHECK_NEAR(perpendicular_distance({5, 1}, {0, 0}, {10, 0}), 1.0, 1e-15);
  CHECK_NEAR(perpendicular_distance({25, -2}, {0, 0}, {10, 0}), 2.0, 1e-15);  // infinite line
  CHECK_NEAR(perpendicular_distance({3, 4}, {1, 1}, {1, 1}), std::hypot(2.0, 3.0), 1e-15);
  CHECK_NEAR(perpendicular_distance({1, 1}, {0, 0}, {1, 1}), 0.0, 1e-15);
}

void test_line_intersection() {
  auto p = line_intersection({0, 1}, {10, 1}, {2, -5}, {2, 5});  // y=1 (east=1) vs n=2
  CHECK(p.has_value());
  CHECK_NEAR(p->n, 2.0, 1e-12);
  CHECK_NEAR(p->e, 1.0, 1e-12);
  CHECK(!line_intersection({0, 0}, {1, 0}, {0, 1}, {1, 1}).has_value());  // parallel
  CHECK(!line_intersection({0, 0}, {1, 1}, {5, 5}, {9, 9}).has_value());  // collinear
  auto q = line_intersection({0, 0}, {1, 1}, {0, 2}, {2, 0});             // diagonals
  CHECK(q.has_value() && std::fabs(q->n - 1.0) < 1e-12 && std::fabs(q->e - 1.0) < 1e-12);
}

void test_project_onto_segment() {
  const Point a{0, 0};
  const Point b{10, 0};  // directed North
  // East of a north-heading line is the RIGHT-hand side: cross-track is positive.
  auto r = project_onto_segment({5, 1}, a, b);
  CHECK_NEAR(r.t, 0.5, 1e-15);
  CHECK_NEAR(r.foot.n, 5.0, 1e-15);
  CHECK_NEAR(r.foot.e, 0.0, 1e-15);
  CHECK_NEAR(r.signed_cross, 1.0, 1e-15);
  CHECK_NEAR(r.dist_to_end_along, 5.0, 1e-15);
  CHECK_NEAR(project_onto_segment({5, -1}, a, b).signed_cross, -1.0, 1e-15);  // west = left
  // Heading South: the same ground side flips sign.
  CHECK_NEAR(project_onto_segment({5, 1}, b, a).signed_cross, -1.0, 1e-15);

  // Handover regression (field 2026-07-30): a rover 3 cm BEFORE the vertex on a perfectly
  // collinear path has NO cross-track error; the along-track gap must not leak into it.
  auto before = project_onto_segment({-0.03, 0.0}, a, b);
  CHECK(before.t == 0.0);
  CHECK(before.signed_cross == 0.0);
  CHECK_NEAR(before.dist_to_end_along, 10.0, 1e-15);
  auto beyond = project_onto_segment({10.03, 0.0}, a, b);
  CHECK(beyond.t == 1.0);
  CHECK(beyond.signed_cross == 0.0);
  CHECK_NEAR(beyond.dist_to_end_along, 0.0, 1e-15);
  // ...and a real lateral offset is preserved when t is clamped.
  CHECK_NEAR(project_onto_segment({-0.03, 0.02}, a, b).signed_cross, 0.02, 1e-15);

  // Degenerate segment: carried quirk (dist_to_end_along = |pos - a|).
  auto d = project_onto_segment({3, 4}, {1, 1}, {1, 1});
  CHECK(d.t == 0.0 && d.signed_cross == 0.0);
  CHECK_NEAR(d.dist_to_end_along, std::hypot(2.0, 3.0), 1e-15);

  // Path overload clamps the index and handles tiny paths.
  const std::vector<Point> two = {{0, 0}, {10, 0}};
  CHECK_NEAR(project_onto_segment({5, 1}, PathView(two), 7).signed_cross, 1.0, 1e-15);
  CHECK_NEAR(project_onto_segment({5, 1}, PathView(two), -3).signed_cross, 1.0, 1e-15);
  const std::vector<Point> one = {{2, 2}};
  CHECK_NEAR(project_onto_segment({5, 6}, PathView(one), 0).dist_to_end_along, 5.0, 1e-15);
  CHECK(!project_onto_segment({0, 0}, PathView(), 0).valid);
}

std::vector<Point> l_shape() {
  // North 10 m, then East 10 m (a right-hand corner for a rover driving the path).
  std::vector<Point> p;
  for (int i = 0; i <= 10; ++i) p.push_back({static_cast<double>(i), 0});
  for (int i = 1; i <= 10; ++i) p.push_back({10, static_cast<double>(i)});
  return p;
}

void test_project_onto_path() {
  const auto path = l_shape();
  ProjectionHint hint;  // invalid: full scan
  auto r = project_onto_path({4.2, 0.3}, PathView(path), hint);
  CHECK(r.seg_idx == 4);
  CHECK_NEAR(r.t, 0.2, 1e-12);
  CHECK_NEAR(r.signed_cross, 0.3, 1e-12);  // east of a north leg: right
  CHECK(hint.valid && hint.seg == 4);
}

void test_project_onto_path_sign_east_leg() {
  const auto path = l_shape();
  ProjectionHint h;
  auto north_of =
      project_onto_path({10.4, 5.5}, PathView(path), h);  // 0.4 m north of an east-bound leg
  CHECK(north_of.seg_idx >= 10);
  CHECK_NEAR(north_of.signed_cross, -0.4, 1e-12);  // left of an east-bound leg
  h = {};
  auto south_of = project_onto_path({9.6, 5.5}, PathView(path), h);
  CHECK_NEAR(south_of.signed_cross, 0.4, 1e-12);  // right
}

void test_windowed_search_agrees_with_full_scan_near_the_hint() {
  const auto path = l_shape();
  ProjectionHint warm;
  for (int i = 0; i < 20; ++i) {
    const double s = 0.2 * i;  // walk along the path
    const Point pos = s <= 10.0 ? Point{s, 0.05} : Point{9.95, s - 10.0};
    auto w = project_onto_path(pos, PathView(path), warm);
    ProjectionHint cold;
    auto f = project_onto_path(pos, PathView(path), cold);
    CHECK(w.seg_idx == f.seg_idx);
    CHECK(w.signed_cross == f.signed_cross);
  }
  // A stale/huge hint must fall back to a full scan instead of reading out of range.
  ProjectionHint stale{1000, true};
  auto r = project_onto_path({5, 0.1}, PathView(path), stale);
  CHECK(r.seg_idx == 4 || r.seg_idx == 5);
  CHECK(stale.valid);
}

void test_project_onto_path_tiny() {
  ProjectionHint h;
  CHECK(!project_onto_path({0, 0}, PathView(), h).valid);
  const std::vector<Point> one = {{3, 4}};
  auto r = project_onto_path({0, 0}, PathView(one), h);
  CHECK(r.seg_idx == 0 && r.t == 0.0);
  CHECK_NEAR(r.signed_cross, 5.0, 1e-15);  // sign undefined: +distance (ancestor)
  const std::vector<Point> degenerate = {{1, 1}, {1, 1}, {1, 1}};
  ProjectionHint hd;
  auto d = project_onto_path({0, 0}, PathView(degenerate), hd);
  CHECK(!d.valid);   // GEO-002: no non-degenerate segment => no projection
  CHECK(!hd.valid);  // nothing usable in the window: hint invalidated, next call full-scans
  // Same with a warm hint, and with more points than the window.
  const std::vector<Point> same_many(20, Point{2, 3});
  ProjectionHint hw{5, true};
  CHECK(!project_onto_path({0, 0}, PathView(same_many), hw).valid);
  CHECK(!hw.valid);
  // A degenerate window inside an otherwise normal path is also reported invalid.
  std::vector<Point> mixed;
  for (int i = 0; i < 10; ++i) mixed.push_back({static_cast<double>(i), 0.0});
  for (int i = 0; i < 12; ++i) mixed.push_back({9.0, 0.0});
  ProjectionHint hm{15, true};
  CHECK(!project_onto_path({4, 1}, PathView(mixed), hm).valid);
  CHECK(!hm.valid);
  ProjectionHint hc;  // next call (cold) full-scans and recovers
  auto ok = project_onto_path({4.5, 1}, PathView(mixed), hc);
  CHECK(ok.valid && ok.seg_idx == 4 && hc.valid);
  CHECK(project_onto_path({4.2, 0.3}, PathView(l_shape()), hc).valid);  // normal path unchanged
}

void test_path_length_and_cumulative() {
  const std::vector<Point> sq = {{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}};
  CHECK_NEAR(path_length(PathView(sq)), 4.0, 1e-15);
  CHECK(path_length(PathView()) == 0.0);
  const std::vector<Point> one = {{1, 2}};
  CHECK(path_length(PathView(one)) == 0.0);
  const auto cum = cumulative_lengths(PathView(sq));
  CHECK(cum.size() == 5 && cum[0] == 0.0);
  for (std::size_t i = 1; i < cum.size(); ++i) CHECK(cum[i] >= cum[i - 1]);
  CHECK(cum.back() == path_length(PathView(sq)));  // same accumulation order => bit-identical
  CHECK(cumulative_lengths(PathView()).size() == 1);
  double small[2];
  CHECK(cumulative_lengths(PathView(sq), small, 2) == 0);  // too small: refuses, writes nothing
}

void test_resample() {
  const std::vector<Point> line = {{0, 0}, {10, 0}};
  auto r = resample(line, 0.5);
  CHECK(r.pts.size() == 21);
  CHECK(r.pts.front().n == 0.0 && r.pts.back().n == 10.0);  // endpoints exact
  for (std::size_t i = 1; i < r.pts.size(); ++i) {
    CHECK_NEAR(distance(r.pts[i - 1], r.pts[i]), 0.5, 1e-12);
    CHECK_NEAR(r.pts[i].e, 0.0, 1e-15);  // straight stays straight
  }
  auto odd = resample(line, 0.3);  // ceil(10/0.3) + 1 = 35 samples at 10/34 spacing
  CHECK(odd.pts.size() == 35);
  CHECK_NEAR(distance(odd.pts[0], odd.pts[1]), 10.0 / 34.0, 1e-12);
  CHECK(resample(line, 20.0).pts.size() == 2);     // total < spacing: endpoints only
  CHECK(resample(line, 0.0).pts.size() == 2);      // spacing <= 0: unchanged
  CHECK(resample({{1, 1}}, 0.1).pts.size() == 1);  // n < 2: unchanged

  // GEO-005: non-finite spacing, absurd sample counts and non-finite paths are refused (input
  // returned unchanged), never converted to an out-of-range integer or allocated.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double bad_spacing : {nan, inf, -inf, -1.0, 1e-300, 1e-9}) {
    auto rr = resample(line, bad_spacing);
    CHECK(rr.pts.size() == 2 && rr.flags.size() == 2);
    CHECK(rr.pts.front().n == 0.0 && rr.pts.back().n == 10.0);
  }
  const std::vector<Point> three_pts = {{0, 0}, {5, 0}, {10, 0}};
  CHECK(resample(three_pts, nan).pts.size() == 3);  // proves "unchanged", not "endpoints"
  CHECK(resample(three_pts, 1e-9).pts.size() == 3);
  CHECK(resample(three_pts, 1e-9).flags.size() == 3);
  const std::vector<Point> nan_path = {{0, 0}, {nan, 0}, {10, 0}};
  CHECK(resample(nan_path, 1.0).pts.size() == 3);
  const std::vector<Point> inf_path = {{0, 0}, {inf, 0}};
  CHECK(resample(inf_path, 1.0).pts.size() == 2);
  // Boundary: exactly kResampleMaxSamples is served, one more is refused.
  const double cap = static_cast<double>(kResampleMaxSamples);
  const std::vector<Point> at_cap = {{0, 0}, {cap - 1.0, 0}, {cap - 1.0, 0}};
  CHECK(resample(at_cap, 1.0).pts.size() == kResampleMaxSamples);
  const std::vector<Point> over_cap = {{0, 0}, {cap, 0}, {cap, 0}};
  CHECK(resample(over_cap, 1.0).pts.size() == 3);  // would need cap + 1 samples: refused

  const std::vector<Point> three = {{0, 0}, {5, 0}, {10, 0}};
  const std::vector<unsigned char> f = {1, 1, 0};
  auto rf = resample(three, 1.0, &f);
  CHECK(rf.flags.size() == rf.pts.size());
  CHECK(rf.flags.front() == 1 && rf.flags.back() == 0);
  // interior flag = AND of the bracketing flags: 1 on the first leg, 0 on the second
  for (std::size_t i = 1; i + 1 < rf.pts.size(); ++i) {
    const double n = rf.pts[i].n;
    if (n < 5.0 - 1e-9) CHECK(rf.flags[i] == 1);
    if (n > 5.0 + 1e-9) CHECK(rf.flags[i] == 0);
  }
  const std::vector<unsigned char> bad = {1};  // length mismatch => all-false (ancestor)
  auto rb = resample(three, 5.0, &bad);
  for (auto v : rb.flags) CHECK(v == 0);
}

// resample_fixed: fixed knots survive verbatim and each span is resampled on its own.
void test_resample_fixed() {
  const std::vector<Point> line = {{0, 0}, {1, 0}, {4, 0}, {10, 0}};
  const std::vector<unsigned char> f = {1, 0, 1, 1};

  // No fixed indices (or only the endpoints) is the plain resample, bit for bit.
  const auto plain = resample(line, 0.5, &f);
  for (const std::vector<std::size_t>& fx :
       {std::vector<std::size_t>{}, std::vector<std::size_t>{0, 3},
        std::vector<std::size_t>{3, 0, 0}, std::vector<std::size_t>{7, 99}}) {
    const auto same = resample_fixed(line, 0.5, &f, fx);
    CHECK(same.pts.size() == plain.pts.size() && same.flags == plain.flags);
    for (std::size_t i = 0; i < plain.pts.size() && i < same.pts.size(); ++i) {
      CHECK(same.pts[i].n == plain.pts[i].n && same.pts[i].e == plain.pts[i].e);
    }
  }

  // A fixed knot at n = 4.0 that is NOT a multiple of the 10/34 uniform grid of the plain resample.
  const auto r = resample_fixed(line, 0.3, &f, {2});
  bool found = false;
  std::size_t at = 0;
  for (std::size_t i = 0; i < r.pts.size(); ++i) {
    if (r.pts[i].n == 4.0 && r.pts[i].e == 0.0) {
      found = true;
      at = i;
    }
  }
  CHECK(found);
  CHECK(r.pts.front().n == 0.0 && r.pts.back().n == 10.0);
  CHECK(r.flags.size() == r.pts.size());
  if (found) {
    CHECK(r.flags[at] == 1);  // the knot keeps its own flag
    // span [0,4]: ceil(4/0.3)+1 = 15 samples; span [4,10]: ceil(6/0.3)+1 = 21 -> 15 + 20 total
    CHECK(r.pts.size() == 35);
    CHECK(at == 14);
    for (std::size_t i = 1; i <= 14; ++i)
      CHECK_NEAR(r.pts[i].n - r.pts[i - 1].n, 4.0 / 14.0, 1e-12);
    for (std::size_t i = 15; i < r.pts.size(); ++i)
      CHECK_NEAR(r.pts[i].n - r.pts[i - 1].n, 6.0 / 20.0, 1e-12);
  }

  // Unsorted / duplicated fixed indices give the same answer as a clean list.
  const auto messy = resample_fixed(line, 0.3, &f, {2, 2, 1, 1, 2});
  const auto clean = resample_fixed(line, 0.3, &f, {1, 2});
  CHECK(messy.pts.size() == clean.pts.size());
  for (std::size_t i = 0; i < messy.pts.size() && i < clean.pts.size(); ++i)
    CHECK(messy.pts[i].n == clean.pts[i].n);

  // A bend that the plain resample cuts through stays exact when it is fixed.
  const std::vector<Point> bend = {{0, 0}, {1.03, 0}, {1.03, 1.07}};
  const auto cut = resample(bend, 0.5);
  const auto kept = resample_fixed(bend, 0.5, nullptr, {1});
  bool cut_has = false, kept_has = false;
  for (const Point& p : cut.pts) cut_has = cut_has || (p.n == 1.03 && p.e == 0.0);
  for (const Point& p : kept.pts) kept_has = kept_has || (p.n == 1.03 && p.e == 0.0);
  CHECK(!cut_has);  // the plain resample drops the corner: the defect this function fixes
  CHECK(kept_has);

  // Coincident knots emit once; the path end stays exact.
  const std::vector<Point> dup = {{0, 0}, {2, 0}, {2, 0}, {2, 0}};
  const auto rd = resample_fixed(dup, 0.5, nullptr, {1, 2});
  CHECK(rd.pts.front().n == 0.0 && rd.pts.back().n == 2.0);
  for (std::size_t i = 1; i < rd.pts.size(); ++i) CHECK(rd.pts[i].n > rd.pts[i - 1].n);

  // Refusals apply to the whole path: the input comes back unchanged, so every knot survives.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<Point> three_pts = {{0, 0}, {5, 0}, {10, 0}};
  CHECK(resample_fixed(three_pts, nan, nullptr, {1}).pts.size() == 3);
  CHECK(resample_fixed(three_pts, 1e-9, nullptr, {1}).pts.size() == 3);
  CHECK(resample_fixed({{1, 1}}, 0.1, nullptr, {0}).pts.size() == 1);
  // A path shorter than the spacing with a fixed interior knot keeps all three knots.
  const auto tiny = resample_fixed({{0, 0}, {0.01, 0}, {0.02, 0}}, 0.5, nullptr, {1});
  CHECK(tiny.pts.size() == 3);
}

// Deterministic jitter: SplitMix64 so the test is reproducible on every platform.
double jitter(std::uint64_t& s, double amp) {
  s += 0x9E3779B97F4A7C15ULL;
  std::uint64_t z = s;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  z ^= z >> 31;
  return amp * (static_cast<double>(z >> 11) / 9007199254740992.0 * 2.0 - 1.0);
}

std::vector<Point> arc(double radius, double sweep_rad, double spacing) {
  const int n = static_cast<int>(std::ceil(radius * sweep_rad / spacing));
  std::vector<Point> p;
  for (int i = 0; i <= n; ++i) {
    const double a = sweep_rad * i / n;
    p.push_back({radius * std::sin(a), radius * (1.0 - std::cos(a))});  // starts North, bends East
  }
  return p;
}

void test_curvature() {
  CHECK_NEAR(menger_curvature({0, 0}, {1, 0}, {2, 0}), 0.0, 1e-15);  // collinear
  CHECK_NEAR(menger_curvature({0, 0}, {1, 0}, {1, 1}), 1.0 / std::sqrt(0.5),
             1e-12);                                       // R = sqrt(2)/2
  CHECK(menger_curvature({0, 0}, {0, 0}, {1, 1}) == 0.0);  // degenerate side
  CHECK(curvature_at(PathView(), 0) == 0.0);

  // Points exactly on a circle: Menger curvature is exactly 1/R for any three of them.
  const double R = 2.4;
  const auto clean = arc(R, deg(120), 0.05);
  const int mid = static_cast<int>(clean.size() / 2);
  CHECK_NEAR(curvature_at(PathView(clean), mid, 0.0), 1.0 / R, 1e-9);
  CHECK_NEAR(curvature_at(PathView(clean), mid, 0.15), 1.0 / R, 1e-9);

  // Field finding (2026-07-30 curve bags): on 4-10 cm spacing the adjacent-vertex form is dominated
  // by coordinate noise (reported kappa_max 1.25 vs a true 0.42, a 3x error) and is stable only
  // from a ~0.10 m half-baseline up. Reproduce that property with +-1.5 mm noise on a 5 cm arc.
  auto noisy = clean;
  std::uint64_t seed = 20260730;
  for (auto& q : noisy) {
    q.n += jitter(seed, 0.0015);
    q.e += jitter(seed, 0.0015);
  }
  double worst_adjacent = 0.0;
  double worst_baseline = 0.0;
  for (int i = 10; i + 10 < static_cast<int>(noisy.size()); ++i) {
    worst_adjacent = std::max(worst_adjacent, curvature_at(PathView(noisy), i, 0.0));
    worst_baseline = std::max(worst_baseline, curvature_at(PathView(noisy), i, 0.30));
  }
  CHECK(worst_adjacent > 2.0 * (1.0 / R));   // adjacent vertices: > 2x too large
  CHECK(worst_baseline < 1.25 * (1.0 / R));  // 0.30 m half-baseline: within 25 %
  CHECK(worst_baseline > 0.75 * (1.0 / R));
}

void test_max_preview_curvature() {
  const double R = 1.5;
  const auto circle = arc(R, deg(200), 0.04);
  const Point foot = circle[0];
  // Preview samples are interpolated ALONG CHORDS of the 4 cm polyline, which cut the circle by a
  // sagitta of s^2/(8R) = 0.13 mm, so the recovered curvature is 1/R only to ~1e-5.
  CHECK_NEAR(max_preview_curvature(PathView(circle), 0, foot, 0.4, 4), 1.0 / R, 5e-5);
  std::vector<Point> straight;
  for (int i = 0; i <= 100; ++i) straight.push_back({0.05 * i, 0.0});
  CHECK_NEAR(max_preview_curvature(PathView(straight), 0, straight[0], 0.4, 4), 0.0, 1e-12);
  CHECK(max_preview_curvature(PathView(circle), 0, foot, 0.4, 1) == 0.0);  // n <= 1
  CHECK(max_preview_curvature(PathView(circle), 0, foot, 0.0, 4) == 0.0);  // l_d <= 0
  // Previews beyond the end of the path add no information and must not crash or invent curvature.
  CHECK_NEAR(max_preview_curvature(PathView(straight), 90, straight[90], 5.0, 10), 0.0, 1e-12);
  CHECK(max_preview_curvature(PathView(), 0, {0, 0}, 0.4, 4) == 0.0);
  // The preview sees a corner before the rover reaches it.
  std::vector<Point> bent;
  for (int i = 0; i <= 40; ++i) bent.push_back({0.05 * i, 0.0});        // 2 m north
  for (int i = 1; i <= 40; ++i) bent.push_back({2.0 + 0.0, 0.05 * i});  // then east (hard corner)
  const double far = max_preview_curvature(PathView(bent), 0, bent[0], 0.4, 6);
  CHECK(far > 0.0);
}

}  // namespace

int main() {
  test_distance();
  test_angle_wrap();
  test_heading_delta();
  test_segment_heading();
  test_perpendicular_distance();
  test_line_intersection();
  test_project_onto_segment();
  test_project_onto_path();
  test_project_onto_path_sign_east_leg();
  test_windowed_search_agrees_with_full_scan_near_the_hint();
  test_project_onto_path_tiny();
  test_path_length_and_cumulative();
  test_resample();
  test_resample_fixed();
  test_curvature();
  test_max_preview_curvature();
  return TEST_MAIN_RESULT();
}
