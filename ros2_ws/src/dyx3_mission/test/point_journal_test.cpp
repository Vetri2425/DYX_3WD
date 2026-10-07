// Point journal: definitional geometry scenarios + a run over a real archived mission artifact.
#include "dyx3_mission/point_journal.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "dyx3_mission/path_artifact.hpp"

using namespace dyx3_mission;  // NOLINT

namespace {
std::vector<ArtifactPoint> line_with_points(std::initializer_list<double> must_hit_n) {
  // A north-heading line 0..10 m, 5 cm spacing; must-hit at the given north positions.
  std::vector<ArtifactPoint> p;
  for (int i = 0; i <= 200; ++i) {
    const double n = 0.05 * i;
    std::uint8_t f = kFlagSpray;
    for (double m : must_hit_n) {
      if (std::fabs(n - m) < 1e-9) f |= kFlagMustHit;
    }
    p.push_back({n, 0.0, f});
  }
  return p;
}
}  // namespace

TEST(PointJournal, ErrorIsTheClosestApproachAndIsReportedWhenTheRoverLeaves) {
  PointJournal j(line_with_points({0.0, 5.0, 10.0}), 0.10);
  EXPECT_EQ(j.point_count(), 3U);
  std::vector<PointEvent> got;
  // Drive north with a constant 3 cm east offset, 1 cm steps.
  for (int i = 0; i <= 1000; ++i) {
    auto ev = j.update(0.01 * i, 0.03);
    got.insert(got.end(), ev.begin(), ev.end());
  }
  auto tail = j.finish();
  got.insert(got.end(), tail.begin(), tail.end());
  ASSERT_EQ(got.size(), 3U);
  for (std::uint32_t k = 0; k < 3; ++k) {
    EXPECT_EQ(got[k].point_index, k);
    EXPECT_EQ(got[k].outcome, PointOutcome::kCompleted);
    EXPECT_NEAR(got[k].error_m, 0.03, 1e-9);  // lateral offset == closest approach
    EXPECT_NEAR(got[k].east_m, 0.03, 1e-12);
    EXPECT_GE(got[k].error_m, 0.0);
  }
  EXPECT_NEAR(got[1].north_m, 5.0, 0.005);
  EXPECT_FALSE(j.active_point().has_value());
}

TEST(PointJournal, ABypassedPointFailsWhenTheNextOneIsCapturedFirst) {
  PointJournal j(line_with_points({3.0, 6.0}), 0.10);
  std::vector<PointEvent> got;
  // Rover passes 1 m east of point 0 (never within the radius), then reaches point 1.
  for (int i = 0; i <= 700; ++i) {
    auto ev = j.update(0.01 * i, i < 450 ? 1.0 : 0.0);
    got.insert(got.end(), ev.begin(), ev.end());
  }
  auto tail = j.finish();
  got.insert(got.end(), tail.begin(), tail.end());
  ASSERT_EQ(got.size(), 2U);
  EXPECT_EQ(got[0].point_index, 0U);
  EXPECT_EQ(got[0].outcome, PointOutcome::kFailed);
  EXPECT_NEAR(got[0].north_m, 3.0, 1e-9);  // a FAILED point reports the planned vertex
  EXPECT_EQ(got[1].outcome, PointOutcome::kCompleted);
}

TEST(PointJournal, SkipAndFinishAndEdgeCases) {
  PointJournal j(line_with_points({2.0, 4.0, 8.0}), 0.10);
  ASSERT_TRUE(j.active_point().has_value());
  EXPECT_EQ(*j.active_point(), 0U);
  const auto s = j.skip();
  ASSERT_TRUE(s.has_value());
  EXPECT_EQ(s->outcome, PointOutcome::kSkipped);
  EXPECT_EQ(s->point_index, 0U);
  EXPECT_EQ(*j.active_point(), 1U);
  EXPECT_TRUE(j.update(std::nan(""), 0.0).empty());  // non-finite position carries no information
  const auto rest = j.finish();                      // never reached: the remaining two FAIL
  ASSERT_EQ(rest.size(), 2U);
  EXPECT_EQ(rest[0].outcome, PointOutcome::kFailed);
  EXPECT_EQ(rest[1].outcome, PointOutcome::kFailed);
  EXPECT_FALSE(j.skip().has_value());
  EXPECT_FALSE(j.active_point().has_value());

  EXPECT_THROW(PointJournal(line_with_points({1.0}), 0.0), std::invalid_argument);
  EXPECT_THROW(PointJournal(line_with_points({1.0}), -1.0), std::invalid_argument);
  PointJournal none(line_with_points({}), 0.1);
  EXPECT_EQ(none.point_count(), 0U);
  EXPECT_FALSE(none.active_point().has_value());
  EXPECT_TRUE(none.update(1.0, 1.0).empty());
}

TEST(PointJournal, CapturingWhenThePathEndsStillCompletes) {
  PointJournal j(line_with_points({10.0}), 0.10);
  std::vector<PointEvent> got;
  for (int i = 0; i <= 1000; ++i) {
    auto ev = j.update(0.01 * i, 0.0);
    got.insert(got.end(), ev.begin(), ev.end());
  }
  EXPECT_TRUE(got.empty());  // still inside the radius at the end of the drive
  const auto fin = j.finish();
  ASSERT_EQ(fin.size(), 1U);
  EXPECT_EQ(fin[0].outcome, PointOutcome::kCompleted);
  EXPECT_NEAR(fin[0].error_m, 0.0, 1e-9);
}

TEST(PointJournal, RealArchivedMissionDrivenWithALateralOffset) {
  // square_2x2 from the Git corpus (planned by the carried engine): 5 must-hit vertices on a closed
  // loop. A rover driving the planned points with a 2 cm offset must complete all five, each with
  // an error equal to that offset (to within the point spacing).
  std::ifstream in(std::string(DYX3_FIXTURES) + "/manifest.txt");
  std::string name, sha;
  in >> name >> sha;
  ASSERT_EQ(name, "square_2x2.dxf");
  const auto res = load_artifact(DYX3_FIXTURES, sha);
  ASSERT_TRUE(res.ok) << res.error;
  PointJournal j(res.artifact.points, 0.10);
  ASSERT_EQ(j.point_count(), 5U);
  std::vector<PointEvent> got;
  const auto& pts = res.artifact.points;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    // offset 2 cm perpendicular to the local direction of travel
    const auto& a = pts[i];
    const auto& b = pts[std::min(i + 1, pts.size() - 1)];
    const double dn = b.north_m - a.north_m, de = b.east_m - a.east_m;
    const double h = std::hypot(dn, de);
    const double ox = h > 0 ? -de / h * 0.02 : 0.0, oy = h > 0 ? dn / h * 0.02 : 0.0;
    auto ev = j.update(a.north_m + ox, a.east_m + oy);
    got.insert(got.end(), ev.begin(), ev.end());
  }
  const auto fin = j.finish();
  got.insert(got.end(), fin.begin(), fin.end());
  ASSERT_EQ(got.size(), 5U);
  for (std::uint32_t k = 0; k < 5; ++k) {
    EXPECT_EQ(got[k].point_index, k);
    EXPECT_EQ(got[k].outcome, PointOutcome::kCompleted) << "point " << k;
    EXPECT_LE(got[k].error_m, 0.03) << "point " << k;
  }
}
