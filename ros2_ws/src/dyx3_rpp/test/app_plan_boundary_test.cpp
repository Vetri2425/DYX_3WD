#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "dyx3_mission/path_artifact.hpp"
#include "dyx3_rpp/path_conditioner.hpp"

TEST(AppPlanBoundary, BackendEncodedArtifactReconstructsSubmittedRuns) {
  // Regenerate with backend/tests/generate_app_plan_fixture.py; never hand-edit the artifact.
  const std::string path = std::string(DYX3_FIXTURES) + "/app_plan_boundary.dyx3path";
  std::ifstream file(path, std::ios::binary);
  ASSERT_TRUE(file.good());
  const std::string bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  const auto art = dyx3_mission::parse_artifact(bytes);
  ASSERT_TRUE(art.ok) << art.error;
  std::vector<dyx3_rpp::Point> pts;
  dyx3_rpp::Flags flags;
  for (const auto& p : art.artifact.points) {
    pts.push_back({p.north_m, p.east_m});
    flags.push_back(static_cast<unsigned char>(p.flags & 1));
  }
  const auto runs = dyx3_rpp::split_runs_by_flag(pts, flags);
  ASSERT_EQ(runs.size(), 3u);
  const std::vector<std::vector<dyx3_rpp::Point>> expected{
      {{0, 0}, {1, 0}}, {{1, 0}, {2, 0}, {2, 1}}, {{2, 1}, {3, 1}}};
  const std::vector<unsigned char> sprays{0, 1, 0};
  for (size_t r = 0; r < runs.size(); ++r) {
    ASSERT_EQ(runs[r].pts.size(), expected[r].size());
    ASSERT_EQ(runs[r].flags.size(), expected[r].size());
    for (size_t i = 0; i < expected[r].size(); ++i) {
      EXPECT_DOUBLE_EQ(runs[r].pts[i].n, expected[r][i].n);
      EXPECT_DOUBLE_EQ(runs[r].pts[i].e, expected[r][i].e);
      EXPECT_EQ(runs[r].flags[i], sprays[r]);
    }
  }
  EXPECT_TRUE(art.artifact.points[1].must_hit());
  EXPECT_TRUE(art.artifact.points[2].must_hit());
  EXPECT_TRUE(art.artifact.points[3].must_hit());
}
