// Evidence replay: dyx3_geometry projection vs cross-track RECORDED by the prototype in a field
// bag.
//
// Fixture: test/fixtures/bag_xtrack_*.txt, produced by tools/extract_geometry_bag_fixture.py from a
// PX4_DXP rosbag2 (not in Git: HANDOFF "LOCAL ACTIONS NEEDED"). Format:
//     BAGXTRACK 1
//     PATH <name> <n>            (n lines: north east flag)   END
//     TICK <pos_n> <pos_e> <recorded_xtrack_m>
// For each tick the C++ closest-segment projection (full scan, no hint) of the pose onto the
// conditioned path is compared with the `/rpp/debug[0]` value the prototype published. The
// extractor keeps only segment-profile TRACK_SEGMENT ticks of single-run missions, because
// `/rpp/conditioned_path` concatenates all runs and the per-run segment index is not recoverable.
//
// The bag pose and the controller's pose differ by up to one control period of motion, so this is a
// STATISTICAL sanity check, not bit equivalence (that is GATE 3). Thresholds below are
// DERIVED - NOT FROM V1 SPEC: they are proposed starting points to be tuned against real data.
//
// Exit code 77 = SKIPPED (no fixture yet): a skipped test is not a passing test.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "check.hpp"
#include "dyx3_geometry/project_onto_path.hpp"

using namespace dyx3_geometry;  // NOLINT

namespace {
constexpr double kMedianAbsMax = 0.002;  // 2 mm
constexpr double kP99AbsMax = 0.010;     // 1 cm
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) return 2;
  std::vector<std::filesystem::path> files;
  for (const auto& e : std::filesystem::directory_iterator(argv[1])) {
    const std::string n = e.path().filename().string();
    if (n.rfind("bag_xtrack_", 0) == 0 && e.path().extension() == ".txt") files.push_back(e.path());
  }
  if (files.empty()) {
    std::printf("SKIPPED: no bag_xtrack_*.txt fixture (extract one from a PX4_DXP bag)\n");
    return 77;
  }
  for (const auto& f : files) {
    std::ifstream in(f);
    std::string line;
    std::getline(in, line);
    if (line != "BAGXTRACK 1") {
      std::fprintf(stderr, "%s: bad header\n", f.string().c_str());
      return 1;
    }
    std::vector<Point> path;
    std::vector<double> diffs;
    while (std::getline(in, line)) {
      std::istringstream is(line);
      std::string w;
      is >> w;
      if (w == "PATH") {
        std::string name;
        long n = 0;
        is >> name >> n;
        for (long i = 0; i < n; ++i) {
          std::getline(in, line);
          std::istringstream q(line);
          double a, b;
          int fl;
          q >> a >> b >> fl;
          path.push_back({a, b});
        }
        std::getline(in, line);  // END
      } else if (w == "TICK") {
        double pn, pe, rec;
        is >> pn >> pe >> rec;
        ProjectionHint cold;  // full scan every tick
        auto r = project_onto_path({pn, pe}, PathView(path), cold);
        diffs.push_back(std::fabs(r.signed_cross - rec));
      }
    }
    CHECK(diffs.size() > 100);
    if (diffs.empty()) continue;
    std::sort(diffs.begin(), diffs.end());
    const double median = diffs[diffs.size() / 2];
    const double p99 =
        diffs[static_cast<std::size_t>(0.99 * static_cast<double>(diffs.size() - 1))];
    std::printf("%s: n=%zu median|d|=%.4f m p99|d|=%.4f m max=%.4f m\n",
                f.filename().string().c_str(), diffs.size(), median, p99, diffs.back());
    CHECK(median <= kMedianAbsMax);
    CHECK(p99 <= kP99AbsMax);
  }
  return TEST_MAIN_RESULT();
}
