// GATE 3 (spec 7.2): numeric equivalence of dyx3_geometry against the Python ancestors.
//
// The vectors in test/fixtures/gate3_*.txt are produced by tools/gate3/gen_geometry_vectors.py
// from the VERBATIM PX4_DXP controller module (not re-implemented, not hand-written) over the
// archived missions in Git. This test parses them and checks every case.
//
// Acceptance: |a-b| <= 1e-12 * max(1, |a|, |b|) for floats, exact for integers/flags/hints.
// The maximum observed absolute difference and ULP distance per function are printed so the
// real margin is on record (HANDOFF), not just "passed".
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
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

constexpr double kTol = 1e-12;

struct Stats {
  long n = 0;
  double max_abs = 0.0;
  std::int64_t max_ulp = 0;
  long failed = 0;
};
std::map<std::string, Stats>& stats() {
  static std::map<std::string, Stats> s;
  return s;
}

std::int64_t ordered(double x) {
  std::int64_t i;
  std::memcpy(&i, &x, sizeof i);
  return i < 0 ? std::numeric_limits<std::int64_t>::min() - i : i;
}

// Compare one float; record statistics under `fn`.
bool near(const std::string& fn, double got, double want, const std::string& ctx) {
  Stats& st = stats()[fn];
  ++st.n;
  ++dyx3_test::checks();
  if (!std::isfinite(got) || !std::isfinite(want)) {
    const bool same = std::isnan(got) == std::isnan(want) && (std::isnan(got) || got == want);
    if (!same) {
      ++st.failed;
      ++dyx3_test::failures();
      if (st.failed <= 5)
        std::fprintf(stderr, "%s: non-finite mismatch got=%g want=%g  [%s]\n", fn.c_str(), got,
                     want, ctx.c_str());
    }
    return same;
  }
  const double d = std::fabs(got - want);
  st.max_abs = std::max(st.max_abs, d);
  const std::int64_t u = std::llabs(ordered(got) - ordered(want));
  st.max_ulp = std::max(st.max_ulp, u);
  if (d <= kTol * std::max({1.0, std::fabs(got), std::fabs(want)})) return true;
  ++st.failed;
  ++dyx3_test::failures();
  if (st.failed <= 5)
    std::fprintf(stderr, "%s: got=%.17g want=%.17g diff=%.3g  [%s]\n", fn.c_str(), got, want, d,
                 ctx.c_str());
  return false;
}

bool same_int(const std::string& fn, long got, long want, const std::string& ctx) {
  Stats& st = stats()[fn + ":int"];
  ++st.n;
  ++dyx3_test::checks();
  if (got == want) return true;
  ++st.failed;
  ++dyx3_test::failures();
  if (st.failed <= 5)
    std::fprintf(stderr, "%s: int got=%ld want=%ld [%s]\n", fn.c_str(), got, want, ctx.c_str());
  return false;
}

std::vector<std::string> tokens(const std::string& line) {
  std::vector<std::string> t;
  std::istringstream is(line);
  std::string w;
  while (is >> w) t.push_back(w);
  return t;
}
double num(const std::string& s) { return std::strtod(s.c_str(), nullptr); }
long inum(const std::string& s) { return std::strtol(s.c_str(), nullptr, 10); }

struct PathData {
  std::vector<Point> pts;
  std::vector<unsigned char> flags;
};
std::map<std::string, PathData> g_paths;

std::size_t arrow(const std::vector<std::string>& t) {
  for (std::size_t i = 0; i < t.size(); ++i)
    if (t[i] == "->") return i;
  return t.size();
}

void scalar(const std::vector<std::string>& t) {
  const std::string& fn = t[1];
  const std::size_t a = arrow(t);
  const std::string ctx = fn;
  auto in = [&](std::size_t k) { return num(t[2 + k]); };
  auto out = [&](std::size_t k) { return num(t[a + 1 + k]); };
  if (fn == "angle_wrap") {
    near(fn, angle_wrap(in(0)), out(0), t[2]);
  } else if (fn == "heading_delta") {
    near(fn, heading_delta(in(0), in(1)), out(0), ctx);
  } else if (fn == "distance") {
    near(fn, distance({in(0), in(1)}, {in(2), in(3)}), out(0), ctx);
  } else if (fn == "segment_heading") {
    near(fn, segment_heading({in(0), in(1)}, {in(2), in(3)}), out(0), ctx);
  } else if (fn == "perp") {
    near(fn, perpendicular_distance({in(0), in(1)}, {in(2), in(3)}, {in(4), in(5)}), out(0), ctx);
  } else if (fn == "line_intersection") {
    auto r = line_intersection({in(0), in(1)}, {in(2), in(3)}, {in(4), in(5)}, {in(6), in(7)});
    const long want = inum(t[a + 1]);
    if (same_int(fn, r.has_value() ? 1 : 0, want, ctx) && want == 1 && r) {
      near(fn, r->n, out(1), ctx);
      near(fn, r->e, out(2), ctx);
    }
  } else if (fn == "path_length") {
    const auto& p = g_paths.at(t[2]);
    near(fn, path_length(PathView(p.pts)), out(0), t[2]);
  } else {
    ++dyx3_test::failures();
    std::fprintf(stderr, "unknown scalar function %s\n", fn.c_str());
  }
}

void sequence(const std::vector<std::string>& head, std::ifstream& in) {
  const std::string kind = head[1];
  const std::string pname = head[2];
  const long count = inum(head[3]);
  const PathData& p = g_paths.at(pname);
  ProjectionHint hint;
  for (long k = 0; k < count; ++k) {
    std::string line;
    if (!std::getline(in, line)) {
      ++dyx3_test::failures();
      std::fprintf(stderr, "truncated sequence %s\n", kind.c_str());
      return;
    }
    const auto t = tokens(line);
    const std::size_t a = arrow(t);
    const std::string ctx = pname + "#" + std::to_string(k);
    auto o = [&](std::size_t i) { return t[a + 1 + i]; };
    if (kind == "path_proj") {
      if (inum(t[3]) == 1) hint = ProjectionHint{};
      auto r = project_onto_path({num(t[1]), num(t[2])}, PathView(p.pts), hint);
      same_int(kind, r.seg_idx, inum(o(0)), ctx);
      near(kind, r.t, num(o(1)), ctx);
      near(kind, r.foot.n, num(o(2)), ctx);
      near(kind, r.foot.e, num(o(3)), ctx);
      near(kind, r.signed_cross, num(o(4)), ctx);
      same_int(kind + ".hint_valid", hint.valid ? 1 : 0, inum(o(5)), ctx);
      same_int(kind + ".hint_seg", hint.seg, inum(o(6)), ctx);
    } else if (kind == "seg_proj") {
      auto r = project_onto_segment({num(t[1]), num(t[2])}, PathView(p.pts),
                                    static_cast<int>(inum(t[3])));
      near(kind, r.t, num(o(0)), ctx);
      near(kind, r.foot.n, num(o(1)), ctx);
      near(kind, r.foot.e, num(o(2)), ctx);
      near(kind, r.signed_cross, num(o(3)), ctx);
      near(kind, r.dist_to_end_along, num(o(4)), ctx);
    } else if (kind == "max_preview") {
      const double k2 =
          max_preview_curvature(PathView(p.pts), static_cast<int>(inum(t[1])),
                                {num(t[2]), num(t[3])}, num(t[4]), static_cast<int>(inum(t[5])));
      near(kind, k2, num(o(0)), ctx);
    } else if (kind == "curvature_at") {
      near(kind, curvature_at(PathView(p.pts), static_cast<int>(inum(t[1])), num(t[2])), num(o(0)),
           ctx);
    } else if (kind == "cumulative") {
      static std::map<std::string, std::vector<double>> cache;
      auto it = cache.find(pname);
      if (it == cache.end()) it = cache.emplace(pname, cumulative_lengths(PathView(p.pts))).first;
      near(kind,
           static_cast<std::size_t>(k) < it->second.size() ? it->second[static_cast<std::size_t>(k)]
                                                           : -1.0,
           num(o(0)), ctx);
    } else if (kind == "resample") {
      // handled below (needs header params)
    }
  }
}

void resample_sequence(const std::vector<std::string>& head, std::ifstream& in) {
  const long count = inum(head[3]);
  const double spacing = num(head[4]);
  const bool use_flags = inum(head[5]) == 1;
  const std::size_t window = static_cast<std::size_t>(inum(head[6]));
  const PathData& p = g_paths.at(head[2]);
  const std::size_t n = std::min(window, p.pts.size());
  const std::vector<Point> sub(p.pts.begin(), p.pts.begin() + static_cast<long>(n));
  const std::vector<unsigned char> subf(p.flags.begin(), p.flags.begin() + static_cast<long>(n));
  const ResampleResult r =
      use_flags ? resample(sub, spacing, &subf) : resample(sub, spacing, nullptr);
  const std::string ctx = head[2] + " sp=" + head[4];
  same_int("resample.count", static_cast<long>(r.pts.size()), count, ctx);
  for (long k = 0; k < count; ++k) {
    std::string line;
    if (!std::getline(in, line)) return;
    const auto t = tokens(line);
    if (static_cast<std::size_t>(k) >= r.pts.size()) continue;
    near("resample", r.pts[static_cast<std::size_t>(k)].n, num(t[1]), ctx);
    near("resample", r.pts[static_cast<std::size_t>(k)].e, num(t[2]), ctx);
    same_int("resample.flag", r.flags[static_cast<std::size_t>(k)], inum(t[3]), ctx);
  }
}

bool load(const std::filesystem::path& file) {
  std::ifstream in(file);
  if (!in) return false;
  std::string line;
  std::getline(in, line);
  if (line != "GATE3 1") {
    std::fprintf(stderr, "%s: bad header\n", file.string().c_str());
    return false;
  }
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    const auto t = tokens(line);
    if (t[0] == "PATH") {
      PathData d;
      const long n = inum(t[2]);
      for (long i = 0; i < n; ++i) {
        std::getline(in, line);
        const auto q = tokens(line);
        d.pts.push_back({num(q[0]), num(q[1])});
        d.flags.push_back(static_cast<unsigned char>(inum(q[2])));
      }
      std::getline(in, line);  // END
      g_paths[t[1]] = std::move(d);
    } else if (t[0] == "S") {
      scalar(t);
    } else if (t[0] == "SEQ") {
      if (t[1] == "resample") {
        resample_sequence(t, in);
      } else {
        sequence(t, in);
      }
      std::getline(in, line);  // END
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: gate3_equivalence_test <fixtures-dir>\n");
    return 2;
  }
  int files = 0;
  for (const auto& e : std::filesystem::directory_iterator(argv[1])) {
    const std::string name = e.path().filename().string();
    if (name.rfind("gate3_", 0) == 0 && e.path().extension() == ".txt") {
      std::printf("loading %s\n", name.c_str());
      if (!load(e.path())) ++dyx3_test::failures();
      ++files;
    }
  }
  if (files == 0) {
    std::fprintf(stderr, "no gate3_*.txt fixtures in %s: GATE 3 cannot pass vacuously\n", argv[1]);
    return 1;
  }
  long total = 0;
  std::printf("%-26s %9s %12s %9s\n", "function", "cases", "max|diff|", "max ULP");
  for (const auto& [fn, st] : stats()) {
    std::printf("%-26s %9ld %12.3e %9lld%s\n", fn.c_str(), st.n, st.max_abs,
                static_cast<long long>(st.max_ulp), st.failed ? "   <-- FAILED" : "");
    total += st.n;
  }
  // A vacuous pass is a failure: the corpus must actually exercise every function.
  const char* required[] = {"angle_wrap", "heading_delta",     "distance",     "segment_heading",
                            "perp",       "line_intersection", "path_length",  "path_proj",
                            "seg_proj",   "max_preview",       "curvature_at", "cumulative",
                            "resample"};
  for (const char* r : required) {
    const long need = std::strcmp(r, "path_length") == 0 ? 10 : 50;  // one per corpus path
    if (stats().find(r) == stats().end() || stats()[r].n < need) {
      ++dyx3_test::failures();
      std::fprintf(stderr, "function %s has too few cases (%ld)\n", r,
                   stats().count(r) ? stats()[r].n : 0L);
    }
  }
  std::printf("total compared values: %ld\n", total);
  return TEST_MAIN_RESULT();
}
