// C++ reader vs the Python writer. The artifacts in test/fixtures were produced by
// tools/gen_path_artifact_fixtures.py from archived missions with the Python implementation; the
// manifest columns (hash, counts, first/last point) are what Python computed. Nothing is
// hand-written.
#include "dyx3_mission/path_artifact.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "dyx3_mission/sha256.hpp"

using namespace dyx3_mission;  // NOLINT

namespace {

struct Row {
  std::string name, sha;
  std::size_t n, spray, must;
  double first_n, first_e, last_n, last_e;
};

std::vector<Row> manifest() {
  std::ifstream in(std::string(DYX3_FIXTURES) + "/manifest.txt");
  std::vector<Row> rows;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream is(line);
    Row r;
    is >> r.name >> r.sha >> r.n >> r.spray >> r.must >> r.first_n >> r.first_e >> r.last_n >>
        r.last_e;
    if (is) rows.push_back(r);
  }
  return rows;
}

std::string read(const std::string& sha) {
  std::ifstream in(std::string(DYX3_FIXTURES) + "/" + sha + ".dyx3path", std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

std::string replace_first(std::string s, const std::string& from, const std::string& to) {
  const auto p = s.find(from);
  if (p != std::string::npos) s.replace(p, from.size(), to);
  return s;
}

}  // namespace

TEST(PathArtifact, ManifestIsPresent) { EXPECT_GE(manifest().size(), 2U); }

TEST(PathArtifact, CppHashEqualsPythonHashAndParsesIdentically) {
  for (const auto& r : manifest()) {
    SCOPED_TRACE(r.name);
    const std::string bytes = read(r.sha);
    ASSERT_FALSE(bytes.empty());
    EXPECT_EQ(sha256_hex(bytes), r.sha);  // C++ SHA-256 == Python hashlib on a real artifact
    const auto res = parse_artifact(bytes, r.sha);
    ASSERT_TRUE(res.ok) << res.error;
    const auto& a = res.artifact;
    EXPECT_EQ(a.sha256, r.sha);
    EXPECT_EQ(a.version, 1);
    ASSERT_EQ(a.points.size(), r.n);
    std::size_t spray = 0, must = 0;
    for (const auto& p : a.points) {
      spray += p.spray();
      must += p.must_hit();
    }
    EXPECT_EQ(spray, r.spray);
    EXPECT_EQ(must, r.must);
    // strtod must reproduce Python repr(float) bit for bit
    EXPECT_EQ(a.points.front().north_m, r.first_n);
    EXPECT_EQ(a.points.front().east_m, r.first_e);
    EXPECT_EQ(a.points.back().north_m, r.last_n);
    EXPECT_EQ(a.points.back().east_m, r.last_e);
    EXPECT_FALSE(a.engine_id.empty());
    EXPECT_EQ(a.meta_json.front(), '{');
  }
}

TEST(PathArtifact, LoadVerifiesTheContentHashAgainstTheFileName) {
  const auto rows = manifest();
  ASSERT_FALSE(rows.empty());
  EXPECT_TRUE(load_artifact(DYX3_FIXTURES, rows[0].sha).ok);
  EXPECT_FALSE(load_artifact(DYX3_FIXTURES, std::string(64, '0')).ok);  // not present
  EXPECT_FALSE(load_artifact(DYX3_FIXTURES, "XYZ").ok);                 // malformed id
  EXPECT_FALSE(load_artifact(DYX3_FIXTURES, std::string(64, 'A')).ok);  // upper case refused
  EXPECT_FALSE(load_artifact(DYX3_FIXTURES, "../" + rows[0].sha).ok);   // no path tricks
}

TEST(PathArtifact, RefusesWhatTheWriterWouldRefuse) {
  const auto rows = manifest();
  const std::string good = read(rows[0].sha);
  ASSERT_TRUE(parse_artifact(good).ok);
  // Same mutation list as the Python tests: every one must be refused.
  const std::vector<std::pair<std::string, std::string>> bad = {
      {"DYX3PATH 1", "DYX3PATH 2"}, {"DYX3PATH", "DYX3PATX"}, {"frame local_ned", "frame enu"},
      {"points 161", "points 162"}, {"\nend 161\n", "\n"},    {"\n", "\r\n"},
  };
  for (const auto& [from, to] : bad) {
    std::string m = good;
    if (from == "\n") {
      std::string out;
      for (char c : m) out += (c == '\n') ? "\r\n" : std::string(1, c);
      m = out;
    } else {
      m = replace_first(m, from, to);
    }
    EXPECT_FALSE(parse_artifact(m).ok) << "mutation accepted: " << from << " -> " << to;
  }
  EXPECT_FALSE(parse_artifact(good + "extra\n").ok);
  EXPECT_FALSE(parse_artifact(good.substr(0, good.size() - 1)).ok);  // no final newline
  EXPECT_FALSE(parse_artifact("").ok);
  EXPECT_FALSE(parse_artifact("DYX3PATH 1\n").ok);
  // a corrupted coordinate or flag
  std::string m = good;
  const auto pos = m.find("\n", m.find("points ")) + 1;
  m.replace(pos, 3, "nan");
  EXPECT_FALSE(parse_artifact(m).ok);
  // flags out of range (last token of the first point line)
  m = good;
  const auto eol = m.find('\n', pos);
  m.replace(eol - 1, 1, "9");
  EXPECT_FALSE(parse_artifact(m).ok);
  // hash mismatch
  EXPECT_FALSE(parse_artifact(good, std::string(64, '0')).ok);
}
