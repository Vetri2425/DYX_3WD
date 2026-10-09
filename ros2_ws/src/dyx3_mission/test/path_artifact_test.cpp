// C++ reader vs the Python writer. The artifacts in test/fixtures were produced by
// tools/gen_path_artifact_fixtures.py from archived missions with the Python implementation; the
// manifest columns (hash, counts, first/last point) are what Python computed. Nothing is
// hand-written.
#include "dyx3_mission/path_artifact.hpp"

#include <gtest/gtest.h>
#include <sys/stat.h>

#include <chrono>
#include <cmath>
#include <filesystem>
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

// MS-004: the size is checked before the file is read.
class ArtifactSizeCap : public ::testing::Test {
protected:
  void SetUp() override {
    dir_ = std::filesystem::temp_directory_path() /
           ("dyx3_artifact_cap_" +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(dir_);
  }
  void TearDown() override {
    std::error_code ec;
    std::filesystem::permissions(dir_ / file(), std::filesystem::perms::owner_all, ec);
    std::filesystem::remove_all(dir_, ec);
  }
  static std::string sha() { return std::string(64, 'c'); }
  static std::string file() { return sha() + ".dyx3path"; }
  std::filesystem::path dir_;
};

TEST_F(ArtifactSizeCap, OversizedFileIsRefusedWithoutBeingRead) {
  // A sparse file just over the default limit, with all permissions removed: opening or reading
  // it would fail with a different error, so "too large" proves the size was checked first.
  const auto p = dir_ / file();
  {
    std::ofstream(p, std::ios::binary) << "x";
  }
  std::filesystem::resize_file(p, kMaxArtifactBytes + 1);
  chmod(p.c_str(), 0);
  const auto res = load_artifact(dir_.string(), sha());
  EXPECT_FALSE(res.ok);
  EXPECT_NE(res.error.find("too large"), std::string::npos) << res.error;
  EXPECT_NE(res.error.find(std::to_string(kMaxArtifactBytes + 1)), std::string::npos) << res.error;
}

TEST_F(ArtifactSizeCap, TheLimitIsInclusiveAndConfigurable) {
  const auto rows = manifest();
  const std::string bytes = read(rows[0].sha);
  const auto p = dir_ / (rows[0].sha + ".dyx3path");
  {
    std::ofstream(p, std::ios::binary) << bytes;
  }
  EXPECT_TRUE(load_artifact(dir_.string(), rows[0].sha, bytes.size()).ok);
  const auto res = load_artifact(dir_.string(), rows[0].sha, bytes.size() - 1);
  EXPECT_FALSE(res.ok);
  EXPECT_NE(res.error.find("too large"), std::string::npos) << res.error;
  EXPECT_TRUE(load_artifact(dir_.string(), rows[0].sha).ok);  // default limit is far above a path
  EXPECT_GE(kMaxArtifactBytes, 20ULL * 1024 * 1024);          // not below the backend upload limit
}

TEST_F(ArtifactSizeCap, NonRegularFilesAreRefused) {
  std::filesystem::create_directory(dir_ / file());  // a directory with the artifact's name
  const auto res = load_artifact(dir_.string(), sha());
  EXPECT_FALSE(res.ok);
  EXPECT_NE(res.error.find("regular file"), std::string::npos) << res.error;
}

TEST(ConditionedArtifact, DeterministicHashSourceIdentityAndStrictFailures) {
  ConditionedRunArtifact run;
  run.profile = 1;
  run.points = {{1.25, -2.5}, {3.0, 4.0}};
  run.flags = {0, 1};
  run.must_hit = {1, 0};
  const std::string source(64, 'a');
  const std::vector<ConditionedRunArtifact> runs{run};
  const std::string one = serialize_conditioned_artifact(source, "spacing=0.25", runs);
  const std::string two = serialize_conditioned_artifact(source, "spacing=0.25", runs);
  ASSERT_FALSE(one.empty());
  EXPECT_EQ(one, two);
  const std::string hash = sha256_hex(one);
  const auto parsed = parse_conditioned_artifact(one, hash);
  ASSERT_TRUE(parsed.ok) << parsed.error;
  EXPECT_EQ(parsed.artifact.source_sha256, source);
  ASSERT_EQ(parsed.artifact.runs.size(), 1U);
  EXPECT_EQ(parsed.artifact.runs[0].points[1].north_m, 3.0);
  EXPECT_EQ(parsed.artifact.runs[0].flags[1], 1);
  EXPECT_NE(sha256_hex(serialize_conditioned_artifact(source, "spacing=0.5", runs)), hash);
  EXPECT_FALSE(parse_conditioned_artifact(one, std::string(64, 'b')).ok);
  EXPECT_FALSE(parse_conditioned_artifact(one + "junk\n").ok);
}
