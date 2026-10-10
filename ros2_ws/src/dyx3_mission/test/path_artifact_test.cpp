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

// MS-005: the reader must refuse what the Python decode() refuses. Every vector below was run
// through backend/src/dyx3_backend/mission/path_artifact.py decode(): the accepted ones are
// accepted there, the refused ones are refused there.
namespace {
std::string artifact(const std::string& meta, const std::vector<std::string>& points,
                     const std::string& head = "DYX3PATH 1", long count = -2) {
  const long n = count == -2 ? static_cast<long>(points.size()) : count;
  std::string s = head + "\nframe local_ned\nengine abcd\nmeta " + meta + "\npoints " +
                  std::to_string(n) + "\n";
  for (const auto& p : points) s += p + "\n";
  return s + "end " + std::to_string(n) + "\n";
}
const std::vector<std::string> kOnePoint = {"1.5 2.5 3"};
}  // namespace

TEST(PathArtifactCanonical, PythonSpellingsOfCoordinatesAreAccepted) {
  for (const char* t : {"1e-05", "1e+16", "100.0", "-0.0", "0.0001", "5e-324",
                        "1.7976931348623157e+308", "1000000000000000.0", "1.2345678901234568e+17",
                        "0.30000000000000004", "-2.5e-07", "12345.678"}) {
    const auto r = parse_artifact(artifact("{}", {std::string(t) + " 2.5 1"}));
    EXPECT_TRUE(r.ok) << t << ": " << r.error;
  }
}

TEST(PathArtifactCanonical, NonCanonicalOrNonFiniteCoordinatesAreRefused) {
  for (const char* t : {"1",
                        "1.50",
                        "+1.0",
                        "1e0",
                        "1.0E5",
                        "1e5",
                        "1e+5",
                        "0x10",
                        "0x1p3",
                        "inf",
                        "-inf",
                        "nan",
                        "1e400",
                        ".5",
                        "5.",
                        "-0",
                        "00.5",
                        "1e-5",
                        "1e+016",
                        "0.00001",
                        "1_0.0",
                        "1e16",
                        "10000000000000000.0"}) {
    EXPECT_FALSE(parse_artifact(artifact("{}", {std::string(t) + " 2.5 1"})).ok) << t;
    EXPECT_FALSE(parse_artifact(artifact("{}", {"2.5 " + std::string(t) + " 1"})).ok) << t;
  }
}

TEST(PathArtifactCanonical, CanonicalMetaIsAccepted) {
  for (const char* m :
       {"{}", "{\"a\":1.0,\"b\":[1,2.5,null,true,\"\\u00e9\"],\"c\":{\"d\":5e-324}}", "{\"\":1}",
        "{\"\\uffff\":1,\"\\ud83d\\ude00\":2}", "{\"a\":\"\\u007f\\u0001\\n\\\"\\\\\"}",
        "{\"a\":-0.0,\"b\":1e+16,\"c\":1e-05,\"d\":12345678901234567890}",
        "{\"a\":\"\\ud83d\\ude00\"}", "{\"z\":1,\"\\u00e9\":2}"}) {
    const auto r = parse_artifact(artifact(m, kOnePoint));
    EXPECT_TRUE(r.ok) << m << ": " << r.error;
    EXPECT_EQ(r.artifact.meta_json, m);
  }
}

TEST(PathArtifactCanonical, NonCanonicalOrInvalidMetaIsRefused) {
  for (const char* m :
       {"{\"b\":1,\"a\":2}",                     // keys not sorted
        "{\"a\":1,\"a\":2}",                     // duplicate key
        "{\"\\ud83d\\ude00\":1,\"\\uffff\":2}",  // sorted by UTF-16 unit, not by code point
        "{\"\\u00e9\":1,\"z\":2}",               // sorted by UTF-16 unit, not by code point
        "{ \"a\":1}",
        "{\"a\": 1}",
        "{\"a\":1 }",  // whitespace
        "{\"a\":1,}",
        "[]",
        "\"x\"",
        "1",
        "",
        "null",  // not a JSON object
        "{\"a\":1.0e2}",
        "{\"a\":1E2}",
        "{\"a\":1.50}",
        "{\"a\":-0}",  // number spelling
        "{\"a\":01}",
        "{\"a\":.5}",
        "{\"a\":1.}",
        "{\"a\":+1}",  // not JSON
        "{\"a\":NaN}",
        "{\"a\":Infinity}",
        "{\"a\":-Infinity}",
        "{\"a\":1e400}",
        "{\"a\":\"\\/\"}",
        "{\"a\":\"\\u0041\"}",
        "{\"a\":\"\\u00E9\"}",
        "{\"a\":\"\\u000a\"}",
        "{\"a\":\"\\x\"}",
        "{\"a\":tru}",
        "{\"a\":1}x",
        "{\"a\":[1,2}",
        "{\"a\":\"abc}"}) {
    EXPECT_FALSE(parse_artifact(artifact(m, kOnePoint)).ok) << m;
  }
  EXPECT_FALSE(parse_artifact(artifact("{\"a\":\"\xc3\xa9\"}", kOnePoint)).ok);  // raw non-ASCII
  std::string deep;  // nesting beyond the reader's bound is refused rather than recursed into
  for (int i = 0; i < 200; ++i) deep += "[";
  for (int i = 0; i < 200; ++i) deep += "]";
  EXPECT_FALSE(parse_artifact(artifact("{\"a\":" + deep + "}", kOnePoint)).ok);
}

TEST(PathArtifactCanonical, HeaderAndCountDeviationsAreRefused) {
  ASSERT_TRUE(parse_artifact(artifact("{}", kOnePoint)).ok);
  EXPECT_FALSE(parse_artifact(artifact("{}", kOnePoint, "DYX3PATH 2")).ok);  // unknown version
  EXPECT_FALSE(parse_artifact(artifact("{}", kOnePoint, "DYX3PATH 01")).ok);
  EXPECT_FALSE(parse_artifact(artifact("{}", kOnePoint, "DYX3PATH 1 ")).ok);
  EXPECT_FALSE(parse_artifact(artifact("{}", kOnePoint, "DYX3PATH 1", -1)).ok);  // negative count
  EXPECT_FALSE(parse_artifact(artifact("{}", {}, "DYX3PATH 1", 0)).ok);
  EXPECT_FALSE(parse_artifact(artifact("{}", kOnePoint, "DYX3PATH 1", 2)).ok);
  std::string plus = artifact("{}", kOnePoint);
  plus.replace(plus.find("points 1"), 8, "points +1");
  EXPECT_FALSE(parse_artifact(plus).ok);
  const std::string good = artifact("{}", kOnePoint);
  EXPECT_FALSE(parse_artifact(good + "extra\n").ok);  // trailing bytes
  EXPECT_FALSE(parse_artifact(good + "\n").ok);
  EXPECT_FALSE(parse_artifact(artifact("{}", {"1.5 2.5 4"})).ok);   // flags out of range
  EXPECT_FALSE(parse_artifact(artifact("{}", {"1.5 2.5 -1"})).ok);  // negative flags
  EXPECT_FALSE(parse_artifact(artifact("{}", {"1.5 2.5"})).ok);
  EXPECT_FALSE(parse_artifact(artifact("{}", {"1.5  2.5 3"})).ok);
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

// The C++ writer (dyx3_mission's execution artifact) reproduces the Python writer byte for byte.
TEST(PathArtifactWriter, ReproducesThePythonWriterExactly) {
  for (const auto& row : manifest()) {
    SCOPED_TRACE(row.name);
    const std::string bytes = read(row.sha);
    const auto r = parse_artifact(bytes, row.sha);
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(serialize_artifact(r.artifact.engine_id, r.artifact.meta_json, r.artifact.points),
              bytes);
  }
}

TEST(PathArtifactWriter, RefusesWhatHasNoCanonicalSpelling) {
  const std::vector<ArtifactPoint> pts = {{1.5, -2.25, 1}, {0.1, 1e-05, 3}};
  const std::string ok = serialize_artifact("eng", "{\"a\":1}", pts);
  ASSERT_FALSE(ok.empty());
  EXPECT_TRUE(parse_artifact(ok).ok);
  EXPECT_NE(ok.find("\n0.1 1e-05 3\n"), std::string::npos);
  EXPECT_TRUE(serialize_artifact("eng", "{\"a\":1}", {}).empty());
  EXPECT_TRUE(serialize_artifact("", "{\"a\":1}", pts).empty());
  EXPECT_TRUE(serialize_artifact("e g", "{\"a\":1}", pts).empty());
  EXPECT_TRUE(serialize_artifact("eng", "{\"b\":1,\"a\":2}", pts).empty());  // unsorted keys
  EXPECT_TRUE(serialize_artifact("eng", "[1]", pts).empty());
  EXPECT_TRUE(serialize_artifact("eng", "{}", {{std::nan(""), 0.0, 0}}).empty());
  EXPECT_TRUE(serialize_artifact("eng", "{}", {{0.0, 0.0, 4}}).empty());
}
