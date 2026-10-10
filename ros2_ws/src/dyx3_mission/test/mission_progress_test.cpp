// Persisted mission progress (docs/contracts/dyx3_mission.md section 9a): the record's exact bytes,
// the strict reader, the atomic write and the directory scan the node runs at construction.
#include "dyx3_mission/mission_progress.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace dyx3_mission;  // NOLINT
namespace fs = std::filesystem;

namespace {

const std::string kSha(64, 'a');
const std::string kOther(64, 'b');

MissionProgress sample(const std::string& sha = kSha) {
  MissionProgress p;
  p.path_artifact_sha256 = sha;
  p.mission_id = 7;
  p.run_index = 2;
  p.point_index = 3;
  p.completed_points = {0, 1, 2};
  p.state = "PAUSED";
  p.updated_utc = "2026-10-10T12:00:00Z";
  return p;
}

std::string read_file(const fs::path& p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

class ProgressDir : public ::testing::Test {
protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("dyx3_mission_progress_test_" + std::to_string(getpid()) + "_" +
            ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(dir_);
    fs::create_directories(dir_);
  }
  void TearDown() override { fs::remove_all(dir_); }
  void put(const std::string& name, const std::string& bytes) {
    fs::create_directories(progress_dir(dir_.string()));
    std::ofstream(fs::path(progress_dir(dir_.string())) / name, std::ios::binary) << bytes;
  }
  fs::path dir_;
};

}  // namespace

TEST(MissionProgress, TheRecordIsOneLineInTheContractsKeyOrder) {
  EXPECT_EQ(
      serialize_progress(sample()),
      "{\"schema\":1,\"path_artifact_sha256\":\"" + kSha +
          "\",\"mission_id\":7,\"run_index\":2,\"point_index\":3,\"completed_points\":[0,1,2],"
          "\"state\":\"PAUSED\",\"updated_utc\":\"2026-10-10T12:00:00Z\"}\n");
  MissionProgress empty = sample();
  empty.completed_points.clear();
  EXPECT_NE(serialize_progress(empty).find("\"completed_points\":[]"), std::string::npos);
}

TEST(MissionProgress, RoundTripsAndOnlyCompletedIsComplete) {
  MissionProgress back;
  std::string err;
  ASSERT_TRUE(parse_progress(serialize_progress(sample()), &back, &err)) << err;
  EXPECT_EQ(back.path_artifact_sha256, kSha);
  EXPECT_EQ(back.mission_id, 7U);
  EXPECT_EQ(back.run_index, 2U);
  EXPECT_EQ(back.point_index, 3U);
  EXPECT_EQ(back.completed_points, (std::vector<std::uint32_t>{0, 1, 2}));
  EXPECT_EQ(back.state, "PAUSED");
  EXPECT_EQ(back.updated_utc, "2026-10-10T12:00:00Z");
  for (const char* s :
       {"PLACING", "ARMING", "ENGAGING", "READY", "RUNNING", "PAUSED", "ABORTED", "ERROR"}) {
    MissionProgress p = sample();
    p.state = s;
    EXPECT_FALSE(p.complete()) << s;  // resumable
  }
  MissionProgress done = sample();
  done.state = "COMPLETED";
  EXPECT_TRUE(done.complete());
  // Whitespace between tokens is accepted (a hand-inspected, pretty-printed file still reads).
  const std::string pretty = "{ \"schema\": 1, \"path_artifact_sha256\": \"" + kSha +
                             "\",\n \"mission_id\": 7, \"run_index\": 0, \"point_index\": 0,\n"
                             " \"completed_points\": [ ], \"state\": \"ERROR\", \"updated_utc\": "
                             "\"x\" }\n";
  EXPECT_TRUE(parse_progress(pretty, &back, &err)) << err;
}

TEST(MissionProgress, TheReaderIsStrict) {
  const std::string good = serialize_progress(sample());
  const auto bad = [](const std::string& bytes) {
    MissionProgress p;
    std::string err;
    const bool ok = parse_progress(bytes, &p, &err);
    EXPECT_FALSE(err.empty() && !ok);
    return !ok;
  };
  const auto replaced = [&good](const std::string& from, const std::string& to) {
    std::string s = good;
    const auto at = s.find(from);
    EXPECT_NE(at, std::string::npos) << from;
    return s.replace(at, from.size(), to);
  };
  EXPECT_TRUE(bad(""));
  EXPECT_TRUE(bad("[]"));
  EXPECT_TRUE(bad(replaced("\"schema\":1", "\"schema\":2")));  // another schema
  EXPECT_TRUE(bad(replaced("\"mission_id\":7,", "")));         // missing key
  EXPECT_TRUE(bad(replaced("\"mission_id\":7", "\"mission_id\":7,\"mission_id\":7")));  // twice
  EXPECT_TRUE(bad(replaced("\"mission_id\":7", "\"mission_id\":7,\"extra\":1")));       // unknown
  EXPECT_TRUE(bad(replaced("\"run_index\":2", "\"run_index\":02")));          // leading zero
  EXPECT_TRUE(bad(replaced("\"run_index\":2", "\"run_index\":-2")));          // negative
  EXPECT_TRUE(bad(replaced("\"run_index\":2", "\"run_index\":4294967296")));  // > uint32
  EXPECT_TRUE(bad(replaced("[0,1,2]", "[0,2]")));  // not 0..k-1: not a record the node wrote
  EXPECT_TRUE(bad(replaced("[0,1,2]", "[1,2,3]")));
  EXPECT_TRUE(bad(replaced("[0,1,2]", "[0,1,]")));
  EXPECT_TRUE(bad(replaced("\"PAUSED\"", "\"SLEEPING\"")));  // unknown state
  EXPECT_TRUE(bad(replaced(kSha, std::string(64, 'A'))));    // not lowercase hex
  EXPECT_TRUE(bad(replaced(kSha, "abc")));
  EXPECT_TRUE(bad(replaced("\"2026-10-10T12:00:00Z\"", "\"a\\\"b\"")));  // no escapes
  EXPECT_TRUE(bad(good + "{}"));                                         // trailing bytes
  EXPECT_TRUE(bad(good.substr(0, good.size() / 2)));                     // truncated
}

TEST_F(ProgressDir, WriteIsAtomicOverwritesAndLeavesNoTemporaryFile) {
  std::string err;
  ASSERT_TRUE(write_progress(dir_.string(), sample(), &err)) << err;  // creates progress/
  const fs::path file = progress_file(dir_.string(), kSha);
  EXPECT_EQ(file, dir_ / "progress" / (kSha + ".json"));
  EXPECT_EQ(read_file(file), serialize_progress(sample()));
  MissionProgress next = sample();
  next.run_index = 0;
  next.completed_points.clear();
  next.state = "LOADING";
  ASSERT_TRUE(write_progress(dir_.string(), next, &err)) << err;  // resume=false overwrites
  EXPECT_EQ(read_file(file), serialize_progress(next));
  std::size_t entries = 0;
  for (const auto& e : fs::directory_iterator(dir_ / "progress")) {
    ++entries;
    EXPECT_EQ(e.path().extension(), ".json") << e.path();
  }
  EXPECT_EQ(entries, 1U);
  // A record that could not be read back is never written.
  MissionProgress wrong = sample();
  wrong.path_artifact_sha256 = "../../etc/passwd";
  EXPECT_FALSE(write_progress(dir_.string(), wrong, &err));
}

TEST_F(ProgressDir, TheScanReadsEveryValidRecordAndSkipsTheRest) {
  std::vector<std::string> warnings;
  EXPECT_TRUE(load_progress_dir(dir_.string(), &warnings).empty());  // no directory yet
  EXPECT_TRUE(warnings.empty());
  std::string err;
  ASSERT_TRUE(write_progress(dir_.string(), sample(kSha), &err)) << err;
  MissionProgress done = sample(kOther);
  done.state = "COMPLETED";
  ASSERT_TRUE(write_progress(dir_.string(), done, &err)) << err;
  const std::string c(64, 'c');
  put(c + ".json", serialize_progress(sample(kSha)));  // holds another artifact's progress
  put(std::string(64, 'd') + ".json", "{not json");
  put("notes.json", serialize_progress(sample(kSha)));
  put(kSha + ".json.tmp.1234", "partial");  // left by a crash mid-write: not a record
  const auto m = load_progress_dir(dir_.string(), &warnings);
  ASSERT_EQ(m.size(), 2U);
  EXPECT_EQ(m.at(kSha).run_index, 2U);
  EXPECT_FALSE(m.at(kSha).complete());
  EXPECT_TRUE(m.at(kOther).complete());
  EXPECT_EQ(m.count(c), 0U);
  EXPECT_EQ(warnings.size(), 3U);
}

TEST(MissionProgress, UtcStampShape) {
  const std::string t = utc_now_iso8601();
  ASSERT_EQ(t.size(), 20U);
  EXPECT_EQ(t[4], '-');
  EXPECT_EQ(t[10], 'T');
  EXPECT_EQ(t.back(), 'Z');
}
