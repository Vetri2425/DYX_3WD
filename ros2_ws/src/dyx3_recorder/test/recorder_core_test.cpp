#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

#include "dyx3_recorder/bag_writer.hpp"
#include "dyx3_recorder/param_snapshot.hpp"
#include "dyx3_recorder/run_lifecycle.hpp"
#include "dyx3_recorder/run_manifest.hpp"
#include "dyx3_recorder/run_store.hpp"
#include "dyx3_recorder/ulog_capture.hpp"
#include "ulog_synth.hpp"

using namespace dyx3_recorder;
namespace fs = std::filesystem;

namespace {
struct TmpDir {
  std::string path;
  TmpDir() {
    path = (fs::temp_directory_path() / ("dyx3_rec_" + std::to_string(getpid()) + "_" +
                                         std::to_string(reinterpret_cast<uintptr_t>(this))))
               .string();
    fs::create_directories(path);
  }
  ~TmpDir() {
    std::error_code ec;
    fs::remove_all(path, ec);
  }
};
std::string slurp(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  std::ostringstream o;
  o << f.rdbuf();
  return o.str();
}
}  // namespace

TEST(Json, EscapingAndNonFiniteAndDeterminism) {
  EXPECT_EQ(json_escape("a\"b\\c\n\t\x01"), "a\\\"b\\\\c\\n\\t\\u0001");
  JsonObject o;
  o.str("k", "v").num("x", NAN).num("y", 0.1).integer("n", -3).boolean("b", true).str_list(
      "l", {"a", "b\"c"});
  const std::string a = o.dump();
  EXPECT_NE(a.find("\"x\": null"), std::string::npos);
  EXPECT_NE(a.find("\"y\": 0.10000000000000001"), std::string::npos);  // %.17g: round-trips exactly
  EXPECT_NE(a.find("\"l\": [\"a\", \"b\\\"c\"]"), std::string::npos);
  EXPECT_EQ(a, o.dump());
  EXPECT_EQ(JsonObject().dump(), "{}");
}

TEST(RunNaming, UtcStampMissionIdAndCollisionSuffix) {
  const time_t t = 1788617730;  // 2026-09-05 14:15:30 UTC
  EXPECT_EQ(iso_utc(t), "2026-09-05T14:15:30Z");
  EXPECT_EQ(run_dir_name(t, 42, 0), "2026-09-05_141530_mission_0042");
  EXPECT_EQ(run_dir_name(t, 42, 3), "2026-09-05_141530_mission_0042_run3");
  TmpDir d;
  const std::string n = run_dir_name(t, 42, 0);
  EXPECT_EQ(unique_run_path(d.path, n), d.path + "/" + n);
  fs::create_directories(d.path + "/" + n);
  EXPECT_EQ(unique_run_path(d.path, n), d.path + "/" + n + "_2");
  fs::create_directories(d.path + "/" + n + "_2");
  EXPECT_EQ(unique_run_path(d.path, n), d.path + "/" + n + "_3");
}

TEST(Manifest, ContainsEveryProvenanceField) {
  RunInfo r;
  r.run_id = "r1";
  r.mission_id = 42;
  r.run_index = 1;
  r.path_artifact_sha256 = std::string(64, 'a');
  r.start_utc = "2026-09-05T14:15:30Z";
  r.vehicle_id = "3wd-01";
  r.operator_name = "op \"x\"";
  r.hostname = "jetson";
  const std::string m = manifest_json(r);
  for (const char* k : {"run_id", "mission_id", "run_index", "path_artifact_sha256", "start_utc",
                        "vehicle_id", "operator", "hostname"}) {
    EXPECT_NE(m.find(std::string("\"") + k + "\""), std::string::npos) << k;
  }
  EXPECT_NE(m.find("op \\\"x\\\""), std::string::npos);
  RunSummary s;
  s.notes = {"a", "b"};
  s.provenance_complete = false;
  const std::string sj = summary_json(s);
  EXPECT_NE(sj.find("\"provenance_complete\": false"), std::string::npos);
  EXPECT_NE(sj.find("\"notes\": [\"a\", \"b\"]"), std::string::npos);
}

TEST(ConfigSnapshot, SecretsNeverReachTheRunDirectory) {
  for (const char* n :
       {"ntrip.env", "platform.env", "ntrip_profile.json", "api_token", "wifi_psk.conf", "my.key",
        "cert.pem", "PASSWORD.txt", "secrets.yaml", "x.token", "hotspot.env", "backend.env",
        "usb-receiver.env", "auth.json", "authorized_keys", "id_ed25519", "store.jks"}) {
    EXPECT_TRUE(is_secret_name(n)) << n;
  }
  for (const char* n : {"rpp.yaml", "spray.yaml", "guard.json", "limits.txt", "mission.yaml",
                        "motion_guard.yaml", "versions.json", "mavlink-router.conf"})
    EXPECT_FALSE(is_secret_name(n)) << n;
  TmpDir src, dst;
  fs::create_directories(src.path + "/rpp");
  fs::create_directories(src.path + "/secrets");
  std::ofstream(src.path + "/rpp/params.yaml") << "a: 1\n";
  std::ofstream(src.path + "/spray.yaml") << "b: 2\n";
  std::ofstream(src.path + "/ntrip.env") << "PASSWORD=hunter2\n";
  std::ofstream(src.path + "/rpp/my.key") << "KEY\n";
  std::ofstream(src.path + "/secrets/x.yaml") << "c: 3\n";
  const CopyResult r = copy_config_tree(src.path, dst.path + "/snap");
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.copied, 2U);
  EXPECT_GE(r.excluded, 3U);
  EXPECT_TRUE(fs::exists(dst.path + "/snap/rpp/params.yaml"));
  EXPECT_TRUE(fs::exists(dst.path + "/snap/spray.yaml"));
  EXPECT_FALSE(fs::exists(dst.path + "/snap/ntrip.env"));
  EXPECT_FALSE(fs::exists(dst.path + "/snap/rpp/my.key"));
  EXPECT_FALSE(fs::exists(dst.path + "/snap/secrets"));
  for (const auto& e : fs::recursive_directory_iterator(dst.path)) {
    if (e.is_regular_file()) EXPECT_EQ(slurp(e.path().string()).find("hunter2"), std::string::npos);
  }
  EXPECT_FALSE(copy_config_tree(src.path + "/nope", dst.path + "/x").ok);
}

TEST(AtomicWrite, ReplacesWholeFileOrNothing) {
  TmpDir d;
  EXPECT_TRUE(write_file_atomic(d.path + "/f.json", "one"));
  EXPECT_TRUE(write_file_atomic(d.path + "/f.json", "two"));
  EXPECT_EQ(slurp(d.path + "/f.json"), "two");
  EXPECT_FALSE(fs::exists(d.path + "/f.json.tmp"));
  EXPECT_FALSE(write_file_atomic(d.path + "/missing/f.json", "x"));
  // REC-010: a failed rename (the target is a non-empty directory) is reported and leaves no
  // temp file behind
  fs::create_directories(d.path + "/busy.json/x");
  EXPECT_FALSE(write_file_atomic(d.path + "/busy.json", "x"));
  EXPECT_FALSE(fs::exists(d.path + "/busy.json.tmp"));
  const std::string big(1 << 20, 'z');  // partial writes are looped
  EXPECT_TRUE(write_file_atomic(d.path + "/big", big));
  EXPECT_EQ(slurp(d.path + "/big"), big);
}

TEST(ParamSnapshot, SortedByteIdenticalAndUnreachableIsExplicit) {
  std::vector<NodeParams> a{
      {"spray", true, {{"spray", "b", "double", "2"}, {"spray", "a", "bool", "true"}}},
      {"guard", false, {}}};
  std::vector<NodeParams> b{a[1], {"spray", true, {a[0].params[1], a[0].params[0]}}};
  const std::string j1 = params_ros_snapshot_json("t", a);
  EXPECT_EQ(j1, params_ros_snapshot_json("t", b));
  EXPECT_LT(j1.find("guard"), j1.find("spray"));
  EXPECT_LT(j1.find("\"a\""), j1.find("\"b\""));
  EXPECT_NE(j1.find("\"reachable\": false"), std::string::npos);
  const std::string file = params_ros_file_json(j1, "");
  EXPECT_NE(file.find("\"end\": null"), std::string::npos);
  EXPECT_NE(unavailable_json("fcu", "why").find("\"status\": \"unavailable\""), std::string::npos);
}

TEST(UlogCapture, RecordsGapsDuplicatesOutOfOrderAndWrap) {
  using namespace ulog_synth;
  TmpDir d;
  UlogCapture u;
  const Stream st = make_stream(60, 65534);  // the sequence wraps after the second chunk
  ASSERT_GE(st.chunks.size(), 10U);
  ASSERT_TRUE(u.open(d.path + "/s.ulg"));  // open before the stream: written as it arrives
  auto feed = [&](size_t i) {
    const auto& c = st.chunks[i];
    return u.on_chunk(c.seq, c.first_message_offset, c.data.data(), c.data.size());
  };
  EXPECT_TRUE(feed(0));
  EXPECT_TRUE(feed(1));
  EXPECT_FALSE(feed(1));  // duplicate
  EXPECT_TRUE(feed(2));   // 65535 -> 0: wrap, no gap
  EXPECT_TRUE(u.gaps().empty());
  feed(3);
  const uint64_t at_gap = u.bytes();
  feed(6);  // 4 and 5 lost
  ASSERT_EQ(u.gaps().size(), 1U);
  EXPECT_EQ(u.gaps()[0].expected_seq, st.chunks[4].seq);
  EXPECT_EQ(u.gaps()[0].got_seq, st.chunks[6].seq);
  EXPECT_EQ(u.gaps()[0].missing_chunks, 2U);
  EXPECT_EQ(u.gaps()[0].file_offset, at_gap);
  EXPECT_EQ(u.gaps()[0].resync_offset, st.chunks[6].first_message_offset);
  EXPECT_FALSE(feed(5));  // late chunk from before the gap: out of order, dropped
  for (size_t i = 7; i < st.chunks.size(); ++i) feed(i);
  EXPECT_EQ(u.duplicates(), 1U);
  EXPECT_EQ(u.out_of_order(), 1U);
  EXPECT_TRUE(u.close());
  EXPECT_EQ(u.header_status(), "complete");
  std::string types, why;
  EXPECT_TRUE(parse(slurp(d.path + "/s.ulg"), true, &types, &why)) << why;  // whole messages only
  EXPECT_EQ(types.substr(0, 5), "BFIPA");
  EXPECT_LT(std::count(types.begin(), types.end(), 'D'), 59);  // the lost chunks' messages
  EXPECT_NE(u.gaps_json().find("\"missing_chunks\": 2"), std::string::npos);
  EXPECT_NE(u.gaps_json().find("\"header\": \"complete\""), std::string::npos);
}

// REC-005: the FCU streams the header once; a run opened mid-stream must still start with it.
TEST(UlogCapture, EveryRunFileStartsWithTheCachedHeaderAndWholeMessages) {
  using namespace ulog_synth;
  TmpDir d;
  UlogCapture u;
  const Stream st = make_stream(200);
  const size_t n = st.chunks.size();
  size_t i = 0;
  for (; i < n / 3; ++i)  // no run open: the capture only caches
    u.on_chunk(st.chunks[i].seq, st.chunks[i].first_message_offset, st.chunks[i].data.data(),
               st.chunks[i].data.size());
  EXPECT_EQ(u.header_state(), UlogCapture::HeaderState::kComplete);
  for (int run = 0; run < 2; ++run) {
    const std::string path = d.path + "/run" + std::to_string(run) + ".ulg";
    ASSERT_TRUE(u.open(path));
    const size_t stop = run == 0 ? 2 * n / 3 : n;
    for (; i < stop; ++i)
      u.on_chunk(st.chunks[i].seq, st.chunks[i].first_message_offset, st.chunks[i].data.data(),
                 st.chunks[i].data.size());
    EXPECT_TRUE(u.close());
    EXPECT_EQ(u.header_status(), "complete");
    const std::string f = slurp(path);
    EXPECT_EQ(f.compare(0, 16, std::string(st.bytes.begin(), st.bytes.begin() + 16)), 0);
    std::string types, why;
    ASSERT_TRUE(parse(f, true, &types, &why)) << run << ": " << why;
    // definitions, then the subscription (cached), then data; the mid-stream parameter change is
    // carried into the second run's header
    EXPECT_EQ(types.substr(0, 5), "BFIPA") << types.substr(0, 12);
    if (run == 1) EXPECT_EQ(types.substr(5, 1), "P");
    EXPECT_GT(std::count(types.begin(), types.end(), 'D'), 30);
    EXPECT_EQ(u.gaps().size(), 0U);
  }
}

TEST(UlogCapture, NoHeaderSeenIsMarkedAndTheFileStillHoldsWholeMessages) {
  using namespace ulog_synth;
  TmpDir d;
  UlogCapture u;
  const Stream st = make_stream(100);
  ASSERT_TRUE(u.open(d.path + "/s.ulg"));  // the recorder started after the stream did
  for (size_t i = 3; i < st.chunks.size(); ++i)
    u.on_chunk(st.chunks[i].seq, st.chunks[i].first_message_offset, st.chunks[i].data.data(),
               st.chunks[i].data.size());
  EXPECT_TRUE(u.close());
  EXPECT_NE(u.header_status().find("incomplete: no header"), std::string::npos);
  EXPECT_FALSE(u.run_header_complete());
  std::string types, why;
  EXPECT_TRUE(parse(slurp(d.path + "/s.ulg"), false, &types, &why)) << why;  // starts at a boundary
  EXPECT_GT(types.size(), 50U);
}

TEST(UlogCapture, AGapInTheDefinitionsLosesTheHeaderAndAStreamRestartRollsTheFile) {
  using namespace ulog_synth;
  TmpDir d;
  UlogCapture u;
  const Stream st = make_stream(40);
  const auto small = chunk(st.bytes, st.starts, 0, 32);  // the definitions span several chunks
  ASSERT_GT(st.data_start, 64U);
  u.on_chunk(small[0].seq, small[0].first_message_offset, small[0].data.data(),
             small[0].data.size());
  for (size_t i = 2; i < small.size() / 2; ++i)  // chunk 1 (inside the definitions) lost
    u.on_chunk(small[i].seq, small[i].first_message_offset, small[i].data.data(),
               small[i].data.size());
  EXPECT_EQ(u.header_state(), UlogCapture::HeaderState::kLost);
  // a run opened now cannot have a header
  ASSERT_TRUE(u.open(d.path + "/stream.ulg"));
  EXPECT_NE(u.header_status().find("header lost"), std::string::npos);
  for (size_t i = small.size() / 2; i < small.size(); ++i)
    u.on_chunk(small[i].seq, small[i].first_message_offset, small[i].data.data(),
               small[i].data.size());
  // the FCU stream restarts (px4_link re-sent LOGGING_START): a file that has data rolls
  const Stream again = make_stream(40);
  for (const auto& c : again.chunks)
    u.on_chunk(c.seq, c.first_message_offset, c.data.data(), c.data.size());
  EXPECT_EQ(u.header_state(), UlogCapture::HeaderState::kComplete);
  EXPECT_TRUE(u.close());
  EXPECT_EQ(u.segments(), 2U);
  std::string types, why;
  EXPECT_TRUE(parse(slurp(d.path + "/stream_2.ulg"), true, &types, &why)) << why;
  EXPECT_EQ(types.substr(0, 5), "BFIPA");
  EXPECT_TRUE(u.gaps().empty());  // a restart at sequence 0 is not a gap
}

TEST(UlogCapture, ClosedCaptureWritesNothing) {
  UlogCapture u;
  const uint8_t a[1] = {1};
  EXPECT_FALSE(u.on_chunk(0, 0, a, 1));
  EXPECT_FALSE(u.open("/nonexistent_dir_dyx3/x.ulg"));
}

TEST(Lifecycle, StartsOnRunningStopsOnTerminalAndSplitsRuns) {
  RunLifecycle l;
  auto a = l.on_mission(kMissionRunning, 1, 0);  // RUNNING without READY (recorder restarted)
  EXPECT_TRUE(a.start);
  EXPECT_TRUE(a.start_running);
  EXPECT_FALSE(a.stop);
  EXPECT_TRUE(l.recording());
  for (uint8_t s : {kMissionPaused, kMissionRunning, kMissionReady, kMissionLoading}) {
    a = l.on_mission(s, 1, 0);
    EXPECT_FALSE(a.start || a.stop || a.running) << int(s);
  }
  a = l.on_mission(kMissionRunning, 2, 0);  // a different mission while recording
  EXPECT_TRUE(a.stop && a.start);
  EXPECT_EQ(a.final_state, "SUPERSEDED");
  a = l.on_mission(kMissionRunning, 2, 1);  // next run of the same mission
  EXPECT_TRUE(a.stop && a.start);
  a = l.on_mission(kMissionAborted, 2, 1);
  EXPECT_TRUE(a.stop);
  EXPECT_FALSE(a.start);
  EXPECT_EQ(a.final_state, "ABORTED");
  EXPECT_FALSE(l.recording());
  EXPECT_FALSE(l.on_mission(kMissionCompleted, 2, 1).stop);  // nothing open: nothing to stop
  EXPECT_FALSE(l.on_mission(kMissionLoading, 3, 0).start);   // LOADING does not open a run
  l.on_mission(kMissionRunning, 3, 0);
  EXPECT_EQ(l.on_mission(kMissionError, 3, 0).final_state, "ERROR");
  l.on_mission(kMissionRunning, 4, 0);
  EXPECT_EQ(l.on_mission(kMissionIdle, 0, 0).final_state, "IDLE");
}

TEST(Lifecycle, PreRollOpensAtReadyAndMarksRunning) {
  RunLifecycle l;
  auto a = l.on_mission(kMissionReady, 7, 0);  // the rover is still stopped: the bag starts now
  EXPECT_TRUE(a.start);
  EXPECT_FALSE(a.start_running);
  EXPECT_FALSE(l.running_seen());
  EXPECT_FALSE(l.on_mission(kMissionReady, 7, 0).start);  // repeated READY: nothing
  a = l.on_mission(kMissionRunning, 7, 0);
  EXPECT_TRUE(a.running);
  EXPECT_FALSE(a.start || a.stop);
  EXPECT_TRUE(l.running_seen());
  EXPECT_FALSE(l.on_mission(kMissionRunning, 7, 0).running);  // marked once
  a = l.on_mission(kMissionCompleted, 7, 0);
  EXPECT_TRUE(a.stop);
  EXPECT_EQ(a.final_state, "COMPLETED");
  EXPECT_FALSE(l.recording());
}

TEST(Lifecycle, ReadyThenIdleClosesAsNotStarted) {
  RunLifecycle l;
  EXPECT_TRUE(l.on_mission(kMissionReady, 8, 0).start);
  auto a = l.on_mission(kMissionIdle, 0, 0);
  EXPECT_TRUE(a.stop);
  EXPECT_EQ(a.final_state, "NOT_STARTED");
  EXPECT_NE(a.stop_note.find("IDLE"), std::string::npos);
  EXPECT_FALSE(l.recording());
  // READY of another mission while pre-rolling: the old pre-roll closes NOT_STARTED
  l.on_mission(kMissionReady, 9, 0);
  a = l.on_mission(kMissionReady, 10, 0);
  EXPECT_TRUE(a.stop && a.start);
  EXPECT_EQ(a.final_state, "NOT_STARTED");
  // RUNNING of another mission after a real run: SUPERSEDED
  l.on_mission(kMissionRunning, 10, 0);
  a = l.on_mission(kMissionReady, 10, 1);
  EXPECT_TRUE(a.stop && a.start);
  EXPECT_EQ(a.final_state, "SUPERSEDED");
  EXPECT_EQ(l.on_mission(kMissionAborted, 10, 1).final_state, "NOT_STARTED");
}

// REC-001 retention: oldest complete runs go first; the active run and runs without summary.json
// are never deleted.
TEST(RunStore, PruneDeletesOldestCompleteRunsOnly) {
  TmpDir d;
  auto make = [&](const std::string& name, size_t bytes, bool complete) {
    fs::create_directories(d.path + "/" + name + "/rosbag2");
    std::ofstream(d.path + "/" + name + "/rosbag2/data", std::ios::binary)
        << std::string(bytes, 'x');
    if (complete) std::ofstream(d.path + "/" + name + "/summary.json") << "{}";
  };
  make("2026-01-01_000000_mission_0001", 1000, false);  // oldest but interrupted: never pruned
  make("2026-01-02_000000_mission_0002", 1000, true);
  make("2026-01-03_000000_mission_0003", 1000, true);
  make("2026-01-04_000000_mission_0004", 1000, true);
  make("2026-01-05_000000_mission_0005", 1000, false);              // the active run
  std::ofstream(d.path + "/stray_file") << std::string(5000, 'y');  // not a run: not counted
  const uint64_t total = dir_bytes(d.path + "/2026-01-02_000000_mission_0002");
  EXPECT_EQ(total, 1002U);
  PruneResult r = prune_runs(d.path, 3 * total, d.path + "/2026-01-05_000000_mission_0005");
  ASSERT_EQ(r.removed.size(), 2U);
  EXPECT_EQ(r.removed[0], "2026-01-02_000000_mission_0002");
  EXPECT_EQ(r.removed[1], "2026-01-03_000000_mission_0003");
  EXPECT_LE(r.bytes_after, 3 * total);
  EXPECT_TRUE(fs::exists(d.path + "/2026-01-01_000000_mission_0001"));
  EXPECT_TRUE(fs::exists(d.path + "/2026-01-04_000000_mission_0004"));
  EXPECT_TRUE(fs::exists(d.path + "/2026-01-05_000000_mission_0005"));
  EXPECT_TRUE(fs::exists(d.path + "/stray_file"));
  // a budget nothing can meet: everything complete goes, the rest stays
  r = prune_runs(d.path, 0, "2026-01-05_000000_mission_0005");
  EXPECT_EQ(r.removed.size(), 1U);
  EXPECT_GT(r.bytes_after, 0U);
  EXPECT_TRUE(fs::exists(d.path + "/2026-01-01_000000_mission_0001"));
  EXPECT_TRUE(fs::exists(d.path + "/2026-01-05_000000_mission_0005"));
  EXPECT_TRUE(prune_runs(d.path + "/missing", 0).removed.empty());
  EXPECT_GT(fs_free_bytes(d.path), 0U);
  EXPECT_GT(fs_free_bytes(d.path + "/not/yet/created"), 0U);  // nearest existing parent
}

TEST(RunStore, RunsWithoutSummaryAreMarkedInterrupted) {
  TmpDir d;
  fs::create_directories(d.path + "/a_done/rosbag2");
  std::ofstream(d.path + "/a_done/summary.json") << "{\"final_state\": \"COMPLETED\"}";
  fs::create_directories(d.path + "/b_cut/rosbag2");
  std::ofstream(d.path + "/b_cut/rosbag2/b_cut_0.db3") << std::string(500, 'x');
  fs::create_directories(d.path + "/c_cut_finalised/rosbag2");
  std::ofstream(d.path + "/c_cut_finalised/rosbag2/metadata.yaml") << "m";
  const auto marked = mark_interrupted_runs(d.path, "2026-09-05T14:15:30Z");
  ASSERT_EQ(marked.size(), 2U);
  EXPECT_EQ(marked[0], "b_cut");
  EXPECT_EQ(slurp(d.path + "/a_done/summary.json"), "{\"final_state\": \"COMPLETED\"}");
  const std::string b = slurp(d.path + "/b_cut/summary.json");
  EXPECT_NE(b.find("\"final_state\": \"INTERRUPTED\""), std::string::npos);
  EXPECT_NE(b.find("\"bag_bytes\": 500"), std::string::npos);
  EXPECT_NE(b.find("metadata.yaml missing"), std::string::npos);
  EXPECT_NE(b.find("2026-09-05T14:15:30Z"), std::string::npos);
  EXPECT_NE(b.find("\"provenance_complete\": false"), std::string::npos);
  EXPECT_EQ(slurp(d.path + "/c_cut_finalised/summary.json").find("metadata.yaml missing"),
            std::string::npos);
  EXPECT_TRUE(mark_interrupted_runs(d.path, "x").empty());  // idempotent
}

// ---- bag supervision against a fake child
// -------------------------------------------------------------------------
TEST(BagWriter, ExecFailureIsReported) {
  BagWriter w;
  EXPECT_FALSE(w.start({"/nonexistent/ros2", "bag"}, "/tmp/x"));
  EXPECT_FALSE(w.running());
}

TEST(BagWriter, GrowsAndStopsOnSigint) {
  TmpDir d;
  BagWriter w;
  ASSERT_TRUE(w.start({"/bin/sh", "-c",
                       "trap 'exit 0' INT; mkdir -p \"$0\"; while :; do echo xxxxxxxxxx >> "
                       "\"$0/data\"; sleep 0.05; done",
                       d.path + "/bag"},
                      d.path + "/bag"));
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  EXPECT_TRUE(w.running());
  const uint64_t b1 = w.bytes();
  EXPECT_GT(b1, 0U);
  EXPECT_EQ(w.stop(3.0, 1.0), 0);  // exited on SIGINT, no escalation
  EXPECT_FALSE(w.running());
  EXPECT_EQ(w.last_exit_code(), 0);
}

TEST(BagWriter, EscalatesAgainstAChildThatIgnoresSigint) {
  BagWriter w;
  ASSERT_TRUE(w.start({"/bin/sh", "-c", "trap '' INT; while :; do sleep 0.05; done"}, "/tmp"));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(w.stop(0.3, 1.0), 1);  // SIGTERM kills it
  EXPECT_FALSE(w.running());
  BagWriter x;
  ASSERT_TRUE(x.start({"/bin/sh", "-c", "trap '' INT TERM; while :; do sleep 0.05; done"}, "/tmp"));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(x.stop(0.2, 0.2), 2);  // needs SIGKILL
  EXPECT_FALSE(x.running());
}

TEST(BagWriter, ChildDeathIsDetected) {
  BagWriter w;
  ASSERT_TRUE(w.start({"/bin/sh", "-c", "sleep 0.1; exit 3"}, "/tmp"));
  for (int i = 0; i < 100 && w.running(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_FALSE(w.running());
  EXPECT_EQ(w.last_exit_code(), 3);
  EXPECT_TRUE(w.exited_abnormally());
}

// REC-013: status polling (running/bytes, the status timer) concurrently with stop (the mission
// callback) on a multi-threaded executor. Run under -fsanitize=thread to prove there is no race.
TEST(BagWriter, StatusPollingDuringStopIsSafe) {
  TmpDir d;
  for (int round = 0; round < 3; ++round) {
    BagWriter w;
    ASSERT_TRUE(w.start({"/bin/sh", "-c",
                         "trap 'sleep 0.2; exit 0' INT; mkdir -p \"$0\"; while :; do echo x >> "
                         "\"$0/data\"; sleep 0.02; done",
                         d.path + "/bag" + std::to_string(round)},
                        d.path + "/bag" + std::to_string(round)));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::atomic<bool> done{false};
    std::atomic<int> polls{0};
    std::thread poller([&] {
      while (!done.load()) {
        (void)w.running();
        (void)w.bytes();
        (void)w.exited_abnormally();
        (void)w.last_exit_code();
        ++polls;
      }
    });
    EXPECT_EQ(w.stop(3.0, 1.0), 0);
    done = true;
    poller.join();
    EXPECT_GT(polls.load(), 0);
    EXPECT_FALSE(w.running());
    EXPECT_EQ(w.last_exit_code(), 0);  // the child's own exit code, reaped exactly once
  }
}
