// mission_progress — the persisted progress of a mission, one file per source path artifact. See
// docs/contracts/dyx3_mission.md section 9a.
//
// Pure C++ (no ROS). The node keeps the latest record of every artifact in memory (read once at
// construction, updated whenever it queues a write), so StartMission(resume) decides from memory
// and never reads a file inside the service callback; the writes run off the executor thread.
//
// File: <missions_dir>/progress/<path_artifact_sha256>.json, written atomically (temporary file,
// fsync, rename, fsync of the directory), so a reader sees the previous record or the new one,
// never a partial file. Content (one line, keys in this order):
//   {"schema":1,"path_artifact_sha256":"<64 hex>","mission_id":N,"run_index":N,"point_index":N,
//    "completed_points":[0,1,...],"state":"RUNNING","updated_utc":"2026-10-10T12:00:00Z"}
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace dyx3_mission {

struct MissionProgress {
  static constexpr int kSchema = 1;

  /// The SOURCE artifact (StartMission.path_artifact_sha256): the execution artifact of an anchored
  /// path depends on the EKF reference, which a transient (a PX4 reboot) may change.
  std::string path_artifact_sha256;
  std::uint32_t mission_id = 0;  ///< the execution that wrote the record
  /// The first run not yet completed: RPP runs its runs in order, so the run it reports is the one
  /// in progress and every earlier run is complete. A resume starts RPP at the start of this run.
  std::uint32_t run_index = 0;
  std::uint32_t point_index = 0;  ///< the active must-hit point when the record was written
  /// Must-hit points (rank among the must-hit vertices) whose PointResult was issued while driving
  /// (COMPLETED, SKIPPED, or FAILED by a bypass), ascending. The journal resolves points in path
  /// order, so this is always 0..k-1; the points FAILED by the end-of-execution sweep of an
  /// ABORTED / ERROR execution are not in it (they were never reached).
  std::vector<std::uint32_t> completed_points;
  std::string state;  ///< MissionState name of the execution when the record was written
  std::string updated_utc;

  /// A COMPLETED execution leaves nothing to resume.
  bool complete() const { return state == "COMPLETED"; }
};

/// Largest progress file the reader accepts. A mission is capped at 50k points by the backend, so
/// the longest point list is under 400 kB. DERIVED — NOT FROM V1 SPEC: a bound against a wrong or
/// hostile file, not a tuning value.
constexpr std::uintmax_t kMaxProgressBytes = 4ULL * 1024 * 1024;

/// The one-line JSON record above (with a final LF).
std::string serialize_progress(const MissionProgress& p);

/// Strict reader of exactly that schema: every key present once, no other key, schema 1, a 64
/// lowercase hex id, a known state name, `completed_points` = 0..k-1. False (and `error`)
/// otherwise.
bool parse_progress(const std::string& bytes, MissionProgress* out, std::string* error);

/// <missions_dir>/progress
std::string progress_dir(const std::string& missions_dir);
/// <missions_dir>/progress/<sha>.json
std::string progress_file(const std::string& missions_dir, const std::string& sha256);

/// Atomic write (temporary file + fsync + rename + directory fsync); creates the directory.
bool write_progress(const std::string& missions_dir, const MissionProgress& p, std::string* error);

/// Every valid record in <missions_dir>/progress, keyed by its artifact id (a file must be named
/// after the id it holds). Unreadable, oversized or invalid files are skipped and reported in
/// `warnings`; a missing directory is an empty result.
std::map<std::string, MissionProgress> load_progress_dir(const std::string& missions_dir,
                                                         std::vector<std::string>* warnings);

/// The system clock as "YYYY-MM-DDThh:mm:ssZ" (a record of when, never used for an age).
std::string utc_now_iso8601();

}  // namespace dyx3_mission
