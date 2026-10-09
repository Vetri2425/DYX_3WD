// run_manifest — deterministic JSON for the run directory, run naming, config snapshot. Contract:
// docs/contracts/dyx3_recorder.md sections 1 and 4. Pure C++ (std only), no ROS.
#pragma once

#include <cmath>
#include <cstdint>
#include <ctime>
#include <string>
#include <utility>
#include <vector>

namespace dyx3_recorder {

std::string json_escape(const std::string& s);

// Insertion-ordered flat JSON object builder. Non-finite numbers become null (JSON has no NaN).
class JsonObject {
public:
  JsonObject& str(const std::string& k, const std::string& v);
  JsonObject& num(const std::string& k, double v);
  JsonObject& integer(const std::string& k, int64_t v);
  JsonObject& boolean(const std::string& k, bool v);
  JsonObject& raw(const std::string& k, const std::string& json);  // already-valid JSON
  JsonObject& str_list(const std::string& k, const std::vector<std::string>& v);
  std::string dump(int indent = 2, int level = 0) const;
  bool empty() const { return items_.empty(); }

private:
  std::vector<std::pair<std::string, std::string>> items_;
};

struct RunInfo {
  std::string run_id;
  uint32_t mission_id{0};
  uint32_t run_index{0};
  std::string path_artifact_sha256;
  std::string start_utc;
  std::string start_state;  // mission state that opened the run: READY (pre-roll) or RUNNING
  std::string vehicle_id;
  std::string operator_name;
  std::string hostname;
  // FCU timesync at the start of the run (F-tasks A1.4). valid=false means no fresh sample: the
  // offset is then not a measurement.
  bool timesync_valid{false};
  int64_t timesync_offset_us{0};
  uint32_t timesync_round_trip_us{0};
};

struct RunSummary {
  std::string end_utc;
  std::string final_state;
  double duration_s{0.0};
  // When the mission reached RUNNING (motion allowed) and how long the bag had been recording by
  // then. Empty / NaN (-> null) when the run never reached RUNNING.
  std::string running_utc;
  double preroll_s{NAN};
  uint64_t bag_bytes{0};
  uint64_t ulog_bytes{0};
  uint64_t ulog_gaps{0};
  std::string ulog_header;  // "complete" or "incomplete: ..." (UlogCapture::header_status)
  bool bag_healthy_throughout{true};
  bool provenance_complete{true};
  bool timesync_valid_end{false};
  int64_t timesync_offset_us_end{0};
  uint32_t timesync_round_trip_us_end{0};
  std::vector<std::string> notes;
};

std::string manifest_json(const RunInfo& r);
std::string summary_json(const RunSummary& s);

// "2026-09-05T14:15:30Z" and "2026-09-05_141530" from a UTC time.
std::string iso_utc(time_t t);
std::string dir_stamp_utc(time_t t);
// "<stamp>_mission_<id:04d>"; a non-zero run_index appends "_run<n>".
std::string run_dir_name(time_t t, uint32_t mission_id, uint32_t run_index);
// Returns a path under `root` that does not exist yet (appends _2, _3 ... on collision).
std::string unique_run_path(const std::string& root, const std::string& name);

// Atomic text write (temp + rename). Returns false on any failure.
bool write_file_atomic(const std::string& path, const std::string& content);

// True for file names that must never be copied into a run directory (credentials, tokens, keys).
bool is_secret_name(const std::string& filename);
struct CopyResult {
  size_t copied{0};
  size_t excluded{0};
  bool ok{true};
  std::vector<std::string> errors;
};
// Recursive copy of `src` into `dst` excluding secret names. A missing `src` is ok=false with an
// error.
CopyResult copy_config_tree(const std::string& src, const std::string& dst);

}  // namespace dyx3_recorder
