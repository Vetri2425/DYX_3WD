// fcu_provenance — the FCU parameter and version read as the recorder sees it (REC-025): which
// script and argv to run, and what its two output files (params_fcu.json, versions_fcu.json,
// written by tools/px4/param_dump.py) say. Contract: docs/contracts/dyx3_recorder.md section 4.
// Pure C++ (std + POSIX), no ROS.
#pragma once

#include <string>
#include <vector>

namespace dyx3_recorder {

// Minimal JSON access for the small flat files the recorder handles (no JSON library in this
// package). The top-level member `key` of a JSON object as raw JSON text (e.g. "\"abc\"", "true",
// "{...}"). False when `json` is not a well-formed object or has no such member.
bool json_member(const std::string& json, const std::string& key, std::string* raw);
// The value of a raw JSON string token, unescaped. False if `raw` is not a string token (or uses an
// escape outside ASCII, which the files read here never contain).
bool json_string(const std::string& raw, std::string* out);
// `json` (an object) with the top-level `key` set to the string `value`: the old value replaced in
// place, or the member appended when absent; every other byte is kept. False when `json` is not a
// well-formed object.
bool json_set_string(const std::string& json, const std::string& key, const std::string& value,
                     std::string* out);

// What the dump left in the run directory.
struct FcuDumpFiles {
  // params_fcu.json is a JSON object written by the dump ("source": "mavlink"), not the recorder's
  // placeholder.
  bool params_written{false};
  bool params_complete{false};
  std::string params_status;  // "complete", "incomplete", "unavailable" (empty when not written)
  std::string params_reason;
  bool version_written{false};    // versions_fcu.json likewise
  std::string firmware_git_hash;  // 16 hex characters in commit order; empty when unavailable
  std::string version_reason;
};
FcuDumpFiles read_fcu_dump(const std::string& params_path, const std::string& versions_path);

// The dump script, in this order: $DYX3_RELEASE_DIR/tools/px4/param_dump.py (when `release_dir_env`
// is set); the first ancestor of `exe_path` holding tools/px4/param_dump.py (the release the
// running binary belongs to:
// <release>/ros2_ws/install/dyx3_recorder/lib/dyx3_recorder/recorder_node);
// /opt/dyx3/current/tools/px4/param_dump.py. Empty when none exists; `searched` (optional) lists
// what was tried.
std::string find_param_dump_script(const std::string& exe_path, const char* release_dir_env,
                                   std::string* searched);

// `preferred` when it is an executable file, else "python3" (looked up in PATH by execvp).
std::string resolve_dump_python(const std::string& preferred);

// Each element of `tmpl` with {python}, {script} and {dir} replaced.
std::vector<std::string> expand_dump_argv(const std::vector<std::string>& tmpl,
                                          const std::string& python, const std::string& script,
                                          const std::string& dir);
bool argv_uses(const std::vector<std::string>& tmpl, const std::string& placeholder);

// True when the FCU's 16-character hash and the release's expected 40-character SHA disagree on
// their common prefix. False when either is empty (nothing to compare).
bool firmware_mismatch(const std::string& running_hash, const std::string& expected_sha);

}  // namespace dyx3_recorder
