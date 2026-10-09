// run_store — the runs directory as a bounded store: free space, sizes, retention. Contract:
// docs/contracts/dyx3_recorder.md section 7. Pure C++ (std + POSIX statvfs).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace dyx3_recorder {

// Bytes available to an unprivileged writer on the file system holding `path` (or its nearest
// existing parent). 0 if it cannot be determined.
uint64_t fs_free_bytes(const std::string& path);
// Recursive size of the regular files under `dir` (0 if missing).
uint64_t dir_bytes(const std::string& dir);

struct PruneResult {
  std::vector<std::string> removed;  // run directory names, oldest first
  uint64_t bytes_before{0};
  uint64_t bytes_after{0};
  uint64_t bytes_removed{0};
  std::vector<std::string> errors;
};

// Retention: while the run directories under `root` exceed `max_bytes`, delete the oldest
// COMPLETE run (one that has summary.json). Never deletes `keep` (the active run, a full path or a
// name), a directory without summary.json (open, or not yet marked), or anything that is not a
// directory. Oldest = lexicographic order of the names (they start with the UTC start stamp).
PruneResult prune_runs(const std::string& root, uint64_t max_bytes, const std::string& keep = "");

// REC-009: a run directory without summary.json at recorder start belongs to a run that was cut
// short (recorder crash, power cut, SIGKILL). Each gets a minimal summary.json with final_state
// INTERRUPTED, so it is visibly incomplete (and becomes prunable). `now_utc` goes into the note.
// Returns the names of the directories marked.
std::vector<std::string> mark_interrupted_runs(const std::string& root, const std::string& now_utc);

}  // namespace dyx3_recorder
