#include "dyx3_recorder/run_store.hpp"

#include <sys/statvfs.h>

#include <algorithm>
#include <cmath>
#include <filesystem>

#include "dyx3_recorder/run_manifest.hpp"

namespace dyx3_recorder {

namespace fs = std::filesystem;

uint64_t fs_free_bytes(const std::string& path) {
  struct statvfs s{};
  std::error_code ec;
  fs::path p = path;
  while (!p.empty() && !fs::exists(p, ec)) p = p.parent_path();
  if (p.empty() || statvfs(p.c_str(), &s) != 0) return 0;
  return static_cast<uint64_t>(s.f_bavail) * static_cast<uint64_t>(s.f_frsize);
}

uint64_t dir_bytes(const std::string& dir) {
  std::error_code ec;
  uint64_t total = 0;
  if (dir.empty() || !fs::is_directory(dir, ec)) return 0;
  for (auto it =
           fs::recursive_directory_iterator(dir, fs::directory_options::skip_permission_denied, ec);
       !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
    std::error_code e2;
    if (it->is_regular_file(e2)) {
      const uint64_t n = it->file_size(e2);
      if (!e2) total += n;
    }
  }
  return total;
}

PruneResult prune_runs(const std::string& root, uint64_t max_bytes, const std::string& keep) {
  PruneResult r;
  std::error_code ec;
  if (!fs::is_directory(root, ec)) return r;
  struct Run {
    std::string name;
    std::string path;
    uint64_t bytes;
    bool complete;
  };
  std::vector<Run> runs;
  const std::string keep_name = keep.empty() ? "" : fs::path(keep).filename().string();
  for (auto it = fs::directory_iterator(root, ec); !ec && it != fs::directory_iterator();
       it.increment(ec)) {
    std::error_code e2;
    if (!it->is_directory(e2) || it->is_symlink(e2)) continue;
    Run run{it->path().filename().string(), it->path().string(), dir_bytes(it->path().string()),
            fs::is_regular_file(it->path() / "summary.json", e2)};
    r.bytes_before += run.bytes;
    runs.push_back(run);
  }
  std::sort(runs.begin(), runs.end(), [](const Run& a, const Run& b) { return a.name < b.name; });
  uint64_t total = r.bytes_before;
  for (const Run& run : runs) {
    if (total <= max_bytes) break;
    if (!run.complete || run.name == keep_name) continue;
    std::error_code e3;
    fs::remove_all(run.path, e3);
    if (e3) {
      r.errors.push_back("could not remove " + run.name + ": " + e3.message());
      continue;
    }
    r.removed.push_back(run.name);
    r.bytes_removed += run.bytes;
    total -= std::min(total, run.bytes);
  }
  r.bytes_after = total;
  return r;
}

std::vector<std::string> mark_interrupted_runs(const std::string& root,
                                               const std::string& now_utc) {
  std::vector<std::string> marked;
  std::error_code ec;
  if (!fs::is_directory(root, ec)) return marked;
  std::vector<fs::path> dirs;
  for (auto it = fs::directory_iterator(root, ec); !ec && it != fs::directory_iterator();
       it.increment(ec)) {
    std::error_code e2;
    if (it->is_directory(e2) && !it->is_symlink(e2) && !fs::exists(it->path() / "summary.json", e2))
      dirs.push_back(it->path());
  }
  std::sort(dirs.begin(), dirs.end());
  for (const fs::path& d : dirs) {
    RunSummary s;
    s.final_state = "INTERRUPTED";
    s.duration_s = NAN;
    s.bag_healthy_throughout = false;
    s.provenance_complete = false;
    s.ulog_bytes = dir_bytes((d / "ulog").string());
    bool bag_finalised = false;
    for (auto it = fs::directory_iterator(d, ec); !ec && it != fs::directory_iterator();
         it.increment(ec)) {
      const std::string n = it->path().filename().string();
      if (n.rfind("rosbag2", 0) != 0) continue;
      s.bag_bytes += dir_bytes(it->path().string());
      std::error_code e2;
      bag_finalised = bag_finalised || fs::exists(it->path() / "metadata.yaml", e2);
    }
    s.notes.push_back("run directory had no summary.json when the recorder started at " + now_utc +
                      ": the recorder or the host stopped during the run");
    if (!bag_finalised)
      s.notes.push_back(
          "rosbag2 metadata.yaml missing: the bag was not finalised (ros2 bag reindex "
          "may recover it)");
    if (write_file_atomic((d / "summary.json").string(), summary_json(s)))
      marked.push_back(d.filename().string());
  }
  return marked;
}

}  // namespace dyx3_recorder
