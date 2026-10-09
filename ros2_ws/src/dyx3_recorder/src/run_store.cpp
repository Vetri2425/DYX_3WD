#include "dyx3_recorder/run_store.hpp"

#include <sys/statvfs.h>

#include <algorithm>
#include <filesystem>

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
  for (auto it = fs::recursive_directory_iterator(dir, fs::directory_options::skip_permission_denied,
                                                  ec);
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

}  // namespace dyx3_recorder
