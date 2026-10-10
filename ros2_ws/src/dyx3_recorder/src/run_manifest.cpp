#include "dyx3_recorder/run_manifest.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>

namespace dyx3_recorder {

namespace fs = std::filesystem;

std::string json_escape(const std::string& s) {
  std::string o;
  o.reserve(s.size() + 2);
  for (const unsigned char c : s) {
    switch (c) {
      case '"':
        o += "\\\"";
        break;
      case '\\':
        o += "\\\\";
        break;
      case '\n':
        o += "\\n";
        break;
      case '\r':
        o += "\\r";
        break;
      case '\t':
        o += "\\t";
        break;
      default:
        if (c < 0x20) {
          char b[8];
          std::snprintf(b, sizeof b, "\\u%04x", c);
          o += b;
        } else {
          o += static_cast<char>(c);
        }
    }
  }
  return o;
}

JsonObject& JsonObject::str(const std::string& k, const std::string& v) {
  items_.emplace_back(k, "\"" + json_escape(v) + "\"");
  return *this;
}
JsonObject& JsonObject::num(const std::string& k, double v) {
  if (!std::isfinite(v)) {
    items_.emplace_back(k, "null");
  } else {
    char b[40];
    std::snprintf(b, sizeof b, "%.17g", v);
    items_.emplace_back(k, b);
  }
  return *this;
}
JsonObject& JsonObject::integer(const std::string& k, int64_t v) {
  items_.emplace_back(k, std::to_string(v));
  return *this;
}
JsonObject& JsonObject::boolean(const std::string& k, bool v) {
  items_.emplace_back(k, v ? "true" : "false");
  return *this;
}
JsonObject& JsonObject::raw(const std::string& k, const std::string& json) {
  items_.emplace_back(k, json);
  return *this;
}
JsonObject& JsonObject::str_list(const std::string& k, const std::vector<std::string>& v) {
  std::string a = "[";
  for (size_t i = 0; i < v.size(); ++i)
    a += (i ? ", " : "") + std::string("\"") + json_escape(v[i]) + "\"";
  items_.emplace_back(k, a + "]");
  return *this;
}

std::string JsonObject::dump(int indent, int level) const {
  if (items_.empty()) return "{}";
  const std::string pad(static_cast<size_t>(indent * (level + 1)), ' ');
  const std::string end(static_cast<size_t>(indent * level), ' ');
  std::string o = "{\n";
  for (size_t i = 0; i < items_.size(); ++i) {
    o += pad + "\"" + json_escape(items_[i].first) + "\": " + items_[i].second;
    o += (i + 1 < items_.size()) ? ",\n" : "\n";
  }
  return o + end + "}";
}

std::string manifest_json(const RunInfo& r) {
  JsonObject o;
  o.integer("schema", 1)
      .str("run_id", r.run_id)
      .integer("mission_id", r.mission_id)
      .integer("run_index", r.run_index)
      .str("path_artifact_sha256", r.path_artifact_sha256)
      .str("conditioned_execution_sha256", r.conditioned_execution_sha256)
      .str("start_utc", r.start_utc)
      .str("start_state", r.start_state)
      .str("vehicle_id", r.vehicle_id)
      .str("operator", r.operator_name)
      .str("hostname", r.hostname)
      .boolean("timesync_valid", r.timesync_valid)
      .integer("timesync_offset_us", r.timesync_offset_us)
      .integer("timesync_round_trip_us", r.timesync_round_trip_us);
  return o.dump() + "\n";
}

std::string summary_json(const RunSummary& s) {
  JsonObject o;
  o.integer("schema", 1)
      .str("end_utc", s.end_utc)
      .str("final_state", s.final_state)
      .num("duration_s", s.duration_s)
      .str("running_utc", s.running_utc)
      .num("preroll_s", s.preroll_s)
      .integer("bag_bytes", static_cast<int64_t>(s.bag_bytes))
      .integer("ulog_bytes", static_cast<int64_t>(s.ulog_bytes))
      .integer("ulog_gaps", static_cast<int64_t>(s.ulog_gaps))
      .str("ulog_header", s.ulog_header)
      .str("conditioned_execution_sha256", s.conditioned_execution_sha256)
      .boolean("bag_healthy_throughout", s.bag_healthy_throughout)
      .boolean("provenance_complete", s.provenance_complete)
      .boolean("timesync_valid_end", s.timesync_valid_end)
      .integer("timesync_offset_us_end", s.timesync_offset_us_end)
      .integer("timesync_round_trip_us_end", s.timesync_round_trip_us_end)
      .str_list("notes", s.notes);
  return o.dump() + "\n";
}

std::string iso_utc(time_t t) {
  struct tm tm{};
  gmtime_r(&t, &tm);
  char b[32];
  std::strftime(b, sizeof b, "%Y-%m-%dT%H:%M:%SZ", &tm);
  return b;
}

std::string dir_stamp_utc(time_t t) {
  struct tm tm{};
  gmtime_r(&t, &tm);
  char b[32];
  std::strftime(b, sizeof b, "%Y-%m-%d_%H%M%S", &tm);
  return b;
}

std::string run_dir_name(time_t t, uint32_t mission_id, uint32_t run_index) {
  char b[48];
  std::snprintf(b, sizeof b, "_mission_%04u", mission_id);
  std::string n = dir_stamp_utc(t) + b;
  if (run_index != 0) n += "_run" + std::to_string(run_index);
  return n;
}

std::string unique_run_path(const std::string& root, const std::string& name) {
  fs::path p = fs::path(root) / name;
  if (!fs::exists(p)) return p.string();
  for (int i = 2; i < 10000; ++i) {
    fs::path q = fs::path(root) / (name + "_" + std::to_string(i));
    if (!fs::exists(q)) return q.string();
  }
  return (fs::path(root) / (name + "_x")).string();
}

bool write_file_atomic(const std::string& path, const std::string& content) {
  // REC-010: data reaches the disk before the name does (write, fsync, close, rename, fsync of the
  // directory). Every step is checked; on failure the temp file is removed and false returned.
  const std::string tmp = path + ".tmp";
  const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) return false;
  bool ok = true;
  const char* p = content.data();
  size_t left = content.size();
  while (ok && left > 0) {
    const ssize_t n = ::write(fd, p, left);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) {
      ok = false;
      break;
    }
    p += n;
    left -= static_cast<size_t>(n);
  }
  ok = ok && ::fsync(fd) == 0;
  ok = (::close(fd) == 0) && ok;
  if (ok) ok = ::rename(tmp.c_str(), path.c_str()) == 0;
  if (!ok) {
    ::unlink(tmp.c_str());
    return false;
  }
  const std::string parent = fs::path(path).parent_path().string();
  const int dfd = ::open(parent.empty() ? "." : parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (dfd < 0) return false;
  ok = ::fsync(dfd) == 0;
  ok = (::close(dfd) == 0) && ok;
  return ok;
}

bool is_secret_name(const std::string& filename) {
  std::string n = filename;
  std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c) { return std::tolower(c); });
  const auto ends = [&](const char* suf) {
    const std::string s(suf);
    return n.size() >= s.size() && n.compare(n.size() - s.size(), s.size(), s) == 0;
  };
  if (ends(".env") || ends(".key") || ends(".pem") || ends(".token") || ends(".p12") ||
      ends(".pfx") || ends(".jks"))
    return true;
  for (const char* w : {"secret", "token", "password", "passwd", "credential", "psk"}) {
    if (n.find(w) != std::string::npos) return true;
  }
  // ntrip*: caster credentials; auth*: the backend's token store (auth.json) and authorized_keys;
  // id_*: ssh private keys
  for (const char* prefix : {"ntrip", "auth", "id_rsa", "id_ecdsa", "id_ed25519", "id_dsa"}) {
    if (n.rfind(prefix, 0) == 0) return true;
  }
  return false;
}

CopyResult copy_config_tree(const std::string& src, const std::string& dst) {
  CopyResult r;
  std::error_code ec;
  if (!fs::is_directory(src, ec)) {
    r.ok = false;
    r.errors.push_back("config dir missing: " + src);
    return r;
  }
  fs::create_directories(dst, ec);
  for (auto it =
           fs::recursive_directory_iterator(src, fs::directory_options::skip_permission_denied, ec);
       it != fs::recursive_directory_iterator(); it.increment(ec)) {
    if (ec) break;
    const fs::path rel = fs::relative(it->path(), src, ec);
    if (it->is_directory(ec)) {
      if (is_secret_name(it->path().filename().string())) {
        it.disable_recursion_pending();
        ++r.excluded;
        continue;
      }
      fs::create_directories(fs::path(dst) / rel, ec);
      continue;
    }
    if (!it->is_regular_file(ec)) continue;  // symlinks and specials are not followed
    if (is_secret_name(it->path().filename().string())) {
      ++r.excluded;
      continue;
    }
    fs::copy_file(it->path(), fs::path(dst) / rel, fs::copy_options::overwrite_existing, ec);
    if (ec) {
      r.ok = false;
      r.errors.push_back("copy failed: " + rel.string());
      ec.clear();
    } else {
      ++r.copied;
    }
  }
  return r;
}

}  // namespace dyx3_recorder
