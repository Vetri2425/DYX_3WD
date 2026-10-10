// mission_progress — see docs/contracts/dyx3_mission.md section 9a.
#include "dyx3_mission/mission_progress.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>

#include "dyx3_mission/mission_fsm.hpp"

namespace dyx3_mission {

namespace {

namespace fs = std::filesystem;

bool is_lower_hex64(const std::string& s) {
  if (s.size() != 64) return false;
  for (char c : s) {
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  }
  return true;
}

bool known_state(const std::string& s) {
  for (State st : {State::kIdle, State::kLoading, State::kReady, State::kRunning, State::kPaused,
                   State::kCompleted, State::kAborted, State::kError, State::kPlacing,
                   State::kArming, State::kEngaging}) {
    if (s == to_string(st)) return true;
  }
  return false;
}

// Strings in the record are ids, state names and a UTC stamp: printable ASCII without '"' or '\'
// (the writer never needs an escape, the reader accepts none).
bool plain_string(const std::string& s) {
  for (char c : s) {
    if (c < 0x20 || c > 0x7e || c == '"' || c == '\\') return false;
  }
  return true;
}

// A minimal strict reader for the one object this file holds.
class Reader {
public:
  explicit Reader(const std::string& s) : s_(s) {}
  void ws() {
    while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\n' || s_[i_] == '\t' || s_[i_] == '\r'))
      ++i_;
  }
  bool eat(char c) {
    ws();
    if (i_ < s_.size() && s_[i_] == c) {
      ++i_;
      return true;
    }
    return false;
  }
  bool peek(char c) {
    ws();
    return i_ < s_.size() && s_[i_] == c;
  }
  bool string(std::string* out) {
    if (!eat('"')) return false;
    out->clear();
    while (i_ < s_.size() && s_[i_] != '"') {
      const char c = s_[i_++];
      if (c < 0x20 || c > 0x7e || c == '\\') return false;
      out->push_back(c);
    }
    if (i_ >= s_.size()) return false;
    ++i_;  // closing quote
    return true;
  }
  bool uint32(std::uint32_t* out) {
    ws();
    const std::size_t start = i_;
    std::uint64_t v = 0;
    while (i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9') {
      v = v * 10 + static_cast<std::uint64_t>(s_[i_] - '0');
      if (v > std::numeric_limits<std::uint32_t>::max()) return false;
      ++i_;
    }
    const std::size_t len = i_ - start;
    if (len == 0 || (len > 1 && s_[start] == '0')) return false;  // no leading zeros
    *out = static_cast<std::uint32_t>(v);
    return true;
  }
  bool at_end() {
    ws();
    return i_ == s_.size();
  }

private:
  const std::string& s_;
  std::size_t i_ = 0;
};

bool fail(std::string* error, const std::string& why) {
  if (error) *error = why;
  return false;
}

// write(2) the whole buffer, retrying on EINTR and short writes.
bool write_all(int fd, const std::string& bytes) {
  std::size_t off = 0;
  while (off < bytes.size()) {
    const ssize_t n = ::write(fd, bytes.data() + off, bytes.size() - off);
    if (n < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    off += static_cast<std::size_t>(n);
  }
  return true;
}

}  // namespace

std::string serialize_progress(const MissionProgress& p) {
  std::ostringstream o;
  o << "{\"schema\":" << MissionProgress::kSchema << ",\"path_artifact_sha256\":\""
    << p.path_artifact_sha256 << "\",\"mission_id\":" << p.mission_id
    << ",\"run_index\":" << p.run_index << ",\"point_index\":" << p.point_index
    << ",\"completed_points\":[";
  for (std::size_t i = 0; i < p.completed_points.size(); ++i) {
    if (i) o << ',';
    o << p.completed_points[i];
  }
  o << "],\"state\":\"" << p.state << "\",\"updated_utc\":\"" << p.updated_utc << "\"}\n";
  return o.str();
}

bool parse_progress(const std::string& bytes, MissionProgress* out, std::string* error) {
  Reader r(bytes);
  MissionProgress p;
  std::uint32_t schema = 0;
  // one bit per key: schema, sha, mission_id, run_index, point_index, completed_points, state, utc
  unsigned seen = 0;
  const auto once = [&seen](unsigned bit) {
    if (seen & bit) return false;
    seen |= bit;
    return true;
  };
  if (!r.eat('{')) return fail(error, "not a JSON object");
  bool first = true;
  while (!r.peek('}')) {
    if (!first && !r.eat(',')) return fail(error, "expected ','");
    first = false;
    std::string key;
    if (!r.string(&key) || !r.eat(':')) return fail(error, "expected a key");
    bool ok = false;
    if (key == "schema") {
      ok = once(1U) && r.uint32(&schema);
    } else if (key == "path_artifact_sha256") {
      ok = once(2U) && r.string(&p.path_artifact_sha256);
    } else if (key == "mission_id") {
      ok = once(4U) && r.uint32(&p.mission_id);
    } else if (key == "run_index") {
      ok = once(8U) && r.uint32(&p.run_index);
    } else if (key == "point_index") {
      ok = once(16U) && r.uint32(&p.point_index);
    } else if (key == "completed_points") {
      ok = once(32U) && r.eat('[');
      if (ok && !r.peek(']')) {
        do {
          std::uint32_t v = 0;
          ok = r.uint32(&v);
          if (ok) p.completed_points.push_back(v);
        } while (ok && r.eat(','));
      }
      ok = ok && r.eat(']');
    } else if (key == "state") {
      ok = once(64U) && r.string(&p.state);
    } else if (key == "updated_utc") {
      ok = once(128U) && r.string(&p.updated_utc);
    } else {
      return fail(error, "unknown key '" + key + "'");
    }
    if (!ok) return fail(error, "bad or repeated value for '" + key + "'");
  }
  r.eat('}');
  if (!r.at_end()) return fail(error, "trailing bytes after the object");
  if (seen != 255U) return fail(error, "a key is missing");
  if (schema != static_cast<std::uint32_t>(MissionProgress::kSchema)) {
    return fail(error, "unsupported schema " + std::to_string(schema));
  }
  if (!is_lower_hex64(p.path_artifact_sha256)) return fail(error, "artifact id is not 64 hex");
  if (!known_state(p.state)) return fail(error, "unknown state '" + p.state + "'");
  for (std::size_t i = 0; i < p.completed_points.size(); ++i) {
    // The journal resolves points in path order: anything but 0..k-1 is not a record it wrote.
    if (p.completed_points[i] != i) return fail(error, "completed_points is not 0..k-1");
  }
  *out = std::move(p);
  return true;
}

std::string progress_dir(const std::string& missions_dir) {
  return (fs::path(missions_dir) / "progress").string();
}

std::string progress_file(const std::string& missions_dir, const std::string& sha256) {
  return (fs::path(progress_dir(missions_dir)) / (sha256 + ".json")).string();
}

bool write_progress(const std::string& missions_dir, const MissionProgress& p, std::string* error) {
  if (!is_lower_hex64(p.path_artifact_sha256)) return fail(error, "artifact id is not 64 hex");
  if (!plain_string(p.state) || !plain_string(p.updated_utc)) {
    return fail(error, "state / stamp not writable");
  }
  const fs::path dir = progress_dir(missions_dir);
  std::error_code ec;
  fs::create_directories(dir, ec);
  if (ec) return fail(error, "cannot create " + dir.string() + ": " + ec.message());
  const fs::path target = progress_file(missions_dir, p.path_artifact_sha256);
  const fs::path tmp = dir / (p.path_artifact_sha256 + ".json.tmp." + std::to_string(getpid()));
  const std::string bytes = serialize_progress(p);
  const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) return fail(error, "cannot open " + tmp.string() + ": " + std::strerror(errno));
  const bool written = write_all(fd, bytes) && ::fsync(fd) == 0;
  const int saved = errno;
  const bool closed = ::close(fd) == 0;
  if (!written || !closed) {
    std::error_code rm;
    fs::remove(tmp, rm);
    return fail(error, "cannot write " + tmp.string() + ": " + std::strerror(saved));
  }
  fs::rename(tmp, target, ec);
  if (ec) {
    std::error_code rm;
    fs::remove(tmp, rm);
    return fail(error, "cannot publish " + target.string() + ": " + ec.message());
  }
  // The rename itself is durable only once the directory entry is.
  const int dfd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (dfd < 0) return fail(error, "cannot open " + dir.string() + " to sync it");
  const bool dir_synced = ::fsync(dfd) == 0;
  ::close(dfd);
  if (!dir_synced) return fail(error, "cannot sync " + dir.string());
  return true;
}

std::map<std::string, MissionProgress> load_progress_dir(const std::string& missions_dir,
                                                         std::vector<std::string>* warnings) {
  std::map<std::string, MissionProgress> out;
  const fs::path dir = progress_dir(missions_dir);
  std::error_code ec;
  if (!fs::is_directory(dir, ec)) return out;
  for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
    const fs::path path = it->path();
    if (path.extension() != ".json") continue;  // a temporary file left by a crash is not a record
    const std::string id = path.stem().string();
    const auto warn = [&](const std::string& why) {
      if (warnings) warnings->push_back(path.string() + ": " + why);
    };
    std::error_code fe;
    if (!is_lower_hex64(id) || !it->is_regular_file(fe)) {
      warn("not a progress record name");
      continue;
    }
    const auto size = fs::file_size(path, fe);
    if (fe || size > kMaxProgressBytes) {
      warn("unreadable or larger than kMaxProgressBytes");
      continue;
    }
    std::ifstream in(path, std::ios::binary);
    std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (!in.good() && !in.eof()) {
      warn("read failed");
      continue;
    }
    MissionProgress p;
    std::string why;
    if (!parse_progress(bytes, &p, &why)) {
      warn(why);
      continue;
    }
    if (p.path_artifact_sha256 != id) {
      warn("holds the progress of another artifact");
      continue;
    }
    out[id] = std::move(p);
  }
  if (ec && warnings) warnings->push_back(dir.string() + ": " + ec.message());
  return out;
}

std::string utc_now_iso8601() {
  const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}

}  // namespace dyx3_mission
