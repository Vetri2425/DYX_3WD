// path_artifact — see docs/contracts/path_artifact.md
#include "dyx3_mission/path_artifact.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <sstream>

#include "dyx3_mission/sha256.hpp"

namespace dyx3_mission {

namespace {

ArtifactResult fail(const std::string& msg) {
  ArtifactResult r;
  r.ok = false;
  r.error = msg;
  return r;
}

bool is_lower_hex64(const std::string& s) {
  if (s.size() != 64) return false;
  for (char c : s) {
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  }
  return true;
}

std::vector<std::string> split(const std::string& s, char sep) {
  std::vector<std::string> out;
  std::size_t start = 0;
  while (true) {
    const std::size_t pos = s.find(sep, start);
    if (pos == std::string::npos) {
      out.push_back(s.substr(start));
      return out;
    }
    out.push_back(s.substr(start, pos - start));
    start = pos + 1;
  }
}

bool parse_double(const std::string& tok, double& out) {
  if (tok.empty()) return false;
  errno = 0;
  char* end = nullptr;
  out = std::strtod(tok.c_str(), &end);
  return end == tok.c_str() + tok.size() && errno != ERANGE && std::isfinite(out);
}

bool parse_uint(const std::string& tok, unsigned long& out) {
  if (tok.empty() || tok.size() > 10) return false;
  for (char c : tok) {
    if (c < '0' || c > '9') return false;
  }
  out = std::strtoul(tok.c_str(), nullptr, 10);
  return true;
}

}  // namespace

ArtifactResult parse_artifact(const std::string& bytes, const std::string& expected_sha256) {
  const std::string digest = sha256_hex(bytes);
  if (!expected_sha256.empty() && digest != expected_sha256) {
    return fail("sha256 mismatch: expected " + expected_sha256 + ", bytes hash to " + digest);
  }
  for (unsigned char c : bytes) {
    if (c > 0x7E || c == '\r' || (c < 0x20 && c != '\n')) {
      return fail("artifact contains a non-ASCII, control or CR byte");
    }
  }
  if (bytes.empty() || bytes.back() != '\n') {
    return fail("artifact must end with a newline");
  }
  const std::vector<std::string> lines = split(bytes.substr(0, bytes.size() - 1), '\n');
  if (lines.size() < 7) return fail("artifact too short");

  if (lines[0] != "DYX3PATH 1") {
    return fail(lines[0].rfind("DYX3PATH ", 0) == 0 ? "unsupported format version" : "bad magic");
  }
  if (lines[1] != "frame local_ned") return fail("unsupported frame line");
  const auto eng = split(lines[2], ' ');
  if (eng.size() != 2 || eng[0] != "engine" || eng[1].empty()) return fail("bad engine line");
  if (lines[3].rfind("meta ", 0) != 0) return fail("bad meta line");
  const auto pl = split(lines[4], ' ');
  unsigned long n = 0;
  if (pl.size() != 2 || pl[0] != "points" || !parse_uint(pl[1], n) || n < 1) {
    return fail("bad points line");
  }
  if (lines.size() != 5 + n + 1) return fail("declared point count does not match the file");
  if (lines.back() != "end " + std::to_string(n)) return fail("missing or wrong end marker");

  ArtifactResult r;
  r.artifact.sha256 = digest;
  r.artifact.version = 1;
  r.artifact.engine_id = eng[1];
  r.artifact.meta_json = lines[3].substr(5);
  r.artifact.points.reserve(n);
  for (unsigned long i = 0; i < n; ++i) {
    const auto f = split(lines[5 + i], ' ');
    if (f.size() != 3) return fail("point " + std::to_string(i) + ": expected 3 fields");
    ArtifactPoint p;
    unsigned long flags = 0;
    if (!parse_double(f[0], p.north_m) || !parse_double(f[1], p.east_m) ||
        !parse_uint(f[2], flags)) {
      return fail("point " + std::to_string(i) + ": unparseable number");
    }
    if (flags > 3) return fail("point " + std::to_string(i) + ": flags out of range");
    p.flags = static_cast<std::uint8_t>(flags);
    r.artifact.points.push_back(p);
  }
  r.ok = true;
  return r;
}

ArtifactResult load_artifact(const std::string& dir, const std::string& sha256) {
  if (!is_lower_hex64(sha256)) return fail("sha256 must be 64 lowercase hex characters");
  std::ifstream in(dir + "/" + sha256 + ".dyx3path", std::ios::binary);
  if (!in) return fail("cannot open artifact " + sha256 + " in " + dir);
  const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  return parse_artifact(bytes, sha256);
}

}  // namespace dyx3_mission
