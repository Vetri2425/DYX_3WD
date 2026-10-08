// path_artifact — see docs/contracts/path_artifact.md
#include "dyx3_mission/path_artifact.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <locale>
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

std::string serialize_conditioned_artifact(const std::string& source_sha256,
                                           const std::string& conditioner_config,
                                           const std::vector<ConditionedRunArtifact>& runs) {
  if (!is_lower_hex64(source_sha256) ||
      conditioner_config.find_first_of("\r\n") != std::string::npos)
    return {};
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "DYX3COND 1\nsource " << source_sha256 << "\nconfig " << conditioner_config << "\nruns "
      << runs.size() << '\n'
      << std::setprecision(std::numeric_limits<double>::max_digits10);
  for (const auto& run : runs) {
    if (run.points.empty() || run.points.size() != run.flags.size() ||
        run.points.size() != run.must_hit.size() || run.profile > 1)
      return {};
    out << "run " << static_cast<unsigned>(run.profile) << ' ' << run.points.size() << '\n';
    for (std::size_t i = 0; i < run.points.size(); ++i) {
      if (!std::isfinite(run.points[i].north_m) || !std::isfinite(run.points[i].east_m) ||
          run.flags[i] > 1 || run.must_hit[i] > 1)
        return {};
      out << run.points[i].north_m << ' ' << run.points[i].east_m << ' '
          << static_cast<unsigned>(run.flags[i]) << ' ' << static_cast<unsigned>(run.must_hit[i])
          << '\n';
    }
    out << "endrun\n";
  }
  out << "end " << runs.size() << '\n';
  return out.str();
}

ConditionedResult parse_conditioned_artifact(const std::string& bytes,
                                             const std::string& expected_sha256) {
  ConditionedResult r;
  const auto fail_cond = [&r](const std::string& msg) {
    r.error = msg;
    return r;
  };
  const std::string digest = sha256_hex(bytes);
  if (!expected_sha256.empty() && digest != expected_sha256)
    return fail_cond("conditioned sha256 mismatch");
  if (bytes.empty() || bytes.back() != '\n' || bytes.find('\r') != std::string::npos)
    return fail_cond("malformed conditioned artifact newline");
  for (unsigned char c : bytes) {
    if (c > 0x7e || (c < 0x20 && c != '\n'))
      return fail_cond("conditioned artifact contains non-ASCII or control bytes");
  }
  std::istringstream in(bytes);
  std::string line;
  if (!std::getline(in, line) || line != "DYX3COND 1")
    return fail_cond("bad conditioned magic/version");
  if (!std::getline(in, line) || line.rfind("source ", 0) != 0)
    return fail_cond("missing source sha");
  r.artifact.source_sha256 = line.substr(7);
  if (!is_lower_hex64(r.artifact.source_sha256)) return fail_cond("invalid source sha");
  if (!std::getline(in, line) || line.rfind("config ", 0) != 0 || line.size() <= 7)
    return fail_cond("missing conditioner config");
  r.artifact.conditioner_config = line.substr(7);
  if (!std::getline(in, line) || line.rfind("runs ", 0) != 0)
    return fail_cond("missing runs count");
  unsigned long count = 0;
  if (!parse_uint(line.substr(5), count) || count == 0 || count > 100000)
    return fail_cond("invalid runs count");
  for (unsigned long ri = 0; ri < count; ++ri) {
    if (!std::getline(in, line)) return fail_cond("truncated run");
    const auto head = split(line, ' ');
    unsigned long profile = 0, n = 0;
    if (head.size() != 3 || head[0] != "run" || !parse_uint(head[1], profile) || profile > 1 ||
        !parse_uint(head[2], n) || n == 0 || n > 10000000)
      return fail_cond("malformed run header");
    ConditionedRunArtifact run;
    run.profile = static_cast<std::uint8_t>(profile);
    for (unsigned long pi = 0; pi < n; ++pi) {
      if (!std::getline(in, line)) return fail_cond("truncated run points");
      const auto fields = split(line, ' ');
      double north = 0.0, east = 0.0;
      unsigned long flag = 0, must = 0;
      if (fields.size() != 4 || !parse_double(fields[0], north) || !parse_double(fields[1], east) ||
          !parse_uint(fields[2], flag) || flag > 1 || !parse_uint(fields[3], must) || must > 1)
        return fail_cond("malformed conditioned point");
      run.points.push_back({north, east});
      run.flags.push_back(static_cast<std::uint8_t>(flag));
      run.must_hit.push_back(static_cast<std::uint8_t>(must));
    }
    if (!std::getline(in, line) || line != "endrun") return fail_cond("missing endrun");
    r.artifact.runs.push_back(std::move(run));
  }
  if (!std::getline(in, line) || line != "end " + std::to_string(count) || std::getline(in, line))
    return fail_cond("bad conditioned end marker or trailing data");
  if (serialize_conditioned_artifact(r.artifact.source_sha256, r.artifact.conditioner_config,
                                     r.artifact.runs) != bytes)
    return fail_cond("conditioned artifact is not canonical");
  r.artifact.sha256 = digest;
  r.ok = true;
  return r;
}

ConditionedResult load_conditioned_artifact(const std::string& dir, const std::string& sha256) {
  if (!is_lower_hex64(sha256)) return {false, "invalid conditioned sha", {}};
  std::ifstream in(dir + "/" + sha256 + ".dyx3cond", std::ios::binary);
  if (!in) return {false, "cannot open conditioned artifact", {}};
  const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  return parse_conditioned_artifact(bytes, sha256);
}

}  // namespace dyx3_mission
