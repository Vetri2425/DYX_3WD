// path_artifact — see docs/contracts/path_artifact.md
#include "dyx3_mission/path_artifact.hpp"

#include <charconv>
#include <cmath>
#include <cstdlib>
#include <filesystem>
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
  char* end = nullptr;
  out = std::strtod(tok.c_str(), &end);
  // ERANGE is not checked: overflow is infinite (refused here), and an underflow to zero spells
  // differently from "0.0" and is refused by the canonical-spelling check. A denormal is valid.
  return end == tok.c_str() + tok.size() && std::isfinite(out);
}

bool parse_uint(const std::string& tok, unsigned long& out) {
  if (tok.empty() || tok.size() > 10) return false;
  for (char c : tok) {
    if (c < '0' || c > '9') return false;
  }
  out = std::strtoul(tok.c_str(), nullptr, 10);
  return true;
}

// Python repr(float) of a finite double: the shortest digits that round-trip, positional notation
// for 1e-4 <= |v| < 1e16, otherwise d[.ddd]e+XX with at least two exponent digits. This is the
// spelling the Python writer emits and its decoder requires (`repr(x) == token`).
std::string python_repr_impl(double v) {
  char buf[64];
  const auto r = std::to_chars(buf, buf + sizeof(buf), v, std::chars_format::scientific);
  if (r.ec != std::errc()) return {};
  const std::string sci(buf, r.ptr);  // [-]d[.ddd]e[+-]XX, shortest round-trip digits
  std::size_t i = 0;
  const bool neg = sci[i] == '-';
  if (neg) ++i;
  const std::size_t epos = sci.find('e');
  if (epos == std::string::npos) return {};
  std::string digits;
  for (std::size_t k = i; k < epos; ++k) {
    if (sci[k] != '.') digits += sci[k];
  }
  const int exp10 = std::atoi(sci.c_str() + epos + 1);
  std::string out = neg ? "-" : "";
  if (digits == "0") return out + "0.0";
  const int decpt = exp10 + 1;  // value = 0.DIGITS x 10^decpt
  const int nd = static_cast<int>(digits.size());
  if (decpt > -4 && decpt <= 16) {
    if (decpt <= 0) {
      out += "0." + std::string(static_cast<std::size_t>(-decpt), '0') + digits;
    } else if (decpt >= nd) {
      out += digits + std::string(static_cast<std::size_t>(decpt - nd), '0') + ".0";
    } else {
      out += digits.substr(0, static_cast<std::size_t>(decpt)) + "." +
             digits.substr(static_cast<std::size_t>(decpt));
    }
    return out;
  }
  out += digits.substr(0, 1);
  if (nd > 1) out += "." + digits.substr(1);
  const int e = decpt - 1;
  const int ae = e < 0 ? -e : e;
  out += e < 0 ? "e-" : "e+";
  if (ae < 10) out += '0';
  return out + std::to_string(ae);
}

// A coordinate token is accepted only in the spelling Python's repr() produces ("1", "1e0",
// "1.50", "+1.0", hex floats and the like parse with strtod but are refused by the decoder).
bool parse_canonical_double(const std::string& tok, double& out) {
  return parse_double(tok, out) && python_repr_impl(out) == tok;
}

// Strict check of the `meta` line against what Python's decode() enforces: valid JSON, an object,
// and byte-identical to json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
// allow_nan=False). The metadata itself stays opaque to C++; nothing is extracted from it.
class CanonicalJson {
public:
  explicit CanonicalJson(const std::string& s) : s_(s) {}
  bool object() { return i_ < s_.size() && s_[i_] == '{' && value(0) && i_ == s_.size(); }

private:
  static constexpr int kMaxDepth = 64;  // stricter than Python on purpose: bounded recursion

  bool value(int depth) {
    if (i_ >= s_.size() || depth > kMaxDepth) return false;
    switch (s_[i_]) {
      case '{':
        return members(depth, '}', true);
      case '[':
        return members(depth, ']', false);
      case '"': {
        std::u32string ignored;
        return string(ignored);
      }
      case 't':
        return literal("true");
      case 'f':
        return literal("false");
      case 'n':
        return literal("null");
      default:
        return number();
    }
  }
  bool literal(const char* word) {
    const std::string w(word);
    if (s_.compare(i_, w.size(), w) != 0) return false;
    i_ += w.size();
    return true;
  }
  bool members(int depth, char close, bool is_object) {
    ++i_;  // opening bracket
    if (i_ < s_.size() && s_[i_] == close) {
      ++i_;
      return true;
    }
    std::u32string prev;
    bool first = true;
    while (true) {
      if (is_object) {
        std::u32string key;
        if (i_ >= s_.size() || s_[i_] != '"' || !string(key)) return false;
        if (!first && !(prev < key)) return false;  // sort_keys, and no duplicate keys
        prev = std::move(key);
        first = false;
        if (i_ >= s_.size() || s_[i_] != ':') return false;
        ++i_;
      }
      if (!value(depth + 1)) return false;
      if (i_ >= s_.size()) return false;
      if (s_[i_] == close) {
        ++i_;
        return true;
      }
      if (s_[i_] != ',') return false;
      ++i_;
    }
  }
  static int hex4(const std::string& s, std::size_t at) {
    if (at + 4 > s.size()) return -1;
    int v = 0;
    for (std::size_t k = at; k < at + 4; ++k) {
      const char c = s[k];
      v <<= 4;
      if (c >= '0' && c <= '9')
        v |= c - '0';
      else if (c >= 'a' && c <= 'f')
        v |= c - 'a' + 10;
      else
        return -1;  // upper-case hex is not what json.dumps writes
    }
    return v;
  }
  // json.dumps(ensure_ascii=True) writes printable ASCII except " and \ literally, \" \\ \n \r
  // \t \b \f as short escapes and everything else (controls, DEL, non-ASCII UTF-16 units) as
  // lower-case \uXXXX. Anything else spells the same string differently and is refused.
  bool string(std::u32string& out) {
    ++i_;  // opening quote
    while (i_ < s_.size()) {
      const unsigned char c = static_cast<unsigned char>(s_[i_]);
      if (c == '"') {
        ++i_;
        return true;
      }
      if (c < 0x20 || c > 0x7e) return false;
      if (c != '\\') {
        out += static_cast<char32_t>(c);
        ++i_;
        continue;
      }
      if (++i_ >= s_.size()) return false;
      const char e = s_[i_++];
      switch (e) {
        case '"':
          out += U'"';
          break;
        case '\\':
          out += U'\\';
          break;
        case 'n':
          out += U'\n';
          break;
        case 'r':
          out += U'\r';
          break;
        case 't':
          out += U'\t';
          break;
        case 'b':
          out += U'\b';
          break;
        case 'f':
          out += U'\f';
          break;
        case 'u': {
          int u = hex4(s_, i_);
          if (u < 0) return false;
          i_ += 4;
          if (u >= 0x20 && u <= 0x7e) return false;  // would have been written literally
          if (u == 0x08 || u == 0x09 || u == 0x0a || u == 0x0c || u == 0x0d) return false;
          char32_t cp = static_cast<char32_t>(u);
          if (u >= 0xd800 && u <= 0xdbff && i_ + 1 < s_.size() && s_[i_] == '\\' &&
              s_[i_ + 1] == 'u') {
            const int lo = hex4(s_, i_ + 2);
            if (lo >= 0xdc00 && lo <= 0xdfff) {  // a surrogate pair is one code point
              cp = 0x10000 + ((static_cast<char32_t>(u) - 0xd800) << 10) +
                   (static_cast<char32_t>(lo) - 0xdc00);
              i_ += 6;
            }
          }
          out += cp;
          break;
        }
        default:
          return false;  // includes the non-canonical "\/"
      }
    }
    return false;
  }
  bool number() {
    const std::size_t start = i_;
    if (i_ < s_.size() && s_[i_] == '-') ++i_;
    if (i_ >= s_.size() || s_[i_] < '0' || s_[i_] > '9') return false;
    if (s_[i_] == '0') {
      ++i_;
    } else {
      while (i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9') ++i_;
    }
    bool is_float = false;
    if (i_ < s_.size() && s_[i_] == '.') {
      is_float = true;
      ++i_;
      if (i_ >= s_.size() || s_[i_] < '0' || s_[i_] > '9') return false;
      while (i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9') ++i_;
    }
    if (i_ < s_.size() && (s_[i_] == 'e' || s_[i_] == 'E')) {
      is_float = true;
      ++i_;
      if (i_ < s_.size() && (s_[i_] == '+' || s_[i_] == '-')) ++i_;
      if (i_ >= s_.size() || s_[i_] < '0' || s_[i_] > '9') return false;
      while (i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9') ++i_;
    }
    const std::string tok = s_.substr(start, i_ - start);
    if (!is_float) return tok != "-0";  // json.loads gives int 0, which dumps as "0"
    double d = 0.0;
    return parse_canonical_double(tok, d);
  }

  const std::string& s_;
  std::size_t i_ = 0;
};

}  // namespace

std::string python_repr(double v) { return python_repr_impl(v); }

std::string serialize_artifact(const std::string& engine_id, const std::string& meta_json,
                               const std::vector<ArtifactPoint>& points) {
  if (points.empty() || engine_id.empty()) return {};
  for (unsigned char c : engine_id) {
    if (c <= 0x20 || c > 0x7e) return {};
  }
  for (unsigned char c : meta_json) {
    if (c < 0x20 || c > 0x7e) return {};
  }
  if (!CanonicalJson(meta_json).object()) return {};
  std::string out = "DYX3PATH 1\nframe local_ned\nengine " + engine_id + "\nmeta " + meta_json +
                    "\npoints " + std::to_string(points.size()) + "\n";
  for (const auto& p : points) {
    if (!std::isfinite(p.north_m) || !std::isfinite(p.east_m) || p.flags > 3) return {};
    out += python_repr_impl(p.north_m) + " " + python_repr_impl(p.east_m) + " " +
           std::to_string(static_cast<unsigned>(p.flags)) + "\n";
  }
  return out + "end " + std::to_string(points.size()) + "\n";
}

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
  if (!CanonicalJson(lines[3].substr(5)).object()) {
    return fail("meta is not a canonical JSON object");
  }
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
    if (!parse_canonical_double(f[0], p.north_m) || !parse_canonical_double(f[1], p.east_m) ||
        !parse_uint(f[2], flags)) {
      return fail("point " + std::to_string(i) + ": unparseable or non-canonical number");
    }
    if (flags > 3) return fail("point " + std::to_string(i) + ": flags out of range");
    p.flags = static_cast<std::uint8_t>(flags);
    r.artifact.points.push_back(p);
  }
  r.ok = true;
  return r;
}

ArtifactResult load_artifact(const std::string& dir, const std::string& sha256,
                             std::uintmax_t max_bytes) {
  if (!is_lower_hex64(sha256)) return fail("sha256 must be 64 lowercase hex characters");
  const std::string path = dir + "/" + sha256 + ".dyx3path";
  // Size first, before any read (MS-004): a huge or special file is refused unread.
  std::error_code ec;
  if (!std::filesystem::is_regular_file(path, ec) || ec) {
    return fail("artifact " + sha256 + " is not a readable regular file in " + dir);
  }
  const std::uintmax_t size = std::filesystem::file_size(path, ec);
  if (ec) return fail("cannot stat artifact " + sha256 + " in " + dir);
  if (size > max_bytes) {
    return fail("artifact " + sha256 + " is too large: " + std::to_string(size) + " bytes, limit " +
                std::to_string(max_bytes));
  }
  std::ifstream in(path, std::ios::binary);
  if (!in) return fail("cannot open artifact " + sha256 + " in " + dir);
  // Read at most the size we checked: a file that grows meanwhile cannot get past the limit (the
  // truncated bytes then fail the hash check).
  std::string bytes(static_cast<std::size_t>(size), '\0');
  in.read(bytes.data(), static_cast<std::streamsize>(size));
  if (static_cast<std::uintmax_t>(in.gcount()) != size) {
    return fail("short read of artifact " + sha256 + " in " + dir);
  }
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
