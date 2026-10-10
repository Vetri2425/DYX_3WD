#include "dyx3_recorder/fcu_provenance.hpp"

#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "dyx3_recorder/run_manifest.hpp"

namespace dyx3_recorder {
namespace {

namespace fs = std::filesystem;

// A recursive-descent scanner that only finds where values start and end; it validates the JSON
// grammar but builds nothing. Positions are byte offsets into `s`.
struct Scanner {
  const std::string& s;
  size_t i{0};
  int depth{0};

  void ws() {
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) ++i;
  }
  bool string() {
    if (i >= s.size() || s[i] != '"') return false;
    for (++i; i < s.size(); ++i) {
      const unsigned char c = static_cast<unsigned char>(s[i]);
      if (c == '"') {
        ++i;
        return true;
      }
      if (c < 0x20) return false;
      if (c == '\\') {
        if (++i >= s.size()) return false;
        if (s[i] == 'u') {
          if (i + 4 >= s.size()) return false;
          for (int k = 1; k <= 4; ++k)
            if (!std::isxdigit(static_cast<unsigned char>(s[i + k]))) return false;
          i += 4;
        } else if (std::string("\"\\/bfnrt").find(s[i]) == std::string::npos) {
          return false;
        }
      }
    }
    return false;
  }
  bool literal(const char* w) {
    const std::string t(w);
    if (s.compare(i, t.size(), t) != 0) return false;
    i += t.size();
    return true;
  }
  bool number() {
    const size_t b = i;
    if (i < s.size() && s[i] == '-') ++i;
    while (i < s.size() && (std::isdigit(static_cast<unsigned char>(s[i])) || s[i] == '.' ||
                            s[i] == 'e' || s[i] == 'E' || s[i] == '+' || s[i] == '-'))
      ++i;
    return i > b && std::isdigit(static_cast<unsigned char>(s[i - 1]));
  }
  bool container(char open, char close, bool object) {
    if (i >= s.size() || s[i] != open || ++depth > 64) return false;
    ++i;
    ws();
    if (i < s.size() && s[i] == close) {
      ++i;
      --depth;
      return true;
    }
    for (;;) {
      if (object) {
        if (!string()) return false;
        ws();
        if (i >= s.size() || s[i] != ':') return false;
        ++i;
        ws();
      }
      if (!value()) return false;
      ws();
      if (i < s.size() && s[i] == ',') {
        ++i;
        ws();
        continue;
      }
      if (i < s.size() && s[i] == close) {
        ++i;
        --depth;
        return true;
      }
      return false;
    }
  }
  bool value() {
    if (i >= s.size()) return false;
    switch (s[i]) {
      case '"':
        return string();
      case '{':
        return container('{', '}', true);
      case '[':
        return container('[', ']', false);
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
};

struct Member {
  size_t value_begin{0}, value_end{0};
};

// Walks the top-level object: the span of `key`'s value, and the position of the closing brace.
bool scan_object(const std::string& json, const std::string& key, bool* found, Member* m,
                 size_t* close_pos, bool* empty) {
  Scanner sc{json};
  sc.ws();
  if (sc.i >= json.size() || json[sc.i] != '{') return false;
  ++sc.i;
  sc.ws();
  *found = false;
  *empty = true;
  if (sc.i < json.size() && json[sc.i] == '}') {
    *close_pos = sc.i++;
  } else {
    *empty = false;
    for (;;) {
      const size_t kb = sc.i;
      if (!sc.string()) return false;
      std::string k;
      if (!json_string(json.substr(kb, sc.i - kb), &k)) return false;
      sc.ws();
      if (sc.i >= json.size() || json[sc.i] != ':') return false;
      ++sc.i;
      sc.ws();
      const size_t vb = sc.i;
      if (!sc.value()) return false;
      if (k == key && !*found) {
        *found = true;
        m->value_begin = vb;
        m->value_end = sc.i;
      }
      sc.ws();
      if (sc.i < json.size() && json[sc.i] == ',') {
        ++sc.i;
        sc.ws();
        continue;
      }
      if (sc.i < json.size() && json[sc.i] == '}') {
        *close_pos = sc.i++;
        break;
      }
      return false;
    }
  }
  sc.ws();
  return sc.i == json.size();  // nothing but whitespace after the object
}

std::string read_text(const std::string& p, bool* ok) {
  std::ifstream f(p, std::ios::binary);
  *ok = static_cast<bool>(f);
  std::ostringstream o;
  o << f.rdbuf();
  return o.str();
}

std::string string_member(const std::string& json, const std::string& key) {
  std::string raw, v;
  if (json_member(json, key, &raw) && json_string(raw, &v)) return v;
  return "";
}

bool is_file(const std::string& p) {
  std::error_code ec;
  return !p.empty() && fs::is_regular_file(p, ec);
}

}  // namespace

bool json_member(const std::string& json, const std::string& key, std::string* raw) {
  bool found = false, empty = true;
  Member m;
  size_t close = 0;
  if (!scan_object(json, key, &found, &m, &close, &empty) || !found) return false;
  *raw = json.substr(m.value_begin, m.value_end - m.value_begin);
  return true;
}

bool json_string(const std::string& raw, std::string* out) {
  if (raw.size() < 2 || raw.front() != '"' || raw.back() != '"') return false;
  std::string o;
  for (size_t i = 1; i + 1 < raw.size(); ++i) {
    const char c = raw[i];
    if (c != '\\') {
      if (c == '"' || static_cast<unsigned char>(c) < 0x20) return false;
      o += c;
      continue;
    }
    if (++i + 1 > raw.size() - 1) return false;
    switch (raw[i]) {
      case '"':
      case '\\':
      case '/':
        o += raw[i];
        break;
      case 'b':
        o += '\b';
        break;
      case 'f':
        o += '\f';
        break;
      case 'n':
        o += '\n';
        break;
      case 'r':
        o += '\r';
        break;
      case 't':
        o += '\t';
        break;
      case 'u': {
        if (i + 4 >= raw.size()) return false;
        unsigned v = 0;
        for (int k = 1; k <= 4; ++k) {
          const char h = raw[i + k];
          if (!std::isxdigit(static_cast<unsigned char>(h))) return false;
          v = v * 16 +
              static_cast<unsigned>(std::isdigit(static_cast<unsigned char>(h))
                                        ? h - '0'
                                        : std::tolower(static_cast<unsigned char>(h)) - 'a' + 10);
        }
        if (v >= 0x80) return false;
        o += static_cast<char>(v);
        i += 4;
        break;
      }
      default:
        return false;
    }
  }
  *out = o;
  return true;
}

bool json_set_string(const std::string& json, const std::string& key, const std::string& value,
                     std::string* out) {
  bool found = false, empty = true;
  Member m;
  size_t close = 0;
  if (!scan_object(json, key, &found, &m, &close, &empty)) return false;
  const std::string v = "\"" + json_escape(value) + "\"";
  if (found) {
    *out = json.substr(0, m.value_begin) + v + json.substr(m.value_end);
    return true;
  }
  std::string head = json.substr(0, close);
  while (!head.empty() && std::isspace(static_cast<unsigned char>(head.back()))) head.pop_back();
  *out = head + (empty ? "" : ",") + "\n  \"" + json_escape(key) + "\": " + v + "\n" +
         json.substr(close);
  return true;
}

FcuDumpFiles read_fcu_dump(const std::string& params_path, const std::string& versions_path) {
  FcuDumpFiles f;
  bool ok = false;
  const std::string p = read_text(params_path, &ok);
  std::string raw;
  if (ok && string_member(p, "source") == "mavlink") {
    f.params_written = true;
    f.params_status = string_member(p, "status");
    f.params_reason = string_member(p, "reason");
    f.params_complete = json_member(p, "complete", &raw) && raw == "true";
  }
  const std::string v = read_text(versions_path, &ok);
  if (ok && string_member(v, "source") == "mavlink") {
    f.version_written = true;
    const std::string h = string_member(v, "firmware_git_hash");
    const bool hex = !h.empty() && std::all_of(h.begin(), h.end(), [](char c) {
      return std::isxdigit(static_cast<unsigned char>(c)) != 0;
    });
    if (string_member(v, "status") == "ok" && hex) {
      f.firmware_git_hash = h;
    } else {
      f.version_reason = string_member(v, "reason");
      if (f.version_reason.empty()) f.version_reason = "no firmware_git_hash in versions_fcu.json";
    }
  }
  return f;
}

std::string find_param_dump_script(const std::string& exe_path, const char* release_dir_env,
                                   std::string* searched) {
  static const std::string kRel = "tools/px4/param_dump.py";
  std::vector<std::string> tried;
  if (release_dir_env != nullptr && *release_dir_env != '\0') {
    const std::string c = (fs::path(release_dir_env) / kRel).string();
    tried.push_back(c);
    if (is_file(c)) return c;
  }
  if (!exe_path.empty()) {
    for (fs::path d = fs::path(exe_path).parent_path(); !d.empty(); d = d.parent_path()) {
      const std::string c = (d / kRel).string();
      if (is_file(c)) return c;
      if (d == d.root_path()) break;
    }
    tried.push_back("the ancestors of " + exe_path);
  }
  const std::string fallback = "/opt/dyx3/current/" + kRel;
  tried.push_back(fallback);
  if (is_file(fallback)) return fallback;
  if (searched != nullptr) {
    searched->clear();
    for (size_t i = 0; i < tried.size(); ++i) *searched += (i ? ", " : "") + tried[i];
  }
  return "";
}

std::string resolve_dump_python(const std::string& preferred) {
  if (is_file(preferred) && access(preferred.c_str(), X_OK) == 0) return preferred;
  return "python3";
}

std::vector<std::string> expand_dump_argv(const std::vector<std::string>& tmpl,
                                          const std::string& python, const std::string& script,
                                          const std::string& dir) {
  const std::pair<std::string, const std::string*> subs[] = {
      {"{python}", &python}, {"{script}", &script}, {"{dir}", &dir}};
  std::vector<std::string> out;
  for (auto a : tmpl) {
    for (const auto& [ph, val] : subs) {
      // continue after the inserted text, so a value containing a placeholder is never expanded
      // again
      for (size_t p = 0; (p = a.find(ph, p)) != std::string::npos; p += val->size())
        a.replace(p, ph.size(), *val);
    }
    out.push_back(a);
  }
  return out;
}

bool argv_uses(const std::vector<std::string>& tmpl, const std::string& placeholder) {
  return std::any_of(tmpl.begin(), tmpl.end(), [&](const std::string& a) {
    return a.find(placeholder) != std::string::npos;
  });
}

bool firmware_mismatch(const std::string& running_hash, const std::string& expected_sha) {
  if (running_hash.empty() || expected_sha.empty()) return false;
  const size_t n = std::min(running_hash.size(), expected_sha.size());
  for (size_t i = 0; i < n; ++i) {
    if (std::tolower(static_cast<unsigned char>(running_hash[i])) !=
        std::tolower(static_cast<unsigned char>(expected_sha[i])))
      return true;
  }
  return false;
}

}  // namespace dyx3_recorder
