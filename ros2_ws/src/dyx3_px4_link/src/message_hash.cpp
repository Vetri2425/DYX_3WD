#include "dyx3_px4_link/message_hash.hpp"

#include <algorithm>
#include <set>

namespace dyx3_px4_link {
namespace {

bool is_builtin(const std::string& t) {
  static const std::set<std::string> kBuiltins = {
      "int8",   "int16",   "int32",   "int64", "uint8", "uint16", "uint32",
      "uint64", "float32", "float64", "bool",  "char",  "byte",   "string"};
  return kBuiltins.count(t) != 0;
}

std::string strip_array(const std::string& t) {
  const auto p = t.find('[');
  return p == std::string::npos ? t : t.substr(0, p);
}

bool build_fields_string(const std::string& name, const MsgResolver& resolver, int depth,
                         std::string* out, std::string* error) {
  if (depth > 16) {
    if (error) *error = "message nesting deeper than 16 at " + name;
    return false;
  }
  const auto text = resolver(name);
  if (!text) {
    if (error) *error = "no .msg definition for " + name;
    return false;
  }
  for (const auto& f : parse_msg_fields(*text)) {
    *out += f.type;
    *out += ' ';
    *out += f.name;
    *out += '\n';
    const std::string base = strip_array(f.type);
    if (!is_builtin(base)) {
      if (!build_fields_string(base, resolver, depth + 1, out, error)) return false;
    }
  }
  return true;
}

}  // namespace

uint32_t fnv1a32(std::string_view data) {
  uint32_t h = 0x811c9dc5u;
  for (const unsigned char c : data) {
    h ^= c;
    h *= 0x1000193u;
  }
  return h;
}

std::vector<MsgField> parse_msg_fields(std::string_view text) {
  std::vector<MsgField> fields;
  size_t pos = 0;
  while (pos <= text.size()) {
    size_t eol = text.find('\n', pos);
    if (eol == std::string_view::npos) eol = text.size();
    std::string_view line = text.substr(pos, eol - pos);
    pos = eol + 1;
    const auto hash = line.find('#');
    if (hash != std::string_view::npos) line = line.substr(0, hash);
    if (line.find('=') != std::string_view::npos) continue;  // constant
    // tokenise on whitespace
    std::vector<std::string> tok;
    size_t i = 0;
    while (i < line.size()) {
      while (i < line.size() && (line[i] == ' ' || line[i] == '\t' || line[i] == '\r')) ++i;
      size_t j = i;
      while (j < line.size() && line[j] != ' ' && line[j] != '\t' && line[j] != '\r') ++j;
      if (j > i) tok.emplace_back(line.substr(i, j - i));
      i = j;
    }
    if (tok.size() >= 2) fields.push_back({tok[0], tok[1]});
  }
  return fields;
}

std::optional<uint32_t> message_hash(const std::string& name, const MsgResolver& resolver,
                                     std::string* error) {
  std::string s;
  if (!build_fields_string(name, resolver, 0, &s, error)) return std::nullopt;
  return fnv1a32(s);
}

}  // namespace dyx3_px4_link
