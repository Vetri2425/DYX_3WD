// json — strict JSON parser and small writers for the gateway protocol. Pure C++ (std only).
// Parser: RFC 8259 subset. Rejects duplicate object keys, trailing garbage, control characters in
// strings, leading zeros, NaN/Infinity, nesting deeper than kMaxDepth. Numbers keep an "is integer"
// flag so ids stay exact.
#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace dyx3_gateway {

constexpr int kMaxDepth = 16;

struct JsonValue {
  enum class Type { Null, Bool, Number, String, Array, Object };
  Type type{Type::Null};
  bool b{false};
  double n{0.0};
  bool is_int{false};
  int64_t i{0};
  std::string s;
  std::vector<JsonValue> a;
  std::vector<std::pair<std::string, JsonValue>> o;

  const JsonValue* get(const std::string& key) const {
    for (const auto& kv : o)
      if (kv.first == key) return &kv.second;
    return nullptr;
  }
};

bool parse_json(const std::string& text, JsonValue* out, std::string* err);

std::string json_escape(const std::string& s);
inline std::string json_str(const std::string& s) { return "\"" + json_escape(s) + "\""; }
std::string json_num(double v);  // 9 significant digits (float32 fields); non-finite -> null
std::string json_dbl(
    double v);  // 17 significant digits (float64 fields such as latitude); non-finite -> null
inline std::string json_bool(bool v) { return v ? "true" : "false"; }

// Insertion-ordered object builder producing one line (no whitespace), for the NDJSON protocol.
class JsonLine {
public:
  JsonLine& str(const std::string& k, const std::string& v);
  JsonLine& num(const std::string& k, double v);
  JsonLine& integer(const std::string& k, int64_t v);
  JsonLine& boolean(const std::string& k, bool v);
  JsonLine& raw(const std::string& k, const std::string& json);
  std::string dump() const;

private:
  std::string body_;
};

}  // namespace dyx3_gateway
