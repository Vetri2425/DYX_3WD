#include "dyx3_system_gateway/json.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace dyx3_gateway {
namespace {

struct P {
  const std::string& t;
  size_t i{0};
  std::string err;
  bool fail(const std::string& m) {
    if (err.empty()) err = m + " at offset " + std::to_string(i);
    return false;
  }
  void ws() {
    while (i < t.size() && (t[i] == ' ' || t[i] == '\t' || t[i] == '\n' || t[i] == '\r')) ++i;
  }
  bool lit(const char* w) {
    size_t n = 0;
    while (w[n]) ++n;
    if (t.compare(i, n, w) != 0) return false;
    i += n;
    return true;
  }
  static void put_utf8(std::string& s, uint32_t c) {
    if (c < 0x80) {
      s += static_cast<char>(c);
    } else if (c < 0x800) {
      s += static_cast<char>(0xC0 | (c >> 6));
      s += static_cast<char>(0x80 | (c & 0x3F));
    } else if (c < 0x10000) {
      s += static_cast<char>(0xE0 | (c >> 12));
      s += static_cast<char>(0x80 | ((c >> 6) & 0x3F));
      s += static_cast<char>(0x80 | (c & 0x3F));
    } else {
      s += static_cast<char>(0xF0 | (c >> 18));
      s += static_cast<char>(0x80 | ((c >> 12) & 0x3F));
      s += static_cast<char>(0x80 | ((c >> 6) & 0x3F));
      s += static_cast<char>(0x80 | (c & 0x3F));
    }
  }
  bool hex4(uint32_t* v) {
    if (i + 4 > t.size()) return fail("short \\u escape");
    uint32_t r = 0;
    for (int k = 0; k < 4; ++k) {
      const char c = t[i++];
      r <<= 4;
      if (c >= '0' && c <= '9')
        r |= static_cast<uint32_t>(c - '0');
      else if (c >= 'a' && c <= 'f')
        r |= static_cast<uint32_t>(c - 'a' + 10);
      else if (c >= 'A' && c <= 'F')
        r |= static_cast<uint32_t>(c - 'A' + 10);
      else
        return fail("bad \\u escape");
    }
    *v = r;
    return true;
  }
  bool string(std::string* out) {
    if (t[i] != '"') return fail("expected string");
    ++i;
    out->clear();
    while (i < t.size()) {
      const unsigned char c = static_cast<unsigned char>(t[i++]);
      if (c == '"') return true;
      if (c < 0x20) return fail("control character in string");
      if (c != '\\') {
        *out += static_cast<char>(c);
        continue;
      }
      if (i >= t.size()) return fail("dangling escape");
      const char e = t[i++];
      switch (e) {
        case '"':
          *out += '"';
          break;
        case '\\':
          *out += '\\';
          break;
        case '/':
          *out += '/';
          break;
        case 'b':
          *out += '\b';
          break;
        case 'f':
          *out += '\f';
          break;
        case 'n':
          *out += '\n';
          break;
        case 'r':
          *out += '\r';
          break;
        case 't':
          *out += '\t';
          break;
        case 'u': {
          uint32_t cp;
          if (!hex4(&cp)) return false;
          if (cp >= 0xD800 && cp <= 0xDBFF) {
            uint32_t lo;
            if (i + 1 >= t.size() || t[i] != '\\' || t[i + 1] != 'u')
              return fail("lone high surrogate");
            i += 2;
            if (!hex4(&lo)) return false;
            if (lo < 0xDC00 || lo > 0xDFFF) return fail("bad low surrogate");
            cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
          } else if (cp >= 0xDC00 && cp <= 0xDFFF) {
            return fail("lone low surrogate");
          }
          put_utf8(*out, cp);
          break;
        }
        default:
          return fail("bad escape");
      }
    }
    return fail("unterminated string");
  }
  bool number(JsonValue* v) {
    const size_t st = i;
    if (t[i] == '-') ++i;
    if (i >= t.size()) return fail("bad number");
    if (t[i] == '0') {
      ++i;
      if (i < t.size() && t[i] >= '0' && t[i] <= '9') return fail("leading zero");
    } else if (t[i] >= '1' && t[i] <= '9') {
      while (i < t.size() && t[i] >= '0' && t[i] <= '9') ++i;
    } else {
      return fail("bad number");
    }
    bool integral = true;
    if (i < t.size() && t[i] == '.') {
      integral = false;
      ++i;
      if (i >= t.size() || t[i] < '0' || t[i] > '9') return fail("bad fraction");
      while (i < t.size() && t[i] >= '0' && t[i] <= '9') ++i;
    }
    if (i < t.size() && (t[i] == 'e' || t[i] == 'E')) {
      integral = false;
      ++i;
      if (i < t.size() && (t[i] == '+' || t[i] == '-')) ++i;
      if (i >= t.size() || t[i] < '0' || t[i] > '9') return fail("bad exponent");
      while (i < t.size() && t[i] >= '0' && t[i] <= '9') ++i;
    }
    const std::string tok = t.substr(st, i - st);
    v->type = JsonValue::Type::Number;
    v->n = std::strtod(tok.c_str(), nullptr);
    if (!std::isfinite(v->n)) return fail("number out of range");
    if (integral && tok.size() <= 18) {
      v->is_int = true;
      v->i = std::strtoll(tok.c_str(), nullptr, 10);
    }
    return true;
  }
  bool value(JsonValue* v, int depth) {
    if (depth > kMaxDepth) return fail("nesting too deep");
    ws();
    if (i >= t.size()) return fail("unexpected end");
    const char c = t[i];
    if (c == '{') {
      ++i;
      v->type = JsonValue::Type::Object;
      ws();
      if (i < t.size() && t[i] == '}') {
        ++i;
        return true;
      }
      for (;;) {
        ws();
        std::string key;
        if (i >= t.size() || t[i] != '"') return fail("expected key");
        if (!string(&key)) return false;
        for (const auto& kv : v->o)
          if (kv.first == key) return fail("duplicate key '" + key + "'");
        ws();
        if (i >= t.size() || t[i] != ':') return fail("expected ':'");
        ++i;
        JsonValue child;
        if (!value(&child, depth + 1)) return false;
        v->o.emplace_back(std::move(key), std::move(child));
        ws();
        if (i < t.size() && t[i] == ',') {
          ++i;
          continue;
        }
        if (i < t.size() && t[i] == '}') {
          ++i;
          return true;
        }
        return fail("expected ',' or '}'");
      }
    }
    if (c == '[') {
      ++i;
      v->type = JsonValue::Type::Array;
      ws();
      if (i < t.size() && t[i] == ']') {
        ++i;
        return true;
      }
      for (;;) {
        JsonValue child;
        if (!value(&child, depth + 1)) return false;
        v->a.push_back(std::move(child));
        ws();
        if (i < t.size() && t[i] == ',') {
          ++i;
          continue;
        }
        if (i < t.size() && t[i] == ']') {
          ++i;
          return true;
        }
        return fail("expected ',' or ']'");
      }
    }
    if (c == '"') {
      v->type = JsonValue::Type::String;
      return string(&v->s);
    }
    if (c == 't') {
      if (!lit("true")) return fail("bad literal");
      v->type = JsonValue::Type::Bool;
      v->b = true;
      return true;
    }
    if (c == 'f') {
      if (!lit("false")) return fail("bad literal");
      v->type = JsonValue::Type::Bool;
      v->b = false;
      return true;
    }
    if (c == 'n') {
      if (!lit("null")) return fail("bad literal");
      v->type = JsonValue::Type::Null;
      return true;
    }
    if (c == '-' || (c >= '0' && c <= '9')) return number(v);
    return fail("unexpected character");
  }
};

}  // namespace

bool parse_json(const std::string& text, JsonValue* out, std::string* err) {
  P p{text};
  JsonValue v;
  if (!p.value(&v, 0)) {
    if (err) *err = p.err;
    return false;
  }
  p.ws();
  if (p.i != text.size()) {
    p.fail("trailing characters");
    if (err) *err = p.err;
    return false;
  }
  *out = std::move(v);
  return true;
}

std::string json_escape(const std::string& s) {
  std::string o;
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

std::string json_num(double v) {
  if (!std::isfinite(v)) return "null";
  char b[40];
  std::snprintf(b, sizeof b, "%.9g", v);
  return b;
}

std::string json_dbl(double v) {
  if (!std::isfinite(v)) return "null";
  char b[40];
  std::snprintf(b, sizeof b, "%.17g", v);
  return b;
}

JsonLine& JsonLine::raw(const std::string& k, const std::string& json) {
  body_ += (body_.empty() ? "" : ",") + json_str(k) + ":" + json;
  return *this;
}
JsonLine& JsonLine::str(const std::string& k, const std::string& v) { return raw(k, json_str(v)); }
JsonLine& JsonLine::num(const std::string& k, double v) { return raw(k, json_num(v)); }
JsonLine& JsonLine::integer(const std::string& k, int64_t v) { return raw(k, std::to_string(v)); }
JsonLine& JsonLine::boolean(const std::string& k, bool v) { return raw(k, json_bool(v)); }
std::string JsonLine::dump() const { return "{" + body_ + "}"; }

}  // namespace dyx3_gateway
