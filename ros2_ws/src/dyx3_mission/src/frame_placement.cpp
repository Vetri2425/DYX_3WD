// frame_placement — see docs/contracts/dyx3_mission.md section 6 and frame_placement.hpp.
#include "dyx3_mission/frame_placement.hpp"

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>

namespace dyx3_mission {

namespace {

// PX4 src/lib/geo/geo.h CONSTANTS_RADIUS_OF_EARTH.
constexpr double kPx4EarthRadiusM = 6371000.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// ---- minimal JSON reader for the two meta keys (the meta was already verified canonical) ----
struct JsonValue {
  enum class Kind { kNull, kBool, kNumber, kString, kObject, kOther } kind = Kind::kOther;
  double number = 0.0;
  std::string text;
  std::size_t begin = 0;  // object: offset of '{' in the source
};

class MetaReader {
public:
  explicit MetaReader(const std::string& s) : s_(s) {}

  // Calls `on_member(key, value_offset)` for each top-level member of the object at `at`. The
  // callback must consume the value by calling value().
  template <typename F>
  bool object(std::size_t at, F&& on_member) {
    i_ = at;
    if (!eat('{')) return false;
    if (eat('}')) return true;
    while (true) {
      std::string key;
      if (!string(key) || !eat(':')) return false;
      if (!on_member(key)) return false;
      if (eat(',')) continue;
      return eat('}');
    }
  }

  // Parses one value at the cursor; objects and arrays are skipped (offset recorded).
  bool value(JsonValue& v, int depth = 0) {
    if (i_ >= s_.size() || depth > 64) return false;
    const char c = s_[i_];
    if (c == '{' || c == '[') {
      v.kind = c == '{' ? JsonValue::Kind::kObject : JsonValue::Kind::kOther;
      v.begin = i_;
      return skip_container(depth);
    }
    if (c == '"') {
      v.kind = JsonValue::Kind::kString;
      return string(v.text);
    }
    if (literal("null")) {
      v.kind = JsonValue::Kind::kNull;
      return true;
    }
    if (literal("true") || literal("false")) {
      v.kind = JsonValue::Kind::kBool;
      return true;
    }
    const std::size_t start = i_;
    while (i_ < s_.size() && std::string("+-0123456789.eE").find(s_[i_]) != std::string::npos) ++i_;
    if (i_ == start) return false;
    const std::string tok = s_.substr(start, i_ - start);
    char* end = nullptr;
    v.number = std::strtod(tok.c_str(), &end);
    v.kind = JsonValue::Kind::kNumber;
    return end == tok.c_str() + tok.size() && std::isfinite(v.number);
  }

private:
  bool eat(char c) {
    if (i_ < s_.size() && s_[i_] == c) {
      ++i_;
      return true;
    }
    return false;
  }
  bool literal(const char* w) {
    const std::string word(w);
    if (s_.compare(i_, word.size(), word) != 0) return false;
    i_ += word.size();
    return true;
  }
  bool string(std::string& out) {
    if (!eat('"')) return false;
    while (i_ < s_.size()) {
      const char c = s_[i_++];
      if (c == '"') return true;
      if (c == '\\') {
        if (i_ >= s_.size()) return false;
        out += '\\';
        out += s_[i_++];  // escapes are kept verbatim: only plain ASCII keys/values are compared
        continue;
      }
      out += c;
    }
    return false;
  }
  bool skip_container(int depth) {
    const char open = s_[i_];
    const char close = open == '{' ? '}' : ']';
    ++i_;
    if (eat(close)) return true;
    while (true) {
      if (open == '{') {
        std::string key;
        if (!string(key) || !eat(':')) return false;
      }
      JsonValue ignored;
      if (!value(ignored, depth + 1)) return false;
      if (eat(',')) continue;
      return eat(close);
    }
  }

  const std::string& s_;
  std::size_t i_ = 0;
};

FrameSpec frame_error(const std::string& why) {
  FrameSpec f;
  f.ok = false;
  f.error = why;
  return f;
}

bool valid_geo(const GeoPoint& g) {
  return std::isfinite(g.lat_deg) && std::isfinite(g.lon_deg) && std::fabs(g.lat_deg) <= 90.0 &&
         std::fabs(g.lon_deg) <= 180.0;
}

Placement placement_error(PlacementError e, const std::string& detail) {
  Placement p;
  p.ok = false;
  p.error = e;
  p.detail = detail;
  return p;
}

}  // namespace

NePoint project_to_ekf(const GeoPoint& ref, const GeoPoint& p) {
  const double ref_lat = ref.lat_deg * kDegToRad;
  const double ref_lon = ref.lon_deg * kDegToRad;
  const double ref_sin_lat = std::sin(ref_lat);
  const double ref_cos_lat = std::cos(ref_lat);
  const double lat_rad = p.lat_deg * kDegToRad;
  const double lon_rad = p.lon_deg * kDegToRad;
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double cos_d_lon = std::cos(lon_rad - ref_lon);
  const double arg =
      std::clamp(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0, 1.0);
  const double c = std::acos(arg);
  double k = 1.0;
  if (std::fabs(c) > 0.0) k = c / std::sin(c);
  NePoint out;
  out.north_m = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * kPx4EarthRadiusM;
  out.east_m = k * cos_lat * std::sin(lon_rad - ref_lon) * kPx4EarthRadiusM;
  return out;
}

FrameSpec read_frame_spec(const std::string& meta_json) {
  MetaReader top(meta_json);
  std::optional<std::string> frame;
  bool frame_not_string = false;
  bool anchor_present = false;  // a non-null anchor
  bool anchor_bad = false;
  std::optional<double> lat, lon;
  std::size_t anchor_at = 0;
  const bool parsed = top.object(0, [&](const std::string& key) {
    JsonValue v;
    if (!top.value(v)) return false;
    if (key == "frame") {
      if (v.kind == JsonValue::Kind::kString) {
        frame = v.text;
      } else {
        frame_not_string = true;
      }
    } else if (key == "anchor") {
      if (v.kind == JsonValue::Kind::kObject) {
        anchor_present = true;
        anchor_at = v.begin;
      } else if (v.kind != JsonValue::Kind::kNull) {
        anchor_bad = true;
      }
    }
    return true;
  });
  if (!parsed) return frame_error("artifact meta is not a JSON object");
  if (anchor_present) {
    MetaReader inner(meta_json);
    const bool ok = inner.object(anchor_at, [&](const std::string& key) {
      JsonValue v;
      if (!inner.value(v)) return false;
      if (key == "lat" || key == "lon") {
        if (v.kind != JsonValue::Kind::kNumber) return false;
        (key == "lat" ? lat : lon) = v.number;
      }
      return true;
    });
    if (!ok || !lat || !lon) anchor_bad = true;
  }
  if (!frame) {
    return frame_error(frame_not_string ? "meta \"frame\" is not a string"
                                        : "meta has no \"frame\": EKF-local is never assumed");
  }
  if (anchor_bad) return frame_error("meta \"anchor\" is not {lat, lon} numbers or null");
  FrameSpec f;
  if (*frame == "local_ned") {
    if (!anchor_present) return frame_error("frame \"local_ned\" without an anchor");
    f.kind = FrameKind::kAnchored;
    f.anchor.lat_deg = *lat;
    f.anchor.lon_deg = *lon;
    if (!valid_geo(f.anchor)) return frame_error("anchor latitude/longitude out of range");
  } else if (*frame == "ekf_local_ned") {
    if (anchor_present) return frame_error("frame \"ekf_local_ned\" with an anchor");
    f.kind = FrameKind::kEkfLocal;
  } else {
    return frame_error("unknown frame \"" + *frame + "\"");
  }
  f.ok = true;
  return f;
}

Placement place_artifact(const PathArtifact& source, const EkfReference& ref,
                         double max_distance_m) {
  const FrameSpec frame = read_frame_spec(source.meta_json);
  if (!frame.ok) return placement_error(PlacementError::kNoPlacementFrame, frame.error);
  if (!(max_distance_m > 0.0) || !std::isfinite(max_distance_m)) {
    return placement_error(PlacementError::kNotSerializable, "invalid placement bound");
  }
  const auto too_far = [max_distance_m](double n, double e) {
    return !std::isfinite(n) || !std::isfinite(e) || std::hypot(n, e) > max_distance_m;
  };

  if (frame.kind == FrameKind::kEkfLocal) {
    for (std::size_t i = 0; i < source.points.size(); ++i) {
      if (too_far(source.points[i].north_m, source.points[i].east_m)) {
        return placement_error(PlacementError::kOutOfBounds,
                               "point " + std::to_string(i) + " is more than " +
                                   python_repr(max_distance_m) + " m from the EKF origin");
      }
    }
    Placement p;
    p.ok = true;
    p.transformed = false;
    p.execution = source;
    p.detail = "frame ekf_local_ned: driven unchanged";
    return p;
  }

  const GeoPoint ekf_ref{ref.lat_deg, ref.lon_deg};
  if (!ref.global_reference_valid || !valid_geo(ekf_ref)) {
    return placement_error(PlacementError::kReferenceInvalid,
                           "no valid EKF global reference to place an anchored trajectory");
  }
  const NePoint a = project_to_ekf(ekf_ref, frame.anchor);
  if (too_far(a.north_m, a.east_m)) {
    return placement_error(PlacementError::kOutOfBounds,
                           "anchor is " + python_repr(std::hypot(a.north_m, a.east_m)) +
                               " m from the EKF origin (limit " + python_repr(max_distance_m) +
                               " m)");
  }
  Placement p;
  p.transformed = true;
  p.anchor_ekf = a;
  p.execution.version = 1;
  p.execution.engine_id = source.engine_id;
  p.execution.points.reserve(source.points.size());
  for (std::size_t i = 0; i < source.points.size(); ++i) {
    ArtifactPoint q = source.points[i];
    q.north_m = a.north_m + source.points[i].north_m;
    q.east_m = a.east_m + source.points[i].east_m;
    if (too_far(q.north_m, q.east_m)) {
      return placement_error(PlacementError::kOutOfBounds,
                             "placed point " + std::to_string(i) + " is more than " +
                                 python_repr(max_distance_m) + " m from the EKF origin");
    }
    p.execution.points.push_back(q);
  }
  // Canonical meta (sorted keys, compact, repr floats). "frame":"ekf_execution" is not a source
  // frame, so an execution artifact can never be started (and re-placed) as a source.
  p.execution.meta_json =
      "{\"execution\":{\"anchor_ekf_ne_m\":[" + python_repr(a.north_m) + "," +
      python_repr(a.east_m) + "],\"ekf_reference\":{\"lat\":" + python_repr(ref.lat_deg) +
      ",\"lon\":" + python_repr(ref.lon_deg) + "},\"source_sha256\":\"" + source.sha256 +
      "\"},\"frame\":\"ekf_execution\",\"source_meta\":" + source.meta_json + "}";
  p.bytes = serialize_artifact(p.execution.engine_id, p.execution.meta_json, p.execution.points);
  if (p.bytes.empty()) {
    return placement_error(PlacementError::kNotSerializable,
                           "execution artifact has no canonical spelling");
  }
  // Self-check: the reader RPP uses accepts exactly these bytes.
  const ArtifactResult check = parse_artifact(p.bytes);
  if (!check.ok) {
    return placement_error(PlacementError::kNotSerializable,
                           "execution artifact refused by the reader: " + check.error);
  }
  p.execution.sha256 = check.artifact.sha256;
  p.ok = true;
  p.detail = "anchor at N " + python_repr(a.north_m) + " m, E " + python_repr(a.east_m) +
             " m in the EKF frame";
  return p;
}

bool store_execution_artifact(const std::string& dir, const Placement& p, std::string* error) {
  if (!p.ok) {
    if (error) *error = "placement failed";
    return false;
  }
  if (!p.transformed) return true;
  namespace fs = std::filesystem;
  const fs::path target = fs::path(dir) / (p.execution.sha256 + ".dyx3path");
  const fs::path tmp =
      fs::path(dir) / (p.execution.sha256 + ".dyx3path.tmp." + std::to_string(getpid()));
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    out.write(p.bytes.data(), static_cast<std::streamsize>(p.bytes.size()));
    out.close();
    if (!out) {
      std::error_code ec;
      fs::remove(tmp, ec);
      if (error) *error = "cannot write " + tmp.string();
      return false;
    }
  }
  std::error_code ec;
  fs::rename(tmp, target, ec);
  if (ec) {
    std::error_code rm;
    fs::remove(tmp, rm);
    if (error) *error = "cannot publish " + target.string() + ": " + ec.message();
    return false;
  }
  return true;
}

}  // namespace dyx3_mission
