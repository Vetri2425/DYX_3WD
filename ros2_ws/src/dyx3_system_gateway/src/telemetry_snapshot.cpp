#include "dyx3_system_gateway/telemetry_snapshot.hpp"

#include "dyx3_system_gateway/json.hpp"

namespace dyx3_gateway {
namespace {
const char* const kSources[] = {
    "vehicle_state", "estimator_health",  "rtk_status",     "gnss_report",  "ntrip_status",
    "px4_link",      "safety_gate",       "emergency_stop", "motion_guard", "rpp",
    "mission",       "last_point_result", "spray",          "recorder"};
}

const char* const* TelemetrySnapshot::all_sources(size_t* n) {
  *n = sizeof(kSources) / sizeof(kSources[0]);
  return kSources;
}

void TelemetrySnapshot::update(const std::string& source, const std::string& fields_json,
                               double now_s) {
  latest_[source] = Entry{fields_json, now_s};
}

std::string TelemetrySnapshot::to_json(double now_s, const std::string& gateway_json) const {
  JsonLine o;
  size_t n;
  const char* const* names = all_sources(&n);
  for (size_t i = 0; i < n; ++i) {
    const auto it = latest_.find(names[i]);
    if (it == latest_.end()) {
      o.raw(names[i], "null");
      continue;
    }
    const double age = now_s - it->second.stamp;
    JsonLine e;
    e.num("age_s", age).boolean("fresh", age <= fresh_s_).raw("data", it->second.json);
    o.raw(names[i], e.dump());
  }
  o.raw("gateway", gateway_json);
  return o.dump();
}

}  // namespace dyx3_gateway
