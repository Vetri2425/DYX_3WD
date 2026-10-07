// telemetry_snapshot — canonical snapshot assembly with per-source ageing. Contract:
// docs/contracts/dyx3_system_gateway.md section 3. Pure C++, clock-injected.
#pragma once

#include <map>
#include <string>

namespace dyx3_gateway {

class TelemetrySnapshot {
public:
  explicit TelemetrySnapshot(double fresh_s = 1.0) : fresh_s_(fresh_s) {}
  // `fields_json` must be a valid JSON object (the node builds it from the ROS message).
  void update(const std::string& source, const std::string& fields_json, double now_s);
  // {"<source>": {"age_s": .., "fresh": .., "data": {...}} | null, ...,"gateway": {...}} — sources
  // are listed in the order given by `all_sources`, never-received ones as null so a consumer
  // cannot mistake absence for a value.
  std::string to_json(double now_s, const std::string& gateway_json) const;
  static const char* const* all_sources(size_t* n);

private:
  struct Entry {
    std::string json;
    double stamp{0.0};
  };
  double fresh_s_;
  std::map<std::string, Entry> latest_;
};

}  // namespace dyx3_gateway
