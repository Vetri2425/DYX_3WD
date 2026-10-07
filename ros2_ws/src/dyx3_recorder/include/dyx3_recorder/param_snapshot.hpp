// param_snapshot — JSON for params_ros.json / params_fcu.json. Contract:
// docs/contracts/dyx3_recorder.md section 4. Pure C++. Collecting the values (rclcpp parameter
// clients, an FCU read path) is the node's job.
#pragma once

#include <string>
#include <vector>

namespace dyx3_recorder {

struct ParamEntry {
  std::string node;
  std::string name;
  std::string type;   // "double", "integer", "bool", "string", ...
  std::string value;  // value as text
};

struct NodeParams {
  std::string node;
  bool reachable{true};
  std::vector<ParamEntry> params;
};

// {"captured_utc": ..., "nodes": {"<node>": {"reachable": true, "params": {"<name>":
// {"type":..,"value":..}}}}} Nodes and parameters are sorted by name so two snapshots of the same
// state are byte-identical.
std::string params_ros_snapshot_json(const std::string& captured_utc,
                                     const std::vector<NodeParams>& nodes);

// params_ros.json = {"start": <snapshot>, "end": <snapshot or null>}
std::string params_ros_file_json(const std::string& start_snapshot,
                                 const std::string& end_snapshot);

// {"status": "unavailable", "reason": ...} — the explicit record of a provenance gap.
std::string unavailable_json(const std::string& what, const std::string& reason);

}  // namespace dyx3_recorder
