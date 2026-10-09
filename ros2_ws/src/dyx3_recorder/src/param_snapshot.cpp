#include "dyx3_recorder/param_snapshot.hpp"

#include <algorithm>
#include <map>

#include "dyx3_recorder/run_manifest.hpp"

namespace dyx3_recorder {

std::string params_ros_snapshot_json(const std::string& captured_utc,
                                     const std::vector<NodeParams>& nodes) {
  std::vector<const NodeParams*> sorted;
  for (const auto& n : nodes) sorted.push_back(&n);
  std::sort(sorted.begin(), sorted.end(), [](auto* a, auto* b) { return a->node < b->node; });
  JsonObject nodes_obj;
  for (const NodeParams* n : sorted) {
    std::map<std::string, const ParamEntry*> by_name;
    for (const auto& p : n->params) by_name[p.name] = &p;
    JsonObject params;
    for (const auto& [name, p] : by_name) {
      JsonObject e;
      e.str("type", p->type).str("value", p->value);
      params.raw(name, e.dump(2, 4));
    }
    JsonObject one;
    one.boolean("reachable", n->reachable);
    if (!n->note.empty()) one.str("note", n->note);
    one.raw("params", params.dump(2, 3));
    nodes_obj.raw(n->node, one.dump(2, 2));
  }
  JsonObject top;
  top.str("captured_utc", captured_utc).raw("nodes", nodes_obj.dump(2, 1));
  return top.dump();
}

std::string params_ros_file_json(const std::string& start_snapshot,
                                 const std::string& end_snapshot) {
  JsonObject top;
  top.integer("schema", 1)
      .raw("start", start_snapshot.empty() ? "null" : start_snapshot)
      .raw("end", end_snapshot.empty() ? "null" : end_snapshot);
  return top.dump() + "\n";
}

std::string unavailable_json(const std::string& what, const std::string& reason) {
  JsonObject o;
  o.integer("schema", 1).str("what", what).str("status", "unavailable").str("reason", reason);
  return o.dump() + "\n";
}

}  // namespace dyx3_recorder
