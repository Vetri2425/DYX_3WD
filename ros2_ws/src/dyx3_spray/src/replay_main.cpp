// spray_projection_replay — run the boundary projection over the PLANNED geometry of mission
// artifacts, gate off and on.
//
//   spray_projection_replay [--gates 0,60,90] [--lateral 0.01] [--noise 0.003] <artifact.dyx3path |
//   directory>...
//
// Prints one row per (mission, gate). This is the decision input for
// `projection_direction_gate_deg` (docs/contracts/dyx3_spray.md section 6): a mission is EXPOSED
// if, with the gate off, the station teleports or the projected MARK/TRANSIT flag disagrees with
// the plan. It drives the planned path, not a recorded trace: bag replay is still a LOCAL ACTION.
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "dyx3_mission/path_artifact.hpp"
#include "dyx3_spray/replay.hpp"

namespace {

std::vector<double> parse_list(const std::string& s) {
  std::vector<double> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) out.push_back(std::strtod(tok.c_str(), nullptr));
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<double> gates{0.0, 60.0};
  dyx3_spray::ReplayConfig base;
  std::vector<std::string> inputs;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--gates" && i + 1 < argc) {
      gates = parse_list(argv[++i]);
    } else if (a == "--lateral" && i + 1 < argc) {
      base.lateral_offset_m = std::strtod(argv[++i], nullptr);
    } else if (a == "--noise" && i + 1 < argc) {
      base.noise_amp_m = std::strtod(argv[++i], nullptr);
    } else {
      inputs.push_back(a);
    }
  }
  if (inputs.empty() || gates.empty()) {
    std::fprintf(stderr,
                 "usage: spray_projection_replay [--gates 0,60] [--lateral m] [--noise m] "
                 "<artifact|dir>...\n");
    return 2;
  }
  std::vector<std::string> files;
  for (const auto& in : inputs) {
    if (std::filesystem::is_directory(in)) {
      for (const auto& e : std::filesystem::directory_iterator(in)) {
        if (e.path().extension() == ".dyx3path") files.push_back(e.path().string());
      }
    } else {
      files.push_back(in);
    }
  }
  std::printf("%-28s %7s %6s %9s %9s %8s %8s %8s %9s\n", "mission", "len_m", "gate", "max_jump",
              "teleports", "wrong", "spurious", "missed", "final_err");
  int exposed = 0;
  for (const auto& f : files) {
    std::ifstream is(f, std::ios::binary);
    std::stringstream buf;
    buf << is.rdbuf();
    const auto r = dyx3_mission::parse_artifact(buf.str());
    if (!r.ok) {
      std::fprintf(stderr, "%s: %s\n", f.c_str(), r.error.c_str());
      continue;
    }
    std::vector<double> n, e;
    std::vector<bool> fl;
    for (const auto& p : r.artifact.points) {
      n.push_back(p.north_m);
      e.push_back(p.east_m);
      fl.push_back(p.spray());
    }
    dyx3_spray::PathModel m;
    if (!dyx3_spray::build_path_model(n, e, fl, &m)) {
      std::fprintf(stderr, "%s: inconsistent geometry\n", f.c_str());
      continue;
    }
    const std::string name = std::filesystem::path(f).stem().string().substr(0, 26);
    for (const double g : gates) {
      dyx3_spray::ReplayConfig c = base;
      c.gate_deg = g;
      const auto s = dyx3_spray::replay_drive_along(m, c);
      std::printf("%-28s %7.2f %6.0f %9.3f %9zu %8zu %8zu %8zu %9.3f\n", name.c_str(),
                  s.path_length_m, g, s.max_jump_m, s.teleports, s.wrong_flag_samples,
                  s.spurious_mark_samples, s.missed_mark_samples, s.final_s_error_m);
      if (g == 0.0 && (s.teleports > 0 || s.wrong_flag_samples > 0)) ++exposed;
    }
  }
  std::printf(
      "\n%d mission(s) EXPOSED with the gate off (teleport or wrong flag). Planned geometry, not a "
      "recorded trace.\n",
      exposed);
  return 0;
}
