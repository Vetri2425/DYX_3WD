// boundary_projection — path model, windowed nearest-segment projection with the direction gate,
// MARK-boundary lead, terminal shutoff and cross-track hysteresis. Contract:
// docs/contracts/dyx3_spray.md sections 5 and 6. Pure C++, no ROS. Port of
// spray_controller_node._build_path_model / _project_onto_path / _make_spray_decision (continuous
// mode), proven against the verbatim Python by tools/gate4/gen_spray_vectors.py.
//
// Frame: local NED, metres (north, east). The prototype converted from ENU poses; that conversion
// does not exist here.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace dyx3_spray {

enum class BoundaryKind : uint8_t { TransitToMark = 0, MarkToTransit = 1 };

struct Boundary {
  double s;
  BoundaryKind kind;
};

struct PathModel {
  std::vector<double> north, east;
  std::vector<bool> flags;        // true = MARK
  std::vector<uint32_t> run_ids;  // conditioned RPP run ownership; cross-run links are invalid
  std::vector<double> cumulative_s;
  std::vector<Boundary> boundaries;
  bool empty() const { return north.empty(); }
  size_t size() const { return north.size(); }
};

// Builds the model. A path that ends on MARK gets a synthetic terminal MARK->TRANSIT boundary at
// the final station (hardening carried from the prototype). Returns nullopt-equivalent (empty
// model) when sizes differ.
bool build_path_model(const std::vector<double>& north, const std::vector<double>& east,
                      const std::vector<bool>& flags, PathModel* out);
bool build_path_model(const std::vector<double>& north, const std::vector<double>& east,
                      const std::vector<bool>& flags, const std::vector<uint32_t>& run_ids,
                      PathModel* out);

struct Projection {
  int segment_index{0};
  double t{0.0};
  double proj_n{0.0}, proj_e{0.0};
  double s{0.0};
  double xtrack_error_m{0.0};
  bool current_flag{false};
};

// Half-angle in degrees -> cos threshold. <=0 or >=180 disables (0.0); 90 maps to 1e-6.
double direction_gate_cos(double gate_deg);

struct ProjectionConfig {
  std::optional<double> prev_s;  // nullopt: global scan (path just loaded)
  double window_back_m{0.0};
  double window_fwd_m{0.0};
  double reacquire_dist_m{0.0};
  std::optional<double> heading_rad;  // NED radians, nullopt: no direction gate
  double direction_gate_cos{0.0};     // 0.0 = gate disabled
};

std::optional<Projection> project_onto_path(const PathModel& m, double point_n, double point_e,
                                            const ProjectionConfig& cfg);

// Body-frame nozzle offsets applied in NED; lateral is positive to rover-right.
void nozzle_position_ned(double pose_n, double pose_e, double yaw_ned, double forward_m,
                         double lateral_m, double* n, double* e);

std::optional<Boundary> next_boundary(const PathModel& m, double current_s, bool current_flag);

enum class LeadEvent : uint8_t { None = 0, OnEarly = 1, OffEarly = 2, TerminalOff = 3 };
const char* to_string(LeadEvent e);

// Returns the new geometry_desired; `event` set to None/OnEarly/OffEarly.
bool apply_mark_boundary_lead(bool geometry_desired, std::optional<BoundaryKind> src_kind,
                              double src_dist, double on_lead, double off_lead, LeadEvent* event);

struct DecisionParams {
  double solenoid_open_delay_s{0.0};
  double solenoid_close_delay_s{0.0};
  double on_overspray_margin_m{0.0};
  double off_overspray_margin_m{0.0};
  double max_xtrack_error_m{0.0};
  bool max_xtrack_from_mission{
      false};  // a mission override IS the gate (not widened by the trip band)
  double xtrack_trip_error_m{0.0};
  double xtrack_gate_min_off_s{0.0};
  double terminal_off_epsilon_m{0.0};
  double terminal_off_speed_mps{0.0};
  double projection_window_back_m{0.0};
  double projection_window_fwd_m{0.0};
  double projection_reacquire_dist_m{0.0};
  double projection_direction_gate_cos{0.0};
};

struct DecisionState {
  std::optional<double> prev_projection_s;  // carried back by the caller after each decision
  bool xtrack_tripped{false};
  double xtrack_tripped_elapsed_s{
      1e300};  // seconds since the trip edge (infinite when not tripped)
};

struct DecisionInput {
  const PathModel* model{nullptr};
  std::optional<double> nozzle_n, nozzle_e;
  double speed_mps{0.0};
  double yaw_rad{0.0};
  bool safety_ok{false};
  std::string safety_reason;
};

struct Decision {
  bool desired{false};
  bool geometry_desired{false};
  bool safety_ok{false};
  std::string safety_reason;
  std::optional<Projection> projection;
  std::optional<Boundary> next_boundary;
  double distance_to_boundary_m;  // +inf when none
  LeadEvent event{LeadEvent::None};
  bool xtrack_tripped{false};
};

Decision make_decision(const DecisionInput& in, const DecisionParams& p, const DecisionState& st);

}  // namespace dyx3_spray
