// vehicle_state_assembler — PX4 samples -> the one canonical VehicleState.
// See docs/contracts/dyx3_px4_link.md section 8. Pure C++ on plain structs.
#pragma once

#include <array>
#include <cstdint>

namespace dyx3_px4_link {

struct LocalPositionSample {
  bool xy_valid{false};
  bool v_xy_valid{false};
  float x{0}, y{0}, z{0};
  float vx{0}, vy{0}, vz{0};
  float heading{0};
  bool heading_good_for_control{false};
  uint8_t xy_reset_counter{0};
  float delta_x{0}, delta_y{0};
  bool xy_global{false};
  double ref_lat{0}, ref_lon{0};
  float ref_alt{0};
  uint64_t timestamp_sample_us{0};
};

struct AttitudeSample {
  std::array<float, 4> q{1, 0, 0, 0};
};

struct StatusSample {
  uint8_t arming_state{0};
  uint8_t nav_state{0};
  bool failsafe{false};
  bool pre_flight_checks_pass{false};
};

struct Freshness {
  bool local_position{false};
  bool attitude{false};
  bool status{false};
};

struct VehicleStateOut {
  bool position_valid{false};
  bool velocity_valid{false};
  bool attitude_valid{false};
  float north{0}, east{0}, down{0};
  float vn{0}, ve{0}, vd{0};
  std::array<float, 4> q{0, 0, 0, 0};
  float heading{0};
  uint8_t xy_reset_counter{0};
  float delta_north{0}, delta_east{0};
  bool global_reference_valid{false};
  double ref_lat{0}, ref_lon{0};
  float ref_alt{0};
  uint8_t arming_state{0};
  uint8_t nav_state{0};
  bool failsafe{false};
  bool preflight_checks_pass{false};
  uint64_t px4_sample_us{0};
};

// A stale or never-received source contributes nothing: its fields stay at the safe zero values
// and its validity flags stay false.
VehicleStateOut assemble(const LocalPositionSample& lp, const AttitudeSample& att,
                         const StatusSample& st, const Freshness& fresh);

}  // namespace dyx3_px4_link
