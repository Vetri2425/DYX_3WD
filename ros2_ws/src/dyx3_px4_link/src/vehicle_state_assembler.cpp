#include "dyx3_px4_link/vehicle_state_assembler.hpp"

#include <cmath>

namespace dyx3_px4_link {

VehicleStateOut assemble(const LocalPositionSample& lp, const AttitudeSample& att,
                         const StatusSample& st, const Freshness& fresh) {
  VehicleStateOut o;
  if (fresh.local_position) {
    o.position_valid = lp.xy_valid && std::isfinite(lp.x) && std::isfinite(lp.y);
    o.velocity_valid = lp.v_xy_valid && std::isfinite(lp.vx) && std::isfinite(lp.vy);
    o.north = lp.x;
    o.east = lp.y;
    o.down = lp.z;
    o.vn = lp.vx;
    o.ve = lp.vy;
    o.vd = lp.vz;
    o.heading = lp.heading;
    o.xy_reset_counter = lp.xy_reset_counter;
    o.delta_north = lp.delta_x;
    o.delta_east = lp.delta_y;
    o.global_reference_valid = lp.xy_global;
    o.ref_lat = lp.ref_lat;
    o.ref_lon = lp.ref_lon;
    o.ref_alt = lp.ref_alt;
    o.px4_sample_us = lp.timestamp_sample_us;
  }
  if (fresh.attitude && fresh.local_position) {
    bool finite = true;
    for (const float c : att.q) finite = finite && std::isfinite(c);
    o.attitude_valid = lp.heading_good_for_control && finite && std::isfinite(lp.heading);
    if (finite) o.q = att.q;
  }
  if (fresh.status) {
    o.arming_state = st.arming_state;
    o.nav_state = st.nav_state;
    o.failsafe = st.failsafe;
    o.preflight_checks_pass = st.pre_flight_checks_pass;
  }
  return o;
}

}  // namespace dyx3_px4_link
