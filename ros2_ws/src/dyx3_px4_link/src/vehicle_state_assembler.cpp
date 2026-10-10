#include "dyx3_px4_link/vehicle_state_assembler.hpp"

#include <cmath>

namespace dyx3_px4_link {
namespace {
constexpr double kTwoPi = 6.283185307179586;
}  // namespace

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
    // The gateway and the map consume the reference: a non-finite one is never presented as valid.
    o.global_reference_valid = lp.xy_global && std::isfinite(lp.ref_lat) &&
                               std::isfinite(lp.ref_lon) && std::isfinite(lp.ref_alt);
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

double yaw_of(const std::array<float, 4>& q) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

void YawRateEstimator::update(const std::array<float, 4>& q, uint64_t timestamp_sample_us,
                              uint8_t reset_counter) {
  bool finite = true;
  for (const float c : q) finite = finite && std::isfinite(c);
  if (!finite) {
    have_prev_ = false;
    have_rate_ = false;
    return;
  }
  const bool continuous = have_prev_ && reset_counter == prev_reset_;
  if (continuous && timestamp_sample_us == prev_us_) return;  // the same sample again
  const double yaw = yaw_of(q);
  if (continuous && timestamp_sample_us > prev_us_) {
    const double dt = static_cast<double>(timestamp_sample_us - prev_us_) * 1e-6;
    if (dt <= kMaxDtS) {
      const double raw = std::remainder(yaw - prev_yaw_, kTwoPi) / dt;
      rate_ = have_rate_ ? rate_ + dt / (tau_s_ + dt) * (raw - rate_) : raw;
      have_rate_ = true;
    } else {
      have_rate_ = false;
    }
  } else {
    have_rate_ = false;
  }
  prev_yaw_ = yaw;
  prev_us_ = timestamp_sample_us;
  prev_reset_ = reset_counter;
  have_prev_ = true;
}

}  // namespace dyx3_px4_link
