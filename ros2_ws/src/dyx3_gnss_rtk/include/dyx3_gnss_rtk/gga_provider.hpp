// gga_provider — NMEA GGA sentence for the NTRIP back-feed. Contract: dyx3_gnss_rtk.md section 4.
// Pure C++.
#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace dyx3_gnss_rtk {

struct GgaInput {
  double latitude_deg{0};
  double longitude_deg{0};
  double altitude_msl_m{0};
  double position_age_s{0};  // age of the position sample (seconds)
  int64_t utc_epoch_s{0};    // POSIX seconds of the report; <= 0 gives 000000.00 (as the prototype)
  uint8_t fix_type{0};       // SensorGps.fix_type
  uint8_t satellites{0};
  double hdop{0};
};

// Reject stale, non-finite, or out-of-range fixes before the back-feed
// (ntrip_protocol.gga_position_is_usable).
bool gga_position_is_usable(double latitude, double longitude, double altitude, double age_s,
                            double max_age_s = 5.0);

// XOR of the bytes between '$' and '*', two upper-case hex digits.
std::string nmea_checksum(const std::string& body);

// Full sentence "$GPGGA,...*CS\r\n", or nullopt when the position is not usable.
// Quality: 1 GPS, 2 DGPS (RTCM code differential), 4 RTK fixed, 5 RTK float (6 = estimated for
// none).
std::optional<std::string> format_gga(const GgaInput& in, double max_age_s = 5.0);

}  // namespace dyx3_gnss_rtk
