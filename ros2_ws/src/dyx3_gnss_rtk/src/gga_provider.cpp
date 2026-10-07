#include "dyx3_gnss_rtk/gga_provider.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>

namespace dyx3_gnss_rtk {

bool gga_position_is_usable(double latitude, double longitude, double altitude, double age_s,
                            double max_age_s) {
  if (!std::isfinite(latitude) || !std::isfinite(longitude) || !std::isfinite(altitude) ||
      !std::isfinite(age_s))
    return false;
  return latitude >= -90.0 && latitude <= 90.0 && longitude >= -180.0 && longitude <= 180.0 &&
         age_s >= 0.0 && age_s <= std::max(0.0, max_age_s);
}

std::string nmea_checksum(const std::string& body) {
  unsigned chk = 0;
  for (const unsigned char c : body) chk ^= c;
  char buf[4];
  std::snprintf(buf, sizeof buf, "%02X", chk & 0xFFU);
  return buf;
}

std::optional<std::string> format_gga(const GgaInput& in, double max_age_s) {
  if (!gga_position_is_usable(in.latitude_deg, in.longitude_deg, in.altitude_msl_m,
                              in.position_age_s, max_age_s)) {
    return std::nullopt;
  }
  const double lat_abs = std::fabs(in.latitude_deg);
  const int lat_deg = static_cast<int>(lat_abs);
  const double lat_min = (lat_abs - lat_deg) * 60.0;
  const char lat_dir = in.latitude_deg >= 0 ? 'N' : 'S';
  const double lon_abs = std::fabs(in.longitude_deg);
  const int lon_deg = static_cast<int>(lon_abs);
  const double lon_min = (lon_abs - lon_deg) * 60.0;
  const char lon_dir = in.longitude_deg >= 0 ? 'E' : 'W';

  char tbuf[16] = "000000.00";
  if (in.utc_epoch_s > 0) {
    const std::time_t tt = static_cast<std::time_t>(in.utc_epoch_s);
    std::tm tm{};
    gmtime_r(&tt, &tm);
    std::snprintf(tbuf, sizeof tbuf, "%02d%02d%02d.00", tm.tm_hour, tm.tm_min, tm.tm_sec);
  }
  int quality = 1;
  if (in.fix_type == 6)
    quality = 4;
  else if (in.fix_type == 5)
    quality = 5;
  else if (in.fix_type == 4)
    quality = 2;

  char body[200];
  std::snprintf(body, sizeof body, "GPGGA,%s,%02d%07.4f,%c,%03d%07.4f,%c,%d,%u,%.1f,%.1f,M,0.0,M,,",
                tbuf, lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir, quality,
                static_cast<unsigned>(in.satellites), in.hdop, in.altitude_msl_m);
  const std::string b(body);
  return "$" + b + "*" + nmea_checksum(b) + "\r\n";
}

}  // namespace dyx3_gnss_rtk
