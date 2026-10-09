#pragma once

#include <string>

namespace dyx3_gnss_rtk {

inline constexpr const char* kByIdPrefix = "/dev/serial/by-id/";
inline constexpr const char* kByPathPrefix = "/dev/serial/by-path/";

// Production callers use the default prefix, which accepts /dev/serial/by-id/ and, for adapters
// without a USB serial number, /dev/serial/by-path/. Tests inject an isolated directory with
// symlinks to a pty; no caller ever guesses tty enumeration order.
bool stable_serial_path(const std::string& path,
                        const std::string& by_id_prefix = "/dev/serial/by-id/");
bool supported_serial_baud(int baud);
int open_serial_exclusive(const std::string& path, int baud,
                          const std::string& by_id_prefix = "/dev/serial/by-id/");

}  // namespace dyx3_gnss_rtk
