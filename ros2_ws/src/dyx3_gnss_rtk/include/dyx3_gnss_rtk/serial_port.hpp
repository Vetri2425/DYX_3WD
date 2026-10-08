#pragma once

#include <string>

namespace dyx3_gnss_rtk {

// Production callers use the default prefix. Tests inject an isolated by-id directory with
// symlinks to a pty; neither caller ever guesses tty enumeration order.
bool stable_serial_path(const std::string& path,
                        const std::string& by_id_prefix = "/dev/serial/by-id/");
bool supported_serial_baud(int baud);
int open_serial_exclusive(const std::string& path, int baud,
                          const std::string& by_id_prefix = "/dev/serial/by-id/");

}  // namespace dyx3_gnss_rtk
