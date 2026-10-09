#include "dyx3_gnss_rtk/serial_port.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cctype>

namespace dyx3_gnss_rtk {
namespace {

struct BaudRate {
  int value;
  speed_t termios_value;
};

constexpr std::array<BaudRate, 10> kBauds{{
    {9600, B9600},
    {19200, B19200},
    {38400, B38400},
    {57600, B57600},
    {115200, B115200},
    {230400, B230400},
    {460800, B460800},
    {500000, B500000},
    {576000, B576000},
    {921600, B921600},
}};

}  // namespace

namespace {
bool stable_name_under(const std::string& path, const std::string& prefix, bool allow_colon) {
  if (prefix.empty() || prefix.back() != '/' || path.rfind(prefix, 0) != 0 ||
      path.size() <= prefix.size())
    return false;
  const std::string name = path.substr(prefix.size());
  if (name == "." || name == ".." || name.find('/') != std::string::npos) return false;
  for (const unsigned char c : name) {
    if (!(std::isalnum(c) || c == '_' || c == '-' || c == '.' || (allow_colon && c == ':')))
      return false;
  }
  return true;
}
}  // namespace

bool stable_serial_path(const std::string& path, const std::string& prefix) {
  if (stable_name_under(path, prefix, false)) return true;
  // A USB-serial bridge without a serial number (the rover's CH340) has no unique by-id name,
  // so the installer records the physical-port identity under by-path instead. Those names
  // contain ':' (e.g. platform-3610000.usb-usb-0:2.1:1.0-port0). Accepted only alongside the
  // production by-id prefix; tests that inject their own prefix keep single-prefix semantics.
  return prefix == kByIdPrefix && stable_name_under(path, kByPathPrefix, true);
}

bool supported_serial_baud(int baud) {
  for (const auto& b : kBauds)
    if (b.value == baud) return true;
  return false;
}

int open_serial_exclusive(const std::string& path, int baud, const std::string& prefix) {
  if (!stable_serial_path(path, prefix)) return -1;
  speed_t speed = 0;
  for (const auto& b : kBauds)
    if (b.value == baud) speed = b.termios_value;
  if (speed == 0) return -1;
  const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) return -1;
  termios tio{};
  if (::ioctl(fd, TIOCEXCL) != 0 || ::tcgetattr(fd, &tio) != 0) {
    ::close(fd);
    return -1;
  }
  ::cfmakeraw(&tio);
  tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
  tio.c_cflag |= CLOCAL | CREAD | CS8;
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (::cfsetispeed(&tio, speed) != 0 || ::cfsetospeed(&tio, speed) != 0 ||
      ::tcsetattr(fd, TCSANOW, &tio) != 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

}  // namespace dyx3_gnss_rtk
