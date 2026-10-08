#include "dyx3_px4_link/spray_ack_tokens.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <charconv>
#include <cstdio>
#include <cstring>
#include <string_view>

namespace dyx3_px4_link {
namespace {
bool write_all(int fd, const char* data, size_t size) {
  while (size > 0) {
    const ssize_t n = ::write(fd, data, size);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) return false;
    data += n;
    size -= static_cast<size_t>(n);
  }
  return true;
}
}  // namespace

SprayAckTokens::SprayAckTokens(std::string state_path) : state_path_(std::move(state_path)) {}

bool SprayAckTokens::load() {
  const int fd = ::open(state_path_.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (fd < 0) return false;
  char contents[16]{};
  const ssize_t n = ::read(fd, contents, sizeof(contents));
  char extra{};
  const ssize_t extra_n = ::read(fd, &extra, 1);
  const int close_result = ::close(fd);
  if (n <= 0 || n >= static_cast<ssize_t>(sizeof(contents)) || extra_n != 0 || close_result != 0)
    return false;
  std::string_view text(contents, static_cast<size_t>(n));
  if (!text.empty() && text.back() == '\n') text.remove_suffix(1);
  unsigned value = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || value < kFirst ||
      value > static_cast<unsigned>(kLast) + 1U)
    return false;
  next_ = static_cast<uint16_t>(value);
  return true;
}

bool SprayAckTokens::persist(uint16_t next) {
  const std::string temporary = state_path_ + ".tmp";
  const int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
                        S_IRUSR | S_IWUSR);
  if (fd < 0) return false;
  const std::string contents = std::to_string(next) + "\n";
  const bool written = write_all(fd, contents.data(), contents.size()) && ::fsync(fd) == 0;
  const int close_result = ::close(fd);
  if (!written || close_result != 0 || ::rename(temporary.c_str(), state_path_.c_str()) != 0) {
    ::unlink(temporary.c_str());
    return false;
  }
  const size_t slash = state_path_.find_last_of('/');
  const std::string parent = slash == std::string::npos ? "." : state_path_.substr(0, slash);
  const int dir_fd = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (dir_fd < 0) return false;
  const bool synced = ::fsync(dir_fd) == 0;
  ::close(dir_fd);
  return synced;
}

std::optional<uint16_t> SprayAckTokens::reserve() {
  if (failed_) return std::nullopt;
  if (!initialized_) {
    initialized_ = true;
    if (state_path_.empty() || !load()) {
      failed_ = true;
      return std::nullopt;
    }
  }
  if (next_ > kLast) return std::nullopt;
  const uint16_t token = next_;
  const uint16_t advanced = static_cast<uint16_t>(token + 1U);
  if (!persist(advanced)) {
    failed_ = true;
    return std::nullopt;
  }
  next_ = advanced;
  return token;
}

}  // namespace dyx3_px4_link
