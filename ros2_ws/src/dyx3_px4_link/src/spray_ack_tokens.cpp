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

SprayAckTokens::SprayAckTokens(std::string state_path) : state_path_(std::move(state_path)) {
  failed_ = state_path_.empty() || !load();
}

bool SprayAckTokens::load() {
  const int fd = ::open(state_path_.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (fd < 0) return false;
  char contents[32]{};
  const ssize_t n = ::read(fd, contents, sizeof(contents));
  char extra{};
  const ssize_t extra_n = ::read(fd, &extra, 1);
  const int close_result = ::close(fd);
  if (n <= 0 || n >= static_cast<ssize_t>(sizeof(contents)) || extra_n != 0 || close_result != 0)
    return false;
  std::string_view text(contents, static_cast<size_t>(n));
  if (!text.empty() && text.back() == '\n') text.remove_suffix(1);
  const bool versioned = text.substr(0, 3) == "v2 ";
  if (versioned) text.remove_prefix(3);
  uint32_t value = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) return false;
  // The old ledger is a high-water component ID for system 1. Preserve every previously
  // allocated pair when upgrading; 1000 means the first pair on system 2 is next.
  if (!versioned) {
    if (value < kFirst || value > static_cast<uint32_t>(kLast) + 1U) return false;
    value -= kFirst;
  }
  if (value > kCapacity) return false;
  next_ = value;
  reserved_ = value;
  return true;
}

bool SprayAckTokens::persist(uint32_t next) {
  ++persists_;
  const std::string temporary = state_path_ + ".tmp";
  const int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
                        S_IRUSR | S_IWUSR);
  if (fd < 0) return false;
  const std::string contents = "v2 " + std::to_string(next) + "\n";
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

std::optional<SprayAckTokens::Identity> SprayAckTokens::reserve() {
  if (failed_) return std::nullopt;
  if (next_ == kCapacity) return std::nullopt;
  if (next_ == reserved_) {
    // Block exhausted: durably move the high-water mark before any pair of the new block is
    // handed out. This is the only disk write, once per kBlock transactions.
    const uint32_t block_end = kCapacity - next_ < kBlock ? kCapacity : next_ + kBlock;
    if (!persist(block_end)) {
      failed_ = true;
      return std::nullopt;
    }
    reserved_ = block_end;
  }
  const uint32_t index = next_++;
  return Identity{static_cast<uint8_t>(index / (kLast - kFirst + 1U) + 1U),
                  static_cast<uint16_t>(index % (kLast - kFirst + 1U) + kFirst)};
}

}  // namespace dyx3_px4_link
