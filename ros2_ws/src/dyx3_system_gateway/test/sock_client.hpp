// Test helpers: a blocking-ish Unix socket client for the gateway's NDJSON protocol.
#pragma once

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

namespace {
struct Sock {
  int fd{-1};
  explicit Sock(const std::string& path) {
    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    sockaddr_un a{};
    a.sun_family = AF_UNIX;
    std::strncpy(a.sun_path, path.c_str(), sizeof(a.sun_path) - 1);
    if (connect(fd, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0) {
      close(fd);
      fd = -1;
    }
  }
  ~Sock() {
    if (fd >= 0) close(fd);
  }
  bool ok() const { return fd >= 0; }
  void write_all(const std::string& s) const { (void)!write(fd, s.data(), s.size()); }
  // read until `want` newline-terminated lines or timeout
  std::vector<std::string> read_lines(size_t want, int timeout_ms = 2000) const {
    std::vector<std::string> out;
    std::string buf;
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (out.size() < want && std::chrono::steady_clock::now() < end) {
      pollfd p{fd, POLLIN, 0};
      if (poll(&p, 1, 50) > 0) {
        char b[4096];
        const ssize_t n = read(fd, b, sizeof b);
        if (n <= 0) break;
        buf.append(b, static_cast<size_t>(n));
        size_t nl;
        while ((nl = buf.find('\n')) != std::string::npos) {
          out.push_back(buf.substr(0, nl));
          buf.erase(0, nl + 1);
        }
      }
    }
    return out;
  }
  bool closed_by_peer(int timeout_ms = 1500) const {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < end) {
      pollfd p{fd, POLLIN, 0};
      if (poll(&p, 1, 50) > 0) {
        char b[64];
        const ssize_t n = read(fd, b, sizeof b);
        if (n == 0) return true;
        if (n < 0 && errno != EAGAIN) return true;
      }
    }
    return false;
  }
};
std::string tmp_sock() {
  return (std::filesystem::temp_directory_path() /
          ("dyx3_gw_" + std::to_string(getpid()) + "_" + std::to_string(rand()) + ".sock"))
      .string();
}
}  // namespace
