#include "dyx3_gnss_rtk/control_socket.hpp"

#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace dyx3_gnss_rtk {

void ControlSocket::start() {
  if (thread_.joinable()) return;
  sockaddr_un address{};
  if (path_.size() >= sizeof address.sun_path)
    throw std::runtime_error("RTK control socket path too long");
  listener_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (listener_ < 0) throw std::runtime_error("cannot create RTK control socket");
  struct stat st{};
  if (::lstat(path_.c_str(), &st) == 0) {
    if (!S_ISSOCK(st.st_mode) || ::unlink(path_.c_str()) != 0) {
      ::close(listener_);
      listener_ = -1;
      throw std::runtime_error("unsafe RTK control socket path");
    }
  } else if (errno != ENOENT) {
    ::close(listener_);
    listener_ = -1;
    throw std::runtime_error("cannot inspect RTK control socket path");
  }
  address.sun_family = AF_UNIX;
  std::strncpy(address.sun_path, path_.c_str(), sizeof address.sun_path - 1);
  if (::bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof address) != 0 ||
      ::chmod(path_.c_str(), 0660) != 0 || ::listen(listener_, 8) != 0) {
    ::close(listener_);
    listener_ = -1;
    ::unlink(path_.c_str());
    throw std::runtime_error("cannot bind RTK control socket");
  }
  bound_ = true;
  stopping_ = false;
  thread_ = std::thread([this] { run(); });
}

void ControlSocket::stop() {
  stopping_ = true;
  if (thread_.joinable()) thread_.join();
  if (listener_ >= 0) ::close(listener_);
  listener_ = -1;
  if (bound_) ::unlink(path_.c_str());
  bound_ = false;
}

void ControlSocket::run() {
  while (!stopping_) {
    pollfd p{listener_, POLLIN, 0};
    const int ready = ::poll(&p, 1, 100);
    if (ready < 0 && errno == EINTR) continue;
    if (ready <= 0) continue;
    const int fd = ::accept4(listener_, nullptr, nullptr, SOCK_CLOEXEC);
    if (fd < 0) continue;
    timeval timeout{3, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof timeout);
    serve_one(fd);
    ::close(fd);
  }
}

void ControlSocket::serve_one(int fd) {
  std::string line;
  char buffer[1024];
  while (line.size() <= 65536) {
    const ssize_t n = ::recv(fd, buffer, sizeof buffer, 0);
    if (n <= 0) return;
    line.append(buffer, static_cast<size_t>(n));
    const size_t newline = line.find('\n');
    if (newline != std::string::npos) {
      line.resize(newline);
      break;
    }
  }
  Json reply;
  try {
    if (line.size() > 65536) throw ConfigError("request too large");
    const Json request = Json::parse(line);
    if (!request.is_object() || request.value("v", 0) != 1 || !request.contains("cmd") ||
        !request.at("cmd").is_string())
      throw ConfigError("invalid protocol request");
    reply = handler_(request);
  } catch (const ConfigError& e) {
    reply = {{"v", 1}, {"ok", false}, {"code", "invalid_config"}, {"reason", e.what()}};
  } catch (const std::exception&) {
    // Do not echo untrusted input or parser diagnostics: a password may be present.
    reply = {{"v", 1}, {"ok", false}, {"code", "bad_request"}, {"reason", "invalid request"}};
  }
  const std::string bytes = reply.dump() + "\n";
  size_t sent = 0;
  while (sent < bytes.size()) {
    const ssize_t n = ::send(fd, bytes.data() + sent, bytes.size() - sent, MSG_NOSIGNAL);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) return;
    sent += static_cast<size_t>(n);
  }
}

}  // namespace dyx3_gnss_rtk
