#include "dyx3_system_gateway/ipc_server.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <vector>

namespace dyx3_gateway {
namespace {
void set_nonblock(int fd) { fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK); }
}  // namespace

bool IpcServer::start(const Config& cfg, OnLine on_line, std::string* err) {
  stop();
  cfg_ = cfg;
  on_line_ = std::move(on_line);
  if (cfg.path.size() >= sizeof(sockaddr_un::sun_path)) {
    if (err) *err = "socket path too long";
    return false;
  }
  listen_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    if (err) *err = std::string("socket: ") + std::strerror(errno);
    return false;
  }
  unlink(cfg.path.c_str());  // a stale socket file from a crashed run is replaced
  sockaddr_un a{};
  a.sun_family = AF_UNIX;
  std::strncpy(a.sun_path, cfg.path.c_str(), sizeof(a.sun_path) - 1);
  const mode_t old = umask(0);
  const int br = bind(listen_fd_, reinterpret_cast<sockaddr*>(&a), sizeof a);
  umask(old);
  if (br != 0 || chmod(cfg.path.c_str(), cfg.mode) != 0 || listen(listen_fd_, 8) != 0) {
    if (err) *err = std::string("bind/listen ") + cfg.path + ": " + std::strerror(errno);
    close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  set_nonblock(listen_fd_);
  if (pipe(wake_) != 0) {
    close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  set_nonblock(wake_[0]);
  set_nonblock(wake_[1]);
  run_ = true;
  th_ = std::thread([this] { loop(); });
  return true;
}

void IpcServer::stop() {
  if (!run_.exchange(false)) {
    if (listen_fd_ >= 0) {
      close(listen_fd_);
      listen_fd_ = -1;
    }
    return;
  }
  const char c = 1;
  (void)!write(wake_[1], &c, 1);
  if (th_.joinable()) th_.join();
  std::lock_guard<std::mutex> lk(mu_);
  for (auto& kv : clients_map_) close(kv.second.fd);
  clients_map_.clear();
  clients_ = 0;
  close(listen_fd_);
  listen_fd_ = -1;
  close(wake_[0]);
  close(wake_[1]);
  wake_[0] = wake_[1] = -1;
  unlink(cfg_.path.c_str());
}

bool IpcServer::connected(int client) const {
  std::lock_guard<std::mutex> lk(mu_);
  return clients_map_.count(client) != 0;
}

void IpcServer::send(int client, const std::string& line) {
  {
    std::lock_guard<std::mutex> lk(mu_);
    const auto it = clients_map_.find(client);
    if (it == clients_map_.end()) return;
    if (it->second.out.size() + line.size() + 1 > cfg_.max_out_bytes) {
      it->second.drop = true;  // slow consumer
    } else {
      it->second.out += line;
      it->second.out += '\n';
    }
  }
  const char c = 1;
  (void)!write(wake_[1], &c, 1);
}

void IpcServer::broadcast(const std::string& line) {
  {
    std::lock_guard<std::mutex> lk(mu_);
    for (auto& kv : clients_map_) {
      if (kv.second.out.size() + line.size() + 1 > cfg_.max_out_bytes) {
        kv.second.drop = true;
      } else {
        kv.second.out += line;
        kv.second.out += '\n';
      }
    }
  }
  const char c = 1;
  (void)!write(wake_[1], &c, 1);
}

void IpcServer::loop() {
  std::vector<pollfd> fds;
  std::vector<int> ids;
  while (run_) {
    fds.clear();
    ids.clear();
    fds.push_back({listen_fd_, POLLIN, 0});
    fds.push_back({wake_[0], POLLIN, 0});
    {
      std::lock_guard<std::mutex> lk(mu_);
      for (auto& kv : clients_map_) {
        short ev = POLLIN;
        if (!kv.second.out.empty()) ev |= POLLOUT;
        fds.push_back({kv.second.fd, ev, 0});
        ids.push_back(kv.first);
      }
    }
    if (poll(fds.data(), fds.size(), 200) < 0 && errno != EINTR) break;
    if (fds[1].revents & POLLIN) {
      char buf[64];
      while (read(wake_[0], buf, sizeof buf) > 0) {
      }
    }
    if (fds[0].revents & POLLIN) {
      for (;;) {
        const int cfd = accept(listen_fd_, nullptr, nullptr);
        if (cfd < 0) break;
        std::lock_guard<std::mutex> lk(mu_);
        if (static_cast<int>(clients_map_.size()) >= cfg_.max_clients) {
          close(cfd);  // refused, not queued
          ++rejected_full_;
          continue;
        }
        set_nonblock(cfd);
        clients_map_[next_id_++] = Client{cfd, {}, {}, false};
        clients_ = static_cast<int>(clients_map_.size());
      }
    }
    std::vector<int> to_close;
    std::vector<std::pair<int, std::string>> lines;
    for (size_t k = 0; k < ids.size(); ++k) {
      const int id = ids[k];
      const short re = fds[k + 2].revents;
      std::lock_guard<std::mutex> lk(mu_);
      auto it = clients_map_.find(id);
      if (it == clients_map_.end()) continue;
      Client& c = it->second;
      if (c.drop) {
        ++dropped_slow_;
        to_close.push_back(id);
        continue;
      }
      if (re & (POLLERR | POLLNVAL)) {
        to_close.push_back(id);
        continue;
      }
      // A hung-up peer is never written to: its remaining input is still read below (a line it
      // sent just before closing, e.g. an E-stop, is delivered), then EOF closes it. send() with
      // MSG_NOSIGNAL turns a peer that closed between poll() and the write into EPIPE instead of a
      // process-killing SIGPIPE (GW-002).
      if ((re & POLLOUT) && !(re & POLLHUP)) {
        const ssize_t n = ::send(c.fd, c.out.data(), c.out.size(), MSG_NOSIGNAL);
        if (n > 0) {
          c.out.erase(0, static_cast<size_t>(n));
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          c.out.clear();  // EPIPE / ECONNRESET: the peer is gone
          to_close.push_back(id);
        }
      }
      if (re & (POLLIN | POLLHUP)) {
        char buf[4096];
        for (;;) {
          const ssize_t n = read(c.fd, buf, sizeof buf);
          if (n > 0) {
            c.in.append(buf, static_cast<size_t>(n));
            size_t nl;
            while ((nl = c.in.find('\n')) != std::string::npos) {
              lines.emplace_back(id, c.in.substr(0, nl));
              c.in.erase(0, nl + 1);
            }
            if (c.in.size() > cfg_.max_line_bytes) {  // an unterminated oversize line
              ++overflows_;
              to_close.push_back(id);
              break;
            }
            if (static_cast<size_t>(n) < sizeof buf) break;
          } else if (n == 0) {
            to_close.push_back(id);
            break;
          } else {
            if (errno != EAGAIN && errno != EINTR) to_close.push_back(id);
            break;
          }
        }
      }
    }
    // Callbacks run without the lock so a handler can call send().
    for (const auto& [id, line] : lines) {
      if (line.size() > cfg_.max_line_bytes) {
        ++overflows_;
        to_close.push_back(id);
        continue;
      }
      on_line_(id, line);
    }
    if (!to_close.empty()) {
      std::lock_guard<std::mutex> lk(mu_);
      for (const int id : to_close) {
        auto it = clients_map_.find(id);
        if (it == clients_map_.end()) continue;
        close(it->second.fd);
        clients_map_.erase(it);
      }
      clients_ = static_cast<int>(clients_map_.size());
    }
  }
}

}  // namespace dyx3_gateway
