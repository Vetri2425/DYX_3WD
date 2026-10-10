// ipc_server — Unix-domain-socket NDJSON server on its own thread. Contract:
// docs/contracts/dyx3_system_gateway.md section 1. POSIX/std only. Complete lines are handed to the
// callback ON THE IPC THREAD; send()/broadcast() are thread safe and never block on a peer: they
// only append to that peer's bounded out buffer (a peer past max_out_bytes is dropped), so a slow
// or stuck client can never stall the caller (the ROS executor) or another client.
// Path ownership: start() takes an exclusive flock on "<path>.lock" and refuses to start while
// another instance holds it; stop() unlinks the socket only if it is still the one this instance
// bound (same device and inode).
#pragma once

#include <sys/types.h>

#include <atomic>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>

namespace dyx3_gateway {

class IpcServer {
public:
  struct Config {
    std::string path;
    int max_clients{4};
    size_t max_line_bytes{65536};
    size_t max_out_bytes{1u << 20};
    unsigned mode{0660};
  };
  using OnLine = std::function<void(int client, const std::string& line)>;
  using OnOverflow =
      std::function<void(int client)>;  // line too long: the client is closed after this
  // A new client was accepted (called ON THE IPC THREAD, without the server lock, before any line
  // of that client is read); the handler may send() to it.
  using OnConnect = std::function<void(int client)>;

  ~IpcServer() { stop(); }
  bool start(const Config& cfg, OnLine on_line, std::string* err, OnConnect on_connect = nullptr);
  void stop();
  void send(int client, const std::string& line);  // appends '\n'
  void broadcast(const std::string& line);
  int clients() const { return clients_.load(); }
  // True while this client id is connected. Ids are never reused, so once false it stays false.
  bool connected(int client) const;
  uint64_t dropped_slow() const { return dropped_slow_.load(); }
  uint64_t rejected_full() const { return rejected_full_.load(); }
  uint64_t overflows() const { return overflows_.load(); }

private:
  struct Client {
    int fd;
    std::string in;
    std::string out;
    bool drop{false};
  };
  void loop();
  void release_path();
  Config cfg_;
  OnLine on_line_;
  OnConnect on_connect_;
  int listen_fd_{-1};
  int lock_fd_{-1};
  bool own_sock_{false};
  dev_t sock_dev_{0};
  ino_t sock_ino_{0};
  int wake_[2]{-1, -1};
  std::thread th_;
  std::atomic<bool> run_{false};
  std::atomic<int> clients_{0};
  std::atomic<uint64_t> dropped_slow_{0}, rejected_full_{0}, overflows_{0};
  mutable std::mutex mu_;  // guards clients_map_ (the out buffers and drop flags)
  std::map<int, Client> clients_map_;
  int next_id_{1};
};

}  // namespace dyx3_gateway
