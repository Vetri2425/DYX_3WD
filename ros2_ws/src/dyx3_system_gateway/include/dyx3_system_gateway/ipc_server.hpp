// ipc_server — Unix-domain-socket NDJSON server on its own thread. Contract:
// docs/contracts/dyx3_system_gateway.md section 1. POSIX/std only. Complete lines are handed to the
// callback ON THE IPC THREAD; send()/broadcast() are thread safe.
#pragma once

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

  ~IpcServer() { stop(); }
  bool start(const Config& cfg, OnLine on_line, std::string* err);
  void stop();
  void send(int client, const std::string& line);  // appends '\n'
  void broadcast(const std::string& line);
  int clients() const { return clients_.load(); }
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
  Config cfg_;
  OnLine on_line_;
  int listen_fd_{-1};
  int wake_[2]{-1, -1};
  std::thread th_;
  std::atomic<bool> run_{false};
  std::atomic<int> clients_{0};
  std::atomic<uint64_t> dropped_slow_{0}, rejected_full_{0}, overflows_{0};
  std::mutex mu_;  // guards clients_map_ (the out buffers and drop flags)
  std::map<int, Client> clients_map_;
  int next_id_{1};
};

}  // namespace dyx3_gateway
