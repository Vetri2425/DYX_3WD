#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <thread>

#include "dyx3_gnss_rtk/rtk_config.hpp"

namespace dyx3_gnss_rtk {

// One newline-delimited JSON request and reply per connection. The worker is the only server;
// the backend is an authenticated HTTP facade, never the RTK process supervisor.
class ControlSocket {
public:
  using Handler = std::function<Json(const Json&)>;
  ControlSocket(std::string path, Handler handler)
      : path_(std::move(path)), handler_(std::move(handler)) {}
  ~ControlSocket() { stop(); }
  ControlSocket(const ControlSocket&) = delete;
  ControlSocket& operator=(const ControlSocket&) = delete;

  void start();
  void stop();

private:
  void run();
  void serve_one(int fd);
  std::string path_;
  Handler handler_;
  std::atomic<bool> stopping_{false};
  std::thread thread_;
  int listener_{-1};
  bool bound_{false};
};

}  // namespace dyx3_gnss_rtk
