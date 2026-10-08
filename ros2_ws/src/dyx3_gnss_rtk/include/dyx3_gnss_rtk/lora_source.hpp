#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "dyx3_gnss_rtk/rtcm_parser.hpp"

namespace dyx3_gnss_rtk {

struct LoraConfig {
  std::string serial_device;
  int baud{0};  // unset until the radio's baud is confirmed on the bench
  double read_timeout_s{0};
  double reopen_delay_s{0};
};

struct LoraSnapshot {
  bool port_open{false};
  uint64_t bytes_received{0};
  uint64_t valid_frames{0};
  uint64_t crc_failures{0};
  uint64_t invalid_headers{0};
  uint64_t resync_bytes{0};
  uint16_t last_message_type{0};
  uint64_t partial_timeouts{0};
  uint64_t read_errors{0};
  uint64_t opens{0};
  uint64_t reopens{0};
};

class LoraSource {
public:
  using FrameSink = std::function<void(const std::vector<uint8_t>&)>;
  LoraSource(LoraConfig config, FrameSink sink, std::string by_id_prefix = "/dev/serial/by-id/");
  ~LoraSource();
  LoraSource(const LoraSource&) = delete;
  LoraSource& operator=(const LoraSource&) = delete;

  void start();
  void stop();
  LoraSnapshot snapshot() const;

private:
  void run();
  void close_fd();
  LoraConfig config_;
  FrameSink sink_;
  std::string by_id_prefix_;
  mutable std::mutex m_;
  LoraSnapshot snapshot_;
  std::thread worker_;
  std::atomic<bool> stopping_{false};
  int wake_[2]{-1, -1};
  int fd_{-1};  // owned by the worker; stop() only wakes it
};

}  // namespace dyx3_gnss_rtk
