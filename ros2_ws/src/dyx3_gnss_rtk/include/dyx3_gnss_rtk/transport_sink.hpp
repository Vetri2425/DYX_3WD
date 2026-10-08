#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include "dyx3_gnss_rtk/rtcm_transport.hpp"
#include "dyx3_gnss_rtk/validated_frame.hpp"

namespace dyx3_gnss_rtk {

struct SinkCounters {
  uint64_t frames_delivered{0};
  uint64_t bytes_delivered{0};
  uint64_t failures{0};
  uint64_t opens{0};
  uint64_t reopens{0};
  double last_delivery_s{-1};
};

class TransportSink {
public:
  virtual ~TransportSink() = default;
  virtual bool open(double now_s) = 0;
  virtual bool deliver(const ValidatedFrame& frame, double now_s) = 0;
  virtual void close() = 0;
  virtual bool active() const = 0;
  virtual SinkCounters counters() const = 0;
};

struct UsbConfig {
  std::string receiver_device;
  int baud{0};  // unset until the receiver COM's baud is confirmed on the bench
  double write_timeout_s{0};
  double reopen_delay_s{0};
};

struct ReceiverReadback {
  int fix_quality{0};
  int satellites{0};
  double hdop{0};
  std::optional<double> correction_age_s;
  double received_at_s{-1};
};

class UsbSerialSink final : public TransportSink {
public:
  explicit UsbSerialSink(UsbConfig config, std::string by_id_prefix = "/dev/serial/by-id/");
  ~UsbSerialSink() override { close(); }
  bool open(double now_s) override;
  bool deliver(const ValidatedFrame& frame, double now_s) override;
  void close() override;
  bool active() const override;
  SinkCounters counters() const override;

  // Read-only observation of receiver GGA on this same fd. No command is ever sent to enable it.
  void read_available(double now_s);
  std::string last_gga(double now_s, double max_age_s) const;
  std::optional<ReceiverReadback> readback(double now_s, double max_age_s) const;

private:
  void close_locked();

  UsbConfig config_;
  std::string by_id_prefix_;
  mutable std::mutex m_;
  int fd_{-1};
  bool ever_opened_{false};
  double next_open_s_{0};
  SinkCounters counters_;
  std::string line_buffer_;
  std::string gga_;
  double gga_at_s_{-1};
  ReceiverReadback readback_;
};

class DdsSink final : public TransportSink {
public:
  using Publish = std::function<bool(const Chunk&)>;
  using Ready = std::function<bool()>;
  DdsSink(Publish publish, Ready ready) : publish_(std::move(publish)), ready_(std::move(ready)) {}
  bool open(double now_s) override;
  bool deliver(const ValidatedFrame& frame, double now_s) override;
  void close() override { active_ = false; }
  bool active() const override { return active_; }
  SinkCounters counters() const override { return counters_; }

private:
  Publish publish_;
  Ready ready_;
  Chunker chunker_;
  bool active_{false};
  SinkCounters counters_;
};

}  // namespace dyx3_gnss_rtk
