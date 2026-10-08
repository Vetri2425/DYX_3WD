#pragma once

#include <cstdint>
#include <mutex>

#include "dyx3_gnss_rtk/transport_sink.hpp"

namespace dyx3_gnss_rtk {

enum class CorrectionSource { Ntrip, Lora };
enum class CorrectionTransport { UsbDirect, Px4Dds };

// Serializes delivery and reconfiguration. A source callback carries the generation it was
// started with, so a callback delayed across a switch cannot inject an old frame.
class InjectionAuthority {
public:
  InjectionAuthority(TransportSink& usb, TransportSink& dds) : usb_(usb), dds_(dds) {}

  uint64_t switch_to(CorrectionSource source, CorrectionTransport transport, double now_s);
  void stop();
  bool deliver(uint64_t generation, const ValidatedFrame& frame, double now_s);

  uint64_t generation() const;
  bool active() const;
  SinkCounters selected_counters() const;
  CorrectionSource source() const;
  CorrectionTransport transport() const;

private:
  mutable std::mutex m_;
  TransportSink& usb_;
  TransportSink& dds_;
  TransportSink* selected_{nullptr};
  CorrectionSource source_{CorrectionSource::Ntrip};
  CorrectionTransport transport_{CorrectionTransport::Px4Dds};
  uint64_t generation_{0};
};

}  // namespace dyx3_gnss_rtk
