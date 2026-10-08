#include "dyx3_gnss_rtk/injection_authority.hpp"

namespace dyx3_gnss_rtk {

uint64_t InjectionAuthority::switch_to(CorrectionSource source, CorrectionTransport transport,
                                       double now_s) {
  std::lock_guard<std::mutex> lk(m_);
  // Generation changes before closing the old sink: any waiting old callback fails its check.
  ++generation_;
  if (selected_) selected_->close();
  selected_ = nullptr;
  source_ = source;
  transport_ = transport;
  TransportSink* next = transport == CorrectionTransport::UsbDirect ? &usb_ : &dds_;
  // A failed open leaves the selected sink available for retry on a fresh frame. The other
  // sink is already closed; there is no fallback to it.
  next->open(now_s);
  selected_ = next;
  return generation_;
}

void InjectionAuthority::stop() {
  std::lock_guard<std::mutex> lk(m_);
  ++generation_;
  if (selected_) selected_->close();
  selected_ = nullptr;
}

bool InjectionAuthority::deliver(uint64_t generation, const ValidatedFrame& frame, double now_s) {
  std::lock_guard<std::mutex> lk(m_);
  if (!selected_ || generation != generation_) return false;
  return selected_->deliver(frame, now_s);
}

uint64_t InjectionAuthority::generation() const {
  std::lock_guard<std::mutex> lk(m_);
  return generation_;
}

bool InjectionAuthority::active() const {
  std::lock_guard<std::mutex> lk(m_);
  return selected_ && selected_->active();
}

SinkCounters InjectionAuthority::selected_counters() const {
  std::lock_guard<std::mutex> lk(m_);
  return selected_ ? selected_->counters() : SinkCounters{};
}

CorrectionSource InjectionAuthority::source() const {
  std::lock_guard<std::mutex> lk(m_);
  return source_;
}

CorrectionTransport InjectionAuthority::transport() const {
  std::lock_guard<std::mutex> lk(m_);
  return transport_;
}

}  // namespace dyx3_gnss_rtk
