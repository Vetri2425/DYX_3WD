// dds_session — per-topic staleness and session liveness. See docs/contracts/dyx3_px4_link.md
// section 5 (upstream PX4 #27388: one topic can stop while the session stays up). Pure C++.
#pragma once

#include <array>
#include <cstdint>

namespace dyx3_px4_link {

// Bit assignment is frozen with dyx3_interfaces/Px4LinkStatus.stale_topics_mask.
enum TopicBit : int {
  kTimesync = 0,
  kLocalPosition = 1,
  kVehicleStatus = 2,
  kAttitude = 3,
  kEstimatorFlags = 4,
  kGps = 5,
  kTopicCount = 6
};

struct StalenessLimits {
  // DERIVED — NOT FROM V1 SPEC: see the contract table; re-validate at GATE 4 from recorded
  // periods.
  std::array<double, kTopicCount> max_age_s{1.0, 0.2, 1.0, 0.2, 1.0, 1.0};
};

struct StalenessReport {
  uint32_t mask{0};
  double worst_age_s{0.0};  // largest age among topics that have been seen; 0 if none
  bool session_alive{false};
  bool any_seen{false};
};

class StalenessMonitor {
public:
  explicit StalenessMonitor(const StalenessLimits& l) : limits_(l) {}

  void on_sample(TopicBit t, double now_s) {
    last_[t] = now_s;
    seen_[t] = true;
  }

  // A topic never received is stale. Session alive == timesync fresh.
  // Counts session resets (alive -> dead -> alive); a reset is reported once by consume_reset().
  StalenessReport evaluate(double now_s);

  uint32_t session_resets() const { return resets_; }

  // True exactly once after each completed alive->dead->alive transition (handshake re-arm).
  bool consume_reset() {
    const bool r = reset_flag_;
    reset_flag_ = false;
    return r;
  }

private:
  StalenessLimits limits_;
  std::array<double, kTopicCount> last_{};
  std::array<bool, kTopicCount> seen_{};
  bool was_alive_{false};
  bool had_dead_after_alive_{false};
  bool reset_flag_{false};
  uint32_t resets_{0};
};

}  // namespace dyx3_px4_link
