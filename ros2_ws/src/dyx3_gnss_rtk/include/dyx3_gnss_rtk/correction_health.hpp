// correction_health — correction age, rate and FIX-transition monitoring. Contract:
// dyx3_gnss_rtk.md section 5. Pure C++. Times are seconds on a monotonic clock.
#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace dyx3_gnss_rtk {

class CorrectionHealth {
public:
  explicit CorrectionHealth(double rate_window_s = 10.0) : window_s_(rate_window_s) {}

  void on_frame(double now_s, size_t bytes);
  void on_connected(double now_s) { connected_since_s_ = now_s; }
  void on_disconnected() { connected_since_s_.reset(); }
  void reset_stream() {
    connected_since_s_.reset();
    have_frame_ = false;
    stamps_.clear();
  }

  // Seconds since the last CRC-valid frame; 1e9 when none ever arrived.
  double age_s(double now_s) const;
  // Fresh = at least one frame ever AND age <= limit AND the stream is connected.
  bool fresh(double now_s, double limit_s) const;
  // Frames per second over the sliding window; 0 until at least two frames are inside it.
  double rate_hz(double now_s) const;
  double connected_for_s(double now_s) const {
    return connected_since_s_ ? now_s - *connected_since_s_ : 0.0;
  }
  bool connected() const { return connected_since_s_.has_value(); }
  uint64_t frames() const { return frames_; }
  bool has_frame() const { return have_frame_; }
  uint64_t bytes() const { return bytes_; }

private:
  double window_s_;
  std::deque<double> stamps_;
  bool have_frame_{false};
  double last_frame_s_{0.0};
  std::optional<double> connected_since_s_;
  uint64_t frames_{0};
  uint64_t bytes_{0};
};

struct FixTransition {
  double t_s;
  uint8_t from;
  uint8_t to;
  double correction_age_s;
  bool corrections_fresh;
  bool suspicious;  // dropped from RTK while corrections were fresh: arriving but useless
};

class FixMonitor {
public:
  // Returns a transition when fix_type changed since the previous call.
  std::optional<FixTransition> update(double now_s, uint8_t fix_type, bool corrections_fresh,
                                      double correction_age_s);
  uint32_t transitions() const { return count_; }
  uint8_t fix_type() const { return fix_; }
  const std::vector<FixTransition>& log() const { return log_; }

private:
  bool have_{false};
  uint8_t fix_{0};
  uint32_t count_{0};
  std::vector<FixTransition> log_;
};

}  // namespace dyx3_gnss_rtk
