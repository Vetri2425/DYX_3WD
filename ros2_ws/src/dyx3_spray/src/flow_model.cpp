#include "dyx3_spray/flow_model.hpp"

namespace dyx3_spray {

FlowModulator::FlowModulator(double min_flow_value, double on_value, double rated_marking_speed_mps,
                             double max_slew_per_s)
    : min_flow_value_(min_flow_value),
      on_value_(on_value),
      rated_(std::max(1e-6, rated_marking_speed_mps)),
      max_slew_(std::max(0.0, max_slew_per_s)),
      lo_(std::min(min_flow_value, on_value)),
      hi_(std::max(min_flow_value, on_value)),
      last_(min_flow_value) {}

double FlowModulator::update(double speed_mps, double dt_s) {
  double frac = std::max(0.0, speed_mps) / rated_;
  frac = std::min(1.0, frac);  // saturate at the rated speed: full flow
  double target = min_flow_value_ + (on_value_ - min_flow_value_) * frac;
  target = std::max(lo_, std::min(hi_, target));
  dt_s = std::max(0.0, dt_s);
  if (max_slew_ > 0.0 && dt_s > 0.0) {
    const double max_step = max_slew_ * dt_s;
    double delta = target - last_;
    if (delta > max_step)
      delta = max_step;
    else if (delta < -max_step)
      delta = -max_step;
    last_ += delta;
  } else {
    last_ = target;
  }
  return last_;
}

}  // namespace dyx3_spray
