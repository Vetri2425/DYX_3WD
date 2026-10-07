// flow_model — speed-proportional flow modulator (plan 7.5). Pure C++. Port of
// spray_flow_model.FlowModulator. It can only move the command WITHIN [min_flow, on_value]: never
// keeps spray on longer, never turns it on earlier, never overrides a gate.
#pragma once

#include <algorithm>

namespace dyx3_spray {

class FlowModulator {
public:
  FlowModulator(double min_flow_value, double on_value, double rated_marking_speed_mps,
                double max_slew_per_s);
  void reset() { last_ = min_flow_value_; }  // OFF->ON edge: re-seed the slew filter to the floor
  double update(double speed_mps, double dt_s);
  double value() const { return last_; }

private:
  double min_flow_value_, on_value_, rated_, max_slew_, lo_, hi_, last_;
};

}  // namespace dyx3_spray
