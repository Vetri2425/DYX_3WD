#include "dyx3_motion_guard/estop_gate.hpp"

namespace dyx3_motion_guard {

bool EstopLatch::request(bool asserted, const std::string& source) {
  if (source != "tablet" && source != "backend" && source != "ble" && source != "physical")
    return false;
  asserted_ = asserted;
  source_ = asserted ? source : std::string();
  return true;
}

}  // namespace dyx3_motion_guard
