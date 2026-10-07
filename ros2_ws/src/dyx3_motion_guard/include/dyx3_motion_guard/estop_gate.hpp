// estop_gate — latched emergency stop. See docs/contracts/dyx3_motion_guard.md section 4.
#pragma once

#include <string>

namespace dyx3_motion_guard {

class EstopLatch {
public:
  // Accepted sources: tablet, backend, ble, physical. Returns false (and changes nothing)
  // otherwise.
  bool request(bool asserted, const std::string& source);
  bool asserted() const { return asserted_; }
  const std::string& source() const { return source_; }

private:
  bool asserted_{
      false};  // DERIVED — NOT FROM V1 SPEC: boot state is clear (see contract section 4)
  std::string source_;
};

}  // namespace dyx3_motion_guard
