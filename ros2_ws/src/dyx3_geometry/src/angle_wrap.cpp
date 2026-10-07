// angle_wrap — see docs/contracts/dyx3_geometry.md
#include "dyx3_geometry/angle_wrap.hpp"

#include <cmath>

namespace dyx3_geometry {

namespace {
constexpr double kPi = 3.14159265358979323846;  // == Python math.pi
constexpr double kTwoPi = 2.0 * kPi;

// CPython float_rem: result has the sign of the divisor (floored modulo).
double py_float_mod(double x, double y) {
  double mod = std::fmod(x, y);
  if (mod != 0.0) {
    if ((y < 0.0) != (mod < 0.0)) {
      mod += y;
    }
  } else {
    mod = std::copysign(0.0, y);
  }
  return mod;
}
}  // namespace

double angle_wrap(double angle) { return py_float_mod(angle + kPi, kTwoPi) - kPi; }

}  // namespace dyx3_geometry
