// angle_wrap — see docs/contracts/dyx3_geometry.md
#pragma once

namespace dyx3_geometry {

/// Wrap to [-pi, pi). Bit-compatible with the prototype's
/// `(a + pi) % (2*pi) - pi` including Python's floored-modulo semantics for floats
/// (ancestor: `_angle_wrap`). Not for NaN/inf (returns NaN).
double angle_wrap(double angle);

}  // namespace dyx3_geometry
