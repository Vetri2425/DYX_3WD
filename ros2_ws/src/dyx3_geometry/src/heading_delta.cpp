// heading_delta — see docs/contracts/dyx3_geometry.md
#include "dyx3_geometry/heading_delta.hpp"

#include <cmath>

#include "dyx3_geometry/angle_wrap.hpp"

namespace dyx3_geometry {

double heading_delta(double h0, double h1) { return std::fabs(angle_wrap(h1 - h0)); }

}  // namespace dyx3_geometry
