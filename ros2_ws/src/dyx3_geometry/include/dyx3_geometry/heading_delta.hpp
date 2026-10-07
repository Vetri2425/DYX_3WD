// heading_delta — see docs/contracts/dyx3_geometry.md
#pragma once

namespace dyx3_geometry {

/// |wrap(h1 - h0)| in [0, pi] (ancestor: `_heading_delta`).
double heading_delta(double h0, double h1);

}  // namespace dyx3_geometry
