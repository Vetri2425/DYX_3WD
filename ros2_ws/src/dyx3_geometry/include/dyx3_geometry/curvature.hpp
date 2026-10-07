// curvature — see docs/contracts/dyx3_geometry.md
#pragma once

#include "dyx3_geometry/point.hpp"

namespace dyx3_geometry {

/// Menger curvature (1/m) of the circle through a, b, c: 2*area2 / (|ab||bc||ca|), where area2
/// is the absolute cross product. 0 when any side < 1e-6 m. Unsigned.
double menger_curvature(Point a, Point b, Point c);

/// Path curvature at vertex `idx` (ancestor: `_path_curvature_at`).
///
/// `baseline_m` is the HALF-baseline: the neighbours are walked outward on ARC LENGTH (not
/// index) until each is at least that far from the centre vertex. 0 keeps the adjacent-vertex
/// form, which is NOT safe for anything quantitative on 4-10 cm spacing (3x error measured on
/// the 2026-07-30 curve bags; stable from ~0.10 m up). Returns 0 for n < 3.
double curvature_at(PathView path, int idx, double baseline_m = 0.0);

/// Worst |kappa| over `n_previews` look-ahead preview points (ancestor: `_max_preview_curvature`
/// + `_walk_path_samples`). Per preview k = 1..N three samples are taken along the path from the
/// projection foot at arc lengths (k-0.5)L, kL, (k+0.5)L (the first sample is never closer than
/// 0.05 m), and their Menger curvature is evaluated. Path-intrinsic: independent of the rover
/// pose. Stops when the middle and last samples both ran off the end. 0 when n_previews <= 1 or
/// l_d <= 0.
///
/// `seg_idx` is clamped to [0, n-2] (the ancestor would raise IndexError). No allocation: the
/// ancestor's target list is generated on the fly.
double max_preview_curvature(PathView path, int seg_idx, Point foot, double l_d, int n_previews);

}  // namespace dyx3_geometry
