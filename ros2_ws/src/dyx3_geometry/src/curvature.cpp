// curvature — see docs/contracts/dyx3_geometry.md
#include "dyx3_geometry/curvature.hpp"

#include <algorithm>
#include <cmath>

#include "dyx3_geometry/distance.hpp"

namespace dyx3_geometry {

double menger_curvature(Point a, Point b, Point c) {
  const double ab = std::hypot(b.n - a.n, b.e - a.e);
  const double bc = std::hypot(c.n - b.n, c.e - b.e);
  const double ca = std::hypot(a.n - c.n, a.e - c.e);
  if (ab < 1e-6 || bc < 1e-6 || ca < 1e-6) {
    return 0.0;
  }
  const double area2 = std::fabs((b.n - a.n) * (c.e - a.e) - (b.e - a.e) * (c.n - a.n));
  return (2.0 * area2) / (ab * bc * ca);
}

double curvature_at(PathView path, int idx, double baseline_m) {
  if (path.n < 3) {
    return 0.0;
  }
  const int n = static_cast<int>(path.n);
  const int i1 = std::max(0, std::min(idx, n - 1));
  int i0 = std::max(0, i1 - 1);
  int i2 = std::min(n - 1, i1 + 1);
  if (baseline_m > 0.0) {
    double d = 0.0;
    while (i0 > 0 && d < baseline_m) {
      const Point a = path[static_cast<std::size_t>(i0)];
      const Point b = path[static_cast<std::size_t>(i0) + 1];
      d += std::hypot(b.n - a.n, b.e - a.e);
      --i0;
    }
    d = 0.0;
    while (i2 < n - 1 && d < baseline_m) {
      const Point a = path[static_cast<std::size_t>(i2)];
      const Point b = path[static_cast<std::size_t>(i2) - 1];
      d += std::hypot(b.n - a.n, b.e - a.e);
      ++i2;
    }
  }
  if (i2 - i0 < 2) {
    return 0.0;
  }
  return menger_curvature(path[static_cast<std::size_t>(i0)], path[static_cast<std::size_t>(i1)],
                          path[static_cast<std::size_t>(i2)]);
}

namespace {

struct Sample {
  Point p;
  bool hit_end;
};

/// Streaming equivalent of the ancestor's `_walk_path_samples`: targets must be requested in
/// ascending order; state advances monotonically along the path.
class Walker {
public:
  Walker(PathView path, int seg_idx, Point foot) : path_(path), foot_(foot) {
    const int n = static_cast<int>(path.n);
    const int s = std::max(0, std::min(seg_idx, n - 2));
    const Point end =
        (s + 1 < n) ? path[static_cast<std::size_t>(s) + 1] : path[static_cast<std::size_t>(s)];
    prev_ = foot;
    next_ = end;
    i_ = s + 1;
  }

  Sample at(double target) {
    const int n = static_cast<int>(path_.n);
    if (n == 0) {
      return {foot_, true};
    }
    if (n == 1) {
      return {path_[0], true};
    }
    while (true) {
      const double seg_len = std::hypot(next_.n - prev_.n, next_.e - prev_.e);
      if (finished_) {
        return {path_[path_.n - 1], true};
      }
      if (arc_ + seg_len >= target) {
        const double remaining = target - arc_;
        double ratio = seg_len > 1e-9 ? remaining / seg_len : 1.0;
        ratio = ratio < 0.0 ? 0.0 : (ratio > 1.0 ? 1.0 : ratio);
        return {Point{prev_.n + ratio * (next_.n - prev_.n), prev_.e + ratio * (next_.e - prev_.e)},
                false};
      }
      arc_ += seg_len;
      ++i_;
      if (i_ >= n) {
        finished_ = true;
        continue;
      }
      prev_ = next_;
      next_ = path_[static_cast<std::size_t>(i_)];
    }
  }

private:
  PathView path_;
  Point foot_;
  Point prev_;
  Point next_;
  double arc_ = 0.0;
  int i_ = 0;
  bool finished_ = false;
};

}  // namespace

double max_preview_curvature(PathView path, int seg_idx, Point foot, double l_d, int n_previews) {
  if (n_previews <= 1 || l_d <= 0.0) {
    return 0.0;
  }
  if (path.n < 2) {
    return 0.0;  // ancestor: every sample is (foot|wp, hit_end) => the first preview breaks => 0
  }
  Walker walker(path, seg_idx, foot);
  const double half = 0.5 * l_d;
  double kappa_max = 0.0;
  for (int k = 1; k <= n_previews; ++k) {
    const double centre = static_cast<double>(k) * l_d;
    const Sample a = walker.at(std::max(0.05, centre - half));
    const Sample b = walker.at(centre);
    const Sample c = walker.at(centre + half);
    if (b.hit_end && c.hit_end) {
      break;
    }
    const double kab = std::hypot(b.p.n - a.p.n, b.p.e - a.p.e);
    const double kbc = std::hypot(c.p.n - b.p.n, c.p.e - b.p.e);
    const double kca = std::hypot(a.p.n - c.p.n, a.p.e - c.p.e);
    if (kab < 1e-6 || kbc < 1e-6 || kca < 1e-6) {
      continue;
    }
    const double area2 =
        std::fabs((b.p.n - a.p.n) * (c.p.e - a.p.e) - (b.p.e - a.p.e) * (c.p.n - a.p.n));
    const double kappa = (2.0 * area2) / (kab * kbc * kca);
    if (kappa > kappa_max) {
      kappa_max = kappa;
    }
  }
  return kappa_max;
}

}  // namespace dyx3_geometry
