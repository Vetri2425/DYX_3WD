// resample — see docs/contracts/dyx3_geometry.md
#include "dyx3_geometry/resample.hpp"

#include <algorithm>
#include <cmath>

#include "dyx3_geometry/path_length.hpp"

namespace dyx3_geometry {

ResampleResult resample(const std::vector<Point>& pts, double spacing,
                        const std::vector<unsigned char>* flags_in) {
  ResampleResult r;
  std::vector<unsigned char> flags;
  if (flags_in != nullptr && flags_in->size() == pts.size()) {
    flags = *flags_in;
  } else {
    flags.assign(pts.size(), 0);
  }

  if (pts.size() < 2 || spacing <= 0.0) {
    r.pts = pts;
    r.flags = flags;
    return r;
  }

  const std::vector<double> cum = cumulative_lengths(PathView(pts));
  const double total = cum.back();
  if (total < spacing) {
    r.pts = {pts.front(), pts.back()};
    r.flags = {flags.front(), flags.back()};
    return r;
  }

  const long n_samples = std::max(2L, static_cast<long>(std::ceil(total / spacing)) + 1);
  r.pts.reserve(static_cast<std::size_t>(n_samples));
  r.flags.reserve(static_cast<std::size_t>(n_samples));
  std::size_t seg = 0;
  for (long k = 0; k < n_samples; ++k) {
    const double target = (static_cast<double>(k) / static_cast<double>(n_samples - 1)) * total;
    while (seg + 1 < cum.size() - 1 && cum[seg + 1] < target) {
      ++seg;
    }
    const double seg_len = cum[seg + 1] - cum[seg];
    if (seg_len < 1e-12) {
      r.pts.push_back(pts[seg]);
      r.flags.push_back(flags[seg]);
      continue;
    }
    double t = (target - cum[seg]) / seg_len;
    t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
    const double n = pts[seg].n + t * (pts[seg + 1].n - pts[seg].n);
    const double e = pts[seg].e + t * (pts[seg + 1].e - pts[seg].e);
    r.pts.push_back(Point{n, e});
    if (k == 0) {
      r.flags.push_back(flags[0]);
    } else if (k == n_samples - 1) {
      r.flags.push_back(flags.back());
    } else {
      r.flags.push_back(static_cast<unsigned char>(flags[seg] != 0 && flags[seg + 1] != 0));
    }
  }
  r.pts.front() = pts.front();
  r.pts.back() = pts.back();
  r.flags.front() = flags.front();
  r.flags.back() = flags.back();
  return r;
}

}  // namespace dyx3_geometry
