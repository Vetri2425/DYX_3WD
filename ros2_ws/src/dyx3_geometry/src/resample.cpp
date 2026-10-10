// resample — see docs/contracts/dyx3_geometry.md
#include "dyx3_geometry/resample.hpp"

#include <algorithm>
#include <cmath>

#include "dyx3_geometry/path_length.hpp"

namespace dyx3_geometry {

ResampleResult resample(const std::vector<Point>& pts, double spacing,
                        const std::vector<unsigned char>* flags_in) {
  return resample_fixed(pts, spacing, flags_in, {});
}

ResampleResult resample_fixed(const std::vector<Point>& pts, double spacing,
                              const std::vector<unsigned char>* flags_in,
                              const std::vector<std::size_t>& fixed) {
  ResampleResult r;
  std::vector<unsigned char> flags;
  if (flags_in != nullptr && flags_in->size() == pts.size()) {
    flags = *flags_in;
  } else {
    flags.assign(pts.size(), 0);
  }

  if (pts.size() < 2 || !std::isfinite(spacing) || spacing <= 0.0) {
    r.pts = pts;
    r.flags = flags;
    return r;
  }

  const std::vector<double> cum = cumulative_lengths(PathView(pts));
  const double total = cum.back();
  if (!std::isfinite(total)) {  // NaN/inf coordinates: the sample count below would be undefined
    r.pts = pts;
    r.flags = flags;
    return r;
  }
  if (total < spacing && fixed.empty()) {
    r.pts = {pts.front(), pts.back()};
    r.flags = {flags.front(), flags.back()};
    return r;
  }

  // Knots: both endpoints plus every in-range fixed index, ascending and unique.
  const std::size_t last = pts.size() - 1;
  std::vector<std::size_t> knots{0};
  {
    std::vector<std::size_t> inner;
    for (const std::size_t k : fixed) {
      if (k > 0 && k < last) inner.push_back(k);
    }
    std::sort(inner.begin(), inner.end());
    inner.erase(std::unique(inner.begin(), inner.end()), inner.end());
    knots.insert(knots.end(), inner.begin(), inner.end());
    knots.push_back(last);
  }
  const std::size_t n_spans = knots.size() - 1;

  // Samples per span (both knots included). A coincident knot pair (zero-length span between two
  // fixed points) contributes nothing: the first of the pair is already emitted. A lone span keeps
  // the whole-path rule: total < spacing returns just its two endpoints (count 2 below).
  std::vector<long> counts(n_spans, 0);
  // Bound the count in double BEFORE any integer conversion (out-of-range float->int is UB), and
  // refuse rather than truncate: a silently shortened path would change its geometry.
  double n_wanted = 1.0;
  for (std::size_t s = 0; s < n_spans; ++s) {
    const double span = cum[knots[s + 1]] - cum[knots[s]];
    if (n_spans > 1 && span < 1e-12) continue;
    const double m = std::max(2.0, std::ceil(span / spacing) + 1.0);
    n_wanted += m - 1.0;
    if (!(n_wanted <= static_cast<double>(kResampleMaxSamples))) break;
    counts[s] = static_cast<long>(m);
  }
  if (!(n_wanted <= static_cast<double>(kResampleMaxSamples))) {
    r.pts = pts;
    r.flags = flags;
    return r;
  }
  r.pts.reserve(static_cast<std::size_t>(n_wanted));
  r.flags.reserve(static_cast<std::size_t>(n_wanted));
  r.pts.push_back(pts.front());
  r.flags.push_back(flags.front());
  for (std::size_t s = 0; s < n_spans; ++s) {
    const long m = counts[s];
    if (m == 0) {  // coincident knots: keep the path end exact
      if (knots[s + 1] == last) {
        r.pts.back() = pts[last];
        r.flags.back() = flags[last];
      }
      continue;
    }
    const std::size_t k0 = knots[s];
    const std::size_t k1 = knots[s + 1];
    const double span = cum[k1] - cum[k0];
    std::size_t seg = k0;
    for (long j = 1; j < m; ++j) {  // j = 0 is knot k0, emitted by the previous span (or above)
      if (j == m - 1) {
        r.pts.push_back(pts[k1]);  // knot k1 exactly
        r.flags.push_back(flags[k1]);
        break;
      }
      const double target = cum[k0] + (static_cast<double>(j) / static_cast<double>(m - 1)) * span;
      while (seg + 1 < k1 && cum[seg + 1] < target) {
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
      r.flags.push_back(static_cast<unsigned char>(flags[seg] != 0 && flags[seg + 1] != 0));
    }
  }
  return r;
}

}  // namespace dyx3_geometry
