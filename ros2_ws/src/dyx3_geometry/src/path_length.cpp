// path_length / cumulative_lengths — see docs/contracts/dyx3_geometry.md
#include "dyx3_geometry/path_length.hpp"

#include <cmath>

namespace dyx3_geometry {

double path_length(PathView path) {
  double total = 0.0;
  for (std::size_t i = 1; i < path.n; ++i) {
    total += std::hypot(path[i].n - path[i - 1].n, path[i].e - path[i - 1].e);
  }
  return total;
}

std::size_t cumulative_lengths(PathView path, double* out, std::size_t out_capacity) {
  const std::size_t needed = path.n == 0 ? 1 : path.n;
  if (out == nullptr || out_capacity < needed) {
    return 0;
  }
  out[0] = 0.0;
  for (std::size_t i = 1; i < path.n; ++i) {
    out[i] = out[i - 1] + std::hypot(path[i].n - path[i - 1].n, path[i].e - path[i - 1].e);
  }
  return needed;
}

std::vector<double> cumulative_lengths(PathView path) {
  std::vector<double> out(path.n == 0 ? 1 : path.n);
  cumulative_lengths(path, out.data(), out.size());
  return out;
}

}  // namespace dyx3_geometry
