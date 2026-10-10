#include "dyx3_rpp/path_conditioner.hpp"

#include <algorithm>
#include <cmath>

#include "dyx3_geometry/angle_wrap.hpp"
#include "dyx3_geometry/heading_delta.hpp"
#include "dyx3_geometry/line_intersection.hpp"
#include "dyx3_geometry/perpendicular_distance.hpp"
#include "dyx3_geometry/resample.hpp"
#include "dyx3_geometry/segment_heading.hpp"

namespace dyx3_rpp {

namespace {
constexpr double kPi = 3.14159265358979323846;
double radians(double deg) { return deg * (kPi / 180.0); }  // CPython: x * (pi / 180)
double degrees(double rad) { return rad * (180.0 / kPi); }
double seg_heading(Point a, Point b) { return dyx3_geometry::segment_heading(a, b); }
double hdelta(double h0, double h1) { return dyx3_geometry::heading_delta(h0, h1); }
bool any_flag(const Flags& f) {
  for (unsigned char c : f) {
    if (c != 0) return true;
  }
  return false;
}
bool same(Point a, Point b) { return a.n == b.n && a.e == b.e; }

// True when any raw vertex in [lo, hi] (inclusive) is must-hit. The absorb erases that whole raw
// range, which holds b, c and any collinear vertex simplify_with_indices dropped between them.
bool range_has_must_hit(const std::vector<Point>& pts, int lo, int hi, const KeySet* must_hit) {
  if (must_hit == nullptr || must_hit->empty()) return false;
  for (int k = lo; k <= hi; ++k) {
    if (must_hit->count(pt_key(pts[static_cast<size_t>(k)])) > 0) return true;
  }
  return false;
}

void dp_mark_keep(const std::vector<Point>& pts, double eps, int lo, int hi,
                  std::vector<unsigned char>& keep) {
  std::vector<std::pair<int, int>> stack{{lo, hi}};
  while (!stack.empty()) {
    const auto [a_i, b_i] = stack.back();
    stack.pop_back();
    if (b_i <= a_i + 1) continue;
    const Point a = pts[static_cast<size_t>(a_i)];
    const Point b = pts[static_cast<size_t>(b_i)];
    double d_max = 0.0;
    int i_max = -1;
    for (int i = a_i + 1; i < b_i; ++i) {
      const double d = dyx3_geometry::perpendicular_distance(pts[static_cast<size_t>(i)], a, b);
      if (d > d_max) {
        d_max = d;
        i_max = i;
      }
    }
    if (i_max >= 0 && d_max > eps) {
      keep[static_cast<size_t>(i_max)] = 1;
      stack.emplace_back(a_i, i_max);
      stack.emplace_back(i_max, b_i);
    }
  }
}
}  // namespace

const char* to_string(Profile p) { return p == Profile::Segment ? "segment" : "smooth"; }

PtKey pt_key(Point p) {
  return {static_cast<int64_t>(std::nearbyint(p.n * 1000.0)),
          static_cast<int64_t>(std::nearbyint(p.e * 1000.0))};
}

std::string normalize_tracking_profile(const std::string& value) {
  size_t b = 0, e = value.size();
  while (b < e && std::isspace(static_cast<unsigned char>(value[b]))) ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(value[e - 1]))) --e;
  std::string s = value.substr(b, e - b);
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (s == "sharp") return "segment";
  if (s == "auto" || s == "segment" || s == "smooth") return s;
  return "auto";
}

PointRun simplify_for_profile(const std::vector<Point>& pts, const Flags* flags_in,
                              double collinear_tol_deg, double max_offset_m,
                              const KeySet* must_hit) {
  Flags flags;
  if (flags_in == nullptr || flags_in->size() != pts.size()) {
    flags.assign(pts.size(), 0);
  } else {
    flags = *flags_in;
  }
  PointRun out;
  if (pts.empty()) return out;
  std::vector<Point> cp{pts[0]};
  Flags cf{static_cast<unsigned char>(flags[0] != 0)};
  for (size_t i = 1; i < pts.size(); ++i) {
    if (std::hypot(pts[i].n - cp.back().n, pts[i].e - cp.back().e) < 1e-6) {
      cf.back() = static_cast<unsigned char>(cf.back() != 0 || flags[i] != 0);
      continue;
    }
    cp.push_back(pts[i]);
    cf.push_back(static_cast<unsigned char>(flags[i] != 0));
  }
  const size_t n = cp.size();
  if (n < 3) {
    out.pts = cp;
    out.flags = cf;
    return out;
  }
  const double tol = radians(collinear_tol_deg);
  Flags keep(n, 0);
  keep[0] = 1;
  keep[n - 1] = 1;
  size_t last_kept = 0;
  for (size_t i = 1; i + 1 < n; ++i) {
    const Point& this_pt = cp[i];
    const double h0 = seg_heading(cp[last_kept], this_pt);
    const double h1 = seg_heading(this_pt, cp[i + 1]);
    const double heading_change = hdelta(h0, h1);
    const bool flag_boundary = cf[i - 1] != cf[i] || cf[i] != cf[i + 1];
    const bool must = must_hit != nullptr && must_hit->count(pt_key(this_pt)) > 0;
    if (heading_change <= tol && !flag_boundary && !must) continue;
    keep[i] = 1;
    last_kept = i;
  }
  if (max_offset_m > 0.0) {
    std::vector<int> anchors;
    for (size_t i = 0; i < n; ++i) {
      if (keep[i] != 0) anchors.push_back(static_cast<int>(i));
    }
    for (size_t k = 0; k + 1 < anchors.size(); ++k)
      dp_mark_keep(cp, max_offset_m, anchors[k], anchors[k + 1], keep);
  }
  for (size_t i = 0; i < n; ++i) {
    if (keep[i] != 0) {
      out.pts.push_back(cp[i]);
      out.flags.push_back(cf[i]);
    }
  }
  return out;
}

Profile classify_auto_profile(const std::vector<Point>& pts, double threshold_deg) {
  const PointRun simple = simplify_for_profile(pts, nullptr, 5.0);
  if (simple.pts.size() <= 2) return Profile::Segment;
  std::vector<double> headings;
  for (size_t i = 0; i + 1 < simple.pts.size(); ++i) {
    if (std::hypot(simple.pts[i + 1].n - simple.pts[i].n, simple.pts[i + 1].e - simple.pts[i].e) >
        1e-6) {
      headings.push_back(seg_heading(simple.pts[i], simple.pts[i + 1]));
    }
  }
  if (headings.size() <= 1) return Profile::Segment;
  const double threshold = radians(std::max(0.0, threshold_deg));
  std::vector<double> deltas;
  for (size_t i = 0; i + 1 < headings.size(); ++i)
    deltas.push_back(hdelta(headings[i], headings[i + 1]));
  if (deltas.empty()) return Profile::Segment;
  if (*std::max_element(deltas.begin(), deltas.end()) >= threshold) return Profile::Segment;
  // Smooth requires SUSTAINED turning: >= 3 turning vertices (> 2 deg each) summing > 20 deg.
  size_t turning_n = 0;
  double turning_sum = 0.0;
  const double two = radians(2.0);
  for (double d : deltas) {
    if (d > two) {
      ++turning_n;
      turning_sum += d;
    }
  }
  if (turning_n >= 3 && turning_sum > radians(20.0)) return Profile::Smooth;
  return Profile::Segment;
}

double pts_length(const std::vector<Point>& pts) {
  double s = 0.0;
  for (size_t i = 0; i + 1 < pts.size(); ++i)
    s += std::hypot(pts[i + 1].n - pts[i].n, pts[i + 1].e - pts[i].e);
  return s;
}

std::vector<double> pts_cumulative_lengths(const std::vector<Point>& pts) {
  std::vector<double> out{0.0};
  for (size_t i = 0; i + 1 < pts.size(); ++i)
    out.push_back(out.back() + std::hypot(pts[i + 1].n - pts[i].n, pts[i + 1].e - pts[i].e));
  return out;
}

std::vector<PointRun> split_runs_by_flag(const std::vector<Point>& pts, const Flags& flags) {
  std::vector<PointRun> groups;
  size_t start = 0;
  for (size_t i = 1; i < pts.size(); ++i) {
    if (flags[i] != flags[i - 1]) {
      groups.push_back(
          {std::vector<Point>(pts.begin() + static_cast<long>(start),
                              pts.begin() + static_cast<long>(i)),
           Flags(flags.begin() + static_cast<long>(start), flags.begin() + static_cast<long>(i))});
      start = i;
    }
  }
  groups.push_back({std::vector<Point>(pts.begin() + static_cast<long>(start), pts.end()),
                    Flags(flags.begin() + static_cast<long>(start), flags.end())});
  std::vector<PointRun> out;
  for (size_t k = 0; k < groups.size(); ++k) {
    PointRun r = groups[k];
    if (k > 0) {
      r.pts.insert(r.pts.begin(), groups[k - 1].pts.back());
      r.flags.insert(r.flags.begin(), r.flags.front());
    }
    out.push_back(std::move(r));
  }
  return out;
}

bool runs_collinear(const std::vector<Point>& prev, const std::vector<Point>& next,
                    double threshold_deg) {
  if (prev.empty() || next.empty()) return false;
  const double gap = std::hypot(next[0].n - prev.back().n, next[0].e - prev.back().e);
  if (gap > 0.05) return false;
  const PointRun sp = simplify_for_profile(prev, nullptr);
  const PointRun sn = simplify_for_profile(next, nullptr);
  if (sp.pts.size() < 2 || sn.pts.size() < 2) return false;
  const double h0 = seg_heading(sp.pts[sp.pts.size() - 2], sp.pts.back());
  const double h1 = seg_heading(sn.pts[0], sn.pts[1]);
  return degrees(hdelta(h0, h1)) < threshold_deg;
}

bool is_short_transit_run(const std::vector<Point>& pts, const Flags& flags, double max_len_m) {
  return max_len_m > 0.0 && !flags.empty() && !any_flag(flags) && pts_length(pts) <= max_len_m;
}

std::vector<PointRun> merge_collinear_runs(const std::vector<PointRun>& runs, double threshold_deg,
                                           double transit_merge_max_len_m) {
  if (runs.size() < 2) return runs;
  std::vector<PointRun> merged{runs[0]};
  for (size_t k = 1; k < runs.size(); ++k) {
    const PointRun& cur = runs[k];
    PointRun& prev = merged.back();
    const bool same_profile = classify_auto_profile(prev.pts, threshold_deg) ==
                              classify_auto_profile(cur.pts, threshold_deg);
    const bool profile_ok = same_profile ||
                            is_short_transit_run(prev.pts, prev.flags, transit_merge_max_len_m) ||
                            is_short_transit_run(cur.pts, cur.flags, transit_merge_max_len_m);
    if (profile_ok && runs_collinear(prev.pts, cur.pts, threshold_deg)) {
      size_t start = 0;
      if (std::hypot(cur.pts[0].n - prev.pts.back().n, cur.pts[0].e - prev.pts.back().e) < 1e-6)
        start = 1;
      prev.pts.insert(prev.pts.end(), cur.pts.begin() + static_cast<long>(start), cur.pts.end());
      prev.flags.insert(prev.flags.end(), cur.flags.begin() + static_cast<long>(start),
                        cur.flags.end());
    } else {
      merged.push_back(cur);
    }
  }
  return merged;
}

SimplifiedIdx simplify_with_indices(const std::vector<Point>& pts, const Flags& flags,
                                    double collinear_tol_deg) {
  SimplifiedIdx out;
  if (pts.empty()) return out;
  std::vector<Point> cp{pts[0]};
  Flags cf{static_cast<unsigned char>(flags[0] != 0)};
  std::vector<int> ci{0};
  for (size_t j = 1; j < pts.size(); ++j) {
    if (std::hypot(pts[j].n - cp.back().n, pts[j].e - cp.back().e) < 1e-6) {
      cf.back() = static_cast<unsigned char>(cf.back() != 0 || flags[j] != 0);
      continue;
    }
    cp.push_back(pts[j]);
    cf.push_back(static_cast<unsigned char>(flags[j] != 0));
    ci.push_back(static_cast<int>(j));
  }
  if (cp.size() < 3) {
    out.pts = cp;
    out.flags = cf;
    out.idx = ci;
    return out;
  }
  const double tol = radians(collinear_tol_deg);
  out.pts.push_back(cp[0]);
  out.flags.push_back(cf[0]);
  out.idx.push_back(ci[0]);
  for (size_t i = 1; i + 1 < cp.size(); ++i) {
    const double h0 = seg_heading(out.pts.back(), cp[i]);
    const double h1 = seg_heading(cp[i], cp[i + 1]);
    const double heading_change = hdelta(h0, h1);
    const bool flag_boundary = cf[i - 1] != cf[i] || cf[i] != cf[i + 1];
    if (heading_change <= tol && !flag_boundary) continue;
    out.pts.push_back(cp[i]);
    out.flags.push_back(cf[i]);
    out.idx.push_back(ci[i]);
  }
  out.pts.push_back(cp.back());
  out.flags.push_back(cf.back());
  out.idx.push_back(ci.back());
  return out;
}

PointRun absorb_short_connectors(const std::vector<Point>& pts, const Flags& flags,
                                 double threshold_deg, double connector_absorb_m,
                                 double min_corner_deg, const KeySet* must_hit) {
  (void)threshold_deg;  // kept for signature parity with the ancestor (unused there as well)
  PointRun unchanged{pts, flags};
  if (connector_absorb_m <= 0.0 || pts.size() < 4) return unchanged;
  const SimplifiedIdx v = simplify_with_indices(pts, flags);
  if (v.pts.size() < 4) return unchanged;
  const double min_corner = radians(min_corner_deg);
  struct Action {
    int raw_a, raw_b;
    Point mid;
    unsigned char flag;
  };
  std::vector<Action> actions;
  long i = 1;
  while (i <= static_cast<long>(v.pts.size()) - 3) {
    const Point a = v.pts[static_cast<size_t>(i - 1)], b = v.pts[static_cast<size_t>(i)],
                c = v.pts[static_cast<size_t>(i + 1)], d = v.pts[static_cast<size_t>(i + 2)];
    const double seg_len = std::hypot(c.n - b.n, c.e - b.e);
    if (seg_len < connector_absorb_m) {
      const double bend_in = hdelta(seg_heading(a, b), seg_heading(b, c));
      const double bend_out = hdelta(seg_heading(b, c), seg_heading(c, d));
      if (bend_in >= min_corner && bend_out >= min_corner &&
          !range_has_must_hit(pts, v.idx[static_cast<size_t>(i)], v.idx[static_cast<size_t>(i + 1)],
                              must_hit)) {
        const Point mid{(b.n + c.n) * 0.5, (b.e + c.e) * 0.5};
        const auto apex = dyx3_geometry::line_intersection(a, b, c, d);
        Point merge = mid;
        if (apex.has_value() &&
            std::hypot(apex->n - mid.n, apex->e - mid.e) <= std::max(0.5, 5.0 * seg_len))
          merge = *apex;
        actions.push_back({v.idx[static_cast<size_t>(i)], v.idx[static_cast<size_t>(i + 1)], merge,
                           v.flags[static_cast<size_t>(i)]});
        i += 2;
        continue;
      }
    }
    ++i;
  }
  if (actions.empty()) return unchanged;
  PointRun out{pts, flags};
  for (auto it = actions.rbegin(); it != actions.rend(); ++it) {
    out.pts.erase(out.pts.begin() + it->raw_a, out.pts.begin() + it->raw_b + 1);
    out.flags.erase(out.flags.begin() + it->raw_a, out.flags.begin() + it->raw_b + 1);
    out.pts.insert(out.pts.begin() + it->raw_a, it->mid);
    out.flags.insert(out.flags.begin() + it->raw_a, it->flag);
  }
  return out;
}

std::vector<PointRun> split_run_at_corners(const std::vector<Point>& pts, const Flags& flags,
                                           double threshold_deg) {
  const PointRun simple = simplify_for_profile(pts, nullptr);
  if (simple.pts.size() < 3) return {{pts, flags}};
  std::vector<Point> corners;
  for (size_t i = 1; i + 1 < simple.pts.size(); ++i) {
    const double h0 = seg_heading(simple.pts[i - 1], simple.pts[i]);
    const double h1 = seg_heading(simple.pts[i], simple.pts[i + 1]);
    if (degrees(hdelta(h0, h1)) >= threshold_deg) corners.push_back(simple.pts[i]);
  }
  if (corners.empty()) return {{pts, flags}};
  std::vector<size_t> splits;
  size_t ci = 0;
  for (size_t idx = 0; idx < pts.size(); ++idx) {
    if (ci < corners.size() && same(pts[idx], corners[ci])) {
      splits.push_back(idx);
      ++ci;
    }
  }
  std::vector<PointRun> out;
  size_t start = 0;
  for (size_t s : splits) {
    if (s > start) {
      out.push_back({std::vector<Point>(pts.begin() + static_cast<long>(start),
                                        pts.begin() + static_cast<long>(s) + 1),
                     Flags(flags.begin() + static_cast<long>(start),
                           flags.begin() + static_cast<long>(s) + 1)});
      start = s;
    }
  }
  if (start + 1 < pts.size()) {
    out.push_back({std::vector<Point>(pts.begin() + static_cast<long>(start), pts.end()),
                   Flags(flags.begin() + static_cast<long>(start), flags.end())});
  }
  if (out.empty()) return {{pts, flags}};
  return out;
}

PointRun smooth_corners(const std::vector<Point>& pts, double radius, int arc_pts,
                        const Flags* flags_in, int* skipped_out, const KeySet* must_hit) {
  Flags flags;
  if (flags_in == nullptr || flags_in->size() != pts.size()) {
    flags.assign(pts.size(), 0);
  } else {
    flags = *flags_in;
  }
  const size_t n = pts.size();
  if (skipped_out != nullptr) *skipped_out = 0;
  if (n < 3 || radius <= 0.0) return {pts, flags};
  PointRun out;
  out.pts.push_back(pts[0]);
  out.flags.push_back(static_cast<unsigned char>(flags[0] != 0));
  int skipped = 0;
  for (size_t i = 1; i + 1 < n; ++i) {
    const double ax = pts[i - 1].n, ay = pts[i - 1].e;
    const double px = pts[i].n, py = pts[i].e;
    const double bx = pts[i + 1].n, by = pts[i + 1].e;
    const double v1n = ax - px, v1e = ay - py;
    const double v2n = bx - px, v2e = by - py;
    const double l1 = std::hypot(v1n, v1e);
    const double l2 = std::hypot(v2n, v2e);
    // A must-hit vertex is never replaced by an arc: it is a survey/CAD vertex the rover must
    // reach, so it stays sharp (exactly like the d > 0.45*min(l1, l2) case below).
    const bool must = must_hit != nullptr && must_hit->count(pt_key(pts[i])) > 0;
    if (must) {
      out.pts.push_back(pts[i]);
      out.flags.push_back(static_cast<unsigned char>(flags[i] != 0));
      continue;
    }
    if (l1 < 1e-9 || l2 < 1e-9) continue;
    const double u1n = v1n / l1, u1e = v1e / l1;
    const double u2n = v2n / l2, u2e = v2e / l2;
    double dot = u1n * u2n + u1e * u2e;
    dot = std::max(-1.0, std::min(1.0, dot));
    const double theta = std::acos(dot);
    if (theta < 1e-3 || kPi - theta < 1e-3) {
      out.pts.push_back(pts[i]);
      out.flags.push_back(static_cast<unsigned char>(flags[i] != 0));
      continue;
    }
    const double d = radius / std::tan(theta / 2.0);
    if (d > 0.45 * std::min(l1, l2)) {
      ++skipped;
      out.pts.push_back(pts[i]);
      out.flags.push_back(static_cast<unsigned char>(flags[i] != 0));
      continue;
    }
    const double sa_n = px + d * u1n, sa_e = py + d * u1e;
    const double sb_n = px + d * u2n, sb_e = py + d * u2e;
    double bis_n = u1n + u2n, bis_e = u1e + u2e;
    const double bl = std::hypot(bis_n, bis_e);
    if (bl < 1e-9) {
      out.pts.push_back(pts[i]);
      out.flags.push_back(static_cast<unsigned char>(flags[i] != 0));
      continue;
    }
    bis_n /= bl;
    bis_e /= bl;
    const double pc = radius / std::sin(theta / 2.0);
    const double cx_n = px + pc * bis_n;
    const double cx_e = py + pc * bis_e;
    const double r1n = sa_n - cx_n, r1e = sa_e - cx_e;
    const double r2n = sb_n - cx_n, r2e = sb_e - cx_e;
    const double ang1 = std::atan2(r1e, r1n);
    const double ang2 = std::atan2(r2e, r2n);
    const double cross_z = r1n * r2e - r1e * r2n;
    double sweep = ang2 - ang1;
    if (cross_z >= 0) {
      if (sweep < 0) sweep += 2.0 * kPi;
    } else {
      if (sweep > 0) sweep -= 2.0 * kPi;
    }
    const unsigned char arc_flag =
        static_cast<unsigned char>(flags[i - 1] != 0 && flags[i] != 0 && flags[i + 1] != 0);
    out.pts.push_back({sa_n, sa_e});
    out.flags.push_back(arc_flag);
    for (int k = 1; k < arc_pts; ++k) {
      const double a = ang1 + sweep * (static_cast<double>(k) / static_cast<double>(arc_pts));
      out.pts.push_back({cx_n + radius * std::cos(a), cx_e + radius * std::sin(a)});
      out.flags.push_back(arc_flag);
    }
    out.pts.push_back({sb_n, sb_e});
    out.flags.push_back(arc_flag);
  }
  out.pts.push_back(pts.back());
  out.flags.push_back(static_cast<unsigned char>(flags.back() != 0));
  if (skipped_out != nullptr) *skipped_out = skipped;
  return out;
}

bool is_closed_run(const std::vector<Point>& pts, double threshold_m, double min_len_m) {
  if (pts.size() < 3) return false;
  if (pts_length(pts) < min_len_m) return false;
  return std::hypot(pts[0].n - pts.back().n, pts[0].e - pts.back().e) <= threshold_m;
}

std::vector<ConditionedRun> condition_path(const std::vector<RawPoint>& raw,
                                           const ConditionParams& p, KeySet* must_hit_keys_out,
                                           int* dropped_slivers) {
  std::vector<ConditionedRun> runs;
  if (dropped_slivers != nullptr) *dropped_slivers = 0;
  if (raw.empty()) return runs;
  std::vector<Point> raw_pts;
  Flags raw_flags;
  KeySet must;
  for (const RawPoint& r : raw) {
    const Point pt{r.n, r.e};
    raw_pts.push_back(pt);
    raw_flags.push_back(static_cast<unsigned char>((r.z & 1) != 0));
    if ((r.z & 2) != 0) must.insert(pt_key(pt));
  }
  if (must_hit_keys_out != nullptr) *must_hit_keys_out = must;
  const std::string requested = normalize_tracking_profile(p.tracking_profile);
  const double threshold = p.segment_corner_threshold_deg;

  std::vector<PointRun> raw_runs;
  if (requested == "auto") {
    for (const PointRun& run : split_runs_by_flag(raw_pts, raw_flags)) {
      const PointRun absorbed = absorb_short_connectors(
          run.pts, run.flags, threshold, p.connector_absorb_m, p.connector_min_corner_deg, &must);
      for (PointRun& sub : split_run_at_corners(absorbed.pts, absorbed.flags, threshold))
        raw_runs.push_back(std::move(sub));
    }
    raw_runs = merge_collinear_runs(raw_runs, threshold, p.transit_merge_max_len_m);
  } else {
    raw_runs.push_back({raw_pts, raw_flags});
  }

  for (const PointRun& rr : raw_runs) {
    const Profile profile = requested != "auto"
                                ? (requested == "segment" ? Profile::Segment : Profile::Smooth)
                                : classify_auto_profile(rr.pts, threshold);
    PointRun c;
    if (profile == Profile::Segment) {
      c = simplify_for_profile(rr.pts, &rr.flags, 5.0, p.segment_simplify_max_offset_m, &must);
    } else {
      c = rr;
      if (p.corner_smooth_radius_m > 0.0 && c.pts.size() >= 3) {
        c = smooth_corners(c.pts, p.corner_smooth_radius_m, std::max(2, p.corner_smooth_arc_pts),
                           &c.flags, nullptr, &must);
      }
      if (p.path_resample_spacing_m > 0.0 && c.pts.size() >= 2) {
        // Must-hit vertices are fixed knots: each span between them is resampled on its own.
        std::vector<size_t> fixed;
        for (size_t k = 0; k < c.pts.size(); ++k) {
          if (must.count(pt_key(c.pts[k])) > 0) fixed.push_back(k);
        }
        const auto rs =
            dyx3_geometry::resample_fixed(c.pts, p.path_resample_spacing_m, &c.flags, fixed);
        c.pts = rs.pts;
        c.flags = rs.flags;
      }
    }
    ConditionedRun cr;
    cr.pts = c.pts;
    cr.flags = c.flags;
    for (const Point& q : cr.pts)
      cr.must_hit.push_back(static_cast<unsigned char>(must.count(pt_key(q)) > 0));
    cr.profile = profile;
    cr.length = pts_length(cr.pts);
    cr.cum_s = pts_cumulative_lengths(cr.pts);
    cr.closed = is_closed_run(cr.pts, p.close_loop_threshold_m, p.close_loop_min_len_m);
    runs.push_back(std::move(cr));
  }
  if (runs.size() > 1) {
    std::vector<ConditionedRun> kept;
    for (auto& r : runs) {
      if (r.length >= 0.05) kept.push_back(r);
    }
    if (!kept.empty() && kept.size() < runs.size()) {
      if (dropped_slivers != nullptr)
        *dropped_slivers = static_cast<int>(runs.size() - kept.size());
      runs = std::move(kept);
    }
  }
  return runs;
}

}  // namespace dyx3_rpp
