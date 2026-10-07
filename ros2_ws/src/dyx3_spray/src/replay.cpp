#include "dyx3_spray/replay.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace dyx3_spray {

ReplayStats replay_drive_along(const PathModel& m, const ReplayConfig& cfg) {
  ReplayStats st;
  st.boundaries = m.boundaries.size();
  if (m.size() < 2 || m.cumulative_s.size() != m.size()) return st;
  st.path_length_m = m.cumulative_s.back();

  auto flag_at =
      [&](double s) {  // the planned flag at true arc length s: segment i is MARK iff both ends are
        size_t i = 0;
        while (i + 2 < m.size() && m.cumulative_s[i + 1] <= s) ++i;
        return static_cast<bool>(m.flags[i]);
      };
  auto near_boundary = [&](double s) {
    for (const auto& b : m.boundaries) {
      if (std::fabs(b.s - s) <= cfg.flag_guard_m) return true;
    }
    return false;
  };

  std::optional<double> prev_s;
  double s_true = 0.0;
  size_t seg = 0;
  const double len = st.path_length_m;
  for (; s_true < len; s_true += cfg.step_m) {
    while (seg + 2 < m.size() && m.cumulative_s[seg + 1] <= s_true) ++seg;
    const double seg_len = m.cumulative_s[seg + 1] - m.cumulative_s[seg];
    if (seg_len < 1e-9) continue;
    const double f = (s_true - m.cumulative_s[seg]) / seg_len;
    const double dn = m.north[seg + 1] - m.north[seg];
    const double de = m.east[seg + 1] - m.east[seg];
    const double heading = std::atan2(de, dn);
    // lateral: to the right of the direction of travel (NED: right of heading is +90 degrees)
    const double ln = -std::sin(heading), le = std::cos(heading);
    // (right-hand normal of (cos h, sin h) in the (n, e) plane is (-sin h, cos h))
    const double lat = cfg.lateral_offset_m + cfg.noise_amp_m * std::sin(s_true * 40.0);
    const double pn = m.north[seg] + f * dn + lat * ln;
    const double pe = m.east[seg] + f * de + lat * le;

    ProjectionConfig pc;
    pc.prev_s = prev_s;
    pc.window_back_m = cfg.window_back_m;
    pc.window_fwd_m = cfg.window_fwd_m;
    pc.reacquire_dist_m = cfg.reacquire_dist_m;
    pc.heading_rad = heading;
    pc.direction_gate_cos = direction_gate_cos(cfg.gate_deg);
    const auto p = project_onto_path(m, pn, pe, pc);
    if (!p) continue;
    ++st.samples;
    if (prev_s) {
      const double jump = std::fabs(p->s - *prev_s);
      st.max_jump_m = std::max(st.max_jump_m, jump);
      if (jump > cfg.teleport_m) ++st.teleports;
    }
    if (!near_boundary(s_true)) {
      const bool planned = flag_at(s_true);
      if (p->current_flag != planned) {
        ++st.wrong_flag_samples;
        if (p->current_flag) {
          ++st.spurious_mark_samples;
        } else {
          ++st.missed_mark_samples;
        }
      }
    }
    prev_s = p->s;
    st.final_s_error_m = p->s - s_true;
  }
  return st;
}

}  // namespace dyx3_spray
