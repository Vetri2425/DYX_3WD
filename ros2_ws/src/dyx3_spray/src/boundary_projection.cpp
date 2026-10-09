#include "dyx3_spray/boundary_projection.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace dyx3_spray {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

struct Scan {
  std::optional<Projection> best;
  double best_dist{kInf};
};

// Verbatim port of the nested _scan(). Ties at shared vertices prefer the later segment.
Scan scan(const PathModel& m, const std::vector<int>& indices, bool use_gate, double head_n,
          double head_e, double gate_cos, double pn, double pe) {
  Scan r;
  for (int i : indices) {
    const double a_n = m.north[i], a_e = m.east[i];
    if (m.run_ids.size() == m.size() && m.run_ids[i] != m.run_ids[i + 1]) continue;
    const double b_n = m.north[i + 1], b_e = m.east[i + 1];
    const double d_n = b_n - a_n, d_e = b_e - a_e;
    const double seg_len_sq = d_n * d_n + d_e * d_e;
    if (use_gate && seg_len_sq > 1e-12) {
      const double inv = 1.0 / std::sqrt(seg_len_sq);
      if ((d_n * inv) * head_n + (d_e * inv) * head_e < gate_cos) continue;
    }
    double t, proj_n, proj_e, seg_len;
    if (seg_len_sq <= 1e-12) {
      t = 0.0;
      proj_n = a_n;
      proj_e = a_e;
      seg_len = 0.0;
    } else {
      t = ((pn - a_n) * d_n + (pe - a_e) * d_e) / seg_len_sq;
      t = std::max(0.0, std::min(1.0, t));
      proj_n = a_n + t * d_n;
      proj_e = a_e + t * d_e;
      seg_len = std::sqrt(seg_len_sq);
    }
    const double dist = std::hypot(pn - proj_n, pe - proj_e);
    if (dist < r.best_dist - 1e-12 || std::fabs(dist - r.best_dist) <= 1e-12) {
      const bool flag = (t >= 1.0 - 1e-12) ? m.flags[i + 1] : m.flags[i];
      r.best_dist = dist;
      Projection p;
      p.segment_index = i;
      p.t = t;
      p.proj_n = proj_n;
      p.proj_e = proj_e;
      p.s = m.cumulative_s[i] + t * seg_len;
      p.xtrack_error_m = dist;
      p.current_flag = flag;
      r.best = p;
    }
  }
  return r;
}
}  // namespace

const char* to_string(LeadEvent e) {
  switch (e) {
    case LeadEvent::OnEarly:
      return "on_early";
    case LeadEvent::OffEarly:
      return "off_early";
    case LeadEvent::TerminalOff:
      return "terminal_off";
    default:
      return "";
  }
}

bool build_path_model(const std::vector<double>& north, const std::vector<double>& east,
                      const std::vector<bool>& flags, PathModel* out) {
  return build_path_model(north, east, flags, {}, out);
}

bool build_path_model(const std::vector<double>& north, const std::vector<double>& east,
                      const std::vector<bool>& flags, const std::vector<uint32_t>& run_ids,
                      PathModel* out) {
  if (north.size() != east.size() || north.size() != flags.size()) return false;
  if (!run_ids.empty() && run_ids.size() != north.size()) return false;
  PathModel m;
  m.north = north;
  m.east = east;
  m.flags = flags;
  m.run_ids = run_ids;
  double total = 0.0;
  m.cumulative_s.reserve(north.size());
  for (size_t i = 0; i < north.size(); ++i) {
    if (i > 0 && (run_ids.empty() || run_ids[i] == run_ids[i - 1]))
      total += std::hypot(north[i] - north[i - 1], east[i] - east[i - 1]);
    m.cumulative_s.push_back(total);
  }
  for (size_t i = 1; i < flags.size(); ++i) {
    if (!run_ids.empty() && run_ids[i] != run_ids[i - 1]) {
      // Run seams do not create a projection segment, but the boundary station remains meaningful
      // when the conditioner split exactly at a MARK/TRANSIT transition.
      if (flags[i - 1] != flags[i]) {
        m.boundaries.push_back(Boundary{m.cumulative_s[i], flags[i] ? BoundaryKind::TransitToMark
                                                                    : BoundaryKind::MarkToTransit});
      }
      continue;
    }
    if (flags[i - 1] == flags[i]) continue;
    m.boundaries.push_back(Boundary{
        m.cumulative_s[i], flags[i] ? BoundaryKind::TransitToMark : BoundaryKind::MarkToTransit});
  }
  // Hardening carried from the prototype: a path that ends on MARK still shuts off at its final
  // station.
  if (!flags.empty() && flags.back()) {
    m.boundaries.push_back(Boundary{m.cumulative_s.back(), BoundaryKind::MarkToTransit});
  }
  *out = std::move(m);
  return true;
}

double direction_gate_cos(double gate_deg) {
  if (gate_deg <= 0.0 || gate_deg >= 180.0) return 0.0;
  constexpr double kPi = 3.14159265358979323846;
  return std::max(1e-6, std::cos(gate_deg * (kPi / 180.0)));
}

std::optional<Projection> project_onto_path(const PathModel& m, double pn, double pe,
                                            const ProjectionConfig& cfg) {
  if (m.empty()) return std::nullopt;
  if (m.size() == 1) {
    Projection p;
    p.segment_index = 0;
    p.t = 0.0;
    p.proj_n = m.north[0];
    p.proj_e = m.east[0];
    p.s = 0.0;
    p.xtrack_error_m = std::hypot(pn - m.north[0], pe - m.east[0]);
    p.current_flag = m.flags[0];
    return p;
  }
  const bool gate_active = cfg.heading_rad.has_value() && cfg.direction_gate_cos > 0.0 &&
                           std::isfinite(*cfg.heading_rad);
  double head_n = 0.0, head_e = 0.0;
  if (gate_active) {
    head_n = std::cos(*cfg.heading_rad);
    head_e = std::sin(*cfg.heading_rad);
  }
  const int n_seg = static_cast<int>(m.size()) - 1;
  const auto trusted = [&](const Scan& s) {
    return s.best.has_value() &&
           (cfg.reacquire_dist_m <= 0.0 || s.best_dist <= cfg.reacquire_dist_m);
  };
  const bool windowed =
      cfg.prev_s.has_value() && (cfg.window_back_m > 0.0 || cfg.window_fwd_m > 0.0);
  if (windowed) {
    const double lo = *cfg.prev_s - std::max(0.0, cfg.window_back_m);
    const double hi = *cfg.prev_s + std::max(0.0, cfg.window_fwd_m);
    std::vector<int> idx;
    for (int i = 0; i < n_seg; ++i) {
      if (m.cumulative_s[i + 1] >= lo && m.cumulative_s[i] <= hi) idx.push_back(i);
    }
    if (!idx.empty()) {
      // Layered fallback: the gate is only ever a preference, never a trap.
      if (gate_active) {
        Scan s = scan(m, idx, true, head_n, head_e, cfg.direction_gate_cos, pn, pe);
        if (trusted(s)) return s.best;
      }
      Scan s = scan(m, idx, false, head_n, head_e, cfg.direction_gate_cos, pn, pe);
      if (trusted(s)) return s.best;
    }
  }
  std::vector<int> all(n_seg);
  for (int i = 0; i < n_seg; ++i) all[i] = i;
  if (gate_active) {
    Scan s = scan(m, all, true, head_n, head_e, cfg.direction_gate_cos, pn, pe);
    if (trusted(s)) return s.best;
  }
  return scan(m, all, false, head_n, head_e, cfg.direction_gate_cos, pn, pe).best;
}

void nozzle_position_ned(double pose_n, double pose_e, double yaw, double fwd, double lat,
                         double* n, double* e) {
  *n = pose_n + fwd * std::cos(yaw) - lat * std::sin(yaw);
  *e = pose_e + fwd * std::sin(yaw) + lat * std::cos(yaw);
}

std::optional<Boundary> next_boundary(const PathModel& m, double current_s, bool current_flag) {
  const BoundaryKind wanted =
      current_flag ? BoundaryKind::MarkToTransit : BoundaryKind::TransitToMark;
  for (const auto& b : m.boundaries) {
    if (b.kind == wanted && b.s > current_s + 1e-9) return b;
  }
  return std::nullopt;
}

bool apply_mark_boundary_lead(bool geometry_desired, std::optional<BoundaryKind> src_kind,
                              double src_dist, double on_lead, double off_lead, LeadEvent* event) {
  *event = LeadEvent::None;
  if (!src_kind.has_value() || !std::isfinite(src_dist)) return geometry_desired;
  if (!geometry_desired && *src_kind == BoundaryKind::TransitToMark && src_dist <= on_lead) {
    *event = LeadEvent::OnEarly;
    return true;
  }
  if (geometry_desired && *src_kind == BoundaryKind::MarkToTransit && src_dist <= off_lead) {
    *event = LeadEvent::OffEarly;
    return false;
  }
  return geometry_desired;
}

Decision make_decision(const DecisionInput& in, const DecisionParams& p, const DecisionState& st) {
  Decision d;
  d.safety_ok = in.safety_ok;
  d.safety_reason = in.safety_reason;
  d.distance_to_boundary_m = kInf;
  const PathModel* model = in.model;

  if (model != nullptr && in.nozzle_n && in.nozzle_e) {
    ProjectionConfig pc;
    pc.prev_s = st.prev_projection_s;
    pc.window_back_m = p.projection_window_back_m;
    pc.window_fwd_m = p.projection_window_fwd_m;
    pc.reacquire_dist_m = p.projection_reacquire_dist_m;
    pc.heading_rad = in.yaw_rad;
    pc.direction_gate_cos = p.projection_direction_gate_cos;
    d.projection = project_onto_path(*model, *in.nozzle_n, *in.nozzle_e, pc);
  }

  if (d.projection) {
    const Projection& pr = *d.projection;
    d.next_boundary = next_boundary(*model, pr.s, pr.current_flag);
    d.geometry_desired = pr.current_flag;

    // Hysteresis: trip at the wide level; once tripped, clear only when back under the tight level
    // AND the minimum off-dwell elapsed. A mission override IS the gate: the param trip band must
    // not widen it.
    const double trip_level = p.max_xtrack_from_mission
                                  ? p.max_xtrack_error_m
                                  : std::max(p.max_xtrack_error_m, p.xtrack_trip_error_m);
    bool tripped;
    if (st.xtrack_tripped) {
      tripped = pr.xtrack_error_m > p.max_xtrack_error_m ||
                st.xtrack_tripped_elapsed_s < p.xtrack_gate_min_off_s;
    } else {
      tripped = pr.xtrack_error_m > trip_level;
    }
    d.xtrack_tripped = tripped;
    if (tripped) {
      d.safety_ok = false;
      char buf[200];
      std::snprintf(buf, sizeof buf, "xtrack error %.3fm gate trip>%.3fm clear<=%.3fm (%s)",
                    pr.xtrack_error_m, trip_level, p.max_xtrack_error_m,
                    p.max_xtrack_from_mission ? "mission" : "param");
      d.safety_reason = buf;
    }

    std::optional<BoundaryKind> src_kind;
    double src_dist = kInf;
    if (d.next_boundary) {
      src_kind = d.next_boundary->kind;
      src_dist = d.next_boundary->s - pr.s;
    }
    if (src_kind && std::isfinite(src_dist)) {
      d.distance_to_boundary_m = src_dist;
      // The debounce delay is part of the time between this decision and the valve moving, so it is
      // led like the solenoid delay (SP-002). Safety OFF does not pass through here.
      const double on_lead =
          in.speed_mps * (p.solenoid_open_delay_s + p.debounce_delay_s) + p.on_overspray_margin_m;
      const double off_lead =
          std::max(0.0, in.speed_mps * (p.solenoid_close_delay_s + p.debounce_delay_s) -
                            p.off_overspray_margin_m);
      LeadEvent ev;
      d.geometry_desired =
          apply_mark_boundary_lead(d.geometry_desired, src_kind, src_dist, on_lead, off_lead, &ev);
      if (ev != LeadEvent::None) d.event = ev;
    }

    // Terminal shutoff: independent of any boundary/lead; the OFF lead is ~1 mm at creep speed so
    // the final geometric boundary would never be crossed.
    if (d.geometry_desired && !model->cumulative_s.empty() &&
        in.speed_mps <= p.terminal_off_speed_mps &&
        (model->cumulative_s.back() - pr.s) <= p.terminal_off_epsilon_m) {
      d.geometry_desired = false;
      d.event = LeadEvent::TerminalOff;
    }
  }
  d.desired = d.geometry_desired && d.safety_ok;
  return d;
}

}  // namespace dyx3_spray
