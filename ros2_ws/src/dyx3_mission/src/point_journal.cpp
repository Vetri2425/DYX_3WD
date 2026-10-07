// point_journal — see docs/contracts/dyx3_mission.md section 6
#include "dyx3_mission/point_journal.hpp"

#include <cmath>
#include <stdexcept>

namespace dyx3_mission {

PointJournal::PointJournal(const std::vector<ArtifactPoint>& path, double capture_radius_m)
    : radius_(capture_radius_m) {
  if (!(capture_radius_m > 0.0) || !std::isfinite(capture_radius_m)) {
    throw std::invalid_argument("capture_radius_m must be finite and > 0");
  }
  for (const auto& p : path) {
    if (p.must_hit()) {
      vertices_.push_back({p.north_m, p.east_m});
    }
  }
}

std::optional<std::uint32_t> PointJournal::active_point() const {
  if (next_ >= vertices_.size()) return std::nullopt;
  return static_cast<std::uint32_t>(next_);
}

PointEvent PointJournal::complete_current() {
  PointEvent ev;
  ev.point_index = static_cast<std::uint32_t>(next_);
  ev.outcome = PointOutcome::kCompleted;
  ev.north_m = min_n_;
  ev.east_m = min_e_;
  ev.error_m = min_d_;
  ++next_;
  capturing_ = false;
  return ev;
}

PointEvent PointJournal::fail_one(std::size_t i) const {
  PointEvent ev;
  ev.point_index = static_cast<std::uint32_t>(i);
  ev.outcome = PointOutcome::kFailed;
  ev.north_m = vertices_[i].n;
  ev.east_m = vertices_[i].e;
  return ev;
}

std::vector<PointEvent> PointJournal::update(double north_m, double east_m) {
  std::vector<PointEvent> out;
  if (!std::isfinite(north_m) || !std::isfinite(east_m)) {
    return out;  // a non-finite position carries no information
  }
  // Re-evaluate until no further point resolves with this same position (consecutive vertices may
  // lie inside one capture radius).
  while (next_ < vertices_.size()) {
    const Vertex& v = vertices_[next_];
    const double d = std::hypot(north_m - v.n, east_m - v.e);

    if (!capturing_) {
      // Bypass: the NEXT vertex was captured first, so this one was missed.
      if (next_ + 1 < vertices_.size()) {
        const Vertex& w = vertices_[next_ + 1];
        if (std::hypot(north_m - w.n, east_m - w.e) <= radius_ && d > radius_) {
          out.push_back(fail_one(next_));
          ++next_;
          continue;
        }
      }
      if (d <= radius_) {
        capturing_ = true;
        min_d_ = d;
        min_n_ = north_m;
        min_e_ = east_m;
      }
      break;
    }

    if (d <= radius_) {
      if (d < min_d_) {
        min_d_ = d;
        min_n_ = north_m;
        min_e_ = east_m;
      }
      break;
    }
    // Left the radius: the closest approach is final.
    out.push_back(complete_current());
  }
  return out;
}

std::optional<PointEvent> PointJournal::skip() {
  if (next_ >= vertices_.size()) return std::nullopt;
  PointEvent ev;
  ev.point_index = static_cast<std::uint32_t>(next_);
  ev.outcome = PointOutcome::kSkipped;
  ev.north_m = vertices_[next_].n;
  ev.east_m = vertices_[next_].e;
  ++next_;
  capturing_ = false;
  return ev;
}

std::vector<PointEvent> PointJournal::finish() {
  std::vector<PointEvent> out;
  if (capturing_ && next_ < vertices_.size()) {
    out.push_back(complete_current());
  }
  for (; next_ < vertices_.size(); ++next_) {
    out.push_back(fail_one(next_));
  }
  return out;
}

}  // namespace dyx3_mission
