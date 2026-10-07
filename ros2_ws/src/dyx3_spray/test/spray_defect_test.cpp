// KNOWN-OPEN DEFECT (spec 7.8, contract section 6): projection continuity on a path that doubles
// back.
//
// This is a SYNTHETIC reproduction of the mechanism, not the field bags (stg_d8a4f2ad /
// stg_46ba8830 replay is a LOCAL ACTION). The approach leg and the marked leg sit centimetres
// apart; a nearest-segment search cannot separate them by distance, so the reported station
// teleports between the legs and jumps over a MARK boundary. The spatial window does not fix it;
// the direction gate does. The gate's DEFAULT is 0.0 (disabled), so the defect is open by default.
//
// The first test ASSERTS THE DEFECT IS PRESENT (it documents it; it will start failing the day
// someone changes the default or the algorithm, which is the signal to revisit the contract). The
// second asserts the gate fixes it.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "dyx3_spray/boundary_projection.hpp"

using namespace dyx3_spray;

namespace {

constexpr double kLeg = 5.0;
constexpr int kPts = 11;
constexpr double kGap = 0.02;
constexpr double kMarkFrom = 4.8;  // on the return leg, vertices at n <= 4.8 are MARK

PathModel out_and_back() {
  std::vector<double> n, e;
  std::vector<bool> f;
  for (int i = 0; i < kPts; ++i) {
    n.push_back(kLeg * i / (kPts - 1));
    e.push_back(0.0);
    f.push_back(false);
  }
  // Return leg: a vertex at the MARK start (4.8) shortly after the turn, then every 0.5 m back to
  // the origin.
  std::vector<double> back{5.0, kMarkFrom, 4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0, 0.5, 0.0};
  for (const double nn : back) {
    n.push_back(nn);
    e.push_back(kGap);
    f.push_back(nn <= kMarkFrom);
  }
  PathModel m;
  EXPECT_TRUE(build_path_model(n, e, f, &m));
  return m;
}

struct Result {
  double max_jump{0.0};
  bool flag_flipped_across_a_jump{false};
  bool ever_marked{false};
  double last_s{0.0};
};

// Drive the APPROACH leg only (heading north), 3 mm lateral noise about the mid line, one tick
// every 7 mm (0.35 m/s at 50 Hz). The rover never reaches the marked leg, so any MARK flag is
// spurious.
Result drive_approach(double gate_deg) {
  const PathModel m = out_and_back();
  DecisionState st;
  Result r;
  std::optional<double> prev;
  bool prev_flag = false;
  for (double north = 0.0; north < 4.9; north += 0.007) {
    const double east = 0.010 + 0.003 * std::sin(north * 40.0);
    ProjectionConfig c;
    c.prev_s = st.prev_projection_s;
    c.window_back_m = 0.5;
    c.window_fwd_m = 2.0;
    c.reacquire_dist_m = 1.0;
    c.heading_rad = 0.0;  // north
    c.direction_gate_cos = direction_gate_cos(gate_deg);
    const auto p = project_onto_path(m, north, east, c);
    EXPECT_TRUE(p.has_value());
    if (!p) return r;
    if (prev) {
      const double jump = std::fabs(p->s - *prev);
      r.max_jump = std::max(r.max_jump, jump);
      if (jump > 0.25 && p->current_flag != prev_flag) r.flag_flipped_across_a_jump = true;
    }
    if (p->current_flag) r.ever_marked = true;
    prev = p->s;
    prev_flag = p->current_flag;
    st.prev_projection_s = p->s;
    r.last_s = p->s;
  }
  return r;
}

}  // namespace

TEST(SprayDefect, GateDisabledTheStationTeleportsAcrossTheLegs_DEFECT_IS_PRESENT) {
  const Result r = drive_approach(0.0);  // 0.0 = disabled = the shipped default
  // The station jumps by more than half a metre while the rover moves 7 mm per tick, and the MARK
  // flag flips across the jump although the rover is still on the TRANSIT approach leg: the valve
  // would open inside the mark.
  EXPECT_GT(r.max_jump, 0.5)
      << "the open defect is no longer reproduced: revisit docs/contracts/dyx3_spray.md section 6";
  EXPECT_TRUE(r.flag_flipped_across_a_jump);
  EXPECT_TRUE(r.ever_marked);
}

TEST(SprayDefect, WindowAloneDoesNotFixIt) {
  // Same drive with the spatial window disabled entirely (the pre-fix A/B arm): no better.
  const PathModel m = out_and_back();
  double max_jump = 0.0;
  std::optional<double> prev;
  for (double north = 0.0; north < 4.9; north += 0.007) {
    const double east = 0.010 + 0.003 * std::sin(north * 40.0);
    ProjectionConfig c;  // no prev_s, no window, no gate: global nearest segment
    const auto p = project_onto_path(m, north, east, c);
    ASSERT_TRUE(p.has_value());
    if (prev) max_jump = std::max(max_jump, std::fabs(p->s - *prev));
    prev = p->s;
  }
  EXPECT_GT(max_jump, 0.5);
}

TEST(SprayDefect, DirectionGateEnabledTheStationIsContinuous) {
  for (const double deg : {30.0, 60.0, 90.0, 120.0}) {
    const Result r = drive_approach(deg);
    EXPECT_LT(r.max_jump, 0.05) << "gate " << deg;
    EXPECT_FALSE(r.flag_flipped_across_a_jump) << "gate " << deg;
    EXPECT_FALSE(r.ever_marked) << "the approach leg is TRANSIT: no spurious MARK, gate " << deg;
    EXPECT_NEAR(r.last_s, 4.9, 0.02) << "gate " << deg;
  }
}

TEST(SprayDefect, GateEnabledTheReturnLegIsMarkedOnceAndTheValveOpensEarlyByTheLead) {
  const PathModel m = out_and_back();
  DecisionParams p;
  p.solenoid_open_delay_s = 0.18;
  p.solenoid_close_delay_s = 0.05;
  p.on_overspray_margin_m = 0.02;
  p.max_xtrack_error_m = 0.05;
  p.xtrack_trip_error_m = 0.08;
  p.terminal_off_epsilon_m = 0.05;
  p.terminal_off_speed_mps = 0.05;
  p.projection_window_back_m = 0.5;
  p.projection_window_fwd_m = 2.0;
  p.projection_reacquire_dist_m = 1.0;
  p.projection_direction_gate_cos = direction_gate_cos(60.0);
  DecisionState st;
  int rising = 0;
  bool prev = false;
  double first_on_s = -1.0;
  // approach north, then (instantaneous turn) return south on the marked leg
  auto tick = [&](double n, double e, double yaw) {
    DecisionInput in;
    in.model = &m;
    in.nozzle_n = n;
    in.nozzle_e = e;
    in.speed_mps = 0.35;
    in.yaw_rad = yaw;
    in.safety_ok = true;
    const Decision d = make_decision(in, p, st);
    if (d.projection) st.prev_projection_s = d.projection->s;
    st.xtrack_tripped = d.xtrack_tripped;
    if (d.desired && !prev) {
      ++rising;
      first_on_s = d.projection ? d.projection->s : -1.0;
    }
    prev = d.desired;
  };
  for (double n = 0.0; n <= 5.0; n += 0.007) tick(n, 0.0, 0.0);
  for (double n = 5.0; n >= 0.0; n -= 0.007) tick(n, kGap, M_PI);
  EXPECT_EQ(rising, 1);
  // The MARK boundary is at s = 5 + 0.02 + 0.5. ON leads it by v*open_delay + margin = 0.083.
  const double boundary = kLeg + kGap + (kLeg - kMarkFrom);
  EXPECT_NEAR(first_on_s, boundary - (0.35 * 0.18 + 0.02), 0.01);
}
