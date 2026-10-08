// spray_gates — the autonomous-spray gate stack. Contract: docs/contracts/dyx3_spray.md section 7.
// Pure C++. Ports rtk_quality.evaluate_rtk_quality and the controller's _gps_gate /
// _pivot_is_active / _auto_safety_status. First failing gate wins and its reason string is
// published. Time is caller-injected (monotonic seconds).
#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace dyx3_spray {

// ---- RTK sample quality (stateless)
// ---------------------------------------------------------------------------
struct RtkQuality {
  bool fresh{false};
  bool acceptable{false};
  std::string fix_name;
  std::string reason;
};

// fix_type is the PX4/MAVLink GPS_FIX_TYPE value. h_acc_m nullopt means "unknown" (the sentinel 0
// from the receiver). sample_age_s nullopt means never received.
RtkQuality evaluate_rtk_quality(int fix_type, std::optional<double> h_acc_m,
                                std::optional<double> sample_age_s, double timeout_s,
                                int min_fix_type, double max_h_acc_m, bool require_accuracy);

// ---- RTK gate with asymmetric hysteresis
// ----------------------------------------------------------------------
struct RtkGateConfig {
  bool require_rtk_fix{true};
  int min_fix_type{6};
  double max_h_acc_m{0.10};
  bool require_accuracy{true};
  double fix_timeout_s{0.5};
  double recover_hold_s{1.0};
};

struct GateResult {
  bool ok{false};
  std::string reason;
};

// A drop is instant; re-enable only after the fix has been continuously good for recover_hold_s.
// Call once per tick.
class RtkGate {
public:
  GateResult evaluate(const RtkGateConfig& cfg, int fix_type, std::optional<double> h_acc_m,
                      std::optional<double> sample_age_s, double now_s);
  void reset() { have_since_ = false; }

private:
  bool have_since_{false};
  double since_s_{0.0};
};

// ---- pivot evidence (CORNER_ALIGN only)
// ------------------------------------------------------------------------ Absent state is "not
// pivoting" (permissive: a missed pivot costs a puddle, a phantom pivot kills a whole run); stale
// state clears itself immediately (B3(b)).
class PivotGate {
public:
  void note_state(bool pivoting, double now_s) {
    pivoting_ = pivoting;
    have_ = true;
    recv_s_ = now_s;
  }
  bool active(bool off_during_pivot, double timeout_s, double now_s);

private:
  bool have_{false};
  bool pivoting_{false};
  double recv_s_{0.0};
};

// ---- mission / RPP ownership (review C1, human decision 2026-10-08)
// ------------------------------------------------------------- Autonomous spray is allowed only
// while the mission is RUNNING and RPP is fresh, reports the same mission and is in a state that
// may lay paint. STOPPING (CORNER_STOP lays the last ~2 cm of the leg) and PIVOTING (its own gate
// below) stay allowed; IDLE, LOADED, COMPLETE, ERROR and any unknown value refuse.
// Values mirror dyx3_interfaces/RppStatus.STATE_*.
enum class RppState : uint8_t {
  Idle = 0,
  Tracking = 1,
  Stopping = 2,
  Pivoting = 3,
  Creeping = 4,
  Complete = 5,
  Error = 6,
  Loaded = 7,
};

struct OwnershipInputs {
  bool mission_running{false};
  uint32_t mission_id{0};
  bool rpp_known{false};
  double rpp_age_s{0.0};
  double rpp_timeout_s{0.5};
  uint8_t rpp_state{0};
  uint32_t rpp_mission_id{0};
};

// Order: mission not running, rpp stale, rpp mission mismatch, rpp not marking.
GateResult ownership_status(const OwnershipInputs& in);

// ---- the stack
// -------------------------------------------------------------------------------------------------
struct GateInputs {
  GateResult ownership{false, "mission not running"};  // result of ownership_status()
  bool armed{false};
  bool offboard{false};
  bool path_loaded{false};
  bool pose_fresh{false};
  bool velocity_fresh{false};
  bool tracking_seen{false};  // RppStatus TRACKING since this path was loaded (B5)
  bool pivoting{false};       // result of PivotGate::active()
  bool estop_clear{false};    // DERIVED: E-stop known (fresh) and not asserted
  bool require_offboard{true};
  GateResult rtk;  // result of RtkGate::evaluate()
};

// Order: E-stop, mission/RPP ownership, disarmed, not OFFBOARD, path not loaded, pose stale,
// velocity stale, RTK, awaiting tracking, pivoting.
GateResult auto_safety_status(const GateInputs& in);

}  // namespace dyx3_spray
