// gateway_node — ROS wiring of the system gateway. Contract: docs/contracts/dyx3_system_gateway.md.
// Single-threaded executor assumed: IPC lines arrive on the IPC thread and are handed over through
// a mutex-guarded inbox that step() drains, so no rclcpp call is ever made from the IPC thread.
// The IPC thread also wakes the executor (a guard condition) so a queued command is dispatched at
// once rather than on the next 10 ms tick. Every service call is asynchronous with its own
// deadline: no command, event or E-stop ever waits for another command's answer.
#pragma once

#include <atomic>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dyx3_interfaces/msg/emergency_stop_state.hpp"
#include "dyx3_interfaces/msg/estimator_health.hpp"
#include "dyx3_interfaces/msg/gnss_report.hpp"
#include "dyx3_interfaces/msg/mission_state.hpp"
#include "dyx3_interfaces/msg/motion_setpoint_status.hpp"
#include "dyx3_interfaces/msg/ntrip_status.hpp"
#include "dyx3_interfaces/msg/operator_link_status.hpp"
#include "dyx3_interfaces/msg/point_result.hpp"
#include "dyx3_interfaces/msg/px4_link_status.hpp"
#include "dyx3_interfaces/msg/recorder_status.hpp"
#include "dyx3_interfaces/msg/rpp_status.hpp"
#include "dyx3_interfaces/msg/rtk_status.hpp"
#include "dyx3_interfaces/msg/safety_gate_status.hpp"
#include "dyx3_interfaces/msg/spray_status.hpp"
#include "dyx3_interfaces/msg/vehicle_state.hpp"
#include "dyx3_interfaces/srv/abort_mission.hpp"
#include "dyx3_interfaces/srv/arm_disarm.hpp"
#include "dyx3_interfaces/srv/pause_mission.hpp"
#include "dyx3_interfaces/srv/resume_mission.hpp"
#include "dyx3_interfaces/srv/set_emergency_stop.hpp"
#include "dyx3_interfaces/srv/set_offboard.hpp"
#include "dyx3_interfaces/srv/set_spray_manual.hpp"
#include "dyx3_interfaces/srv/skip_point.hpp"
#include "dyx3_interfaces/srv/start_mission.hpp"
#include "dyx3_system_gateway/command_validator.hpp"
#include "dyx3_system_gateway/event_stream.hpp"
#include "dyx3_system_gateway/ipc_server.hpp"
#include "dyx3_system_gateway/operator_link.hpp"
#include "dyx3_system_gateway/telemetry_snapshot.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dyx3_gateway {

using ClockFn = std::function<double()>;
class WakeWaitable;

class GatewayNode : public rclcpp::Node {
public:
  // Commands queued between two steps; beyond it every command except an E-stop is refused "busy".
  static constexpr size_t kInboxCap = 256;
  // Service calls awaiting their answer; beyond it every command except an E-stop is refused
  // "busy", so a client that floods slow commands cannot grow the pending table without bound.
  static constexpr size_t kPendingCap = 64;
  // Line received on the IPC thread -> ROS service request sent (steady clock, microseconds).
  struct DispatchStats {
    uint64_t count{0};
    double last_us{0.0};
    double max_us{0.0};
    double sum_us{0.0};
    double estop_last_us{0.0};
  };

  explicit GatewayNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                       ClockFn clock = nullptr, bool create_timer = true);
  ~GatewayNode() override;
  void step(double now_s);  // public for deterministic tests
  const IpcServer& ipc() const { return ipc_; }
  // Order in which the most recent non-empty batch of client commands was processed: E-stops,
  // then the one (coalesced) heartbeat, then the rest in arrival order.
  const std::vector<CmdKind>& last_batch() const { return last_batch_; }
  // Test hook: removes every request still pending inside the rclcpp clients and returns how many
  // there were. Requests the gateway already answered (reply or timeout) must not be among them.
  size_t prune_rclcpp_pending_requests();
  // Test hook: commands queued for the next step (at most kInboxCap, plus any E-stops).
  size_t inbox_depth() const;
  // Test hook (read-only): the downstream services this node's clients cannot reach right now.
  std::vector<std::string> unavailable_services() const;
  // Test hooks (read on the executor thread): dispatch latency, and the sequence number of the
  // last event pushed (0 = none yet).
  const DispatchStats& dispatch_stats() const { return dispatch_; }
  uint64_t event_seq() const { return events_->seq(); }
  // The answer deadline of a command (contract section 2, per-command timeouts).
  double timeout_for(CmdKind k) const;

private:
  struct Inbound {
    int client;
    ParseResult pr;
    double rx_steady_s{0.0};  // when the IPC thread queued it (real steady clock)
  };
  struct Pending {
    int client;
    bool has_id;
    int64_t id;
    CmdKind kind;
    double deadline_s;
    double timeout_s;
    std::string request_id;        // echoed in every reply to a start_mission, "" when absent
    std::string what;              // audit label, e.g. "E-stop assert (source=tablet)"
    std::function<void()> forget;  // drops the request from the rclcpp client (GW-007)
  };
  void declare_params();
  void on_line(int client, const std::string& line);
  void process(const Inbound& in, double now_s);
  void on_connect(int client);
  void track_operator_link(double now_s);
  void track_stale_sources(double now_s);
  void note_dispatch(const Inbound& in);
  void reply(int client, bool has_id, int64_t id, bool ok, const std::string& code,
             const std::string& reason, const std::string& data_json = "{}");
  template <typename Srv, typename Fill, typename Render>
  void call(const Inbound& in, double now_s, typename rclcpp::Client<Srv>::SharedPtr cli,
            const char* name, Fill fill, Render render);
  std::string gateway_json(double now_s) const;
  OperatorLinkState link_state(double now_s) const;
  void publish_operator_link(double now_s);
  void audit_ipc(double now_s);

  ClockFn clock_;
  std::string socket_path_;
  int max_clients_{4};
  double telemetry_hz_{5.0}, operator_link_timeout_s_{2.0}, service_timeout_s_{2.0},
      snapshot_fresh_s_{1.0}, operator_link_hz_{10.0};
  // Per-command answer deadlines (contract section 2); service_timeout_s_ covers the fast accepts.
  double estop_timeout_s_{1.0}, arm_timeout_s_{4.0}, offboard_timeout_s_{5.0};
  double event_coalesce_s_{0.01};  // events of one kind closer than this are folded into one

  IpcServer ipc_;
  OperatorLink link_{2.0};
  TelemetrySnapshot snap_{1.0};
  mutable std::mutex inbox_mu_;
  std::deque<Inbound> inbox_;
  std::atomic<uint64_t> inbox_refused_{0};
  std::map<uint64_t, Pending> pending_;
  uint64_t next_token_{1};
  std::vector<CmdKind> last_batch_;
  DispatchStats dispatch_;
  // Pushed status events (contract section 1.3). Built after ipc_ (it broadcasts through it).
  std::unique_ptr<EventStream> events_;
  double last_link_pub_s_{-1e18}, last_tel_s_{-1e18};
  // Telemetry frames pushed so far (contract section 1): the frame's `seq`, separate from the event
  // seq_.
  uint64_t telemetry_seq_{0};
  // Audit log state (XR-GW-001): last logged operator-link state and IPC counters.
  bool link_alive_{false};
  double last_audit_s_{-1e18};
  int audit_clients_{0};
  uint64_t audit_dropped_{0}, audit_overflows_{0}, audit_rejected_{0}, audit_busy_{0};

  rclcpp::Publisher<dyx3_interfaces::msg::OperatorLinkStatus>::SharedPtr pub_link_;
  rclcpp::Client<dyx3_interfaces::srv::StartMission>::SharedPtr cli_start_;
  rclcpp::Client<dyx3_interfaces::srv::AbortMission>::SharedPtr cli_abort_;
  rclcpp::Client<dyx3_interfaces::srv::PauseMission>::SharedPtr cli_pause_;
  rclcpp::Client<dyx3_interfaces::srv::ResumeMission>::SharedPtr cli_resume_;
  rclcpp::Client<dyx3_interfaces::srv::SkipPoint>::SharedPtr cli_skip_;
  rclcpp::Client<dyx3_interfaces::srv::SetEmergencyStop>::SharedPtr cli_estop_;
  rclcpp::Client<dyx3_interfaces::srv::ArmDisarm>::SharedPtr cli_arm_;
  rclcpp::Client<dyx3_interfaces::srv::SetOffboard>::SharedPtr cli_offboard_;
  rclcpp::Client<dyx3_interfaces::srv::SetSprayManual>::SharedPtr cli_spray_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<WakeWaitable>
      wake_;  // set before the IPC thread starts, null in step()-driven tests
};

}  // namespace dyx3_gateway
