// rtk_node — see docs/contracts/dyx3_gnss_rtk.md
#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "dyx3_gnss_rtk/control_socket.hpp"
#include "dyx3_gnss_rtk/correction_health.hpp"
#include "dyx3_gnss_rtk/injection_authority.hpp"
#include "dyx3_gnss_rtk/lora_source.hpp"
#include "dyx3_gnss_rtk/ntrip_client.hpp"
#include "dyx3_gnss_rtk/rtcm_transport.hpp"
#include "dyx3_gnss_rtk/rtk_config.hpp"
#include "dyx3_gnss_rtk/worker_state.hpp"
#include "dyx3_interfaces/msg/gnss_report.hpp"
#include "dyx3_interfaces/msg/mission_state.hpp"
#include "dyx3_interfaces/msg/ntrip_status.hpp"
#include "dyx3_interfaces/msg/px4_link_status.hpp"
#include "dyx3_interfaces/msg/rtcm_data.hpp"
#include "dyx3_interfaces/msg/rtk_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dyx3_gnss_rtk {

using ClockFn = std::function<double()>;

class RtkNode : public rclcpp::Node {
public:
  // create_timers=false: the owner calls publish_status()/publish_ntrip_status().
  // start_client=false: no socket thread (tests inject frames with on_frame()).
  explicit RtkNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                   ClockFn clock = nullptr, bool create_timers = true, bool start_client = true,
                   std::optional<Json> test_config = std::nullopt);
  ~RtkNode() override;

  // Called from the NTRIP thread for every CRC-valid RTCM frame; thread-safe.
  void on_frame(const std::vector<uint8_t>& frame);
  void publish_status(double now_s);
  void publish_ntrip_status(double now_s);
  Json status_json(double now_s);
  Json handle_control(const Json& request);

private:
  std::optional<std::string> gga_for_caster();
  bool report_fresh(double now_s) const;  // requires m_ held
  // True while a fresh MissionState says an execution occupies the system. Locks m_.
  bool mission_active(double now_s) const;
  void on_source_frame(uint64_t generation, const std::vector<uint8_t>& frame);
  void start_selected_source(uint64_t generation);
  void stop_selected_source();
  void activate_config();
  void apply_config(Json candidate);
  void update_worker_state(double now_s);
  void transition(WorkerState next, const std::string& code, const std::string& reason);

  ClockFn clock_;
  double correction_fresh_s_{10.0};
  double gnss_report_max_age_s_{1.0};
  double gga_max_fix_age_s_{5.0};
  bool configured_{false};
  std::string config_error_;
  bool start_client_{true};
  RtkConfigStore config_store_;
  Json config_;
  // Serializes whole reconfigurations (apply_config). Never taken by a status path, so it can be
  // held across the worker join; lifecycle_m_ (which the 200 ms status timer needs) cannot.
  std::mutex reconfig_m_;
  std::mutex lifecycle_m_;
  bool reconfiguring_{false};  // guarded by lifecycle_m_
  std::mutex state_m_;
  WorkerStateMachine state_;
  std::unique_ptr<ControlSocket> control_;
  std::unique_ptr<UsbSerialSink> usb_;
  std::unique_ptr<DdsSink> dds_;
  std::unique_ptr<InjectionAuthority> authority_;
  std::unique_ptr<LoraSource> lora_;
  std::atomic<uint64_t> chunks_handed_off_{0};
  dyx3_interfaces::msg::Px4LinkStatus link_status_;
  bool have_link_status_{false};
  double link_status_at_s_{0};
  uint8_t mission_state_{dyx3_interfaces::msg::MissionState::STATE_IDLE};
  bool have_mission_state_{false};
  double mission_state_at_s_{0};
  double started_at_s_{0};

  mutable std::mutex m_;
  CorrectionHealth health_;
  FixMonitor fix_monitor_;
  bool have_report_{false};
  double report_t_{0.0};
  dyx3_interfaces::msg::GnssReport report_;
  NtripSnapshot ntrip_;

  std::unique_ptr<NtripClient> client_;
  rclcpp::Publisher<dyx3_interfaces::msg::RtcmData>::SharedPtr pub_rtcm_;
  rclcpp::Publisher<dyx3_interfaces::msg::RtkStatus>::SharedPtr pub_rtk_;
  rclcpp::Publisher<dyx3_interfaces::msg::NtripStatus>::SharedPtr pub_ntrip_;
  rclcpp::Subscription<dyx3_interfaces::msg::GnssReport>::SharedPtr sub_report_;
  rclcpp::Subscription<dyx3_interfaces::msg::Px4LinkStatus>::SharedPtr sub_link_status_;
  rclcpp::Subscription<dyx3_interfaces::msg::MissionState>::SharedPtr sub_mission_state_;
  rclcpp::TimerBase::SharedPtr t_rtk_, t_ntrip_;
};

}  // namespace dyx3_gnss_rtk
