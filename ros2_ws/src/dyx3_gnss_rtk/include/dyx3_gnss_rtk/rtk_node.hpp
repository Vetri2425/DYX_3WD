// rtk_node — see docs/contracts/dyx3_gnss_rtk.md
#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "dyx3_gnss_rtk/correction_health.hpp"
#include "dyx3_gnss_rtk/ntrip_client.hpp"
#include "dyx3_gnss_rtk/rtcm_transport.hpp"
#include "dyx3_interfaces/msg/gnss_report.hpp"
#include "dyx3_interfaces/msg/ntrip_status.hpp"
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
                   ClockFn clock = nullptr, bool create_timers = true, bool start_client = true);
  ~RtkNode() override;

  // Called from the NTRIP thread for every CRC-valid RTCM frame; thread-safe.
  void on_frame(const std::vector<uint8_t>& frame);
  void publish_status(double now_s);
  void publish_ntrip_status(double now_s);

private:
  std::optional<std::string> gga_for_caster();
  bool report_fresh(double now_s) const;  // requires m_ held

  ClockFn clock_;
  double correction_fresh_s_{10.0};
  double gnss_report_max_age_s_{1.0};
  double gga_max_fix_age_s_{5.0};
  bool configured_{false};
  std::string config_error_;

  mutable std::mutex m_;
  CorrectionHealth health_;
  FixMonitor fix_monitor_;
  Chunker chunker_;
  uint64_t chunks_handed_off_{0};
  bool have_report_{false};
  double report_t_{0.0};
  dyx3_interfaces::msg::GnssReport report_;
  NtripSnapshot ntrip_;

  std::unique_ptr<NtripClient> client_;
  rclcpp::Publisher<dyx3_interfaces::msg::RtcmData>::SharedPtr pub_rtcm_;
  rclcpp::Publisher<dyx3_interfaces::msg::RtkStatus>::SharedPtr pub_rtk_;
  rclcpp::Publisher<dyx3_interfaces::msg::NtripStatus>::SharedPtr pub_ntrip_;
  rclcpp::Subscription<dyx3_interfaces::msg::GnssReport>::SharedPtr sub_report_;
  rclcpp::TimerBase::SharedPtr t_rtk_, t_ntrip_;
};

}  // namespace dyx3_gnss_rtk
