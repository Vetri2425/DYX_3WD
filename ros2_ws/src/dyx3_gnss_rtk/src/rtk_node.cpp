#include "dyx3_gnss_rtk/rtk_node.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

#include "dyx3_gnss_rtk/gga_provider.hpp"

namespace dyx3_gnss_rtk {
namespace {

double steady_now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string env(const char* name) {
  const char* v = std::getenv(name);
  return v == nullptr ? std::string() : std::string(v);
}

void require(bool ok, const std::string& what) {
  if (!ok) throw std::invalid_argument("dyx3_gnss_rtk parameter invalid: " + what);
}

float clamp_f(double v) {
  return static_cast<float>(std::min(v, static_cast<double>(std::numeric_limits<float>::max())));
}

}  // namespace

RtkNode::RtkNode(const rclcpp::NodeOptions& options, ClockFn clock, bool create_timers,
                 bool start_client)
    : rclcpp::Node("gnss_rtk", options),
      clock_(clock ? std::move(clock) : ClockFn(steady_now_s)),
      health_(10.0) {
  correction_fresh_s_ = declare_parameter<double>("correction_fresh_s", 10.0);
  gnss_report_max_age_s_ = declare_parameter<double>("gnss_report_max_age_s", 1.0);
  const double rate_window = declare_parameter<double>("rate_window_s", 10.0);
  gga_max_fix_age_s_ = declare_parameter<double>("gga_max_fix_age_s", 5.0);
  NtripConfig cfg;
  cfg.connect_timeout_s = declare_parameter<double>("connect_timeout_s", 10.0);
  cfg.stream_timeout_s = declare_parameter<double>("stream_timeout_s", 10.0);
  cfg.gga_interval_s = declare_parameter<double>("gga_interval_s", 10.0);
  cfg.backoff_base_s = declare_parameter<double>("backoff_base_s", 5.0);
  cfg.backoff_max_s = declare_parameter<double>("backoff_max_s", 60.0);
  cfg.max_buffer_bytes = static_cast<size_t>(declare_parameter<int>("max_buffer_bytes", 8192));
  for (const double v : {correction_fresh_s_, gnss_report_max_age_s_, rate_window,
                         gga_max_fix_age_s_, cfg.connect_timeout_s, cfg.stream_timeout_s,
                         cfg.gga_interval_s, cfg.backoff_base_s, cfg.backoff_max_s}) {
    require(std::isfinite(v) && v > 0.0, "every time parameter must be finite and > 0");
  }
  require(cfg.backoff_max_s >= cfg.backoff_base_s, "backoff_max_s must be >= backoff_base_s");
  require(cfg.max_buffer_bytes >= 2048,
          "max_buffer_bytes must be >= 2048 (largest RTCM frame is 1029 bytes)");
  health_ = CorrectionHealth(rate_window);

  // Credentials come from the environment (EnvironmentFile=/etc/dyx3/ntrip.env), never argv or a
  // ROS parameter.
  cfg.host = env("DYX3_NTRIP_HOST");
  cfg.mountpoint = env("DYX3_NTRIP_MOUNTPOINT");
  cfg.user = env("DYX3_NTRIP_USER");
  cfg.password = env("DYX3_NTRIP_PASSWORD");
  const std::string port = env("DYX3_NTRIP_PORT");
  if (!port.empty()) cfg.port = std::atoi(port.c_str());
  configured_ = !cfg.host.empty() && !cfg.mountpoint.empty() && !cfg.user.empty() &&
                !cfg.password.empty() && cfg.port > 0 && cfg.port < 65536;
  if (!configured_) config_error_ = "NTRIP not configured (DYX3_NTRIP_* environment incomplete)";

  pub_rtcm_ =
      create_publisher<dyx3_interfaces::msg::RtcmData>("/dyx3/rtcm", rclcpp::QoS(32).reliable());
  pub_rtk_ = create_publisher<dyx3_interfaces::msg::RtkStatus>("/dyx3/rtk_status",
                                                               rclcpp::QoS(1).reliable());
  pub_ntrip_ = create_publisher<dyx3_interfaces::msg::NtripStatus>("/dyx3/ntrip_status",
                                                                   rclcpp::QoS(1).reliable());
  sub_report_ = create_subscription<dyx3_interfaces::msg::GnssReport>(
      "/dyx3/gnss_report", rclcpp::QoS(5),
      [this](dyx3_interfaces::msg::GnssReport::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(m_);
        report_ = *m;
        report_t_ = clock_();
        have_report_ = true;
      });

  if (configured_ && start_client) {
    client_ = std::make_unique<NtripClient>(
        cfg, [this](const std::vector<uint8_t>& f) { on_frame(f); },
        [this] { return gga_for_caster(); });
    client_->set_event_callback([this](NtripState s, const std::string& detail) {
      switch (s) {
        case NtripState::Streaming: {
          std::lock_guard<std::mutex> lk(m_);
          health_.on_connected(clock_());
        }
          RCLCPP_INFO(get_logger(), "NTRIP connected - streaming RTCM");
          break;
        case NtripState::Error: {
          std::lock_guard<std::mutex> lk(m_);
          health_.on_disconnected();
        }
          RCLCPP_ERROR(get_logger(), "NTRIP error: %s", detail.c_str());
          break;
        case NtripState::Reconnecting:
          RCLCPP_INFO(get_logger(), "NTRIP %s", detail.c_str());
          break;
        default:
          break;
      }
    });
    client_->start();
  }
  if (create_timers) {
    t_rtk_ =
        create_wall_timer(std::chrono::milliseconds(200), [this]() { publish_status(clock_()); });
    t_ntrip_ =
        create_wall_timer(std::chrono::seconds(1), [this]() { publish_ntrip_status(clock_()); });
  }
  if (!configured_) RCLCPP_WARN(get_logger(), "%s", config_error_.c_str());
  RCLCPP_INFO(get_logger(), "gnss_rtk up (correction_fresh %.1f s, gnss_report_max_age %.1f s)",
              correction_fresh_s_, gnss_report_max_age_s_);
}

RtkNode::~RtkNode() {
  if (client_) client_->stop();
}

bool RtkNode::report_fresh(double now_s) const {
  return have_report_ && now_s >= report_t_ && (now_s - report_t_) <= gnss_report_max_age_s_;
}

void RtkNode::on_frame(const std::vector<uint8_t>& frame) {
  std::vector<Chunk> chunks;
  {
    std::lock_guard<std::mutex> lk(m_);
    health_.on_frame(clock_(), frame.size());
    chunks = chunker_.split(frame);
    chunks_forwarded_ += chunks.size();
  }
  for (const auto& c : chunks) {
    dyx3_interfaces::msg::RtcmData m;
    m.stamp = get_clock()->now();
    m.flags = c.flags;
    m.data = c.data;
    pub_rtcm_->publish(m);
  }
}

std::optional<std::string> RtkNode::gga_for_caster() {
  GgaInput in;
  {
    std::lock_guard<std::mutex> lk(m_);
    if (!have_report_ || !report_.valid) return std::nullopt;
    in.latitude_deg = report_.latitude_deg;
    in.longitude_deg = report_.longitude_deg;
    in.altitude_msl_m = report_.altitude_msl_m;
    in.position_age_s = clock_() - report_t_;
    in.fix_type = report_.fix_type;
    in.satellites = report_.satellites_used;
    in.hdop = report_.hdop;
  }
  in.utc_epoch_s = static_cast<int64_t>(std::chrono::duration_cast<std::chrono::seconds>(
                                            std::chrono::system_clock::now().time_since_epoch())
                                            .count());
  return format_gga(in, gga_max_fix_age_s_);
}

void RtkNode::publish_status(double now_s) {
  dyx3_interfaces::msg::RtkStatus s;  // fail-safe defaults: unknown fix, not fresh
  bool fresh_report;
  std::optional<FixTransition> tr;
  {
    std::lock_guard<std::mutex> lk(m_);
    fresh_report = report_fresh(now_s);
    const bool corrections = health_.fresh(now_s, correction_fresh_s_);
    s.stamp = get_clock()->now();
    if (fresh_report && report_.valid) {
      s.fix_type = report_.fix_type;
      s.horizontal_accuracy_m =
          report_.horizontal_accuracy_m;  // 0 = unknown sentinel, passed through
      s.satellites_used = report_.satellites_used;
    }
    s.corrections_fresh = corrections && fresh_report;
    s.correction_age_s = clamp_f(health_.age_s(now_s));
    tr = fix_monitor_.update(now_s, s.fix_type, corrections, health_.age_s(now_s));
  }
  if (tr) {
    if (tr->suspicious) {
      RCLCPP_WARN(get_logger(),
                  "FIX dropped %u -> %u while corrections are fresh (age %.1f s): corrections "
                  "arriving but not "
                  "accepted? check fragmenting, mountpoint and `gps status` on the FCU",
                  static_cast<unsigned>(tr->from), static_cast<unsigned>(tr->to),
                  tr->correction_age_s);
    } else {
      RCLCPP_INFO(get_logger(), "FIX %u -> %u (corrections fresh=%d, age %.1f s)",
                  static_cast<unsigned>(tr->from), static_cast<unsigned>(tr->to),
                  tr->corrections_fresh ? 1 : 0, tr->correction_age_s);
    }
  }
  pub_rtk_->publish(s);
}

void RtkNode::publish_ntrip_status(double now_s) {
  dyx3_interfaces::msg::NtripStatus n;  // default: no corrections
  NtripSnapshot snap;
  if (client_) snap = client_->snapshot();
  {
    std::lock_guard<std::mutex> lk(m_);
    n.stamp = get_clock()->now();
    const bool fresh = health_.fresh(now_s, correction_fresh_s_);
    switch (snap.state) {
      case NtripState::Starting:
        n.state = dyx3_interfaces::msg::NtripStatus::STATE_STARTING;
        break;
      case NtripState::Connecting:
        n.state = dyx3_interfaces::msg::NtripStatus::STATE_CONNECTING;
        break;
      case NtripState::Streaming:
        n.state = dyx3_interfaces::msg::NtripStatus::STATE_STREAMING;
        break;
      case NtripState::Reconnecting:
        n.state = dyx3_interfaces::msg::NtripStatus::STATE_RECONNECTING;
        break;
      case NtripState::Error:
        n.state = dyx3_interfaces::msg::NtripStatus::STATE_ERROR;
        break;
      case NtripState::Stopped:
        n.state = dyx3_interfaces::msg::NtripStatus::STATE_STOPPED;
        break;
    }
    n.connected = snap.connected;
    n.streaming = snap.connected && fresh;
    n.correction_age_s = clamp_f(health_.age_s(now_s));
    n.correction_rate_hz = static_cast<float>(health_.rate_hz(now_s));
    n.frames_total = health_.frames();
    n.bytes_total = health_.bytes();
    n.crc_failures = snap.crc_failures;
    n.resync_bytes = snap.resync_bytes;
    n.reconnect_count = snap.reconnects;
    n.connected_for_s = static_cast<float>(health_.connected_for_s(now_s));
    n.last_error = configured_ ? snap.last_error : config_error_;
    if (!configured_) n.state = dyx3_interfaces::msg::NtripStatus::STATE_ERROR;
    n.chunks_forwarded = chunks_forwarded_;
    n.gga_sent = snap.gga_sent;
    n.fix_transitions = fix_monitor_.transitions();
    n.fix_type = fix_monitor_.fix_type();
  }
  pub_ntrip_->publish(n);
}

}  // namespace dyx3_gnss_rtk
