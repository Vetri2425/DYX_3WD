#include "dyx3_gnss_rtk/rtk_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <stdexcept>

#include "dyx3_gnss_rtk/gga_provider.hpp"

namespace dyx3_gnss_rtk {
namespace {
double steady_now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string utc_now() {
  const std::time_t now = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&now, &tm);
  char result[32]{};
  std::strftime(result, sizeof result, "%Y-%m-%dT%H:%M:%SZ", &tm);
  return result;
}
std::string env_or(const char* key, const char* fallback) {
  const char* value = std::getenv(key);
  return value && *value ? value : fallback;
}
float clamp_f(double value) {
  return static_cast<float>(
      std::min(value, static_cast<double>(std::numeric_limits<float>::max())));
}
Json age_json(double last, double now) {
  return last < 0 || now < last ? Json(nullptr) : Json(now - last);
}
}  // namespace

RtkNode::RtkNode(const rclcpp::NodeOptions& options, ClockFn clock, bool create_timers,
                 bool start_client, std::optional<Json> test_config)
    : rclcpp::Node("gnss_rtk", options),
      clock_(clock ? std::move(clock) : ClockFn(steady_now_s)),
      start_client_(start_client),
      config_store_(env_or("DYX3_RTK_STATE_DIR", "/var/lib/dyx3/rtk")),
      health_(10.0) {
  started_at_s_ = clock_();
  correction_fresh_s_ = declare_parameter<double>("correction_fresh_s", 10.0);
  gnss_report_max_age_s_ = declare_parameter<double>("gnss_report_max_age_s", 1.0);
  const double rate_window = declare_parameter<double>("rate_window_s", 10.0);
  gga_max_fix_age_s_ = declare_parameter<double>("gga_max_fix_age_s", 5.0);
  for (const double value :
       {correction_fresh_s_, gnss_report_max_age_s_, rate_window, gga_max_fix_age_s_})
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument("RTK time parameters must be finite and positive");
  health_ = CorrectionHealth(rate_window);

  pub_rtcm_ =
      create_publisher<dyx3_interfaces::msg::RtcmData>("/dyx3/rtcm", rclcpp::QoS(32).reliable());
  pub_rtk_ = create_publisher<dyx3_interfaces::msg::RtkStatus>("/dyx3/rtk_status",
                                                               rclcpp::QoS(1).reliable());
  pub_ntrip_ = create_publisher<dyx3_interfaces::msg::NtripStatus>("/dyx3/ntrip_status",
                                                                   rclcpp::QoS(1).reliable());
  sub_report_ = create_subscription<dyx3_interfaces::msg::GnssReport>(
      "/dyx3/gnss_report", rclcpp::QoS(5),
      [this](dyx3_interfaces::msg::GnssReport::ConstSharedPtr report) {
        std::lock_guard<std::mutex> lk(m_);
        report_ = *report;
        report_t_ = clock_();
        have_report_ = true;
      });
  sub_link_status_ = create_subscription<dyx3_interfaces::msg::Px4LinkStatus>(
      "/dyx3/px4_link/status", rclcpp::QoS(5).reliable(),
      [this](dyx3_interfaces::msg::Px4LinkStatus::ConstSharedPtr status) {
        std::lock_guard<std::mutex> lk(m_);
        link_status_ = *status;
        link_status_at_s_ = clock_();
        have_link_status_ = true;
      });

  std::string startup_error;
  if (test_config) {
    config_ = *test_config;
    RtkConfigStore::validate(config_);
  } else {
    // An unreadable or invalid config file or seed must not crash-loop the service: the
    // control socket has to come up so the operator can replace the configuration. Start
    // STOPPED (nothing injected) and leave the bad file in place until SET_CONFIG replaces it.
    try {
      const auto loaded = config_store_.load();
      config_ = loaded ? *loaded : RtkConfigStore::initial_from_environment();
      if (!loaded) config_store_.save(config_);
    } catch (const ConfigError& e) {
      startup_error = e.what();  // fixed, secret-free texts only
    } catch (const std::exception&) {
      startup_error = "RTK configuration could not be loaded";
    }
    if (!startup_error.empty()) {
      RCLCPP_ERROR(get_logger(), "%s; corrections stopped until the configuration is replaced",
                   startup_error.c_str());
      config_ = RtkConfigStore::defaults();
      config_["desired_state"] = "STOPPED";
    }
  }
  activate_config();
  if (!startup_error.empty()) transition(WorkerState::Error, "CONFIG_INVALID", startup_error);
  if (start_client_) {
    control_ = std::make_unique<ControlSocket>(
        env_or("DYX3_RTK_CONTROL_SOCKET", "/run/dyx3/rtk-control.sock"),
        [this](const Json& request) { return handle_control(request); });
    control_->start();
  }
  if (create_timers) {
    t_rtk_ = create_wall_timer(std::chrono::milliseconds(200), [this] {
      const double now = clock_();
      publish_status(now);
      update_worker_state(now);
    });
    t_ntrip_ =
        create_wall_timer(std::chrono::seconds(1), [this] { publish_ntrip_status(clock_()); });
  }
  RCLCPP_INFO(get_logger(), "gnss_rtk up (source=%s, transport=%s, revision=%llu)",
              config_.at("source").get<std::string>().c_str(),
              config_.at("transport").get<std::string>().c_str(),
              static_cast<unsigned long long>(config_.at("revision").get<uint64_t>()));
}

RtkNode::~RtkNode() {
  if (control_) control_->stop();
  std::lock_guard<std::mutex> lk(lifecycle_m_);
  if (authority_) authority_->stop();
  stop_selected_source();
}

void RtkNode::transition(WorkerState next, const std::string& code, const std::string& reason) {
  std::lock_guard<std::mutex> lk(state_m_);
  state_.transition(next, code, reason, utc_now(), config_.at("revision").get<uint64_t>());
}

void RtkNode::stop_selected_source() {
  if (client_) client_->stop();
  if (lora_) lora_->stop();
  client_.reset();
  lora_.reset();
  std::lock_guard<std::mutex> lk(m_);
  health_.reset_stream();
}

void RtkNode::start_selected_source(uint64_t generation) {
  configured_ = false;
  config_error_.clear();
  if (config_.at("source") == "NTRIP") {
    try {
      const NtripConfig cfg = RtkConfigStore::ntrip_config(config_);
      configured_ = !cfg.host.empty() && !cfg.mountpoint.empty() && !cfg.user.empty() &&
                    !cfg.password.empty() && cfg.security.has_value();
      if (!configured_) throw ConfigError("NTRIP profile incomplete");
      if (cfg.security == NtripSecurity::Plaintext)
        RCLCPP_WARN(get_logger(), "NTRIP PLAINTEXT sends credentials without transport encryption");
      if (start_client_) {
        client_ = std::make_unique<NtripClient>(
            cfg,
            [this, generation](const std::vector<uint8_t>& frame) {
              on_source_frame(generation, frame);
            },
            [this] { return gga_for_caster(); });
        client_->set_event_callback([this](NtripState state, const std::string& detail) {
          {
            std::lock_guard<std::mutex> lk(m_);
            if (state == NtripState::Streaming) health_.on_connected(clock_());
            if (state == NtripState::Error || state == NtripState::Stopped)
              health_.on_disconnected();
          }
          if (state == NtripState::Error)
            RCLCPP_WARN(get_logger(), "NTRIP connection: %s", detail.c_str());
        });
        client_->start();
      }
    } catch (const ConfigError&) {
      config_error_ = "NTRIP profile is not configured";
    }
  } else {
    const LoraConfig cfg = RtkConfigStore::lora_config(config_);
    configured_ = !cfg.serial_device.empty();
    if (!configured_) {
      config_error_ = "LoRa serial device is not configured";
    } else if (start_client_) {
      lora_ =
          std::make_unique<LoraSource>(cfg, [this, generation](const std::vector<uint8_t>& frame) {
            on_source_frame(generation, frame);
          });
      lora_->start();
    }
  }
  if (!configured_) transition(WorkerState::WaitSource, "SOURCE_NOT_CONFIGURED", config_error_);
}

void RtkNode::activate_config() {
  usb_ = std::make_unique<UsbSerialSink>(RtkConfigStore::usb_config(config_));
  dds_ = std::make_unique<DdsSink>(
      [this](const Chunk& chunk) {
        dyx3_interfaces::msg::RtcmData message;
        message.stamp = get_clock()->now();
        message.flags = chunk.flags;
        message.data = chunk.data;
        pub_rtcm_->publish(message);
        ++chunks_handed_off_;
        return true;
      },
      [this] {
        if (!start_client_) return true;  // deterministic node tests have no FCU process
        std::lock_guard<std::mutex> lk(m_);
        return have_link_status_ && clock_() >= link_status_at_s_ &&
               clock_() - link_status_at_s_ <= gnss_report_max_age_s_ &&
               link_status_.session_alive && link_status_.handshake_ok &&
               pub_rtcm_->get_subscription_count() > 0;
      });
  authority_ = std::make_unique<InjectionAuthority>(*usb_, *dds_);
  if (config_.at("desired_state") == "STOPPED") {
    transition(WorkerState::Stopped, "STOP_REQUESTED", "RTK corrections stopped by operator");
    return;
  }
  transition(WorkerState::Starting, "SERVICE_START",
             "starting configured RTK source and transport");
  const auto source =
      config_.at("source") == "NTRIP" ? CorrectionSource::Ntrip : CorrectionSource::Lora;
  const auto transport = config_.at("transport") == "USB_DIRECT" ? CorrectionTransport::UsbDirect
                                                                 : CorrectionTransport::Px4Dds;
  const uint64_t generation = authority_->switch_to(source, transport, clock_());
  start_selected_source(generation);
}

void RtkNode::apply_config(Json candidate) {
  std::lock_guard<std::mutex> lk(lifecycle_m_);
  const uint64_t old_revision = config_.at("revision").get<uint64_t>();
  if (!candidate.contains("revision") || !candidate.at("revision").is_number_integer() ||
      candidate.at("revision").get<uint64_t>() != old_revision)
    throw ConfigError("configuration revision conflict");
  candidate = RtkConfigStore::merge_write_only_passwords(config_, std::move(candidate));
  candidate["revision"] = old_revision + 1;
  candidate["updated_at"] = utc_now();
  RtkConfigStore::validate(candidate);
  transition(WorkerState::Reconfiguring, "CONFIG_CHANGE", "operator changed RTK configuration");
  // Break before make. A callback already in deliver completes before stop returns. Old
  // callbacks waiting behind it fail the generation check. Join source before replacing sinks.
  authority_->stop();
  stop_selected_source();
  try {
    config_store_.save(candidate);
  } catch (...) {
    // A failure before rename leaves the old file. A directory-fsync error after rename
    // may leave the new file. Reload it so runtime and persistent authority agree.
    try {
      if (auto persisted = config_store_.load()) config_ = *persisted;
      activate_config();
    } catch (const std::exception&) {
      transition(WorkerState::Error, "CONFIG_RECOVERY_FAILED", "RTK configuration recovery failed");
    }
    throw;
  }
  config_ = std::move(candidate);
  activate_config();
}

Json RtkNode::handle_control(const Json& request) {
  const std::string command = request.at("cmd").get<std::string>();
  if (command == "GET_STATUS") return {{"v", 1}, {"ok", true}, {"data", status_json(clock_())}};
  if (command == "GET_CONFIG") {
    std::lock_guard<std::mutex> lk(lifecycle_m_);
    return {{"v", 1}, {"ok", true}, {"data", RtkConfigStore::public_view(config_)}};
  }
  if (command == "SET_CONFIG") {
    if (!request.contains("config") || !request.at("config").is_object())
      throw ConfigError("config object required");
    apply_config(request.at("config"));
    std::lock_guard<std::mutex> lk(lifecycle_m_);
    return {{"v", 1}, {"ok", true}, {"data", RtkConfigStore::public_view(config_)}};
  }
  if (command == "START" || command == "STOP") {
    Json next;
    {
      std::lock_guard<std::mutex> lk(lifecycle_m_);
      next = config_;
    }
    next["desired_state"] = command == "START" ? "RUNNING" : "STOPPED";
    apply_config(std::move(next));
    return {{"v", 1}, {"ok", true}, {"data", status_json(clock_())}};
  }
  return {{"v", 1},
          {"ok", false},
          {"code", "unknown_command"},
          {"reason", "unsupported RTK control command"}};
}

bool RtkNode::report_fresh(double now_s) const {
  return have_report_ && now_s >= report_t_ && now_s - report_t_ <= gnss_report_max_age_s_;
}

void RtkNode::on_source_frame(uint64_t generation, const std::vector<uint8_t>& bytes) {
  const auto frame = ValidatedFrame::parse(bytes);
  if (!frame) return;
  const double now = clock_();
  const bool delivered = authority_ && authority_->deliver(generation, *frame, now);
  {
    std::lock_guard<std::mutex> lk(m_);
    if (config_.at("source") == "LORA" && !health_.connected()) health_.on_connected(now);
    health_.on_frame(now, bytes.size());
  }
  if (delivered)
    transition(WorkerState::Injecting, "RTCM_DELIVERED",
               "valid RTCM delivered to selected transport");
}

void RtkNode::on_frame(const std::vector<uint8_t>& frame) {
  on_source_frame(authority_ ? authority_->generation() : 0, frame);
}

std::optional<std::string> RtkNode::gga_for_caster() {
  if (authority_ && authority_->transport() == CorrectionTransport::UsbDirect && usb_) {
    const std::string readback = usb_->last_gga(clock_(), gga_max_fix_age_s_);
    if (!readback.empty()) return readback;
  }
  GgaInput input;
  {
    std::lock_guard<std::mutex> lk(m_);
    if (!have_report_ || !report_.valid) return std::nullopt;
    input.latitude_deg = report_.latitude_deg;
    input.longitude_deg = report_.longitude_deg;
    input.altitude_msl_m = report_.altitude_msl_m;
    input.position_age_s = clock_() - report_t_;
    input.fix_type = report_.fix_type;
    input.satellites = report_.satellites_used;
    input.hdop = report_.hdop;
  }
  input.utc_epoch_s = std::chrono::duration_cast<std::chrono::seconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
  return format_gga(input, gga_max_fix_age_s_);
}

void RtkNode::publish_status(double now_s) {
  double delivery_at = -1;
  bool lora_disconnected = false;
  {
    std::lock_guard<std::mutex> lk(lifecycle_m_);
    if (usb_ && config_.at("transport") == "USB_DIRECT") usb_->read_available(now_s);
    if (authority_) delivery_at = authority_->selected_counters().last_delivery_s;
    lora_disconnected = config_.at("source") == "LORA" && lora_ && !lora_->snapshot().port_open;
  }
  dyx3_interfaces::msg::RtkStatus status;
  std::optional<FixTransition> change;
  {
    std::lock_guard<std::mutex> lk(m_);
    if (lora_disconnected) health_.on_disconnected();
    const bool source_fresh = health_.fresh(now_s, correction_fresh_s_);
    const bool delivery_fresh =
        delivery_at >= 0 && now_s >= delivery_at && now_s - delivery_at <= correction_fresh_s_;
    status.stamp = get_clock()->now();
    if (report_fresh(now_s) && report_.valid) {
      status.fix_type = report_.fix_type;
      status.horizontal_accuracy_m = report_.horizontal_accuracy_m;
      status.satellites_used = report_.satellites_used;
    }
    status.corrections_fresh = source_fresh && delivery_fresh && report_fresh(now_s);
    status.correction_age_s = clamp_f(health_.age_s(now_s));
    change =
        fix_monitor_.update(now_s, status.fix_type, status.corrections_fresh, health_.age_s(now_s));
  }
  if (change)
    RCLCPP_INFO(get_logger(), "GNSS fix %u -> %u", static_cast<unsigned>(change->from),
                static_cast<unsigned>(change->to));
  pub_rtk_->publish(status);
}

void RtkNode::publish_ntrip_status(double now_s) {
  NtripSnapshot snap;
  bool ntrip_selected;
  bool configured;
  std::string config_error;
  {
    std::lock_guard<std::mutex> lk(lifecycle_m_);
    if (client_) snap = client_->snapshot();
    ntrip_selected = config_.at("source") == "NTRIP";
    configured = configured_;
    config_error = config_error_;
  }
  dyx3_interfaces::msg::NtripStatus status;
  {
    std::lock_guard<std::mutex> lk(m_);
    status.stamp = get_clock()->now();
    switch (snap.state) {
      case NtripState::Starting:
        status.state = dyx3_interfaces::msg::NtripStatus::STATE_STARTING;
        break;
      case NtripState::Connecting:
        status.state = dyx3_interfaces::msg::NtripStatus::STATE_CONNECTING;
        break;
      case NtripState::Streaming:
        status.state = dyx3_interfaces::msg::NtripStatus::STATE_STREAMING;
        break;
      case NtripState::Reconnecting:
        status.state = dyx3_interfaces::msg::NtripStatus::STATE_RECONNECTING;
        break;
      case NtripState::Error:
        status.state = dyx3_interfaces::msg::NtripStatus::STATE_ERROR;
        break;
      case NtripState::Stopped:
        status.state = dyx3_interfaces::msg::NtripStatus::STATE_STOPPED;
        break;
    }
    if (!ntrip_selected) status.state = dyx3_interfaces::msg::NtripStatus::STATE_STOPPED;
    if (ntrip_selected && !configured)
      status.state = dyx3_interfaces::msg::NtripStatus::STATE_ERROR;
    status.connected = snap.connected;
    status.streaming = snap.connected && health_.fresh(now_s, correction_fresh_s_);
    status.correction_age_s = clamp_f(health_.age_s(now_s));
    status.correction_rate_hz = static_cast<float>(health_.rate_hz(now_s));
    status.frames_total = health_.frames();
    status.bytes_total = health_.bytes();
    status.crc_failures = snap.crc_failures;
    status.resync_bytes = snap.resync_bytes;
    status.reconnect_count = snap.reconnects;
    status.connected_for_s = static_cast<float>(health_.connected_for_s(now_s));
    status.last_error = configured ? snap.last_error : config_error;
    status.source_bytes_received = snap.bytes;
    status.valid_rtcm_frames = snap.frames;
    status.chunks_handed_off = chunks_handed_off_.load();
    status.gga_sent = snap.gga_sent;
    status.fix_transitions = fix_monitor_.transitions();
    status.fix_type = fix_monitor_.fix_type();
    status.security = snap.security == NtripSecurity::Plaintext
                          ? dyx3_interfaces::msg::NtripStatus::SECURITY_PLAINTEXT
                      : snap.security == NtripSecurity::Tls
                          ? dyx3_interfaces::msg::NtripStatus::SECURITY_TLS
                          : dyx3_interfaces::msg::NtripStatus::SECURITY_UNSPECIFIED;
    status.tls_verified = snap.tls_verified;
    status.tls_verification_failed = snap.tls_verification_failed;
    status.plaintext_credentials_warning = snap.plaintext_credentials_warning;
  }
  pub_ntrip_->publish(status);
}

void RtkNode::update_worker_state(double now_s) {
  std::lock_guard<std::mutex> lk(lifecycle_m_);
  if (config_.at("desired_state") == "STOPPED") return;
  if (!configured_) {
    transition(WorkerState::WaitSource, "SOURCE_NOT_CONFIGURED", config_error_);
    return;
  }
  const bool source_ready = config_.at("source") == "NTRIP"
                                ? client_ && client_->snapshot().connected
                                : lora_ && lora_->snapshot().port_open;
  if (!source_ready) {
    transition(WorkerState::Degraded, "SOURCE_UNAVAILABLE", "configured source is reconnecting");
    return;
  }
  double source_age;
  {
    std::lock_guard<std::mutex> health_lock(m_);
    source_age = health_.age_s(now_s);
  }
  if (source_age > correction_fresh_s_) {
    transition(WorkerState::WaitRtcm, "RTCM_STALE", "waiting for valid RTCM from selected source");
    return;
  }
  const auto delivery = authority_->selected_counters();
  if (!authority_->active() || delivery.last_delivery_s < 0 ||
      now_s - delivery.last_delivery_s > correction_fresh_s_)
    transition(WorkerState::WaitTransport, "TRANSPORT_UNAVAILABLE",
               "selected transport has not delivered recent RTCM");
  else
    transition(WorkerState::Injecting, "RTCM_DELIVERED",
               "valid RTCM delivered to selected transport");
}

Json RtkNode::status_json(double now_s) {
  Json config;
  NtripSnapshot ntrip;
  LoraSnapshot lora;
  SinkCounters sink;
  bool active = false;
  bool configured = false;
  std::string error;
  std::optional<ReceiverReadback> readback;
  {
    std::lock_guard<std::mutex> lk(lifecycle_m_);
    config = RtkConfigStore::public_view(config_);
    if (client_) ntrip = client_->snapshot();
    if (lora_) lora = lora_->snapshot();
    if (authority_) {
      sink = authority_->selected_counters();
      active = authority_->active();
    }
    if (usb_ && config_.at("transport") == "USB_DIRECT")
      readback = usb_->readback(now_s, gnss_report_max_age_s_);
    configured = configured_;
    error = config_error_;
  }
  Json source, transport, receiver;
  {
    std::lock_guard<std::mutex> lk(m_);
    source = {
        {"selected", config.at("source")},
        {"connected", config.at("source") == "NTRIP" ? ntrip.connected : lora.port_open},
        {"bytes_received", config.at("source") == "NTRIP" ? ntrip.bytes : lora.bytes_received},
        {"valid_frames", config.at("source") == "NTRIP" ? ntrip.frames : lora.valid_frames},
        {"crc_failures", config.at("source") == "NTRIP" ? ntrip.crc_failures : lora.crc_failures},
        {"invalid_headers",
         config.at("source") == "NTRIP" ? ntrip.invalid_headers : lora.invalid_headers},
        {"resync_bytes", config.at("source") == "NTRIP" ? ntrip.resync_bytes : lora.resync_bytes},
        {"partial_timeouts",
         config.at("source") == "NTRIP" ? ntrip.partial_timeouts : lora.partial_timeouts},
        {"last_message_type",
         config.at("source") == "NTRIP" ? ntrip.last_message_type : lora.last_message_type},
        {"read_errors", lora.read_errors},
        {"reopens", config.at("source") == "NTRIP" ? ntrip.reconnects : lora.reopens},
        {"source_valid_rtcm_age_s",
         health_.has_frame() ? Json(health_.age_s(now_s)) : Json(nullptr)},
        {"rtcm_rate_hz", health_.rate_hz(now_s)},
        {"gga_sent", ntrip.gga_sent},
        {"security", ntrip.security == NtripSecurity::Tls         ? "TLS"
                     : ntrip.security == NtripSecurity::Plaintext ? "PLAINTEXT"
                                                                  : "UNSPECIFIED"},
        {"plaintext_credentials_warning", ntrip.plaintext_credentials_warning},
        {"tls_verified", ntrip.tls_verified},
        {"last_error", configured ? ntrip.last_error : error}};
    receiver = {
        {"valid", report_fresh(now_s) && report_.valid},
        {"fix_type", report_fresh(now_s) && report_.valid ? report_.fix_type : 0},
        {"horizontal_accuracy_m",
         report_fresh(now_s) && report_.valid ? report_.horizontal_accuracy_m : 0},
        {"vertical_accuracy_m",
         report_fresh(now_s) && report_.valid ? report_.vertical_accuracy_m : 0},
        {"satellites_used", report_fresh(now_s) && report_.valid ? report_.satellites_used : 0},
        {"hdop", report_fresh(now_s) && report_.valid ? report_.hdop : 0},
        {"fix_transitions", fix_monitor_.transitions()},
        {"receiver_correction_age_valid", false},
        {"receiver_correction_age_s", nullptr}};
    if (readback) {
      receiver["receiver_readback"] = "AVAILABLE";
      receiver["gga_fix_quality"] = readback->fix_quality;
      receiver["gga_satellites"] = readback->satellites;
      receiver["gga_hdop"] = readback->hdop;
      if (readback->correction_age_s) {
        receiver["receiver_correction_age_valid"] = true;
        receiver["receiver_correction_age_s"] = *readback->correction_age_s;
      }
    } else {
      receiver["receiver_readback"] = "UNAVAILABLE";
    }
    if (have_link_status_ && now_s >= link_status_at_s_ &&
        now_s - link_status_at_s_ <= gnss_report_max_age_s_) {
      transport = {{"px4_rtcm_chunks_accepted", link_status_.rtcm_chunks_accepted},
                   {"px4_rtcm_chunks_dropped", link_status_.rtcm_chunks_dropped},
                   {"dds_session_alive", link_status_.session_alive},
                   {"dds_handshake_ok", link_status_.handshake_ok}};
    } else {
      transport = {{"px4_rtcm_chunks_accepted", nullptr},
                   {"px4_rtcm_chunks_dropped", nullptr},
                   {"dds_session_alive", false},
                   {"dds_handshake_ok", false}};
    }
  }
  transport["selected"] = config.at("transport");
  transport["ready"] = active;
  transport["frames_delivered"] = sink.frames_delivered;
  transport["bytes_delivered"] = sink.bytes_delivered;
  transport["failures"] = sink.failures;
  transport["opens"] = sink.opens;
  transport["reopens"] = sink.reopens;
  transport["transport_delivery_age_s"] = age_json(sink.last_delivery_s, now_s);
  Json state, events;
  {
    std::lock_guard<std::mutex> lk(state_m_);
    state = state_name(state_.state());
    events = state_.events();
  }
  return {{"schema", 1},
          {"worker_state", state},
          {"desired_state", config.at("desired_state")},
          {"config_revision", config.at("revision")},
          {"worker_uptime_s", now_s - started_at_s_},
          {"source", source},
          {"transport", transport},
          {"receiver", receiver},
          {"events", events}};
}

}  // namespace dyx3_gnss_rtk
