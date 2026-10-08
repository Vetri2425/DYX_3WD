#pragma once

#include <functional>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "dyx3_gnss_rtk/lora_source.hpp"
#include "dyx3_gnss_rtk/ntrip_client.hpp"
#include "dyx3_gnss_rtk/transport_sink.hpp"

namespace dyx3_gnss_rtk {

using Json = nlohmann::json;

struct ConfigError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

class RtkConfigStore {
public:
  explicit RtkConfigStore(std::string directory) : directory_(std::move(directory)) {}

  // Missing config means first boot. The read-only NTRIP environment is imported once if it
  // contains a configured caster (upgrade -> DDS). Otherwise this is a fresh USB-default install.
  static Json initial_from_environment();
  // Fresh-install defaults (NTRIP + USB_DIRECT, nothing configured). Reads no environment.
  static Json defaults();
  std::optional<Json> load() const;
  void save(const Json& config, const std::function<void()>& before_rename = {}) const;

  static void validate(const Json& config);
  static Json public_view(const Json& config);
  static Json merge_write_only_passwords(const Json& old_config, Json candidate);
  static NtripConfig ntrip_config(const Json& config);
  static LoraConfig lora_config(const Json& config);
  static UsbConfig usb_config(const Json& config);
  const std::string& directory() const { return directory_; }

private:
  std::string directory_;
};

}  // namespace dyx3_gnss_rtk
