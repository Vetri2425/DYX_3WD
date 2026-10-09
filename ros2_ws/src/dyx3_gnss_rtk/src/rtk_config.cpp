#include "dyx3_gnss_rtk/rtk_config.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <set>
#include <stdexcept>

#include "dyx3_gnss_rtk/serial_port.hpp"

namespace dyx3_gnss_rtk {
namespace {

std::string env(const char* name) {
  const char* value = std::getenv(name);
  return value ? std::string(value) : std::string();
}

void need(bool ok, const char* reason) {
  if (!ok) throw ConfigError(reason);
}

bool clean_text(const std::string& value, size_t max_size = 255) {
  if (value.empty() || value.size() > max_size) return false;
  for (const unsigned char c : value)
    if (c < 0x20 || c == 0x7f) return false;
  return true;
}

void keys(const Json& object, const std::set<std::string>& allowed) {
  need(object.is_object(), "configuration object expected");
  for (auto it = object.begin(); it != object.end(); ++it)
    need(allowed.count(it.key()) > 0, "unknown configuration field");
}

double positive(const Json& object, const char* key) {
  need(object.contains(key) && object.at(key).is_number(), "positive number required");
  const double value = object.at(key).get<double>();
  need(std::isfinite(value) && value > 0.0 && value <= 3600.0, "number out of range");
  return value;
}

void unset_or_positive(const Json& object, const char* key, bool configured) {
  if (configured) {
    positive(object, key);
    return;
  }
  need(object.contains(key) && object.at(key).is_number(), "number required");
  const double value = object.at(key).get<double>();
  need(std::isfinite(value) && value >= 0.0 && value <= 3600.0, "number out of range");
}

std::string required_string(const Json& object, const char* key, size_t max_size = 255) {
  need(object.contains(key) && object.at(key).is_string(), "string required");
  const std::string value = object.at(key).get<std::string>();
  need(clean_text(value, max_size), "string empty or contains control characters");
  return value;
}

const Json& active_profile(const Json& config) {
  const std::string id = config.at("ntrip").at("active_profile_id").get<std::string>();
  for (const auto& profile : config.at("ntrip").at("profiles"))
    if (profile.at("id") == id) return profile;
  throw ConfigError("active NTRIP profile not found");
}

}  // namespace

Json RtkConfigStore::defaults() {
  return {
      {"schema", 1},
      {"revision", 1},
      {"updated_at", ""},
      {"desired_state", "RUNNING"},
      {"source", "NTRIP"},
      {"transport", "USB_DIRECT"},
      {"ntrip", {{"active_profile_id", ""}, {"profiles", Json::array()}}},
      {"lora",
       {{"serial_device", ""}, {"baud", 0}, {"read_timeout_s", 0.0}, {"reopen_delay_s", 0.0}}},
      {"usb",
       {{"receiver_device", ""}, {"baud", 0}, {"write_timeout_s", 0.0}, {"reopen_delay_s", 0.0}}},
  };
}

Json RtkConfigStore::initial_from_environment() {
  Json config = defaults();
  const std::string host = env("DYX3_NTRIP_HOST");
  if (!host.empty()) {
    // Upgrade: the existing live NTRIP -> DDS installation retains its transport.
    config["transport"] = "PX4_DDS";
    config["ntrip"]["active_profile_id"] = "legacy";
    const std::string port = env("DYX3_NTRIP_PORT");
    int parsed_port = 2101;
    if (!port.empty()) {
      try {
        size_t used = 0;
        parsed_port = std::stoi(port, &used);
        need(used == port.size(), "invalid legacy NTRIP port");
      } catch (const std::exception&) {
        throw ConfigError("invalid legacy NTRIP port");
      }
    }
    config["ntrip"]["profiles"].push_back({
        {"id", "legacy"},
        {"name", "Imported NTRIP"},
        {"host", host},
        {"port", parsed_port},
        {"mountpoint", env("DYX3_NTRIP_MOUNTPOINT")},
        {"username", env("DYX3_NTRIP_USER")},
        {"password", env("DYX3_NTRIP_PASSWORD")},
        {"security", env("DYX3_NTRIP_SECURITY")},
        {"ca_file", env("DYX3_NTRIP_CA_FILE")},
        {"connect_timeout_s", 10.0},
        {"stream_timeout_s", 10.0},
        {"gga_interval_s", 10.0},
        {"backoff_base_s", 5.0},
        {"backoff_max_s", 60.0},
    });
  }
  validate(config);
  return config;
}

void RtkConfigStore::validate(const Json& c) {
  keys(c, {"schema", "revision", "updated_at", "desired_state", "source", "transport", "ntrip",
           "lora", "usb"});
  need(c.contains("schema") && c.at("schema") == 1, "unsupported RTK config schema");
  need(c.contains("revision") && c.at("revision").is_number_integer() &&
           c.at("revision").get<uint64_t>() > 0,
       "invalid config revision");
  need(c.contains("updated_at") && c.at("updated_at").is_string(), "updated_at required");
  need(c.contains("desired_state") &&
           (c.at("desired_state") == "RUNNING" || c.at("desired_state") == "STOPPED"),
       "invalid desired_state");
  need(c.contains("source") && (c.at("source") == "NTRIP" || c.at("source") == "LORA"),
       "invalid source");
  need(c.contains("transport") &&
           (c.at("transport") == "USB_DIRECT" || c.at("transport") == "PX4_DDS"),
       "invalid transport");

  const auto& ntrip = c.at("ntrip");
  keys(ntrip, {"active_profile_id", "profiles"});
  need(ntrip.contains("active_profile_id") && ntrip.at("active_profile_id").is_string(),
       "active_profile_id required");
  need(ntrip.contains("profiles") && ntrip.at("profiles").is_array() &&
           ntrip.at("profiles").size() <= 16,
       "profiles must be an array of at most 16");
  std::set<std::string> ids;
  for (const auto& p : ntrip.at("profiles")) {
    keys(p, {"id", "name", "host", "port", "mountpoint", "username", "password", "security",
             "ca_file", "connect_timeout_s", "stream_timeout_s", "gga_interval_s", "backoff_base_s",
             "backoff_max_s"});
    const std::string id = required_string(p, "id", 64);
    for (const unsigned char ch : id)
      need(std::isalnum(ch) || ch == '-' || ch == '_', "invalid profile id");
    need(ids.insert(id).second, "duplicate profile id");
    required_string(p, "name", 80);
    required_string(p, "host");
    need(p.contains("port") && p.at("port").is_number_integer() && p.at("port").get<int>() > 0 &&
             p.at("port").get<int>() < 65536,
         "invalid NTRIP port");
    required_string(p, "mountpoint");
    required_string(p, "username");
    need(p.contains("password") && p.at("password").is_string() &&
             p.at("password").get<std::string>().size() <= 1024,
         "NTRIP password required");
    need(p.contains("security") && (p.at("security") == "PLAINTEXT" || p.at("security") == "TLS"),
         "NTRIP security must be PLAINTEXT or TLS");
    need(p.contains("ca_file") && p.at("ca_file").is_string(), "ca_file must be a string");
    const std::string ca_file = p.at("ca_file").get<std::string>();
    need(ca_file.empty() || (ca_file.front() == '/' && clean_text(ca_file, 255)),
         "ca_file must be an absolute path");
    positive(p, "connect_timeout_s");
    positive(p, "stream_timeout_s");
    positive(p, "gga_interval_s");
    need(positive(p, "backoff_base_s") <= positive(p, "backoff_max_s"),
         "invalid NTRIP backoff range");
  }
  const std::string active = ntrip.at("active_profile_id").get<std::string>();
  need(active.empty() || ids.count(active) == 1, "active NTRIP profile not found");

  const auto& lora = c.at("lora");
  keys(lora, {"serial_device", "baud", "read_timeout_s", "reopen_delay_s"});
  need(lora.contains("serial_device") && lora.at("serial_device").is_string(),
       "LoRa device must be a string");
  const std::string lora_path = lora.at("serial_device").get<std::string>();
  need(lora_path.empty() || stable_serial_path(lora_path),
       "LoRa requires a /dev/serial/by-id or by-path path");
  need(lora.contains("baud") && lora.at("baud").is_number_integer() &&
           (lora_path.empty() ? lora.at("baud").get<int>() == 0 ||
                                    supported_serial_baud(lora.at("baud").get<int>())
                              : supported_serial_baud(lora.at("baud").get<int>())),
       "unsupported LoRa baud");
  unset_or_positive(lora, "read_timeout_s", !lora_path.empty());
  unset_or_positive(lora, "reopen_delay_s", !lora_path.empty());

  const auto& usb = c.at("usb");
  keys(usb, {"receiver_device", "baud", "write_timeout_s", "reopen_delay_s"});
  need(usb.contains("receiver_device") && usb.at("receiver_device").is_string(),
       "USB device must be a string");
  const std::string usb_path = usb.at("receiver_device").get<std::string>();
  need(usb_path.empty() || stable_serial_path(usb_path),
       "USB requires a /dev/serial/by-id or by-path path");
  need(usb_path.empty() || usb_path != lora_path, "LoRa and USB may not use the same port");
  need(usb.contains("baud") && usb.at("baud").is_number_integer() &&
           (usb_path.empty()
                ? usb.at("baud").get<int>() == 0 || supported_serial_baud(usb.at("baud").get<int>())
                : supported_serial_baud(usb.at("baud").get<int>())),
       "unsupported USB baud");
  unset_or_positive(usb, "write_timeout_s", !usb_path.empty());
  unset_or_positive(usb, "reopen_delay_s", !usb_path.empty());
}

Json RtkConfigStore::public_view(const Json& config) {
  Json result = config;
  for (auto& profile : result["ntrip"]["profiles"]) {
    profile["password_set"] = !profile.at("password").get<std::string>().empty();
    profile.erase("password");
  }
  return result;
}

Json RtkConfigStore::merge_write_only_passwords(const Json& old_config, Json candidate) {
  if (!candidate.contains("ntrip") || !candidate.at("ntrip").is_object() ||
      !candidate.at("ntrip").contains("profiles") ||
      !candidate.at("ntrip").at("profiles").is_array())
    throw ConfigError("profiles array required");
  for (auto& profile : candidate["ntrip"]["profiles"]) {
    profile.erase("password_set");
    if (profile.contains("password")) continue;
    if (!profile.contains("id") || !profile.at("id").is_string()) continue;
    for (const auto& old : old_config.at("ntrip").at("profiles")) {
      if (old.at("id") == profile.at("id")) {
        profile["password"] = old.at("password");
        break;
      }
    }
  }
  return candidate;
}

NtripConfig RtkConfigStore::ntrip_config(const Json& c) {
  const auto& p = active_profile(c);
  NtripConfig out;
  out.host = p.at("host").get<std::string>();
  out.port = p.at("port").get<int>();
  out.mountpoint = p.at("mountpoint").get<std::string>();
  out.user = p.at("username").get<std::string>();
  out.password = p.at("password").get<std::string>();
  out.security = p.at("security") == "TLS" ? NtripSecurity::Tls : NtripSecurity::Plaintext;
  out.ca_file = p.at("ca_file").get<std::string>();
  out.connect_timeout_s = p.at("connect_timeout_s").get<double>();
  out.stream_timeout_s = p.at("stream_timeout_s").get<double>();
  out.gga_interval_s = p.at("gga_interval_s").get<double>();
  out.backoff_base_s = p.at("backoff_base_s").get<double>();
  out.backoff_max_s = p.at("backoff_max_s").get<double>();
  return out;
}

LoraConfig RtkConfigStore::lora_config(const Json& c) {
  const auto& l = c.at("lora");
  return {l.at("serial_device").get<std::string>(), l.at("baud").get<int>(),
          l.at("read_timeout_s").get<double>(), l.at("reopen_delay_s").get<double>()};
}

UsbConfig RtkConfigStore::usb_config(const Json& c) {
  const auto& u = c.at("usb");
  return {u.at("receiver_device").get<std::string>(), u.at("baud").get<int>(),
          u.at("write_timeout_s").get<double>(), u.at("reopen_delay_s").get<double>()};
}

std::optional<Json> RtkConfigStore::load() const {
  std::ifstream input(directory_ + "/config.json");
  if (!input.is_open()) {
    if (errno == ENOENT) return std::nullopt;
    throw ConfigError("cannot read RTK configuration");
  }
  Json config;
  try {
    input >> config;
    validate(config);
  } catch (const std::exception&) {
    throw ConfigError("RTK configuration is invalid");  // never echo secret-containing input
  }
  return config;
}

void RtkConfigStore::save(const Json& config, const std::function<void()>& before_rename) const {
  validate(config);
  const int dir = ::open(directory_.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
  if (dir < 0) throw ConfigError("cannot open RTK state directory");
  static std::atomic<uint64_t> sequence{0};
  const std::string temp =
      "config.json.tmp." + std::to_string(::getpid()) + "." + std::to_string(++sequence);
  int fd = ::openat(dir, temp.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (fd < 0) {
    ::close(dir);
    throw ConfigError("cannot create RTK temporary config");
  }
  bool renamed = false;
  try {
    const std::string bytes = config.dump(2) + "\n";
    size_t done = 0;
    while (done < bytes.size()) {
      const ssize_t n = ::write(fd, bytes.data() + done, bytes.size() - done);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) throw ConfigError("cannot write RTK config");
      done += static_cast<size_t>(n);
    }
    if (::fsync(fd) != 0) throw ConfigError("cannot sync RTK config");
    const int close_result = ::close(fd);
    fd = -1;
    if (close_result != 0) throw ConfigError("cannot close RTK config");
    if (before_rename) before_rename();
    if (::renameat(dir, temp.c_str(), dir, "config.json") != 0)
      throw ConfigError("cannot replace RTK config");
    renamed = true;
    if (::fsync(dir) != 0) throw ConfigError("cannot sync RTK state directory");
  } catch (...) {
    if (fd >= 0) ::close(fd);
    if (!renamed) ::unlinkat(dir, temp.c_str(), 0);
    ::close(dir);
    throw;
  }
  ::close(dir);
}

}  // namespace dyx3_gnss_rtk
