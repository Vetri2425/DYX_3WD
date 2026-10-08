// Durable, non-wrapping PX4 source_component identities for spray command ACKs.
#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace dyx3_px4_link {

class SprayAckTokens {
public:
  explicit SprayAckTokens(std::string state_path);

  // Reserves and durably advances the token before it may be published. Returns nullopt
  // permanently on corrupt/unwritable state or after the component-ID range is exhausted.
  std::optional<uint16_t> reserve();

  static constexpr uint16_t kFirst = 2;
  static constexpr uint16_t kLast = 999;  // 1000+ identifies PX4 mode executors.

private:
  bool load();
  bool persist(uint16_t next);

  std::string state_path_;
  bool initialized_{false};
  bool failed_{false};
  uint16_t next_{kFirst};
};

}  // namespace dyx3_px4_link
