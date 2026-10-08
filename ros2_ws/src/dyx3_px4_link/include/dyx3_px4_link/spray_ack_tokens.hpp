// Durable, non-wrapping PX4 source-system/component identities for spray ACKs.
#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace dyx3_px4_link {

class SprayAckTokens {
public:
  explicit SprayAckTokens(std::string state_path);

  struct Identity {
    uint8_t system;
    uint16_t component;
  };

  // Reserves and durably advances the pair before it may be published. Returns nullopt
  // permanently on corrupt/unwritable state or after the pair space is exhausted.
  std::optional<Identity> reserve();

  uint32_t used() const { return next_; }
  uint32_t remaining() const { return failed_ ? 0U : kCapacity - next_; }
  bool exhausted() const { return failed_ || next_ == kCapacity; }

  static constexpr uint16_t kFirst = 2;
  static constexpr uint16_t kLast = 999;  // 1000+ identifies PX4 mode executors.
  static constexpr uint32_t kCapacity = 255U * (kLast - kFirst + 1U);

private:
  bool load();
  bool persist(uint32_t next);

  std::string state_path_;
  bool failed_{false};
  uint32_t next_{0};
};

}  // namespace dyx3_px4_link
