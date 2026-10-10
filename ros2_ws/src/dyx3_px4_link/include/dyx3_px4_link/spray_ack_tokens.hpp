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

  // Hands out the next pair. The persisted high-water mark is always beyond every pair handed out:
  // pairs are reserved durably in blocks of kBlock (one fsync per block, not per pair), so a
  // restart resumes at the persisted mark and skips the unused rest of the block, never reusing a
  // pair. Returns nullopt permanently on corrupt/unwritable state or after the pair space is
  // exhausted.
  std::optional<Identity> reserve();

  uint32_t used() const { return next_; }
  uint32_t remaining() const { return failed_ ? 0U : kCapacity - next_; }
  bool exhausted() const { return failed_ || next_ == kCapacity; }
  uint32_t persist_count() const { return persists_; }

  static constexpr uint16_t kFirst = 2;
  static constexpr uint16_t kLast = 999;  // 1000+ identifies PX4 mode executors.
  static constexpr uint32_t kCapacity = 255U * (kLast - kFirst + 1U);
  static constexpr uint32_t kBlock = 64;

private:
  bool load();
  bool persist(uint32_t next);

  std::string state_path_;
  bool failed_{false};
  uint32_t next_{0};      // next pair index to hand out
  uint32_t reserved_{0};  // persisted high-water mark: every index below it may have been used
  uint32_t persists_{0};
};

}  // namespace dyx3_px4_link
