// message_hash — PX4 uORB message format hash. See docs/contracts/dyx3_px4_link.md section 6.
// Pure C++, no ROS and no px4_msgs. Proven against the firmware's own Python implementation
// (tools/px4_msg_hash/, test/fixtures/px4_msg_hash_vectors.txt).
#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace dyx3_px4_link {

// FNV-1a 32-bit over raw bytes (offset 0x811c9dc5, prime 0x1000193).
uint32_t fnv1a32(std::string_view data);

struct MsgField {
  std::string type;  // as written in the .msg file, array suffix included ("float32[3]")
  std::string name;
};

// Fields of a .msg file in file order. Constants (any line containing '=') and comments are
// skipped.
std::vector<MsgField> parse_msg_fields(std::string_view text);

// Returns the .msg text of a bare message name (e.g. "EscReport"), or nullopt if unknown.
using MsgResolver = std::function<std::optional<std::string>(const std::string& name)>;

// Hash of message `name`. nullopt (with *error set) when a definition is missing or nesting is
// pathological (depth > 16). A missing definition is never skipped: it is an error.
std::optional<uint32_t> message_hash(const std::string& name, const MsgResolver& resolver,
                                     std::string* error = nullptr);

}  // namespace dyx3_px4_link
