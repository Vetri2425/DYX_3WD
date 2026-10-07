// sha256 — self-contained SHA-256 (FIPS 180-4) so the mission package needs no crypto dependency.
// Used ONLY to verify content-addressed path artifacts (docs/contracts/path_artifact.md); it is not
// a security boundary against a malicious local user, it is an integrity check.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace dyx3_mission {

/// Lowercase hex SHA-256 of `len` bytes.
std::string sha256_hex(const void* data, std::size_t len);
inline std::string sha256_hex(const std::string& s) { return sha256_hex(s.data(), s.size()); }

}  // namespace dyx3_mission
