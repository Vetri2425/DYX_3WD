// path_artifact — C++ reader for the DYX3PATH 1 artifact. See docs/contracts/path_artifact.md.
//
// The Python backend (dyx3_backend.mission.path_artifact) is the writer and enforces canonical
// form before storing. The reader here verifies the content hash and parses at least as strictly as
// the Python decode(): coordinates must be spelled exactly as Python's repr() writes them
// (shortest round-trip digits, see python_repr in path_artifact.cpp), and the opaque `meta` line
// must be the canonical JSON object json.dumps(sort_keys, compact, ensure_ascii) would produce.
// The reader is deliberately a little stricter in places (control bytes, empty engine id,
// flags spelling, JSON nesting depth).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace dyx3_mission {

constexpr std::uint8_t kFlagSpray = 1;
constexpr std::uint8_t kFlagMustHit = 2;

struct ArtifactPoint {
  double north_m = 0.0;
  double east_m = 0.0;
  std::uint8_t flags = 0;
  bool spray() const { return (flags & kFlagSpray) != 0; }
  bool must_hit() const { return (flags & kFlagMustHit) != 0; }
};

struct PathArtifact {
  std::string sha256;
  int version = 0;
  std::string engine_id;
  std::string meta_json;  ///< opaque to C++ (covered by the hash)
  std::vector<ArtifactPoint> points;
};

struct ArtifactResult {
  bool ok = false;
  std::string error;
  PathArtifact artifact;
};

/// Python repr(float) spelling of a finite double, the only coordinate spelling the reader accepts.
/// For C++ writers of DYX3PATH artifacts (tests, tools); empty for a non-finite value.
std::string python_repr(double v);

/// Parse artifact bytes. When `expected_sha256` is non-empty the bytes must hash to it.
ArtifactResult parse_artifact(const std::string& bytes, const std::string& expected_sha256 = "");

/// Largest artifact file the reader will load. A planned path is a few MB at most (the backend caps
/// a mission at 50k points; its upload limit is 20 MiB); this bound keeps a wrong or hostile file
/// from stalling the Start service callback in read + SHA-256 or exhausting memory. DERIVED.
constexpr std::uintmax_t kMaxArtifactBytes = 64ULL * 1024 * 1024;

/// Read `<dir>/<sha256>.dyx3path` and verify it hashes to its own name. The file must be a regular
/// file of at most `max_bytes`; the size is checked BEFORE anything is read.
ArtifactResult load_artifact(const std::string& dir, const std::string& sha256,
                             std::uintmax_t max_bytes = kMaxArtifactBytes);

struct ConditionedRunArtifact {
  struct Point {
    double north_m{0.0};
    double east_m{0.0};
  };
  std::vector<Point> points;
  std::vector<std::uint8_t> flags;
  std::vector<std::uint8_t> must_hit;
  std::uint8_t profile{0};
};
struct ConditionedArtifact {
  std::string sha256;
  std::string source_sha256;
  std::string conditioner_config;
  std::vector<ConditionedRunArtifact> runs;
};
struct ConditionedResult {
  bool ok{false};
  std::string error;
  ConditionedArtifact artifact;
};
std::string serialize_conditioned_artifact(const std::string& source_sha256,
                                           const std::string& conditioner_config,
                                           const std::vector<ConditionedRunArtifact>& runs);
ConditionedResult parse_conditioned_artifact(const std::string& bytes,
                                             const std::string& expected_sha256 = "");
ConditionedResult load_conditioned_artifact(const std::string& dir, const std::string& sha256);

}  // namespace dyx3_mission
