// Message hash vs vectors produced by the FIRMWARE's own Python (tools/px4_msg_hash/).
#include "dyx3_px4_link/message_hash.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>

using namespace dyx3_px4_link;

namespace {
std::string slurp(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}
std::map<std::string, uint32_t> vectors() {
  std::map<std::string, uint32_t> m;
  std::ifstream f(std::string(DYX3_FIXTURES) + "/px4_msg_hash_vectors.txt");
  std::string name;
  uint64_t h;
  while (f >> name >> h) m[name] = static_cast<uint32_t>(h);
  return m;
}
MsgResolver dir_resolver(const std::string& dir) {
  return [dir](const std::string& n) -> std::optional<std::string> {
    std::ifstream f(dir + "/" + n + ".msg", std::ios::binary);
    if (!f) return std::nullopt;
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
  };
}
}  // namespace

TEST(Fnv1a, KnownValues) {
  EXPECT_EQ(fnv1a32(""), 0x811c9dc5u);
  EXPECT_EQ(fnv1a32("a"), 0xe40c292cu);
  EXPECT_EQ(fnv1a32("foobar"), 0xbf9cf968u);
}

TEST(Parse, SkipsConstantsAndComments) {
  const auto f = parse_msg_fields(
      "uint32 MESSAGE_VERSION = 1\n# note\nuint64 timestamp # us\nfloat32[3] v # m = x\n\nbool "
      "b\n");
  ASSERT_EQ(f.size(), 3U);
  EXPECT_EQ(f[0].type, "uint64");
  EXPECT_EQ(f[1].type, "float32[3]");
  EXPECT_EQ(f[1].name, "v");
  EXPECT_EQ(f[2].name, "b");
}

TEST(MessageHash, FixtureSetMatchesFirmware) {
  const auto vec = vectors();
  ASSERT_GE(vec.size(), 200U);
  const auto res = dir_resolver(std::string(DYX3_FIXTURES) + "/msgdefs");
  int checked = 0;
  for (const auto& kv : vec) {
    if (!res(kv.first)) continue;  // only the messages carried in fixtures/msgdefs
    std::string err;
    const auto h = message_hash(kv.first, res, &err);
    ASSERT_TRUE(h.has_value()) << kv.first << ": " << err;
    EXPECT_EQ(*h, kv.second) << kv.first;
    ++checked;
  }
  // Includes nested cases: EscStatus (EscReport[8]), PositionSetpointTriplet, ArmingCheckReply.
  EXPECT_GE(checked, 20);
}

TEST(MessageHash, NestedTypesAreExpanded) {
  const auto res = dir_resolver(std::string(DYX3_FIXTURES) + "/msgdefs");
  const auto vec = vectors();
  for (const char* n : {"EscStatus", "PositionSetpointTriplet", "ArmingCheckReply"}) {
    const auto h = message_hash(n, res);
    ASSERT_TRUE(h.has_value()) << n;
    EXPECT_EQ(*h, vec.at(n)) << n;
  }
}

TEST(MessageHash, MissingDefinitionIsAnError) {
  const auto res = dir_resolver(std::string(DYX3_FIXTURES) + "/msgdefs");
  std::string err;
  EXPECT_FALSE(message_hash("NoSuchMessage", res, &err).has_value());
  EXPECT_FALSE(err.empty());
  // a nested type that cannot be resolved is not skipped silently
  const auto partial = [&res](const std::string& n) -> std::optional<std::string> {
    if (n == "EscReport") return std::nullopt;
    return res(n);
  };
  err.clear();
  EXPECT_FALSE(message_hash("EscStatus", partial, &err).has_value());
  EXPECT_NE(err.find("EscReport"), std::string::npos);
}

TEST(MessageHash, SelfNestingTerminates) {
  const auto res = [](const std::string&) -> std::optional<std::string> { return "Loop x\n"; };
  std::string err;
  EXPECT_FALSE(message_hash("Loop", res, &err).has_value());
  EXPECT_NE(err.find("nesting"), std::string::npos);
}

// Whole pinned set. Skipped (not passed) unless DYX3_PX4_MSGS_SRC points at <px4_msgs>/msg.
TEST(MessageHash, WholePinnedSetMatchesFirmware) {
  const char* dir = std::getenv("DYX3_PX4_MSGS_SRC");
  if (dir == nullptr) GTEST_SKIP() << "DYX3_PX4_MSGS_SRC not set: whole-set comparison not run";
  const auto res = dir_resolver(dir);
  int n = 0;
  for (const auto& kv : vectors()) {
    const auto h = message_hash(kv.first, res);
    ASSERT_TRUE(h.has_value()) << kv.first;
    EXPECT_EQ(*h, kv.second) << kv.first;
    ++n;
  }
  EXPECT_EQ(n, 235);
}
