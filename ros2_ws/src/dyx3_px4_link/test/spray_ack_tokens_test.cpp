#include "dyx3_px4_link/spray_ack_tokens.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <fstream>
#include <string>

namespace {
std::string temp_state_path() {
  static std::atomic<unsigned> serial{0};
  return "/tmp/dyx3_spray_ack_tokens_" + std::to_string(getpid()) + "_" +
         std::to_string(serial.fetch_add(1));
}

void write_state(const std::string& path, const std::string& value) {
  std::ofstream out(path);
  out << value;
}
}  // namespace

TEST(SprayAckTokens, PairOrderingAndBoundaryFailClosed) {
  const std::string path = temp_state_path();
  write_state(path, "2\n");  // legacy first-install ledger
  dyx3_px4_link::SprayAckTokens tokens(path);
  const auto first = tokens.reserve();
  ASSERT_TRUE(first);
  EXPECT_EQ(first->system, 1U);
  EXPECT_EQ(first->component, 2U);
  write_state(path, "v2 254488\n");
  dyx3_px4_link::SprayAckTokens boundary(path);
  const auto penultimate = boundary.reserve();
  const auto last = boundary.reserve();
  ASSERT_TRUE(penultimate);
  ASSERT_TRUE(last);
  EXPECT_EQ(penultimate->system, 255U);
  EXPECT_EQ(penultimate->component, 998U);
  EXPECT_EQ(last->system, 255U);
  EXPECT_EQ(last->component, 999U);
  EXPECT_EQ(boundary.used(), dyx3_px4_link::SprayAckTokens::kCapacity);
  EXPECT_EQ(boundary.remaining(), 0U);
  EXPECT_TRUE(boundary.exhausted());
  EXPECT_FALSE(boundary.reserve());
  unlink(path.c_str());
}

TEST(SprayAckTokens, LegacyHighWaterAndProcessRestartNeverReuseAPair) {
  const std::string path = temp_state_path();
  write_state(path, "999\n");
  {
    dyx3_px4_link::SprayAckTokens first(path);
    const auto pair = first.reserve();
    ASSERT_TRUE(pair);
    EXPECT_EQ(pair->system, 1U);
    EXPECT_EQ(pair->component, 999U);
  }
  dyx3_px4_link::SprayAckTokens restarted(path);
  const auto pair = restarted.reserve();
  ASSERT_TRUE(pair);
  EXPECT_EQ(pair->system, 2U);
  EXPECT_EQ(pair->component, 2U);
  unlink(path.c_str());
}

TEST(SprayAckTokens, MissingOrCorruptStateFailsClosed) {
  const std::string path = temp_state_path();
  dyx3_px4_link::SprayAckTokens missing(path);
  EXPECT_FALSE(missing.reserve());
  write_state(path, "v2 254491\n");
  dyx3_px4_link::SprayAckTokens corrupt(path);
  EXPECT_FALSE(corrupt.reserve());
  unlink(path.c_str());
}

TEST(SprayAckTokens, ExhaustedStateSurvivesRestart) {
  const std::string path = temp_state_path();
  write_state(path, "v2 254490\n");
  dyx3_px4_link::SprayAckTokens tokens(path);
  EXPECT_FALSE(tokens.reserve());
  EXPECT_TRUE(tokens.exhausted());
  unlink(path.c_str());
}
