#include "dyx3_px4_link/spray_ack_tokens.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <fstream>
#include <string>
#include <vector>

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

TEST(SprayAckTokens, ReservesEveryIdentityOnceThenFailsClosedAtBoundary) {
  const std::string path = temp_state_path();
  write_state(path, "2\n");
  dyx3_px4_link::SprayAckTokens tokens(path);
  std::vector<uint16_t> allocated;
  while (const auto token = tokens.reserve()) allocated.push_back(*token);

  ASSERT_EQ(allocated.size(), 998U);
  EXPECT_EQ(allocated.front(), 2U);
  EXPECT_EQ(allocated.back(), 999U);
  EXPECT_FALSE(tokens.reserve());
  unlink(path.c_str());
}

TEST(SprayAckTokens, OldestTokenCannotBeReusedAfterBoundary) {
  const std::string path = temp_state_path();
  write_state(path, "2\n");
  dyx3_px4_link::SprayAckTokens tokens(path);
  std::vector<uint16_t> allocated;
  while (const auto token = tokens.reserve()) allocated.push_back(*token);

  const uint16_t delayed_oldest_ack_target_component = allocated.front();
  EXPECT_EQ(delayed_oldest_ack_target_component, 2U);
  EXPECT_FALSE(tokens.reserve());  // It cannot become a current transaction's correlation ID.
  unlink(path.c_str());
}

TEST(SprayAckTokens, DurableHighWaterMarkSurvivesProcessRestart) {
  const std::string path = temp_state_path();
  write_state(path, "2\n");
  {
    dyx3_px4_link::SprayAckTokens first_process(path);
    EXPECT_EQ(first_process.reserve(), 2U);
    EXPECT_EQ(first_process.reserve(), 3U);
  }
  dyx3_px4_link::SprayAckTokens restarted_process(path);
  EXPECT_EQ(restarted_process.reserve(), 4U);
  unlink(path.c_str());
}

TEST(SprayAckTokens, CorruptStateFailsClosed) {
  const std::string path = temp_state_path();
  write_state(path, "2\n999\n");
  dyx3_px4_link::SprayAckTokens tokens(path);
  EXPECT_FALSE(tokens.reserve());
  unlink(path.c_str());
}
}  // namespace
