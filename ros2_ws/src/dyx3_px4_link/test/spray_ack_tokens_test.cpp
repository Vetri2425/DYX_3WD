#include "dyx3_px4_link/spray_ack_tokens.hpp"

#include <gtest/gtest.h>
#include <sys/stat.h>
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
  // The first process persisted a whole block (index 997 + 64 = 1061): the restart resumes there,
  // beyond every pair the first process could have handed out.
  dyx3_px4_link::SprayAckTokens restarted(path);
  const auto pair = restarted.reserve();
  ASSERT_TRUE(pair);
  EXPECT_EQ(pair->system, 2U);
  EXPECT_EQ(pair->component, 1061U - 998U + 2U);
  unlink(path.c_str());
}

// PXL-004: identities are persisted in blocks, not once per transaction.
TEST(SprayAckTokens, BlockReservationPersistsOncePerBlockAndRestartResumesBeyondIt) {
  using dyx3_px4_link::SprayAckTokens;
  const std::string path = temp_state_path();
  write_state(path, "v2 0\n");
  uint32_t last_index = 0;
  {
    SprayAckTokens tokens(path);
    for (uint32_t i = 0; i < 200; ++i) {
      const auto pair = tokens.reserve();
      ASSERT_TRUE(pair) << i;
      last_index = (pair->system - 1U) * (SprayAckTokens::kLast - SprayAckTokens::kFirst + 1U) +
                   (pair->component - SprayAckTokens::kFirst);
      ASSERT_EQ(last_index, i);
    }
    EXPECT_LE(tokens.persist_count(), 4U);
    EXPECT_EQ(tokens.used(), 200U);
    std::ifstream in(path);
    std::string text;
    std::getline(in, text);
    EXPECT_EQ(text, "v2 256");  // the persisted mark is ahead of every pair handed out
  }
  SprayAckTokens restarted(path);  // e.g. after a power loss: resume at the persisted mark
  EXPECT_EQ(restarted.used(), 256U);
  const auto pair = restarted.reserve();
  ASSERT_TRUE(pair);
  EXPECT_EQ(pair->system, 1U);
  EXPECT_EQ(pair->component, 256U + SprayAckTokens::kFirst);
  EXPECT_GT(256U, last_index);
  EXPECT_EQ(restarted.persist_count(), 1U);
  unlink(path.c_str());
}

TEST(SprayAckTokens, BlockPersistFailureFailsClosedPermanently) {
  const std::string path = temp_state_path();
  write_state(path, "v2 0\n");
  const std::string blocker = path + ".tmp";  // an existing temp path makes the next persist fail
  {
    dyx3_px4_link::SprayAckTokens tokens(path);
    for (uint32_t i = 0; i < dyx3_px4_link::SprayAckTokens::kBlock; ++i)
      ASSERT_TRUE(tokens.reserve()) << i;
    ASSERT_EQ(mkdir(blocker.c_str(), 0700), 0);
    EXPECT_FALSE(tokens.reserve());  // the next block cannot be made durable: nothing handed out
    EXPECT_TRUE(tokens.exhausted());
    EXPECT_EQ(tokens.remaining(), 0U);
    ASSERT_EQ(rmdir(blocker.c_str()), 0);
    EXPECT_FALSE(tokens.reserve());  // latched until restart
  }
  ASSERT_EQ(mkdir(blocker.c_str(), 0700), 0);
  dyx3_px4_link::SprayAckTokens first_block(path);
  EXPECT_FALSE(first_block.reserve());  // the very first block of a process also fails closed
  ASSERT_EQ(rmdir(blocker.c_str()), 0);
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
