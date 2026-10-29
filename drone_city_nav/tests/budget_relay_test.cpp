// The budget relay's table and its reading of stamps off serialized bytes (roadmap item
// 36, K53).
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "budget_relay.hpp"

namespace {

using namespace drone_city_nav::budget_relay;

std::vector<std::uint8_t> headerFirstBytes(const std::int32_t seconds,
                                           const std::uint32_t nanoseconds) {
  std::vector<std::uint8_t> bytes(16U, 0U);
  bytes[1] = 1U; // CDR little-endian encapsulation
  std::memcpy(&bytes[4], &seconds, sizeof(seconds));
  std::memcpy(&bytes[8], &nanoseconds, sizeof(nanoseconds));
  return bytes;
}

TEST(BudgetRelay, ReadsAHeadersStampAfterTheEncapsulation) {
  const auto bytes = headerFirstBytes(1234, 567'000'001U);
  EXPECT_EQ(stampNsOf(bytes.data(), bytes.size(), true), 1'234'567'000'001LL);
  EXPECT_FALSE(stampNsOf(bytes.data(), 8U, true).has_value());
}

TEST(BudgetRelay, ReadsTheAutopilotOdometrysSampleStamp) {
  std::vector<std::uint8_t> bytes(24U, 0U);
  const std::uint64_t timestamp = 99U;
  const std::uint64_t sample_us = 1'234'567U;
  std::memcpy(&bytes[4], &timestamp, sizeof(timestamp));
  std::memcpy(&bytes[12], &sample_us, sizeof(sample_us));
  EXPECT_EQ(stampNsOf(bytes.data(), bytes.size(), false), 1'234'567'000LL);
  EXPECT_FALSE(stampNsOf(bytes.data(), 16U, false).has_value());
}

TEST(BudgetRelay, EveryOutputIsBudgetedOrTheAutopilotsAndHasAnAge) {
  for (const Relayed& relayed : kRelayed) {
    const std::string target{relayed.target};
    EXPECT_TRUE(target.ends_with(std::string{kBudgetedSuffix}) ||
                target == std::string{kOdometry})
        << target;
    EXPECT_GT(relayed.stereo_age_ms + relayed.lidar_age_ms, 0.0) << relayed.name;
  }
  EXPECT_EQ(kRelayed[0].name, "depth_points");
  EXPECT_EQ(kRelayed[0].lidar_age_ms, 0.0) << "the lidar set has no depth returns";
}

} // namespace
