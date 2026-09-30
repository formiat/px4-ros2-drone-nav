#include "drone_city_nav/dead_reckoning_landing.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {

constexpr std::int64_t kSecondNs{1'000'000'000};

TEST(DeadReckoningLanding, LandsOnceTheSourceHasDeadReckonedThreeSeconds) {
  DeadReckoningLanding landing;
  EXPECT_FALSE(landing.due(100 * kSecondNs));

  landing.observe(10 * kSecondNs, false);
  EXPECT_FALSE(landing.due(100 * kSecondNs));

  landing.observe(20 * kSecondNs, true);
  landing.observe(21 * kSecondNs, true);
  EXPECT_FALSE(landing.due(22 * kSecondNs + kSecondNs / 2));
  EXPECT_TRUE(landing.due(23 * kSecondNs));
}

TEST(DeadReckoningLanding, ASourceThatSeesAgainTakesTheFlightBack) {
  DeadReckoningLanding landing;
  landing.observe(20 * kSecondNs, true);
  ASSERT_TRUE(landing.due(24 * kSecondNs));

  landing.observe(24 * kSecondNs, false);
  EXPECT_FALSE(landing.due(30 * kSecondNs));

  // The next outage is counted from its own start.
  landing.observe(40 * kSecondNs, true);
  EXPECT_FALSE(landing.due(41 * kSecondNs));
  EXPECT_TRUE(landing.due(43 * kSecondNs));
}

TEST(DeadReckoningLanding, ASilentSourceIsStillDeadReckoning) {
  DeadReckoningLanding landing;
  landing.observe(20 * kSecondNs, true);
  EXPECT_TRUE(landing.due(500 * kSecondNs));
}

} // namespace drone_city_nav
