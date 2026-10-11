#include "drone_city_nav/dead_reckoning_landing.hpp"

#include <gtest/gtest.h>

#include <numbers>
#include <optional>

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

TEST(BlindDescentReflex, ASurfaceOnTheLeftSendsTheVehicleRight) {
  const Point2 velocity = blindDescentReflexVelocityNed(
      TofRingRanges{
          .front_m = 2.5, .back_m = std::nullopt, .left_m = 0.8, .right_m = 2.0},
      0.0);
  EXPECT_NEAR(velocity.x, 0.0, 1.0e-9);
  EXPECT_NEAR(velocity.y, 0.3, 1.0e-9);
}

TEST(BlindDescentReflex, TheVelocityFollowsTheHeading) {
  const Point2 velocity = blindDescentReflexVelocityNed(
      TofRingRanges{.front_m = 0.9, .back_m = 2.5, .left_m = 2.5, .right_m = 2.5},
      0.5 * std::numbers::pi);
  // Backward in the body is south when the vehicle faces east.
  EXPECT_NEAR(velocity.x, 0.0, 1.0e-9);
  EXPECT_NEAR(velocity.y, -0.3, 1.0e-9);
}

TEST(BlindDescentReflex, FacingSurfacesCancelAndFarOnesCount) {
  const Point2 both = blindDescentReflexVelocityNed(
      TofRingRanges{.front_m = 2.5, .back_m = 2.5, .left_m = 0.7, .right_m = 0.9}, 1.0);
  EXPECT_NEAR(both.x, 0.0, 1.0e-9);
  EXPECT_NEAR(both.y, 0.0, 1.0e-9);
  const Point2 none = blindDescentReflexVelocityNed(
      TofRingRanges{
          .front_m = 1.3, .back_m = std::nullopt, .left_m = 2.8, .right_m = 2.8},
      1.0);
  EXPECT_NEAR(none.x, 0.0, 1.0e-9);
  EXPECT_NEAR(none.y, 0.0, 1.0e-9);
}

} // namespace drone_city_nav
