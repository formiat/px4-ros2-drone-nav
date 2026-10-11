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
  BlindDescentReflex reflex;
  const Point2 velocity = reflex.observe(TofRingRanges{.front_m = 2.5,
                                                       .back_m = std::nullopt,
                                                       .left_m = 0.8,
                                                       .right_m = 2.0,
                                                       .down_m = std::nullopt},
                                         0, 0.0);
  EXPECT_NEAR(velocity.x, 0.0, 1.0e-9);
  EXPECT_NEAR(velocity.y, 0.3, 1.0e-9);
}

TEST(BlindDescentReflex, TheVelocityFollowsTheHeading) {
  BlindDescentReflex reflex;
  const Point2 velocity = reflex.observe(TofRingRanges{.front_m = 0.9,
                                                       .back_m = 2.5,
                                                       .left_m = 2.5,
                                                       .right_m = 2.5,
                                                       .down_m = std::nullopt},
                                         0, 0.5 * std::numbers::pi);
  // Backward in the body is south when the vehicle faces east.
  EXPECT_NEAR(velocity.x, 0.0, 1.0e-9);
  EXPECT_NEAR(velocity.y, -0.3, 1.0e-9);
}

TEST(BlindDescentReflex, FacingSurfacesCancelAndFarOnesCount) {
  BlindDescentReflex reflex;
  const Point2 both = reflex.observe(TofRingRanges{.front_m = 2.5,
                                                   .back_m = 2.5,
                                                   .left_m = 0.7,
                                                   .right_m = 0.9,
                                                   .down_m = std::nullopt},
                                     0, 1.0);
  EXPECT_NEAR(both.x, 0.0, 1.0e-9);
  EXPECT_NEAR(both.y, 0.0, 1.0e-9);
  const Point2 none = reflex.observe(TofRingRanges{.front_m = 1.3,
                                                   .back_m = std::nullopt,
                                                   .left_m = 2.8,
                                                   .right_m = 2.8,
                                                   .down_m = std::nullopt},
                                     kSecondNs, 1.0);
  EXPECT_NEAR(none.x, 0.0, 1.0e-9);
  EXPECT_NEAR(none.y, 0.0, 1.0e-9);
}

// Specification K19: within half a metre of the floor the down unit reads the
// push stops, whatever a lateral unit says (a sloped floor closes its range
// by the descent alone, r1267, r1268); higher up it resumes afresh.
TEST(BlindDescentReflex, NoPushOverANearFloor) {
  BlindDescentReflex reflex;
  TofRingRanges ranges{
      .front_m = 0.8, .back_m = 2.5, .left_m = 2.5, .right_m = 2.5, .down_m = 0.5};
  const Point2 near_floor = reflex.observe(ranges, 0, 0.0);
  EXPECT_NEAR(near_floor.x, 0.0, 1.0e-9);
  EXPECT_NEAR(near_floor.y, 0.0, 1.0e-9);
  ranges.down_m = 2.0;
  EXPECT_NEAR(reflex.observe(ranges, kSecondNs, 0.0).x, -0.3, 1.0e-9);
}

// Specification K19: the push is closed on the range the unit reads. A range
// that does not open (the drift of the estimate the autopilot's velocity loop
// runs on, r1237 to r1240) raises the push by a metre per second for each
// metre of lag; one that opens as asked keeps it at 0.3 m/s; one that opens
// faster lowers it and never pulls toward the surface; the push is bounded.
TEST(BlindDescentReflex, ThePushFollowsTheRangeItReads) {
  BlindDescentReflex still;
  const TofRingRanges left_near{.front_m = 2.5,
                                .back_m = 2.5,
                                .left_m = 0.8,
                                .right_m = 2.5,
                                .down_m = std::nullopt};
  EXPECT_NEAR(still.observe(left_near, 0, 0.0).y, 0.3, 1.0e-9);
  EXPECT_NEAR(still.observe(left_near, kSecondNs, 0.0).y, 0.6, 1.0e-9);
  EXPECT_NEAR(still.observe(left_near, 5 * kSecondNs, 0.0).y, 1.0, 1.0e-9);

  BlindDescentReflex opening;
  EXPECT_NEAR(opening.observe(left_near, 0, 0.0).y, 0.3, 1.0e-9);
  TofRingRanges opened = left_near;
  opened.left_m = 1.1;
  EXPECT_NEAR(opening.observe(opened, kSecondNs, 0.0).y, 0.3, 1.0e-9);
  opened.left_m = 1.19;
  EXPECT_NEAR(opening.observe(opened, kSecondNs + kSecondNs / 10, 0.0).y, 0.24, 1.0e-9);

  BlindDescentReflex fast;
  EXPECT_NEAR(fast.observe(left_near, 0, 0.0).y, 0.3, 1.0e-9);
  opened.left_m = 1.19;
  EXPECT_NEAR(fast.observe(opened, kSecondNs / 2, 0.0).y, 0.06, 1.0e-9);
  opened.left_m = 1.15;
  fast.reset();
  // After a reset the push begins afresh from the range read.
  EXPECT_NEAR(fast.observe(opened, 3 * kSecondNs, 0.0).y, 0.3, 1.0e-9);
}

} // namespace drone_city_nav
