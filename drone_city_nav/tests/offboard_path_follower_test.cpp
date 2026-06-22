#include "drone_city_nav/offboard_path_follower.hpp"

#include <gtest/gtest.h>

#include <numbers>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OffboardPathFollowerConfig testConfig() {
  OffboardPathFollowerConfig config{};
  config.acceptance_radius_m = 1.0;
  config.turn_preview_distance_m = 32.0;
  return config;
}

} // namespace

TEST(OffboardPathFollower, AdvancesWaypointAfterAcceptanceRadius) {
  const std::vector<Point2> path{{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}};

  const std::size_t index =
      advanceWaypointIndex(path, Point2{5.2, 0.0}, 0U, testConfig());

  EXPECT_EQ(index, 2U);
}

TEST(OffboardPathFollower, UpcomingTurnReportsDistanceAndAngle) {
  const std::vector<Point2> path{{0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}};

  const UpcomingTurn turn =
      upcomingTurnAtWaypoint(path, 1U, Point2{4.0, 0.0}, true, testConfig());

  ASSERT_TRUE(turn.valid);
  EXPECT_EQ(turn.waypoint_index, 1U);
  EXPECT_NEAR(turn.distance_to_turn_m, 1.0, 1.0e-9);
  EXPECT_NEAR(turn.angle_rad, std::numbers::pi / 2.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(turn.turn_point.x, 5.0);
  EXPECT_DOUBLE_EQ(turn.turn_point.y, 0.0);
}

TEST(OffboardPathFollower, UpcomingTurnUsesPathDistanceAcrossSegments) {
  const std::vector<Point2> path{{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {10.0, 5.0}};

  const UpcomingTurn turn =
      upcomingTurnAtWaypoint(path, 1U, Point2{4.0, 0.0}, true, testConfig());

  ASSERT_TRUE(turn.valid);
  EXPECT_EQ(turn.waypoint_index, 2U);
  EXPECT_NEAR(turn.distance_to_turn_m, 6.0, 1.0e-9);
  EXPECT_NEAR(turn.angle_rad, std::numbers::pi / 2.0, 1.0e-9);
}

TEST(OffboardPathFollower, UpcomingTurnIgnoresStraightPath) {
  const std::vector<Point2> path{{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}};

  const UpcomingTurn turn =
      upcomingTurnAtWaypoint(path, 1U, Point2{4.0, 0.0}, true, testConfig());

  EXPECT_FALSE(turn.valid);
  EXPECT_DOUBLE_EQ(turn.angle_rad, 0.0);
}

TEST(OffboardPathFollower, UpcomingTurnIgnoresDistantTurnOutsidePreview) {
  OffboardPathFollowerConfig config = testConfig();
  config.turn_preview_distance_m = 5.0;
  const std::vector<Point2> path{{0.0, 0.0}, {30.0, 0.0}, {30.0, 30.0}};

  const UpcomingTurn turn =
      upcomingTurnAtWaypoint(path, 1U, Point2{0.0, 0.0}, true, config);

  EXPECT_FALSE(turn.valid);
}

TEST(OffboardPathFollower, UpcomingTurnHandlesDegeneratePath) {
  const std::vector<Point2> path{{0.0, 0.0}, {5.0, 0.0}};

  const UpcomingTurn turn =
      upcomingTurnAtWaypoint(path, 1U, Point2{4.0, 0.0}, true, testConfig());

  EXPECT_FALSE(turn.valid);
}

} // namespace drone_city_nav
