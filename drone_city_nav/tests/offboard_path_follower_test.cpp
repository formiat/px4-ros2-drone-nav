#include "drone_city_nav/offboard_path_follower.hpp"

#include <gtest/gtest.h>

#include <numbers>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OffboardPathFollowerConfig testConfig() {
  OffboardPathFollowerConfig config{};
  config.acceptance_radius_m = 1.0;
  config.turn_slowdown_preview_distance_m = 32.0;
  config.path_switch_hysteresis_m = 2.0;
  config.path_continuity_reuse_radius_m = 5.0;
  config.path_continuity_max_target_distance_m = 30.0;
  return config;
}

} // namespace

TEST(OffboardPathFollower, ClosestWaypointReturnsNearestPathPoint) {
  const std::vector<Point2> path{{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}};

  const std::size_t index = closestWaypointIndex(path, Point2{4.8, 0.3});

  EXPECT_EQ(index, 1U);
}

TEST(OffboardPathFollower, ContinuityKeepsNearPreviousTarget) {
  const std::vector<Point2> path{{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}, {30.0, 0.0}};

  const std::size_t index = continuityWaypointIndex(
      path, Point2{9.0, 0.0}, Point2{10.2, 0.1}, 3U, true, testConfig());

  EXPECT_EQ(index, 1U);
}

TEST(OffboardPathFollower, AdvancesWaypointAfterAcceptanceRadius) {
  const std::vector<Point2> path{{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}};

  const std::size_t index =
      advanceWaypointIndex(path, Point2{5.2, 0.0}, 0U, testConfig());

  EXPECT_EQ(index, 2U);
}

TEST(OffboardPathFollower, PathTurnAngleUsesNearbyWaypoint) {
  const std::vector<Point2> path{{0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}};

  const double angle =
      pathTurnAngleAtWaypoint(path, 1U, Point2{4.0, 0.0}, true, testConfig());

  EXPECT_NEAR(angle, std::numbers::pi / 2.0, 1.0e-9);
}

TEST(OffboardPathFollower, PathTurnAngleUsesConfiguredPreviewDistance) {
  const std::vector<Point2> path{{0.0, 0.0}, {30.0, 0.0}, {30.0, 30.0}};

  const double angle =
      pathTurnAngleAtWaypoint(path, 1U, Point2{0.0, 0.0}, true, testConfig());

  EXPECT_NEAR(angle, std::numbers::pi / 2.0, 1.0e-9);
}

TEST(OffboardPathFollower, PathTurnAngleIgnoresDistantWaypointOutsidePreview) {
  OffboardPathFollowerConfig config = testConfig();
  config.turn_slowdown_preview_distance_m = 10.0;
  const std::vector<Point2> path{{0.0, 0.0}, {30.0, 0.0}, {30.0, 30.0}};

  const double angle =
      pathTurnAngleAtWaypoint(path, 1U, Point2{0.0, 0.0}, true, config);

  EXPECT_DOUBLE_EQ(angle, 0.0);
}

} // namespace drone_city_nav
