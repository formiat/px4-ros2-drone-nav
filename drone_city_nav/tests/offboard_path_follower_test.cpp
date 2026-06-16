#include "drone_city_nav/offboard_path_follower.hpp"

#include <gtest/gtest.h>

#include <numbers>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OffboardPathFollowerConfig testConfig() {
  OffboardPathFollowerConfig config{};
  config.acceptance_radius_m = 1.0;
  config.lookahead_distance_m = 8.0;
  config.lookahead_time_s = 1.0;
  config.min_lookahead_distance_m = 8.0;
  config.max_lookahead_distance_m = 20.0;
  config.path_switch_hysteresis_m = 2.0;
  config.path_continuity_reuse_radius_m = 5.0;
  config.path_continuity_max_target_distance_m = 30.0;
  config.max_setpoint_distance_m = 4.0;
  config.dynamic_lookahead_enabled = true;
  return config;
}

} // namespace

TEST(OffboardPathFollower, EmptyPathHoldsCurrentPosition) {
  const Point2 current{3.0, -2.0};

  const Point2 target =
      lookaheadTargetOnPath(std::vector<Point2>{}, current, 0U, testConfig(), 5.0);

  EXPECT_DOUBLE_EQ(target.x, current.x);
  EXPECT_DOUBLE_EQ(target.y, current.y);
}

TEST(OffboardPathFollower, LookaheadSelectsForwardWaypointThatProgressesToGoal) {
  const std::vector<Point2> path{{0.0, 0.0}, {4.0, 0.0}, {9.0, 0.0}, {15.0, 0.0}};

  const std::size_t index = lookaheadWaypointIndex(
      path, Point2{0.0, 0.0}, Point2{15.0, 0.0}, testConfig(), 5.0);

  EXPECT_EQ(index, 2U);
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

TEST(OffboardPathFollower, ClampsTargetToMaxSetpointDistance) {
  const Point2 target = limitedTarget(Point2{10.0, 0.0}, Point2{0.0, 0.0}, true, 4.0);

  EXPECT_NEAR(target.x, 4.0, 1.0e-9);
  EXPECT_NEAR(target.y, 0.0, 1.0e-9);
}

TEST(OffboardPathFollower, ZeroTargetStepFallsBackToHoldAtCurrentPosition) {
  CommandTargetState state{true, Point2{3.0, 0.0}};

  const Point2 target = smoothedCommandTarget(Point2{10.0, 0.0}, 0.0, false,
                                              Point2{1.0, 2.0}, true, 4.0, state);

  EXPECT_TRUE(state.valid);
  EXPECT_NEAR(target.x, 1.0, 1.0e-9);
  EXPECT_NEAR(target.y, 2.0, 1.0e-9);
}

TEST(OffboardPathFollower, MinimumTargetLeadExtendsCloseCommandTarget) {
  const Point2 target = enforceMinimumTargetLead(Point2{0.5, 0.0}, Point2{10.0, 0.0},
                                                 Point2{0.0, 0.0}, true, 4.0, 12.0);

  EXPECT_NEAR(target.x, 4.0, 1.0e-9);
  EXPECT_NEAR(target.y, 0.0, 1.0e-9);
}

TEST(OffboardPathFollower, MinimumTargetLeadRespectsMaxSetpointDistance) {
  const Point2 target = enforceMinimumTargetLead(Point2{0.5, 0.0}, Point2{10.0, 0.0},
                                                 Point2{0.0, 0.0}, true, 4.0, 2.0);

  EXPECT_NEAR(target.x, 2.0, 1.0e-9);
  EXPECT_NEAR(target.y, 0.0, 1.0e-9);
}

TEST(OffboardPathFollower, MinimumTargetLeadReplacesBehindCommandTarget) {
  const Point2 target = enforceMinimumTargetLead(Point2{-5.0, 0.0}, Point2{10.0, 0.0},
                                                 Point2{0.0, 0.0}, true, 4.0, 12.0);

  EXPECT_NEAR(target.x, 4.0, 1.0e-9);
  EXPECT_NEAR(target.y, 0.0, 1.0e-9);
}

TEST(OffboardPathFollower, MinimumTargetLeadReplacesLateralCommandTarget) {
  const Point2 target = enforceMinimumTargetLead(Point2{0.0, 5.0}, Point2{10.0, 0.0},
                                                 Point2{0.0, 0.0}, true, 4.0, 12.0);

  EXPECT_NEAR(target.x, 4.0, 1.0e-9);
  EXPECT_NEAR(target.y, 0.0, 1.0e-9);
}

TEST(OffboardPathFollower, MinimumTargetLeadLeavesFarTargetUnchanged) {
  const Point2 target = enforceMinimumTargetLead(Point2{5.0, 1.0}, Point2{10.0, 0.0},
                                                 Point2{0.0, 0.0}, true, 4.0, 12.0);

  EXPECT_NEAR(target.x, 5.0, 1.0e-9);
  EXPECT_NEAR(target.y, 1.0, 1.0e-9);
}

TEST(OffboardPathFollower, MissingLocalPositionInvalidatesCommandState) {
  CommandTargetState state{true, Point2{3.0, 0.0}};

  const Point2 target =
      smoothedCommandTarget(Point2{10.0, 2.0}, 1.0, false, Point2{}, false, 4.0, state);

  EXPECT_FALSE(state.valid);
  EXPECT_NEAR(target.x, 10.0, 1.0e-9);
  EXPECT_NEAR(target.y, 2.0, 1.0e-9);
}

TEST(OffboardPathFollower, PathTurnAngleUsesNearbyWaypoint) {
  const std::vector<Point2> path{{0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}};

  const double angle =
      pathTurnAngleAtWaypoint(path, 1U, Point2{4.0, 0.0}, true, testConfig(), 5.0);

  EXPECT_NEAR(angle, std::numbers::pi / 2.0, 1.0e-9);
}

} // namespace drone_city_nav
