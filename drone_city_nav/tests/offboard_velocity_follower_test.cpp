#include "drone_city_nav/offboard_velocity_follower.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] VelocityFollowerConfig testConfig() {
  VelocityFollowerConfig config{};
  config.cruise_speed_mps = 12.0;
  config.min_turn_speed_mps = 2.0;
  config.max_accel_mps2 = 3.0;
  config.max_decel_mps2 = 4.0;
  config.max_lateral_accel_mps2 = 3.0;
  config.turn_preview_distance_m = 40.0;
  config.turn_radius_base_m = 10.0;
  config.braking_margin_m = 2.0;
  config.cross_track_gain = 0.25;
  config.max_cross_track_correction_angle_rad = 0.35;
  config.final_acceptance_radius_m = 1.0;
  return config;
}

[[nodiscard]] double turnTargetSpeed(const double angle_rad,
                                     const VelocityFollowerConfig& config) {
  const double severity = std::sin(angle_rad * 0.5);
  const double radius = config.turn_radius_base_m / severity;
  return std::clamp(std::sqrt(config.max_lateral_accel_mps2 * radius),
                    config.min_turn_speed_mps, config.cruise_speed_mps);
}

[[nodiscard]] double smoothLimit(const double remaining_distance_m,
                                 const double target_speed_mps,
                                 const VelocityFollowerConfig& config) {
  const double speed_delta_sq =
      std::max(0.0, config.cruise_speed_mps * config.cruise_speed_mps -
                        target_speed_mps * target_speed_mps);
  const double effective_decel =
      std::min(config.max_decel_mps2,
               std::min(config.max_accel_mps2, config.max_lateral_accel_mps2));
  const double braking_distance =
      speed_delta_sq / (2.0 * effective_decel) + config.braking_margin_m;
  if (remaining_distance_m >= braking_distance) {
    return config.cruise_speed_mps;
  }
  const double profile_decel = speed_delta_sq / (2.0 * braking_distance);
  return std::sqrt(std::max(0.0, target_speed_mps * target_speed_mps +
                                     2.0 * profile_decel * remaining_distance_m));
}

[[nodiscard]] OffboardPathProjection
projectionOnFirstSegment(const double x_m, const double segment_length_m) {
  return OffboardPathProjection{0U, x_m / segment_length_m, 0.0, Point2{x_m, 0.0}};
}

} // namespace

TEST(OffboardVelocityFollower, StraightPathReturnsCruiseVelocityAlongSegment) {
  const std::vector<Point2> path{{0.0, 0.0}, {100.0, 0.0}};
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      path, Point2{10.0, 0.0}, Point2{12.0, 0.0}, true, 1U, 0.1, state, testConfig());

  ASSERT_TRUE(plan.valid);
  EXPECT_FALSE(plan.final_goal_reached);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kStraight);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kNone);
  EXPECT_TRUE(plan.final_stop.valid);
  EXPECT_NEAR(plan.velocity_xy.x, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.y, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.speed_mps, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.projection.x, 10.0, 1.0e-9);
  EXPECT_NEAR(plan.projection.y, 0.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, FarTurnKeepsCruiseSpeed) {
  const std::vector<Point2> path{{0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}};
  VelocityFollowerConfig config = testConfig();
  config.turn_preview_distance_m = 120.0;
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      path, Point2{20.0, 0.0}, Point2{12.0, 0.0}, true, 1U, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  ASSERT_TRUE(plan.turn.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kStraight);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kNone);
  EXPECT_NEAR(plan.turn.distance_to_turn_m, 80.0, 1.0e-9);
  EXPECT_NEAR(plan.raw_speed_limit_mps, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.speed_mps, 12.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, NearTurnReducesAllowedSpeedContinuously) {
  const std::vector<Point2> path{{0.0, 0.0}, {30.0, 0.0}, {30.0, 30.0}};
  const VelocityFollowerConfig config = testConfig();
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      path, Point2{20.0, 0.0}, Point2{12.0, 0.0}, true, 1U, 0.1, state, config);

  const double expected_turn_speed = turnTargetSpeed(std::numbers::pi / 2.0, config);
  const double expected_raw_limit = smoothLimit(10.0, expected_turn_speed, config);

  ASSERT_TRUE(plan.valid);
  ASSERT_TRUE(plan.turn.valid);
  EXPECT_TRUE(plan.final_stop.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kBrakingForTurn);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kTurn);
  EXPECT_EQ(plan.limiting_constraint_index, 1U);
  EXPECT_NEAR(plan.turn.angle_rad, std::numbers::pi / 2.0, 1.0e-9);
  EXPECT_NEAR(plan.turn.distance_to_turn_m, 10.0, 1.0e-9);
  EXPECT_NEAR(plan.turn.target_turn_speed_mps, expected_turn_speed, 1.0e-9);
  EXPECT_NEAR(plan.raw_speed_limit_mps, expected_raw_limit, 1.0e-9);
  EXPECT_NEAR(plan.limiting_allowed_speed_now_mps, expected_raw_limit, 1.0e-9);
  EXPECT_NEAR(plan.accel_limited_speed_mps, 11.7, 1.0e-9);
  EXPECT_NEAR(plan.speed_mps, 11.7, 1.0e-9);
  EXPECT_NEAR(plan.velocity_delta_mps, 0.3, 1.0e-9);
}

TEST(OffboardVelocityFollower, AllowedSpeedNowDecreasesAsTurnConstraintApproaches) {
  const std::vector<Point2> path{{0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}};
  VelocityFollowerConfig config = testConfig();
  config.turn_preview_distance_m = 120.0;

  const TurnSpeedPlan far =
      speedLimitForUpcomingTurn(path, projectionOnFirstSegment(60.0, 100.0), config);
  const TurnSpeedPlan near =
      speedLimitForUpcomingTurn(path, projectionOnFirstSegment(90.0, 100.0), config);

  ASSERT_TRUE(far.valid);
  ASSERT_TRUE(near.valid);
  EXPECT_LT(near.raw_speed_limit_mps, far.raw_speed_limit_mps);
}

TEST(OffboardVelocityFollower, LargeTurnLimitsMoreThanSmallTurnAtSameDistance) {
  VelocityFollowerConfig config = testConfig();
  const std::vector<Point2> small_turn{{0.0, 0.0}, {30.0, 0.0}, {60.0, 10.0}};
  const std::vector<Point2> large_turn{{0.0, 0.0}, {30.0, 0.0}, {30.0, 30.0}};

  const TurnSpeedPlan small = speedLimitForUpcomingTurn(
      small_turn, projectionOnFirstSegment(20.0, 30.0), config);
  const TurnSpeedPlan large = speedLimitForUpcomingTurn(
      large_turn, projectionOnFirstSegment(20.0, 30.0), config);

  ASSERT_TRUE(small.valid);
  ASSERT_TRUE(large.valid);
  EXPECT_LT(large.target_turn_speed_mps, small.target_turn_speed_mps);
  EXPECT_LT(large.raw_speed_limit_mps, small.raw_speed_limit_mps);
}

TEST(OffboardVelocityFollower, MostRestrictiveUpcomingConstraintWins) {
  const std::vector<Point2> path{{0.0, 0.0}, {20.0, 0.0}, {25.0, 1.0}, {25.0, 30.0}};
  VelocityFollowerConfig config = testConfig();
  config.turn_preview_distance_m = 40.0;

  const TurnSpeedPlan plan =
      speedLimitForUpcomingTurn(path, projectionOnFirstSegment(15.0, 20.0), config);

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.waypoint_index, 2U);
}

TEST(OffboardVelocityFollower, BrakingDistanceUsesEffectiveVectorDeceleration) {
  const std::vector<Point2> path{{0.0, 0.0}, {30.0, 0.0}, {30.0, 30.0}};
  VelocityFollowerConfig config = testConfig();
  config.max_decel_mps2 = 10.0;
  config.max_accel_mps2 = 3.0;
  config.max_lateral_accel_mps2 = 2.0;
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      path, Point2{20.0, 0.0}, Point2{12.0, 0.0}, true, 1U, 0.1, state, config);

  const double expected_turn_speed = turnTargetSpeed(std::numbers::pi / 2.0, config);
  const double expected_braking_distance =
      (config.cruise_speed_mps * config.cruise_speed_mps -
       expected_turn_speed * expected_turn_speed) /
          (2.0 * config.max_lateral_accel_mps2) +
      config.braking_margin_m;

  ASSERT_TRUE(plan.valid);
  ASSERT_TRUE(plan.turn.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kBrakingForTurn);
  EXPECT_NEAR(plan.turn.braking_distance_m, expected_braking_distance, 1.0e-9);
  EXPECT_NEAR(plan.accel_limited_speed_mps, 11.8, 1.0e-9);
  EXPECT_NEAR(plan.speed_mps, 11.8, 1.0e-9);
  EXPECT_NEAR(plan.velocity_delta_mps, 0.2, 1.0e-9);
}

TEST(OffboardVelocityFollower, AccelerationLimitClampsSpeedIncrease) {
  const std::vector<Point2> path{{0.0, 0.0}, {100.0, 0.0}};
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{2.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      path, Point2{10.0, 0.0}, Point2{2.0, 0.0}, true, 1U, 0.1, state, testConfig());

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.accel_limited_speed_mps, 2.3, 1.0e-9);
  EXPECT_NEAR(plan.speed_mps, 2.3, 1.0e-9);
}

TEST(OffboardVelocityFollower, GoalConstraintUsesSameSpeedLimitFormula) {
  const std::vector<Point2> path{{0.0, 0.0}, {100.0, 0.0}};
  const VelocityFollowerConfig config = testConfig();
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      path, Point2{80.0, 0.0}, Point2{12.0, 0.0}, true, 1U, 0.1, state, config);

  const double expected_raw_limit =
      smoothLimit(20.0 - config.final_acceptance_radius_m, 0.0, config);

  ASSERT_TRUE(plan.valid);
  ASSERT_TRUE(plan.final_stop.valid);
  EXPECT_FALSE(plan.final_goal_reached);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kFinalApproach);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kGoal);
  EXPECT_EQ(plan.limiting_constraint_index, 1U);
  EXPECT_NEAR(plan.final_stop.distance_to_stop_m, 20.0, 1.0e-9);
  EXPECT_NEAR(plan.final_stop.braking_distance_m, 27.0, 1.0e-9);
  EXPECT_NEAR(plan.raw_speed_limit_mps, expected_raw_limit, 1.0e-9);
  EXPECT_NEAR(plan.accel_limited_speed_mps, 11.7, 1.0e-9);
}

TEST(OffboardVelocityFollower, FarGoalDoesNotBeatNearTurnConstraint) {
  const std::vector<Point2> path{{0.0, 0.0}, {30.0, 0.0}, {30.0, 30.0}};
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      path, Point2{20.0, 0.0}, Point2{12.0, 0.0}, true, 1U, 0.1, state, testConfig());

  ASSERT_TRUE(plan.valid);
  ASSERT_TRUE(plan.turn.valid);
  ASSERT_TRUE(plan.final_stop.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kBrakingForTurn);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kTurn);
  EXPECT_NEAR(plan.turn.distance_to_turn_m, 10.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, BrakingMarginDoesNotForceTurnCrawlBeforeCorner) {
  const std::vector<Point2> path{{0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}};
  VelocityFollowerConfig config = testConfig();
  config.braking_margin_m = 20.0;
  config.turn_preview_distance_m = 120.0;
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      path, Point2{90.0, 0.0}, Point2{12.0, 0.0}, true, 1U, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  ASSERT_TRUE(plan.turn.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kBrakingForTurn);
  EXPECT_NEAR(plan.turn.distance_to_turn_m, 10.0, 1.0e-9);
  EXPECT_GT(plan.raw_speed_limit_mps, config.min_turn_speed_mps + 1.0);
}

TEST(OffboardVelocityFollower, BrakingMarginDoesNotForceFinalCrawlBeforeGoal) {
  const std::vector<Point2> path{{0.0, 0.0}, {100.0, 0.0}};
  VelocityFollowerConfig config = testConfig();
  config.braking_margin_m = 20.0;
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      path, Point2{90.0, 0.0}, Point2{12.0, 0.0}, true, 1U, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  ASSERT_TRUE(plan.final_stop.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kFinalApproach);
  EXPECT_NEAR(plan.final_stop.distance_to_stop_m, 10.0, 1.0e-9);
  EXPECT_GT(plan.raw_speed_limit_mps, 3.0);
}

TEST(OffboardVelocityFollower, VectorDeltaLimitClampsAbruptDirectionChange) {
  const VelocityVectorLimitResult result =
      limitVelocityVectorDelta(Point2{0.0, 12.0}, Point2{12.0, 0.0}, true, 0.1, 3.0);

  EXPECT_NEAR(result.delta_mps, 0.3, 1.0e-9);
  EXPECT_NEAR(std::hypot(result.velocity.x - 12.0, result.velocity.y), 0.3, 1.0e-9);
}

TEST(OffboardVelocityFollower, CrossTrackCorrectionIsBounded) {
  const std::vector<Point2> path{{0.0, 0.0}, {100.0, 0.0}};
  VelocityFollowerConfig config = testConfig();
  config.cross_track_gain = 10.0;
  config.max_cross_track_correction_angle_rad = 0.1;
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      path, Point2{10.0, 10.0}, Point2{12.0, 0.0}, true, 1U, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_LE(plan.cross_track_correction_mps,
            std::max(plan.accel_limited_speed_mps, 1.0) *
                    std::tan(config.max_cross_track_correction_angle_rad) +
                1.0e-9);
}

TEST(OffboardVelocityFollower, TinyTurnAngleRemainsFiniteAndAtCruise) {
  const std::vector<Point2> path{{0.0, 0.0}, {30.0, 0.0}, {60.0, 1.0e-8}};
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      path, Point2{20.0, 0.0}, Point2{12.0, 0.0}, true, 1U, 0.1, state, testConfig());

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kStraight);
  EXPECT_TRUE(std::isfinite(plan.raw_speed_limit_mps));
  EXPECT_NEAR(plan.raw_speed_limit_mps, 12.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, EmptyPathReturnsInvalidPlan) {
  const std::vector<Point2> path{};

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(path, Point2{0.0, 0.0}, Point2{}, false, 0U, 0.1,
                           VelocityFollowerState{}, testConfig());

  EXPECT_FALSE(plan.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kInvalidPath);
}

TEST(OffboardVelocityFollower, VelocityCruisePathUsabilityRejectsInvalidPaths) {
  const std::vector<Point2> empty_path{};
  const std::vector<Point2> single_point{{0.0, 0.0}};
  const std::vector<Point2> degenerate_path{{0.0, 0.0}, {0.0, 0.0}};
  const std::vector<Point2> non_finite_path{
      {0.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 0.0}};
  const std::vector<Point2> valid_path{{0.0, 0.0}, {10.0, 0.0}};

  EXPECT_FALSE(velocityCruisePathIsUsable(empty_path, Point2{0.0, 0.0}, 0U));
  EXPECT_FALSE(velocityCruisePathIsUsable(single_point, Point2{0.0, 0.0}, 0U));
  EXPECT_FALSE(velocityCruisePathIsUsable(degenerate_path, Point2{1.0, 0.0}, 1U));
  EXPECT_FALSE(velocityCruisePathIsUsable(non_finite_path, Point2{0.0, 0.0}, 1U));
  EXPECT_FALSE(velocityCruisePathIsUsable(
      valid_path, Point2{std::numeric_limits<double>::quiet_NaN(), 0.0}, 1U));
  EXPECT_TRUE(velocityCruisePathIsUsable(valid_path, Point2{1.0, 0.0}, 1U));
}

TEST(OffboardVelocityFollower, DegenerateLeadingSegmentUsesNextUsableSegment) {
  const std::vector<Point2> path{{0.0, 0.0}, {0.0, 0.0}, {10.0, 0.0}};

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(path, Point2{0.0, 0.0}, Point2{}, false, 1U, 0.1,
                           VelocityFollowerState{}, testConfig());

  ASSERT_TRUE(velocityCruisePathIsUsable(path, Point2{0.0, 0.0}, 1U));
  ASSERT_TRUE(plan.valid);
  EXPECT_FALSE(plan.final_goal_reached);
  EXPECT_NEAR(plan.path_tangent.x, 1.0, 1.0e-9);
  EXPECT_NEAR(plan.path_tangent.y, 0.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, NonFinitePositionReturnsInvalidPlan) {
  const std::vector<Point2> path{{0.0, 0.0}, {100.0, 0.0}};

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      path, Point2{std::numeric_limits<double>::quiet_NaN(), 0.0}, Point2{}, false, 0U,
      0.1, VelocityFollowerState{}, testConfig());

  EXPECT_FALSE(plan.valid);
}

TEST(OffboardVelocityFollower, DegenerateSegmentDoesNotDivideByZero) {
  const std::vector<Point2> path{{0.0, 0.0}, {0.0, 0.0}};

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(path, Point2{0.0, 0.0}, Point2{}, false, 0U, 0.1,
                           VelocityFollowerState{}, testConfig());

  EXPECT_TRUE(plan.final_goal_reached);
}

TEST(OffboardVelocityFollower, FinalGoalReachedRequestsHold) {
  const std::vector<Point2> path{{0.0, 0.0}, {10.0, 0.0}};

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(path, Point2{9.2, 0.0}, Point2{1.0, 0.0}, true, 1U, 0.1,
                           VelocityFollowerState{}, testConfig());

  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.final_goal_reached);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kHold);
  EXPECT_NEAR(plan.speed_mps, 0.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, DistanceFromProjectionToWaypointSpansSegments) {
  const std::vector<Point2> path{{0.0, 0.0}, {10.0, 0.0}, {10.0, 5.0}};
  const OffboardPathProjection projection{0U, 0.5, 0.0, Point2{5.0, 0.0}};

  EXPECT_NEAR(distanceFromProjectionToWaypoint(path, projection, 2U), 10.0, 1.0e-9);
}

} // namespace drone_city_nav
