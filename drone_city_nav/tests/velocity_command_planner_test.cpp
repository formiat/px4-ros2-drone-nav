#include "drone_city_nav/velocity_command_planner.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace drone_city_nav {
namespace {

[[nodiscard]] TrajectoryProjection projectionOnXAxis(const double distance_sq = 0.0) {
  TrajectoryProjection projection{};
  projection.valid = true;
  projection.point = Point2{0.0, 0.0};
  projection.tangent = Point2{1.0, 0.0};
  projection.distance_sq = distance_sq;
  return projection;
}

[[nodiscard]] TrajectoryProjection curvedProjectionOnXAxis(const double curvature_1pm) {
  TrajectoryProjection projection = projectionOnXAxis();
  projection.curvature_1pm = curvature_1pm;
  return projection;
}

[[nodiscard]] VelocityFollowerConfig testConfig() {
  VelocityFollowerConfig config{};
  config.cross_track_gain = 1.0;
  config.cross_track_derivative_gain = 1.0;
  config.max_lateral_control_angle_rad = 1.0;
  config.max_lateral_control_rate_mps2 = 100.0;
  return config;
}

} // namespace

TEST(VelocityCommandPlanner, StraightTrajectoryReturnsTangentVelocity) {
  const VelocityCommandPlan plan =
      planVelocityCommand(VelocityCommandQuery{.projection = projectionOnXAxis(),
                                               .current_position = Point2{0.0, 0.0},
                                               .current_velocity = Point2{10.0, 0.0},
                                               .current_velocity_valid = true,
                                               .scalar_speed_mps = 10.0,
                                               .dt_s = 0.1},
                          testConfig());

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.desired_velocity_xy.x, 10.0, 1.0e-9);
  EXPECT_NEAR(plan.desired_velocity_xy.y, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.desired_velocity_tangent_mps, 10.0, 1.0e-9);
  EXPECT_NEAR(plan.desired_velocity_normal_mps, 0.0, 1.0e-9);
}

TEST(VelocityCommandPlanner, LateralControlIsBoundedByAngle) {
  VelocityFollowerConfig config = testConfig();
  config.cross_track_gain = 10.0;
  config.max_lateral_control_angle_rad = 0.1;

  const VelocityCommandPlan plan =
      planVelocityCommand(VelocityCommandQuery{.projection = projectionOnXAxis(100.0),
                                               .current_position = Point2{0.0, 10.0},
                                               .current_velocity = Point2{10.0, 0.0},
                                               .current_velocity_valid = true,
                                               .scalar_speed_mps = 10.0,
                                               .dt_s = 0.1},
                          config);

  ASSERT_TRUE(plan.valid);
  EXPECT_LE(plan.lateral_control_mps,
            10.0 * std::tan(config.max_lateral_control_angle_rad) + 1.0e-9);
  EXPECT_GT(plan.raw_lateral_control_mps, plan.lateral_control_mps);
  EXPECT_GT(plan.cross_track_feedback_mps, 0.0);
}

TEST(VelocityCommandPlanner, DerivativeDampsCorrectionWhenMovingTowardPath) {
  VelocityFollowerConfig config = testConfig();
  config.max_lateral_control_angle_rad = 1.0;

  const VelocityCommandPlan moving_toward =
      planVelocityCommand(VelocityCommandQuery{.projection = projectionOnXAxis(25.0),
                                               .current_position = Point2{0.0, 5.0},
                                               .current_velocity = Point2{0.0, -2.0},
                                               .current_velocity_valid = true,
                                               .scalar_speed_mps = 10.0,
                                               .dt_s = 0.1},
                          config);
  const VelocityCommandPlan moving_away =
      planVelocityCommand(VelocityCommandQuery{.projection = projectionOnXAxis(25.0),
                                               .current_position = Point2{0.0, 5.0},
                                               .current_velocity = Point2{0.0, 2.0},
                                               .current_velocity_valid = true,
                                               .scalar_speed_mps = 10.0,
                                               .dt_s = 0.1},
                          config);

  ASSERT_TRUE(moving_toward.valid);
  ASSERT_TRUE(moving_away.valid);
  EXPECT_GT(moving_toward.cross_track_lateral_velocity_mps, 0.0);
  EXPECT_LT(moving_away.cross_track_lateral_velocity_mps, 0.0);
  EXPECT_LT(moving_toward.lateral_control_mps, moving_away.lateral_control_mps);
  EXPECT_GT(moving_toward.cross_track_derivative_damping_mps, 0.0);
}

TEST(VelocityCommandPlanner, LateralControlRateLimitSmoothsVelocity) {
  VelocityFollowerConfig config = testConfig();
  config.cross_track_gain = 10.0;
  config.max_lateral_control_rate_mps2 = 1.0;

  const VelocityCommandPlan plan = planVelocityCommand(
      VelocityCommandQuery{.projection = projectionOnXAxis(100.0),
                           .current_position = Point2{0.0, 10.0},
                           .current_velocity = Point2{10.0, 0.0},
                           .current_velocity_valid = true,
                           .scalar_speed_mps = 10.0,
                           .dt_s = 0.1,
                           .previous_lateral_control_velocity = Point2{},
                           .previous_lateral_control_velocity_valid = true},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.lateral_control_delta_mps, 0.1, 1.0e-9);
  EXPECT_NEAR(plan.lateral_control_mps, 0.1, 1.0e-9);
}

TEST(VelocityCommandPlanner, AdaptiveResponseBoostsGrowingCrossTrackError) {
  VelocityFollowerConfig config = testConfig();
  config.cross_track_gain = 1.0;
  config.cross_track_derivative_gain = 0.0;
  config.max_lateral_control_angle_rad = 1.0;
  config.adaptive_lateral_response_scale_m = 3.0;
  config.adaptive_lateral_response_max_factor = 2.5;

  const VelocityCommandPlan plan =
      planVelocityCommand(VelocityCommandQuery{.projection = projectionOnXAxis(25.0),
                                               .current_position = Point2{0.0, 5.0},
                                               .current_velocity = Point2{10.0, 0.0},
                                               .current_velocity_valid = true,
                                               .scalar_speed_mps = 10.0,
                                               .dt_s = 0.1,
                                               .current_cross_track_error_m = 1.0,
                                               .predicted_cross_track_error_m = 5.0},
                          config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.adaptive_lateral_response_factor, 2.5, 1.0e-9);
  EXPECT_NEAR(plan.raw_lateral_control_mps, 12.5, 1.0e-9);
  EXPECT_NEAR(plan.lateral_control_mps, 12.5, 1.0e-9);
}

TEST(VelocityCommandPlanner, AdaptiveResponseBoostsLateralRateLimit) {
  VelocityFollowerConfig config = testConfig();
  config.cross_track_gain = 10.0;
  config.cross_track_derivative_gain = 0.0;
  config.max_lateral_control_rate_mps2 = 1.0;
  config.adaptive_lateral_response_scale_m = 3.0;
  config.adaptive_lateral_response_max_factor = 2.5;

  const VelocityCommandPlan plan = planVelocityCommand(
      VelocityCommandQuery{.projection = projectionOnXAxis(100.0),
                           .current_position = Point2{0.0, 10.0},
                           .current_velocity = Point2{10.0, 0.0},
                           .current_velocity_valid = true,
                           .scalar_speed_mps = 10.0,
                           .dt_s = 0.1,
                           .previous_lateral_control_velocity = Point2{},
                           .previous_lateral_control_velocity_valid = true,
                           .current_cross_track_error_m = 1.0,
                           .predicted_cross_track_error_m = 5.0},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.adaptive_lateral_response_factor, 2.5, 1.0e-9);
  EXPECT_NEAR(plan.lateral_control_delta_mps, 0.25, 1.0e-9);
  EXPECT_NEAR(plan.lateral_control_mps, 0.25, 1.0e-9);
}

TEST(VelocityCommandPlanner, CurvatureFeedforwardBendsVelocityDirection) {
  VelocityFollowerConfig config = testConfig();
  config.curvature_feedforward_time_s = 0.5;
  config.max_curvature_feedforward_angle_rad = 1.0;

  const VelocityCommandPlan plan = planVelocityCommand(
      VelocityCommandQuery{.projection = curvedProjectionOnXAxis(0.1),
                           .current_position = Point2{0.0, 0.0},
                           .current_velocity = Point2{10.0, 0.0},
                           .current_velocity_valid = true,
                           .scalar_speed_mps = 10.0,
                           .dt_s = 0.1},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_GT(plan.curvature_feedforward_mps, 0.0);
  EXPECT_GT(plan.lateral_control_mps, 0.0);
  EXPECT_NEAR(plan.curvature_feedforward_angle_rad, 0.5, 1.0e-9);
  EXPECT_GT(plan.desired_velocity_normal_mps, 0.0);
  EXPECT_LT(plan.desired_velocity_tangent_mps, 10.0);
}

TEST(VelocityCommandPlanner, LateralControlRateLimitSmoothsCurvatureFeedforward) {
  VelocityFollowerConfig config = testConfig();
  config.curvature_feedforward_time_s = 0.5;
  config.max_curvature_feedforward_angle_rad = 1.0;
  config.max_lateral_control_rate_mps2 = 1.0;

  const VelocityCommandPlan plan = planVelocityCommand(
      VelocityCommandQuery{.projection = curvedProjectionOnXAxis(0.1),
                           .current_position = Point2{0.0, 0.0},
                           .current_velocity = Point2{10.0, 0.0},
                           .current_velocity_valid = true,
                           .scalar_speed_mps = 10.0,
                           .dt_s = 0.1,
                           .previous_lateral_control_velocity = Point2{},
                           .previous_lateral_control_velocity_valid = true},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_GT(plan.raw_lateral_control_mps, plan.lateral_control_mps);
  EXPECT_NEAR(plan.lateral_control_mps, 0.1, 1.0e-9);
  EXPECT_NEAR(plan.lateral_control_delta_mps, 0.1, 1.0e-9);
}

TEST(VelocityCommandPlanner, InvalidProjectionReturnsInvalidPlan) {
  TrajectoryProjection invalid_projection = projectionOnXAxis();
  invalid_projection.tangent = Point2{};

  const VelocityCommandPlan plan =
      planVelocityCommand(VelocityCommandQuery{.projection = invalid_projection,
                                               .current_position = Point2{0.0, 0.0},
                                               .scalar_speed_mps = 10.0,
                                               .dt_s = 0.1},
                          testConfig());

  EXPECT_FALSE(plan.valid);
}

} // namespace drone_city_nav
