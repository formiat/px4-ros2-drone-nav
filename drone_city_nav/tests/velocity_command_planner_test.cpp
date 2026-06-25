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
  config.max_cross_track_correction_angle_rad = 1.0;
  config.max_cross_track_correction_rate_mps2 = 100.0;
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

TEST(VelocityCommandPlanner, CrossTrackCorrectionIsBoundedByAngle) {
  VelocityFollowerConfig config = testConfig();
  config.cross_track_gain = 10.0;
  config.max_cross_track_correction_angle_rad = 0.1;

  const VelocityCommandPlan plan =
      planVelocityCommand(VelocityCommandQuery{.projection = projectionOnXAxis(100.0),
                                               .current_position = Point2{0.0, 10.0},
                                               .current_velocity = Point2{10.0, 0.0},
                                               .current_velocity_valid = true,
                                               .scalar_speed_mps = 10.0,
                                               .dt_s = 0.1},
                          config);

  ASSERT_TRUE(plan.valid);
  EXPECT_LE(plan.cross_track_correction_mps,
            10.0 * std::tan(config.max_cross_track_correction_angle_rad) + 1.0e-9);
  EXPECT_GT(plan.raw_cross_track_correction_mps, plan.cross_track_correction_mps);
}

TEST(VelocityCommandPlanner, DerivativeDampsCorrectionWhenMovingTowardPath) {
  VelocityFollowerConfig config = testConfig();
  config.max_cross_track_correction_angle_rad = 1.0;

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
  EXPECT_LT(moving_toward.cross_track_correction_mps,
            moving_away.cross_track_correction_mps);
}

TEST(VelocityCommandPlanner, CorrectionRateLimitSmoothsCorrectionVelocity) {
  VelocityFollowerConfig config = testConfig();
  config.cross_track_gain = 10.0;
  config.max_cross_track_correction_rate_mps2 = 1.0;

  const VelocityCommandPlan plan = planVelocityCommand(
      VelocityCommandQuery{.projection = projectionOnXAxis(100.0),
                           .current_position = Point2{0.0, 10.0},
                           .current_velocity = Point2{10.0, 0.0},
                           .current_velocity_valid = true,
                           .scalar_speed_mps = 10.0,
                           .dt_s = 0.1,
                           .previous_cross_track_correction_velocity = Point2{},
                           .previous_cross_track_correction_velocity_valid = true},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.cross_track_correction_delta_mps, 0.1, 1.0e-9);
  EXPECT_NEAR(plan.cross_track_correction_mps, 0.1, 1.0e-9);
}

TEST(VelocityCommandPlanner, CurvatureAnticipationBendsVelocityDirection) {
  VelocityFollowerConfig config = testConfig();
  config.curvature_velocity_anticipation_time_s = 0.5;
  config.max_curvature_velocity_anticipation_angle_rad = 1.0;

  const VelocityCommandPlan plan = planVelocityCommand(
      VelocityCommandQuery{.projection = curvedProjectionOnXAxis(0.1),
                           .current_position = Point2{0.0, 0.0},
                           .current_velocity = Point2{10.0, 0.0},
                           .current_velocity_valid = true,
                           .scalar_speed_mps = 10.0,
                           .dt_s = 0.1},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_GT(plan.raw_curvature_anticipation_mps, 0.0);
  EXPECT_GT(plan.curvature_anticipation_mps, 0.0);
  EXPECT_NEAR(plan.curvature_anticipation_angle_rad, 0.5, 1.0e-9);
  EXPECT_GT(plan.desired_velocity_normal_mps, 0.0);
  EXPECT_LT(plan.desired_velocity_tangent_mps, 10.0);
}

TEST(VelocityCommandPlanner, CurvatureAnticipationRateLimitSmoothsVelocityBias) {
  VelocityFollowerConfig config = testConfig();
  config.curvature_velocity_anticipation_time_s = 0.5;
  config.max_curvature_velocity_anticipation_angle_rad = 1.0;
  config.max_curvature_velocity_anticipation_rate_mps2 = 1.0;

  const VelocityCommandPlan plan = planVelocityCommand(
      VelocityCommandQuery{.projection = curvedProjectionOnXAxis(0.1),
                           .current_position = Point2{0.0, 0.0},
                           .current_velocity = Point2{10.0, 0.0},
                           .current_velocity_valid = true,
                           .scalar_speed_mps = 10.0,
                           .dt_s = 0.1,
                           .previous_curvature_anticipation_velocity = Point2{},
                           .previous_curvature_anticipation_velocity_valid = true},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_GT(plan.raw_curvature_anticipation_mps, plan.curvature_anticipation_mps);
  EXPECT_NEAR(plan.curvature_anticipation_mps, 0.1, 1.0e-9);
  EXPECT_NEAR(plan.curvature_anticipation_delta_mps, 0.1, 1.0e-9);
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
