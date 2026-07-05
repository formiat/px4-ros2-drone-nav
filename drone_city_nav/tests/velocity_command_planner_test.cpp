#include "drone_city_nav/velocity_command_planner.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

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
  config.cross_track_progressive_feedback_min_factor = 1.0;
  config.cross_track_progressive_feedback_max_factor = 1.0;
  config.max_lateral_control_angle_rad = 1.0;
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

TEST(VelocityCommandPlanner, CrossTrackFeedbackProgressivelyIncreasesWithError) {
  VelocityFollowerConfig config = testConfig();
  config.cross_track_gain = 1.0;
  config.cross_track_derivative_gain = 0.0;
  config.cross_track_progressive_feedback_start_m = 0.3;
  config.cross_track_progressive_feedback_full_m = 2.5;
  config.cross_track_progressive_feedback_min_factor = 0.25;
  config.cross_track_progressive_feedback_max_factor = 1.3;

  const VelocityCommandPlan near_path =
      planVelocityCommand(VelocityCommandQuery{.projection = projectionOnXAxis(0.04),
                                               .current_position = Point2{0.0, 0.2},
                                               .current_velocity = Point2{10.0, 0.0},
                                               .current_velocity_valid = true,
                                               .scalar_speed_mps = 10.0,
                                               .dt_s = 0.1},
                          config);
  const VelocityCommandPlan mid_error =
      planVelocityCommand(VelocityCommandQuery{.projection = projectionOnXAxis(1.96),
                                               .current_position = Point2{0.0, 1.4},
                                               .current_velocity = Point2{10.0, 0.0},
                                               .current_velocity_valid = true,
                                               .scalar_speed_mps = 10.0,
                                               .dt_s = 0.1},
                          config);
  const VelocityCommandPlan far_from_path =
      planVelocityCommand(VelocityCommandQuery{.projection = projectionOnXAxis(9.0),
                                               .current_position = Point2{0.0, 3.0},
                                               .current_velocity = Point2{10.0, 0.0},
                                               .current_velocity_valid = true,
                                               .scalar_speed_mps = 10.0,
                                               .dt_s = 0.1},
                          config);

  ASSERT_TRUE(near_path.valid);
  ASSERT_TRUE(mid_error.valid);
  ASSERT_TRUE(far_from_path.valid);
  EXPECT_NEAR(near_path.cross_track_progressive_feedback_factor, 0.25, 1.0e-9);
  EXPECT_GT(mid_error.cross_track_progressive_feedback_factor,
            near_path.cross_track_progressive_feedback_factor);
  EXPECT_LT(mid_error.cross_track_progressive_feedback_factor,
            far_from_path.cross_track_progressive_feedback_factor);
  EXPECT_NEAR(far_from_path.cross_track_progressive_feedback_factor, 1.3, 1.0e-9);
  EXPECT_NEAR(near_path.cross_track_feedback_mps, 0.05, 1.0e-9);
  EXPECT_NEAR(far_from_path.cross_track_feedback_mps, 3.9, 1.0e-9);
}

TEST(VelocityCommandPlanner, SpeedAwareDerivativeDampingBoostsOnlyWhenReturningFast) {
  VelocityFollowerConfig config = testConfig();
  config.cross_track_gain = 1.0;
  config.cross_track_derivative_gain = 1.0;
  config.max_lateral_control_angle_rad = 1.0;
  config.speed_aware_derivative_damping_min_speed_mps = 8.0;
  config.speed_aware_derivative_damping_full_speed_mps = 20.0;
  config.speed_aware_derivative_damping_max_factor = 1.5;

  const VelocityCommandPlan moving_toward =
      planVelocityCommand(VelocityCommandQuery{.projection = projectionOnXAxis(25.0),
                                               .current_position = Point2{0.0, 5.0},
                                               .current_velocity = Point2{20.0, -2.0},
                                               .current_velocity_valid = true,
                                               .scalar_speed_mps = 20.0,
                                               .dt_s = 0.1},
                          config);
  const VelocityCommandPlan moving_away =
      planVelocityCommand(VelocityCommandQuery{.projection = projectionOnXAxis(25.0),
                                               .current_position = Point2{0.0, 5.0},
                                               .current_velocity = Point2{20.0, 2.0},
                                               .current_velocity_valid = true,
                                               .scalar_speed_mps = 20.0,
                                               .dt_s = 0.1},
                          config);

  ASSERT_TRUE(moving_toward.valid);
  ASSERT_TRUE(moving_away.valid);
  EXPECT_NEAR(moving_toward.cross_track_derivative_damping_factor, 1.5, 1.0e-9);
  EXPECT_NEAR(moving_toward.cross_track_derivative_gain_effective, 1.5, 1.0e-9);
  EXPECT_NEAR(moving_toward.cross_track_derivative_damping_mps, 3.0, 1.0e-9);
  EXPECT_NEAR(moving_away.cross_track_derivative_damping_factor, 1.0, 1.0e-9);
  EXPECT_NEAR(moving_away.cross_track_derivative_gain_effective, 1.0, 1.0e-9);
  EXPECT_NEAR(moving_away.cross_track_derivative_damping_mps, 2.0, 1.0e-9);
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

TEST(VelocityCommandPlanner, CurvatureFeedforwardAttenuatesTinyCurvature) {
  VelocityFollowerConfig config = testConfig();
  config.curvature_feedforward_time_s = 1.0;
  config.curvature_feedforward_deadband_angle_rad = 2.0 * std::numbers::pi / 180.0;
  config.curvature_feedforward_full_angle_rad = 8.0 * std::numbers::pi / 180.0;
  config.max_curvature_feedforward_angle_rad = 1.0;

  const VelocityCommandPlan plan = planVelocityCommand(
      VelocityCommandQuery{.projection = curvedProjectionOnXAxis(
                               (1.0 * std::numbers::pi / 180.0) / 10.0),
                           .current_position = Point2{0.0, 0.0},
                           .current_velocity = Point2{10.0, 0.0},
                           .current_velocity_valid = true,
                           .scalar_speed_mps = 10.0,
                           .dt_s = 0.1},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.curvature_feedforward_raw_angle_rad, 1.0 * std::numbers::pi / 180.0,
              1.0e-9);
  EXPECT_NEAR(plan.curvature_feedforward_scale, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.curvature_feedforward_angle_rad, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.curvature_feedforward_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.desired_velocity_normal_mps, 0.0, 1.0e-9);
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
