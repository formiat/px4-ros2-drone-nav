#include "offboard_velocity_follower_test_helpers.hpp"

namespace drone_city_nav {

using offboard_velocity_follower_test_helpers::lineTrajectory;
using offboard_velocity_follower_test_helpers::normalizedTestVector;
using offboard_velocity_follower_test_helpers::testConfig;
using offboard_velocity_follower_test_helpers::trajectoryWithArc;

TEST(OffboardVelocityFollower, VectorDeltaLimitClampsAbruptDirectionChange) {
  const VelocityVectorLimitResult result =
      limitVelocityVectorDelta(Point2{0.0, 12.0}, Point2{12.0, 0.0}, true, 0.1, 3.0);

  EXPECT_NEAR(result.delta_mps, std::sqrt(0.3 * 0.3 + 0.3 * 0.3), 1.0e-9);
  EXPECT_NEAR(result.velocity.x, 11.7, 1.0e-9);
  EXPECT_NEAR(result.velocity.y, 0.3, 1.0e-9);
}

TEST(OffboardVelocityFollower, VectorDeltaAllowsAggressiveLongitudinalBraking) {
  const VelocityVectorLimitResult result = limitVelocityVectorDelta(
      Point2{8.0, 0.0}, Point2{12.0, 0.0}, true, 0.1, 3.0, 12.0);

  EXPECT_NEAR(result.delta_mps, 1.2, 1.0e-9);
  EXPECT_NEAR(result.velocity.x, 10.8, 1.0e-9);
  EXPECT_NEAR(result.velocity.y, 0.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, LateralControlIsBounded) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());
  VelocityFollowerConfig config = testConfig();
  config.cross_track_gain = 10.0;
  config.max_lateral_control_angle_rad = 0.1;
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{10.0, 10.0}, Point2{12.0, 0.0},
                           true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_LE(plan.lateral_control_mps,
            std::max(plan.accel_limited_speed_mps, 1.0) *
                    std::tan(config.max_lateral_control_angle_rad) +
                1.0e-9);
  EXPECT_NEAR(plan.trajectory_cross_track_error_m, 10.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, CrossTrackErrorDoesNotReduceScalarSpeed) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());
  VelocityFollowerConfig config = testConfig();
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{4.2, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{10.0, 10.0}, Point2{4.2, 0.0},
                           true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.profile_speed_limit_mps, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.raw_speed_limit_mps, 12.0, 1.0e-9);
  EXPECT_GT(plan.accel_limited_speed_mps, 4.2);
}

TEST(OffboardVelocityFollower, ScalarSpeedStateIsIndependentFromVectorSetpoint) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());
  VelocityFollowerConfig config = testConfig();
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{0.5, 0.0};
  state.previous_velocity_setpoint_valid = true;
  state.previous_scalar_speed_command_mps = 10.0;
  state.previous_scalar_speed_command_valid = true;

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      trajectory, profile, Point2{10.0, 0.0}, Point2{}, false, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.accel_limited_speed_mps, 10.3, 1.0e-9);
  EXPECT_GT(plan.accel_limited_speed_mps,
            std::hypot(state.previous_velocity_setpoint.x,
                       state.previous_velocity_setpoint.y));
}

TEST(OffboardVelocityFollower, LookaheadStageLimitsScalarSpeedBeforeCommandPlanner) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  TrajectorySpeedProfile profile{};
  profile.valid = true;
  profile.samples = {
      TrajectorySpeedSample{.s_m = 0.0,
                            .geometric_limit_mps = 12.0,
                            .profiled_limit_mps = 12.0,
                            .reason = SpeedConstraintType::kNone,
                            .constraint_s_m = 0.0,
                            .constraint_limit_mps = 12.0},
      TrajectorySpeedSample{.s_m = 8.0,
                            .geometric_limit_mps = 4.0,
                            .profiled_limit_mps = 4.0,
                            .reason = SpeedConstraintType::kArc,
                            .segment_index = 0U,
                            .curvature_1pm = 0.25,
                            .radius_m = 4.0,
                            .constraint_s_m = 8.0,
                            .constraint_limit_mps = 4.0},
      TrajectorySpeedSample{.s_m = 100.0,
                            .geometric_limit_mps = 0.0,
                            .profiled_limit_mps = 0.0,
                            .reason = SpeedConstraintType::kGoal,
                            .segment_index = 0U,
                            .constraint_s_m = 100.0,
                            .constraint_limit_mps = 0.0},
  };
  VelocityFollowerConfig config = testConfig();
  config.max_decel_mps2 = 100.0;
  config.speed_profile_lookahead_time_s = 1.0;
  config.speed_profile_lookahead_min_m = 5.0;
  config.speed_profile_lookahead_max_m = 35.0;
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{0.0, 0.0}, Point2{12.0, 0.0},
                           true, 1.0, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kTrajectorySpeedProfile);
  EXPECT_NEAR(plan.profile_speed_limit_mps, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.speed_lookahead_distance_m, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.lookahead_speed_limit_mps, 4.0, 1.0e-9);
  EXPECT_NEAR(plan.speed_after_lookahead_mps, 4.0, 1.0e-9);
  EXPECT_EQ(plan.lookahead_limiting_constraint_type, SpeedConstraintType::kArc);
  EXPECT_NEAR(plan.lookahead_limiting_constraint_distance_m, 8.0, 1.0e-9);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kArc);
  EXPECT_NEAR(plan.limiting_curve_radius_m, 4.0, 1.0e-9);
  EXPECT_EQ(plan.trajectory_segment_kind, TrajectorySegmentKind::kLine);
  EXPECT_NEAR(plan.trajectory_curvature_1pm, 0.0, 1.0e-9);
  EXPECT_FALSE(std::isfinite(plan.trajectory_arc_radius_m));
  EXPECT_NEAR(plan.curvature_feedforward_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.raw_lateral_control_mps, 0.0, 1.0e-9);
}

TEST(OffboardVelocityFollower,
     CrossTrackDerivativeDampsCorrectionWhenMovingTowardPath) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());
  VelocityFollowerConfig config = testConfig();
  config.cross_track_gain = 1.0;
  config.cross_track_derivative_gain = 1.0;
  config.cross_track_progressive_feedback_min_factor = 1.0;
  config.cross_track_progressive_feedback_max_factor = 1.0;
  config.max_lateral_control_angle_rad = 1.0;
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan moving_toward_path =
      planVelocitySetpoint(trajectory, profile, Point2{10.0, 5.0}, Point2{0.0, -2.0},
                           true, 0.1, state, config);
  const VelocitySetpointPlan moving_away_from_path =
      planVelocitySetpoint(trajectory, profile, Point2{10.0, 5.0}, Point2{0.0, 2.0},
                           true, 0.1, state, config);

  ASSERT_TRUE(moving_toward_path.valid);
  ASSERT_TRUE(moving_away_from_path.valid);
  EXPECT_GT(moving_toward_path.cross_track_lateral_velocity_mps, 0.0);
  EXPECT_LT(moving_away_from_path.cross_track_lateral_velocity_mps, 0.0);
  EXPECT_LT(moving_toward_path.lateral_control_mps,
            moving_away_from_path.lateral_control_mps);
  EXPECT_GT(moving_toward_path.cross_track_derivative_damping_mps, 0.0);
}

TEST(OffboardVelocityFollower, ArcProjectionAddsCurvatureFeedforward) {
  const std::vector<TrajectorySegment> trajectory = trajectoryWithArc(20.0);
  const double current_s_m = trajectory[1].s_start_m + 1.0;
  const Point2 current_position = trajectoryPointAtS(trajectory, current_s_m);
  const Point2 current_tangent = trajectoryTangentAtS(trajectory, current_s_m);
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());
  VelocityFollowerConfig config = testConfig();
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, current_position,
                           Point2{current_tangent.x * 12.0, current_tangent.y * 12.0},
                           true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.trajectory_segment_kind, TrajectorySegmentKind::kArc);
  EXPECT_NEAR(plan.trajectory_curvature_1pm, -0.05, 1.0e-9);
  EXPECT_NEAR(plan.trajectory_arc_radius_m, 20.0, 1.0e-9);
  EXPECT_GT(plan.curvature_feedforward_mps, 0.0);
  EXPECT_GT(plan.lateral_control_mps, 0.0);
  EXPECT_GT(std::abs(plan.curvature_feedforward_velocity.x) +
                std::abs(plan.curvature_feedforward_velocity.y),
            0.0);
}

TEST(OffboardVelocityFollower, VelocityJerkLimitSmoothsLongitudinalBraking) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  TrajectorySpeedProfile profile{};
  profile.valid = true;
  profile.samples = {
      TrajectorySpeedSample{.s_m = 0.0,
                            .geometric_limit_mps = 2.0,
                            .profiled_limit_mps = 2.0,
                            .reason = SpeedConstraintType::kArc,
                            .segment_index = 0U,
                            .curvature_1pm = 0.2,
                            .radius_m = 5.0,
                            .constraint_s_m = 0.0,
                            .constraint_limit_mps = 2.0},
      TrajectorySpeedSample{.s_m = 100.0,
                            .geometric_limit_mps = 0.0,
                            .profiled_limit_mps = 0.0,
                            .reason = SpeedConstraintType::kGoal,
                            .segment_index = 0U,
                            .curvature_1pm = 0.0,
                            .constraint_s_m = 100.0,
                            .constraint_limit_mps = 0.0},
  };
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_lateral_accel_mps2 = 100.0;
  config.max_decel_mps2 = 20.0;
  config.max_velocity_jerk_mps3 = 1.0;
  config.max_lateral_velocity_jerk_mps3 = 1.0;
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;
  state.previous_velocity_acceleration_setpoint = Point2{};
  state.previous_velocity_acceleration_setpoint_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{0.0, 0.0}, Point2{12.0, 0.0},
                           true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.raw_speed_limit_mps, 2.0, 1.0e-9);
  EXPECT_NEAR(plan.accel_limited_speed_mps, 11.8, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.x, 11.99, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.y, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_setpoint_acceleration_xy.x, -0.1, 1.0e-9);
  EXPECT_NEAR(plan.velocity_setpoint_acceleration_xy.y, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_setpoint_jerk_mps3, 1.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, VelocityJerkLimitSmoothsDirectionChange) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_lateral_accel_mps2 = 100.0;
  config.velocity_lateral_response_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;
  config.cross_track_gain = 10.0;
  config.max_lateral_control_angle_rad = 1.0;
  config.max_velocity_jerk_mps3 = 1.0;
  config.max_lateral_velocity_jerk_mps3 = 1.0;
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;
  state.previous_velocity_acceleration_setpoint = Point2{};
  state.previous_velocity_acceleration_setpoint_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{10.0, 10.0}, Point2{12.0, 0.0},
                           true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_GT(plan.desired_velocity_delta_mps, 0.0);
  EXPECT_LE(std::abs(plan.velocity_setpoint_acceleration_xy.y), 0.1 + 1.0e-9);
  EXPECT_LE(std::abs(plan.velocity_xy.y), 0.01 + 1.0e-9);
  EXPECT_LT(plan.velocity_xy.x, 12.0);
}

} // namespace drone_city_nav
