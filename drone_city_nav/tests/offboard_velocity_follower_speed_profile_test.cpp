#include <array>

#include "offboard_velocity_follower_test_helpers.hpp"

namespace drone_city_nav {

using offboard_velocity_follower_test_helpers::lineTrajectory;
using offboard_velocity_follower_test_helpers::normalizedTestVector;
using offboard_velocity_follower_test_helpers::testConfig;
using offboard_velocity_follower_test_helpers::trajectoryWithArc;

TEST(OffboardVelocityFollower, StraightTrajectoryReturnsCruiseVelocityAlongTangent) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{10.0, 0.0}, Point2{12.0, 0.0},
                           true, 0.1, state, testConfig());

  ASSERT_TRUE(plan.valid);
  EXPECT_FALSE(plan.final_goal_reached);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kStraight);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kNone);
  EXPECT_NEAR(plan.velocity_xy.x, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.y, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.desired_velocity_xy.x, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.desired_velocity_xy.y, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_tracking_error_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.current_velocity_tangent_mps, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.current_velocity_normal_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.desired_velocity_tangent_mps, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.desired_velocity_normal_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.setpoint_velocity_tangent_mps, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.setpoint_velocity_normal_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.trajectory_s_m, 10.0, 1.0e-9);
  EXPECT_EQ(plan.trajectory_segment_kind, TrajectorySegmentKind::kLine);
}

TEST(OffboardVelocityFollower, NoStaticPolicyCapsOnlyItsOwnSpeedProfile) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  VelocityFollowerConfig static_config = testConfig();
  const TrajectorySpeedProfile static_profile =
      buildTrajectorySpeedProfile(trajectory, static_config);

  VelocityFollowerConfig no_static_config = static_config;
  no_static_config.no_static_speed_policy.enabled = true;
  no_static_config.no_static_speed_policy.max_speed_mps = 10.0;
  no_static_config.no_static_speed_policy.braking_decel_mps2 = 4.0;
  const TrajectorySpeedProfile no_static_profile =
      buildTrajectorySpeedProfile(trajectory, no_static_config);

  EXPECT_NEAR(speedProfileSampleAtS(static_profile, 10.0).profiled_limit_mps, 12.0,
              1.0e-9);
  EXPECT_NEAR(speedProfileSampleAtS(no_static_profile, 10.0).profiled_limit_mps, 10.0,
              1.0e-9);
}

TEST(OffboardVelocityFollower, NoStaticObservedBoundaryAppliesBrakingSpeedCap) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  VelocityFollowerConfig config = testConfig();
  config.no_static_speed_policy.enabled = true;
  config.no_static_speed_policy.max_speed_mps = 10.0;
  config.no_static_speed_policy.braking_decel_mps2 = 4.0;
  config.no_static_speed_policy.reaction_time_s = 2.0;
  config.no_static_speed_policy.safety_margin_m = 4.0;
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, config);
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{10.0, 0.0};
  state.previous_velocity_setpoint_valid = true;
  state.previous_scalar_speed_command_mps = 10.0;
  state.previous_scalar_speed_command_valid = true;

  const NoStaticSpeedConstraint constraint{
      .observation_valid = true,
      .boundary_distance_m = 22.0,
      .boundary = NoStaticSpeedBoundary::kUnknown,
  };
  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{10.0, 0.0}, Point2{10.0, 0.0},
                           true, 1.0, state, config, constraint);

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kNoStaticObservation);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kNoStaticObservation);
  EXPECT_TRUE(plan.no_static_speed_cap_active);
  EXPECT_EQ(plan.no_static_boundary, NoStaticSpeedBoundary::kUnknown);
  EXPECT_NEAR(plan.no_static_boundary_distance_m, 22.0, 1.0e-9);
  EXPECT_NEAR(plan.no_static_speed_limit_mps, -8.0 + std::sqrt(208.0), 1.0e-9);
  EXPECT_NEAR(plan.accel_limited_speed_mps, -8.0 + std::sqrt(208.0), 1.0e-9);
}

TEST(OffboardVelocityFollower, PredictionHorizonProjectsControlAheadOfCurrentPose) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  VelocityFollowerConfig config = testConfig();
  config.tracking_prediction_horizon_s = 0.5;
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, config);
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{10.0, 0.0};
  state.previous_velocity_setpoint_valid = true;
  state.previous_scalar_speed_command_mps = 10.0;
  state.previous_scalar_speed_command_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{10.0, 10.0}, Point2{10.0, 0.0},
                           true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.prediction_horizon_s, 0.5, 1.0e-9);
  EXPECT_NEAR(plan.prediction_distance_m, 5.0, 1.0e-9);
  EXPECT_NEAR(plan.predicted_position.x, 15.0, 1.0e-9);
  EXPECT_NEAR(plan.predicted_position.y, 10.0, 1.0e-9);
  EXPECT_NEAR(plan.current_projection.x, 10.0, 1.0e-9);
  EXPECT_NEAR(plan.current_projection.y, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.predicted_projection.x, 15.0, 1.0e-9);
  EXPECT_NEAR(plan.predicted_projection.y, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.projection.x, 15.0, 1.0e-9);
  EXPECT_NEAR(plan.trajectory_s_m, 15.0, 1.0e-9);
  EXPECT_NEAR(plan.current_cross_track_error_m, 10.0, 1.0e-9);
  EXPECT_NEAR(plan.predicted_cross_track_error_m, 10.0, 1.0e-9);
  EXPECT_NEAR(plan.trajectory_cross_track_error_m, plan.predicted_cross_track_error_m,
              1.0e-9);
  EXPECT_NEAR(plan.response_delay_distance_m, 5.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, PredictionHorizonIsIncludedInFinalBrakingDistance) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  VelocityFollowerConfig config = testConfig();
  config.tracking_prediction_horizon_s = 0.5;
  config.speed_profile_decel_mps2 = 2.0;
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, config);
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{10.0, 0.0};
  state.previous_velocity_setpoint_valid = true;
  state.previous_scalar_speed_command_mps = 10.0;
  state.previous_scalar_speed_command_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{80.0, 0.0}, Point2{10.0, 0.0},
                           true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_FALSE(plan.final_goal_reached);
  EXPECT_NEAR(plan.trajectory_s_m, 85.0, 1.0e-9);
  EXPECT_NEAR(plan.final_stop.distance_to_stop_m, 15.0, 1.0e-9);
  EXPECT_NEAR(plan.response_delay_distance_m, 5.0, 1.0e-9);
  EXPECT_NEAR(plan.final_stop.braking_distance_m, 30.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, NarrowArcGetsLowerGeometricLimitThanWideArc) {
  VelocityFollowerConfig config = testConfig();
  const std::vector<TrajectorySegment> narrow = trajectoryWithArc(5.0);
  const std::vector<TrajectorySegment> wide = trajectoryWithArc(20.0);

  const TrajectorySpeedProfile narrow_profile =
      buildTrajectorySpeedProfile(narrow, config);
  const TrajectorySpeedProfile wide_profile = buildTrajectorySpeedProfile(wide, config);

  const TrajectorySpeedSample narrow_arc =
      speedProfileSampleAtS(narrow_profile, narrow[1].s_start_m + 0.5);
  const TrajectorySpeedSample wide_arc =
      speedProfileSampleAtS(wide_profile, wide[1].s_start_m + 0.5);

  ASSERT_TRUE(narrow_profile.valid);
  ASSERT_TRUE(wide_profile.valid);
  EXPECT_EQ(narrow_arc.reason, SpeedConstraintType::kArc);
  EXPECT_EQ(wide_arc.reason, SpeedConstraintType::kArc);
  EXPECT_LT(narrow_arc.geometric_limit_mps, wide_arc.geometric_limit_mps);
  EXPECT_NEAR(narrow_arc.geometric_limit_mps, std::sqrt(3.0 * 5.0), 1.0e-9);
}

TEST(OffboardVelocityFollower, ProfileStartsBrakingBeforeArc) {
  VelocityFollowerConfig config = testConfig();
  config.cruise_speed_mps = 12.0;
  config.speed_profile_decel_mps2 = 2.0;
  const std::vector<TrajectorySegment> trajectory = trajectoryWithArc(3.0);
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, config);

  const TrajectorySpeedSample before_arc =
      speedProfileSampleAtS(profile, trajectory[1].s_start_m - 2.0);
  const TrajectorySpeedSample far_before =
      speedProfileSampleAtS(profile, trajectory[1].s_start_m - 18.0);

  ASSERT_TRUE(profile.valid);
  EXPECT_LT(before_arc.profiled_limit_mps, config.cruise_speed_mps);
  EXPECT_GT(far_before.profiled_limit_mps, before_arc.profiled_limit_mps);
}

TEST(OffboardVelocityFollower, ShortArcIsConstrainedWhenShorterThanSampleStep) {
  VelocityFollowerConfig config = testConfig();
  config.speed_profile_sample_step_m = 50.0;
  const std::vector<TrajectorySegment> trajectory = trajectoryWithArc(2.0);
  ASSERT_LT(trajectory[1].length_m, config.speed_profile_sample_step_m);

  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, config);
  const TrajectorySpeedSample arc_sample = speedProfileSampleAtS(
      profile, trajectory[1].s_start_m + trajectory[1].length_m * 0.5);

  ASSERT_TRUE(profile.valid);
  EXPECT_EQ(arc_sample.reason, SpeedConstraintType::kArc);
  EXPECT_NEAR(arc_sample.geometric_limit_mps, std::sqrt(3.0 * 2.0), 1.0e-9);
}

TEST(OffboardVelocityFollower, ProjectionNearArcEndKeepsArcConstraint) {
  VelocityFollowerConfig config = testConfig();
  config.speed_profile_sample_step_m = 50.0;
  const std::vector<TrajectorySegment> trajectory = trajectoryWithArc(2.0);
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, config);
  const double projection_s = trajectory[1].s_start_m + trajectory[1].length_m - 0.05;
  const Point2 current_position = trajectoryPointAtS(trajectory, projection_s);

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, current_position, Point2{}, false, 0.1,
                           VelocityFollowerState{}, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kArc);
  EXPECT_NEAR(plan.limiting_constraint_speed_mps, std::sqrt(3.0 * 2.0), 1.0e-9);
  EXPECT_LT(plan.limiting_constraint_distance_m, 0.1);
}

TEST(OffboardVelocityFollower, FinalStopProfileLimitsSpeedNearGoal) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{90.0, 0.0}, Point2{0.0, 0.0},
                           true, 0.1, state, testConfig());

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kFinalApproach);
  EXPECT_FALSE(plan.terminal_capture_active);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kGoal);
  EXPECT_LT(plan.raw_speed_limit_mps, testConfig().cruise_speed_mps);
  EXPECT_GT(plan.raw_speed_limit_mps, 0.0);
  EXPECT_NEAR(plan.limiting_constraint_distance_m, 10.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, TerminalCaptureActivatesByBrakingDistance) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  VelocityFollowerConfig config = testConfig();
  config.terminal_capture_radius_m = 8.0;
  config.terminal_capture_decel_mps2 = 4.0;
  config.terminal_capture_braking_margin_m = 2.0;
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, config);

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{85.0, 0.0}, Point2{12.0, 0.0},
                           true, 0.1, VelocityFollowerState{}, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kTerminalCapture);
  EXPECT_TRUE(plan.terminal_capture_active);
  EXPECT_NEAR(plan.terminal_goal_distance_m, 15.0, 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_braking_distance_m, 18.0, 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_activation_distance_m, 20.0, 1.0e-9);
  EXPECT_TRUE(plan.terminal_capture_goal_distance_triggered);
  EXPECT_TRUE(plan.terminal_capture_remaining_distance_triggered);
}

TEST(OffboardVelocityFollower,
     TerminalCaptureIgnoresFinalPlaneProjectionWhenRemainingPathFar) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectoryFromPoints(
      std::vector<Point2>{{0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}, {0.0, 100.0}});
  VelocityFollowerConfig config = testConfig();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, config);

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{50.0, 0.0}, Point2{20.0, 0.0},
                           true, 0.1, VelocityFollowerState{}, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NE(plan.reason, VelocitySetpointReason::kTerminalCapture);
  EXPECT_FALSE(plan.terminal_capture_active);
  EXPECT_NEAR(plan.terminal_signed_along_track_distance_m, 50.0, 1.0e-9);
  EXPECT_NEAR(plan.terminal_remaining_trajectory_distance_m, 250.0, 1.0e-9);
  EXPECT_FALSE(plan.terminal_capture_goal_distance_triggered);
  EXPECT_FALSE(plan.terminal_capture_remaining_distance_triggered);
}

TEST(OffboardVelocityFollower, TerminalCaptureSpeedUsesBrakingLimit) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  VelocityFollowerConfig config = testConfig();
  config.terminal_capture_max_speed_mps = 10.0;
  config.terminal_capture_gain_1ps = 10.0;
  config.terminal_capture_decel_mps2 = 1.0;
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, config);

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{96.0, 0.0}, Point2{4.0, 0.0},
                           true, 0.1, VelocityFollowerState{}, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kTerminalCapture);
  EXPECT_NEAR(plan.terminal_goal_distance_m, 4.0, 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_braking_speed_limit_mps, std::sqrt(6.0), 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_speed_limit_mps, std::sqrt(6.0), 1.0e-9);
  EXPECT_NEAR(plan.desired_speed_mps, std::sqrt(6.0), 1.0e-9);
}

TEST(OffboardVelocityFollower, TerminalCaptureSpeedLimitDoesNotIncreaseOnceActive) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  VelocityFollowerConfig config = testConfig();
  config.terminal_capture_max_speed_mps = 8.0;
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, config);
  VelocityFollowerState state{};
  state.previous_terminal_capture_active = true;
  state.previous_terminal_capture_speed_limit_valid = true;
  state.previous_terminal_capture_speed_limit_mps = 3.0;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{95.0, 0.0}, Point2{8.0, 0.0},
                           true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kTerminalCapture);
  EXPECT_TRUE(plan.terminal_capture_active);
  EXPECT_NEAR(plan.terminal_capture_gain_speed_limit_mps, 5.0, 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_braking_speed_limit_mps, std::sqrt(32.0), 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_speed_limit_mps, 3.0, 1.0e-9);
  EXPECT_NEAR(plan.desired_speed_mps, 3.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, PreArcBrakingReportsDistanceToArcConstraint) {
  VelocityFollowerConfig config = testConfig();
  config.speed_profile_decel_mps2 = 2.0;
  const std::vector<TrajectorySegment> trajectory = trajectoryWithArc(3.0);
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, config);

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{18.0, 0.0}, Point2{}, false, 0.1,
                           VelocityFollowerState{}, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kArc);
  EXPECT_NEAR(plan.limiting_constraint_distance_m, 2.0, 1.0e-9);
  EXPECT_NEAR(plan.limiting_constraint_speed_mps, std::sqrt(3.0 * 3.0), 1.0e-9);
}

TEST(OffboardVelocityFollower, SampledCurvatureBuildsTrajectorySpeedProfile) {
  VelocityFollowerConfig config = testConfig();
  config.speed_profile_decel_mps2 = 4.0;
  std::vector<TrajectoryPointSample> samples;
  for (std::size_t i = 0U; i < 6U; ++i) {
    TrajectoryPointSample sample{};
    sample.s_m = static_cast<double>(i);
    sample.point = Point2{static_cast<double>(i), 0.0};
    sample.tangent = Point2{1.0, 0.0};
    sample.curvature_1pm = i == 3U ? 0.2 : 0.0;
    samples.push_back(sample);
  }

  const TrajectorySpeedProfile profile = buildTrajectorySpeedProfile(samples, config);

  ASSERT_TRUE(profile.valid);
  const TrajectorySpeedSample curve_sample = speedProfileSampleAtS(profile, 3.0);
  EXPECT_EQ(curve_sample.reason, SpeedConstraintType::kArc);
  EXPECT_NEAR(curve_sample.geometric_limit_mps, std::sqrt(3.0 / 0.2), 1.0e-9);
}

TEST(OffboardVelocityFollower, SampledTrajectoryCurvatureFeedsVelocityAnticipation) {
  VelocityFollowerConfig config = testConfig();
  std::vector<TrajectoryPointSample> samples;
  for (std::size_t i = 0U; i < 8U; ++i) {
    TrajectoryPointSample sample{};
    sample.s_m = static_cast<double>(i) * 2.0;
    sample.point = Point2{sample.s_m, 0.0};
    sample.tangent = Point2{1.0, 0.0};
    sample.curvature_1pm = 0.1;
    samples.push_back(sample);
  }
  const TrajectorySpeedProfile profile = buildTrajectorySpeedProfile(samples, config);
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{6.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      samples, profile, Point2{4.0, 0.0}, Point2{6.0, 0.0}, true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.trajectory_segment_kind, TrajectorySegmentKind::kArc);
  EXPECT_NEAR(plan.trajectory_curvature_1pm, 0.1, 1.0e-9);
  EXPECT_NEAR(plan.trajectory_arc_radius_m, 10.0, 1.0e-9);
  EXPECT_GT(plan.curvature_feedforward_mps, 0.0);
  EXPECT_GT(plan.lateral_control_mps, 0.0);
  EXPECT_GT(plan.desired_velocity_normal_mps, 0.0);
}

TEST(OffboardVelocityFollower, VerticalTrackabilityCapsSpeedDuringPassageLag) {
  VelocityFollowerConfig config = testConfig();
  config.tracking_prediction_horizon_s = 0.0;
  config.setpoint_forward_decel_mps2 = 100.0;
  config.vertical_trackability_max_climb_speed_mps = 4.0;
  config.vertical_trackability_max_descent_speed_mps = 1.5;
  config.vertical_trackability_max_vertical_accel_mps2 = 3.5;
  config.vertical_trackability_altitude_tolerance_m = 0.4;
  config.vertical_trackability_response_time_s = 0.4;
  config.vertical_trackability_min_speed_mps = 1.0;

  std::vector<TrajectoryPointSample> samples;
  for (std::size_t i = 0U; i < 9U; ++i) {
    TrajectoryPointSample sample{};
    sample.s_m = static_cast<double>(i) * 10.0;
    sample.point = Point2{sample.s_m, 0.0};
    sample.tangent = Point2{1.0, 0.0};
    sample.z_m = 10.0;
    if (sample.s_m >= 20.0 && sample.s_m <= 30.0) {
      sample.vertical_profile_passage_id = "window";
      sample.vertical_constraint_active = true;
    }
    samples.push_back(sample);
  }
  const TrajectorySpeedProfile profile = buildTrajectorySpeedProfile(samples, config);
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;
  state.previous_scalar_speed_command_mps = 12.0;
  state.previous_scalar_speed_command_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(samples, profile, Point2{20.0, 0.0}, Point2{12.0, 0.0}, true,
                           18.0, true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kTrajectorySpeedProfile);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kVerticalTrackability);
  EXPECT_TRUE(plan.vertical_trackability_speed_cap_active);
  EXPECT_LT(plan.vertical_trackability_speed_limit_mps, config.cruise_speed_mps);
  EXPECT_NEAR(plan.vertical_trackability_altitude_error_m, -8.0, 1.0e-9);
  EXPECT_NEAR(plan.vertical_trackability_constraint_distance_m, 20.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, VerticalTrackabilityUsesHardWindowWithoutPassageId) {
  VelocityFollowerConfig config = testConfig();
  config.tracking_prediction_horizon_s = 0.0;
  config.setpoint_forward_decel_mps2 = 100.0;
  config.vertical_trackability_max_climb_speed_mps = 4.0;
  config.vertical_trackability_max_descent_speed_mps = 1.5;
  config.vertical_trackability_max_vertical_accel_mps2 = 3.5;
  config.vertical_trackability_altitude_tolerance_m = 0.4;
  config.vertical_trackability_response_time_s = 0.4;
  config.vertical_trackability_min_speed_mps = 1.0;

  std::vector<TrajectoryPointSample> samples;
  for (std::size_t i = 0U; i < 9U; ++i) {
    TrajectoryPointSample sample{};
    sample.s_m = static_cast<double>(i) * 10.0;
    sample.point = Point2{sample.s_m, 0.0};
    sample.tangent = Point2{1.0, 0.0};
    sample.z_m = 10.0;
    if (sample.s_m >= 20.0 && sample.s_m <= 30.0) {
      sample.vertical_hard_window_active = true;
      sample.vertical_safe_min_z_m = 8.0;
      sample.vertical_safe_max_z_m = 10.0;
      sample.vertical_gate_z_m = 10.0;
    }
    samples.push_back(sample);
  }
  const TrajectorySpeedProfile profile = buildTrajectorySpeedProfile(samples, config);
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;
  state.previous_scalar_speed_command_mps = 12.0;
  state.previous_scalar_speed_command_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(samples, profile, Point2{20.0, 0.0}, Point2{12.0, 0.0}, true,
                           10.2, true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kVerticalTrackability);
  EXPECT_TRUE(plan.vertical_trackability_speed_cap_active);
  EXPECT_NEAR(plan.vertical_trackability_altitude_error_m, -0.2, 1.0e-9);
  EXPECT_NEAR(plan.vertical_trackability_constraint_distance_m, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.vertical_trackability_speed_limit_mps,
              config.vertical_trackability_min_speed_mps, 1.0e-9);
}

TEST(OffboardVelocityFollower, VerticalTrackabilityUsesCurrentVerticalVelocity) {
  VelocityFollowerConfig config = testConfig();
  config.tracking_prediction_horizon_s = 0.0;
  config.setpoint_forward_decel_mps2 = 100.0;
  config.vertical_trackability_max_climb_speed_mps = 4.0;
  config.vertical_trackability_max_descent_speed_mps = 1.5;
  config.vertical_trackability_max_vertical_accel_mps2 = 3.5;
  config.vertical_trackability_altitude_tolerance_m = 0.4;
  config.vertical_trackability_response_time_s = 0.4;
  config.vertical_trackability_min_speed_mps = 1.0;

  std::vector<TrajectoryPointSample> samples;
  for (std::size_t i = 0U; i < 9U; ++i) {
    TrajectoryPointSample sample{};
    sample.s_m = static_cast<double>(i) * 10.0;
    sample.point = Point2{sample.s_m, 0.0};
    sample.tangent = Point2{1.0, 0.0};
    sample.z_m = 10.0;
    if (sample.s_m >= 20.0 && sample.s_m <= 30.0) {
      sample.vertical_hard_window_active = true;
      sample.vertical_safe_min_z_m = 8.0;
      sample.vertical_safe_max_z_m = 10.0;
      sample.vertical_gate_z_m = 10.0;
    }
    samples.push_back(sample);
  }
  const TrajectorySpeedProfile profile = buildTrajectorySpeedProfile(samples, config);
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;
  state.previous_scalar_speed_command_mps = 12.0;
  state.previous_scalar_speed_command_valid = true;

  const VelocitySetpointPlan toward =
      planVelocitySetpoint(samples, profile, Point2{0.0, 0.0}, Point2{12.0, 0.0}, true,
                           18.0, true, -1.0, true, 0.1, state, config);
  const VelocitySetpointPlan away =
      planVelocitySetpoint(samples, profile, Point2{0.0, 0.0}, Point2{12.0, 0.0}, true,
                           18.0, true, 1.0, true, 0.1, state, config);

  ASSERT_TRUE(toward.vertical_trackability_speed_cap_active);
  ASSERT_TRUE(away.vertical_trackability_speed_cap_active);
  EXPECT_GT(toward.vertical_trackability_speed_limit_mps,
            away.vertical_trackability_speed_limit_mps);
}

TEST(OffboardVelocityFollower, SmoothsControlTangentOnStraightishSampledTrajectory) {
  VelocityFollowerConfig config = testConfig();
  config.control_tangent_smoothing_back_m = 5.0;
  config.control_tangent_smoothing_forward_m = 10.0;
  config.control_tangent_smoothing_max_heading_span_rad =
      25.0 * std::numbers::pi / 180.0;
  config.control_tangent_smoothing_max_abs_curvature_1pm = 0.01;

  std::vector<TrajectoryPointSample> samples;
  for (std::size_t i = 0U; i < 5U; ++i) {
    TrajectoryPointSample sample{};
    sample.s_m = static_cast<double>(i) * 5.0;
    sample.point = Point2{sample.s_m, 0.0};
    sample.tangent = (i % 2U == 0U) ? normalizedTestVector(1.0, -0.2)
                                    : normalizedTestVector(1.0, 0.2);
    samples.push_back(sample);
  }
  const TrajectorySpeedProfile profile = buildTrajectorySpeedProfile(samples, config);

  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;
  const VelocitySetpointPlan plan = planVelocitySetpoint(
      samples, profile, Point2{10.0, 0.0}, Point2{12.0, 0.0}, true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.control_tangent_smoothed);
  EXPECT_NEAR(plan.control_tangent_raw.x, normalizedTestVector(1.0, -0.2).x, 1.0e-9);
  EXPECT_NEAR(plan.control_tangent_raw.y, normalizedTestVector(1.0, -0.2).y, 1.0e-9);
  EXPECT_NEAR(plan.path_tangent.x, 1.0, 1.0e-9);
  EXPECT_NEAR(plan.path_tangent.y, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.desired_velocity_normal_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.control_tangent_smoothing_window_start_s_m, 5.0, 1.0e-9);
  EXPECT_NEAR(plan.control_tangent_smoothing_window_end_s_m, 20.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, StraightishSmoothingSuppressesLocalSCurveCurvature) {
  VelocityFollowerConfig config = testConfig();
  config.setpoint_forward_accel_mps2 = 100.0;
  config.setpoint_forward_decel_mps2 = 100.0;
  config.setpoint_lateral_response_accel_mps2 = 100.0;
  config.terminal_capture_decel_mps2 = 100.0;
  config.terminal_capture_braking_margin_m = 0.0;
  config.curvature_feedforward_time_s = 0.5;
  config.curvature_feedforward_deadband_angle_rad = 0.0;
  config.curvature_feedforward_full_angle_rad = 0.0;
  config.control_tangent_smoothing_back_m = 5.0;
  config.control_tangent_smoothing_forward_m = 10.0;
  config.control_tangent_smoothing_max_heading_span_rad =
      25.0 * std::numbers::pi / 180.0;
  config.control_tangent_smoothing_max_abs_curvature_1pm = 0.01;

  const std::array<double, 5U> curvatures{-0.008, -0.004, 0.008, 0.004, -0.008};
  std::vector<TrajectoryPointSample> samples;
  double sample_s_m = 0.0;
  for (const double curvature_1pm : curvatures) {
    TrajectoryPointSample sample{};
    sample.s_m = sample_s_m;
    sample.point = Point2{sample.s_m, 0.0};
    sample.tangent = (samples.size() % 2U == 0U) ? normalizedTestVector(1.0, -0.05)
                                                 : normalizedTestVector(1.0, 0.05);
    sample.curvature_1pm = curvature_1pm;
    samples.push_back(sample);
    sample_s_m += 5.0;
  }
  const TrajectorySpeedProfile profile = buildTrajectorySpeedProfile(samples, config);

  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;
  const VelocitySetpointPlan plan = planVelocitySetpoint(
      samples, profile, Point2{10.0, 0.0}, Point2{12.0, 0.0}, true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.control_tangent_smoothed);
  EXPECT_NEAR(plan.trajectory_curvature_1pm, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.curvature_feedforward_context_scale, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.curvature_feedforward_mps, 0.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, UsesShortCurveSmoothingAcrossRealCurvature) {
  VelocityFollowerConfig config = testConfig();
  config.control_tangent_smoothing_back_m = 5.0;
  config.control_tangent_smoothing_forward_m = 10.0;
  config.control_tangent_smoothing_max_heading_span_rad =
      25.0 * std::numbers::pi / 180.0;
  config.control_tangent_smoothing_max_abs_curvature_1pm = 0.01;
  config.control_curve_smoothing_back_m = 2.0;
  config.control_curve_smoothing_forward_m = 6.0;
  config.control_curve_smoothing_max_heading_span_rad = 45.0 * std::numbers::pi / 180.0;

  std::vector<TrajectoryPointSample> samples;
  for (std::size_t i = 0U; i < 5U; ++i) {
    TrajectoryPointSample sample{};
    sample.s_m = static_cast<double>(i) * 5.0;
    sample.point = Point2{sample.s_m, 0.0};
    sample.tangent = (i % 2U == 0U) ? normalizedTestVector(1.0, -0.2)
                                    : normalizedTestVector(1.0, 0.2);
    sample.curvature_1pm = i == 2U ? 0.05 : 0.0;
    samples.push_back(sample);
  }
  const TrajectorySpeedProfile profile = buildTrajectorySpeedProfile(samples, config);

  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;
  const VelocitySetpointPlan plan = planVelocitySetpoint(
      samples, profile, Point2{10.0, 0.0}, Point2{12.0, 0.0}, true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.control_tangent_smoothed);
  EXPECT_GT(plan.control_tangent_smoothing_max_abs_curvature_1pm,
            config.control_tangent_smoothing_max_abs_curvature_1pm);
  EXPECT_NEAR(plan.path_tangent.x, 1.0, 1.0e-9);
  EXPECT_NEAR(plan.path_tangent.y, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.trajectory_curvature_1pm, 0.02, 1.0e-9);
  EXPECT_NEAR(plan.control_tangent_smoothing_window_start_s_m, 8.0, 1.0e-9);
  EXPECT_NEAR(plan.control_tangent_smoothing_window_end_s_m, 16.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, EstimatesTraversalTimeFromSampledTrajectory) {
  VelocityFollowerConfig config = testConfig();
  std::vector<TrajectoryPointSample> samples;
  for (std::size_t i = 0U; i < 11U; ++i) {
    TrajectoryPointSample sample{};
    sample.s_m = static_cast<double>(i) * 2.0;
    sample.point = Point2{sample.s_m, 0.0};
    sample.tangent = Point2{1.0, 0.0};
    samples.push_back(sample);
  }

  const TraversalTimeEstimate estimate = estimateTraversalTime(samples, config, false);

  ASSERT_TRUE(estimate.valid);
  EXPECT_TRUE(std::isfinite(estimate.estimated_time_s));
  EXPECT_NEAR(estimate.estimated_time_s, 20.0 / config.cruise_speed_mps, 1.0e-9);
  EXPECT_NEAR(estimate.min_speed_limit_mps, config.cruise_speed_mps, 1.0e-9);
  EXPECT_NEAR(estimate.max_speed_limit_mps, config.cruise_speed_mps, 1.0e-9);
  EXPECT_EQ(estimate.curvature_limited_samples, 0U);
}

TEST(OffboardVelocityFollower, ProfiledTraversalTimeAccountsForFinalStop) {
  VelocityFollowerConfig config = testConfig();
  config.speed_profile_decel_mps2 = 2.0;
  std::vector<TrajectoryPointSample> samples;
  for (std::size_t i = 0U; i < 21U; ++i) {
    TrajectoryPointSample sample{};
    sample.s_m = static_cast<double>(i);
    sample.point = Point2{sample.s_m, 0.0};
    sample.tangent = Point2{1.0, 0.0};
    samples.push_back(sample);
  }

  const TraversalTimeEstimate geometric = estimateTraversalTime(samples, config, false);
  const TraversalTimeEstimate profiled = estimateTraversalTime(samples, config, true);

  ASSERT_TRUE(geometric.valid);
  ASSERT_TRUE(profiled.valid);
  EXPECT_GT(profiled.estimated_time_s, geometric.estimated_time_s);
  EXPECT_NEAR(profiled.min_speed_limit_mps, 0.0, 1.0e-9);
  EXPECT_LT(profiled.max_speed_limit_mps, config.cruise_speed_mps);
  EXPECT_GT(profiled.max_speed_limit_mps, config.min_turn_speed_mps);
}

} // namespace drone_city_nav
