#include "drone_city_nav/offboard_velocity_follower.hpp"

#include <gtest/gtest.h>

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
  config.speed_profile_sample_step_m = 1.0;
  config.final_acceptance_radius_m = 1.0;
  config.final_hold_max_speed_mps = 0.8;
  return config;
}

[[nodiscard]] std::vector<TrajectorySegment> lineTrajectory() {
  return lineTrajectoryFromPoints(std::vector<Point2>{{0.0, 0.0}, {100.0, 0.0}});
}

[[nodiscard]] std::vector<TrajectorySegment> trajectoryWithArc(const double radius_m) {
  std::vector<TrajectorySegment> trajectory;
  trajectory.push_back(makeLineSegment(Point2{0.0, 0.0}, Point2{20.0, 0.0}));
  trajectory.push_back(makeArcSegment(Point2{20.0, 0.0},
                                      Point2{20.0 + radius_m, radius_m},
                                      Point2{20.0, radius_m}, -std::numbers::pi / 2.0));
  trajectory.push_back(makeLineSegment(Point2{20.0 + radius_m, radius_m},
                                       Point2{20.0 + radius_m, radius_m + 40.0}));
  assignTrajectoryStationing(trajectory);
  return trajectory;
}

} // namespace

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
      planVelocitySetpoint(trajectory, profile, Point2{95.0, 0.0}, Point2{12.0, 0.0},
                           true, 0.1, state, testConfig());

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kFinalApproach);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kGoal);
  EXPECT_LT(plan.raw_speed_limit_mps, testConfig().cruise_speed_mps);
  EXPECT_GT(plan.raw_speed_limit_mps, 0.0);
  EXPECT_NEAR(plan.limiting_constraint_distance_m, 5.0, 1.0e-9);
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
  for (std::size_t i = 0U; i < 6U; ++i) {
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
  EXPECT_GT(plan.raw_curvature_anticipation_mps, 0.0);
  EXPECT_GT(plan.curvature_anticipation_mps, 0.0);
  EXPECT_GT(plan.desired_velocity_normal_mps, 0.0);
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

TEST(OffboardVelocityFollower, CrossTrackCorrectionIsBounded) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());
  VelocityFollowerConfig config = testConfig();
  config.cross_track_gain = 10.0;
  config.max_cross_track_correction_angle_rad = 0.1;
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{10.0, 10.0}, Point2{12.0, 0.0},
                           true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_LE(plan.cross_track_correction_mps,
            std::max(plan.accel_limited_speed_mps, 1.0) *
                    std::tan(config.max_cross_track_correction_angle_rad) +
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
  EXPECT_NEAR(plan.cross_track_speed_factor, 1.0, 1.0e-9);
  EXPECT_NEAR(plan.cross_track_limited_speed_mps, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.raw_speed_limit_mps, 12.0, 1.0e-9);
  EXPECT_GT(plan.accel_limited_speed_mps, 4.2);
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
  EXPECT_NEAR(plan.cross_track_limited_speed_mps, 4.0, 1.0e-9);
  EXPECT_EQ(plan.trajectory_segment_kind, TrajectorySegmentKind::kLine);
  EXPECT_NEAR(plan.trajectory_curvature_1pm, 0.0, 1.0e-9);
  EXPECT_FALSE(std::isfinite(plan.trajectory_arc_radius_m));
  EXPECT_NEAR(plan.curvature_anticipation_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.raw_curvature_anticipation_mps, 0.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, CrossTrackCorrectionRateLimitSmoothsCorrection) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());
  VelocityFollowerConfig config = testConfig();
  config.cross_track_gain = 10.0;
  config.max_cross_track_correction_angle_rad = 1.0;
  config.max_cross_track_correction_rate_mps2 = 1.0;
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;
  state.previous_cross_track_correction_velocity = Point2{};
  state.previous_cross_track_correction_velocity_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{10.0, 10.0}, Point2{12.0, 0.0},
                           true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_GT(plan.raw_cross_track_correction_mps, plan.cross_track_correction_mps);
  EXPECT_NEAR(plan.cross_track_correction_delta_mps, 0.1, 1.0e-9);
  EXPECT_NEAR(plan.cross_track_correction_mps, 0.1, 1.0e-9);
}

TEST(OffboardVelocityFollower,
     CrossTrackDerivativeDampsCorrectionWhenMovingTowardPath) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());
  VelocityFollowerConfig config = testConfig();
  config.cross_track_gain = 1.0;
  config.cross_track_derivative_gain = 1.0;
  config.max_cross_track_correction_angle_rad = 1.0;
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
  EXPECT_LT(moving_toward_path.cross_track_correction_mps,
            moving_away_from_path.cross_track_correction_mps);
}

TEST(OffboardVelocityFollower, ArcProjectionAddsVelocityAnticipation) {
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
  EXPECT_GT(plan.raw_curvature_anticipation_mps, 0.0);
  EXPECT_GT(plan.curvature_anticipation_mps, 0.0);
  EXPECT_GT(std::abs(plan.curvature_anticipation_velocity.x) +
                std::abs(plan.curvature_anticipation_velocity.y),
            0.0);
}

TEST(OffboardVelocityFollower, CurvatureAnticipationRateLimitSmoothsVelocityBias) {
  const std::vector<TrajectorySegment> trajectory = trajectoryWithArc(20.0);
  const double current_s_m = trajectory[1].s_start_m + 1.0;
  const Point2 current_position = trajectoryPointAtS(trajectory, current_s_m);
  const Point2 current_tangent = trajectoryTangentAtS(trajectory, current_s_m);
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());
  VelocityFollowerConfig config = testConfig();
  config.max_curvature_velocity_anticipation_rate_mps2 = 1.0;
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;
  state.previous_curvature_anticipation_velocity = Point2{};
  state.previous_curvature_anticipation_velocity_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, current_position,
                           Point2{current_tangent.x * 12.0, current_tangent.y * 12.0},
                           true, 0.1, state, config);

  ASSERT_TRUE(plan.valid);
  EXPECT_GT(plan.raw_curvature_anticipation_mps, plan.curvature_anticipation_mps);
  EXPECT_NEAR(plan.curvature_anticipation_mps, 0.1, 1.0e-9);
  EXPECT_NEAR(plan.curvature_anticipation_delta_mps, 0.1, 1.0e-9);
}

TEST(OffboardVelocityFollower, VelocityJerkLimitDoesNotBlockLongitudinalBraking) {
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
  EXPECT_NEAR(plan.accel_limited_speed_mps, 10.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.x, 10.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_xy.y, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_setpoint_acceleration_xy.x, -20.0, 1.0e-9);
  EXPECT_NEAR(plan.velocity_setpoint_acceleration_xy.y, 0.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, VelocityJerkLimitSmoothsDirectionChange) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());
  VelocityFollowerConfig config = testConfig();
  config.max_accel_mps2 = 100.0;
  config.max_lateral_accel_mps2 = 100.0;
  config.max_decel_mps2 = 100.0;
  config.cross_track_gain = 10.0;
  config.max_cross_track_correction_angle_rad = 1.0;
  config.max_velocity_jerk_mps3 = 1.0;
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

TEST(OffboardVelocityFollower, EmptyTrajectoryReturnsInvalidPlan) {
  const std::vector<TrajectorySegment> trajectory{};
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{0.0, 0.0}, Point2{}, false, 0.1,
                           VelocityFollowerState{}, testConfig());

  EXPECT_FALSE(plan.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kInvalidPath);
}

TEST(OffboardVelocityFollower, FinalGoalReachedRequestsHold) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{99.2, 0.0}, Point2{0.5, 0.0},
                           true, 0.1, VelocityFollowerState{}, testConfig());

  ASSERT_TRUE(plan.valid);
  EXPECT_TRUE(plan.final_goal_reached);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kHold);
  EXPECT_NEAR(plan.speed_mps, 0.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, FastGoalFlyThroughKeepsVelocityBrakingActive) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());
  VelocityFollowerState state{};
  state.previous_velocity_setpoint = Point2{12.0, 0.0};
  state.previous_velocity_setpoint_valid = true;

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{99.2, 0.0}, Point2{12.0, 0.0},
                           true, 0.1, state, testConfig());

  ASSERT_TRUE(plan.valid);
  EXPECT_FALSE(plan.final_goal_reached);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kFinalApproach);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kGoal);
  EXPECT_LT(plan.raw_speed_limit_mps, testConfig().cruise_speed_mps);
}

TEST(OffboardVelocityFollower, NonFinitePositionReturnsInvalidPlan) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());

  const VelocitySetpointPlan plan = planVelocitySetpoint(
      trajectory, profile, Point2{std::numeric_limits<double>::quiet_NaN(), 0.0},
      Point2{}, false, 0.1, VelocityFollowerState{}, testConfig());

  EXPECT_FALSE(plan.valid);
}

} // namespace drone_city_nav
