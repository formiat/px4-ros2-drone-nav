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
  config.speed_profile_decel_mps2 = 4.0;
  config.speed_profile_sample_step_m = 1.0;
  config.cross_track_gain = 0.25;
  config.max_cross_track_correction_angle_rad = 0.35;
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
