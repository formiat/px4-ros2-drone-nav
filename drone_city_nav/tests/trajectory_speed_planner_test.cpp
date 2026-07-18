#include "drone_city_nav/trajectory_speed_planner.hpp"

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
  config.setpoint_forward_accel_mps2 = 3.0;
  config.setpoint_forward_decel_mps2 = 20.0;
  config.turn_speed_lateral_accel_mps2 = 3.0;
  config.speed_profile_decel_mps2 = 2.0;
  config.speed_profile_sample_step_m = 1.0;
  config.speed_profile_lookahead_time_s = 1.0;
  config.speed_profile_lookahead_min_m = 5.0;
  config.speed_profile_lookahead_max_m = 35.0;
  return config;
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

[[nodiscard]] TrajectorySpeedProfile simpleProfile() {
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
                            .segment_index = 1U,
                            .curvature_1pm = 0.25,
                            .radius_m = 4.0,
                            .constraint_s_m = 8.0,
                            .constraint_limit_mps = 4.0},
      TrajectorySpeedSample{.s_m = 40.0,
                            .geometric_limit_mps = 0.0,
                            .profiled_limit_mps = 0.0,
                            .reason = SpeedConstraintType::kGoal,
                            .segment_index = 2U,
                            .constraint_s_m = 40.0,
                            .constraint_limit_mps = 0.0},
  };
  return profile;
}

[[nodiscard]] TrajectorySpeedProfile unconstrainedProfile() {
  TrajectorySpeedProfile profile{};
  profile.valid = true;
  profile.samples = {
      TrajectorySpeedSample{.s_m = 0.0,
                            .geometric_limit_mps = 20.0,
                            .profiled_limit_mps = 20.0,
                            .reason = SpeedConstraintType::kNone,
                            .constraint_s_m = 0.0,
                            .constraint_limit_mps = 20.0},
      TrajectorySpeedSample{.s_m = 40.0,
                            .geometric_limit_mps = 20.0,
                            .profiled_limit_mps = 20.0,
                            .reason = SpeedConstraintType::kNone,
                            .constraint_s_m = 40.0,
                            .constraint_limit_mps = 20.0},
  };
  return profile;
}

} // namespace

TEST(TrajectorySpeedPlanner, NarrowArcGetsLowerGeometricLimitThanWideArc) {
  const VelocityFollowerConfig config = testConfig();
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

TEST(TrajectorySpeedPlanner, BackwardPassBrakesBeforeArcAndGoal) {
  VelocityFollowerConfig config = testConfig();
  config.cruise_speed_mps = 12.0;
  config.speed_profile_decel_mps2 = 2.0;
  const std::vector<TrajectorySegment> trajectory = trajectoryWithArc(3.0);

  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, config);

  ASSERT_TRUE(profile.valid);
  const TrajectorySpeedSample before_arc =
      speedProfileSampleAtS(profile, trajectory[1].s_start_m - 2.0);
  const TrajectorySpeedSample far_before =
      speedProfileSampleAtS(profile, trajectory[1].s_start_m - 18.0);
  const TrajectorySpeedSample near_goal =
      speedProfileSampleAtS(profile, trajectoryLengthM(trajectory) - 2.0);
  EXPECT_LT(before_arc.profiled_limit_mps, config.cruise_speed_mps);
  EXPECT_GT(far_before.profiled_limit_mps, before_arc.profiled_limit_mps);
  EXPECT_EQ(near_goal.reason, SpeedConstraintType::kGoal);
  EXPECT_LT(near_goal.profiled_limit_mps, config.cruise_speed_mps);
}

TEST(TrajectorySpeedPlanner, LookaheadSeesUpcomingLowSpeedConstraint) {
  VelocityFollowerConfig config = testConfig();
  config.setpoint_forward_decel_mps2 = 100.0;

  const ScalarSpeedPlan plan =
      planScalarSpeed(simpleProfile(),
                      ScalarSpeedQuery{.trajectory_s_m = 0.0,
                                       .previous_command_speed_mps = 12.0,
                                       .current_speed_mps = 12.0,
                                       .dt_s = 1.0},
                      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.profile_speed_limit_mps, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.lookahead_distance_m, 12.0, 1.0e-9);
  EXPECT_NEAR(plan.lookahead_speed_limit_mps, 4.0, 1.0e-9);
  EXPECT_NEAR(plan.speed_after_lookahead_mps, 4.0, 1.0e-9);
  EXPECT_EQ(plan.lookahead_constraint_type, SpeedConstraintType::kArc);
  EXPECT_NEAR(plan.lookahead_constraint_distance_m, 8.0, 1.0e-9);
}

TEST(TrajectorySpeedPlanner, ScalarAccelerationIgnoresLateralTurnLimit) {
  VelocityFollowerConfig config = testConfig();
  config.cruise_speed_mps = 20.0;
  config.setpoint_forward_accel_mps2 = 5.0;
  config.turn_speed_lateral_accel_mps2 = 1.0;

  const ScalarSpeedPlan plan =
      planScalarSpeed(unconstrainedProfile(),
                      ScalarSpeedQuery{.trajectory_s_m = 0.0,
                                       .previous_command_speed_mps = 0.0,
                                       .current_speed_mps = 0.0,
                                       .dt_s = 1.0},
                      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.speed_after_lookahead_mps, 20.0, 1.0e-9);
  EXPECT_NEAR(plan.accel_limited_speed_mps, 5.0, 1.0e-9);
}

TEST(TrajectorySpeedPlanner, VerticalTrackabilityCapLimitsScalarSpeed) {
  VelocityFollowerConfig config = testConfig();
  config.cruise_speed_mps = 20.0;
  config.speed_profile_decel_mps2 = 100.0;
  config.setpoint_forward_decel_mps2 = 100.0;

  const ScalarSpeedPlan plan = planScalarSpeed(
      unconstrainedProfile(),
      ScalarSpeedQuery{.trajectory_s_m = 0.0,
                       .previous_command_speed_mps = 12.0,
                       .current_speed_mps = 12.0,
                       .dt_s = 1.0,
                       .vertical_trackability_speed_cap_active = true,
                       .vertical_trackability_speed_limit_mps = 5.0,
                       .vertical_trackability_constraint_distance_m = 7.5,
                       .vertical_trackability_altitude_error_m = -2.0},
      config);

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.constraint_type, SpeedConstraintType::kVerticalTrackability);
  EXPECT_NEAR(plan.speed_after_lookahead_mps, 5.0, 1.0e-9);
  EXPECT_NEAR(plan.accel_limited_speed_mps, 5.0, 1.0e-9);
  EXPECT_TRUE(plan.vertical_trackability_speed_cap_active);
  EXPECT_NEAR(plan.vertical_trackability_speed_limit_mps, 5.0, 1.0e-9);
  EXPECT_NEAR(plan.vertical_trackability_constraint_distance_m, 7.5, 1.0e-9);
  EXPECT_NEAR(plan.vertical_trackability_altitude_error_m, -2.0, 1.0e-9);
}

TEST(TrajectorySpeedPlanner, TopConstraintsAreUniqueAndFlagIsolatedSpikes) {
  TrajectorySpeedProfile profile{};
  profile.valid = true;
  profile.samples = {
      TrajectorySpeedSample{.s_m = 0.0,
                            .geometric_limit_mps = 12.0,
                            .profiled_limit_mps = 9.0,
                            .reason = SpeedConstraintType::kArc,
                            .segment_index = 2U,
                            .curvature_1pm = 0.01,
                            .radius_m = 100.0,
                            .constraint_s_m = 2.0,
                            .constraint_limit_mps = 6.0},
      TrajectorySpeedSample{.s_m = 1.0,
                            .geometric_limit_mps = 12.0,
                            .profiled_limit_mps = 8.0,
                            .reason = SpeedConstraintType::kArc,
                            .segment_index = 2U,
                            .curvature_1pm = 0.01,
                            .radius_m = 100.0,
                            .constraint_s_m = 2.0,
                            .constraint_limit_mps = 6.0},
      TrajectorySpeedSample{.s_m = 2.0,
                            .geometric_limit_mps = 6.0,
                            .profiled_limit_mps = 6.0,
                            .reason = SpeedConstraintType::kArc,
                            .segment_index = 2U,
                            .curvature_1pm = 0.12,
                            .radius_m = 8.333333333,
                            .constraint_s_m = 2.0,
                            .constraint_limit_mps = 6.0},
      TrajectorySpeedSample{.s_m = 3.0,
                            .geometric_limit_mps = 12.0,
                            .profiled_limit_mps = 10.0,
                            .reason = SpeedConstraintType::kArc,
                            .segment_index = 4U,
                            .curvature_1pm = 0.02,
                            .radius_m = 50.0,
                            .constraint_s_m = 4.0,
                            .constraint_limit_mps = 8.0},
      TrajectorySpeedSample{.s_m = 4.0,
                            .geometric_limit_mps = 8.0,
                            .profiled_limit_mps = 8.0,
                            .reason = SpeedConstraintType::kArc,
                            .segment_index = 4U,
                            .curvature_1pm = 0.02,
                            .radius_m = 50.0,
                            .constraint_s_m = 4.0,
                            .constraint_limit_mps = 8.0},
  };

  const std::vector<SpeedProfileConstraintDiagnostic> constraints =
      topSpeedProfileConstraints(profile, 5U);

  ASSERT_EQ(constraints.size(), 2U);
  EXPECT_NEAR(constraints[0].s_m, 2.0, 1.0e-9);
  EXPECT_NEAR(constraints[0].speed_limit_mps, 6.0, 1.0e-9);
  EXPECT_TRUE(constraints[0].isolated_curvature_spike);
  EXPECT_NEAR(constraints[1].s_m, 4.0, 1.0e-9);
  EXPECT_NEAR(constraints[1].speed_limit_mps, 8.0, 1.0e-9);
}

TEST(TrajectorySpeedPlanner, VerticalProfileCapsSpeedAndTopConstraintSource) {
  VelocityFollowerConfig config = testConfig();
  config.cruise_speed_mps = 20.0;
  config.min_turn_speed_mps = 2.0;
  config.vertical_profile_max_climb_speed_mps = 2.0;
  config.vertical_profile_max_descent_speed_mps = 2.0;
  config.vertical_profile_max_vertical_accel_mps2 = 100.0;
  config.vertical_profile_max_vertical_jerk_mps3 = 1000.0;

  std::vector<TrajectoryPointSample> samples;
  for (std::size_t i = 0U; i < 5U; ++i) {
    TrajectoryPointSample sample{};
    sample.s_m = static_cast<double>(i) * 5.0;
    sample.point = Point2{sample.s_m, 0.0};
    sample.tangent = Point2{1.0, 0.0};
    sample.z_m = static_cast<double>(i) * 5.0;
    samples.push_back(sample);
  }

  const TrajectorySpeedProfile profile = buildTrajectorySpeedProfile(samples, config);

  ASSERT_TRUE(profile.valid);
  const TrajectorySpeedSample ramp = speedProfileSampleAtS(profile, 10.0);
  EXPECT_EQ(ramp.reason, SpeedConstraintType::kVerticalProfile);
  EXPECT_NEAR(ramp.geometric_limit_mps, 2.0, 1.0e-9);
  EXPECT_NEAR(ramp.vertical_slope_dz_ds, 1.0, 1.0e-9);

  const std::vector<SpeedProfileConstraintDiagnostic> constraints =
      topSpeedProfileConstraints(profile, 3U);
  ASSERT_FALSE(constraints.empty());
  EXPECT_EQ(constraints.front().source, SpeedConstraintType::kVerticalProfile);
  EXPECT_NEAR(constraints.front().speed_limit_mps, 2.0, 1.0e-9);
}

TEST(TrajectorySpeedPlanner, KnownPassageHardWindowCapsSpeedWithoutVerticalSlope) {
  VelocityFollowerConfig config = testConfig();
  config.cruise_speed_mps = 20.0;
  config.known_passage_traversal_speed_limit_mps = 9.0;
  config.speed_profile_decel_mps2 = 100.0;

  std::vector<TrajectoryPointSample> samples;
  for (std::size_t i = 0U; i < 5U; ++i) {
    TrajectoryPointSample sample{};
    sample.s_m = static_cast<double>(i) * 5.0;
    sample.point = Point2{sample.s_m, 0.0};
    sample.tangent = Point2{1.0, 0.0};
    sample.z_m = 10.0;
    sample.vertical_hard_window_active = i >= 1U && i <= 3U;
    samples.push_back(sample);
  }

  const TrajectorySpeedProfile profile = buildTrajectorySpeedProfile(samples, config);

  ASSERT_TRUE(profile.valid);
  const TrajectorySpeedSample passage = speedProfileSampleAtS(profile, 10.0);
  EXPECT_NEAR(passage.geometric_limit_mps, 9.0, 1.0e-9);
  EXPECT_NEAR(passage.vertical_speed_limit_mps, 9.0, 1.0e-9);

  const std::vector<SpeedProfileConstraintDiagnostic> constraints =
      topSpeedProfileConstraints(profile, 3U);
  ASSERT_FALSE(constraints.empty());
  EXPECT_EQ(constraints.front().source, SpeedConstraintType::kVerticalProfile);
  EXPECT_NEAR(constraints.front().speed_limit_mps, 9.0, 1.0e-9);
}

TEST(TrajectorySpeedPlanner, SpeedProfileSampleInterpolatesBetweenSamples) {
  const TrajectorySpeedSample sample = speedProfileSampleAtS(simpleProfile(), 4.0);

  EXPECT_EQ(sample.reason, SpeedConstraintType::kArc);
  EXPECT_NEAR(sample.profiled_limit_mps, std::sqrt(80.0), 1.0e-9);
  EXPECT_NEAR(sample.geometric_limit_mps, std::sqrt(80.0), 1.0e-9);
  EXPECT_NEAR(sample.constraint_s_m, 8.0, 1.0e-9);
  EXPECT_NEAR(sample.constraint_limit_mps, 4.0, 1.0e-9);
}

TEST(TrajectorySpeedPlanner, LookaheadDistanceIsClamped) {
  VelocityFollowerConfig config = testConfig();
  config.speed_profile_lookahead_min_m = 5.0;
  config.speed_profile_lookahead_max_m = 8.0;

  const ScalarSpeedPlan slow =
      planScalarSpeed(simpleProfile(),
                      ScalarSpeedQuery{.trajectory_s_m = 0.0,
                                       .previous_command_speed_mps = 1.0,
                                       .current_speed_mps = 1.0,
                                       .dt_s = 0.1},
                      config);
  const ScalarSpeedPlan fast =
      planScalarSpeed(simpleProfile(),
                      ScalarSpeedQuery{.trajectory_s_m = 0.0,
                                       .previous_command_speed_mps = 100.0,
                                       .current_speed_mps = 100.0,
                                       .dt_s = 0.1},
                      config);

  ASSERT_TRUE(slow.valid);
  ASSERT_TRUE(fast.valid);
  EXPECT_NEAR(slow.lookahead_distance_m, 5.0, 1.0e-9);
  EXPECT_NEAR(fast.lookahead_distance_m, 8.0, 1.0e-9);
}

TEST(TrajectorySpeedPlanner, InvalidInputReturnsInvalidScalarPlan) {
  const ScalarSpeedPlan empty_plan =
      planScalarSpeed(TrajectorySpeedProfile{}, ScalarSpeedQuery{}, testConfig());
  const ScalarSpeedPlan nonfinite_plan = planScalarSpeed(
      simpleProfile(),
      ScalarSpeedQuery{.trajectory_s_m = std::numeric_limits<double>::quiet_NaN(),
                       .previous_command_speed_mps = 0.0,
                       .current_speed_mps = 0.0,
                       .dt_s = 0.1},
      testConfig());

  EXPECT_FALSE(empty_plan.valid);
  EXPECT_FALSE(nonfinite_plan.valid);
}

} // namespace drone_city_nav
