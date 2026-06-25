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
  config.max_accel_mps2 = 3.0;
  config.max_decel_mps2 = 20.0;
  config.max_lateral_accel_mps2 = 3.0;
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
  config.max_decel_mps2 = 100.0;

  const ScalarSpeedPlan plan =
      planScalarSpeed(simpleProfile(),
                      ScalarSpeedQuery{.trajectory_s_m = 0.0,
                                       .cross_track_error_m = 0.0,
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

TEST(TrajectorySpeedPlanner, LookaheadDistanceIsClamped) {
  VelocityFollowerConfig config = testConfig();
  config.speed_profile_lookahead_min_m = 5.0;
  config.speed_profile_lookahead_max_m = 8.0;

  const ScalarSpeedPlan slow =
      planScalarSpeed(simpleProfile(),
                      ScalarSpeedQuery{.trajectory_s_m = 0.0,
                                       .cross_track_error_m = 0.0,
                                       .previous_command_speed_mps = 1.0,
                                       .current_speed_mps = 1.0,
                                       .dt_s = 0.1},
                      config);
  const ScalarSpeedPlan fast =
      planScalarSpeed(simpleProfile(),
                      ScalarSpeedQuery{.trajectory_s_m = 0.0,
                                       .cross_track_error_m = 0.0,
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
                       .cross_track_error_m = 0.0,
                       .previous_command_speed_mps = 0.0,
                       .current_speed_mps = 0.0,
                       .dt_s = 0.1},
      testConfig());

  EXPECT_FALSE(empty_plan.valid);
  EXPECT_FALSE(nonfinite_plan.valid);
}

} // namespace drone_city_nav
