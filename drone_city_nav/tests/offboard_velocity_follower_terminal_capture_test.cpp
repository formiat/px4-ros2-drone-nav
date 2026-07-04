#include "offboard_velocity_follower_test_helpers.hpp"

namespace drone_city_nav {

using offboard_velocity_follower_test_helpers::lineTrajectory;
using offboard_velocity_follower_test_helpers::normalizedTestVector;
using offboard_velocity_follower_test_helpers::testConfig;
using offboard_velocity_follower_test_helpers::trajectoryWithArc;

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

TEST(OffboardVelocityFollower, FastGoalFlyThroughUsesTerminalCaptureUntilSlow) {
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
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kTerminalCapture);
  EXPECT_TRUE(plan.terminal_capture_active);
  EXPECT_NEAR(plan.terminal_goal_distance_m, 0.8, 1.0e-9);
  EXPECT_NEAR(plan.terminal_remaining_trajectory_distance_m, 0.8, 1.0e-9);
  EXPECT_NEAR(plan.terminal_acceptance_radius_m, 1.0, 1.0e-9);
  EXPECT_NEAR(plan.terminal_hold_max_speed_mps, 0.8, 1.0e-9);
  EXPECT_TRUE(plan.terminal_hold_distance_met);
  EXPECT_FALSE(plan.terminal_hold_speed_met);
  EXPECT_TRUE(plan.terminal_capture_goal_distance_triggered);
  EXPECT_TRUE(plan.terminal_capture_remaining_distance_triggered);
  EXPECT_NEAR(plan.terminal_capture_gain_speed_limit_mps, 0.8, 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_max_speed_mps, 8.0, 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_decel_mps2, 4.0, 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_braking_margin_m, 2.0, 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_braking_distance_m, 18.0, 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_activation_distance_m, 20.0, 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_braking_speed_limit_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_speed_limit_mps, 0.0, 1.0e-9);
  EXPECT_EQ(plan.limiting_constraint_type, SpeedConstraintType::kGoal);
  EXPECT_NEAR(plan.desired_speed_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.raw_speed_limit_mps, 0.0, 1.0e-9);
}

TEST(OffboardVelocityFollower,
     TerminalCaptureReturnsToGoalAfterFinalPlaneWhenFastButFar) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{104.0, 4.0}, Point2{6.0, 0.0},
                           true, 0.1, VelocityFollowerState{}, testConfig());

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kTerminalCapture);
  EXPECT_TRUE(plan.terminal_capture_active);
  EXPECT_NEAR(plan.terminal_goal_distance_m, std::sqrt(32.0), 1.0e-9);
  EXPECT_NEAR(plan.terminal_remaining_trajectory_distance_m, 0.0, 1.0e-9);
  EXPECT_FALSE(plan.terminal_hold_distance_met);
  EXPECT_FALSE(plan.terminal_hold_speed_met);
  EXPECT_TRUE(plan.terminal_capture_goal_distance_triggered);
  EXPECT_TRUE(plan.terminal_capture_remaining_distance_triggered);
  EXPECT_NEAR(plan.terminal_signed_along_track_distance_m, -4.0, 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_gain_speed_limit_mps, std::sqrt(32.0), 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_braking_speed_limit_mps,
              std::sqrt(8.0 * (std::sqrt(32.0) - 1.0)), 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_speed_limit_mps, 1.6, 1.0e-9);
  EXPECT_NEAR(plan.desired_velocity_xy.x, -1.6 / std::sqrt(2.0), 1.0e-9);
  EXPECT_NEAR(plan.desired_velocity_xy.y, -1.6 / std::sqrt(2.0), 1.0e-9);
  EXPECT_NEAR(plan.path_tangent.x, -1.0 / std::sqrt(2.0), 1.0e-9);
  EXPECT_NEAR(plan.path_tangent.y, -1.0 / std::sqrt(2.0), 1.0e-9);
}

TEST(OffboardVelocityFollower, TerminalCaptureBrakesAfterFinalPlaneWhenFastAndNear) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{100.4, 0.0}, Point2{6.0, 0.0},
                           true, 0.1, VelocityFollowerState{}, testConfig());

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kTerminalCapture);
  EXPECT_TRUE(plan.terminal_capture_active);
  EXPECT_NEAR(plan.terminal_goal_distance_m, 0.4, 1.0e-9);
  EXPECT_NEAR(plan.terminal_remaining_trajectory_distance_m, 0.0, 1.0e-9);
  EXPECT_TRUE(plan.terminal_hold_distance_met);
  EXPECT_FALSE(plan.terminal_hold_speed_met);
  EXPECT_TRUE(plan.terminal_capture_goal_distance_triggered);
  EXPECT_TRUE(plan.terminal_capture_remaining_distance_triggered);
  EXPECT_NEAR(plan.terminal_signed_along_track_distance_m, -0.4, 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_speed_limit_mps, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.desired_velocity_xy.x, 0.0, 1.0e-9);
  EXPECT_NEAR(plan.desired_velocity_xy.y, 0.0, 1.0e-9);
}

TEST(OffboardVelocityFollower, TerminalCaptureReturnsToGoalAfterFinalPlaneWhenSlow) {
  const std::vector<TrajectorySegment> trajectory = lineTrajectory();
  const TrajectorySpeedProfile profile =
      buildTrajectorySpeedProfile(trajectory, testConfig());

  const VelocitySetpointPlan plan =
      planVelocitySetpoint(trajectory, profile, Point2{104.0, 4.0}, Point2{0.2, 0.0},
                           true, 0.1, VelocityFollowerState{}, testConfig());

  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.reason, VelocitySetpointReason::kTerminalCapture);
  EXPECT_TRUE(plan.terminal_capture_active);
  EXPECT_FALSE(plan.final_goal_reached);
  EXPECT_FALSE(plan.terminal_hold_distance_met);
  EXPECT_TRUE(plan.terminal_hold_speed_met);
  EXPECT_NEAR(plan.terminal_goal_distance_m, std::sqrt(32.0), 1.0e-9);
  EXPECT_NEAR(plan.terminal_signed_along_track_distance_m, -4.0, 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_gain_speed_limit_mps, std::sqrt(32.0), 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_braking_speed_limit_mps,
              std::sqrt(8.0 * (std::sqrt(32.0) - 1.0)), 1.0e-9);
  EXPECT_NEAR(plan.terminal_capture_speed_limit_mps, 1.6, 1.0e-9);
  EXPECT_NEAR(plan.desired_velocity_xy.x, -1.6 / std::sqrt(2.0), 1.0e-9);
  EXPECT_NEAR(plan.desired_velocity_xy.y, -1.6 / std::sqrt(2.0), 1.0e-9);
  EXPECT_NEAR(plan.path_tangent.x, -1.0 / std::sqrt(2.0), 1.0e-9);
  EXPECT_NEAR(plan.path_tangent.y, -1.0 / std::sqrt(2.0), 1.0e-9);
  EXPECT_NEAR(plan.limiting_constraint_distance_m, std::sqrt(32.0), 1.0e-9);
}

} // namespace drone_city_nav
