#include "drone_city_nav/offboard_trajectory_state.hpp"

#include <nav_msgs/msg/path.hpp>

#include <gtest/gtest.h>

#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] nav_msgs::msg::Path makePath() {
  nav_msgs::msg::Path path;
  path.header.stamp.sec = 2;
  path.header.stamp.nanosec = 3;
  geometry_msgs::msg::PoseStamped first;
  first.pose.position.x = 0.0;
  first.pose.position.y = 0.0;
  first.pose.position.z = 10.0;
  geometry_msgs::msg::PoseStamped second;
  second.pose.position.x = 4.0;
  second.pose.position.y = 0.0;
  second.pose.position.z = 12.0;
  geometry_msgs::msg::PoseStamped third;
  third.pose.position.x = 4.0;
  third.pose.position.y = 3.0;
  third.pose.position.z = 14.0;
  path.poses = {first, second, third};
  return path;
}

} // namespace

TEST(OffboardTrajectoryState, ConvertsPathAndBuildsExecutableState) {
  const nav_msgs::msg::Path path = makePath();

  const std::vector<Point2> points = pathPointsFromMessage(path);
  const std::vector<TrajectoryPointSample> samples = pathSamplesFromMessage(path);
  const OffboardTrajectoryState state =
      buildOffboardTrajectoryState(samples, VelocityFollowerConfig{});

  ASSERT_EQ(points.size(), 3U);
  ASSERT_EQ(samples.size(), 3U);
  EXPECT_EQ(messageStampNanoseconds(path.header.stamp), 2'000'000'003U);
  EXPECT_TRUE(state.valid);
  EXPECT_EQ(state.samples.size(), 3U);
  EXPECT_DOUBLE_EQ(state.samples[0].z_m, 10.0);
  EXPECT_DOUBLE_EQ(state.samples[1].z_m, 12.0);
  EXPECT_DOUBLE_EQ(state.samples[2].z_m, 14.0);
  EXPECT_EQ(state.trajectory.size(), 2U);
  EXPECT_EQ(state.stats.input_points, 3U);
  EXPECT_EQ(state.stats.samples, 3U);
  EXPECT_DOUBLE_EQ(state.metrics.length_m, 7.0);
}

TEST(OffboardTrajectoryState, EmptyPathBuildsInvalidState) {
  const std::vector<Point2> points;

  const OffboardTrajectoryState state =
      buildOffboardTrajectoryState(points, VelocityFollowerConfig{});

  EXPECT_FALSE(state.valid);
  EXPECT_TRUE(state.samples.empty());
  EXPECT_TRUE(state.trajectory.empty());
  EXPECT_EQ(state.stats.status, TrajectoryPlannerStatus::kInvalidTrajectory);
}

TEST(OffboardTrajectoryState, RejectsNonFiniteAltitudeBeforeStalePoseReset) {
  nav_msgs::msg::Path invalid_path = makePath();
  invalid_path.poses[1].pose.position.z = std::numeric_limits<double>::quiet_NaN();
  const std::vector<TrajectoryPointSample> invalid_samples =
      pathSamplesFromMessage(invalid_path);
  const OffboardTrajectoryState invalid_candidate =
      buildOffboardTrajectoryState(invalid_samples, VelocityFollowerConfig{});
  const std::vector<Point2> current_points{{0.0, 0.0}, {20.0, 0.0}};
  const OffboardTrajectoryState current_state =
      buildOffboardTrajectoryState(current_points, VelocityFollowerConfig{});

  ASSERT_TRUE(current_state.valid);
  ASSERT_FALSE(invalid_candidate.valid);

  const TrajectoryContinuityResult continuity =
      evaluateOffboardTrajectoryUpdateContinuity(
          current_state.samples, current_state.speed_profile, invalid_candidate,
          Point2{2.0, 0.0}, Point2{12.0, 0.0}, true, false);

  EXPECT_EQ(continuity.decision, TrajectoryContinuityDecision::kRejectTrajectory);
  EXPECT_STREQ(continuity.reason, "new_trajectory_invalid");
}

TEST(OffboardTrajectoryState, RejectsInvalidCandidateBeforeStalePoseReset) {
  const std::vector<Point2> current_points{{0.0, 0.0}, {20.0, 0.0}};
  const std::vector<Point2> invalid_points{{10.0, 0.0}};
  const OffboardTrajectoryState current_state =
      buildOffboardTrajectoryState(current_points, VelocityFollowerConfig{});
  const OffboardTrajectoryState invalid_candidate =
      buildOffboardTrajectoryState(invalid_points, VelocityFollowerConfig{});

  ASSERT_TRUE(current_state.valid);
  ASSERT_FALSE(invalid_candidate.valid);

  const TrajectoryContinuityResult continuity =
      evaluateOffboardTrajectoryUpdateContinuity(
          current_state.samples, current_state.speed_profile, invalid_candidate,
          Point2{2.0, 0.0}, Point2{12.0, 0.0}, true, false);

  EXPECT_EQ(continuity.decision, TrajectoryContinuityDecision::kRejectTrajectory);
  EXPECT_STREQ(continuity.reason, "new_trajectory_invalid");
}

TEST(OffboardTrajectoryState, ResetsForValidCandidateWhenPoseIsStale) {
  const std::vector<Point2> current_points{{0.0, 0.0}, {20.0, 0.0}};
  const std::vector<Point2> candidate_points{{0.0, 0.2}, {20.0, 0.2}};
  const OffboardTrajectoryState current_state =
      buildOffboardTrajectoryState(current_points, VelocityFollowerConfig{});
  const OffboardTrajectoryState candidate_state =
      buildOffboardTrajectoryState(candidate_points, VelocityFollowerConfig{});

  ASSERT_TRUE(current_state.valid);
  ASSERT_TRUE(candidate_state.valid);

  const TrajectoryContinuityResult continuity =
      evaluateOffboardTrajectoryUpdateContinuity(
          current_state.samples, current_state.speed_profile, candidate_state,
          Point2{2.0, 0.0}, Point2{12.0, 0.0}, true, false);

  EXPECT_EQ(continuity.decision, TrajectoryContinuityDecision::kResetSmoother);
  EXPECT_STREQ(continuity.reason, "pose_stale");
}

TEST(OffboardTrajectoryState, MatchesAndMergesPlannerDiagnostics) {
  TrajectoryPlannerDiagnosticsEnvelope diagnostics;
  diagnostics.path_stamp_ns = 100U;
  diagnostics.planner_path_id = 9U;
  diagnostics.stats.corridor.mean_width_m = 12.0;
  diagnostics.stats.trajectory_optimizer.final_length_m = 34.0;
  diagnostics.stats.turn_smoothing.smoothed_corners = 2U;
  diagnostics.stats.total_duration_ms = 56.0;
  diagnostics.stats.speed_profile_duration_ms = 99.0;
  diagnostics.stats.speed_profile_min_mps = 1.0;
  diagnostics.stats.speed_profile_construction_config_fingerprint = 10U;
  diagnostics.stats.runtime_speed_policy_config_fingerprint = 11U;
  diagnostics.stats.runtime_velocity_control_config_fingerprint = 12U;
  diagnostics.stats.top_speed_constraints.push_back(SpeedProfileConstraintDiagnostic{
      .sample_index = 7U,
      .s_m = 11.0,
      .radius_m = 12.0,
      .curvature_1pm = 0.08,
      .speed_limit_mps = 6.0,
      .profiled_limit_mps = 6.0,
      .source = SpeedConstraintType::kArc,
      .isolated_curvature_spike = false,
  });

  EXPECT_TRUE(trajectoryDiagnosticsMatchesPath(diagnostics, 100U, true, 9U));
  EXPECT_FALSE(trajectoryDiagnosticsMatchesPath(diagnostics, 101U, true, 9U));
  EXPECT_FALSE(trajectoryDiagnosticsMatchesPath(diagnostics, 100U, true, 10U));
  EXPECT_TRUE(trajectoryDiagnosticsMatchesPath(diagnostics, 100U, false, 10U));

  TrajectoryPlannerStats stats;
  stats.speed_profile_duration_ms = 1.5;
  stats.speed_profile_min_mps = 4.0;
  stats.speed_profile_construction_config_fingerprint = 20U;
  stats.runtime_speed_policy_config_fingerprint = 21U;
  stats.runtime_velocity_control_config_fingerprint = 22U;
  mergePlannerDiagnosticsIntoTrajectoryStats(stats, diagnostics);

  EXPECT_DOUBLE_EQ(stats.corridor.mean_width_m, 12.0);
  EXPECT_DOUBLE_EQ(stats.trajectory_optimizer.final_length_m, 34.0);
  EXPECT_EQ(stats.turn_smoothing.smoothed_corners, 2U);
  EXPECT_DOUBLE_EQ(stats.total_duration_ms, 56.0);
  EXPECT_DOUBLE_EQ(stats.speed_profile_duration_ms, 1.5);
  EXPECT_DOUBLE_EQ(stats.speed_profile_min_mps, 4.0);
  EXPECT_EQ(stats.speed_profile_construction_config_fingerprint, 20U);
  EXPECT_EQ(stats.runtime_speed_policy_config_fingerprint, 21U);
  EXPECT_EQ(stats.runtime_velocity_control_config_fingerprint, 22U);
  EXPECT_TRUE(stats.top_speed_constraints.empty());
}

} // namespace drone_city_nav
