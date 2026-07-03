#include "drone_city_nav/offboard_trajectory_state.hpp"

#include <nav_msgs/msg/path.hpp>

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] nav_msgs::msg::Path makePath() {
  nav_msgs::msg::Path path;
  path.header.stamp.sec = 2;
  path.header.stamp.nanosec = 3;
  geometry_msgs::msg::PoseStamped first;
  first.pose.position.x = 0.0;
  first.pose.position.y = 0.0;
  geometry_msgs::msg::PoseStamped second;
  second.pose.position.x = 4.0;
  second.pose.position.y = 0.0;
  geometry_msgs::msg::PoseStamped third;
  third.pose.position.x = 4.0;
  third.pose.position.y = 3.0;
  path.poses = {first, second, third};
  return path;
}

} // namespace

TEST(OffboardTrajectoryState, ConvertsPathAndBuildsExecutableState) {
  const nav_msgs::msg::Path path = makePath();

  const std::vector<Point2> points = pathPointsFromMessage(path);
  const OffboardTrajectoryState state =
      buildOffboardTrajectoryState(points, VelocityFollowerConfig{});

  ASSERT_EQ(points.size(), 3U);
  EXPECT_EQ(messageStampNanoseconds(path.header.stamp), 2'000'000'003U);
  EXPECT_TRUE(state.valid);
  EXPECT_EQ(state.samples.size(), 3U);
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

TEST(OffboardTrajectoryState, MatchesAndMergesPlannerDiagnostics) {
  TrajectoryPlannerDiagnosticsEnvelope diagnostics;
  diagnostics.path_stamp_ns = 100U;
  diagnostics.planner_path_id = 9U;
  diagnostics.stats.corridor.mean_width_m = 12.0;
  diagnostics.stats.trajectory_optimizer.final_length_m = 34.0;
  diagnostics.stats.turn_smoothing.smoothed_corners = 2U;
  diagnostics.stats.total_duration_ms = 56.0;

  EXPECT_TRUE(trajectoryDiagnosticsMatchesPath(diagnostics, 100U, true, 9U));
  EXPECT_FALSE(trajectoryDiagnosticsMatchesPath(diagnostics, 101U, true, 9U));
  EXPECT_FALSE(trajectoryDiagnosticsMatchesPath(diagnostics, 100U, true, 10U));
  EXPECT_TRUE(trajectoryDiagnosticsMatchesPath(diagnostics, 100U, false, 10U));

  TrajectoryPlannerStats stats;
  mergePlannerDiagnosticsIntoTrajectoryStats(stats, diagnostics);

  EXPECT_DOUBLE_EQ(stats.corridor.mean_width_m, 12.0);
  EXPECT_DOUBLE_EQ(stats.trajectory_optimizer.final_length_m, 34.0);
  EXPECT_EQ(stats.turn_smoothing.smoothed_corners, 2U);
  EXPECT_DOUBLE_EQ(stats.total_duration_ms, 56.0);
}

} // namespace drone_city_nav
