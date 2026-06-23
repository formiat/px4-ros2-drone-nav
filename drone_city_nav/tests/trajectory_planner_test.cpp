#include "drone_city_nav/trajectory_planner.hpp"

#include <gtest/gtest.h>

#include <span>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D testGrid() {
  OccupancyGrid2D grid{GridBounds{-20.0, -20.0, 1.0, 80, 80}};
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      grid.setFree(GridIndex{x, y});
    }
  }
  return grid;
}

[[nodiscard]] TrajectoryPlannerConfig testConfig() {
  TrajectoryPlannerConfig config{};
  config.corridor.max_radius_m = 10.0;
  config.corridor.sample_step_m = 2.5;
  config.corridor.safety_margin_m = 0.0;
  config.racing_line.max_iterations = 20U;
  config.racing_line.initial_offset_step_m = 1.0;
  config.racing_line.min_offset_step_m = 0.1;
  config.racing_line.weight_curvature = 30.0;
  config.racing_line.weight_center_bias = 0.0;
  config.debug_sample_step_m = 1.0;
  config.speed_profile.cruise_speed_mps = 12.0;
  config.speed_profile.max_lateral_accel_mps2 = 3.0;
  config.speed_profile.speed_profile_decel_mps2 = 4.0;
  config.speed_profile.speed_profile_sample_step_m = 1.0;
  return config;
}

} // namespace

TEST(TrajectoryPlanner, EmptyRouteIsInvalid) {
  const TrajectoryPlannerResult result =
      planTrajectory(TrajectoryPlannerInput{}, testConfig());

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kInvalidRoute);
}

TEST(TrajectoryPlanner, MissingGridIsInvalid) {
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}};

  const TrajectoryPlannerResult result = planTrajectory(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()},
                             nullptr},
      testConfig());

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kMissingGrid);
  EXPECT_TRUE(result.compact_segments.empty());
  EXPECT_TRUE(result.samples.empty());
  EXPECT_FALSE(result.speed_profile.valid);
}

TEST(TrajectoryPlanner, MissingGridDoesNotBuildUnknownCorners) {
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};

  const TrajectoryPlannerResult result = planTrajectory(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()},
                             nullptr},
      testConfig());

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kMissingGrid);
  EXPECT_EQ(result.stats.arc_segments, 0U);
  EXPECT_TRUE(result.compact_segments.empty());
}

TEST(TrajectoryPlanner, RacingTrajectoryProducesSamplesAndSpeedProfile) {
  const OccupancyGrid2D grid = testGrid();
  const std::vector<Point2> route{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};

  const TrajectoryPlannerResult result = planTrajectory(
      TrajectoryPlannerInput{std::span<const Point2>{route.data(), route.size()},
                             &grid},
      testConfig());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.stats.status, TrajectoryPlannerStatus::kOk);
  EXPECT_GE(result.samples.size(), 3U);
  EXPECT_TRUE(result.speed_profile.valid);
  EXPECT_GT(result.stats.corridor.samples, 0U);
  EXPECT_GT(result.stats.racing_line.candidate_evaluations, 0U);
}

TEST(TrajectoryGridRebuildReason, InvalidTrajectoryRebuildsOnFirstGrid) {
  TrajectoryGridRebuildDecisionInput input{};
  input.trajectory_valid = false;
  input.status = TrajectoryPlannerStatus::kMissingGrid;

  EXPECT_EQ(trajectoryGridRebuildReason(input),
            TrajectoryGridRebuildReason::kInvalidTrajectory);
}

TEST(TrajectoryGridRebuildReason, UnchangedCorridorDoesNotChurn) {
  CorridorStats corridor{};
  corridor.samples = 5U;
  corridor.min_width_m = 8.0;
  corridor.mean_width_m = 9.0;
  corridor.max_width_m = 10.0;
  corridor.min_clearance_m = 4.0;

  TrajectoryGridRebuildDecisionInput input{};
  input.trajectory_valid = true;
  input.current_corridor_valid = true;
  input.previous_corridor = corridor;
  input.current_corridor = corridor;
  input.corridor_width_threshold_m = 0.5;

  EXPECT_EQ(trajectoryGridRebuildReason(input), TrajectoryGridRebuildReason::kNone);
}

TEST(TrajectoryGridRebuildReason, ProhibitedIntersectionForcesRebuild) {
  TrajectoryGridRebuildDecisionInput input{};
  input.trajectory_valid = true;
  input.final_trajectory_intersects_prohibited = true;

  EXPECT_EQ(trajectoryGridRebuildReason(input),
            TrajectoryGridRebuildReason::kProhibitedIntersection);
}

TEST(TrajectoryGridRebuildReason, MaterialCorridorChangeForcesRebuild) {
  CorridorStats previous{};
  previous.samples = 5U;
  previous.min_width_m = 8.0;
  previous.mean_width_m = 9.0;
  previous.max_width_m = 10.0;
  previous.min_clearance_m = 4.0;
  CorridorStats current = previous;
  current.min_width_m = 7.0;

  TrajectoryGridRebuildDecisionInput input{};
  input.trajectory_valid = true;
  input.current_corridor_valid = true;
  input.previous_corridor = previous;
  input.current_corridor = current;
  input.corridor_width_threshold_m = 0.5;

  EXPECT_EQ(trajectoryGridRebuildReason(input),
            TrajectoryGridRebuildReason::kCorridorBoundsChanged);
}

} // namespace drone_city_nav
