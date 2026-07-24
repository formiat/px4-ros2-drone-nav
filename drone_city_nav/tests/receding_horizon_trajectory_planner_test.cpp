#include "drone_city_nav/receding_horizon_trajectory_planner.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D freeGrid() {
  OccupancyGrid2D grid{GridBounds{-20.0, -20.0, 0.5, 160, 160}};
  grid.reset(CellState::kFree);
  return grid;
}

void occupyWorldPoint(OccupancyGrid2D& grid, const Point2 point) {
  const std::optional<GridIndex> cell = grid.worldToCell(point);
  ASSERT_TRUE(cell.has_value());
  grid.setOccupied(*cell);
}

} // namespace

TEST(RecedingHorizonTrajectoryPlanner, SelectsDirectOpenSpaceCandidate) {
  OccupancyGrid2D raw = freeGrid();
  OccupancyGrid2D prohibited = raw;
  OccupancyGrid2D planning = raw;
  const RecedingHorizonTrajectoryPlanner planner;

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {50.0, 0.0},
                                .raw_grid = &raw,
                                .prohibited_grid = &prohibited,
                                .planning_grid = &planning});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_NEAR(result.ranked_candidates.front().heading_offset_rad, 0.0, 1.0e-9);
  EXPECT_EQ(result.ranked_candidates.front().tier,
            RolloutTraversabilityTier::kPlanningClearance);
  EXPECT_GT(result.ranked_candidates.front().samples.back().point.x, 0.0);
  EXPECT_LE(result.ranked_candidates.front().samples.back().point.x, 25.0);
}

TEST(RecedingHorizonTrajectoryPlanner, RejectsRawOccupiedCandidates) {
  OccupancyGrid2D raw = freeGrid();
  OccupancyGrid2D prohibited = raw;
  OccupancyGrid2D planning = raw;
  for (int y = 0; y < raw.height(); ++y) {
    for (int x = 0; x < raw.width(); ++x) {
      raw.setOccupied(GridIndex{x, y});
    }
  }
  raw.setFree(GridIndex{40, 40});
  const RecedingHorizonTrajectoryPlanner planner;

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {50.0, 0.0},
                                .raw_grid = &raw,
                                .prohibited_grid = &prohibited,
                                .planning_grid = &planning});

  EXPECT_TRUE(result.ranked_candidates.empty());
  EXPECT_EQ(result.reject_reason, RolloutRejectReason::kNoCandidate);
  EXPECT_GT(result.diagnostics.raw_occupied_rejections, 0U);
}

TEST(RecedingHorizonTrajectoryPlanner, UsesDeterministicTieBreak) {
  OccupancyGrid2D raw = freeGrid();
  OccupancyGrid2D prohibited = raw;
  OccupancyGrid2D planning = raw;
  const RecedingHorizonTrajectoryPlanner planner{
      RolloutPlannerConfig{.heading_samples = 3U, .max_heading_offset_rad = 0.5}};
  const RolloutInput input{.position = {0.0, 0.0},
                           .velocity = {0.0, 0.0},
                           .preferred_target = {30.0, 0.0},
                           .raw_grid = &raw,
                           .prohibited_grid = &prohibited,
                           .planning_grid = &planning};

  const RolloutResult first = planner.plan(input);
  const RolloutResult second = planner.plan(input);

  ASSERT_EQ(first.ranked_candidates.size(), second.ranked_candidates.size());
  for (std::size_t index = 0U; index < first.ranked_candidates.size(); ++index) {
    EXPECT_EQ(first.ranked_candidates[index].deterministic_index,
              second.ranked_candidates[index].deterministic_index);
  }
}

TEST(RecedingHorizonTrajectoryPlanner, RejectsRawObstacleBetweenSparseSamples) {
  OccupancyGrid2D raw = freeGrid();
  OccupancyGrid2D prohibited = raw;
  OccupancyGrid2D planning = raw;
  occupyWorldPoint(raw, Point2{1.5, 0.0});
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 6.0,
      .sample_step_m = 5.0,
      .heading_samples = 1U,
      .speed_samples = 1U,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 2.0,
  }};

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .raw_grid = &raw,
                                .prohibited_grid = &prohibited,
                                .planning_grid = &planning});

  EXPECT_TRUE(result.ranked_candidates.empty());
  EXPECT_GT(result.diagnostics.raw_occupied_rejections, 0U);
}

TEST(RecedingHorizonTrajectoryPlanner, FindsLeftOrRightAvoidanceCandidate) {
  OccupancyGrid2D raw = freeGrid();
  OccupancyGrid2D prohibited = raw;
  OccupancyGrid2D planning = raw;
  for (int y_index = -1; y_index <= 1; ++y_index) {
    occupyWorldPoint(raw, Point2{5.0, 0.5 * static_cast<double>(y_index)});
  }
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .horizon_m = 12.0,
      .heading_samples = 9U,
      .speed_samples = 1U,
      .minimum_speed_mps = 4.0,
      .maximum_speed_mps = 4.0,
      .maximum_lateral_acceleration_mps2 = 10.0,
  }};

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {4.0, 0.0},
                                .preferred_target = {30.0, 0.0},
                                .raw_grid = &raw,
                                .prohibited_grid = &prohibited,
                                .planning_grid = &planning});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_NE(result.ranked_candidates.front().heading_offset_rad, 0.0);
}

TEST(RecedingHorizonTrajectoryPlanner, UsesDegradedTraversabilityTier) {
  OccupancyGrid2D raw = freeGrid();
  OccupancyGrid2D prohibited = raw;
  OccupancyGrid2D planning = raw;
  occupyWorldPoint(planning, Point2{3.0, 0.0});
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .heading_samples = 1U,
      .speed_samples = 1U,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 2.0,
  }};

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {30.0, 0.0},
                                .raw_grid = &raw,
                                .prohibited_grid = &prohibited,
                                .planning_grid = &planning});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_EQ(result.ranked_candidates.front().tier,
            RolloutTraversabilityTier::kRuntimeProhibited);
}

TEST(RecedingHorizonTrajectoryPlanner, RejectsDynamicLimitViolation) {
  OccupancyGrid2D raw = freeGrid();
  OccupancyGrid2D prohibited = raw;
  OccupancyGrid2D planning = raw;
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .heading_samples = 1U,
      .speed_samples = 1U,
      .minimum_speed_mps = 5.0,
      .maximum_speed_mps = 5.0,
      .maximum_curvature_1pm = 0.01,
  }};

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {5.0, 0.0},
                                .preferred_target = {0.0, 30.0},
                                .raw_grid = &raw,
                                .prohibited_grid = &prohibited,
                                .planning_grid = &planning});

  EXPECT_TRUE(result.ranked_candidates.empty());
  EXPECT_GT(result.diagnostics.dynamic_limit_rejections, 0U);
}

TEST(RecedingHorizonTrajectoryPlanner, StopsAtGoalInsideHorizon) {
  OccupancyGrid2D raw = freeGrid();
  OccupancyGrid2D prohibited = raw;
  OccupancyGrid2D planning = raw;
  const RecedingHorizonTrajectoryPlanner planner;

  const RolloutResult result = planner.plan(RolloutInput{.position = {0.0, 0.0},
                                                         .velocity = {2.0, 0.0},
                                                         .preferred_target = {4.0, 0.0},
                                                         .raw_grid = &raw,
                                                         .prohibited_grid = &prohibited,
                                                         .planning_grid = &planning});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_NEAR(result.ranked_candidates.front().samples.back().point.x, 4.0, 0.1);
}

TEST(RecedingHorizonTrajectoryPlanner, HandlesZeroVelocityStart) {
  OccupancyGrid2D raw = freeGrid();
  OccupancyGrid2D prohibited = raw;
  OccupancyGrid2D planning = raw;
  const RecedingHorizonTrajectoryPlanner planner;

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {0.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .raw_grid = &raw,
                                .prohibited_grid = &prohibited,
                                .planning_grid = &planning});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_GT(result.ranked_candidates.front().progress_m, 0.0);
}

} // namespace drone_city_nav
