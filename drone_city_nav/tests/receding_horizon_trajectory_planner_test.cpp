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
  OccupancyGrid2D prohibited = freeGrid();
  const RecedingHorizonTrajectoryPlanner planner;

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {50.0, 0.0},
                                .grid = &prohibited});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_NEAR(result.ranked_candidates.front().heading_offset_rad, 0.0, 1.0e-9);
  EXPECT_GT(result.ranked_candidates.front().samples.back().point.x, 0.0);
  EXPECT_LE(result.ranked_candidates.front().samples.back().point.x, 25.0);
}

TEST(RecedingHorizonTrajectoryPlanner, RejectsProhibitedCandidates) {
  OccupancyGrid2D prohibited = freeGrid();
  for (int y = 0; y < prohibited.height(); ++y) {
    for (int x = 0; x < prohibited.width(); ++x) {
      prohibited.setOccupied(GridIndex{x, y});
    }
  }
  prohibited.setFree(GridIndex{40, 40});
  const RecedingHorizonTrajectoryPlanner planner;

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {50.0, 0.0},
                                .grid = &prohibited});

  EXPECT_TRUE(result.ranked_candidates.empty());
  EXPECT_EQ(result.reject_reason, RolloutRejectReason::kNoCandidate);
  EXPECT_GT(result.diagnostics.grid_rejections, 0U);
}

TEST(RecedingHorizonTrajectoryPlanner, UsesDeterministicTieBreak) {
  OccupancyGrid2D prohibited = freeGrid();
  const RecedingHorizonTrajectoryPlanner planner{
      RolloutPlannerConfig{.heading_samples = 3U, .max_heading_offset_rad = 0.5}};
  const RolloutInput input{.position = {0.0, 0.0},
                           .velocity = {0.0, 0.0},
                           .preferred_target = {30.0, 0.0},
                           .grid = &prohibited};

  const RolloutResult first = planner.plan(input);
  const RolloutResult second = planner.plan(input);

  ASSERT_EQ(first.ranked_candidates.size(), second.ranked_candidates.size());
  for (std::size_t index = 0U; index < first.ranked_candidates.size(); ++index) {
    EXPECT_EQ(first.ranked_candidates[index].deterministic_index,
              second.ranked_candidates[index].deterministic_index);
  }
}

TEST(RecedingHorizonTrajectoryPlanner, RejectsProhibitedObstacleBetweenSparseSamples) {
  OccupancyGrid2D prohibited = freeGrid();
  occupyWorldPoint(prohibited, Point2{1.5, 0.0});
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
                                .grid = &prohibited});

  EXPECT_TRUE(result.ranked_candidates.empty());
  EXPECT_GT(result.diagnostics.grid_rejections, 0U);
}

TEST(RecedingHorizonTrajectoryPlanner, FindsLeftOrRightAvoidanceCandidate) {
  OccupancyGrid2D prohibited = freeGrid();
  for (int y_index = -1; y_index <= 1; ++y_index) {
    occupyWorldPoint(prohibited, Point2{5.0, 0.5 * static_cast<double>(y_index)});
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
                                .grid = &prohibited});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_NE(result.ranked_candidates.front().heading_offset_rad, 0.0);
}

TEST(RecedingHorizonTrajectoryPlanner, EvaluatesOnlyTheRequestedGrid) {
  OccupancyGrid2D prohibited = freeGrid();
  OccupancyGrid2D planning = prohibited;
  occupyWorldPoint(planning, Point2{3.0, 0.0});
  const RecedingHorizonTrajectoryPlanner planner{RolloutPlannerConfig{
      .heading_samples = 1U,
      .speed_samples = 1U,
      .minimum_speed_mps = 2.0,
      .maximum_speed_mps = 2.0,
  }};

  const RolloutResult planning_result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {30.0, 0.0},
                                .grid = &planning});
  const RolloutResult prohibited_result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {2.0, 0.0},
                                .preferred_target = {30.0, 0.0},
                                .grid = &prohibited});

  EXPECT_TRUE(planning_result.ranked_candidates.empty());
  EXPECT_FALSE(prohibited_result.ranked_candidates.empty());
}

TEST(RecedingHorizonTrajectoryPlanner, RejectsDynamicLimitViolation) {
  OccupancyGrid2D prohibited = freeGrid();
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
                                .grid = &prohibited});

  EXPECT_TRUE(result.ranked_candidates.empty());
  EXPECT_GT(result.diagnostics.dynamic_limit_rejections, 0U);
}

TEST(RecedingHorizonTrajectoryPlanner, StopsAtGoalInsideHorizon) {
  OccupancyGrid2D prohibited = freeGrid();
  const RecedingHorizonTrajectoryPlanner planner;

  const RolloutResult result = planner.plan(RolloutInput{.position = {0.0, 0.0},
                                                         .velocity = {2.0, 0.0},
                                                         .preferred_target = {4.0, 0.0},
                                                         .grid = &prohibited});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_NEAR(result.ranked_candidates.front().samples.back().point.x, 4.0, 0.1);
}

TEST(RecedingHorizonTrajectoryPlanner, HandlesZeroVelocityStart) {
  OccupancyGrid2D prohibited = freeGrid();
  const RecedingHorizonTrajectoryPlanner planner;

  const RolloutResult result =
      planner.plan(RolloutInput{.position = {0.0, 0.0},
                                .velocity = {0.0, 0.0},
                                .preferred_target = {20.0, 0.0},
                                .grid = &prohibited});

  ASSERT_FALSE(result.ranked_candidates.empty());
  EXPECT_GT(result.ranked_candidates.front().progress_m, 0.0);
}

} // namespace drone_city_nav
