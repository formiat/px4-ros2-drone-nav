#include "drone_city_nav/receding_horizon_trajectory_planner.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D freeGrid() {
  OccupancyGrid2D grid{GridBounds{-20.0, -20.0, 0.5, 160, 160}};
  grid.reset(CellState::kFree);
  return grid;
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
  EXPECT_NEAR(result.ranked_candidates.front().samples.back().point.x, 25.0, 0.1);
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

} // namespace drone_city_nav
