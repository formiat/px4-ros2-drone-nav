#include "drone_city_nav/directed_inflation_escape.hpp"

#include <gtest/gtest.h>

#include <algorithm>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D emptyGrid(const int width = 24, const int height = 24) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, width, height}};
  grid.reset(CellState::kFree);
  return grid;
}

[[nodiscard]] DirectedInflationEscapeConfig testConfig() {
  return DirectedInflationEscapeConfig{
      .enabled = true,
      .tunnel_width_m = 5.0,
      .max_length_m = 20.0,
      .exit_depth_m = 1.0,
      .inflation_exposure_cost_weight = 1.0,
      .occupied_clearance_cost_weight = 2.0,
      .mission_egress_distance_m = 7.0,
      .stable_exit_cycles = 3U,
  };
}

} // namespace

TEST(DirectedInflationEscape, DoesNotStartFromAllowedCell) {
  DirectedInflationEscapePlanner planner;
  const OccupancyGrid2D grid = emptyGrid();

  const DirectedInflationEscapeResult result =
      planner.update(grid, Point2{4.5, 4.5}, Point2{20.5, 20.5}, testConfig());

  EXPECT_EQ(result.need, InflationEscapeNeed::kNotNeeded);
  EXPECT_EQ(result.state, DirectedInflationEscapeState::kInactive);
  EXPECT_FALSE(result.applied);
}

TEST(DirectedInflationEscape, RefusesOccupiedStart) {
  DirectedInflationEscapePlanner planner;
  OccupancyGrid2D grid = emptyGrid();
  grid.setOccupied(GridIndex{4, 4});

  const DirectedInflationEscapeResult result =
      planner.update(grid, Point2{4.5, 4.5}, Point2{20.5, 20.5}, testConfig());

  EXPECT_EQ(result.need, InflationEscapeNeed::kStartOccupied);
  EXPECT_EQ(result.state, DirectedInflationEscapeState::kFailed);
  EXPECT_FALSE(result.applied);
}

TEST(DirectedInflationEscape, ClearsTunnelAndPreservesOccupiedCells) {
  DirectedInflationEscapePlanner planner;
  OccupancyGrid2D grid = emptyGrid();
  grid.setOccupied(GridIndex{12, 12});
  grid.rebuildInflation(4.0);
  ASSERT_TRUE(grid.isInflated(GridIndex{9, 12}));

  DirectedInflationEscapeResult result =
      planner.update(grid, Point2{9.5, 12.5}, Point2{2.5, 12.5}, testConfig());
  ASSERT_EQ(result.need, InflationEscapeNeed::kNeeded);
  ASSERT_EQ(result.state, DirectedInflationEscapeState::kStarted);
  ASSERT_TRUE(result.applied);
  ASSERT_FALSE(result.centerline.empty());

  const LocalInflationRelaxationStats relaxation =
      applyDirectedInflationEscape(grid, result, testConfig().tunnel_width_m);

  EXPECT_GT(relaxation.inflated_cells_cleared, 0U);
  EXPECT_TRUE(grid.isOccupied(GridIndex{12, 12}));
  for (const Point2 point : result.centerline) {
    const auto cell = grid.worldToCell(point);
    ASSERT_TRUE(cell.has_value());
    EXPECT_FALSE(grid.isProhibited(*cell));
  }
}

TEST(DirectedInflationEscape, DoesNotCrossOccupiedWallForCloserEuclideanExit) {
  DirectedInflationEscapePlanner planner;
  OccupancyGrid2D grid = emptyGrid();
  for (int y = 2; y <= 21; ++y) {
    grid.setOccupied(GridIndex{12, y});
  }
  grid.rebuildInflation(3.0);
  ASSERT_TRUE(grid.isInflated(GridIndex{10, 12}));

  const DirectedInflationEscapeResult result =
      planner.update(grid, Point2{10.5, 12.5}, Point2{20.5, 12.5}, testConfig());

  ASSERT_TRUE(result.applied);
  ASSERT_FALSE(result.centerline.empty());
  EXPECT_TRUE(std::ranges::all_of(result.centerline,
                                  [](const Point2 point) { return point.x < 12.0; }));
}

TEST(DirectedInflationEscape, ReportsNoExitWithinMaximumLength) {
  DirectedInflationEscapePlanner planner;
  OccupancyGrid2D grid = emptyGrid();
  grid.setOccupied(GridIndex{12, 12});
  grid.rebuildInflation(10.0);
  DirectedInflationEscapeConfig config = testConfig();
  config.max_length_m = 1.0;

  const DirectedInflationEscapeResult result =
      planner.update(grid, Point2{9.5, 12.5}, Point2{2.5, 12.5}, config);

  EXPECT_EQ(result.need, InflationEscapeNeed::kNoReachableExit);
  EXPECT_EQ(result.state, DirectedInflationEscapeState::kFailed);
  EXPECT_FALSE(result.applied);
}

TEST(DirectedInflationEscape, KeepsEpisodeUntilStableExit) {
  DirectedInflationEscapePlanner planner;
  OccupancyGrid2D inflated = emptyGrid();
  inflated.setOccupied(GridIndex{12, 12});
  inflated.rebuildInflation(4.0);
  const DirectedInflationEscapeConfig config = testConfig();

  const DirectedInflationEscapeResult started =
      planner.update(inflated, Point2{9.5, 12.5}, Point2{2.5, 12.5}, config);
  ASSERT_TRUE(started.applied);

  OccupancyGrid2D clear = emptyGrid();
  const DirectedInflationEscapeResult first =
      planner.update(clear, started.target, Point2{2.5, 12.5}, config);
  const DirectedInflationEscapeResult second =
      planner.update(clear, started.target, Point2{2.5, 12.5}, config);
  const DirectedInflationEscapeResult third =
      planner.update(clear, started.target, Point2{2.5, 12.5}, config);

  EXPECT_EQ(first.state, DirectedInflationEscapeState::kActive);
  EXPECT_EQ(second.state, DirectedInflationEscapeState::kActive);
  EXPECT_EQ(third.state, DirectedInflationEscapeState::kCompleted);
  EXPECT_EQ(third.episode_generation, started.episode_generation);
}

TEST(DirectedInflationEscape, CompletesAfterPassingTargetInAllowedSpace) {
  DirectedInflationEscapePlanner planner;
  OccupancyGrid2D inflated = emptyGrid();
  inflated.setOccupied(GridIndex{12, 12});
  inflated.rebuildInflation(4.0);
  const DirectedInflationEscapeConfig config = testConfig();

  const DirectedInflationEscapeResult started =
      planner.update(inflated, Point2{9.5, 12.5}, Point2{2.5, 12.5}, config);
  ASSERT_TRUE(started.applied);

  OccupancyGrid2D clear = emptyGrid();
  const Point2 past_target{started.target.x - 4.0, started.target.y};
  EXPECT_EQ(planner.update(clear, past_target, Point2{2.5, 12.5}, config).state,
            DirectedInflationEscapeState::kActive);
  EXPECT_EQ(planner.update(clear, past_target, Point2{2.5, 12.5}, config).state,
            DirectedInflationEscapeState::kActive);
  EXPECT_EQ(planner.update(clear, past_target, Point2{2.5, 12.5}, config).state,
            DirectedInflationEscapeState::kCompleted);
}

TEST(DirectedInflationEscape, DoesNotCompleteWithoutMissionEgress) {
  DirectedInflationEscapePlanner planner;
  OccupancyGrid2D inflated = emptyGrid();
  inflated.setOccupied(GridIndex{12, 12});
  inflated.rebuildInflation(4.0);
  const DirectedInflationEscapeConfig config = testConfig();

  const DirectedInflationEscapeResult started =
      planner.update(inflated, Point2{9.5, 12.5}, Point2{2.5, 12.5}, config);
  ASSERT_TRUE(started.applied);

  OccupancyGrid2D blocked_egress = emptyGrid();
  blocked_egress.setOccupied(GridIndex{static_cast<int>(started.target.x) - 2,
                                       static_cast<int>(started.target.y)});
  for (std::size_t cycle = 0U; cycle < config.stable_exit_cycles + 1U; ++cycle) {
    const DirectedInflationEscapeResult active =
        planner.update(blocked_egress, started.target, Point2{2.5, 12.5}, config);
    EXPECT_EQ(active.state, DirectedInflationEscapeState::kActive);
    EXPECT_FALSE(active.mission_egress_available);
  }
}

TEST(DirectedInflationEscape, KeepsTargetStableAcrossGridUpdates) {
  DirectedInflationEscapePlanner planner;
  OccupancyGrid2D grid = emptyGrid();
  grid.setOccupied(GridIndex{12, 12});
  grid.rebuildInflation(4.0);
  const DirectedInflationEscapeConfig config = testConfig();

  const DirectedInflationEscapeResult started =
      planner.update(grid, Point2{9.5, 12.5}, Point2{2.5, 12.5}, config);
  ASSERT_TRUE(started.applied);
  grid.setOccupied(GridIndex{20, 20});

  const DirectedInflationEscapeResult active =
      planner.update(grid, Point2{9.5, 12.5}, Point2{2.5, 12.5}, config);

  EXPECT_EQ(active.state, DirectedInflationEscapeState::kActive);
  EXPECT_EQ(active.episode_generation, started.episode_generation);
  EXPECT_DOUBLE_EQ(active.target.x, started.target.x);
  EXPECT_DOUBLE_EQ(active.target.y, started.target.y);
}

TEST(DirectedInflationEscape, RebuildsEpisodeWhenCenterlineBecomesOccupied) {
  DirectedInflationEscapePlanner planner;
  OccupancyGrid2D grid = emptyGrid();
  grid.setOccupied(GridIndex{12, 12});
  grid.rebuildInflation(4.0);
  const DirectedInflationEscapeConfig config = testConfig();

  const DirectedInflationEscapeResult started =
      planner.update(grid, Point2{9.5, 12.5}, Point2{2.5, 12.5}, config);
  ASSERT_GT(started.centerline.size(), 2U);
  const Point2 blocked_point = started.centerline[started.centerline.size() / 2U];
  const auto blocked_cell = grid.worldToCell(blocked_point);
  ASSERT_TRUE(blocked_cell.has_value());
  grid.setOccupied(*blocked_cell);

  const DirectedInflationEscapeResult rebuilt =
      planner.update(grid, Point2{9.5, 12.5}, Point2{2.5, 12.5}, config);

  EXPECT_TRUE(rebuilt.centerline_blocked);
  if (rebuilt.applied) {
    EXPECT_GT(rebuilt.episode_generation, started.episode_generation);
    EXPECT_TRUE(
        std::ranges::none_of(rebuilt.centerline, [blocked_point](const Point2 point) {
          return point.x == blocked_point.x && point.y == blocked_point.y;
        }));
  }
}

TEST(DirectedInflationEscape, RebuildsEpisodeAfterLeavingTunnelWhileInflated) {
  DirectedInflationEscapePlanner planner;
  OccupancyGrid2D grid = emptyGrid(40, 40);
  grid.setOccupied(GridIndex{20, 20});
  grid.rebuildInflation(12.0);
  DirectedInflationEscapeConfig config = testConfig();
  config.max_length_m = 30.0;

  const DirectedInflationEscapeResult started =
      planner.update(grid, Point2{10.5, 20.5}, Point2{2.5, 20.5}, config);
  ASSERT_TRUE(started.applied);

  const DirectedInflationEscapeResult rebuilt =
      planner.update(grid, Point2{20.5, 10.5}, Point2{20.5, 2.5}, config);

  EXPECT_TRUE(rebuilt.episode_off_centerline);
  ASSERT_TRUE(rebuilt.applied);
  EXPECT_GT(rebuilt.episode_generation, started.episode_generation);
}

} // namespace drone_city_nav
