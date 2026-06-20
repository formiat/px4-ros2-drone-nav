#include "drone_city_nav/offboard_target_continuity.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D emptyGrid() {
  OccupancyGrid2D grid{GridBounds{-5.0, -5.0, 1.0, 30, 30}};
  grid.reset(CellState::kFree);
  return grid;
}

} // namespace

TEST(OffboardTargetContinuity, KeepsPreviousTargetWhenItIsOnNewPathAndSafe) {
  const std::vector<Point2> path{{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};
  OccupancyGrid2D grid = emptyGrid();

  const TargetContinuityDecision decision =
      decideTargetAfterReplan(path, Point2{2.0, 0.0}, Point2{10.1, 0.2},
                              Point2{10.0, 0.0}, true, 1U, 1.0, &grid);

  EXPECT_EQ(decision.reason, TargetContinuityDecisionReason::kKeptPreviousTarget);
  EXPECT_TRUE(decision.previous_target_safe);
  EXPECT_NEAR(decision.target.x, 10.1, 1.0e-9);
  EXPECT_NEAR(decision.target.y, 0.2, 1.0e-9);
  EXPECT_NEAR(decision.path_error_m, 0.2, 1.0e-9);
}

TEST(OffboardTargetContinuity, SwitchesWhenPreviousTargetIsFarFromNewPath) {
  const std::vector<Point2> path{{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};
  OccupancyGrid2D grid = emptyGrid();

  const TargetContinuityDecision decision =
      decideTargetAfterReplan(path, Point2{2.0, 0.0}, Point2{10.0, 5.0},
                              Point2{10.0, 0.0}, true, 1U, 1.0, &grid);

  EXPECT_EQ(decision.reason, TargetContinuityDecisionReason::kSwitchedToNewWaypoint);
  EXPECT_NEAR(decision.target.x, 10.0, 1.0e-9);
  EXPECT_NEAR(decision.target.y, 0.0, 1.0e-9);
  EXPECT_GT(decision.path_error_m, 1.0);
}

TEST(OffboardTargetContinuity, SwitchesWhenPreviousTargetSegmentCrossesProhibited) {
  const std::vector<Point2> path{{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};
  OccupancyGrid2D grid = emptyGrid();
  grid.setOccupied(GridIndex{10, 5});
  grid.rebuildInflation(0.0);

  const TargetContinuityDecision decision =
      decideTargetAfterReplan(path, Point2{2.0, 0.0}, Point2{10.0, 0.0},
                              Point2{10.0, 0.0}, true, 1U, 1.0, &grid);

  EXPECT_EQ(decision.reason,
            TargetContinuityDecisionReason::kForcedSwitchUnsafePrevious);
  EXPECT_FALSE(decision.previous_target_safe);
}

TEST(OffboardTargetContinuity, AllowsEscapeFromProhibitedAtSegmentStart) {
  const std::vector<Point2> path{{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};
  OccupancyGrid2D grid = emptyGrid();
  grid.setOccupied(GridIndex{7, 5});
  grid.rebuildInflation(0.0);

  const TargetContinuityDecision decision =
      decideTargetAfterReplan(path, Point2{2.1, 0.0}, Point2{10.0, 0.0},
                              Point2{10.0, 0.0}, true, 1U, 1.0, &grid);

  EXPECT_EQ(decision.reason, TargetContinuityDecisionReason::kKeptPreviousTarget);
  EXPECT_TRUE(decision.previous_target_safe);
}

TEST(OffboardTargetContinuity, SwitchesWhenPreviousTargetIsBehindPathProgress) {
  const std::vector<Point2> path{{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}, {30.0, 0.0}};
  OccupancyGrid2D grid = emptyGrid();

  const TargetContinuityDecision decision =
      decideTargetAfterReplan(path, Point2{20.0, 0.0}, Point2{5.0, 0.0},
                              Point2{20.0, 0.0}, true, 2U, 1.0, &grid);

  EXPECT_EQ(decision.reason,
            TargetContinuityDecisionReason::kForcedSwitchPreviousBehindPath);
  EXPECT_NEAR(decision.target.x, 20.0, 1.0e-9);
}

TEST(OffboardTargetContinuity, NoPreviousTargetUsesProposedTarget) {
  const std::vector<Point2> path{{0.0, 0.0}, {10.0, 0.0}};
  OccupancyGrid2D grid = emptyGrid();

  const TargetContinuityDecision decision = decideTargetAfterReplan(
      path, Point2{0.0, 0.0}, Point2{}, Point2{10.0, 0.0}, false, 0U, 1.0, &grid);

  EXPECT_EQ(decision.reason, TargetContinuityDecisionReason::kNoPreviousTarget);
  EXPECT_NEAR(decision.target.x, 10.0, 1.0e-9);
  EXPECT_NEAR(decision.target.y, 0.0, 1.0e-9);
}

} // namespace drone_city_nav
