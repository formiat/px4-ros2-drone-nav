#include "drone_city_nav/obstacle_risk_field.hpp"

#include <gtest/gtest.h>

#include <array>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D makeGrid() {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 1.0, 20, 20}};
  grid.reset(CellState::kFree);
  grid.setOccupied(GridIndex{5, 5});
  return grid;
}

} // namespace

TEST(ObstacleRiskField, ClassifiesConfiguredDistanceBands) {
  const OccupancyGrid2D grid = makeGrid();
  const ObstacleRiskField field = ObstacleRiskField::build(
      grid, {.critical_distance_m = 1.5, .preferred_distance_m = 4.0});

  EXPECT_EQ(field.tierAt(GridIndex{5, 5}), ObstacleRiskTier::kCriticalBand);
  EXPECT_EQ(field.tierAt(GridIndex{6, 5}), ObstacleRiskTier::kCriticalBand);
  EXPECT_EQ(field.tierAt(GridIndex{7, 5}), ObstacleRiskTier::kPlanningBand);
  EXPECT_EQ(field.tierAt(GridIndex{10, 5}), ObstacleRiskTier::kPreferred);
}

TEST(ObstacleRiskField, EvaluatesWholeSegmentAndRejectsRawCollision) {
  const OccupancyGrid2D grid = makeGrid();
  const ObstacleRiskField field = ObstacleRiskField::build(
      grid, {.critical_distance_m = 1.0, .preferred_distance_m = 4.0});
  const std::array<Point2, 2U> points{Point2{1.5, 5.5}, Point2{9.5, 5.5}};

  const PathRiskScore score = field.evaluate(grid, points);

  EXPECT_TRUE(score.intersects_raw_occupied);
  EXPECT_FALSE(score.hardValid());
  EXPECT_EQ(score.minimum_raw_clearance_m, 0.0);
}

TEST(ObstacleRiskField, LexicographicTierDominatesGeometryCost) {
  RankedPathCost preferred{};
  preferred.algorithm_cost = 1.0e9;
  RankedPathCost critical{};
  critical.risk.worst_tier = ObstacleRiskTier::kCriticalBand;
  critical.risk.critical_exposure_m = 0.1;
  critical.algorithm_cost = -1.0e9;

  EXPECT_TRUE(rankedPathCostLess(preferred, critical));
  EXPECT_FALSE(rankedPathCostLess(critical, preferred));
}

TEST(ObstacleRiskField, ExposureBreaksTiesWithinTier) {
  PathRiskScore shorter{};
  shorter.worst_tier = ObstacleRiskTier::kCriticalBand;
  shorter.critical_exposure_m = 1.0;
  PathRiskScore longer = shorter;
  longer.critical_exposure_m = 2.0;

  EXPECT_TRUE(pathRiskLess(shorter, longer));
  EXPECT_FALSE(pathRiskEqual(shorter, longer));
}

TEST(ObstacleRiskField, OutsideBoundsIsHardInvalid) {
  const OccupancyGrid2D grid = makeGrid();
  const ObstacleRiskField field = ObstacleRiskField::build(
      grid, {.critical_distance_m = 1.0, .preferred_distance_m = 4.0});
  const std::array<Point2, 2U> points{Point2{1.0, 1.0}, Point2{-1.0, 1.0}};

  const PathRiskScore score = field.evaluate(grid, points);

  EXPECT_TRUE(score.outside_bounds);
  EXPECT_FALSE(score.hardValid());
}

} // namespace drone_city_nav
