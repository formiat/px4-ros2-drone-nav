#include "drone_city_nav/distance_field.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D makeGrid() {
  return OccupancyGrid2D{GridBounds{0.0, 0.0, 1.0, 8, 8}};
}

[[nodiscard]] std::vector<std::uint8_t>
referenceInflationMask(const OccupancyGrid2D& grid, const double radius_m) {
  std::vector<std::uint8_t> inflated(grid.cellCount(), 0U);
  if (!(radius_m > 0.0)) {
    return inflated;
  }
  const int radius_cells = static_cast<int>(std::ceil(radius_m / grid.resolution()));
  const double radius_with_margin = radius_m + (0.5 * grid.resolution());
  const double radius_sq = radius_with_margin * radius_with_margin;
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const GridIndex obstacle{x, y};
      if (!grid.isOccupied(obstacle)) {
        continue;
      }
      const Point2 obstacle_center = grid.cellCenter(obstacle);
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const GridIndex candidate{x + dx, y + dy};
          if (!grid.contains(candidate)) {
            continue;
          }
          if (squaredDistance(obstacle_center, grid.cellCenter(candidate)) <=
              radius_sq) {
            inflated[grid.linearIndex(candidate)] = 1U;
          }
        }
      }
    }
  }
  return inflated;
}

void expectInflationMatchesReference(const OccupancyGrid2D& source_grid,
                                     const double radius_m) {
  OccupancyGrid2D actual = source_grid;
  actual.rebuildInflation(radius_m);
  const std::vector<std::uint8_t> expected =
      referenceInflationMask(source_grid, radius_m);
  ASSERT_EQ(expected.size(), actual.inflatedCells().size());
  for (std::size_t index = 0U; index < expected.size(); ++index) {
    EXPECT_EQ(actual.inflatedCells()[index], expected[index]) << "index=" << index;
  }
}

} // namespace

TEST(DistanceField2D, MeasuresExactMetricDistanceFromOccupiedCells) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{2, 2});

  const DistanceField2D field =
      DistanceField2D::build(grid, 0.0, DistanceFieldSource::kOccupied);

  EXPECT_EQ(field.source(), DistanceFieldSource::kOccupied);
  EXPECT_EQ(field.stats().source_cells, 1U);
  EXPECT_DOUBLE_EQ(field.distanceAt(GridIndex{2, 2}), 0.0);
  EXPECT_DOUBLE_EQ(field.distanceAt(GridIndex{3, 2}), 1.0);
  EXPECT_NEAR(field.distanceAt(GridIndex{3, 3}), std::numbers::sqrt2, 1.0e-9);
}

TEST(DistanceField2D, CanUseProhibitedCellsAsSources) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{2, 2});
  grid.rebuildInflation(1.1);
  ASSERT_TRUE(grid.isInflated(GridIndex{3, 2}));

  const DistanceField2D occupied =
      DistanceField2D::build(grid, 4.0, DistanceFieldSource::kOccupied);
  const DistanceField2D prohibited =
      DistanceField2D::build(grid, 4.0, DistanceFieldSource::kProhibited);

  EXPECT_DOUBLE_EQ(occupied.distanceAt(GridIndex{3, 2}), 1.0);
  EXPECT_DOUBLE_EQ(prohibited.distanceAt(GridIndex{3, 2}), 0.0);
  EXPECT_DOUBLE_EQ(prohibited.distanceAt(GridIndex{4, 2}), 1.0);
}

TEST(DistanceField2D, IgnoresFreeAndUnknownCellsAsOccupiedSources) {
  OccupancyGrid2D grid = makeGrid();
  grid.setFree(GridIndex{2, 2});

  const DistanceField2D field =
      DistanceField2D::build(grid, 0.0, DistanceFieldSource::kOccupied);

  EXPECT_EQ(field.stats().source_cells, 0U);
  EXPECT_TRUE(std::isinf(field.distanceAt(GridIndex{2, 2})));
  EXPECT_TRUE(std::isinf(field.distanceAt(GridIndex{0, 0})));
}

TEST(DistanceField2D, LeavesCellsOutsideMaxDistanceAtInfinity) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{0, 0});

  const DistanceField2D field =
      DistanceField2D::build(grid, 2.0, DistanceFieldSource::kOccupied);

  EXPECT_DOUBLE_EQ(field.distanceAt(GridIndex{0, 0}), 0.0);
  EXPECT_TRUE(std::isinf(field.distanceAt(GridIndex{4, 4})));
}

TEST(DistanceField2D, InflationMatchesPreviousBrushSemantics) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{2, 2});
  grid.setOccupied(GridIndex{6, 4});

  expectInflationMatchesReference(grid, 0.0);
  expectInflationMatchesReference(grid, 1.1);
  expectInflationMatchesReference(grid, 2.0);
  expectInflationMatchesReference(grid, 4.0);
}

} // namespace drone_city_nav
