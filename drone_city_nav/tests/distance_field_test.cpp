#include "drone_city_nav/bounded_worker_pool.hpp"
#include "drone_city_nav/distance_field.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D makeGrid() {
  return OccupancyGrid2D{GridBounds{0.0, 0.0, 1.0, 8, 8}};
}

} // namespace

TEST(DistanceField2D, MeasuresExactMetricDistanceFromOccupiedCells) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{2, 2});

  const DistanceField2D field =
      DistanceField2D::build(grid, 0.0, DistanceFieldSource::kOccupied);

  EXPECT_EQ(field.source(), DistanceFieldSource::kOccupied);
  EXPECT_EQ(field.stats().source_cells, 1U);
  EXPECT_GE(field.stats().x_pass_ms, 0.0);
  EXPECT_GE(field.stats().y_pass_ms, 0.0);
  EXPECT_GE(field.stats().finalize_ms, 0.0);
  EXPECT_GE(field.stats().duration_ms,
            field.stats().x_pass_ms + field.stats().y_pass_ms);
  EXPECT_DOUBLE_EQ(field.distanceAt(GridIndex{2, 2}), 0.0);
  EXPECT_DOUBLE_EQ(field.distanceAt(GridIndex{3, 2}), 1.0);
  EXPECT_NEAR(field.distanceAt(GridIndex{3, 3}), std::numbers::sqrt2, 1.0e-9);
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

TEST(DistanceField2D, UsesOnlyRawOccupiedSources) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{2, 2});
  grid.setOccupied(GridIndex{6, 4});

  const DistanceField2D field =
      DistanceField2D::build(grid, 8.0, DistanceFieldSource::kOccupied);

  EXPECT_DOUBLE_EQ(field.distanceAt(GridIndex{2, 2}), 0.0);
  EXPECT_DOUBLE_EQ(field.distanceAt(GridIndex{6, 4}), 0.0);
  EXPECT_DOUBLE_EQ(field.distanceAt(GridIndex{3, 2}), 1.0);
}

TEST(DistanceField2D, ParallelBuildMatchesSerialBuildExactly) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{1, 1});
  grid.setOccupied(GridIndex{6, 5});
  const DistanceField2D serial =
      DistanceField2D::build(grid, 8.0, DistanceFieldSource::kOccupied);
  BoundedWorkerPool worker_pool{4U};
  const DistanceField2D parallel =
      DistanceField2D::build(grid, 8.0, DistanceFieldSource::kOccupied, &worker_pool);

  ASSERT_EQ(serial.distancesM().size(), parallel.distancesM().size());
  for (std::size_t index = 0U; index < serial.distancesM().size(); ++index) {
    EXPECT_EQ(serial.distancesM()[index], parallel.distancesM()[index]);
  }
  EXPECT_EQ(serial.stats().source_cells, parallel.stats().source_cells);
}

} // namespace drone_city_nav
