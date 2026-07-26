#include "drone_city_nav/clearance_field.hpp"

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

TEST(ClearanceField2D, MeasuresMetricDistanceFromOccupiedCells) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{2, 2});

  const ClearanceField2D field =
      ClearanceField2D::build(grid, 4.0, ClearanceSource::kOccupied);

  EXPECT_EQ(field.source(), ClearanceSource::kOccupied);
  EXPECT_DOUBLE_EQ(field.distanceAt(GridIndex{2, 2}), 0.0);
  EXPECT_DOUBLE_EQ(field.distanceAt(GridIndex{3, 2}), 1.0);
  EXPECT_NEAR(field.distanceAt(GridIndex{3, 3}), std::numbers::sqrt2, 1.0e-9);
}

TEST(ClearanceField2D, IgnoresFreeAndUnknownCellsAsSources) {
  OccupancyGrid2D grid = makeGrid();
  grid.setFree(GridIndex{2, 2});

  const ClearanceField2D field =
      ClearanceField2D::build(grid, 2.0, ClearanceSource::kOccupied);

  EXPECT_TRUE(std::isinf(field.distanceAt(GridIndex{2, 2})));
  EXPECT_TRUE(std::isinf(field.distanceAt(GridIndex{0, 0})));
}

TEST(ClearanceField2D, LeavesCellsOutsideSearchRadiusAtInfinity) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{0, 0});

  const ClearanceField2D field =
      ClearanceField2D::build(grid, 2.0, ClearanceSource::kOccupied);

  EXPECT_DOUBLE_EQ(field.distanceAt(GridIndex{0, 0}), 0.0);
  EXPECT_TRUE(std::isinf(field.distanceAt(GridIndex{4, 4})));
}

TEST(ClearanceFieldCache, ReusesIdenticalGridRadiusAndSource) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{2, 2});
  ClearanceFieldCache cache;

  const ClearanceFieldCacheLookup first =
      cache.getOrBuild(grid, 4.0, ClearanceSource::kOccupied);
  const ClearanceFieldCacheLookup second =
      cache.getOrBuild(grid, 4.0, ClearanceSource::kOccupied);

  ASSERT_NE(first.field, nullptr);
  ASSERT_NE(second.field, nullptr);
  EXPECT_FALSE(first.cache_hit);
  EXPECT_TRUE(second.cache_hit);
  EXPECT_DOUBLE_EQ(first.field->distanceAt(GridIndex{4, 2}),
                   second.field->distanceAt(GridIndex{4, 2}));
}

TEST(ClearanceFieldCache, InvalidatesWhenOccupiedCellsChange) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{2, 2});
  ClearanceFieldCache cache;
  ASSERT_FALSE(cache.getOrBuild(grid, 4.0, ClearanceSource::kOccupied).cache_hit);
  ASSERT_TRUE(cache.getOrBuild(grid, 4.0, ClearanceSource::kOccupied).cache_hit);

  grid.setOccupied(GridIndex{5, 5});

  const ClearanceFieldCacheLookup changed =
      cache.getOrBuild(grid, 4.0, ClearanceSource::kOccupied);

  ASSERT_NE(changed.field, nullptr);
  EXPECT_FALSE(changed.cache_hit);
  EXPECT_DOUBLE_EQ(changed.field->distanceAt(GridIndex{5, 5}), 0.0);
}

TEST(ClearanceFieldCache, InvalidatesWhenRadiusChanges) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{2, 2});
  ClearanceFieldCache cache;
  ASSERT_FALSE(cache.getOrBuild(grid, 2.0, ClearanceSource::kOccupied).cache_hit);
  ASSERT_TRUE(cache.getOrBuild(grid, 2.0, ClearanceSource::kOccupied).cache_hit);

  EXPECT_FALSE(cache.getOrBuild(grid, 3.0, ClearanceSource::kOccupied).cache_hit);
  ASSERT_TRUE(cache.getOrBuild(grid, 3.0, ClearanceSource::kOccupied).cache_hit);
}

} // namespace drone_city_nav
