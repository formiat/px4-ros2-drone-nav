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

  EXPECT_DOUBLE_EQ(field.distanceAt(GridIndex{2, 2}), 0.0);
  EXPECT_DOUBLE_EQ(field.distanceAt(GridIndex{3, 2}), 1.0);
  EXPECT_NEAR(field.distanceAt(GridIndex{3, 3}), std::numbers::sqrt2, 1.0e-9);
}

TEST(ClearanceField2D, CanUseInflatedCellsAsProhibitedSources) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{2, 2});
  grid.rebuildInflation(1.1);
  ASSERT_TRUE(grid.isInflated(GridIndex{3, 2}));
  ASSERT_FALSE(grid.isOccupied(GridIndex{3, 2}));

  const ClearanceField2D occupied_field =
      ClearanceField2D::build(grid, 4.0, ClearanceSource::kOccupied);
  const ClearanceField2D prohibited_field =
      ClearanceField2D::build(grid, 4.0, ClearanceSource::kProhibited);

  EXPECT_DOUBLE_EQ(occupied_field.distanceAt(GridIndex{3, 2}), 1.0);
  EXPECT_DOUBLE_EQ(prohibited_field.distanceAt(GridIndex{3, 2}), 0.0);
  EXPECT_DOUBLE_EQ(prohibited_field.distanceAt(GridIndex{4, 2}), 1.0);
  EXPECT_DOUBLE_EQ(prohibited_field.distanceAt(GridIndex{5, 2}), 2.0);
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
  grid.rebuildInflation(1.1);
  ClearanceFieldCache cache;

  const ClearanceFieldCacheLookup first =
      cache.getOrBuild(grid, 4.0, ClearanceSource::kProhibited);
  const ClearanceFieldCacheLookup second =
      cache.getOrBuild(grid, 4.0, ClearanceSource::kProhibited);

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
  grid.rebuildInflation(0.0);
  ClearanceFieldCache cache;
  ASSERT_FALSE(cache.getOrBuild(grid, 4.0, ClearanceSource::kProhibited).cache_hit);
  ASSERT_TRUE(cache.getOrBuild(grid, 4.0, ClearanceSource::kProhibited).cache_hit);

  grid.setOccupied(GridIndex{5, 5});

  const ClearanceFieldCacheLookup changed =
      cache.getOrBuild(grid, 4.0, ClearanceSource::kProhibited);

  ASSERT_NE(changed.field, nullptr);
  EXPECT_FALSE(changed.cache_hit);
  EXPECT_DOUBLE_EQ(changed.field->distanceAt(GridIndex{5, 5}), 0.0);
}

TEST(ClearanceFieldCache, InvalidatesWhenInflationMaskChanges) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{2, 2});
  grid.rebuildInflation(0.0);
  ClearanceFieldCache cache;
  ASSERT_FALSE(cache.getOrBuild(grid, 4.0, ClearanceSource::kProhibited).cache_hit);
  ASSERT_TRUE(cache.getOrBuild(grid, 4.0, ClearanceSource::kProhibited).cache_hit);
  ASSERT_FALSE(grid.isInflated(GridIndex{3, 2}));

  grid.rebuildInflation(1.1);

  const ClearanceFieldCacheLookup changed =
      cache.getOrBuild(grid, 4.0, ClearanceSource::kProhibited);

  ASSERT_NE(changed.field, nullptr);
  EXPECT_FALSE(changed.cache_hit);
  ASSERT_TRUE(grid.isInflated(GridIndex{3, 2}));
  EXPECT_DOUBLE_EQ(changed.field->distanceAt(GridIndex{3, 2}), 0.0);
}

TEST(ClearanceFieldCache, InvalidatesWhenRadiusOrSourceChanges) {
  OccupancyGrid2D grid = makeGrid();
  grid.setOccupied(GridIndex{2, 2});
  grid.rebuildInflation(1.1);
  ClearanceFieldCache cache;
  ASSERT_FALSE(cache.getOrBuild(grid, 2.0, ClearanceSource::kProhibited).cache_hit);
  ASSERT_TRUE(cache.getOrBuild(grid, 2.0, ClearanceSource::kProhibited).cache_hit);

  EXPECT_FALSE(cache.getOrBuild(grid, 3.0, ClearanceSource::kProhibited).cache_hit);
  ASSERT_TRUE(cache.getOrBuild(grid, 3.0, ClearanceSource::kProhibited).cache_hit);
  EXPECT_FALSE(cache.getOrBuild(grid, 3.0, ClearanceSource::kOccupied).cache_hit);
}

} // namespace drone_city_nav
