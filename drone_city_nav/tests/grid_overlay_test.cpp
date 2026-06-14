#include "drone_city_nav/grid_overlay.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] OccupancyGrid2D makeOverlayGrid() {
  return OccupancyGrid2D{GridBounds{0.0, 0.0, 1.0, 6, 6}};
}

} // namespace

TEST(GridOverlay, StaticOccupiedWinsOverMemoryFree) {
  OccupancyGrid2D destination = makeOverlayGrid();
  OccupancyGrid2D memory = makeOverlayGrid();
  destination.setOccupied(GridIndex{2, 2});
  memory.setFree(GridIndex{2, 2});

  const GridOverlayStats stats = overlayKnownMemoryCells(destination, memory);

  EXPECT_TRUE(destination.isOccupied(GridIndex{2, 2}));
  EXPECT_EQ(stats.source_free_cells, 1U);
  EXPECT_EQ(stats.free_cells_applied, 0U);
  EXPECT_EQ(stats.occupied_cells_preserved, 1U);
}

TEST(GridOverlay, MemoryOccupiedBlocksWithoutStaticMap) {
  OccupancyGrid2D destination = makeOverlayGrid();
  OccupancyGrid2D memory = makeOverlayGrid();
  memory.setOccupied(GridIndex{3, 2});

  const GridOverlayStats stats = overlayKnownMemoryCells(destination, memory);

  EXPECT_TRUE(destination.isOccupied(GridIndex{3, 2}));
  EXPECT_EQ(stats.source_occupied_cells, 1U);
  EXPECT_EQ(stats.occupied_cells_applied, 1U);
}

TEST(GridOverlay, CurrentLidarOccupiedBlocksWithoutMemory) {
  OccupancyGrid2D destination = makeOverlayGrid();
  OccupancyGrid2D current_lidar = makeOverlayGrid();
  current_lidar.setOccupied(GridIndex{4, 2});

  const GridOverlayStats stats = overlayCurrentLidarCells(destination, current_lidar);

  EXPECT_TRUE(destination.isOccupied(GridIndex{4, 2}));
  EXPECT_EQ(stats.source_occupied_cells, 1U);
  EXPECT_EQ(stats.occupied_cells_applied, 1U);
}

TEST(GridOverlay, EmptySourceDoesNotChangeDestination) {
  OccupancyGrid2D destination = makeOverlayGrid();
  OccupancyGrid2D empty_source = makeOverlayGrid();

  const GridOverlayStats stats = overlayOccupiedCells(destination, empty_source);

  EXPECT_EQ(destination.state(GridIndex{1, 1}), CellState::kUnknown);
  EXPECT_EQ(stats.source_occupied_cells, 0U);
  EXPECT_EQ(stats.occupied_cells_applied, 0U);
}

TEST(GridOverlay, InflationRunsAfterUnionOverlay) {
  OccupancyGrid2D destination = makeOverlayGrid();
  OccupancyGrid2D static_source = makeOverlayGrid();
  static_source.setOccupied(GridIndex{2, 2});

  const GridOverlayStats overlay_stats =
      overlayOccupiedCells(destination, static_source);
  destination.rebuildInflation(1.1);

  EXPECT_EQ(overlay_stats.occupied_cells_applied, 1U);
  EXPECT_TRUE(destination.isBlocked(GridIndex{2, 2}));
  EXPECT_TRUE(destination.isBlocked(GridIndex{3, 2}));
}

TEST(GridOverlay, RejectsMismatchedGeometry) {
  OccupancyGrid2D destination = makeOverlayGrid();
  OccupancyGrid2D other{GridBounds{0.0, 0.0, 0.5, 6, 6}};

  EXPECT_FALSE(haveSameGridGeometry(destination, other));
  EXPECT_THROW(
      {
        const GridOverlayStats stats = overlayOccupiedCells(destination, other);
        (void)stats;
      },
      std::invalid_argument);
}

} // namespace drone_city_nav
