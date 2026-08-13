#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <ranges>

namespace drone_city_nav {
namespace {

TEST(OccupancyGrid3D, StoresSparseVoxelsAcrossChunks) {
  OccupancyGrid3D grid{GridBounds3D{0.0, 0.0, 0.0, 0.5, 40, 40, 40}, 42U};
  grid.setOccupied({0, 0, 0});
  grid.setOccupied({17, 18, 19});
  grid.setOccupied({17, 18, 19});

  EXPECT_TRUE(grid.isOccupied({0, 0, 0}));
  EXPECT_TRUE(grid.isOccupied({17, 18, 19}));
  EXPECT_FALSE(grid.isOccupied({16, 18, 19}));
  EXPECT_EQ(grid.occupiedVoxelCount(), 2U);
  EXPECT_EQ(grid.occupiedChunkCount(), 2U);
  EXPECT_EQ(grid.fingerprint(), 42U);
}

TEST(OccupancyGrid3D, ConvertsWorldCoordinates) {
  OccupancyGrid3D grid{GridBounds3D{-1.0, -2.0, 3.0, 0.5, 4, 6, 8}};
  const std::optional<GridIndex3D> cell = grid.worldToCell({-0.74, -1.26, 4.26});
  if (!cell.has_value()) {
    FAIL() << "expected in-bounds cell";
  }
  const GridIndex3D index = cell.value_or(GridIndex3D{});
  EXPECT_EQ(index, (GridIndex3D{0, 1, 2}));
  const Point3 center = grid.cellCenter(index);
  EXPECT_DOUBLE_EQ(center.x, -0.75);
  EXPECT_DOUBLE_EQ(center.y, -1.25);
  EXPECT_DOUBLE_EQ(center.z, 4.25);
  EXPECT_FALSE(grid.worldToCell({10.0, 0.0, 0.0}).has_value());
}

TEST(OccupancyGrid3D, StartsWithEmptyDerivedPortalGraph) {
  const OccupancyGrid3D grid{GridBounds3D{0.0, 0.0, 0.0, 0.5, 40, 40, 40}};

  EXPECT_TRUE(grid.portalGraph().regions.empty());
  EXPECT_TRUE(grid.portalGraph().portals.empty());
  EXPECT_TRUE(grid.portalGraph().traversal_edges.empty());
}

TEST(OccupancyGrid3D, LoadsDerivedPortalGraphFromGeneratedArtifact) {
  const OccupancyGrid3D grid = OccupancyGrid3D::load(TEST_OCCUPANCY3D_PATH);
  const DerivedPortalGraph& graph = grid.portalGraph();

  ASSERT_EQ(graph.regions.size(), 3U);
  ASSERT_EQ(graph.portals.size(), 7U);
  ASSERT_EQ(graph.traversal_edges.size(), 5U);
  EXPECT_EQ(std::ranges::count_if(graph.regions,
                                  [](const PassageRegion& region) {
                                    return region.portal_ids.size() == 3U;
                                  }),
            1);
  for (const PassagePortal& portal : graph.portals) {
    EXPECT_EQ(portal.opening_polygon.size(), 4U);
    EXPECT_NEAR(std::sqrt(portal.outward_normal.x * portal.outward_normal.x +
                          portal.outward_normal.y * portal.outward_normal.y +
                          portal.outward_normal.z * portal.outward_normal.z),
                1.0, 1.0e-9);
  }
  for (const ConstrainedFreeSpaceEdge& edge : graph.traversal_edges) {
    EXPECT_EQ(edge.id.find("channel_"), std::string::npos);
    EXPECT_EQ(edge.region_id.find("passage_region_"), 0U);
    EXPECT_GT(edge.centerline.size(), 2U);
    EXPECT_DOUBLE_EQ(edge.min_z_m, 1.5);
    EXPECT_DOUBLE_EQ(edge.max_z_m, 8.5);
    EXPECT_DOUBLE_EQ(edge.width_m, 30.0);
    EXPECT_DOUBLE_EQ(edge.height_m, 7.0);
    EXPECT_DOUBLE_EQ(edge.minimum_clearance_m, 3.5);
    EXPECT_DOUBLE_EQ(edge.speed_limit_mps, 10.0);
    const SweptFootprintConfig footprint{};
    for (std::size_t index = 1U; index < edge.centerline.size(); ++index) {
      EXPECT_TRUE(validateRawSweptFootprint(
                      grid, edge.centerline[index - 1U].position, FootprintBodyAxis{},
                      edge.centerline[index].position, FootprintBodyAxis{}, footprint)
                      .accepted())
          << edge.id << " segment " << index;
    }
  }

  const auto straight = std::ranges::find_if(
      graph.traversal_edges, [](const ConstrainedFreeSpaceEdge& edge) {
        return std::abs(edge.entry.x - 54.0) < 1.0e-6 &&
               std::abs(edge.exit.x - 54.0) < 1.0e-6 &&
               std::abs(edge.entry.y - 123.0) < 1.0e-6 &&
               std::abs(edge.exit.y - 201.0) < 1.0e-6;
      });
  ASSERT_NE(straight, graph.traversal_edges.end());
}

} // namespace
} // namespace drone_city_nav
