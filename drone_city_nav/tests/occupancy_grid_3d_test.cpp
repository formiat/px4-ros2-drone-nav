#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
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

TEST(OccupancyGrid3D, WritesAndReloadsSparseArtifact) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      "drone_city_nav_occupancy_grid_3d_round_trip.occupancy3d";
  std::filesystem::remove(path);
  OccupancyGrid3D original{GridBounds3D{-2.0, 3.0, -4.0, 0.5, 40, 50, 60}, 123456U};
  original.setOccupied({0, 0, 0});
  original.setOccupied({17, 34, 51});

  original.write(path);
  const OccupancyGrid3D loaded = OccupancyGrid3D::load(path);
  std::filesystem::remove(path);

  EXPECT_EQ(loaded.bounds(), original.bounds());
  EXPECT_EQ(loaded.fingerprint(), 123456U);
  EXPECT_EQ(loaded.occupiedVoxelCount(), 2U);
  EXPECT_TRUE(loaded.isOccupied({0, 0, 0}));
  EXPECT_TRUE(loaded.isOccupied({17, 34, 51}));
  EXPECT_TRUE(loaded.portalGraph().regions.empty());
}

TEST(OccupancyGrid3D, PreservesDerivedPortalGraphWhenRewritten) {
  const OccupancyGrid3D original = OccupancyGrid3D::load(TEST_OCCUPANCY3D_PATH);
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      "drone_city_nav_occupancy_grid_3d_portal_round_trip.occupancy3d";
  std::filesystem::remove(path);

  original.write(path);
  const OccupancyGrid3D loaded = OccupancyGrid3D::load(path);
  std::filesystem::remove(path);

  const DerivedPortalGraph& original_graph = original.portalGraph();
  const DerivedPortalGraph& loaded_graph = loaded.portalGraph();
  ASSERT_EQ(loaded_graph.regions.size(), original_graph.regions.size());
  ASSERT_EQ(loaded_graph.portals.size(), original_graph.portals.size());
  ASSERT_EQ(loaded_graph.traversal_edges.size(), original_graph.traversal_edges.size());
  for (std::size_t index = 0U; index < original_graph.regions.size(); ++index) {
    EXPECT_EQ(loaded_graph.regions[index].id, original_graph.regions[index].id);
    EXPECT_EQ(loaded_graph.regions[index].portal_ids,
              original_graph.regions[index].portal_ids);
  }
  for (std::size_t index = 0U; index < original_graph.traversal_edges.size(); ++index) {
    const ConstrainedFreeSpaceEdge& original_edge =
        original_graph.traversal_edges[index];
    const ConstrainedFreeSpaceEdge& loaded_edge = loaded_graph.traversal_edges[index];
    EXPECT_EQ(loaded_edge.id, original_edge.id);
    EXPECT_EQ(loaded_edge.region_id, original_edge.region_id);
    EXPECT_EQ(loaded_edge.entry_portal_id, original_edge.entry_portal_id);
    EXPECT_EQ(loaded_edge.exit_portal_id, original_edge.exit_portal_id);
    EXPECT_NEAR(distance3D(loaded_edge.entry, original_edge.entry), 0.0, 1.0e-5);
    EXPECT_NEAR(distance3D(loaded_edge.exit, original_edge.exit), 0.0, 1.0e-5);
    EXPECT_DOUBLE_EQ(loaded_edge.min_z_m, original_edge.min_z_m);
    EXPECT_DOUBLE_EQ(loaded_edge.max_z_m, original_edge.max_z_m);
    EXPECT_DOUBLE_EQ(loaded_edge.minimum_clearance_m,
                     original_edge.minimum_clearance_m);
  }
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
