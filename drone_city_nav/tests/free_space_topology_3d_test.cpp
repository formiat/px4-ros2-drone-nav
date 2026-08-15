#include "drone_city_nav/free_space_topology_3d.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <ranges>

namespace drone_city_nav {
namespace {

TEST(FreeSpaceTopology3D, LoadsFingerprintBoundGeneratedArtifact) {
  const OccupancyGrid3D occupancy = OccupancyGrid3D::load(TEST_OCCUPANCY3D_PATH);
  const FreeSpaceTopology3D topology =
      FreeSpaceTopology3D::load(TEST_FREE_SPACE_TOPOLOGY3D_PATH);

  EXPECT_TRUE(topology.compatibleWith(occupancy));
  EXPECT_EQ(topology.occupancyFingerprint(), occupancy.fingerprint());
  EXPECT_EQ(topology.occupancyBounds(), occupancy.bounds());
  ASSERT_EQ(topology.regions().size(), 3U);
  ASSERT_EQ(topology.portals().size(), 7U);
  ASSERT_EQ(topology.traversalEdges().size(), 5U);
  EXPECT_EQ(std::ranges::count_if(topology.regions(),
                                  [](const FreeSpaceRegion& region) {
                                    return region.portal_ids.size() == 3U;
                                  }),
            1);

  for (const PassagePortal& portal : topology.portals()) {
    EXPECT_EQ(portal.opening_polygon.size(), 4U);
    EXPECT_NEAR(std::sqrt(portal.outward_normal.x * portal.outward_normal.x +
                          portal.outward_normal.y * portal.outward_normal.y +
                          portal.outward_normal.z * portal.outward_normal.z),
                1.0, 1.0e-9);
  }
  for (const PassageTraversalEdge& edge : topology.traversalEdges()) {
    EXPECT_EQ(edge.id.value().find("passage_structure_"), std::string::npos);
    EXPECT_EQ(edge.region_id.value().find("passage_region_"), 0U);
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
                      occupancy, edge.centerline[index - 1U].position,
                      FootprintBodyAxis{}, edge.centerline[index].position,
                      FootprintBodyAxis{}, footprint)
                      .accepted())
          << edge.id << " segment " << index;
    }
  }

  const auto straight = std::ranges::find_if(
      topology.traversalEdges(), [](const PassageTraversalEdge& edge) {
        return std::abs(edge.entry.x - 54.0) < 1.0e-6 &&
               std::abs(edge.exit.x - 54.0) < 1.0e-6 &&
               std::abs(edge.entry.y - 123.0) < 1.0e-6 &&
               std::abs(edge.exit.y - 201.0) < 1.0e-6;
      });
  ASSERT_NE(straight, topology.traversalEdges().end());
}

TEST(FreeSpaceTopology3D, PreservesTopologyWhenRewritten) {
  const FreeSpaceTopology3D original =
      FreeSpaceTopology3D::load(TEST_FREE_SPACE_TOPOLOGY3D_PATH);
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      "drone_city_nav_free_space_topology_3d_round_trip.topology3d";
  std::filesystem::remove(path);

  original.write(path);
  const FreeSpaceTopology3D loaded = FreeSpaceTopology3D::load(path);
  std::filesystem::remove(path);

  EXPECT_EQ(loaded.occupancyFingerprint(), original.occupancyFingerprint());
  EXPECT_EQ(loaded.occupancyBounds(), original.occupancyBounds());
  ASSERT_EQ(loaded.regions().size(), original.regions().size());
  ASSERT_EQ(loaded.portals().size(), original.portals().size());
  ASSERT_EQ(loaded.traversalEdges().size(), original.traversalEdges().size());
  for (std::size_t index = 0U; index < original.regions().size(); ++index) {
    EXPECT_EQ(loaded.regions()[index].id, original.regions()[index].id);
    EXPECT_EQ(loaded.regions()[index].portal_ids, original.regions()[index].portal_ids);
  }
  for (std::size_t index = 0U; index < original.traversalEdges().size(); ++index) {
    const PassageTraversalEdge& original_edge = original.traversalEdges()[index];
    const PassageTraversalEdge& loaded_edge = loaded.traversalEdges()[index];
    EXPECT_EQ(loaded_edge.id, original_edge.id);
    EXPECT_EQ(loaded_edge.region_id, original_edge.region_id);
    EXPECT_EQ(loaded_edge.entry_portal_id, original_edge.entry_portal_id);
    EXPECT_EQ(loaded_edge.exit_portal_id, original_edge.exit_portal_id);
    EXPECT_NEAR(distance3D(loaded_edge.entry, original_edge.entry), 0.0, 1.0e-5);
    EXPECT_NEAR(distance3D(loaded_edge.exit, original_edge.exit), 0.0, 1.0e-5);
    EXPECT_DOUBLE_EQ(loaded_edge.minimum_clearance_m,
                     original_edge.minimum_clearance_m);
  }
}

TEST(FreeSpaceTopology3D, RejectsTruncatedArtifact) {
  const FreeSpaceTopology3D topology =
      FreeSpaceTopology3D::load(TEST_FREE_SPACE_TOPOLOGY3D_PATH);
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      "drone_city_nav_truncated_free_space_topology.topology3d";
  std::filesystem::remove(path);
  topology.write(path);
  ASSERT_GT(std::filesystem::file_size(path), 1U);
  std::filesystem::resize_file(path, std::filesystem::file_size(path) - 1U);

  EXPECT_THROW(static_cast<void>(FreeSpaceTopology3D::load(path)), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(FreeSpaceTopology3D, RejectsDifferentOccupancyFingerprint) {
  const FreeSpaceTopology3D topology =
      FreeSpaceTopology3D::load(TEST_FREE_SPACE_TOPOLOGY3D_PATH);
  const OccupancyGrid3D different_occupancy{topology.occupancyBounds(),
                                            topology.occupancyFingerprint() + 1U};

  EXPECT_FALSE(topology.compatibleWith(different_occupancy));
}

TEST(FreeSpaceTopology3D, SupportsEmptyAccelerationIndex) {
  const GridBounds3D bounds{-1.0, -2.0, -3.0, 0.5, 20, 30, 40};
  const FreeSpaceTopology3D topology{42U, bounds, std::vector<FreeSpaceRegion>{},
                                     std::vector<PassagePortal>{},
                                     std::vector<PassageTraversalEdge>{}};

  EXPECT_EQ(topology.occupancyFingerprint(), 42U);
  EXPECT_EQ(topology.occupancyBounds(), bounds);
  EXPECT_TRUE(topology.regions().empty());
  EXPECT_TRUE(topology.portals().empty());
  EXPECT_TRUE(topology.traversalEdges().empty());
}

} // namespace
} // namespace drone_city_nav
