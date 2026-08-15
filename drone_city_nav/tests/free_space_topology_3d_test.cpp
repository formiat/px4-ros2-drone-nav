#include "drone_city_nav/free_space_topology_3d.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
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
  ASSERT_EQ(topology.regions().size(), 1U);
  ASSERT_EQ(topology.portals().size(), 5U);
  ASSERT_EQ(topology.segments().size(), 6U);
  EXPECT_TRUE(topology.traversalEdges().empty());

  for (const PassagePortal& portal : topology.portals()) {
    EXPECT_GE(portal.opening_polygon.size(), 3U);
    EXPECT_FALSE(portal.surface_voxels.empty());
    EXPECT_FALSE(portal.traversable_anchors.empty());
    EXPECT_NEAR(std::sqrt(portal.outward_normal.x * portal.outward_normal.x +
                          portal.outward_normal.y * portal.outward_normal.y +
                          portal.outward_normal.z * portal.outward_normal.z),
                1.0, 1.0e-6);
  }
  std::map<PassagePortalId, std::size_t> segment_count_by_portal;
  for (const PassageSegment& segment : topology.segments()) {
    EXPECT_EQ(segment.id.value().find("passage_structure_"), std::string::npos);
    EXPECT_GT(segment.centerline.size(), 1U);
    EXPECT_FALSE(segment.endpoint_portal_ids.empty());
    EXPECT_GT(segment.minimum_clearance_m, 0.0);
    EXPECT_DOUBLE_EQ(segment.speed_limit_mps, 10.0);
    for (const PassagePortalId& portal_id : segment.endpoint_portal_ids) {
      ++segment_count_by_portal[portal_id];
    }
    const SweptFootprintConfig footprint{};
    for (std::size_t index = 1U; index < segment.centerline.size(); ++index) {
      EXPECT_TRUE(validateRawSweptFootprint(
                      occupancy, segment.centerline[index - 1U].position,
                      FootprintBodyAxis{}, segment.centerline[index].position,
                      FootprintBodyAxis{}, footprint)
                      .accepted())
          << segment.id << " sample " << index;
    }
  }
  EXPECT_TRUE(std::ranges::any_of(segment_count_by_portal,
                                  [](const auto& entry) { return entry.second > 1U; }));
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
  ASSERT_EQ(loaded.segments().size(), original.segments().size());
  ASSERT_EQ(loaded.traversalEdges().size(), original.traversalEdges().size());
  for (std::size_t index = 0U; index < original.regions().size(); ++index) {
    EXPECT_EQ(loaded.regions()[index].id, original.regions()[index].id);
    EXPECT_EQ(loaded.regions()[index].portal_ids, original.regions()[index].portal_ids);
  }
  for (std::size_t index = 0U; index < original.segments().size(); ++index) {
    const PassageSegment& original_segment = original.segments()[index];
    const PassageSegment& loaded_segment = loaded.segments()[index];
    EXPECT_EQ(loaded_segment.id, original_segment.id);
    EXPECT_EQ(loaded_segment.endpoint_portal_ids, original_segment.endpoint_portal_ids);
    EXPECT_EQ(loaded_segment.first_endpoint_neighbors,
              original_segment.first_endpoint_neighbors);
    EXPECT_EQ(loaded_segment.second_endpoint_neighbors,
              original_segment.second_endpoint_neighbors);
    ASSERT_EQ(loaded_segment.centerline.size(), original_segment.centerline.size());
    EXPECT_NEAR(distance3D(loaded_segment.centerline.front().position,
                           original_segment.centerline.front().position),
                0.0, 1.0e-5);
    EXPECT_NEAR(distance3D(loaded_segment.centerline.back().position,
                           original_segment.centerline.back().position),
                0.0, 1.0e-5);
    EXPECT_DOUBLE_EQ(loaded_segment.minimum_clearance_m,
                     original_segment.minimum_clearance_m);
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

TEST(FreeSpaceTopology3D, AllowsPortalNodeSharedBySparseSegments) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 10, 10, 10};
  const FreeSpaceRegionId region_id{"region:shared-portal"};
  const auto portal = [&region_id](const char* id, const Point3 center,
                                   const GridIndex3D voxel) {
    return PassagePortal{
        .id = PassagePortalId{id},
        .region_id = region_id,
        .center = center,
        .outward_normal = {1.0, 0.0, 0.0},
        .opening_polygon = {{center.x, center.y - 0.5, center.z - 0.5},
                            {center.x, center.y + 0.5, center.z - 0.5},
                            {center.x, center.y, center.z + 0.5}},
        .surface_voxels = {voxel},
        .traversable_anchors = {center},
        .local_u_axis = {0.0, 1.0, 0.0},
        .local_v_axis = {0.0, 0.0, 1.0},
        .minimum_clearance_m = 1.0,
        .mean_clearance_m = 1.5,
        .maximum_clearance_m = 2.0,
    };
  };
  const PassagePortal shared = portal("portal:shared", {5.5, 5.5, 5.5}, {5, 5, 5});
  const PassagePortal first = portal("portal:first", {2.5, 5.5, 5.5}, {2, 5, 5});
  const PassagePortal second = portal("portal:second", {5.5, 8.5, 5.5}, {5, 8, 5});
  const PassageSegmentId first_segment_id{"segment:first"};
  const PassageSegmentId second_segment_id{"segment:second"};
  const PassageSegment first_segment{
      .id = first_segment_id,
      .centerline = {{.position = shared.center}, {.position = first.center}},
      .endpoint_portal_ids = {shared.id, first.id},
      .first_endpoint_neighbors = {second_segment_id},
      .second_endpoint_neighbors = {},
      .minimum_clearance_m = 1.0,
      .speed_limit_mps = 5.0,
  };
  const PassageSegment second_segment{
      .id = second_segment_id,
      .centerline = {{.position = shared.center}, {.position = second.center}},
      .endpoint_portal_ids = {shared.id, second.id},
      .first_endpoint_neighbors = {first_segment_id},
      .second_endpoint_neighbors = {},
      .minimum_clearance_m = 1.0,
      .speed_limit_mps = 5.0,
  };

  EXPECT_NO_THROW(static_cast<void>(FreeSpaceTopology3D{
      42U,
      bounds,
      {FreeSpaceRegion{.id = region_id,
                       .representative = {5.5, 5.5, 5.5},
                       .maximum_clearance_m = 2.0,
                       .portal_ids = {shared.id, first.id, second.id}}},
      {shared, first, second},
      {first_segment, second_segment},
  }));
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
