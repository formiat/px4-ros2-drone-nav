#include "drone_city_nav/contracted_topology_graph_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <span>

namespace drone_city_nav {
namespace {

[[nodiscard]] IncrementalTopologyGraph3DConfig makeConfig() {
  IncrementalTopologyGraph3DConfig config;
  config.tile_size_cells = 4;
  config.coarse_sample_stride_cells = 1;
  config.refined_sample_stride_cells = 1;
  config.footprint.radius_m = 0.2;
  config.footprint.lower_extent_m = 0.2;
  config.footprint.upper_extent_m = 0.2;
  config.footprint.perimeter_samples = 8U;
  config.footprint.radial_rings = 1U;
  config.footprint.axial_samples = 2U;
  config.footprint.sweep_step_m = 0.25;
  config.observability.maximum_observation_range_m = 4.0;
  config.observability.minimum_known_free_ray_m = 1.0;
  config.observability.minimum_supporting_rays = 1U;
  config.observability.minimum_information_gain_voxels = 1U;
  return config;
}

void fillStateBox(ObservedOccupancyGrid3D& occupancy, const int minimum_x,
                  const int maximum_x, const int minimum_y, const int maximum_y,
                  const int minimum_z, const int maximum_z,
                  const ObservedVoxelState state) {
  for (int z = minimum_z; z <= maximum_z; ++z) {
    for (int y = minimum_y; y <= maximum_y; ++y) {
      for (int x = minimum_x; x <= maximum_x; ++x) {
        static_cast<void>(occupancy.setState({x, y, z}, state));
        ASSERT_EQ(occupancy.state({x, y, z}), state);
      }
    }
  }
}

void fillFreeBox(ObservedOccupancyGrid3D& occupancy, const int minimum_x,
                 const int maximum_x, const int minimum_y, const int maximum_y,
                 const int minimum_z, const int maximum_z) {
  fillStateBox(occupancy, minimum_x, maximum_x, minimum_y, maximum_y, minimum_z,
               maximum_z, ObservedVoxelState::kFree);
}

void fillOccupied(ObservedOccupancyGrid3D& occupancy) {
  const GridBounds3D& bounds = occupancy.bounds();
  fillStateBox(occupancy, 0, bounds.width_cells - 1, 0, bounds.height_cells - 1, 0,
               bounds.depth_cells - 1, ObservedVoxelState::kOccupied);
}

TEST(ContractedTopologyGraph3DTest, ContractsLongDegreeTwoCorridorIntoOneEdge) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 24, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 45, 9, 11, 5, 7);
  IncrementalTopologyGraph3D source{makeConfig()};
  static_cast<void>(source.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot source_snapshot = source.snapshot();

  const ContractedTopologyGraph3D contracted =
      contractIncrementalTopologyGraph3D(source_snapshot);

  ASSERT_GT(source_snapshot.nodes().size(), 2U);
  ASSERT_EQ(contracted.nodes().size(), 2U);
  ASSERT_EQ(contracted.edges().size(), 1U);
  EXPECT_EQ(contracted.edges().front().source_nodes.size(),
            source_snapshot.nodes().size());
  EXPECT_EQ(contracted.edges().front().source_edges.size(),
            source_snapshot.edges().size());
  EXPECT_GT(contracted.edges().front().length_m, 35.0);
  EXPECT_GT(contracted.edges().front().polyline.size(), 2U);
}

TEST(ContractedTopologyGraph3DTest, PreservesJunctionAndTurnsAsGraphEvents) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 48, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 45, 21, 23, 5, 7);
  fillFreeBox(occupancy, 21, 23, 2, 45, 5, 7);
  IncrementalTopologyGraph3D source{makeConfig()};
  static_cast<void>(source.update(occupancy, 1U, {}, true));

  const ContractedTopologyGraph3D contracted =
      contractIncrementalTopologyGraph3D(source.snapshot());

  EXPECT_TRUE(std::ranges::any_of(contracted.nodes(), [](const auto& node) {
    return node.traits.junction && node.degree >= 4U;
  }));
  EXPECT_GE(contracted.edges().size(), 4U);
}

TEST(ContractedTopologyGraph3DTest, PreservesVerticalConnectorAsGraphEvent) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 24, 32}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 22, 9, 11, 3, 5);
  fillFreeBox(occupancy, 20, 22, 9, 11, 3, 24);
  fillFreeBox(occupancy, 20, 45, 9, 11, 22, 24);
  IncrementalTopologyGraph3D source{makeConfig()};
  static_cast<void>(source.update(occupancy, 1U, {}, true));

  const ContractedTopologyGraph3D contracted =
      contractIncrementalTopologyGraph3D(source.snapshot());

  EXPECT_TRUE(std::ranges::any_of(contracted.nodes(), [](const auto& node) {
    return node.traits.vertical_connector;
  }));
}

TEST(ContractedTopologyGraph3DTest, KeepsStableIdentityForUnchangedCorridor) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 64, 32, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 61, 21, 23, 5, 7);
  IncrementalTopologyGraph3D source{makeConfig()};
  static_cast<void>(source.update(occupancy, 1U, {}, true));
  const ContractedTopologyGraph3D before =
      contractIncrementalTopologyGraph3D(source.snapshot());
  ASSERT_EQ(before.edges().size(), 1U);
  const ContractedTopologyEdgeId3D stable_id = before.edges().front().id;

  fillFreeBox(occupancy, 3, 7, 3, 5, 5, 7);
  const OccupancyChunkIndex3D dirty = ObservedOccupancyGrid3D::chunkIndex({4, 4, 6});
  static_cast<void>(source.update(occupancy, 2U, std::span{&dirty, 1U}, false));
  const ContractedTopologyGraph3D after =
      contractIncrementalTopologyGraph3D(source.snapshot());

  EXPECT_NE(after.findEdge(stable_id), nullptr);
}

} // namespace
} // namespace drone_city_nav
