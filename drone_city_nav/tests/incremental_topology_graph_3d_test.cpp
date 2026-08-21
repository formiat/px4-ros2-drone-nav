#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <unordered_map>

namespace drone_city_nav {
namespace {

[[nodiscard]] IncrementalTopologyGraph3DConfig makeConfig() {
  IncrementalTopologyGraph3DConfig config;
  config.tile_size_cells = 4;
  config.sample_stride_cells = 1;
  config.maximum_frontier_evaluations_per_component = 128U;
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

[[nodiscard]] bool
hasNodeWithMinimumDegree(const IncrementalTopologyGraph3DSnapshot& snapshot,
                         const std::size_t minimum_degree) {
  return std::ranges::any_of(snapshot.nodes(), [minimum_degree](const auto& node) {
    return node.degree >= minimum_degree;
  });
}

TEST(IncrementalTopologyGraph3DTest, ExtractsTJunctionAndFrontierNodes) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 32, 24}};
  fillFreeBox(occupancy, 2, 44, 14, 16, 10, 12);
  fillFreeBox(occupancy, 22, 24, 2, 16, 10, 12);
  IncrementalTopologyGraph3D graph{makeConfig()};

  const IncrementalTopologyGraph3DUpdate update = graph.update(occupancy, 1U, {}, true);
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();

  EXPECT_TRUE(update.full_reset);
  EXPECT_GT(update.rebuilt_tiles, 0U);
  EXPECT_TRUE(hasNodeWithMinimumDegree(snapshot, 3U));
  EXPECT_TRUE(std::ranges::any_of(
      snapshot.nodes(), [](const auto& node) { return node.traits.junction; }));
  EXPECT_TRUE(std::ranges::any_of(snapshot.nodes(), [](const auto& node) {
    return node.traits.frontier && node.observation_frontier.has_value();
  }));
}

TEST(IncrementalTopologyGraph3DTest, ExtractsXJunctionAndLoop) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 48, 16}};
  fillFreeBox(occupancy, 2, 44, 22, 24, 6, 8);
  fillFreeBox(occupancy, 22, 24, 2, 44, 6, 8);
  fillFreeBox(occupancy, 6, 40, 6, 8, 6, 8);
  fillFreeBox(occupancy, 6, 8, 6, 40, 6, 8);
  fillFreeBox(occupancy, 40, 42, 6, 40, 6, 8);
  fillFreeBox(occupancy, 6, 42, 40, 42, 6, 8);
  IncrementalTopologyGraph3D graph{makeConfig()};

  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();

  EXPECT_TRUE(hasNodeWithMinimumDegree(snapshot, 4U));
  EXPECT_GE(snapshot.edges().size(), snapshot.nodes().size());
}

TEST(IncrementalTopologyGraph3DTest, ClassifiesVerticalConnector) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 24, 32}};
  fillFreeBox(occupancy, 2, 22, 10, 12, 3, 5);
  fillFreeBox(occupancy, 20, 22, 10, 12, 3, 24);
  fillFreeBox(occupancy, 20, 44, 10, 12, 22, 24);
  IncrementalTopologyGraph3D graph{makeConfig()};

  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();

  EXPECT_TRUE(std::ranges::any_of(snapshot.nodes(), [](const auto& node) {
    return node.traits.vertical_connector;
  }));
  const std::optional<IncrementalTopologyNodeId> lower =
      snapshot.nearestNode({18.5, 11.5, 4.5}, 8.0);
  const std::optional<IncrementalTopologyNodeId> upper =
      snapshot.nearestNode({26.5, 11.5, 23.5}, 8.0);
  EXPECT_TRUE(lower.has_value());
  EXPECT_TRUE(upper.has_value());
  EXPECT_NE(lower, upper);
}

TEST(IncrementalTopologyGraph3DTest, ClassifiesClosedCulDeSacTerminalsAndCorridorTurn) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 40, 40, 16}};
  fillStateBox(occupancy, 0, 39, 0, 39, 0, 15, ObservedVoxelState::kOccupied);
  fillFreeBox(occupancy, 3, 20, 8, 10, 5, 7);
  fillFreeBox(occupancy, 18, 20, 8, 32, 5, 7);
  IncrementalTopologyGraph3D graph{makeConfig()};

  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();

  EXPECT_TRUE(std::ranges::any_of(
      snapshot.nodes(), [](const auto& node) { return node.traits.terminal; }));
  EXPECT_TRUE(std::ranges::any_of(snapshot.nodes(),
                                  [](const auto& node) { return node.traits.turn; }));
  EXPECT_TRUE(std::ranges::none_of(snapshot.nodes(), [](const auto& node) {
    return node.traits.frontier || node.observation_frontier.has_value();
  }));
}

TEST(IncrementalTopologyGraph3DTest, DirtyUpdateRetainsUnaffectedNodeIdentity) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 64, 32, 16}};
  fillFreeBox(occupancy, 2, 61, 14, 16, 6, 8);
  IncrementalTopologyGraph3D graph{makeConfig()};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot before = graph.snapshot();
  const std::optional<IncrementalTopologyNodeId> distant_before =
      before.nearestNode({56.5, 15.5, 7.5}, 8.0);
  ASSERT_TRUE(distant_before.has_value());
  const IncrementalTopologyNodeId distant_id =
      distant_before.value_or(IncrementalTopologyNodeId{});

  fillFreeBox(occupancy, 8, 10, 4, 16, 6, 8);
  const std::array dirty_chunks{ObservedOccupancyGrid3D::chunkIndex({8, 8, 7})};
  const IncrementalTopologyGraph3DUpdate update =
      graph.update(occupancy, 2U, dirty_chunks, false);
  const IncrementalTopologyGraph3DSnapshot after = graph.snapshot();
  const std::optional<IncrementalTopologyNodeId> distant_after =
      after.nearestNode({56.5, 15.5, 7.5}, 8.0);

  EXPECT_FALSE(update.full_reset);
  EXPECT_EQ(update.requested_dirty_chunks, 1U);
  EXPECT_GT(update.retained_node_ids, 0U);
  EXPECT_EQ(distant_after, distant_before);
  const IncrementalTopologyNode3D* distant_node = after.findNode(distant_id);
  ASSERT_NE(distant_node, nullptr);
  EXPECT_EQ(distant_node->geometry_revision, 1U);
}

TEST(IncrementalTopologyGraph3DTest, StaticMapSeedsCompleteGraphWithoutFrontiers) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 20, 12}};
  IncrementalTopologyGraph3D graph{makeConfig()};

  const IncrementalTopologyGraph3DUpdate update = graph.reset(occupancy, 7U);
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();

  EXPECT_TRUE(update.full_reset);
  EXPECT_GT(snapshot.nodes().size(), 1U);
  EXPECT_GT(snapshot.edges().size(), 0U);
  EXPECT_TRUE(std::ranges::none_of(snapshot.nodes(), [](const auto& node) {
    return node.traits.frontier || node.observation_frontier.has_value();
  }));
}

} // namespace
} // namespace drone_city_nav
