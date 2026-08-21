#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <unordered_map>
#include <unordered_set>

namespace drone_city_nav {
namespace {

[[nodiscard]] IncrementalTopologyGraph3DConfig makeConfig() {
  IncrementalTopologyGraph3DConfig config;
  config.tile_size_cells = 4;
  config.coarse_sample_stride_cells = 1;
  config.refined_sample_stride_cells = 1;
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

[[nodiscard]] bool nodesConnected(const IncrementalTopologyGraph3DSnapshot& snapshot,
                                  const IncrementalTopologyNodeId start,
                                  const IncrementalTopologyNodeId goal) {
  std::unordered_map<IncrementalTopologyNodeId, std::vector<IncrementalTopologyNodeId>,
                     IncrementalTopologyNodeIdHash>
      adjacency;
  for (const IncrementalTopologyEdge3D& edge : snapshot.edges()) {
    adjacency[edge.first].push_back(edge.second);
    adjacency[edge.second].push_back(edge.first);
  }
  std::deque<IncrementalTopologyNodeId> pending{start};
  std::unordered_set<IncrementalTopologyNodeId, IncrementalTopologyNodeIdHash> visited{
      start};
  while (!pending.empty()) {
    const IncrementalTopologyNodeId current = pending.front();
    pending.pop_front();
    if (current == goal) {
      return true;
    }
    for (const IncrementalTopologyNodeId neighbor : adjacency[current]) {
      if (visited.insert(neighbor).second) {
        pending.push_back(neighbor);
      }
    }
  }
  return false;
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

TEST(IncrementalTopologyGraph3DTest,
     AdaptivelyRefinesAFeasibleCorridorMissedByCoarseSampling) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 8, 8}};
  fillFreeBox(occupancy, 1, 18, 1, 1, 1, 1);
  IncrementalTopologyGraph3DConfig adaptive_config = makeConfig();
  adaptive_config.coarse_sample_stride_cells = 2;
  adaptive_config.refined_sample_stride_cells = 1;
  IncrementalTopologyGraph3D adaptive_graph{adaptive_config};

  const IncrementalTopologyGraph3DUpdate adaptive_update =
      adaptive_graph.update(occupancy, 1U, {}, true);
  const IncrementalTopologyGraph3DSnapshot adaptive = adaptive_graph.snapshot();

  EXPECT_GT(adaptive_update.adaptively_refined_tiles, 0U);
  EXPECT_GT(adaptive_update.sampled_navigable_cells, 0U);
  EXPECT_TRUE(adaptive.nearestNode({1.5, 1.5, 1.5}, 2.0).has_value());
  EXPECT_TRUE(adaptive.nearestNode({18.5, 1.5, 1.5}, 2.0).has_value());
  EXPECT_GT(adaptive.edges().size(), 0U);

  IncrementalTopologyGraph3DConfig coarse_only_config = adaptive_config;
  coarse_only_config.refined_sample_stride_cells = 2;
  IncrementalTopologyGraph3D coarse_only_graph{coarse_only_config};
  const IncrementalTopologyGraph3DUpdate coarse_only_update =
      coarse_only_graph.update(occupancy, 1U, {}, true);

  EXPECT_EQ(coarse_only_update.adaptively_refined_tiles, 0U);
  EXPECT_EQ(coarse_only_graph.snapshot().nodes().size(), 0U);
}

TEST(IncrementalTopologyGraph3DTest,
     ConnectsRefinedObstacleBoundaryToCoarseOpenVolume) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 32, 16, 16}};
  occupancy.setOccupied({6, 6, 6});
  IncrementalTopologyGraph3DConfig config = makeConfig();
  config.coarse_sample_stride_cells = 2;
  config.refined_sample_stride_cells = 1;
  IncrementalTopologyGraph3D graph{config};

  const IncrementalTopologyGraph3DUpdate update = graph.reset(occupancy, 1U);
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();
  const std::optional<IncrementalTopologyNodeId> refined =
      snapshot.nearestNode({7.5, 7.5, 7.5}, 5.0);
  const std::optional<IncrementalTopologyNodeId> coarse =
      snapshot.nearestNode({26.5, 8.5, 8.5}, 5.0);

  ASSERT_TRUE(refined.has_value());
  ASSERT_TRUE(coarse.has_value());
  const IncrementalTopologyNodeId refined_id =
      refined.value_or(IncrementalTopologyNodeId{});
  const IncrementalTopologyNodeId coarse_id =
      coarse.value_or(IncrementalTopologyNodeId{});
  EXPECT_GT(update.adaptively_refined_tiles, 0U);
  EXPECT_LT(update.adaptively_refined_tiles, update.rebuilt_tiles);
  EXPECT_TRUE(nodesConnected(snapshot, refined_id, coarse_id));
}

TEST(IncrementalTopologyGraph3DTest,
     RetainsNodeIdentityWhenDirtyTileChangesSamplingResolution) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 16, 16, 8}};
  fillFreeBox(occupancy, 0, 15, 0, 15, 0, 7);
  IncrementalTopologyGraph3DConfig config = makeConfig();
  config.coarse_sample_stride_cells = 2;
  config.refined_sample_stride_cells = 1;
  IncrementalTopologyGraph3D graph{config};
  const IncrementalTopologyGraph3DUpdate initial =
      graph.update(occupancy, 1U, {}, true);
  const std::optional<IncrementalTopologyNodeId> before =
      graph.snapshot().nearestNode({6.5, 6.5, 4.5}, 5.0);
  ASSERT_TRUE(before.has_value());
  EXPECT_EQ(initial.adaptively_refined_tiles, 0U);

  static_cast<void>(occupancy.setState({6, 6, 4}, ObservedVoxelState::kOccupied));
  const OccupancyChunkIndex3D dirty = ObservedOccupancyGrid3D::chunkIndex({6, 6, 4});
  const IncrementalTopologyGraph3DUpdate refined_update =
      graph.update(occupancy, 2U, std::span{&dirty, 1U}, false);
  const IncrementalTopologyGraph3DSnapshot after = graph.snapshot();
  const IncrementalTopologyNodeId before_id =
      before.value_or(IncrementalTopologyNodeId{});
  const IncrementalTopologyNode3D* retained = after.findNode(before_id);

  ASSERT_NE(retained, nullptr);
  EXPECT_GT(refined_update.adaptively_refined_tiles, 0U);
  EXPECT_GT(refined_update.retained_node_ids, 0U);
  EXPECT_EQ(retained->geometry_revision, 2U);
}

} // namespace
} // namespace drone_city_nav
