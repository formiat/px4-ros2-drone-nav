#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <unordered_set>

namespace drone_city_nav {
namespace {

[[nodiscard]] IncrementalTopologyGraph3DConfig makeConfig() {
  IncrementalTopologyGraph3DConfig config;
  config.block_size_cells = 4;
  config.coarse_sample_stride_cells = 1;
  config.refined_sample_stride_cells = 1;
  config.maximum_observed_blocks_per_update = 64U;
  config.footprint.radius_m = 0.2;
  config.footprint.lower_extent_m = 0.2;
  config.footprint.upper_extent_m = 0.2;
  config.footprint.perimeter_samples = 8U;
  config.footprint.radial_rings = 1U;
  config.footprint.axial_samples = 2U;
  config.footprint.sweep_step_m = 0.25;
  return config;
}

[[nodiscard]] IncrementalTopologyGraph3DConfig makeKnownSpaceConfig() {
  IncrementalTopologyGraph3DConfig config = makeConfig();
  config.require_known_free_space = true;
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

TEST(IncrementalTopologyGraph3DTest,
     BuildsRegionPortalConnectivityWithoutPersistentFrontiers) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 32, 24}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 44, 14, 16, 10, 12);
  fillFreeBox(occupancy, 22, 24, 2, 16, 10, 12);
  IncrementalTopologyGraph3D graph{makeKnownSpaceConfig()};

  const IncrementalTopologyGraph3DUpdate update = graph.update(occupancy, 1U, {}, true);
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();

  EXPECT_TRUE(update.full_reset);
  EXPECT_GT(update.rebuilt_blocks, 0U);
  EXPECT_TRUE(hasNodeWithMinimumDegree(snapshot, 3U));
  EXPECT_TRUE(std::ranges::none_of(snapshot.nodes(), [](const auto& node) {
    return node.traits.frontier || node.traits.junction;
  }));
}

TEST(IncrementalTopologyGraph3DTest,
     ExpiredDeadlineGuaranteesOneBlockAndTheSameRevisionResumesTheRest) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 8, 8, 8}};
  fillFreeBox(occupancy, 0, 7, 0, 7, 0, 7);
  IncrementalTopologyGraph3D graph{makeKnownSpaceConfig()};

  const IncrementalTopologyGraph3DUpdate deferred = graph.update(
      occupancy, 1U, {}, true, std::nullopt, std::chrono::steady_clock::now());
  EXPECT_TRUE(deferred.deadline_exhausted);
  EXPECT_TRUE(deferred.minimum_progress_guaranteed);
  EXPECT_EQ(deferred.rebuilt_blocks, 1U);
  EXPECT_GT(deferred.pending_blocks, 0U);
  EXPECT_EQ(deferred.source_seen_revision, 1U);
  EXPECT_EQ(deferred.materialized_revision, 1U);
  EXPECT_EQ(deferred.coverage_complete_through_revision, 0U);

  const IncrementalTopologyGraph3DSnapshot partial = graph.snapshot();
  EXPECT_EQ(partial.sourceSeenRevision(), 1U);
  EXPECT_EQ(partial.materializedRevision(), 1U);
  EXPECT_EQ(partial.coverageCompleteThroughRevision(), 0U);

  const IncrementalTopologyGraph3DUpdate resumed =
      graph.update(occupancy, 1U, {}, false);
  EXPECT_FALSE(resumed.deadline_exhausted);
  EXPECT_GT(resumed.rebuilt_blocks, 0U);
  EXPECT_EQ(resumed.pending_blocks, 0U);
  EXPECT_EQ(resumed.coverage_complete_through_revision, 1U);
}

TEST(IncrementalTopologyGraph3DTest, DrainsPendingBlocksWithoutANewerRawRevision) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 16, 8}};
  fillFreeBox(occupancy, 0, 23, 0, 15, 0, 7);
  IncrementalTopologyGraph3DConfig config = makeKnownSpaceConfig();
  config.maximum_observed_blocks_per_update = 1U;
  config.minimum_oldest_blocks_per_update = 0U;
  IncrementalTopologyGraph3D graph{config};

  IncrementalTopologyGraph3DUpdate update = graph.update(occupancy, 1U, {}, true);
  ASSERT_GT(update.pending_blocks, 0U);
  std::size_t continuation_count{0U};
  while (update.pending_blocks > 0U && continuation_count < 1'000U) {
    update = graph.update(occupancy, 1U, {}, false);
    ++continuation_count;
    EXPECT_EQ(update.revision, 1U);
  }

  EXPECT_GT(continuation_count, 0U);
  EXPECT_EQ(update.pending_blocks, 0U);
  EXPECT_LT(continuation_count, 1'000U);
}

TEST(IncrementalTopologyGraph3DTest, ExtractsXJunctionAndLoop) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 48, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 44, 22, 24, 6, 8);
  fillFreeBox(occupancy, 22, 24, 2, 44, 6, 8);
  fillFreeBox(occupancy, 6, 40, 6, 8, 6, 8);
  fillFreeBox(occupancy, 6, 8, 6, 40, 6, 8);
  fillFreeBox(occupancy, 40, 42, 6, 40, 6, 8);
  fillFreeBox(occupancy, 6, 42, 40, 42, 6, 8);
  IncrementalTopologyGraph3D graph{makeKnownSpaceConfig()};

  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();

  EXPECT_TRUE(hasNodeWithMinimumDegree(snapshot, 4U));
  EXPECT_GE(snapshot.edges().size(), snapshot.nodes().size());
}

TEST(IncrementalTopologyGraph3DTest, ClassifiesVerticalConnector) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 24, 32}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 22, 10, 12, 3, 5);
  fillFreeBox(occupancy, 20, 22, 10, 12, 3, 24);
  fillFreeBox(occupancy, 20, 44, 10, 12, 22, 24);
  IncrementalTopologyGraph3D graph{makeKnownSpaceConfig()};

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

TEST(IncrementalTopologyGraph3DTest, ClassifiesOnlyProvenClosedCulDeSacTerminals) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 40, 40, 16}};
  fillStateBox(occupancy, 0, 39, 0, 39, 0, 15, ObservedVoxelState::kOccupied);
  fillFreeBox(occupancy, 3, 20, 8, 10, 5, 7);
  fillFreeBox(occupancy, 18, 20, 8, 32, 5, 7);
  IncrementalTopologyGraph3D graph{makeKnownSpaceConfig()};

  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();

  EXPECT_TRUE(std::ranges::any_of(
      snapshot.nodes(), [](const auto& node) { return node.traits.terminal; }));
  EXPECT_TRUE(std::ranges::none_of(snapshot.nodes(), [](const auto& node) {
    return node.traits.frontier || node.unknown_boundary_exposure;
  }));
}

TEST(IncrementalTopologyGraph3DTest, DirtyUpdateRetainsUnaffectedNodeIdentity) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 64, 32, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 61, 14, 16, 6, 8);
  IncrementalTopologyGraph3D graph{makeKnownSpaceConfig()};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot before = graph.snapshot();
  ASSERT_GT(before.sampleBlockCount(), 0U);
  ASSERT_GT(before.sampleCount(), 0U);
  const std::size_t before_sample_block_count = before.sampleBlockCount();
  const std::size_t before_sample_count = before.sampleCount();
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
  const std::optional<IncrementalTopologyConnector3D> retained_connector =
      before.connectObserved(occupancy, {8.5, 15.5, 7.5}, 2.0,
                             makeKnownSpaceConfig().footprint,
                             ObservedSpaceValidationPolicy::kRequireKnownFree);
  const std::optional<IncrementalTopologyNodeId> distant_after =
      after.nearestNode({56.5, 15.5, 7.5}, 8.0);

  ASSERT_TRUE(retained_connector.has_value());
  EXPECT_FALSE(update.full_reset);
  EXPECT_EQ(update.requested_dirty_chunks, 1U);
  EXPECT_GT(update.retained_node_ids, 0U);
  EXPECT_EQ(before.sampleBlockCount(), before_sample_block_count);
  EXPECT_EQ(before.sampleCount(), before_sample_count);
  EXPECT_EQ(distant_after, distant_before);
  const IncrementalTopologyNode3D* distant_node = after.findNode(distant_id);
  ASSERT_NE(distant_node, nullptr);
  EXPECT_EQ(distant_node->validated_through_revision, 1U);
}

TEST(IncrementalTopologyGraph3DTest,
     SparseDirtyVoxelsDoNotInvalidateTheBoundingVolumeBetweenThem) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 32, 32, 32}};
  fillStateBox(occupancy, 0, 31, 0, 31, 0, 31, ObservedVoxelState::kOccupied);
  IncrementalTopologyGraph3DConfig config = makeKnownSpaceConfig();
  config.maximum_observed_blocks_per_update = 1024U;
  IncrementalTopologyGraph3D graph{config};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));

  ASSERT_TRUE(occupancy.setState({1, 1, 1}, ObservedVoxelState::kFree));
  ASSERT_TRUE(occupancy.setState({30, 30, 30}, ObservedVoxelState::kFree));
  const OccupancyChunkIndex3D dirty = ObservedOccupancyGrid3D::chunkIndex({1, 1, 1});
  const IncrementalTopologyGraph3DUpdate update =
      graph.update(occupancy, 2U, std::span{&dirty, 1U}, false);

  // The old chunk-wide min/max box covered the entire 8x8x8 block volume.
  // A topology update now touches only each changed block and its footprint halo.
  EXPECT_LT(update.discovered_dirty_blocks, 100U);
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
    return node.traits.frontier || node.unknown_boundary_exposure;
  }));
}

TEST(IncrementalTopologyGraph3DTest,
     AdaptivelyRefinesAFeasibleCorridorMissedByCoarseSampling) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 8, 8}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 1, 18, 1, 1, 1, 1);
  IncrementalTopologyGraph3DConfig adaptive_config = makeKnownSpaceConfig();
  adaptive_config.coarse_sample_stride_cells = 2;
  adaptive_config.refined_sample_stride_cells = 1;
  IncrementalTopologyGraph3D adaptive_graph{adaptive_config};

  const IncrementalTopologyGraph3DUpdate adaptive_update =
      adaptive_graph.update(occupancy, 1U, {}, true);
  const IncrementalTopologyGraph3DSnapshot adaptive = adaptive_graph.snapshot();

  EXPECT_GT(adaptive_update.adaptively_refined_blocks, 0U);
  EXPECT_GT(adaptive_update.sampled_navigable_cells, 0U);
  EXPECT_TRUE(adaptive.nearestNode({1.5, 1.5, 1.5}, 2.0).has_value());
  EXPECT_TRUE(adaptive.nearestNode({18.5, 1.5, 1.5}, 2.0).has_value());
  EXPECT_GT(adaptive.edges().size(), 0U);

  IncrementalTopologyGraph3DConfig coarse_only_config = adaptive_config;
  coarse_only_config.refined_sample_stride_cells = 2;
  IncrementalTopologyGraph3D coarse_only_graph{coarse_only_config};
  const IncrementalTopologyGraph3DUpdate coarse_only_update =
      coarse_only_graph.update(occupancy, 1U, {}, true);

  EXPECT_EQ(coarse_only_update.adaptively_refined_blocks, 0U);
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
  EXPECT_GT(update.adaptively_refined_blocks, 0U);
  EXPECT_LT(update.adaptively_refined_blocks, update.rebuilt_blocks);
  EXPECT_TRUE(nodesConnected(snapshot, refined_id, coarse_id));
}

TEST(IncrementalTopologyGraph3DTest,
     RetainsNodeIdentityWhenDirtyBlockChangesSamplingResolution) {
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
  EXPECT_EQ(initial.adaptively_refined_blocks, 0U);

  static_cast<void>(occupancy.setState({6, 6, 4}, ObservedVoxelState::kOccupied));
  const OccupancyChunkIndex3D dirty = ObservedOccupancyGrid3D::chunkIndex({6, 6, 4});
  const IncrementalTopologyGraph3DUpdate refined_update =
      graph.update(occupancy, 2U, std::span{&dirty, 1U}, false);
  const IncrementalTopologyGraph3DSnapshot after = graph.snapshot();
  const IncrementalTopologyNodeId before_id =
      before.value_or(IncrementalTopologyNodeId{});
  const IncrementalTopologyNode3D* retained = after.findNode(before_id);

  ASSERT_NE(retained, nullptr);
  EXPECT_GT(refined_update.adaptively_refined_blocks, 0U);
  EXPECT_GT(refined_update.retained_node_ids, 0U);
  EXPECT_EQ(retained->validated_through_revision, 2U);
}

TEST(IncrementalTopologyGraph3DTest,
     PermissiveResetMaterializesBlocksContainingAnyObservedEvidence) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 96, 96, 48}};
  ASSERT_TRUE(occupancy.setState({1, 1, 1}, ObservedVoxelState::kFree));
  ASSERT_TRUE(occupancy.setState({47, 47, 23}, ObservedVoxelState::kFree));
  ASSERT_TRUE(occupancy.setState({95, 95, 47}, ObservedVoxelState::kOccupied));
  IncrementalTopologyGraph3D graph{makeConfig()};

  const IncrementalTopologyGraph3DUpdate update = graph.update(occupancy, 1U, {}, true);

  EXPECT_TRUE(update.full_reset);
  EXPECT_GT(update.rebuilt_blocks, 0U);
  EXPECT_LE(update.rebuilt_blocks, 16U);
  EXPECT_EQ(graph.snapshot().blockCoverage().size(), 3U);
}

TEST(IncrementalTopologyGraph3DTest,
     PermissiveFreeEvidenceRefreshesWithoutGeometryRebuild) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 96, 96, 48}};
  ASSERT_TRUE(occupancy.setState({17, 17, 17}, ObservedVoxelState::kFree));
  IncrementalTopologyGraph3D graph{makeConfig()};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const std::optional<IncrementalTopologyNodeId> before =
      graph.snapshot().nearestNode({17.5, 17.5, 17.5}, 8.0);
  ASSERT_TRUE(before.has_value());

  ASSERT_TRUE(occupancy.setState({18, 17, 17}, ObservedVoxelState::kFree));
  const OccupancyChunkIndex3D dirty = ObservedOccupancyGrid3D::chunkIndex({18, 17, 17});
  const IncrementalTopologyGraph3DUpdate update =
      graph.update(occupancy, 2U, std::span{&dirty, 1U}, false);

  EXPECT_FALSE(update.full_reset);
  EXPECT_EQ(update.requested_dirty_chunks, 1U);
  EXPECT_EQ(update.rebuilt_blocks, 0U);
  EXPECT_GT(update.refreshed_observation_blocks, 0U);
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();
  EXPECT_EQ(snapshot.nearestNode({17.5, 17.5, 17.5}, 8.0), before);
  const IncrementalTopologyNode3D* retained =
      snapshot.findNode(before.value_or(IncrementalTopologyNodeId{}));
  ASSERT_NE(retained, nullptr);
  EXPECT_EQ(retained->validated_through_revision, 2U);
  EXPECT_EQ(retained->complete_through_revision, 1U);
  const auto coverage = std::ranges::find(snapshot.blockCoverage(), retained->block,
                                          &IncrementalTopologyBlockCoverage3D::block);
  ASSERT_NE(coverage, snapshot.blockCoverage().end());
  EXPECT_EQ(coverage->validated_through_revision, 2U);
  EXPECT_EQ(coverage->complete_through_revision, 1U);
}

TEST(IncrementalTopologyGraph3DTest,
     PermissiveOccupiedTransitionRebuildsAffectedGeometry) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 32, 32, 24}};
  ASSERT_TRUE(occupancy.setState({17, 17, 17}, ObservedVoxelState::kFree));
  IncrementalTopologyGraph3D graph{makeConfig()};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));

  ASSERT_TRUE(occupancy.setState({18, 17, 17}, ObservedVoxelState::kOccupied));
  const OccupancyChunkIndex3D dirty = ObservedOccupancyGrid3D::chunkIndex({18, 17, 17});
  const IncrementalTopologyGraph3DUpdate update =
      graph.update(occupancy, 2U, std::span{&dirty, 1U}, false);

  EXPECT_EQ(update.requested_dirty_chunks, 1U);
  EXPECT_GT(update.rebuilt_blocks, 0U);
  EXPECT_LE(update.rebuilt_blocks, 64U);
}

TEST(IncrementalTopologyGraph3DTest,
     StrictFreeEvidenceTransitionRebuildsAffectedGeometry) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 32, 32, 24}};
  ASSERT_TRUE(occupancy.setState({17, 17, 17}, ObservedVoxelState::kFree));
  IncrementalTopologyGraph3D graph{makeKnownSpaceConfig()};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));

  ASSERT_TRUE(occupancy.setState({18, 17, 17}, ObservedVoxelState::kFree));
  const OccupancyChunkIndex3D dirty = ObservedOccupancyGrid3D::chunkIndex({18, 17, 17});
  const IncrementalTopologyGraph3DUpdate update =
      graph.update(occupancy, 2U, std::span{&dirty, 1U}, false);

  EXPECT_EQ(update.requested_dirty_chunks, 1U);
  EXPECT_GT(update.rebuilt_blocks, 0U);
  EXPECT_EQ(update.refreshed_observation_blocks, 0U);
}

TEST(IncrementalTopologyGraph3DTest,
     CompleteSnapshotAfterInitializationPreservesIncrementalIdentity) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 64, 32, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 61, 14, 16, 6, 8);
  IncrementalTopologyGraph3D graph{makeKnownSpaceConfig()};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const std::optional<IncrementalTopologyNodeId> before =
      graph.snapshot().nearestNode({56.5, 15.5, 7.5}, 8.0);
  ASSERT_TRUE(before.has_value());

  fillFreeBox(occupancy, 8, 10, 4, 16, 6, 8);
  const IncrementalTopologyGraph3DUpdate update = graph.update(occupancy, 2U, {}, true);
  const std::optional<IncrementalTopologyNodeId> after =
      graph.snapshot().nearestNode({56.5, 15.5, 7.5}, 8.0);

  EXPECT_FALSE(update.full_reset);
  EXPECT_GT(update.requested_dirty_chunks, 0U);
  EXPECT_GT(update.retained_node_ids, 0U);
  EXPECT_EQ(after, before);
}

TEST(IncrementalTopologyGraph3DTest,
     DefersObservedBlockWorkWithoutDroppingDirtyGeometry) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 96, 32, 16}};
  fillFreeBox(occupancy, 2, 10, 14, 16, 6, 8);
  IncrementalTopologyGraph3DConfig config = makeKnownSpaceConfig();
  config.maximum_observed_blocks_per_update = 1U;
  config.minimum_oldest_blocks_per_update = 0U;
  IncrementalTopologyGraph3D graph{config};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));

  fillFreeBox(occupancy, 18, 22, 14, 16, 6, 8);
  fillFreeBox(occupancy, 66, 70, 14, 16, 6, 8);
  const std::array dirty_chunks{
      ObservedOccupancyGrid3D::chunkIndex({18, 15, 7}),
      ObservedOccupancyGrid3D::chunkIndex({66, 15, 7}),
  };
  const IncrementalTopologyGraph3DUpdate first =
      graph.update(occupancy, 2U, dirty_chunks, false);

  EXPECT_GT(first.discovered_dirty_blocks, 1U);
  EXPECT_EQ(first.rebuilt_blocks, 1U);
  EXPECT_GT(first.pending_blocks, 0U);

  std::size_t previous_pending = first.pending_blocks;
  std::uint64_t revision = 3U;
  while (previous_pending > 0U) {
    const IncrementalTopologyGraph3DUpdate next =
        graph.update(occupancy, revision++, {}, false);
    EXPECT_EQ(next.rebuilt_blocks, 1U);
    EXPECT_LT(next.pending_blocks, previous_pending);
    previous_pending = next.pending_blocks;
  }
  EXPECT_TRUE(graph.snapshot().nearestNode({68.5, 15.5, 7.5}, 8.0).has_value());
}

TEST(IncrementalTopologyGraph3DTest,
     PrioritizesDeferredObservedBlocksNearTheCurrentVehicle) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 96, 32, 16}};
  fillFreeBox(occupancy, 2, 10, 14, 16, 6, 8);
  IncrementalTopologyGraph3DConfig config = makeKnownSpaceConfig();
  config.maximum_observed_blocks_per_update = 1U;
  config.minimum_oldest_blocks_per_update = 0U;
  IncrementalTopologyGraph3D graph{config};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));

  fillFreeBox(occupancy, 18, 22, 14, 16, 6, 8);
  fillFreeBox(occupancy, 66, 70, 14, 16, 6, 8);
  const std::array dirty_chunks{
      ObservedOccupancyGrid3D::chunkIndex({18, 15, 7}),
      ObservedOccupancyGrid3D::chunkIndex({66, 15, 7}),
  };
  const IncrementalTopologyGraph3DUpdate update =
      graph.update(occupancy, 2U, dirty_chunks, false,
                   IncrementalTopologyBuildPriority3D{.position = {68.5, 15.5, 7.5},
                                                      .target = {68.5, 15.5, 7.5}});
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();

  EXPECT_EQ(update.rebuilt_blocks, 1U);
  EXPECT_GT(update.pending_blocks, 0U);
  EXPECT_TRUE(snapshot.nearestNode({68.5, 15.5, 7.5}, 8.0).has_value());
  EXPECT_FALSE(snapshot.nearestNode({20.5, 15.5, 7.5}, 8.0).has_value());
}

TEST(IncrementalTopologyGraph3DTest,
     RecurrentPriorityWorkCannotStarveOlderDirtyBlocks) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 96, 32, 16}};
  fillFreeBox(occupancy, 2, 10, 14, 16, 6, 8);
  IncrementalTopologyGraph3DConfig config = makeKnownSpaceConfig();
  config.maximum_observed_blocks_per_update = 2U;
  config.minimum_oldest_blocks_per_update = 1U;
  IncrementalTopologyGraph3D graph{config};
  std::uint64_t revision = 1U;
  IncrementalTopologyGraph3DUpdate update =
      graph.update(occupancy, revision++, {}, true,
                   IncrementalTopologyBuildPriority3D{.position = {20.5, 15.5, 7.5},
                                                      .target = {80.5, 15.5, 7.5}});
  while (update.pending_blocks > 0U) {
    update = graph.update(occupancy, revision++, {}, false);
  }

  fillFreeBox(occupancy, 18, 22, 14, 16, 6, 8);
  fillFreeBox(occupancy, 66, 70, 14, 16, 6, 8);
  const OccupancyChunkIndex3D near_chunk =
      ObservedOccupancyGrid3D::chunkIndex({18, 15, 7});
  const OccupancyChunkIndex3D far_chunk =
      ObservedOccupancyGrid3D::chunkIndex({66, 15, 7});
  const std::array initial_dirty{near_chunk, far_chunk};
  update =
      graph.update(occupancy, revision++, initial_dirty, false,
                   IncrementalTopologyBuildPriority3D{.position = {20.5, 15.5, 7.5},
                                                      .target = {80.5, 15.5, 7.5}});

  for (std::size_t attempt = 0U;
       attempt < 64U &&
       !graph.snapshot().nearestNode({68.5, 15.5, 7.5}, 8.0).has_value();
       ++attempt) {
    const GridIndex3D changing_cell{18, 15, 7};
    const ObservedVoxelState changing_state =
        attempt % 2U == 0U ? ObservedVoxelState::kUnknown : ObservedVoxelState::kFree;
    ASSERT_TRUE(occupancy.setState(changing_cell, changing_state));
    const std::array recurrent_dirty{near_chunk};
    update =
        graph.update(occupancy, revision++, recurrent_dirty, false,
                     IncrementalTopologyBuildPriority3D{.position = {20.5, 15.5, 7.5},
                                                        .target = {80.5, 15.5, 7.5}});
  }

  EXPECT_TRUE(graph.snapshot().nearestNode({68.5, 15.5, 7.5}, 8.0).has_value());
}

TEST(IncrementalTopologyGraph3DTest,
     InitialObservedResetIsBudgetedAndPrioritizedNearTheVehicle) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 96, 32, 16}};
  fillFreeBox(occupancy, 2, 10, 14, 16, 6, 8);
  fillFreeBox(occupancy, 66, 74, 14, 16, 6, 8);
  IncrementalTopologyGraph3DConfig config = makeKnownSpaceConfig();
  config.maximum_observed_blocks_per_update = 1U;
  config.minimum_oldest_blocks_per_update = 0U;
  IncrementalTopologyGraph3D graph{config};

  const IncrementalTopologyGraph3DUpdate update =
      graph.update(occupancy, 1U, {}, true,
                   IncrementalTopologyBuildPriority3D{.position = {70.5, 15.5, 7.5},
                                                      .target = {70.5, 15.5, 7.5}});
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();

  EXPECT_TRUE(update.full_reset);
  EXPECT_EQ(update.rebuilt_blocks, 1U);
  EXPECT_GT(update.pending_blocks, 0U);
  EXPECT_TRUE(snapshot.nearestNode({70.5, 15.5, 7.5}, 8.0).has_value());
  EXPECT_FALSE(snapshot.nearestNode({6.5, 15.5, 7.5}, 8.0).has_value());
}

TEST(IncrementalTopologyGraph3DTest,
     LargeBacklogUsesBoundedBoostAndStillReservesOldestWork) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 16, 8}};
  fillFreeBox(occupancy, 0, 47, 0, 15, 0, 7);
  IncrementalTopologyGraph3DConfig config = makeKnownSpaceConfig();
  config.maximum_observed_blocks_per_update = 2U;
  config.maximum_backlog_blocks_per_update = 5U;
  config.backlog_boost_threshold_blocks = 4U;
  config.minimum_oldest_blocks_per_update = 1U;
  IncrementalTopologyGraph3D graph{config};
  const IncrementalTopologyBuildPriority3D priority{
      .position = {2.0, 2.0, 2.0},
      .target = {46.0, 2.0, 2.0},
      .local_radius_m = 2.0,
      .forward_corridor_radius_m = 4.0,
      .forward_corridor_lookahead_m = 44.0,
  };

  const IncrementalTopologyGraph3DUpdate first =
      graph.update(occupancy, 1U, {}, true, priority);
  ASSERT_TRUE(first.backlog_boosted);
  EXPECT_EQ(first.scheduled_block_budget, 5U);
  EXPECT_EQ(first.rebuilt_blocks, 5U);
  EXPECT_GT(first.pending_blocks, 0U);
  EXPECT_GT(first.local_priority_blocks, 0U);
  EXPECT_GT(first.forward_corridor_blocks, 0U);

  const IncrementalTopologyGraph3DUpdate second =
      graph.update(occupancy, 1U, {}, false, priority);
  EXPECT_TRUE(second.backlog_boosted);
  EXPECT_EQ(second.rebuilt_blocks, 5U);
  EXPECT_EQ(second.oldest_preserved_blocks, 1U);
  EXPECT_LT(second.pending_blocks, first.pending_blocks);
}

TEST(IncrementalTopologyGraph3DTest,
     StoresRawSafePortalPolylinesWhenTheRepresentativeChordIsBlocked) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 16, 12, 6}};
  fillStateBox(occupancy, 0, 15, 0, 11, 0, 5, ObservedVoxelState::kOccupied);
  fillFreeBox(occupancy, 1, 6, 1, 3, 1, 3);
  fillFreeBox(occupancy, 1, 3, 1, 8, 1, 3);
  fillFreeBox(occupancy, 1, 14, 6, 8, 1, 3);
  IncrementalTopologyGraph3DConfig config = makeKnownSpaceConfig();
  config.block_size_cells = 8;
  IncrementalTopologyGraph3D graph{config};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();

  bool found_non_chord_edge = false;
  for (const IncrementalTopologyEdge3D& edge : snapshot.edges()) {
    const IncrementalTopologyNode3D* first = snapshot.findNode(edge.first);
    const IncrementalTopologyNode3D* second = snapshot.findNode(edge.second);
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    const bool polyline_safe = std::ranges::all_of(
        std::views::iota(std::size_t{1U}, edge.polyline.size()),
        [&](const std::size_t index) {
          return rawSweptFootprintIsNavigable(occupancy, edge.polyline[index - 1U],
                                              FootprintBodyAxis{}, edge.polyline[index],
                                              FootprintBodyAxis{}, config.footprint);
        });
    EXPECT_TRUE(polyline_safe);
    if (!rawSweptFootprintIsNavigable(occupancy, first->representative,
                                      FootprintBodyAxis{}, second->representative,
                                      FootprintBodyAxis{}, config.footprint)) {
      EXPECT_GT(edge.polyline.size(), 2U);
      found_non_chord_edge = true;
    }
  }
  EXPECT_TRUE(found_non_chord_edge);
}

TEST(IncrementalTopologyGraph3DTest,
     UShapedFrontierAnchorUsesParentCellsInsteadOfABlockedChord) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 16, 12, 6}};
  fillStateBox(occupancy, 0, 15, 0, 11, 0, 5, ObservedVoxelState::kOccupied);
  fillFreeBox(occupancy, 1, 14, 1, 3, 1, 3);
  fillFreeBox(occupancy, 1, 3, 1, 8, 1, 3);
  fillFreeBox(occupancy, 1, 14, 6, 8, 1, 3);
  IncrementalTopologyGraph3DConfig config = makeKnownSpaceConfig();
  config.block_size_cells = 16;
  IncrementalTopologyGraph3D graph{config};
  static_cast<void>(graph.update(occupancy, 5U, {}, true));
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();

  bool reconstructed_non_chord = false;
  for (int y = 0; y < occupancy.bounds().height_cells && !reconstructed_non_chord;
       ++y) {
    for (int x = 0; x < occupancy.bounds().width_cells && !reconstructed_non_chord;
         ++x) {
      const GridIndex3D cell{x, y, 2};
      const std::optional<IncrementalTopologyNodeId> node_id =
          snapshot.nodeForSampleCell(cell);
      const IncrementalTopologyNode3D* const node =
          node_id.has_value() ? snapshot.findNode(*node_id) : nullptr;
      if (node == nullptr ||
          rawSweptFootprintIsNavigable(occupancy, occupancy.cellCenter(cell),
                                       FootprintBodyAxis{}, node->representative,
                                       FootprintBodyAxis{}, config.footprint)) {
        continue;
      }
      const std::optional<IncrementalTopologyConnector3D> connector =
          snapshot.connectObservedSample(
              occupancy, cell, config.footprint,
              ObservedSpaceValidationPolicy::kRequireKnownFree);
      if (!connector.has_value()) {
        FAIL() << "sampled parent-cell connector must be reconstructible";
      }
      const IncrementalTopologyConnector3D& reconstructed = connector.value();
      ASSERT_GT(reconstructed.polyline.size(), 2U);
      EXPECT_TRUE(std::ranges::all_of(
          std::views::iota(std::size_t{1U}, reconstructed.polyline.size()),
          [&](const std::size_t index) {
            return validateObservedSweptFootprint(
                       occupancy, reconstructed.polyline[index - 1U],
                       FootprintBodyAxis{}, reconstructed.polyline[index],
                       FootprintBodyAxis{}, config.footprint,
                       ObservedSpaceValidationPolicy::kRequireKnownFree)
                .accepted();
          }));
      EXPECT_EQ(reconstructed.evidence.kind,
                IncrementalTopologyTransitionKind3D::kObservedFree);
      EXPECT_FALSE(reconstructed.evidence.unknown_exposure);
      EXPECT_EQ(reconstructed.evidence.validated_through_revision, 5U);
      EXPECT_EQ(reconstructed.evidence.complete_through_revision, 5U);
      EXPECT_NE(reconstructed.evidence.lineage_id, 0U);
      reconstructed_non_chord = true;
    }
  }
  EXPECT_TRUE(reconstructed_non_chord);
}

TEST(IncrementalTopologyGraph3DTest,
     DistinguishesObservedFreeAndOptimisticUnknownTransitions) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 12, 6}};
  fillStateBox(occupancy, 0, 23, 0, 11, 0, 5, ObservedVoxelState::kOccupied);
  fillFreeBox(occupancy, 1, 9, 4, 6, 1, 3);
  fillStateBox(occupancy, 10, 22, 4, 6, 1, 3, ObservedVoxelState::kUnknown);
  IncrementalTopologyGraph3DConfig config = makeConfig();
  config.maximum_observed_blocks_per_update = 4096U;
  IncrementalTopologyGraph3D graph{config};
  static_cast<void>(graph.update(occupancy, 7U, {}, true));
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();

  bool observed_free = false;
  bool optimistic_unknown = false;
  for (const IncrementalTopologyEdge3D& edge : snapshot.edges()) {
    EXPECT_GT(edge.evidence.support_segment_count, 0U);
    EXPECT_EQ(edge.evidence.validated_through_revision, 7U);
    EXPECT_EQ(edge.evidence.complete_through_revision, 7U);
    EXPECT_NE(edge.evidence.lineage_id, 0U);
    if (edge.evidence.kind == IncrementalTopologyTransitionKind3D::kOptimisticUnknown) {
      EXPECT_TRUE(edge.evidence.unknown_exposure);
      optimistic_unknown = true;
    } else {
      EXPECT_FALSE(edge.evidence.unknown_exposure);
      observed_free = true;
    }
  }
  EXPECT_TRUE(observed_free);
  EXPECT_TRUE(optimistic_unknown);
}

TEST(IncrementalTopologyGraph3DTest,
     FreeEvidenceAdvancesValidationWithoutRewritingTransitionLineage) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 16, 8, 6}};
  fillStateBox(occupancy, 0, 15, 0, 7, 0, 5, ObservedVoxelState::kOccupied);
  fillStateBox(occupancy, 1, 14, 2, 4, 1, 3, ObservedVoxelState::kUnknown);
  IncrementalTopologyGraph3DConfig config = makeConfig();
  config.maximum_observed_blocks_per_update = 4096U;
  IncrementalTopologyGraph3D graph{config};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot optimistic = graph.snapshot();
  ASSERT_FALSE(optimistic.edges().empty());
  const IncrementalTopologyEdge3D first = optimistic.edges().front();
  EXPECT_EQ(first.evidence.kind,
            IncrementalTopologyTransitionKind3D::kOptimisticUnknown);
  EXPECT_EQ(first.evidence.validated_through_revision, 1U);
  EXPECT_EQ(first.evidence.complete_through_revision, 1U);

  fillFreeBox(occupancy, 1, 14, 2, 4, 1, 3);
  const OccupancyChunkIndex3D dirty = ObservedOccupancyGrid3D::chunkIndex({1, 2, 1});
  const IncrementalTopologyGraph3DUpdate update =
      graph.update(occupancy, 2U, std::span{&dirty, 1U}, false);
  const IncrementalTopologyGraph3DSnapshot observed = graph.snapshot();
  const auto retained =
      std::ranges::find(observed.edges(), first.id, &IncrementalTopologyEdge3D::id);

  ASSERT_NE(retained, observed.edges().end());
  EXPECT_EQ(update.rebuilt_blocks, 0U);
  EXPECT_GT(update.refreshed_observation_blocks, 0U);
  EXPECT_EQ(retained->evidence.kind,
            IncrementalTopologyTransitionKind3D::kObservedFree);
  EXPECT_FALSE(retained->evidence.unknown_exposure);
  EXPECT_EQ(retained->evidence.validated_through_revision, 2U);
  EXPECT_EQ(retained->evidence.complete_through_revision, 1U);
  EXPECT_EQ(retained->evidence.lineage_id, first.evidence.lineage_id);
}

TEST(IncrementalTopologyGraph3DTest, ObservedConnectorKeepsUnknownPolicyExplicit) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 12, 6}};
  fillStateBox(occupancy, 0, 19, 0, 11, 0, 5, ObservedVoxelState::kOccupied);
  fillFreeBox(occupancy, 1, 6, 1, 3, 1, 3);
  fillFreeBox(occupancy, 1, 3, 1, 8, 1, 3);
  fillFreeBox(occupancy, 1, 9, 6, 8, 1, 3);
  fillStateBox(occupancy, 10, 19, 6, 8, 1, 3, ObservedVoxelState::kUnknown);
  IncrementalTopologyGraph3DConfig config = makeConfig();
  config.block_size_cells = 16;
  IncrementalTopologyGraph3D graph{config};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();

  const std::optional<IncrementalTopologyConnector3D> parent_tree =
      snapshot.connectObserved(occupancy, {12.5, 7.5, 2.5}, 20.0, config.footprint,
                               ObservedSpaceValidationPolicy::kAllowUnknown);
  if (!parent_tree.has_value()) {
    FAIL() << "permissive observed connector must accept unknown exposure";
  }
  const IncrementalTopologyConnector3D& connector = *parent_tree;
  ASSERT_GE(connector.polyline.size(), 2U);
  EXPECT_EQ(connector.evidence.kind,
            IncrementalTopologyTransitionKind3D::kOptimisticUnknown);
  EXPECT_TRUE(connector.evidence.unknown_exposure);
  EXPECT_GT(connector.evidence.support_segment_count, 0U);
  EXPECT_EQ(connector.evidence.validated_through_revision, 1U);
  EXPECT_EQ(connector.evidence.complete_through_revision, 1U);
  EXPECT_NE(connector.evidence.lineage_id, 0U);
  EXPECT_TRUE(std::ranges::all_of(
      std::views::iota(std::size_t{1U}, connector.polyline.size()),
      [&](const std::size_t index) {
        const SweptFootprintResult evidence = validateRawSweptFootprint(
            occupancy, connector.polyline[index - 1U], FootprintBodyAxis{},
            connector.polyline[index], FootprintBodyAxis{}, config.footprint);
        return !evidence.evidence.raw_collision &&
               !evidence.evidence.outside_grid_exposure;
      }));

  EXPECT_FALSE(snapshot
                   .connectObserved(occupancy, {12.5, 7.5, 2.5}, 20.0, config.footprint,
                                    ObservedSpaceValidationPolicy::kRequireKnownFree)
                   .has_value());
}

TEST(IncrementalTopologyGraph3DTest, ObservedConnectorHonorsExpiredDeadline) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 12, 6}};
  fillFreeBox(occupancy, 0, 19, 0, 11, 0, 5);
  IncrementalTopologyGraph3DConfig config = makeConfig();
  IncrementalTopologyGraph3D graph{config};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();
  const auto expired = std::chrono::steady_clock::now() - std::chrono::milliseconds{1};

  EXPECT_FALSE(snapshot
                   .connectObserved(occupancy, {5.5, 5.5, 2.5}, 20.0, config.footprint,
                                    ObservedSpaceValidationPolicy::kRequireKnownFree,
                                    expired)
                   .has_value());
}

TEST(IncrementalTopologyGraph3DTest,
     ObservedConnectorSelectsTheLocalComponentAmongManyDistantSamples) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 192, 24, 8}};
  fillStateBox(occupancy, 0, 191, 0, 23, 0, 7, ObservedVoxelState::kOccupied);
  for (int minimum_x = 2; minimum_x < 188; minimum_x += 12) {
    fillFreeBox(occupancy, minimum_x, minimum_x + 5, 9, 13, 2, 5);
  }
  IncrementalTopologyGraph3DConfig config = makeKnownSpaceConfig();
  config.maximum_observed_blocks_per_update = 4096U;
  IncrementalTopologyGraph3D graph{config};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();

  const std::optional<IncrementalTopologyConnector3D> connector =
      snapshot.connectObserved(occupancy, {183.25, 11.5, 3.5}, 5.0, config.footprint,
                               ObservedSpaceValidationPolicy::kRequireKnownFree);

  if (!connector.has_value()) {
    FAIL() << "local observed connector must exist";
  }
  const IncrementalTopologyConnector3D& local = *connector;
  const IncrementalTopologyNode3D* node = snapshot.findNode(local.node);
  if (node == nullptr) {
    FAIL() << "connector node must belong to the snapshot";
  }
  EXPECT_GT(node->representative.x, 178.0);
  EXPECT_LT(local.length_m, 5.0);
  EXPECT_TRUE(std::ranges::all_of(
      std::views::iota(std::size_t{1U}, local.polyline.size()),
      [&](const std::size_t index) {
        return validateObservedSweptFootprint(
                   occupancy, local.polyline[index - 1U], FootprintBodyAxis{},
                   local.polyline[index], FootprintBodyAxis{}, config.footprint,
                   ObservedSpaceValidationPolicy::kRequireKnownFree)
            .accepted();
      }));
}

TEST(IncrementalTopologyGraph3DTest, RecordsExplicitMergeAndSplitLineage) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 16, 12, 8}};
  fillStateBox(occupancy, 0, 15, 0, 11, 0, 7, ObservedVoxelState::kOccupied);
  fillFreeBox(occupancy, 2, 13, 2, 3, 3, 4);
  fillFreeBox(occupancy, 2, 13, 8, 9, 3, 4);
  IncrementalTopologyGraph3DConfig config = makeConfig();
  config.block_size_cells = 16;
  IncrementalTopologyGraph3D graph{config};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot separated = graph.snapshot();
  ASSERT_EQ(separated.nodes().size(), 2U);
  const std::array predecessor_ids{separated.nodes()[0].id, separated.nodes()[1].id};

  fillFreeBox(occupancy, 7, 8, 4, 7, 3, 4);
  const OccupancyChunkIndex3D dirty = ObservedOccupancyGrid3D::chunkIndex({7, 5, 3});
  static_cast<void>(graph.update(occupancy, 2U, std::span{&dirty, 1U}, false));
  const IncrementalTopologyGraph3DSnapshot merged = graph.snapshot();
  ASSERT_EQ(merged.nodes().size(), 1U);
  EXPECT_EQ(merged.nodes()[0].lineage_event, IncrementalTopologyLineageEvent3D::kMerge);
  EXPECT_EQ(merged.nodes()[0].predecessors.size(), 2U);
  EXPECT_TRUE(std::ranges::all_of(predecessor_ids, [&](const auto predecessor) {
    return std::ranges::find(merged.nodes()[0].predecessors, predecessor) !=
           merged.nodes()[0].predecessors.end();
  }));
  const IncrementalTopologyNodeId merged_id = merged.nodes()[0].id;

  fillStateBox(occupancy, 7, 8, 4, 7, 3, 4, ObservedVoxelState::kOccupied);
  static_cast<void>(graph.update(occupancy, 3U, std::span{&dirty, 1U}, false));
  const IncrementalTopologyGraph3DSnapshot split = graph.snapshot();
  ASSERT_EQ(split.nodes().size(), 2U);
  EXPECT_TRUE(std::ranges::all_of(split.nodes(), [&](const auto& node) {
    return node.lineage_event == IncrementalTopologyLineageEvent3D::kSplit &&
           std::ranges::find(node.predecessors, merged_id) != node.predecessors.end() &&
           node.generation > merged.nodes()[0].generation;
  }));
}

TEST(IncrementalTopologyGraph3DTest,
     ExposesMixedValidatedRevisionsAndPendingCoverageWithoutGlobalEquality) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 40, 12, 8}};
  fillFreeBox(occupancy, 1, 38, 4, 6, 2, 4);
  IncrementalTopologyGraph3DConfig config = makeKnownSpaceConfig();
  config.maximum_observed_blocks_per_update = 1U;
  config.minimum_oldest_blocks_per_update = 0U;
  IncrementalTopologyGraph3D graph{config};
  const IncrementalTopologyGraph3DUpdate first = graph.update(occupancy, 1U, {}, true);
  ASSERT_GT(first.pending_blocks, 0U);

  const IncrementalTopologyGraph3DUpdate second =
      graph.update(occupancy, 2U, {}, false);
  const IncrementalTopologyGraph3DSnapshot snapshot = graph.snapshot();
  EXPECT_GT(second.pending_blocks, 0U);
  EXPECT_TRUE(std::ranges::any_of(snapshot.blockCoverage(), [](const auto& coverage) {
    return !coverage.pending_rebuild && coverage.validated_through_revision == 1U &&
           coverage.complete_through_revision == 1U;
  }));
  EXPECT_TRUE(std::ranges::any_of(snapshot.blockCoverage(), [](const auto& coverage) {
    return !coverage.pending_rebuild && coverage.validated_through_revision == 2U &&
           coverage.complete_through_revision == 2U;
  }));
  EXPECT_TRUE(std::ranges::any_of(snapshot.blockCoverage(), [](const auto& coverage) {
    return coverage.pending_rebuild;
  }));
}

} // namespace
} // namespace drone_city_nav
