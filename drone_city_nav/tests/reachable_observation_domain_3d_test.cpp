#include "drone_city_nav/reachable_observation_domain_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <optional>
#include <ranges>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] IncrementalTopologyGraph3DConfig makeGraphConfig() {
  IncrementalTopologyGraph3DConfig config;
  config.block_size_cells = 16;
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
  config.require_known_free_space = true;
  return config;
}

void fillBox(ObservedOccupancyGrid3D& occupancy, const GridIndex3D minimum,
             const GridIndex3D maximum, const ObservedVoxelState state) {
  for (int z = minimum.z; z <= maximum.z; ++z) {
    for (int y = minimum.y; y <= maximum.y; ++y) {
      for (int x = minimum.x; x <= maximum.x; ++x) {
        static_cast<void>(occupancy.setState({x, y, z}, state));
        ASSERT_EQ(occupancy.state({x, y, z}), state);
      }
    }
  }
}

[[nodiscard]] std::vector<IncrementalTopologyNodeId>
allNodeIds(const IncrementalTopologyGraph3DSnapshot& graph) {
  std::vector<IncrementalTopologyNodeId> result;
  result.reserve(graph.nodes().size());
  std::ranges::transform(graph.nodes(), std::back_inserter(result),
                         &IncrementalTopologyNode3D::id);
  return result;
}

[[nodiscard]] bool containsCandidateNear(const ReachableObservationDomain3D& domain,
                                         const ObservedOccupancyGrid3D& occupancy,
                                         const Point3& target) {
  return std::ranges::any_of(domain.candidateCells(), [&](const GridIndex3D cell) {
    return distance3D(occupancy.cellCenter(cell), target) < 1.0;
  });
}

TEST(ReachableObservationDomain3DTest,
     ExtendsAStaleGraphThroughFreshFreeSpaceWithAParentTreeConnector) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 12, 6}};
  fillBox(occupancy, {1, 1, 1}, {7, 3, 3}, ObservedVoxelState::kFree);

  const IncrementalTopologyGraph3DConfig graph_config = makeGraphConfig();
  IncrementalTopologyGraph3D graph{graph_config};
  static_cast<void>(graph.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot stale_graph = graph.snapshot();
  ASSERT_FALSE(stale_graph.nodes().empty());

  fillBox(occupancy, {0, 0, 0}, {19, 11, 5}, ObservedVoxelState::kOccupied);
  fillBox(occupancy, {1, 1, 1}, {7, 3, 3}, ObservedVoxelState::kFree);
  fillBox(occupancy, {1, 1, 1}, {3, 8, 3}, ObservedVoxelState::kFree);
  fillBox(occupancy, {1, 6, 1}, {14, 8, 3}, ObservedVoxelState::kFree);

  const std::vector<IncrementalTopologyNodeId> reachable_nodes =
      allNodeIds(stale_graph);
  const ReachableObservationDomain3D domain = buildReachableObservationDomain3D(
      stale_graph, occupancy, reachable_nodes,
      ReachableObservationDomain3DConfig{
          .maximum_fresh_extension_m = 30.0,
          .footprint = graph_config.footprint,
          .validation_policy = ObservedSpaceValidationPolicy::kRequireKnownFree,
      });

  const Point3 target{13.5, 7.5, 2.5};
  EXPECT_TRUE(containsCandidateNear(domain, occupancy, target));
  EXPECT_FALSE(stale_graph
                   .connectObserved(occupancy, target, 30.0, graph_config.footprint,
                                    ObservedSpaceValidationPolicy::kAllowUnknown)
                   .has_value());

  const std::optional<IncrementalTopologyConnector3D> connector =
      domain.connect(target, 30.0);
  if (!connector.has_value()) {
    FAIL() << "fresh observed free-space extension must connect around the wall";
  }
  const IncrementalTopologyConnector3D& route = *connector;
  EXPECT_GT(route.polyline.size(), 2U);
  EXPECT_TRUE(std::ranges::all_of(
      std::views::iota(std::size_t{1U}, route.polyline.size()),
      [&](const std::size_t index) {
        return rawSweptFootprintIsNavigable(
            occupancy, route.polyline[index - 1U], FootprintBodyAxis{},
            route.polyline[index], FootprintBodyAxis{}, graph_config.footprint);
      }));
}

TEST(ReachableObservationDomain3DTest,
     KeepsUnknownTraversableButRawOccupiedHardUnderExplicitPolicies) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 20, 12, 7};
  ObservedOccupancyGrid3D graph_occupancy{bounds};
  fillBox(graph_occupancy, {1, 3, 2}, {5, 5, 4}, ObservedVoxelState::kFree);

  const IncrementalTopologyGraph3DConfig graph_config = makeGraphConfig();
  IncrementalTopologyGraph3D graph{graph_config};
  static_cast<void>(graph.update(graph_occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot stale_graph = graph.snapshot();
  ASSERT_FALSE(stale_graph.nodes().empty());
  const std::vector<IncrementalTopologyNodeId> reachable_nodes =
      allNodeIds(stale_graph);

  ObservedOccupancyGrid3D occupancy{bounds};
  fillBox(occupancy, {1, 3, 2}, {5, 5, 4}, ObservedVoxelState::kFree);
  fillBox(occupancy, {5, 4, 3}, {14, 4, 3}, ObservedVoxelState::kFree);
  SweptFootprintConfig footprint = graph_config.footprint;
  footprint.radius_m = 0.6;
  footprint.lower_extent_m = 0.6;
  footprint.upper_extent_m = 0.6;
  const Point3 target = occupancy.cellCenter({13, 4, 3});

  const ReachableObservationDomain3D permissive = buildReachableObservationDomain3D(
      stale_graph, occupancy, reachable_nodes,
      ReachableObservationDomain3DConfig{
          .maximum_fresh_extension_m = 30.0,
          .footprint = footprint,
          .validation_policy = ObservedSpaceValidationPolicy::kAllowUnknown,
      });
  EXPECT_TRUE(containsCandidateNear(permissive, occupancy, target));
  EXPECT_TRUE(permissive.connect(target, 30.0).has_value());

  const ReachableObservationDomain3D strict = buildReachableObservationDomain3D(
      stale_graph, occupancy, reachable_nodes,
      ReachableObservationDomain3DConfig{
          .maximum_fresh_extension_m = 30.0,
          .footprint = footprint,
          .validation_policy = ObservedSpaceValidationPolicy::kRequireKnownFree,
      });
  EXPECT_FALSE(containsCandidateNear(strict, occupancy, target));
  EXPECT_FALSE(strict.connect(target, 30.0).has_value());

  fillBox(occupancy, {9, 0, 0}, {9, 11, 6}, ObservedVoxelState::kOccupied);
  const ReachableObservationDomain3D blocked = buildReachableObservationDomain3D(
      stale_graph, occupancy, reachable_nodes,
      ReachableObservationDomain3DConfig{
          .maximum_fresh_extension_m = 30.0,
          .footprint = footprint,
          .validation_policy = ObservedSpaceValidationPolicy::kAllowUnknown,
      });
  EXPECT_FALSE(containsCandidateNear(blocked, occupancy, target));
  EXPECT_FALSE(blocked.connect(target, 30.0).has_value());
}

} // namespace
} // namespace drone_city_nav
