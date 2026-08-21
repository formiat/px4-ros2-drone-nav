#include "drone_city_nav/incremental_topological_navigation_3d.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

namespace drone_city_nav {
namespace {

void fillOccupied(ObservedOccupancyGrid3D& occupancy) {
  const GridBounds3D& bounds = occupancy.bounds();
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        ASSERT_TRUE(occupancy.setState({x, y, z}, ObservedVoxelState::kOccupied));
      }
    }
  }
}

void fillFreeBox(ObservedOccupancyGrid3D& occupancy, const int min_x, const int max_x,
                 const int min_y, const int max_y, const int min_z, const int max_z) {
  for (int z = min_z; z <= max_z; ++z) {
    for (int y = min_y; y <= max_y; ++y) {
      for (int x = min_x; x <= max_x; ++x) {
        ASSERT_TRUE(occupancy.setState({x, y, z}, ObservedVoxelState::kFree));
      }
    }
  }
}

TEST(IncrementalTopologicalNavigation3DTest,
     RetainsPrivateTraversalAndFrontierEvidenceAcrossDirtyUpdates) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 40, 24, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 3, 34, 9, 11, 5, 7);
  for (int z = 5; z <= 7; ++z) {
    for (int y = 9; y <= 11; ++y) {
      ASSERT_TRUE(occupancy.setState({35, y, z}, ObservedVoxelState::kUnknown));
    }
  }
  IncrementalTopologyGraph3DConfig graph_config;
  graph_config.tile_size_cells = 4;
  graph_config.sample_stride_cells = 1;
  graph_config.footprint = SweptFootprintConfig{.radius_m = 0.1,
                                                .lower_extent_m = 0.1,
                                                .upper_extent_m = 0.1,
                                                .perimeter_samples = 4,
                                                .radial_rings = 1,
                                                .axial_samples = 2,
                                                .sweep_step_m = 0.25};
  graph_config.observability.footprint = graph_config.footprint;
  graph_config.observability.minimum_supporting_rays = 1U;
  graph_config.observability.minimum_information_gain_voxels = 1U;
  graph_config.observability.minimum_known_free_ray_m = 0.0;
  IncrementalTopologicalPlanner3DConfig planner_config;
  planner_config.maximum_start_anchor_distance_m = 6.0;
  planner_config.maximum_goal_anchor_distance_m = 6.0;
  IncrementalTopologicalNavigation3D navigation{graph_config, planner_config};

  const IncrementalTopologicalWorldUpdate3D initial =
      navigation.updateObserved(occupancy, 1U, {}, true);
  const Point3 start{4.5, 10.5, 6.5};
  const Point3 middle{18.5, 10.5, 6.5};
  const Point3 goal{39.0, 10.5, 6.5};
  const IncrementalTopologicalPlan3D first =
      navigation.plan(initial.snapshot, start, goal);
  ASSERT_TRUE(first.executableTargetSelected());
  ASSERT_FALSE(first.route_nodes.empty());
  EXPECT_TRUE(navigation.commitAcceptedPlan(first).accepted);
  const IncrementalTopologicalNavigationObservation3D first_observation =
      navigation.observePosition(initial.snapshot, start);
  const IncrementalTopologicalNavigationObservation3D second_observation =
      navigation.observePosition(initial.snapshot, middle);
  EXPECT_TRUE(first_observation.current_node.has_value());
  EXPECT_TRUE(second_observation.current_node.has_value());
  EXPECT_GT(second_observation.traversed_edges, 0U);

  ASSERT_TRUE(occupancy.setState({2, 2, 2}, ObservedVoxelState::kFree));
  const IncrementalTopologicalWorldUpdate3D updated = navigation.updateObserved(
      occupancy, 2U, std::array{OccupancyChunkIndex3D{0, 0, 0}}, false);
  const IncrementalTopologicalPlan3D second =
      navigation.plan(updated.snapshot, start, goal);

  EXPECT_TRUE(second.executableTargetSelected());
  EXPECT_GT(second.directed_traversal_count, 0U);
  EXPECT_GT(second.repeated_edge_distance_m, 0.0);
}

TEST(IncrementalTopologicalNavigation3DTest,
     StaticResetUsesTheSameGraphAndPlanningContract) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 32, 16, 12}};
  IncrementalTopologyGraph3DConfig graph_config;
  graph_config.tile_size_cells = 4;
  graph_config.sample_stride_cells = 1;
  graph_config.footprint = SweptFootprintConfig{.radius_m = 0.1,
                                                .lower_extent_m = 0.1,
                                                .upper_extent_m = 0.1,
                                                .perimeter_samples = 4,
                                                .radial_rings = 1,
                                                .axial_samples = 2,
                                                .sweep_step_m = 0.25};
  graph_config.observability.footprint = graph_config.footprint;
  IncrementalTopologicalNavigation3D navigation{graph_config};

  const IncrementalTopologicalWorldUpdate3D world =
      navigation.resetStatic(occupancy, 9U);
  const IncrementalTopologicalPlan3D plan =
      navigation.plan(world.snapshot, {3.5, 7.5, 5.5}, {28.5, 7.5, 5.5});

  EXPECT_EQ(world.graph.revision, 9U);
  EXPECT_GT(world.graph.node_count, 0U);
  EXPECT_EQ(plan.status, IncrementalTopologicalPlanStatus3D::kMissionRoute);
  EXPECT_TRUE(plan.reaches_mission_goal);
}

} // namespace
} // namespace drone_city_nav
