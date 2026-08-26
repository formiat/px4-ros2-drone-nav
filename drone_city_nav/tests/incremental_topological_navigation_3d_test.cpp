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

[[nodiscard]] SensorObservabilityConfig
observabilityFor(const IncrementalTopologyGraph3DConfig& graph_config) {
  SensorObservabilityConfig result;
  result.footprint = graph_config.footprint;
  result.minimum_supporting_rays = 1U;
  result.minimum_information_gain_voxels = 1U;
  result.minimum_known_free_ray_m = 0.0;
  return result;
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
  graph_config.block_size_cells = 4;
  graph_config.coarse_sample_stride_cells = 1;
  graph_config.refined_sample_stride_cells = 1;
  graph_config.maximum_observed_blocks_per_update = 4096U;
  graph_config.footprint = SweptFootprintConfig{.radius_m = 0.1,
                                                .lower_extent_m = 0.1,
                                                .upper_extent_m = 0.1,
                                                .perimeter_samples = 4,
                                                .radial_rings = 1,
                                                .axial_samples = 2,
                                                .sweep_step_m = 0.25};
  IncrementalTopologicalPlanner3DConfig planner_config;
  planner_config.maximum_start_anchor_distance_m = 6.0;
  planner_config.maximum_goal_anchor_distance_m = 6.0;
  IncrementalTopologicalNavigation3D navigation{
      graph_config, planner_config, {}, observabilityFor(graph_config)};

  const IncrementalTopologicalWorldUpdate3D initial =
      navigation.updateObserved(occupancy, 17U, 1U, {}, true);
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
      occupancy, 17U, 2U, std::array{OccupancyChunkIndex3D{0, 0, 0}}, false);
  const IncrementalTopologicalPlan3D second =
      navigation.plan(updated.snapshot, start, goal);

  EXPECT_TRUE(second.executableTargetSelected());
  EXPECT_GT(second.directed_traversal_count, 0U);
  EXPECT_GT(second.repeated_edge_distance_m, 0.0);
}

TEST(IncrementalTopologicalNavigation3DTest,
     RejectsLocalOccupancyForAWorldScaleTopologySnapshot) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 40, 24, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 3, 34, 9, 11, 5, 7);
  IncrementalTopologyGraph3DConfig graph_config;
  graph_config.block_size_cells = 4;
  graph_config.coarse_sample_stride_cells = 1;
  graph_config.refined_sample_stride_cells = 1;
  graph_config.maximum_observed_blocks_per_update = 4096U;
  graph_config.footprint = SweptFootprintConfig{.radius_m = 0.1,
                                                .lower_extent_m = 0.1,
                                                .upper_extent_m = 0.1,
                                                .perimeter_samples = 4,
                                                .radial_rings = 1,
                                                .axial_samples = 2,
                                                .sweep_step_m = 0.25};
  IncrementalTopologicalNavigation3D navigation{
      graph_config, {}, {}, observabilityFor(graph_config)};
  const IncrementalTopologicalWorldUpdate3D world =
      navigation.updateObserved(occupancy, 17U, 1U, {}, true);
  const ObservedOccupancyGrid3D local =
      occupancy.crop(GridBounds3D{0.0, 0.0, 0.0, 1.0, 20, 24, 16});

  const IncrementalTopologicalPlan3D plan = navigation.planObserved(
      world.snapshot, local, {4.5, 10.5, 6.5}, {32.5, 10.5, 6.5});

  EXPECT_EQ(plan.status, IncrementalTopologicalPlanStatus3D::kInvalidInput);
}

TEST(IncrementalTopologicalNavigation3DTest,
     ReusesAnObservedRawValidatedStartAnchorForPlanning) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 40, 24, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 3, 34, 9, 11, 5, 7);
  IncrementalTopologyGraph3DConfig graph_config;
  graph_config.block_size_cells = 4;
  graph_config.coarse_sample_stride_cells = 1;
  graph_config.refined_sample_stride_cells = 1;
  graph_config.maximum_observed_blocks_per_update = 4096U;
  graph_config.footprint = SweptFootprintConfig{.radius_m = 0.1,
                                                .lower_extent_m = 0.1,
                                                .upper_extent_m = 0.1,
                                                .perimeter_samples = 4,
                                                .radial_rings = 1,
                                                .axial_samples = 2,
                                                .sweep_step_m = 0.25};
  IncrementalTopologicalNavigation3D navigation{
      graph_config, {}, {}, observabilityFor(graph_config)};
  const IncrementalTopologicalWorldUpdate3D world =
      navigation.updateObserved(occupancy, 17U, 1U, {}, true);
  const Point3 start{4.25, 10.25, 6.25};
  const Point3 goal{32.5, 10.5, 6.5};

  const IncrementalTopologicalNavigationObservation3D observation =
      navigation.observePosition(world.snapshot, start, &occupancy);
  if (!observation.current_anchor.has_value()) {
    FAIL() << "observed position must retain its validated graph anchor";
  }
  const IncrementalTopologyConnector3D& anchor = observation.current_anchor.value();
  ASSERT_FALSE(anchor.polyline.empty());
  EXPECT_NEAR(distance3D(anchor.polyline.front(), start), 0.0, 1.0e-9);

  const IncrementalTopologicalPlan3D plan = navigation.planObserved(
      world.snapshot, occupancy, start, goal, std::nullopt, &observation);

  EXPECT_TRUE(plan.executableTargetSelected());
  EXPECT_EQ(plan.start_node, anchor.node);
}

TEST(IncrementalTopologicalNavigation3DTest,
     RepeatedPlanCommitDoesNotDuplicateSelectionOrDeadEndEvidence) {
  IncrementalTopologicalNavigation3D navigation;
  IncrementalTopologicalPlan3D frontier_plan;
  frontier_plan.status = IncrementalTopologicalPlanStatus3D::kFrontierRoute;
  frontier_plan.purpose = IncrementalTopologicalRoutePurpose3D::kObservationFrontier;
  frontier_plan.planned_on_revision = 7U;
  frontier_plan.selected_frontier =
      ObservationFrontier{.id = ObservationFrontierId{23U}};

  const IncrementalTopologicalPlanCommit3D first_frontier_commit =
      navigation.commitAcceptedPlan(frontier_plan);
  const IncrementalTopologicalPlanCommit3D repeated_frontier_commit =
      navigation.commitAcceptedPlan(frontier_plan);
  EXPECT_TRUE(first_frontier_commit.accepted);
  EXPECT_FALSE(first_frontier_commit.replaced_frontier_coverage_recorded);
  EXPECT_TRUE(first_frontier_commit.frontier_selection_recorded);
  EXPECT_TRUE(repeated_frontier_commit.accepted);
  EXPECT_FALSE(repeated_frontier_commit.replaced_frontier_coverage_recorded);
  EXPECT_FALSE(repeated_frontier_commit.frontier_selection_recorded);

  navigation.rejectObservationFrontier(frontier_plan.selected_frontier->id);
  const IncrementalTopologicalPlanCommit3D rejected_frontier_retry =
      navigation.commitAcceptedPlan(frontier_plan);
  EXPECT_TRUE(rejected_frontier_retry.accepted);
  EXPECT_TRUE(rejected_frontier_retry.frontier_selection_recorded);

  navigation.completeObservationFrontier(*frontier_plan.selected_frontier, 42U);
  const IncrementalTopologicalPlanCommit3D completed_frontier_retry =
      navigation.commitAcceptedPlan(frontier_plan);
  EXPECT_TRUE(completed_frontier_retry.accepted);
  EXPECT_TRUE(completed_frontier_retry.frontier_selection_recorded);

  frontier_plan.selected_frontier->observation_pose = {3.0, 0.0, 0.0};
  const IncrementalTopologicalPlanCommit3D advanced_frontier_commit =
      navigation.commitAcceptedPlan(frontier_plan);
  EXPECT_TRUE(advanced_frontier_commit.accepted);
  EXPECT_TRUE(advanced_frontier_commit.replaced_frontier_coverage_recorded);
  EXPECT_TRUE(advanced_frontier_commit.frontier_selection_recorded);

  IncrementalTopologicalPlan3D backtrack_plan;
  backtrack_plan.status = IncrementalTopologicalPlanStatus3D::kBacktrackRoute;
  backtrack_plan.purpose = IncrementalTopologicalRoutePurpose3D::kTopologicalBacktrack;
  backtrack_plan.planned_on_revision = 7U;
  backtrack_plan.dead_end_conclusion = TopologicalDeadEndConclusion3D{
      .attempted_direction =
          DirectedTopologyEdge3D{
              .edge_id = IncrementalTopologyEdgeId{41U},
              .from = IncrementalTopologyNodeId{5U},
              .to = IncrementalTopologyNodeId{6U},
          },
      .validated_through_revision = 7U,
  };

  const IncrementalTopologicalPlanCommit3D first_dead_end_commit =
      navigation.commitAcceptedPlan(backtrack_plan);
  const IncrementalTopologicalPlanCommit3D repeated_dead_end_commit =
      navigation.commitAcceptedPlan(backtrack_plan);
  EXPECT_TRUE(first_dead_end_commit.accepted);
  EXPECT_TRUE(first_dead_end_commit.dead_end_recorded);
  EXPECT_TRUE(repeated_dead_end_commit.accepted);
  EXPECT_FALSE(repeated_dead_end_commit.dead_end_recorded);
}

TEST(IncrementalTopologicalNavigation3DTest,
     NewMissionLegDropsActivePlanWithoutResettingGraph) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 32, 16, 12}};
  IncrementalTopologyGraph3DConfig graph_config;
  graph_config.block_size_cells = 4;
  graph_config.coarse_sample_stride_cells = 1;
  graph_config.refined_sample_stride_cells = 1;
  graph_config.maximum_observed_blocks_per_update = 4096U;
  graph_config.footprint = SweptFootprintConfig{.radius_m = 0.1,
                                                .lower_extent_m = 0.1,
                                                .upper_extent_m = 0.1,
                                                .perimeter_samples = 4,
                                                .radial_rings = 1,
                                                .axial_samples = 2,
                                                .sweep_step_m = 0.25};
  IncrementalTopologicalNavigation3D navigation{
      graph_config, {}, {}, observabilityFor(graph_config)};
  const IncrementalTopologicalWorldUpdate3D world =
      navigation.resetStatic(occupancy, 9U);
  const IncrementalTopologicalPlan3D first =
      navigation.plan(world.snapshot, {3.5, 7.5, 5.5}, {28.5, 7.5, 5.5});
  ASSERT_TRUE(navigation.commitAcceptedPlan(first).accepted);

  navigation.beginMissionLeg();

  const auto retained_graph = navigation.snapshot();
  ASSERT_NE(retained_graph, nullptr);
  EXPECT_EQ(retained_graph->revision(), 9U);
  EXPECT_EQ(retained_graph->nodes().size(), world.snapshot->nodes().size());
  const IncrementalTopologicalPlan3D return_plan =
      navigation.plan(retained_graph, {28.5, 7.5, 5.5}, {3.5, 7.5, 5.5});
  EXPECT_EQ(return_plan.status, IncrementalTopologicalPlanStatus3D::kMissionRoute);
  EXPECT_TRUE(return_plan.reaches_mission_goal);
}

TEST(IncrementalTopologicalNavigation3DTest,
     DoesNotInferTraversalFromDiscoveredGraphConnectivity) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 24, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 3, 44, 9, 11, 5, 7);
  IncrementalTopologyGraph3DConfig graph_config;
  graph_config.block_size_cells = 4;
  graph_config.coarse_sample_stride_cells = 1;
  graph_config.refined_sample_stride_cells = 1;
  graph_config.maximum_observed_blocks_per_update = 4096U;
  graph_config.footprint = SweptFootprintConfig{.radius_m = 0.1,
                                                .lower_extent_m = 0.1,
                                                .upper_extent_m = 0.1,
                                                .perimeter_samples = 4,
                                                .radial_rings = 1,
                                                .axial_samples = 2,
                                                .sweep_step_m = 0.25};
  IncrementalTopologicalNavigation3D navigation{
      graph_config, {}, {}, observabilityFor(graph_config)};
  const IncrementalTopologicalWorldUpdate3D world =
      navigation.updateObserved(occupancy, 17U, 1U, {}, true);

  const IncrementalTopologicalNavigationObservation3D first =
      navigation.observePosition(world.snapshot, {4.5, 10.5, 6.5});
  const IncrementalTopologicalNavigationObservation3D second =
      navigation.observePosition(world.snapshot, {42.5, 10.5, 6.5});

  ASSERT_TRUE(first.current_node.has_value());
  ASSERT_TRUE(second.current_node.has_value());
  EXPECT_NE(first.current_node, second.current_node);
  EXPECT_EQ(second.traversed_edges, 0U);
  EXPECT_TRUE(second.trail_reset);
}

TEST(IncrementalTopologicalNavigation3DTest,
     RecordsShortObservedMultiEdgeTransitionWithoutResettingTrail) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 12, 8}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 21, 4, 6, 2, 4);
  IncrementalTopologyGraph3DConfig graph_config;
  graph_config.block_size_cells = 2;
  graph_config.coarse_sample_stride_cells = 1;
  graph_config.refined_sample_stride_cells = 1;
  graph_config.maximum_observed_blocks_per_update = 4096U;
  graph_config.footprint = SweptFootprintConfig{.radius_m = 0.1,
                                                .lower_extent_m = 0.1,
                                                .upper_extent_m = 0.1,
                                                .perimeter_samples = 4,
                                                .radial_rings = 1,
                                                .axial_samples = 2,
                                                .sweep_step_m = 0.25};
  IncrementalTopologicalPlanner3DConfig planner_config;
  planner_config.maximum_start_anchor_distance_m = 3.0;
  TopologicalExplorationMemory3DConfig memory_config;
  memory_config.maximum_observed_transition_m = 4.0;
  IncrementalTopologicalNavigation3D navigation{
      graph_config, planner_config, memory_config, observabilityFor(graph_config)};
  const IncrementalTopologicalWorldUpdate3D world =
      navigation.updateObserved(occupancy, 19U, 1U, {}, true);

  const IncrementalTopologicalNavigationObservation3D first =
      navigation.observePosition(world.snapshot, {3.5, 5.5, 3.5});
  const IncrementalTopologicalNavigationObservation3D second =
      navigation.observePosition(world.snapshot, {6.5, 5.5, 3.5});

  ASSERT_TRUE(first.current_node.has_value());
  ASSERT_TRUE(second.current_node.has_value());
  EXPECT_NE(first.current_node, second.current_node);
  EXPECT_GE(second.traversed_edges, 2U);
  EXPECT_FALSE(second.trail_reset);
}

TEST(IncrementalTopologicalNavigation3DTest,
     StaticResetUsesTheSameGraphAndPlanningContract) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 32, 16, 12}};
  IncrementalTopologyGraph3DConfig graph_config;
  graph_config.block_size_cells = 4;
  graph_config.coarse_sample_stride_cells = 1;
  graph_config.refined_sample_stride_cells = 1;
  graph_config.footprint = SweptFootprintConfig{.radius_m = 0.1,
                                                .lower_extent_m = 0.1,
                                                .upper_extent_m = 0.1,
                                                .perimeter_samples = 4,
                                                .radial_rings = 1,
                                                .axial_samples = 2,
                                                .sweep_step_m = 0.25};
  IncrementalTopologicalNavigation3D navigation{
      graph_config, {}, {}, observabilityFor(graph_config)};

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
