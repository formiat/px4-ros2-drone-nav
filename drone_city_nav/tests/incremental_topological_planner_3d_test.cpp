#include "drone_city_nav/incremental_topological_planner_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] IncrementalTopologyGraph3DConfig graphConfig() {
  IncrementalTopologyGraph3DConfig config;
  config.tile_size_cells = 4;
  config.sample_stride_cells = 1;
  config.maximum_frontier_evaluations_per_component = 256U;
  config.footprint.radius_m = 0.2;
  config.footprint.lower_extent_m = 0.2;
  config.footprint.upper_extent_m = 0.2;
  config.footprint.perimeter_samples = 8U;
  config.footprint.radial_rings = 1U;
  config.footprint.axial_samples = 2U;
  config.footprint.sweep_step_m = 0.25;
  config.observability.maximum_observation_range_m = 6.0;
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

[[nodiscard]] IncrementalTopologyGraph3DSnapshot
buildGraph(const ObservedOccupancyGrid3D& occupancy,
           const std::uint64_t revision = 1U) {
  IncrementalTopologyGraph3D graph{graphConfig()};
  static_cast<void>(graph.update(occupancy, revision, {}, true));
  return graph.snapshot();
}

TEST(IncrementalTopologicalPlanner3DTest, KnownMissionRouteHasPriority) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 24, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 45, 9, 11, 5, 7);
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy);
  IncrementalTopologicalPlanner3D planner;
  TopologicalExplorationMemory3D memory;

  const IncrementalTopologicalPlan3D plan =
      planner.plan(graph, {3.5, 10.5, 6.5}, {44.5, 10.5, 6.5}, memory);

  EXPECT_EQ(plan.status, IncrementalTopologicalPlanStatus3D::kMissionRoute);
  EXPECT_EQ(plan.purpose, IncrementalTopologicalRoutePurpose3D::kMissionTransit);
  EXPECT_TRUE(plan.reaches_mission_goal);
  ASSERT_FALSE(plan.route_steps.empty());
  EXPECT_GT(plan.guidance_points.size(), 2U);
}

TEST(IncrementalTopologicalPlanner3DTest,
     SelectsSafeFrontierEvenWhenInitialMotionMovesAwayFromGoal) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 32, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 4, 30, 13, 15, 5, 7);
  fillStateBox(occupancy, 0, 3, 13, 15, 5, 7, ObservedVoxelState::kUnknown);
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy);
  IncrementalTopologicalPlanner3D planner;
  TopologicalExplorationMemory3D memory;
  const Point3 start{28.5, 14.5, 6.5};
  const Point3 goal{44.5, 14.5, 6.5};

  const IncrementalTopologicalPlan3D plan = planner.plan(graph, start, goal, memory);

  ASSERT_EQ(plan.status, IncrementalTopologicalPlanStatus3D::kFrontierRoute);
  if (!plan.selected_frontier.has_value()) {
    FAIL() << "frontier route must identify its observation frontier";
  }
  const ObservationFrontier& selected_frontier = *plan.selected_frontier;
  EXPECT_LT(selected_frontier.observation_pose.x, start.x);
  EXPECT_LT(plan.goal_progress_m, 0.0);
  EXPECT_TRUE(plan.executableTargetSelected());
}

TEST(IncrementalTopologicalPlanner3DTest, FairnessMovesSelectionToOtherTBranch) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 48, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 6, 41, 21, 23, 5, 7);
  fillFreeBox(occupancy, 21, 23, 21, 43, 5, 7);
  fillStateBox(occupancy, 0, 5, 21, 23, 5, 7, ObservedVoxelState::kUnknown);
  fillStateBox(occupancy, 42, 47, 21, 23, 5, 7, ObservedVoxelState::kUnknown);
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy);
  IncrementalTopologicalPlanner3DConfig config;
  config.frontier_selection_penalty = 100.0;
  IncrementalTopologicalPlanner3D planner{config};
  TopologicalExplorationMemory3D memory;
  const Point3 start{22.5, 41.5, 6.5};
  const Point3 goal{22.5, 100.0, 6.5};

  const IncrementalTopologicalPlan3D first = planner.plan(graph, start, goal, memory);
  if (!first.selected_frontier.has_value()) {
    FAIL() << "first plan must select a frontier";
  }
  const ObservationFrontier& first_frontier = *first.selected_frontier;
  for (std::size_t count = 0U; count < 4U; ++count) {
    memory.recordFrontierSelection(first_frontier.id);
  }
  const IncrementalTopologicalPlan3D second = planner.plan(graph, start, goal, memory);

  if (!second.selected_frontier.has_value()) {
    FAIL() << "selection history must leave another reachable frontier";
  }
  EXPECT_NE(second.selected_frontier->id, first_frontier.id);
}

TEST(IncrementalTopologicalPlanner3DTest,
     ClosedTerminalProducesOrdinaryBacktrackAndRevisionedConclusion) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 24, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 4, 15, 9, 11, 5, 7);
  fillFreeBox(occupancy, 9, 11, 4, 11, 5, 7);
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy, 7U);
  const IncrementalTopologyNode3D* junction = nullptr;
  const IncrementalTopologyNode3D* terminal = nullptr;
  const IncrementalTopologyEdge3D* branch = nullptr;
  for (const IncrementalTopologyEdge3D& edge : graph.edges()) {
    const IncrementalTopologyNode3D* first = graph.findNode(edge.first);
    const IncrementalTopologyNode3D* second = graph.findNode(edge.second);
    if (first != nullptr && second != nullptr && first->traits.junction &&
        second->traits.terminal) {
      junction = first;
      terminal = second;
      branch = &edge;
      break;
    }
    if (first != nullptr && second != nullptr && second->traits.junction &&
        first->traits.terminal) {
      junction = second;
      terminal = first;
      branch = &edge;
      break;
    }
  }
  ASSERT_NE(junction, nullptr);
  ASSERT_NE(terminal, nullptr);
  ASSERT_NE(branch, nullptr);
  TopologicalExplorationMemory3D memory;
  memory.resetTrail(junction->id);
  memory.recordTrailTransition(junction->id, terminal->id);
  IncrementalTopologicalPlanner3D planner;

  const IncrementalTopologicalPlan3D plan =
      planner.plan(graph, terminal->representative, {100.0, 100.0, 6.5}, memory);

  ASSERT_EQ(plan.status, IncrementalTopologicalPlanStatus3D::kBacktrackRoute);
  EXPECT_EQ(plan.target_node, junction->id);
  EXPECT_EQ(plan.backtrack_reason, TopologicalBacktrackReason3D::kConfirmedTerminal);
  if (!plan.dead_end_conclusion.has_value()) {
    FAIL() << "terminal backtrack must record revisioned dead-end evidence";
  }
  const TopologicalDeadEndConclusion3D& conclusion = *plan.dead_end_conclusion;
  EXPECT_EQ(conclusion.attempted_direction.edge_id, branch->id);
  EXPECT_EQ(conclusion.attempted_direction.from, junction->id);
  EXPECT_EQ(conclusion.attempted_direction.to, terminal->id);
  EXPECT_EQ(conclusion.supporting_revision, 7U);
}

TEST(IncrementalTopologicalPlanner3DTest, RoutesThroughVerticalConnector) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 24, 32}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 22, 9, 11, 3, 5);
  fillFreeBox(occupancy, 20, 22, 9, 11, 3, 24);
  fillFreeBox(occupancy, 20, 45, 9, 11, 22, 24);
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy);
  IncrementalTopologicalPlanner3D planner;
  TopologicalExplorationMemory3D memory;

  const IncrementalTopologicalPlan3D plan =
      planner.plan(graph, {3.5, 10.5, 4.5}, {44.5, 10.5, 23.5}, memory);

  ASSERT_EQ(plan.status, IncrementalTopologicalPlanStatus3D::kMissionRoute);
  EXPECT_TRUE(std::ranges::any_of(plan.route_nodes, [&graph](const auto node_id) {
    const IncrementalTopologyNode3D* node = graph.findNode(node_id);
    return node != nullptr && node->traits.vertical_connector;
  }));
}

} // namespace
} // namespace drone_city_nav
