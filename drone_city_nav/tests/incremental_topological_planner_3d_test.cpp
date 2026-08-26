#include "drone_city_nav/incremental_topological_planner_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] IncrementalTopologyGraph3DConfig graphConfig() {
  IncrementalTopologyGraph3DConfig config;
  config.block_size_cells = 4;
  config.coarse_sample_stride_cells = 1;
  config.refined_sample_stride_cells = 1;
  config.maximum_observed_blocks_per_update = 4096U;
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

[[nodiscard]] SensorObservabilityConfig
observabilityConfig(const SweptFootprintConfig& footprint) {
  SensorObservabilityConfig config;
  config.footprint = footprint;
  config.maximum_observation_range_m = 6.0;
  config.minimum_known_free_ray_m = 1.0;
  config.minimum_supporting_rays = 1U;
  config.minimum_information_gain_voxels = 1U;
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

void carveRotatedVariableWidthTunnel(ObservedOccupancyGrid3D& occupancy,
                                     const Point3& first, const Point3& second,
                                     const double minimum_half_width_m,
                                     const double maximum_half_width_m,
                                     const double vertical_half_extent_m) {
  const Vec3 axis{second.x - first.x, second.y - first.y, 0.0};
  const double squared_length = axis.x * axis.x + axis.y * axis.y;
  ASSERT_GT(squared_length, 0.0);
  const GridBounds3D& bounds = occupancy.bounds();
  for (int z = 0; z < bounds.depth_cells; ++z) {
    for (int y = 0; y < bounds.height_cells; ++y) {
      for (int x = 0; x < bounds.width_cells; ++x) {
        const GridIndex3D cell{x, y, z};
        const Point3 center = occupancy.cellCenter(cell);
        const double projection =
            std::clamp(((center.x - first.x) * axis.x + (center.y - first.y) * axis.y) /
                           squared_length,
                       0.0, 1.0);
        const Point3 closest{first.x + projection * axis.x,
                             first.y + projection * axis.y, first.z};
        const double end_distance = std::abs(2.0 * projection - 1.0);
        const double half_width =
            std::lerp(minimum_half_width_m, maximum_half_width_m, end_distance);
        if (std::hypot(center.x - closest.x, center.y - closest.y) <= half_width &&
            std::abs(center.z - closest.z) <= vertical_half_extent_m) {
          ASSERT_TRUE(occupancy.setState(cell, ObservedVoxelState::kFree));
        }
      }
    }
  }
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
  EXPECT_FALSE(plan.unknown_exposure);
  EXPECT_GT(plan.transition_support_segment_count, 0U);
  EXPECT_EQ(plan.validated_through_revision, 1U);
  EXPECT_EQ(plan.complete_through_revision, 1U);
  EXPECT_NE(plan.topology_lineage_id, 0U);
  EXPECT_TRUE(std::ranges::all_of(plan.route_steps, [](const auto& step) {
    return step.evidence.kind == IncrementalTopologyTransitionKind3D::kObservedFree &&
           !step.evidence.unknown_exposure &&
           step.evidence.support_segment_count > 0U && step.evidence.lineage_id != 0U;
  }));
}

TEST(IncrementalTopologicalPlanner3DTest,
     ExpiredDeadlineCannotProduceAnExecutableTopologyRoute) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 24, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 45, 9, 11, 5, 7);
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy);
  IncrementalTopologicalPlanner3D planner;
  TopologicalExplorationMemory3D memory;

  const IncrementalTopologicalPlan3D plan =
      planner.plan(graph, {3.5, 10.5, 6.5}, {44.5, 10.5, 6.5}, memory,
                   std::chrono::steady_clock::now() - std::chrono::milliseconds{1});

  EXPECT_EQ(plan.status, IncrementalTopologicalPlanStatus3D::kDeadlineExceeded);
  EXPECT_FALSE(plan.executableTargetSelected());
  EXPECT_TRUE(plan.guidance_points.empty());
}

TEST(IncrementalTopologicalPlanner3DTest,
     RejectsNegativeFreshFrontierMaterializationReserve) {
  IncrementalTopologicalPlanner3DConfig config;
  config.fresh_frontier_materialization_reserve_ms = -1.0;

  EXPECT_THROW(static_cast<void>(IncrementalTopologicalPlanner3D{config}),
               std::invalid_argument);
}

TEST(IncrementalTopologicalPlanner3DTest,
     KnownConnectivityContinuesTowardAnUnanchoredMissionGoal) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 64, 40, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 7, 9, 7, 24, 5, 7);
  fillFreeBox(occupancy, 7, 40, 7, 9, 5, 7);
  fillFreeBox(occupancy, 38, 40, 7, 24, 5, 7);
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy);
  IncrementalTopologicalPlanner3D planner;
  TopologicalExplorationMemory3D memory;
  const Point3 start{8.5, 23.5, 6.5};
  const Point3 goal{57.5, 23.5, 6.5};

  const IncrementalTopologicalPlan3D plan = planner.plan(graph, start, goal, memory);

  ASSERT_EQ(plan.status, IncrementalTopologicalPlanStatus3D::kMissionContinuationRoute);
  EXPECT_EQ(plan.purpose, IncrementalTopologicalRoutePurpose3D::kMissionTransit);
  EXPECT_FALSE(plan.reaches_mission_goal);
  EXPECT_FALSE(plan.goal_node.has_value());
  EXPECT_FALSE(isExplicitTopologicalBacktrack3D(plan));
  EXPECT_GT(plan.reachable_mission_continuation_count, 0U);
  EXPECT_GT(plan.maximum_reachable_mission_continuation_goal_progress_m, 0.0);
  EXPECT_GT(plan.goal_progress_m, 0.0);
  ASSERT_FALSE(plan.guidance_points.empty());
  EXPECT_LT(distance3D(plan.guidance_points.back(), goal), distance3D(start, goal));
  EXPECT_TRUE(std::ranges::any_of(plan.guidance_points,
                                  [](const Point3& point) { return point.y < 12.0; }));
}

TEST(IncrementalTopologicalPlanner3DTest,
     KnownConnectivityDoesNotRelabelNegativeProgressAsMissionTransit) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 24, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 4, 20, 9, 11, 5, 7);
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy);
  IncrementalTopologicalPlanner3D planner;
  TopologicalExplorationMemory3D memory;

  const IncrementalTopologicalPlan3D plan =
      planner.plan(graph, {20.5, 10.5, 6.5}, {44.5, 10.5, 6.5}, memory);

  EXPECT_EQ(plan.status, IncrementalTopologicalPlanStatus3D::kNoRoute);
  EXPECT_EQ(plan.reachable_mission_continuation_count, 0U);
  EXPECT_DOUBLE_EQ(plan.maximum_reachable_mission_continuation_goal_progress_m, 0.0);
  EXPECT_FALSE(plan.executableTargetSelected());
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

  const IncrementalTopologicalPlan3D plan = planner.planObserved(
      graph, occupancy, observabilityConfig(graphConfig().footprint), start, goal,
      memory);

  ASSERT_EQ(plan.status, IncrementalTopologicalPlanStatus3D::kFrontierRoute);
  if (!plan.selected_frontier.has_value()) {
    FAIL() << "frontier route must identify its observation frontier";
  }
  const ObservationFrontier& selected_frontier = *plan.selected_frontier;
  EXPECT_LT(selected_frontier.observation_pose.x, start.x);
  EXPECT_LT(plan.goal_progress_m, 0.0);
  EXPECT_TRUE(plan.executableTargetSelected());
  EXPECT_FALSE(isExplicitTopologicalBacktrack3D(plan));
  ASSERT_GE(plan.guidance_points.size(), 2U);
  EXPECT_TRUE(std::ranges::all_of(
      std::views::iota(std::size_t{1U}, plan.guidance_points.size()),
      [&](const std::size_t index) {
        return validateObservedSweptFootprint(
                   occupancy, plan.guidance_points[index - 1U], FootprintBodyAxis{},
                   plan.guidance_points[index], FootprintBodyAxis{},
                   graphConfig().footprint,
                   ObservedSpaceValidationPolicy::kAllowUnknown)
            .accepted();
      }));
}

TEST(IncrementalTopologicalPlanner3DTest,
     OnlyTypedBacktrackPlansRequireTheBacktrackingRolloutGate) {
  IncrementalTopologicalPlan3D frontier_plan;
  frontier_plan.status = IncrementalTopologicalPlanStatus3D::kFrontierRoute;
  frontier_plan.purpose = IncrementalTopologicalRoutePurpose3D::kObservationFrontier;
  frontier_plan.goal_progress_m = -20.0;

  IncrementalTopologicalPlan3D backtrack_plan;
  backtrack_plan.status = IncrementalTopologicalPlanStatus3D::kBacktrackRoute;
  backtrack_plan.purpose = IncrementalTopologicalRoutePurpose3D::kTopologicalBacktrack;
  backtrack_plan.goal_progress_m = 5.0;

  EXPECT_FALSE(isExplicitTopologicalBacktrack3D(frontier_plan));
  EXPECT_TRUE(isExplicitTopologicalBacktrack3D(backtrack_plan));
}

TEST(IncrementalTopologicalPlanner3DTest,
     ObservedPlanRetiresFrontierInvalidatedByFreshOccupancy) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 32, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 4, 30, 13, 15, 5, 7);
  fillStateBox(occupancy, 0, 3, 13, 15, 5, 7, ObservedVoxelState::kUnknown);
  const IncrementalTopologyGraph3DConfig config = graphConfig();
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy);
  IncrementalTopologicalPlanner3D planner;
  TopologicalExplorationMemory3D memory;

  const IncrementalTopologicalPlan3D stale =
      planner.planObserved(graph, occupancy, observabilityConfig(config.footprint),
                           {28.5, 14.5, 6.5}, {44.5, 14.5, 6.5}, memory);
  ASSERT_TRUE(stale.selected_frontier.has_value());

  fillStateBox(occupancy, 0, 3, 13, 15, 5, 7, ObservedVoxelState::kOccupied);
  const IncrementalTopologicalPlan3D current = planner.planObserved(
      graph, occupancy, observabilityConfig(config.footprint), {28.5, 14.5, 6.5},
      {44.5, 14.5, 6.5}, memory, stale.selected_frontier);

  EXPECT_FALSE(current.selected_frontier.has_value());
  EXPECT_EQ(current.fresh_frontier_discovered_count, 0U);
}

TEST(IncrementalTopologicalPlanner3DTest,
     MissionContinuationRejectsAMicroStepOnAStaleTopologySnapshot) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 56, 32, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 4, 20, 13, 15, 5, 7);
  fillStateBox(occupancy, 21, 55, 13, 15, 5, 7, ObservedVoxelState::kUnknown);
  const IncrementalTopologyGraph3DConfig graph_config = graphConfig();
  const IncrementalTopologyGraph3DSnapshot stale_graph = buildGraph(occupancy);
  IncrementalTopologicalPlanner3DConfig planner_config;
  planner_config.maximum_fresh_frontier_evaluations = 256U;
  IncrementalTopologicalPlanner3D planner{planner_config};
  TopologicalExplorationMemory3D memory;

  fillFreeBox(occupancy, 21, 32, 13, 15, 5, 7);
  const IncrementalTopologicalPlan3D stale_plan = planner.planObserved(
      stale_graph, occupancy, observabilityConfig(graph_config.footprint),
      {18.5, 14.5, 6.5}, {52.5, 14.5, 6.5}, memory);
  EXPECT_EQ(stale_plan.status, IncrementalTopologicalPlanStatus3D::kNoRoute);
  EXPECT_EQ(stale_plan.reachable_mission_continuation_count, 0U);

  const IncrementalTopologyGraph3DSnapshot refreshed_graph = buildGraph(occupancy, 2U);
  const IncrementalTopologicalPlan3D plan = planner.planObserved(
      refreshed_graph, occupancy, observabilityConfig(graph_config.footprint),
      {18.5, 14.5, 6.5}, {52.5, 14.5, 6.5}, memory);

  ASSERT_EQ(plan.status, IncrementalTopologicalPlanStatus3D::kFrontierRoute)
      << "reachable=" << plan.reachable_frontier_count
      << " fresh_candidates=" << plan.fresh_frontier_candidate_count
      << " fresh_evaluated=" << plan.fresh_frontier_evaluated_count
      << " fresh_discovered=" << plan.fresh_frontier_discovered_count
      << " fresh_budget_exhausted=" << plan.fresh_frontier_budget_exhausted;
  if (!plan.selected_frontier.has_value()) {
    FAIL() << "frontier route must identify its observation frontier";
  }
  const ObservationFrontier& selected_frontier = *plan.selected_frontier;
  EXPECT_GT(selected_frontier.observation_pose.x, 20.5);
  EXPECT_GT(plan.fresh_frontier_candidate_count, 0U);
  EXPECT_GT(plan.fresh_frontier_evaluated_count, 0U);
  EXPECT_GT(plan.fresh_frontier_discovered_count, 0U);
  EXPECT_LE(plan.fresh_frontier_candidate_count, refreshed_graph.nodes().size());
  EXPECT_GE(plan.route_length_m,
            distance3D({18.5, 14.5, 6.5}, selected_frontier.observation_pose));
  EXPECT_TRUE(plan.executableTargetSelected());
}

TEST(IncrementalTopologicalPlanner3DTest,
     ObservationFrontierNeverCreatesATerminalMicroRoute) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 56, 32, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 4, 32, 13, 15, 5, 7);
  fillStateBox(occupancy, 33, 55, 13, 15, 5, 7, ObservedVoxelState::kUnknown);
  const IncrementalTopologyGraph3DConfig graph_config = graphConfig();
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy);
  IncrementalTopologicalPlanner3DConfig planner_config;
  planner_config.maximum_fresh_frontier_evaluations = 256U;
  planner_config.minimum_observation_target_displacement_m = 3.0;
  IncrementalTopologicalPlanner3D planner{planner_config};
  TopologicalExplorationMemory3D memory;
  const Point3 start{31.5, 14.5, 6.5};

  const IncrementalTopologicalPlan3D plan = planner.planObserved(
      graph, occupancy, observabilityConfig(graph_config.footprint), start,
      {52.5, 14.5, 6.5}, memory);

  // A planner may wait for a later observation rather than manufacture a
  // terminal-control-sized route. If a frontier is selected, it must still
  // satisfy the minimum executable displacement.
  if (plan.selected_frontier.has_value()) {
    EXPECT_GE(distance3D(start, plan.selected_frontier.value().observation_pose),
              planner_config.minimum_observation_target_displacement_m);
  }
}

TEST(IncrementalTopologicalPlanner3DTest,
     NormalPolicyConnectsToObservedGraphAcrossUnknownSpace) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 64, 32, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 4, 20, 13, 15, 5, 7);
  fillStateBox(occupancy, 21, 63, 13, 15, 5, 7, ObservedVoxelState::kUnknown);
  const IncrementalTopologyGraph3DConfig graph_config = graphConfig();
  const IncrementalTopologyGraph3DSnapshot stale_graph = buildGraph(occupancy);
  TopologicalExplorationMemory3D memory;

  IncrementalTopologicalPlanner3DConfig normal_config;
  normal_config.maximum_start_anchor_distance_m = 20.0;
  IncrementalTopologicalPlanner3D normal_planner{normal_config};
  const IncrementalTopologicalPlan3D normal = normal_planner.planObserved(
      stale_graph, occupancy, observabilityConfig(graph_config.footprint),
      {34.5, 14.5, 6.5}, {52.5, 14.5, 6.5}, memory);
  ASSERT_TRUE(normal.executableTargetSelected());
  EXPECT_NE(normal.status, IncrementalTopologicalPlanStatus3D::kStartNotRepresented);
  EXPECT_TRUE(normal.unknown_exposure);
  EXPECT_GT(normal.transition_support_segment_count, 0U);
  EXPECT_NE(normal.topology_lineage_id, 0U);

  IncrementalTopologicalPlanner3DConfig strict_config = normal_config;
  strict_config.require_known_free_space = true;
  IncrementalTopologicalPlanner3D strict_planner{strict_config};
  const IncrementalTopologicalPlan3D strict = strict_planner.planObserved(
      stale_graph, occupancy, observabilityConfig(graph_config.footprint),
      {34.5, 14.5, 6.5}, {52.5, 14.5, 6.5}, memory);
  EXPECT_EQ(strict.status, IncrementalTopologicalPlanStatus3D::kStartNotRepresented);
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

  const IncrementalTopologicalPlan3D first = planner.planObserved(
      graph, occupancy, observabilityConfig(graphConfig().footprint), start, goal,
      memory);
  if (!first.selected_frontier.has_value()) {
    FAIL() << "first plan must select a frontier";
  }
  const ObservationFrontier& first_frontier = *first.selected_frontier;
  for (std::size_t count = 0U; count < 4U; ++count) {
    memory.recordFrontierSelection(first_frontier.id);
  }
  const IncrementalTopologicalPlan3D second = planner.planObserved(
      graph, occupancy, observabilityConfig(graphConfig().footprint), start, goal,
      memory);

  if (!second.selected_frontier.has_value()) {
    FAIL() << "selection history must leave another reachable frontier";
  }
  EXPECT_NE(second.selected_frontier.value().id, first_frontier.id);
}

TEST(IncrementalTopologicalPlanner3DTest,
     CompletedFrontierSoftlyYieldsToAnAlternativeBranch) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 48, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 6, 41, 21, 23, 5, 7);
  fillFreeBox(occupancy, 21, 23, 21, 43, 5, 7);
  fillStateBox(occupancy, 0, 5, 21, 23, 5, 7, ObservedVoxelState::kUnknown);
  fillStateBox(occupancy, 42, 47, 21, 23, 5, 7, ObservedVoxelState::kUnknown);
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy);
  IncrementalTopologicalPlanner3DConfig config;
  config.frontier_selection_penalty = 0.0;
  config.frontier_completion_penalty = 100.0;
  IncrementalTopologicalPlanner3D planner{config};
  TopologicalExplorationMemory3D memory;
  const Point3 start{22.5, 41.5, 6.5};
  const Point3 goal{22.5, 100.0, 6.5};

  const IncrementalTopologicalPlan3D first = planner.planObserved(
      graph, occupancy, observabilityConfig(graphConfig().footprint), start, goal,
      memory);
  if (!first.selected_frontier.has_value()) {
    FAIL() << "first plan must select a frontier";
  }
  const ObservationFrontier& first_frontier = *first.selected_frontier;
  memory.recordFrontierCompletion(first_frontier.id);
  const IncrementalTopologicalPlan3D second = planner.planObserved(
      graph, occupancy, observabilityConfig(graphConfig().footprint), start, goal,
      memory);

  if (!second.selected_frontier.has_value()) {
    FAIL() << "completion penalty must leave a frontier candidate";
  }
  EXPECT_NE(second.selected_frontier->id, first_frontier.id);
  EXPECT_EQ(second.selected_frontier_completion_count, 0U);
}

TEST(IncrementalTopologicalPlanner3DTest,
     ActiveFrontierDoesNotPayItsCommittedSelectionPenaltyAgain) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 48, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 20, 27, 21, 23, 5, 7);
  fillStateBox(occupancy, 0, 19, 21, 23, 5, 7, ObservedVoxelState::kUnknown);
  fillStateBox(occupancy, 28, 47, 21, 23, 5, 7, ObservedVoxelState::kUnknown);
  fillFreeBox(occupancy, 8, 19, 21, 23, 5, 7);
  fillFreeBox(occupancy, 28, 39, 21, 23, 5, 7);
  const IncrementalTopologyGraph3DConfig graph_config = graphConfig();
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy);
  IncrementalTopologicalPlanner3DConfig planner_config;
  planner_config.frontier_selection_penalty = 100.0;
  planner_config.maximum_fresh_frontier_evaluations = 256U;
  IncrementalTopologicalPlanner3D planner{planner_config};
  TopologicalExplorationMemory3D memory;
  const Point3 start{23.5, 22.5, 6.5};
  const Point3 goal{23.5, 100.0, 6.5};

  const IncrementalTopologicalPlan3D first = planner.planObserved(
      graph, occupancy, observabilityConfig(graph_config.footprint), start, goal,
      memory);
  if (!first.selected_frontier.has_value()) {
    FAIL() << "first plan must select a frontier";
  }
  const ObservationFrontier& first_frontier = *first.selected_frontier;
  memory.recordFrontierSelection(first_frontier.id);
  const IncrementalTopologicalPlan3D ordinary = planner.planObserved(
      graph, occupancy, observabilityConfig(graph_config.footprint), start, goal,
      memory);
  if (!ordinary.selected_frontier.has_value()) {
    FAIL() << "ordinary replanning must retain a frontier candidate";
  }
  ASSERT_NE(ordinary.selected_frontier->id, first_frontier.id);

  const IncrementalTopologicalPlan3D continued = planner.planObserved(
      graph, occupancy, observabilityConfig(graph_config.footprint), start, goal,
      memory, first.selected_frontier);

  if (!continued.selected_frontier.has_value()) {
    FAIL() << "committed frontier must remain continuable";
  }
  EXPECT_EQ(continued.selected_frontier->id, first_frontier.id);
}

TEST(IncrementalTopologicalPlanner3DTest,
     ActiveFrontierRemainsAReplaceableSoftPreference) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 48, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 20, 27, 21, 23, 5, 7);
  fillStateBox(occupancy, 0, 19, 21, 23, 5, 7, ObservedVoxelState::kUnknown);
  fillStateBox(occupancy, 28, 47, 21, 23, 5, 7, ObservedVoxelState::kUnknown);
  fillFreeBox(occupancy, 8, 19, 21, 23, 5, 7);
  fillFreeBox(occupancy, 28, 39, 21, 23, 5, 7);
  const IncrementalTopologyGraph3DConfig graph_config = graphConfig();
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy);
  IncrementalTopologicalPlanner3DConfig planner_config;
  planner_config.frontier_selection_penalty = 100.0;
  planner_config.maximum_fresh_frontier_evaluations = 256U;
  IncrementalTopologicalPlanner3D planner{planner_config};
  TopologicalExplorationMemory3D memory;
  const Point3 start{23.5, 22.5, 6.5};
  const Point3 goal{23.5, 100.0, 6.5};

  const IncrementalTopologicalPlan3D first = planner.planObserved(
      graph, occupancy, observabilityConfig(graph_config.footprint), start, goal,
      memory);
  if (!first.selected_frontier.has_value()) {
    FAIL() << "first plan must select a frontier";
  }
  const ObservationFrontier& first_frontier = *first.selected_frontier;
  memory.recordFrontierSelection(first_frontier.id);
  memory.recordFrontierSelection(first_frontier.id);

  const IncrementalTopologicalPlan3D replacement = planner.planObserved(
      graph, occupancy, observabilityConfig(graph_config.footprint), start, goal,
      memory, first.selected_frontier);

  if (!replacement.selected_frontier.has_value()) {
    FAIL() << "soft preference must leave a replacement frontier";
  }
  EXPECT_NE(replacement.selected_frontier->id, first_frontier.id);
}

TEST(IncrementalTopologicalPlanner3DTest,
     RetiredFrontierAllowsGoalDirectedKnownConnectivity) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 56, 32, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 8, 39, 13, 15, 5, 7);
  fillStateBox(occupancy, 0, 7, 13, 15, 5, 7, ObservedVoxelState::kUnknown);
  fillStateBox(occupancy, 40, 55, 13, 15, 5, 7, ObservedVoxelState::kUnknown);
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy);
  fillStateBox(occupancy, 40, 55, 13, 15, 5, 7, ObservedVoxelState::kOccupied);
  IncrementalTopologicalPlanner3DConfig planner_config;
  planner_config.maximum_fresh_frontier_evaluations = 256U;
  IncrementalTopologicalPlanner3D planner{planner_config};
  TopologicalExplorationMemory3D memory;
  const ObservationFrontier retired{
      .id = ObservationFrontierId{999U},
      .observation_pose = {38.5, 14.5, 6.5},
      .boundary_centroid = {40.5, 14.5, 6.5},
      .observation_direction = {1.0, 0.0, 0.0},
  };

  const IncrementalTopologicalPlan3D plan = planner.planObserved(
      graph, occupancy, observabilityConfig(graphConfig().footprint), {23.5, 14.5, 6.5},
      {52.5, 14.5, 6.5}, memory, retired);

  EXPECT_EQ(plan.status, IncrementalTopologicalPlanStatus3D::kMissionContinuationRoute);
  EXPECT_FALSE(plan.selected_frontier.has_value());
  EXPECT_GT(plan.goal_progress_m, 0.0);
  EXPECT_GT(plan.reachable_frontier_count, 0U);
}

TEST(IncrementalTopologicalPlanner3DTest, RouteCoverageSoftlyPrefersAnUnvisitedBranch) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 48, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 6, 41, 21, 23, 5, 7);
  fillFreeBox(occupancy, 21, 23, 21, 43, 5, 7);
  fillStateBox(occupancy, 0, 5, 21, 23, 5, 7, ObservedVoxelState::kUnknown);
  fillStateBox(occupancy, 42, 47, 21, 23, 5, 7, ObservedVoxelState::kUnknown);
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy);
  IncrementalTopologicalPlanner3DConfig planner_config;
  planner_config.path_cost_weight = 0.0;
  planner_config.information_gain_reward = 0.0;
  planner_config.clearance_reward = 0.0;
  planner_config.goal_progress_reward = 0.0;
  planner_config.directed_traversal_penalty = 0.0;
  planner_config.repeated_distance_penalty = 0.0;
  planner_config.frontier_selection_penalty = 0.0;
  planner_config.coverage_penalty_weight = 10.0;
  IncrementalTopologicalPlanner3D planner{planner_config};
  TopologicalExplorationMemory3DConfig memory_config;
  memory_config.coverage_resolution_m = 1.0;
  memory_config.coverage_influence_radius_m = 2.0;
  memory_config.visit_penalty_weight = 4.0;
  memory_config.observation_penalty_weight = 0.0;
  TopologicalExplorationMemory3D memory{memory_config};
  const Point3 start{22.5, 41.5, 6.5};
  const Point3 goal{22.5, 100.0, 6.5};

  const IncrementalTopologicalPlan3D first = planner.planObserved(
      graph, occupancy, observabilityConfig(graphConfig().footprint), start, goal,
      memory);
  if (!first.selected_frontier.has_value()) {
    FAIL() << "first plan must select a frontier";
  }
  const ObservationFrontier& first_frontier = *first.selected_frontier;
  ASSERT_FALSE(first.guidance_points.empty());
  memory.recordVisitedPath(first.guidance_points, graph.revision(), 0.5);
  const IncrementalTopologicalPlan3D second = planner.planObserved(
      graph, occupancy, observabilityConfig(graphConfig().footprint), start, goal,
      memory);

  if (!second.selected_frontier.has_value()) {
    FAIL() << "coverage preference must leave another frontier";
  }
  EXPECT_NE(second.selected_frontier->id, first_frontier.id);
}

TEST(IncrementalTopologicalPlanner3DTest,
     DeadEndEvidencePenalizesButDoesNotForbidTheOnlyFrontierRoute) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 24, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 4, 40, 9, 11, 5, 7);
  fillStateBox(occupancy, 41, 47, 9, 11, 5, 7, ObservedVoxelState::kUnknown);
  const IncrementalTopologyGraph3DSnapshot graph = buildGraph(occupancy, 7U);
  IncrementalTopologicalPlanner3D planner;
  TopologicalExplorationMemory3D memory;
  const Point3 start{5.5, 10.5, 6.5};
  const Point3 goal{100.0, 10.5, 6.5};

  const IncrementalTopologicalPlan3D initial = planner.planObserved(
      graph, occupancy, observabilityConfig(graphConfig().footprint), start, goal,
      memory);
  ASSERT_EQ(initial.status, IncrementalTopologicalPlanStatus3D::kFrontierRoute)
      << "nodes=" << graph.nodes().size() << " edges=" << graph.edges().size()
      << " reachable=" << initial.reachable_frontier_count
      << " fresh_candidates=" << initial.fresh_frontier_candidate_count
      << " fresh_evaluated=" << initial.fresh_frontier_evaluated_count
      << " fresh_discovered=" << initial.fresh_frontier_discovered_count
      << " no_unknown_boundary=" << initial.fresh_frontier_status_counts.at(1U)
      << " no_executable_pose=" << initial.fresh_frontier_status_counts.at(4U);
  ASSERT_FALSE(initial.route_steps.empty());
  for (const TopologicalRouteStep3D& step : initial.route_steps) {
    for (const DirectedTopologyEdge3D& edge : step.directed_source_edges) {
      memory.recordDeadEnd(edge, step.evidence.validated_through_revision);
    }
  }

  const IncrementalTopologicalPlan3D retried = planner.planObserved(
      graph, occupancy, observabilityConfig(graphConfig().footprint), start, goal,
      memory);

  EXPECT_EQ(retried.status, IncrementalTopologicalPlanStatus3D::kFrontierRoute);
  EXPECT_TRUE(retried.executableTargetSelected());
  EXPECT_GT(retried.selection_score, initial.selection_score);
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
    if (first != nullptr && second != nullptr && first->degree >= 3U &&
        second->traits.terminal) {
      junction = first;
      terminal = second;
      branch = &edge;
      break;
    }
    if (first != nullptr && second != nullptr && second->degree >= 3U &&
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
  memory.recordTraversal(DirectedTopologyEdge3D{.edge_id = branch->id,
                                                .from = junction->id,
                                                .to = terminal->id},
                         branch->evidence.validated_through_revision, branch->length_m);
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
  EXPECT_EQ(conclusion.validated_through_revision, 7U);
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

TEST(IncrementalTopologicalPlanner3DTest,
     RoutesRawSafelyThroughRotatedVariableWidthTunnel) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 0.25, 88, 88, 28}};
  fillOccupied(occupancy);
  const Point3 tunnel_first{2.5, 3.5, 3.5};
  const Point3 tunnel_second{19.5, 18.5, 3.5};
  carveRotatedVariableWidthTunnel(occupancy, tunnel_first, tunnel_second, 1.35, 1.8,
                                  1.5);
  IncrementalTopologyGraph3DConfig config = graphConfig();
  config.block_size_cells = 8;
  config.coarse_sample_stride_cells = 2;
  config.refined_sample_stride_cells = 1;
  config.footprint = SweptFootprintConfig{};
  IncrementalTopologyGraph3D topology{config};
  const IncrementalTopologyGraph3DUpdate update =
      topology.update(occupancy, 1U, {}, true);
  const IncrementalTopologyGraph3DSnapshot graph = topology.snapshot();
  IncrementalTopologicalPlanner3D planner;
  TopologicalExplorationMemory3D memory;
  const Point3 start{4.2, 5.0, 3.5};
  const Point3 goal{17.8, 17.0, 3.5};

  const IncrementalTopologicalPlan3D plan = planner.plan(graph, start, goal, memory);

  ASSERT_EQ(plan.status, IncrementalTopologicalPlanStatus3D::kMissionRoute);
  ASSERT_TRUE(plan.reaches_mission_goal);
  ASSERT_GE(plan.guidance_points.size(), 2U);
  EXPECT_GT(update.adaptively_refined_blocks, 0U);
  for (std::size_t index = 1U; index < plan.guidance_points.size(); ++index) {
    const SweptFootprintResult validation = validateRawSweptFootprint(
        occupancy, plan.guidance_points[index - 1U], FootprintBodyAxis{},
        plan.guidance_points[index], FootprintBodyAxis{}, config.footprint);
    EXPECT_TRUE(validation.accepted())
        << "segment=" << index - 1U
        << " status=" << static_cast<int>(validation.status);
  }
}

} // namespace
} // namespace drone_city_nav
