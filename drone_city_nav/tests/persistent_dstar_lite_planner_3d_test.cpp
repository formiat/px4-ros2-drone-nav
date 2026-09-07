#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"
#include "persistent_dstar_lite_planner_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

TEST(PersistentDStarLitePlanner3DTest,
     CoordinatorPublishesAndContinuesTheIndependentUpdateAxes) {
  PlannerUpdate3D update;
  update.input_status = PlannerInputStatus3D::kAccepted;
  update.progress = SearchProgress3D::kRunning;

  PlannerDispatch3D dispatch = coordinatePlannerUpdate3D(update);
  EXPECT_FALSE(dispatch.publish_incumbent);
  EXPECT_TRUE(dispatch.continue_search);
  EXPECT_FALSE(dispatch.terminal);

  update.improved_incumbent = SpatialRouteCandidate3D{
      .points = {{0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}},
      .source = SpatialRouteCandidateSource3D::kFeasibilitySearch,
      .path_length_m = 1.0,
      .estimated_execution_time_s = 1.0,
      .estimated_translation_time_s = 1.0,
      .estimated_stationary_turn_time_s = 0.0,
  };
  dispatch = coordinatePlannerUpdate3D(update);
  EXPECT_TRUE(dispatch.publish_incumbent);
  EXPECT_TRUE(dispatch.continue_search);
  EXPECT_FALSE(dispatch.terminal);

  update.progress = SearchProgress3D::kConverged;
  dispatch = coordinatePlannerUpdate3D(update);
  EXPECT_TRUE(dispatch.publish_incumbent);
  EXPECT_FALSE(dispatch.continue_search);
  EXPECT_TRUE(dispatch.terminal);
}

TEST(PersistentDStarLitePlanner3DTest, AReversalHasToEarnItsReplacement) {
  const auto route = [](const double second_x, const double second_y,
                        const double objective_s) {
    SpatialRouteCandidate3D candidate{
        .points = {{0.0, 0.0, 1.0}, {second_x, second_y, 1.0}, {20.0, 0.0, 1.0}},
        .source = SpatialRouteCandidateSource3D::kFeasibilitySearch,
        .path_length_m = 20.0,
        .estimated_execution_time_s = objective_s,
        .estimated_translation_time_s = objective_s,
        .estimated_stationary_turn_time_s = 0.0,
    };
    return candidate;
  };
  constexpr double kMargin{1.0};

  // A candidate continuing the same heading replaces the incumbent as soon as
  // it is better at all.
  {
    detail::AnytimePlannerCoordinator3D coordinator{kMargin};
    ASSERT_TRUE(coordinator.consider(route(1.0, 0.0, 10.0)).has_value());
    EXPECT_TRUE(coordinator.consider(route(1.0, 0.0, 9.9)).has_value());
  }
  // One that turns the vehicle around does not, until it is better by the
  // margin.
  {
    detail::AnytimePlannerCoordinator3D coordinator{kMargin};
    ASSERT_TRUE(coordinator.consider(route(1.0, 0.0, 10.0)).has_value());
    EXPECT_FALSE(coordinator.consider(route(-1.0, 0.0, 9.9)).has_value());
    EXPECT_TRUE(coordinator.consider(route(-1.0, 0.0, 8.5)).has_value());
  }
  // A zero margin restores the plain best-objective rule.
  {
    detail::AnytimePlannerCoordinator3D coordinator{0.0};
    ASSERT_TRUE(coordinator.consider(route(1.0, 0.0, 10.0)).has_value());
    EXPECT_TRUE(coordinator.consider(route(-1.0, 0.0, 9.9)).has_value());
  }
}

TEST(PersistentDStarLitePlanner3DTest, CandidatesCompeteOnTheRankedExecutionTime) {
  SpatialRouteCandidate3D plain{
      .points = {{0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}},
      .source = SpatialRouteCandidateSource3D::kFeasibilitySearch,
      .path_length_m = 1.0,
      .estimated_execution_time_s = 1.0,
      .estimated_translation_time_s = 1.0,
      .estimated_stationary_turn_time_s = 0.0,
  };
  EXPECT_TRUE(plain.valid());
  EXPECT_DOUBLE_EQ(plain.objectiveS(), 1.0);
  SpatialRouteCandidate3D ranked = plain;
  ranked.ranked_execution_time_s = 4.0;
  EXPECT_TRUE(ranked.valid());
  EXPECT_DOUBLE_EQ(ranked.objectiveS(), 4.0);
  ranked.ranked_execution_time_s = -1.0;
  EXPECT_FALSE(ranked.valid());
}

TEST(PersistentDStarLitePlanner3DTest, ANewSessionReceivesTheResidentIncumbent) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 12, 8});
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{1.5, 1.5, 1.5};
  const Point3 goal{10.5, 10.5, 5.5};
  PersistentPlannerRequest3D first_session = request(start, goal, world(occupancy, 1U));
  first_session.session_id = 1U;

  const PlannerUpdate3D converged = planner.plan(first_session);
  ASSERT_TRUE(converged.publishable());
  EXPECT_EQ(converged.progress, SearchProgress3D::kConverged);
  const std::vector<Point3> route = candidate(converged).points;

  // The same session already holds the incumbent: nothing new to publish.
  const PlannerUpdate3D same_session = planner.plan(first_session);
  EXPECT_FALSE(same_session.publishable());
  EXPECT_EQ(same_session.progress, SearchProgress3D::kConverged);

  // A new session of the same mission holds no route of this search yet, so
  // the retained incumbent is delivered to it without any improvement.
  PersistentPlannerRequest3D second_session = first_session;
  second_session.session_id = 2U;
  const PlannerUpdate3D new_session = planner.plan(second_session);
  ASSERT_TRUE(new_session.publishable());
  EXPECT_TRUE(new_session.telemetry.incumbent_retained);
  expectSamePath(candidate(new_session).points, route);
  EXPECT_FALSE(planner.plan(second_session).publishable());
}

TEST(PersistentDStarLitePlanner3DTest,
     ARejectedIncumbentIsDroppedAndTheSearchRestartsFromTheVehicle) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 12, 8});
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{1.5, 1.5, 1.5};
  const Point3 goal{10.5, 10.5, 5.5};
  PersistentPlannerRequest3D session = request(start, goal, world(occupancy, 1U));
  session.session_id = 1U;
  ASSERT_TRUE(planner.plan(session).publishable());

  // The consumer could not enter the delivered incumbent and the vehicle now
  // waits elsewhere: the continuation carries a newer rejection sequence.
  PersistentPlannerRequest3D rejected = session;
  rejected.start = Point3{1.5, 6.5, 4.5};
  rejected.incumbent_rejection_sequence = 1U;
  const PlannerUpdate3D restarted = planner.plan(rejected);

  ASSERT_TRUE(restarted.publishable());
  EXPECT_FALSE(restarted.telemetry.incumbent_retained);
  EXPECT_DOUBLE_EQ(candidate(restarted).points.front().x, rejected.start.x);
  EXPECT_DOUBLE_EQ(candidate(restarted).points.front().y, rejected.start.y);
  EXPECT_DOUBLE_EQ(candidate(restarted).points.front().z, rejected.start.z);
  // The same sequence applies once; an unchanged request publishes nothing new.
  EXPECT_FALSE(planner.plan(rejected).publishable());
}

TEST(PersistentDStarLitePlanner3DTest,
     SolvesOneWorldFixedTwentySixConnectedMissionToTheExactGoal) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 12, 8});
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{1.5, 1.5, 1.5};
  const Point3 goal{10.5, 10.5, 5.5};

  const PlannerUpdate3D result =
      planner.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(result.publishable());
  EXPECT_EQ(result.input_status, PlannerInputStatus3D::kAccepted);
  EXPECT_EQ(result.progress, SearchProgress3D::kConverged);
  EXPECT_FALSE(result.telemetry.search_state_reused);
  EXPECT_EQ(result.telemetry.search_generation, 1U);
  EXPECT_DOUBLE_EQ(candidate(result).points.front().x, start.x);
  EXPECT_DOUBLE_EQ(candidate(result).points.front().y, start.y);
  EXPECT_DOUBLE_EQ(candidate(result).points.front().z, start.z);
  EXPECT_DOUBLE_EQ(candidate(result).points.back().x, goal.x);
  EXPECT_DOUBLE_EQ(candidate(result).points.back().y, goal.y);
  EXPECT_DOUBLE_EQ(candidate(result).points.back().z, goal.z);
  EXPECT_GT(candidate(result).estimated_translation_time_s, 0.0);
  EXPECT_GE(candidate(result).estimated_execution_time_s,
            candidate(result).estimated_translation_time_s);
  expectRawValid(candidate(result).points, *occupancy,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     UsesWorldFixedMultiresolutionEdgesWithoutChangingTheExactMissionPath) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 12, 8});
  const Point3 start{0.5, 4.5, 4.5};
  const Point3 goal{40.5, 4.5, 4.5};
  PersistentPlannerConfig3D adaptive_config = testConfig();
  adaptive_config.maximum_adaptive_lattice_level = 2U;
  PersistentDStarLitePlanner3D adaptive{adaptive_config};
  PersistentPlannerConfig3D fixed_config = adaptive_config;
  fixed_config.maximum_adaptive_lattice_level = 0U;
  PersistentDStarLitePlanner3D fixed{fixed_config};

  const PlannerUpdate3D adaptive_result =
      adaptive.plan(request(start, goal, world(occupancy, 1U)));
  const PlannerUpdate3D fixed_result =
      fixed.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(adaptive_result.publishable());
  ASSERT_TRUE(fixed_result.publishable());
  expectSamePath(candidate(adaptive_result).points, candidate(fixed_result).points);
  EXPECT_DOUBLE_EQ(candidate(adaptive_result).path_length_m,
                   candidate(fixed_result).path_length_m);
  EXPECT_GT(adaptive_result.telemetry.adaptive_edge_queries, 0U);
  EXPECT_GT(adaptive_result.telemetry.adaptive_edges_in_extracted_path, 0U);
  EXPECT_EQ(adaptive_result.telemetry.maximum_queried_lattice_level, 2U);
  EXPECT_EQ(fixed_result.telemetry.adaptive_edge_queries, 0U);
  EXPECT_EQ(fixed_result.telemetry.maximum_queried_lattice_level, 0U);
}

TEST(PersistentDStarLitePlanner3DTest,
     FreeUnknownRelabelPreservesPathCostAndIncrementalSearchState) {
  auto first = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 8, 6});
  static_cast<void>(first->setState(GridIndex3D{3, 3, 2}, ObservedVoxelState::kFree));
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{1.5, 3.5, 2.5};
  const Point3 goal{10.5, 3.5, 2.5};
  const PlannerUpdate3D initial = planner.plan(request(start, goal, world(first, 1U)));
  ASSERT_TRUE(initial.publishable());

  auto relabeled = std::make_shared<ObservedOccupancyGrid3D>(*first);
  static_cast<void>(
      relabeled->setState(GridIndex3D{3, 3, 2}, ObservedVoxelState::kUnknown));
  static_cast<void>(
      relabeled->setState(GridIndex3D{8, 4, 2}, ObservedVoxelState::kFree));
  const PlannerUpdate3D updated = planner.plan(
      request(start, goal,
              world(relabeled, 2U,
                    {ObservedOccupancyGrid3D::chunkIndex(GridIndex3D{3, 3, 2})})));

  EXPECT_FALSE(updated.publishable());
  EXPECT_TRUE(updated.telemetry.search_state_reused);
  EXPECT_TRUE(updated.telemetry.occupied_world_unchanged);
  EXPECT_EQ(updated.telemetry.changed_occupied_voxels, 0U);
  EXPECT_EQ(updated.telemetry.search_generation, initial.telemetry.search_generation);
  EXPECT_TRUE(updated.telemetry.incumbent_retained);
  EXPECT_TRUE(updated.telemetry.incumbent_available);
  EXPECT_EQ(updated.progress, SearchProgress3D::kConverged);
}

TEST(PersistentDStarLitePlanner3DTest, ARefinedDepartureLeavesAPocketNoNodeReaches) {
  // A corridor with an alcove off it, and a lattice twice as coarse as the
  // map. The alcove is one cell wide and its column carries no lattice node,
  // so the vehicle resting inside it reaches no node in a single segment: the
  // swept body clips a jamb on every straight line out. Two legs clear it —
  // straight out of the mouth, then across to a corridor node.
  constexpr int kWidth = 16;
  constexpr int kHeight = 9;
  constexpr int kDepth = 5;
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, kWidth, kHeight, kDepth});
  for (int x = 0; x < kWidth; ++x) {
    for (int z = 0; z < kDepth; ++z) {
      // South wall of the corridor, and its north wall with the alcove mouth.
      ASSERT_TRUE(occupancy->setState({x, 1, z}, ObservedVoxelState::kOccupied));
      if (x != 2) {
        ASSERT_TRUE(occupancy->setState({x, 4, z}, ObservedVoxelState::kOccupied));
      }
      // Everything north of the mouth except the alcove column itself.
      if (x != 2) {
        ASSERT_TRUE(occupancy->setState({x, 5, z}, ObservedVoxelState::kOccupied));
      }
      for (int y = 6; y < kHeight; ++y) {
        ASSERT_TRUE(occupancy->setState({x, y, z}, ObservedVoxelState::kOccupied));
      }
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.minimum_horizontal_step_m = 2.0;
  config.minimum_vertical_step_m = 2.0;
  config.physical_footprint.radius_m = 0.4;
  config.physical_footprint.perimeter_samples = 8U;
  config.physical_footprint.radial_rings = 1U;
  config.physical_footprint.axial_samples = 1U;
  const Point3 start{2.5, 5.5, 2.5};
  const Point3 goal{14.5, 2.5, 2.5};

  PersistentPlannerConfig3D unrefined_config = config;
  unrefined_config.departure_refinement_subdivisions = 0U;
  PersistentDStarLitePlanner3D unrefined{unrefined_config};
  const PlannerUpdate3D stuck =
      unrefined.plan(request(start, goal, world(occupancy, 1U)));
  ASSERT_EQ(stuck.input_status, PlannerInputStatus3D::kStartUnavailable)
      << "the fixture no longer reproduces a pocket";

  PersistentDStarLitePlanner3D refined{config};
  PlannerUpdate3D update = refined.plan(request(start, goal, world(occupancy, 1U)));
  for (int attempt = 0; attempt < 8 && !update.publishable(); ++attempt) {
    update = refined.plan(request(start, goal, world(occupancy, 1U)));
  }

  EXPECT_NE(update.input_status, PlannerInputStatus3D::kStartUnavailable);
  ASSERT_TRUE(update.publishable())
      << "refined departure did not recover a pocket the plain one lost";
  EXPECT_TRUE(update.telemetry.departure_waypoint_used);
  const std::vector<Point3>& points = candidate(update).points;
  ASSERT_GE(points.size(), 2U);
  EXPECT_NEAR(points.front().x, start.x, 1.0e-9);
  EXPECT_NEAR(points.front().y, start.y, 1.0e-9);
  expectRawValid(points, *occupancy, refined.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     AVanishedCellRepricesOnlyBlockedEdgesAndRestoresTheDirectRoute) {
  auto open_occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 10, 6});
  PersistentPlannerConfig3D config = testConfig();
  config.physical_footprint.radius_m = 0.4;
  config.physical_footprint.perimeter_samples = 8U;
  config.physical_footprint.radial_rings = 1U;
  config.physical_footprint.axial_samples = 1U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 5.5, 2.5};
  const Point3 goal{12.5, 5.5, 2.5};
  const PlannerUpdate3D initial =
      planner.plan(request(start, goal, world(open_occupancy, 1U)));
  ASSERT_TRUE(initial.publishable());

  auto blocked = std::make_shared<ObservedOccupancyGrid3D>(*open_occupancy);
  const GridIndex3D obstacle{7, 5, 2};
  ASSERT_TRUE(blocked->setState(obstacle, ObservedVoxelState::kOccupied));
  const PlannerUpdate3D detoured = planner.plan(
      request(start, goal,
              world(blocked, 2U, {ObservedOccupancyGrid3D::chunkIndex(obstacle)})));
  ASSERT_TRUE(detoured.publishable());
  // The cell that appeared can only block clear edges; every priced edge it
  // touches was clear, so all of them are forgotten.
  EXPECT_GT(detoured.telemetry.schedule_edges_forgotten, 0U);
  EXPECT_GT(candidate(detoured).path_length_m, candidate(initial).path_length_m);

  // A cell that vanishes can only unblock blocked edges: the detour's edges
  // beside the obstacle stay priced, and the direct route comes back.
  auto reopened = std::make_shared<ObservedOccupancyGrid3D>(*blocked);
  ASSERT_TRUE(reopened->setState(obstacle, ObservedVoxelState::kFree));
  const PlannerUpdate3D restored = planner.plan(
      request(start, goal,
              world(reopened, 3U, {ObservedOccupancyGrid3D::chunkIndex(obstacle)})));
  ASSERT_TRUE(restored.publishable());
  EXPECT_TRUE(restored.telemetry.search_state_reused);
  EXPECT_GT(restored.telemetry.schedule_edges_forgotten, 0U);
  EXPECT_LE(restored.telemetry.schedule_edges_forgotten,
            detoured.telemetry.schedule_edges_forgotten);
  EXPECT_NEAR(candidate(restored).path_length_m, candidate(initial).path_length_m,
              1.0e-6);
  expectRawValid(candidate(restored).points, *reopened,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     AChangeOutsideTheBodyReachSchedulesNoRepairUntilTheSearchConsultsIt) {
  // One lattice row under a low flight envelope: labels live on the route
  // line and one vertical step above it. A cell six metres above the highest
  // label is only stamped for the lazy clearance check; a cell within the
  // body reach forgets edges and repairs them.
  auto open_occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 1, 12});
  // One occupied cell beyond every label's reach keeps the chunk ranked, so
  // the labels cache their clearances instead of skipping the ranking.
  ASSERT_TRUE(open_occupancy->setState({0, 0, 11}, ObservedVoxelState::kOccupied));
  PersistentPlannerConfig3D config = testConfig();
  config.clearance_ranking_weight = 1.5;
  config.clearance_ranking_distance_m = 6.0;
  config.flight_envelope.maximum_target_z_m = 3.9;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 0.5, 2.5};
  const Point3 goal{12.5, 0.5, 2.5};
  const PlannerUpdate3D initial =
      planner.plan(request(start, goal, world(open_occupancy, 1U)));
  ASSERT_TRUE(initial.publishable());

  auto far_cell = std::make_shared<ObservedOccupancyGrid3D>(*open_occupancy);
  const GridIndex3D far{7, 0, 9};
  ASSERT_TRUE(far_cell->setState(far, ObservedVoxelState::kOccupied));
  const PlannerUpdate3D cached = planner.plan(request(
      start, goal, world(far_cell, 2U, {ObservedOccupancyGrid3D::chunkIndex(far)})));
  // Nothing to repair: the session keeps its incumbent and converges at once.
  EXPECT_FALSE(cached.publishable());
  EXPECT_TRUE(cached.telemetry.incumbent_retained);
  EXPECT_EQ(cached.progress, SearchProgress3D::kConverged);
  EXPECT_EQ(cached.telemetry.changed_occupied_voxels, 1U);
  EXPECT_EQ(cached.telemetry.schedule_edges_forgotten, 0U);
  EXPECT_EQ(cached.telemetry.affected_lattice_states, 0U);

  // At the level of the highest label the cell touches its edges: they are
  // forgotten, re-priced, and the moved clearance repairs the labels.
  auto near_cell = std::make_shared<ObservedOccupancyGrid3D>(*open_occupancy);
  const GridIndex3D near{7, 0, 3};
  ASSERT_TRUE(near_cell->setState(near, ObservedVoxelState::kOccupied));
  PersistentDStarLitePlanner3D fresh{config};
  ASSERT_TRUE(
      fresh.plan(request(start, goal, world(open_occupancy, 1U))).publishable());
  const PlannerUpdate3D repaired = fresh.plan(request(
      start, goal, world(near_cell, 2U, {ObservedOccupancyGrid3D::chunkIndex(near)})));
  EXPECT_GT(repaired.telemetry.schedule_edges_forgotten, 0U);
  EXPECT_GT(repaired.telemetry.affected_lattice_states, 0U);
  EXPECT_GT(repaired.telemetry.schedule_clearances_rederived, 0U);
}

TEST(PersistentDStarLitePlanner3DTest,
     RepairsTheResidentSearchAfterAnOccupiedCellAppearsOnItsPath) {
  auto initial_occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 10, 6});
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{1.5, 5.5, 2.5};
  const Point3 goal{12.5, 5.5, 2.5};
  const PlannerUpdate3D initial =
      planner.plan(request(start, goal, world(initial_occupancy, 1U)));
  ASSERT_TRUE(initial.publishable());

  auto changed = std::make_shared<ObservedOccupancyGrid3D>(*initial_occupancy);
  const GridIndex3D obstacle{7, 5, 2};
  ASSERT_TRUE(changed->setState(obstacle, ObservedVoxelState::kOccupied));
  const PlannerUpdate3D repaired = planner.plan(
      request(start, goal,
              world(changed, 2U, {ObservedOccupancyGrid3D::chunkIndex(obstacle)})));

  ASSERT_TRUE(repaired.publishable());
  EXPECT_TRUE(repaired.telemetry.search_state_reused);
  EXPECT_EQ(repaired.telemetry.changed_occupied_voxels, 1U);
  EXPECT_GT(repaired.telemetry.affected_lattice_states, 0U);
  EXPECT_GT(repaired.telemetry.adaptive_edge_queries, 0U);
  EXPECT_EQ(repaired.telemetry.search_generation, initial.telemetry.search_generation);
  EXPECT_EQ(repaired.telemetry.repair_generation, 1U);
  EXPECT_GT(candidate(repaired).path_length_m, candidate(initial).path_length_m);
  expectRawValid(candidate(repaired).points, *changed,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     MissingIncrementalPredecessorRepairsFromAnExactGridDifference) {
  auto initial_occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 10, 6});
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{1.5, 5.5, 2.5};
  const Point3 goal{12.5, 5.5, 2.5};
  const PlannerUpdate3D initial =
      planner.plan(request(start, goal, world(initial_occupancy, 1U)));
  ASSERT_TRUE(initial.publishable());

  auto changed = std::make_shared<ObservedOccupancyGrid3D>(*initial_occupancy);
  const GridIndex3D obstacle{7, 5, 2};
  ASSERT_TRUE(changed->setState(obstacle, ObservedVoxelState::kOccupied));
  PersistentPlannerWorld3D skipped_predecessor =
      world(changed, 3U, {ObservedOccupancyGrid3D::chunkIndex(obstacle)});
  skipped_predecessor.incremental_parent_revision = 2U;
  const PlannerUpdate3D repaired =
      planner.plan(request(start, goal, std::move(skipped_predecessor)));

  // The incomplete dirty-chunk delta is replaced by an exact comparison of the
  // two resident grids, so the search keeps its labels and repairs locally.
  ASSERT_TRUE(repaired.publishable());
  EXPECT_TRUE(repaired.telemetry.search_state_reused);
  EXPECT_EQ(repaired.telemetry.changed_occupied_voxels, 1U);
  EXPECT_EQ(repaired.telemetry.search_generation, initial.telemetry.search_generation);
  EXPECT_GT(candidate(repaired).path_length_m, candidate(initial).path_length_m);
  expectRawValid(candidate(repaired).points, *changed,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     BoundedWorldRepairResumesBeforeContinuingTheShortestPathSearch) {
  auto initial_occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 10, 6});
  PersistentPlannerConfig3D config = testConfig();
  config.maximum_expansions_per_update = 1U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 5.5, 2.5};
  const Point3 goal{12.5, 5.5, 2.5};
  PlannerUpdate3D initial =
      planner.plan(request(start, goal, world(initial_occupancy, 1U)));
  for (std::size_t attempt = 0U; attempt < 5000U && !initial.publishable(); ++attempt) {
    initial = planner.plan(request(start, goal, world(initial_occupancy, 1U)));
  }
  ASSERT_TRUE(initial.publishable());

  auto changed = std::make_shared<ObservedOccupancyGrid3D>(*initial_occupancy);
  const GridIndex3D obstacle{7, 5, 2};
  ASSERT_TRUE(changed->setState(obstacle, ObservedVoxelState::kOccupied));
  PlannerUpdate3D repaired = planner.plan(
      request(start, goal,
              world(changed, 2U, {ObservedOccupancyGrid3D::chunkIndex(obstacle)})));

  EXPECT_TRUE(repaired.telemetry.search_state_reused);
  EXPECT_EQ(repaired.telemetry.search_generation, initial.telemetry.search_generation);
  EXPECT_EQ(repaired.telemetry.repair_generation, 1U);
  EXPECT_GT(repaired.telemetry.affected_lattice_states, 1U);
  EXPECT_EQ(repaired.telemetry.repair_lattice_states_processed, 1U);
  EXPECT_TRUE(repaired.telemetry.repair_pending);
  EXPECT_GT(repaired.telemetry.repair_lattice_states_pending, 0U);
  EXPECT_EQ(repaired.telemetry.expansions, 0U);

  for (std::size_t attempt = 0U; attempt < 5000U && !repaired.publishable();
       ++attempt) {
    repaired = planner.plan(
        request(start, goal,
                world(changed, 2U, {ObservedOccupancyGrid3D::chunkIndex(obstacle)})));
  }

  ASSERT_TRUE(repaired.publishable());
  EXPECT_FALSE(repaired.telemetry.repair_pending);
  EXPECT_EQ(repaired.telemetry.repair_lattice_states_pending, 0U);
  expectRawValid(candidate(repaired).points, *changed,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     PropagatesANewlyOpenedAdaptiveEdgeIntoPreviouslyUnseenStates) {
  auto blocked = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 10, 6});
  for (int z = 0; z < 6; ++z) {
    for (int y = 0; y < 10; ++y) {
      static_cast<void>(
          blocked->setState(GridIndex3D{7, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{2.5, 5.5, 2.5};
  const Point3 goal{11.5, 5.5, 2.5};
  const PlannerUpdate3D initial =
      planner.plan(request(start, goal, world(blocked, 1U)));
  ASSERT_EQ(initial.progress, SearchProgress3D::kNoRoute);

  auto opened = std::make_shared<ObservedOccupancyGrid3D>(*blocked);
  const GridIndex3D opening{7, 5, 2};
  ASSERT_TRUE(opened->setState(opening, ObservedVoxelState::kUnknown));
  const PlannerUpdate3D repaired = planner.plan(request(
      start, goal, world(opened, 2U, {ObservedOccupancyGrid3D::chunkIndex(opening)})));

  ASSERT_TRUE(repaired.publishable());
  EXPECT_TRUE(repaired.telemetry.search_state_reused);
  EXPECT_EQ(repaired.telemetry.changed_occupied_voxels, 1U);
  EXPECT_GT(repaired.telemetry.affected_lattice_states, 0U);
  EXPECT_EQ(repaired.telemetry.search_generation, initial.telemetry.search_generation);
  EXPECT_EQ(repaired.telemetry.repair_generation, 1U);
  expectRawValid(candidate(repaired).points, *opened,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     ExecutionTimeRefinementPrefersFewerStopsOverTheShortestZigzag) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 9, 3});
  for (int z = 0; z < 3; ++z) {
    for (int y = 0; y < 9; ++y) {
      for (int x = 0; x < 14; ++x) {
        ASSERT_TRUE(
            occupancy->setState(GridIndex3D{x, y, z}, ObservedVoxelState::kOccupied));
      }
    }
  }
  const auto open = [&](const int x, const int y) {
    static_cast<void>(
        occupancy->setState(GridIndex3D{x, y, 1}, ObservedVoxelState::kUnknown));
  };

  // The lower corridor is shorter but forces a repeated stop-and-turn zigzag.
  for (int x = 1; x <= 3; ++x) {
    open(x, 3);
  }
  for (int x = 3; x <= 5; ++x) {
    open(x, 2);
  }
  for (int x = 5; x <= 7; ++x) {
    open(x, 3);
  }
  for (int x = 7; x <= 9; ++x) {
    open(x, 2);
  }
  for (int x = 9; x <= 12; ++x) {
    open(x, 3);
  }

  // The upper corridor is longer but has only two right-angle stops.
  for (int y = 3; y <= 7; ++y) {
    open(1, y);
    open(12, y);
  }
  for (int x = 1; x <= 12; ++x) {
    open(x, 7);
  }

  PersistentPlannerConfig3D timed_config = testConfig();
  timed_config.maximum_adaptive_lattice_level = 0U;
  timed_config.time_model.maximum_horizontal_acceleration_mps2 = 0.75;
  timed_config.time_model.maximum_vertical_acceleration_mps2 = 0.75;
  timed_config.time_model.maximum_control_jerk_mps3 = 1.5;
  timed_config.time_model.maximum_yaw_rate_radps = 0.5;
  timed_config.time_model.maximum_yaw_acceleration_radps2 = 0.5;
  PersistentDStarLitePlanner3D timed_planner{timed_config};
  const Point3 start{1.5, 3.5, 1.5};
  const Point3 goal{12.5, 3.5, 1.5};

  const PlannerUpdate3D timed =
      timed_planner.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(timed.publishable());
  EXPECT_TRUE(timed.telemetry.execution_time_search_complete);
  EXPECT_EQ(timed.progress, SearchProgress3D::kConverged);
  EXPECT_GT(timed.telemetry.execution_time_search_expansions, 0U);
  EXPECT_GT(timed.telemetry.execution_time_search_objective_s, 0.0);
  EXPECT_TRUE(std::ranges::any_of(candidate(timed).points,
                                  [](const Point3& point) { return point.y > 6.5; }));
  EXPECT_GT(candidate(timed).estimated_stationary_turn_time_s, 0.0);

  PersistentPlannerConfig3D translation_only_config = timed_config;
  translation_only_config.minimum_continuous_turn_alignment = -1.0;
  PersistentDStarLitePlanner3D translation_only_planner{translation_only_config};
  const PlannerUpdate3D translation_only =
      translation_only_planner.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(translation_only.publishable());
  EXPECT_LT(candidate(translation_only).path_length_m, candidate(timed).path_length_m);
  EXPECT_FALSE(std::ranges::any_of(candidate(translation_only).points,
                                   [](const Point3& point) { return point.y > 6.5; }));
  expectRawValid(candidate(timed).points, *occupancy,
                 timed_planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     MovingStartReusesTheBackwardSearchAndMayPublishAStrictImprovement) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 16, 8, 6});
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 first_start{1.5, 3.5, 2.5};
  const Point3 moved_start{5.5, 3.5, 2.5};
  const Point3 goal{14.5, 3.5, 2.5};
  const PlannerUpdate3D initial =
      planner.plan(request(first_start, goal, world(occupancy, 1U)));
  ASSERT_TRUE(initial.publishable());

  const PlannerUpdate3D moved =
      planner.plan(request(moved_start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(moved.publishable());
  EXPECT_TRUE(moved.telemetry.search_state_reused);
  EXPECT_EQ(moved.telemetry.search_generation, initial.telemetry.search_generation);
  EXPECT_TRUE(moved.telemetry.incumbent_retained);
  EXPECT_TRUE(moved.telemetry.incumbent_available);
  EXPECT_DOUBLE_EQ(candidate(moved).points.front().x, moved_start.x);
  EXPECT_DOUBLE_EQ(candidate(moved).points.front().y, moved_start.y);
  EXPECT_DOUBLE_EQ(candidate(moved).points.front().z, moved_start.z);
  EXPECT_LT(candidate(moved).path_length_m, candidate(initial).path_length_m);
}

TEST(PersistentDStarLitePlanner3DTest,
     MovingTransientDepartureEvidenceDoesNotResetThePersistentRawSearch) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 18, 18, 8});
  PersistentPlannerConfig3D config = testConfig();
  config.maximum_expansions_per_update = 1U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 1.5, 1.5};
  const Point3 goal{16.5, 16.5, 6.5};
  const ProprioceptiveFreeSpaceSeed3D initial_seed{
      .position = start,
      .body_axis = FootprintBodyAxis{},
      .footprint = SweptFootprintConfig{},
  };
  const LaunchSupportContact3D initial_support{
      .seed = initial_seed,
      .contact_cells = {AxisAlignedBox3D{
          .minimum = Point3{1.0, 1.0, 1.0},
          .maximum = Point3{2.0, 2.0, 2.0},
      }},
      .occupied_evidence_cells = 0U,
      .evidence_source = LaunchSupportEvidenceSource::kVehicleLandDetector,
      .maximum_lateral_departure_m = 1.0,
      .minimum_axial_departure_m = 0.0,
      .maximum_axial_settling_m = 1.0,
  };
  ASSERT_TRUE(launchSupportContactValid3D(initial_support));
  PersistentPlannerWorld3D initial_world = world(occupancy, 1U);
  initial_world.proprioceptive_free_space_seed = initial_seed;
  initial_world.launch_support_contact = initial_support;

  const PlannerUpdate3D initial =
      planner.plan(request(start, goal, std::move(initial_world)));
  ASSERT_EQ(initial.progress, SearchProgress3D::kRunning);
  ASSERT_FALSE(initial.telemetry.search_state_reused);

  ProprioceptiveFreeSpaceSeed3D moved_seed = initial_seed;
  moved_seed.position.x += 0.25;
  LaunchSupportContact3D moved_support = initial_support;
  moved_support.seed = moved_seed;
  ASSERT_TRUE(launchSupportContactValid3D(moved_support));
  PersistentPlannerWorld3D refreshed_world = world(occupancy, 1U);
  refreshed_world.proprioceptive_free_space_seed = moved_seed;
  refreshed_world.launch_support_contact = moved_support;
  const PlannerUpdate3D refreshed =
      planner.plan(request(start, goal, std::move(refreshed_world)));

  EXPECT_TRUE(refreshed.telemetry.search_state_reused);
  EXPECT_TRUE(refreshed.telemetry.occupied_world_unchanged);
  EXPECT_EQ(refreshed.telemetry.search_generation, initial.telemetry.search_generation);
  EXPECT_GE(refreshed.telemetry.records, initial.telemetry.records);
}

TEST(PersistentDStarLitePlanner3DTest,
     BoundedSearchPublishesTheSpatialIncumbentBeforeTimeOptimality) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 18, 18, 8});
  PersistentPlannerConfig3D config = testConfig();
  config.maximum_expansions_per_update = 4U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 1.5, 1.5};
  const Point3 goal{16.5, 16.5, 6.5};

  PlannerUpdate3D result = planner.plan(request(start, goal, world(occupancy, 1U)));
  ASSERT_EQ(result.progress, SearchProgress3D::kRunning);
  EXPECT_FALSE(result.improved_incumbent.has_value());
  const std::uint64_t generation = result.telemetry.search_generation;
  for (std::size_t attempt = 0U; attempt < 5000U && !result.publishable(); ++attempt) {
    result = planner.plan(request(start, goal, world(occupancy, 1U)));
  }

  ASSERT_TRUE(result.publishable())
      << "spatial_expansions=" << result.telemetry.expansions
      << " time_expansions=" << result.telemetry.execution_time_search_expansions
      << " time_records=" << result.telemetry.execution_time_search_records;
  EXPECT_TRUE(result.telemetry.search_state_reused);
  EXPECT_EQ(result.telemetry.search_generation, generation);
  EXPECT_FALSE(result.telemetry.execution_time_search_complete);
  EXPECT_EQ(result.progress, SearchProgress3D::kRunning);
  // Anytime publications are shortcut-simplified like converged ones; the
  // refinement still continues afterwards.
  expectRawValid(candidate(result).points, *occupancy,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     LongAnisotropicUnknownMissionPublishesAnAnytimeRouteWithinTheHealthBudget) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{-30.0, -30.0, 0.0, 0.25, 1380, 2100, 160});
  PersistentPlannerConfig3D config = testConfig();
  config.minimum_horizontal_step_m = 2.0;
  config.minimum_vertical_step_m = 1.0;
  config.maximum_adaptive_lattice_level = 2U;
  config.time_model.maximum_horizontal_speed_mps = 6.567;
  config.time_model.maximum_vertical_speed_mps = 5.0;
  config.time_model.maximum_translational_speed_mps = 6.567;
  config.goal_tolerance_m = 2.0;
  config.feasibility_first_enabled = true;
  config.maximum_compute_time_ms = 150.0;
  config.flight_envelope.minimum_target_z_m = 1.0;
  config.flight_envelope.maximum_target_z_m = 39.0;
  PersistentDStarLitePlanner3D planner{config};

  PlannerUpdate3D result = planner.plan(request(
      Point3{54.0, 54.0, 18.0}, Point3{216.0, 378.0, 18.0}, world(occupancy, 1U)));
  std::size_t slices = 1U;
  for (; slices < 30U && !result.publishable(); ++slices) {
    result = planner.plan(request(Point3{54.0, 54.0, 18.0}, Point3{216.0, 378.0, 18.0},
                                  world(occupancy, 1U)));
  }

  EXPECT_TRUE(result.publishable())
      << "slices=" << slices << " records=" << result.telemetry.records
      << " open=" << result.telemetry.open_entries
      << " expansions=" << result.telemetry.expansions
      << " time_expansions=" << result.telemetry.execution_time_search_expansions
      << " search_ms=" << result.telemetry.search_ms;
  // The anytime route comes from whichever search reaches the goal first
  // within the health budget: the feasibility-first search, or the persistent
  // search itself once its vertex maintenance leaves it enough of the budget.
  EXPECT_LE(slices, 10U);
  EXPECT_TRUE(result.telemetry.feasibility_attempted ||
              candidate(result).source ==
                  SpatialRouteCandidateSource3D::kExecutionTimeRefinement);
  if (candidate(result).source == SpatialRouteCandidateSource3D::kFeasibilitySearch) {
    EXPECT_TRUE(result.telemetry.feasibility_route_found);
    EXPECT_GT(result.telemetry.feasibility_expansions, 0U);
  }
  EXPECT_FALSE(result.telemetry.execution_time_search_complete);
  EXPECT_EQ(result.progress, SearchProgress3D::kRunning);
  expectRawValid(candidate(result).points, *occupancy,
                 planner.config().physical_footprint);
}

// A corridor crossed by a thin wall whose doorway is offset from the lattice
// rows: on a 2 m lattice over a 0.25 m map with a 0.5 m body, the rows lie at
// y = 1, 3 and 5, the wall stands at x in [8, 8.25) between the nodes at x = 7
// and x = 9, and the door spans y in [2.75, 4.25). The straight row segment at
// y = 3 clips the lower jamb, the diagonals clip a jamb too, and a lateral
// step to y = 3.5 at the wall clears it.
struct RefinedDoorwayFixture {
  static constexpr double kResolution = 0.25;
  static constexpr int kWidth = 64;  // 16 m
  static constexpr int kHeight = 24; // 6 m
  static constexpr int kDepth = 16;  // 4 m
  static constexpr int kWallX = 32;  // x in [8, 8.25)
  std::shared_ptr<ObservedOccupancyGrid3D> occupancy{
      std::make_shared<ObservedOccupancyGrid3D>(
          GridBounds3D{0.0, 0.0, 0.0, kResolution, kWidth, kHeight, kDepth})};
  PersistentPlannerConfig3D config{testConfig()};
  Point3 start{3.0, 3.0, 3.0};
  Point3 goal{13.0, 3.0, 3.0};
  // The door edge on the upper lattice level (z = 3).
  detail::PersistentPlannerNode3D door_west{3, 1, 1};
  detail::PersistentPlannerNode3D door_east{4, 1, 1};

  RefinedDoorwayFixture() {
    for (int y = 0; y < kHeight; ++y) {
      const double y_m = static_cast<double>(y) * kResolution;
      if (y_m >= 2.75 && y_m < 4.25) {
        continue;
      }
      for (int z = 0; z < kDepth; ++z) {
        if (!occupancy->setState({kWallX, y, z}, ObservedVoxelState::kOccupied)) {
          throw std::logic_error{"fixture cell outside the grid"};
        }
      }
    }
    config.minimum_horizontal_step_m = 2.0;
    config.minimum_vertical_step_m = 2.0;
    config.physical_footprint.radius_m = 0.5;
    config.physical_footprint.perimeter_samples = 8U;
    config.physical_footprint.radial_rings = 1U;
    config.physical_footprint.axial_samples = 1U;
    config.physical_footprint.sweep_step_m = 0.1;
    config.edge_refinement_offsets = 3U;
    config.maximum_edge_refinement_probes = 256U;
  }

  // Cells above the upper level's waypoint at (8, 3.5, 3): x in [7.75, 8.25),
  // y in [3.75, 4.25), z in [2.75, 3.25). They touch the body at the waypoint
  // and on both legs, and their centres lie at least 0.875 m from the straight
  // row segment at y = 3 against a body margin of 0.5 m plus a half voxel
  // diagonal. The lower level's door at z = 1 is untouched.
  [[nodiscard]] static std::vector<GridIndex3D> cellsAboveTheUpperWaypoint() {
    std::vector<GridIndex3D> cells;
    for (const int x : {31, 32}) {
      for (const int y : {15, 16}) {
        for (const int z : {11, 12}) {
          cells.push_back({x, y, z});
        }
      }
    }
    return cells;
  }

  [[nodiscard]] static std::shared_ptr<ObservedOccupancyGrid3D>
  withCells(const ObservedOccupancyGrid3D& base,
            const std::vector<GridIndex3D>& cells) {
    auto changed = std::make_shared<ObservedOccupancyGrid3D>(base);
    for (const GridIndex3D cell : cells) {
      if (!changed->setState(cell, ObservedVoxelState::kOccupied)) {
        throw std::logic_error{"changed cell outside the grid"};
      }
    }
    return changed;
  }

  [[nodiscard]] static std::vector<OccupancyChunkIndex3D>
  chunksOf(const std::vector<GridIndex3D>& cells) {
    std::vector<OccupancyChunkIndex3D> chunks;
    for (const GridIndex3D cell : cells) {
      chunks.push_back(ObservedOccupancyGrid3D::chunkIndex(cell));
    }
    return chunks;
  }

  [[nodiscard]] static PlannerUpdate3D
  planUntilPublishable(PersistentDStarLitePlanner3D& planner,
                       const PersistentPlannerRequest3D& req) {
    PlannerUpdate3D update = planner.plan(req);
    for (int attempt = 0; attempt < 8 && !update.publishable(); ++attempt) {
      update = planner.plan(req);
    }
    return update;
  }
};

TEST(PersistentDStarLitePlanner3DTest, ARouteThroughARefinedEdgeCarriesItsWaypoint) {
  RefinedDoorwayFixture fixture;

  PersistentPlannerConfig3D unrefined_config = fixture.config;
  unrefined_config.edge_refinement_offsets = 0U;
  PersistentDStarLitePlanner3D unrefined{unrefined_config};
  const PlannerUpdate3D stuck = fixture.planUntilPublishable(
      unrefined, request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
  ASSERT_FALSE(stuck.publishable()) << "the fixture no longer needs a refined edge";

  PersistentDStarLitePlanner3D planner{fixture.config};
  const PlannerUpdate3D update = fixture.planUntilPublishable(
      planner, request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
  ASSERT_TRUE(update.publishable())
      << "edge refinement did not open the doorway: input="
      << static_cast<int>(update.input_status)
      << " expansions=" << update.telemetry.expansions
      << " time_expansions=" << update.telemetry.execution_time_search_expansions;
  const std::vector<Point3>& points = candidate(update).points;
  // The waypoint is off the lattice row and inside the door.
  const bool carries_waypoint = std::ranges::any_of(points, [](const Point3& point) {
    return point.x > 7.5 && point.x < 8.5 && point.y > 3.1 && point.y < 4.0;
  });
  EXPECT_TRUE(carries_waypoint);
  expectRawValid(points, *fixture.occupancy, planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest, ARejectedSegmentNamesTheEdgeThatAdmittedIt) {
  // The raw sweep is the authority on a candidate, and a leg it rejects has
  // to reach the cache entry that admitted it: a straight edge, or a refined
  // edge through its waypoint. Rejecting the edge forgets it and records it
  // for the persistent session's repair.
  RefinedDoorwayFixture fixture;
  detail::PlannerLattice3D lattice{fixture.config};
  const PersistentPlannerWorld3D initial = world(fixture.occupancy, 1U);
  lattice.configureGridGeometry(*initial.bounds());
  lattice.installWorld(initial);
  ASSERT_TRUE(
      std::isfinite(lattice.rankedEdgeCost(fixture.door_west, fixture.door_east)));
  const std::optional<Point3> waypoint =
      lattice.edgeWaypoint(fixture.door_west, fixture.door_east);
  ASSERT_TRUE(waypoint.has_value()) << "the doorway edge was not refined";
  EXPECT_NEAR(waypoint->x, 8.0, 1.0e-9);
  EXPECT_NEAR(waypoint->y, 3.5, 1.0e-9);
  const Point3 west = lattice.pointFor(fixture.door_west);
  const Point3 east = lattice.pointFor(fixture.door_east);

  const std::vector<Point3> through_waypoint{fixture.start, west, *waypoint, east};
  for (const std::size_t leg : {2U, 3U}) {
    const std::optional<detail::PlannerLattice3D::PathSegmentEdge3D> priced =
        lattice.pricedEdgeForSegment(through_waypoint, leg);
    ASSERT_TRUE(priced.has_value()) << "leg " << leg;
    EXPECT_EQ(priced->from, fixture.door_west);
    EXPECT_EQ(priced->to, fixture.door_east);
  }
  // The departure from the exact start is not a lattice edge.
  EXPECT_FALSE(
      lattice.pricedEdgeForSegment({Point3{6.4, 3.2, 3.0}, west}, 1U).has_value());
  // A straight edge names itself.
  const std::optional<detail::PlannerLattice3D::PathSegmentEdge3D> straight =
      lattice.pricedEdgeForSegment({west, lattice.pointFor({2, 1, 1})}, 1U);
  ASSERT_TRUE(straight.has_value());
  EXPECT_EQ(straight->from, fixture.door_west);
  EXPECT_EQ(straight->to, (detail::PersistentPlannerNode3D{2, 1, 1}));

  const detail::PersistentPlannerEdge3D edge =
      detail::canonicalEdge(fixture.door_west, fixture.door_east);
  EXPECT_TRUE(lattice.rejectEdgeBySweep(edge));
  EXPECT_FALSE(lattice.edgeWaypoint(fixture.door_west, fixture.door_east).has_value());
  EXPECT_FALSE(lattice.hasEdgeCost(edge));
  EXPECT_FALSE(lattice.rejectEdgeBySweep(edge)) << "nothing was cached to reject";
  const std::vector<detail::PersistentPlannerEdge3D> recorded =
      lattice.takeSweepRejectedEdges();
  ASSERT_EQ(recorded.size(), 1U);
  EXPECT_EQ(recorded.front(), edge);
  EXPECT_TRUE(lattice.takeSweepRejectedEdges().empty());
}

TEST(PersistentDStarLitePlanner3DTest,
     AChangeBesideARefinedEdgesWaypointForgetsTheEdge) {
  // The change scheduling tests the geometry the vehicle flies. A cell that
  // touches a refined edge's leg through its waypoint, but lies farther from
  // the straight node-to-node segment than the body margin, has to forget the
  // edge all the same; otherwise the cache keeps admitting a leg the sweep
  // rejects, and every search on the lattice rediscovers the same blocked
  // route for as long as the cell stays.
  RefinedDoorwayFixture fixture;
  detail::PlannerLattice3D lattice{fixture.config};
  const PersistentPlannerWorld3D initial = world(fixture.occupancy, 1U);
  lattice.configureGridGeometry(*initial.bounds());
  lattice.installWorld(initial);
  ASSERT_TRUE(
      std::isfinite(lattice.rankedEdgeCost(fixture.door_west, fixture.door_east)));
  const std::optional<Point3> waypoint =
      lattice.edgeWaypoint(fixture.door_west, fixture.door_east);
  ASSERT_TRUE(waypoint.has_value()) << "the doorway edge was not refined";

  const std::vector<GridIndex3D> cells =
      RefinedDoorwayFixture::cellsAboveTheUpperWaypoint();
  const PersistentPlannerWorld3D after =
      world(RefinedDoorwayFixture::withCells(*fixture.occupancy, cells), 2U,
            RefinedDoorwayFixture::chunksOf(cells));
  lattice.installWorld(after);
  const Point3 west = lattice.pointFor(fixture.door_west);
  const Point3 east = lattice.pointFor(fixture.door_east);
  ASSERT_FALSE(lattice.pathTraversable({west, *waypoint, east}))
      << "the cells do not block the refined legs";
  ASSERT_TRUE(lattice.edgeWaypoint(fixture.door_west, fixture.door_east).has_value())
      << "installing the world must not forget edges by itself";

  detail::DStarLiteSession3D session{fixture.config, lattice};
  std::size_t affected_states = 0U;
  session.scheduleAffectedVertices(after, cells, affected_states);
  EXPECT_FALSE(lattice.edgeWaypoint(fixture.door_west, fixture.door_east).has_value());
  EXPECT_FALSE(
      lattice.hasEdgeCost(detail::canonicalEdge(fixture.door_west, fixture.door_east)));
}

TEST(PersistentDStarLitePlanner3DTest,
     AChangeBesideTheWaypointReroutesWithoutRestartingTheFeasibilitySearch) {
  // The doorway is refined on both lattice levels. Cells that block the upper
  // level's waypoint leave the lower one: the searches have to re-route
  // through it from their labels, not restart from nothing and rediscover
  // the blocked edge from the cache on every update.
  RefinedDoorwayFixture fixture;
  fixture.config.feasibility_first_enabled = true;
  PersistentDStarLitePlanner3D planner{fixture.config};
  PlannerUpdate3D update = fixture.planUntilPublishable(
      planner, request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
  ASSERT_TRUE(update.publishable());

  const std::vector<GridIndex3D> cells =
      RefinedDoorwayFixture::cellsAboveTheUpperWaypoint();
  const std::shared_ptr<ObservedOccupancyGrid3D> changed =
      RefinedDoorwayFixture::withCells(*fixture.occupancy, cells);
  PersistentPlannerRequest3D blocked =
      request(fixture.start, fixture.goal,
              world(changed, 2U, RefinedDoorwayFixture::chunksOf(cells)));
  // The consumer could not enter the route it held, so the feasibility
  // search runs.
  blocked.incumbent_rejection_sequence = 1U;
  std::size_t restarts = 0U;
  for (int attempt = 0; attempt < 12; ++attempt) {
    update = planner.plan(blocked);
    restarts = std::max(restarts, update.telemetry.feasibility_restarts);
    if (update.publishable()) {
      break;
    }
  }
  ASSERT_TRUE(update.publishable())
      << "no route below the blocked waypoint: restarts=" << restarts
      << " closest_goal_m=" << update.telemetry.feasibility_closest_goal_distance_m
      << " exhausted=" << update.telemetry.feasibility_frontier_exhausted;
  EXPECT_EQ(restarts, 0U) << "the search restarted instead of forgetting the edge";
  const std::vector<Point3>& points = candidate(update).points;
  expectRawValid(points, *changed, planner.config().physical_footprint);
  EXPECT_TRUE(std::ranges::any_of(points, [](const Point3& point) {
    return point.x > 7.5 && point.x < 8.5 && point.z < 2.6;
  })) << "the route does not pass the door below the blocked cells";
}

TEST(PersistentDStarLitePlanner3DTest,
     RepairRunsWhileTheFeasibilitySearchLooksForAFirstRoute) {
  // No route is held and the world just changed: the feasibility search needs
  // most of the update, and the persistent session's repair queue is what
  // makes its labels true again. The repair has to get a share of the update
  // before the feasibility search takes the rest, or the session reports a
  // shortest path on labels the world has moved for as long as no route exists.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 40, 40, 3});
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.maximum_compute_time_ms = 20.0;
  config.maximum_feasibility_compute_time_ms = 19.0;
  config.maximum_feasibility_expansions_per_update = 1U << 20U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 20.5, 1.5};
  const Point3 goal{38.5, 20.5, 1.5};
  PlannerUpdate3D update = planner.plan(request(start, goal, world(occupancy, 1U)));
  for (int attempt = 0; attempt < 8 && !update.publishable(); ++attempt) {
    update = planner.plan(request(start, goal, world(occupancy, 1U)));
  }
  ASSERT_TRUE(update.publishable())
      << "input=" << static_cast<int>(update.input_status)
      << " feasibility_expansions=" << update.telemetry.feasibility_expansions
      << " expansions=" << update.telemetry.expansions
      << " closest_goal_m=" << update.telemetry.feasibility_closest_goal_distance_m;

  // Enclose the goal: the labels around it move, and no route exists, so the
  // feasibility search runs to its deadline on every update.
  auto enclosed = std::make_shared<ObservedOccupancyGrid3D>(*occupancy);
  std::vector<OccupancyChunkIndex3D> dirty;
  for (int x = 35; x < 40; ++x) {
    for (int y = 17; y < 24; ++y) {
      if (x == 35 || y == 17 || y == 23) {
        for (int z = 0; z < 3; ++z) {
          const GridIndex3D cell{x, y, z};
          ASSERT_TRUE(enclosed->setState(cell, ObservedVoxelState::kOccupied));
          dirty.push_back(ObservedOccupancyGrid3D::chunkIndex(cell));
        }
      }
    }
  }
  PersistentPlannerRequest3D blocked =
      request(start, goal, world(enclosed, 2U, std::move(dirty)));
  blocked.incumbent_rejection_sequence = 1U;
  update = planner.plan(blocked);
  ASSERT_TRUE(update.telemetry.feasibility_attempted);
  EXPECT_GT(update.telemetry.repair_lattice_states_processed, 0U)
      << "pending=" << update.telemetry.repair_lattice_states_pending
      << " feasibility_ms=" << update.telemetry.feasibility_ms
      << " repair_ms=" << update.telemetry.repair_ms;
}

TEST(PersistentDStarLitePlanner3DTest, TheStartAnchorIsKeptWhileItStaysAdmissible) {
  // A vehicle drifting between two nodes keeps the anchor its searches were
  // seeded from; re-anchoring on every flip of the nearest node would restart
  // them for nothing. The walk over the anchors on exhaustion still wins.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 5, 3});
  PersistentPlannerConfig3D config = testConfig();
  config.minimum_horizontal_step_m = 2.0;
  detail::PlannerLattice3D lattice{config};
  const PersistentPlannerWorld3D open = world(occupancy, 1U);
  lattice.configureGridGeometry(*open.bounds());
  lattice.installWorld(open);
  const detail::PersistentPlannerNode3D first{1, 1, 1};
  const detail::PersistentPlannerNode3D next{2, 1, 1};
  ASSERT_NEAR(lattice.pointFor(first).x, 3.0, 1.0e-9);
  ASSERT_NEAR(lattice.pointFor(next).x, 5.0, 1.0e-9);

  const Point3 nearer_the_next{4.2, 3.0, 1.5};
  const detail::PlannerLattice3D::DepartureConnection3D nearest =
      lattice.selectDepartureConnection(nearer_the_next);
  ASSERT_TRUE(nearest.available());
  EXPECT_EQ(*nearest.anchor, next);

  const detail::PlannerLattice3D::DepartureConnection3D kept =
      lattice.selectDepartureConnection(nearer_the_next, 0U, first);
  ASSERT_TRUE(kept.available());
  EXPECT_EQ(*kept.anchor, first) << "the anchor flipped to the nearest node";

  const detail::PlannerLattice3D::DepartureConnection3D walked =
      lattice.selectDepartureConnection(nearer_the_next, 1U, first);
  const detail::PlannerLattice3D::DepartureConnection3D walked_unpreferred =
      lattice.selectDepartureConnection(nearer_the_next, 1U);
  ASSERT_TRUE(walked.available());
  ASSERT_TRUE(walked_unpreferred.available());
  EXPECT_EQ(*walked.anchor, *walked_unpreferred.anchor)
      << "the preference must not alter the walk on exhaustion";

  // A preferred anchor the body no longer reaches is not kept.
  auto walled = std::make_shared<ObservedOccupancyGrid3D>(*occupancy);
  for (int y = 0; y < 5; ++y) {
    for (int z = 0; z < 3; ++z) {
      ASSERT_TRUE(walled->setState({3, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  lattice.installWorld(world(walled, 2U));
  const detail::PlannerLattice3D::DepartureConnection3D released =
      lattice.selectDepartureConnection(nearer_the_next, 0U, first);
  ASSERT_TRUE(released.available());
  EXPECT_EQ(*released.anchor, next);
}

// Two rooms joined by a corridor 1.5 m wide whose centreline lies half a
// lattice step off the lattice rows: on a 2 m lattice over a 0.25 m map with a
// 0.5 m body, no node inside the corridor is valid and no straight lattice
// edge crosses it, so the graph has no path from one room to the other. The
// corridor is wide enough for the body, and a vehicle in the first room has a
// way out at the body's own scale.
struct NodelessCorridorFixture {
  static constexpr double kResolution = 0.25;
  static constexpr int kWidth = 64;  // 16 m
  static constexpr int kHeight = 24; // 6 m
  static constexpr int kDepth = 8;   // 2 m
  std::shared_ptr<ObservedOccupancyGrid3D> occupancy{
      std::make_shared<ObservedOccupancyGrid3D>(
          GridBounds3D{0.0, 0.0, 0.0, kResolution, kWidth, kHeight, kDepth})};
  PersistentPlannerConfig3D config{testConfig()};
  Point3 start{3.0, 3.5, 1.0};
  Point3 goal{13.0, 3.0, 1.0};

  NodelessCorridorFixture() {
    // Walls between the rooms, x in [6, 10), except the corridor y in
    // [2.75, 4.25).
    for (int x = 24; x < 40; ++x) {
      for (int y = 0; y < kHeight; ++y) {
        const double y_m = static_cast<double>(y) * kResolution;
        if (y_m >= 2.75 && y_m < 4.25) {
          continue;
        }
        for (int z = 0; z < kDepth; ++z) {
          if (!occupancy->setState({x, y, z}, ObservedVoxelState::kOccupied)) {
            throw std::logic_error{"fixture cell outside the grid"};
          }
        }
      }
    }
    config.minimum_horizontal_step_m = 2.0;
    config.minimum_vertical_step_m = 2.0;
    config.physical_footprint.radius_m = 0.5;
    config.physical_footprint.perimeter_samples = 8U;
    config.physical_footprint.radial_rings = 1U;
    config.physical_footprint.axial_samples = 1U;
    config.physical_footprint.sweep_step_m = 0.1;
    config.flight_envelope.maximum_target_z_m = 2.0;
    config.feasibility_first_enabled = true;
    config.maximum_compute_time_ms = 200.0;
    config.maximum_feasibility_compute_time_ms = 100.0;
    config.departure_refinement_subdivisions = 4U;
    config.escape_search_radius_cells = 4U;
    config.escape_search_maximum_probes_per_update = 1U << 16U;
  }
};

TEST(PersistentDStarLitePlanner3DTest,
     AnExhaustedFrontierIsReportedAndWalksTheAnchors) {
  // Without an escape search a closed component is terminal, and the planner
  // has to say so: the exhaustion survives the search's own restart, and the
  // walk over the start's anchors advances on it.
  NodelessCorridorFixture fixture;
  fixture.config.escape_search_radius_cells = 0U;
  PersistentDStarLitePlanner3D planner{fixture.config};
  bool exhausted = false;
  std::size_t skip = 0U;
  PlannerUpdate3D update;
  for (int attempt = 0; attempt < 12; ++attempt) {
    update = planner.plan(
        request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
    exhausted = exhausted || update.telemetry.feasibility_frontier_exhausted;
    skip = std::max(skip, update.telemetry.departure_anchor_skip);
  }
  EXPECT_FALSE(update.publishable());
  EXPECT_TRUE(exhausted) << "the exhaustion was hidden by the restart";
  EXPECT_GT(skip, 0U) << "the anchor walk never advanced";
  EXPECT_FALSE(update.telemetry.escape_search_attempted);
}

TEST(PersistentDStarLitePlanner3DTest,
     TheEscapeSearchLeavesAClosedComponentThroughTheCorridor) {
  NodelessCorridorFixture fixture;
  PersistentDStarLitePlanner3D planner{fixture.config};
  bool exhausted = false;
  bool escape_found = false;
  PlannerUpdate3D update;
  for (int attempt = 0; attempt < 40; ++attempt) {
    update = planner.plan(
        request(fixture.start, fixture.goal, world(fixture.occupancy, 1U)));
    exhausted = exhausted || update.telemetry.feasibility_frontier_exhausted;
    escape_found = escape_found || update.telemetry.escape_search_found;
    if (update.publishable()) {
      break;
    }
  }
  EXPECT_TRUE(exhausted);
  EXPECT_TRUE(escape_found) << "explored="
                            << update.telemetry.escape_search_explored_cells
                            << " exhausted="
                            << update.telemetry.escape_search_exhausted;
  ASSERT_TRUE(update.publishable()) << "no route through the corridor";
  EXPECT_TRUE(update.telemetry.escape_connection_active);
  EXPECT_TRUE(update.telemetry.departure_waypoint_used);
  EXPECT_GT(update.telemetry.departure_waypoint_count, 1U);
  const std::vector<Point3>& points = candidate(update).points;
  ASSERT_GE(points.size(), 2U);
  EXPECT_NEAR(points.front().x, fixture.start.x, 1.0e-9);
  EXPECT_NEAR(points.back().x, fixture.goal.x, 1.0e-9);
  EXPECT_TRUE(std::ranges::any_of(points, [](const Point3& point) {
    return point.x > 6.0 && point.x < 10.0 && point.y > 2.9 && point.y < 4.1;
  })) << "the route does not pass the corridor";
  expectRawValid(points, *fixture.occupancy, planner.config().physical_footprint);
}

} // namespace
} // namespace drone_city_nav
