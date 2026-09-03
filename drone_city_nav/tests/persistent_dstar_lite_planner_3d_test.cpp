#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] PersistentPlannerConfig3D testConfig() {
  PersistentPlannerConfig3D config;
  config.minimum_horizontal_step_m = 1.0;
  config.minimum_vertical_step_m = 1.0;
  config.time_model.maximum_horizontal_speed_mps = 5.0;
  config.time_model.maximum_vertical_speed_mps = 2.0;
  config.goal_tolerance_m = 0.01;
  config.feasibility_first_enabled = false;
  config.maximum_compute_time_ms = 1000.0;
  config.maximum_expansions_per_update = 100000U;
  config.physical_footprint.radius_m = 0.0;
  config.physical_footprint.lower_extent_m = 0.0;
  config.physical_footprint.upper_extent_m = 0.0;
  config.physical_footprint.perimeter_samples = 0U;
  config.physical_footprint.radial_rings = 0U;
  config.physical_footprint.axial_samples = 0U;
  config.physical_footprint.sweep_step_m = 0.2;
  config.flight_envelope.minimum_target_z_m = 0.0;
  config.flight_envelope.maximum_target_z_m = 20.0;
  return config;
}

[[nodiscard]] PersistentPlannerWorld3D
world(std::shared_ptr<const ObservedOccupancyGrid3D> occupancy,
      const std::uint64_t revision,
      std::vector<OccupancyChunkIndex3D> dirty_chunks = {},
      const bool full_reset = false) {
  const OccupancyGrid3D occupied = occupancy->occupiedSnapshot();
  return PersistentPlannerWorld3D{
      .observed_occupancy = std::move(occupancy),
      .static_occupancy = nullptr,
      .proprioceptive_free_space_seed = std::nullopt,
      .launch_support_contact = std::nullopt,
      .dirty_chunks = std::move(dirty_chunks),
      .producer_instance_id = 17U,
      .revision = revision,
      .incremental_parent_revision = revision > 1U ? revision - 1U : 0U,
      .occupied_fingerprint = occupied.contentFingerprint(),
      .full_reset = full_reset,
  };
}

[[nodiscard]] PersistentPlannerRequest3D
request(const Point3& start, const Point3& goal, PersistentPlannerWorld3D raw_world,
        const std::uint64_t mission_epoch = 3U) {
  return PersistentPlannerRequest3D{
      .start = start,
      .velocity = {},
      .mission_goal = goal,
      .mission_epoch = mission_epoch,
      .world = std::move(raw_world),
  };
}

[[nodiscard]] const SpatialRouteCandidate3D& candidate(const PlannerUpdate3D& update) {
  if (!update.improved_incumbent.has_value()) {
    throw std::logic_error{"planner update has no improved incumbent"};
  }
  return *update.improved_incumbent;
}

void expectSamePath(const std::vector<Point3>& first,
                    const std::vector<Point3>& second) {
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0U; index < first.size(); ++index) {
    EXPECT_NEAR(first[index].x, second[index].x, 1.0e-12);
    EXPECT_NEAR(first[index].y, second[index].y, 1.0e-12);
    EXPECT_NEAR(first[index].z, second[index].z, 1.0e-12);
  }
}

void expectRawValid(const std::vector<Point3>& path,
                    const ObservedOccupancyGrid3D& occupancy,
                    const SweptFootprintConfig& footprint) {
  ASSERT_GE(path.size(), 2U);
  for (std::size_t index = 1U; index < path.size(); ++index) {
    EXPECT_TRUE(validateRawSweptFootprint(occupancy, path[index - 1U],
                                          FootprintBodyAxis{}, path[index],
                                          FootprintBodyAxis{}, footprint)
                    .accepted());
  }
}

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
     TheFeasibilityFirstRouteKeepsItsBodyOutOfTheCriticalBand) {
  // A known floor two cells thick under a corridor: the unranked feasibility
  // search would fly the lowest level right above it; ranked within its short
  // reach, it lifts the interior of the route clear of the critical band.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 3, 8});
  for (int x = 0; x < 14; ++x) {
    for (int y = 0; y < 3; ++y) {
      for (int z = 0; z < 2; ++z) {
        ASSERT_TRUE(occupancy->setState({x, y, z}, ObservedVoxelState::kOccupied));
      }
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.clearance_ranking_weight = 1.5;
  config.clearance_ranking_distance_m = 6.0;
  config.clearance_ranking_critical_distance_m = 1.0;
  config.clearance_ranking_critical_weight = 100.0;
  config.feasibility_clearance_ranking_distance_m = 2.0;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 1.5, 2.5};
  const Point3 goal{12.5, 1.5, 2.5};
  PlannerUpdate3D update = planner.plan(request(start, goal, world(occupancy, 1U)));
  for (int attempt = 0; attempt < 8 && !update.publishable(); ++attempt) {
    update = planner.plan(request(start, goal, world(occupancy, 1U)));
  }
  ASSERT_TRUE(update.publishable());
  EXPECT_TRUE(update.telemetry.feasibility_route_found);
  const std::vector<Point3>& points = candidate(update).points;
  ASSERT_GE(points.size(), 3U);
  double interior_minimum_z = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1U; index + 1U < points.size(); ++index) {
    interior_minimum_z = std::min(interior_minimum_z, points[index].z);
  }
  // Node centres at z = 2.5 sit half a metre above the floor's top at 2.0;
  // the ranked search climbs at least one level to leave the critical band.
  EXPECT_GE(interior_minimum_z, 3.4) << "interior minimum z " << interior_minimum_z;
  expectRawValid(points, *occupancy, planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     TheRankedFeasibilitySearchStaysDirectedAcrossAnOpenWorld) {
  // A ranking curve that charged open space would loosen the unranked
  // heuristic and turn the first search into a breadth-first sweep; scaled to
  // its reach it prices only the band near occupied evidence, so a far goal
  // across an open world is reached within a few thousand explored nodes.
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 120, 120, 16});
  for (int x = 0; x < 120; ++x) {
    for (int y = 0; y < 120; ++y) {
      ASSERT_TRUE(occupancy->setState({x, y, 0}, ObservedVoxelState::kOccupied));
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.clearance_ranking_weight = 1.5;
  config.clearance_ranking_distance_m = 6.0;
  config.feasibility_clearance_ranking_distance_m = 2.0;
  config.maximum_compute_time_ms = 2000.0;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{2.5, 2.5, 5.5};
  const Point3 goal{117.5, 117.5, 5.5};
  PlannerUpdate3D update = planner.plan(request(start, goal, world(occupancy, 1U)));
  for (int attempt = 0; attempt < 4 && !update.publishable(); ++attempt) {
    update = planner.plan(request(start, goal, world(occupancy, 1U)));
  }
  ASSERT_TRUE(update.publishable());
  EXPECT_TRUE(update.telemetry.feasibility_route_found);
  EXPECT_LT(update.telemetry.feasibility_explored_nodes, 60000U)
      << "explored " << update.telemetry.feasibility_explored_nodes;
  expectRawValid(candidate(update).points, *occupancy,
                 planner.config().physical_footprint);
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
     PreparatoryVerticalMotionPassesThroughTheOnlyLowerOpening) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 10, 8});
  for (int z = 3; z < 8; ++z) {
    for (int y = 0; y < 10; ++y) {
      static_cast<void>(
          occupancy->setState(GridIndex3D{6, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.maximum_feasibility_expansions_per_update = 100000U;
  config.maximum_feasibility_compute_time_ms = 900.0;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{2.5, 5.5, 5.5};
  const Point3 goal{11.5, 5.5, 5.5};

  const PlannerUpdate3D result =
      planner.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(result.publishable());
  EXPECT_TRUE(result.telemetry.feasibility_route_found);
  EXPECT_LT(std::ranges::min_element(candidate(result).points, {}, &Point3::z)->z,
            start.z);
  expectRawValid(candidate(result).points, *occupancy,
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
     FeasibilityIncumbentDoesNotStopTheAnytimeSessionBeforeConvergence) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 18, 18, 8});
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.maximum_feasibility_expansions_per_update = 64U;
  config.maximum_expansions_per_update = 4U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 1.5, 1.5};
  const Point3 goal{16.5, 16.5, 6.5};

  PlannerUpdate3D update = planner.plan(request(start, goal, world(occupancy, 1U)));
  ASSERT_TRUE(update.publishable());
  ASSERT_EQ(candidate(update).source,
            SpatialRouteCandidateSource3D::kFeasibilitySearch);
  ASSERT_EQ(update.progress, SearchProgress3D::kRunning);
  double best_objective_s = candidate(update).estimated_execution_time_s;
  std::size_t refinement_expansions = update.telemetry.execution_time_search_expansions;

  for (std::size_t continuation = 0U;
       continuation < 5000U && update.progress == SearchProgress3D::kRunning;
       ++continuation) {
    update = planner.plan(request(start, goal, world(occupancy, 1U)));
    refinement_expansions += update.telemetry.execution_time_search_expansions;
    if (update.publishable()) {
      EXPECT_LT(candidate(update).estimated_execution_time_s, best_objective_s);
      best_objective_s = candidate(update).estimated_execution_time_s;
    }
  }

  EXPECT_EQ(update.progress, SearchProgress3D::kConverged);
  EXPECT_TRUE(update.telemetry.execution_time_search_complete);
  EXPECT_TRUE(update.telemetry.incumbent_available);
  EXPECT_GT(refinement_expansions, 0U);
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
  EXPECT_TRUE(result.telemetry.feasibility_attempted);
  EXPECT_TRUE(result.telemetry.feasibility_route_found);
  EXPECT_GT(result.telemetry.feasibility_expansions, 0U);
  EXPECT_FALSE(result.telemetry.execution_time_search_complete);
  EXPECT_EQ(result.progress, SearchProgress3D::kRunning);
  expectRawValid(candidate(result).points, *occupancy,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     FeasibilityFirstSearchUsesTheFull3DTimeObjectiveForARawValidDetour) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 30, 20, 5});
  for (int z = 0; z < 5; ++z) {
    for (int y = 3; y < 20; ++y) {
      ASSERT_TRUE(
          occupancy->setState(GridIndex3D{12, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.physical_footprint.radius_m = 0.4;
  config.physical_footprint.lower_extent_m = 0.2;
  config.physical_footprint.upper_extent_m = 0.2;
  config.physical_footprint.perimeter_samples = 8U;
  config.physical_footprint.radial_rings = 1U;
  config.physical_footprint.axial_samples = 3U;
  config.physical_footprint.sweep_step_m = 0.25;
  const Point3 start{2.5, 3.5, 2.5};
  const Point3 goal{27.5, 17.5, 2.5};
  ASSERT_FALSE(validateRawSweptFootprint(*occupancy, start, FootprintBodyAxis{}, goal,
                                         FootprintBodyAxis{}, config.physical_footprint)
                   .accepted());

  PersistentDStarLitePlanner3D planner{config};
  const PlannerUpdate3D result =
      planner.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(result.publishable());
  EXPECT_TRUE(result.telemetry.feasibility_attempted);
  EXPECT_TRUE(result.telemetry.feasibility_route_found)
      << "feasibility_expansions=" << result.telemetry.feasibility_expansions
      << " raw_checks=" << result.telemetry.raw_edge_validation_checks
      << " graph_expansions=" << result.telemetry.expansions
      << " records=" << result.telemetry.records
      << " open=" << result.telemetry.open_entries
      << " search_ms=" << result.telemetry.search_ms;
  EXPECT_GT(result.telemetry.feasibility_expansions, 1U);
  EXPECT_GT(candidate(result).points.size(), 2U);
  expectRawValid(candidate(result).points, *occupancy,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     ExhaustedFeasibilitySliceStillAdvancesTheResidentOptimalSearch) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 16, 5});
  for (int z = 0; z < 5; ++z) {
    for (int y = 2; y < 16; ++y) {
      ASSERT_TRUE(
          occupancy->setState(GridIndex3D{11, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  config.maximum_feasibility_expansions_per_update = 1U;
  config.maximum_expansions_per_update = 4U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{2.5, 3.5, 2.5};
  const Point3 goal{21.5, 13.5, 2.5};

  const PlannerUpdate3D initial =
      planner.plan(request(start, goal, world(occupancy, 1U)));
  ASSERT_EQ(initial.progress, SearchProgress3D::kRunning);
  EXPECT_TRUE(initial.telemetry.feasibility_attempted);
  EXPECT_FALSE(initial.telemetry.feasibility_route_found);
  EXPECT_EQ(initial.telemetry.feasibility_expansions, 1U);
  EXPECT_GT(initial.telemetry.expansions, 0U);

  const PlannerUpdate3D continued =
      planner.plan(request(start, goal, world(occupancy, 1U)));
  EXPECT_TRUE(continued.telemetry.search_state_reused);
  EXPECT_EQ(continued.telemetry.search_generation, initial.telemetry.search_generation);
  EXPECT_GE(continued.telemetry.records, initial.telemetry.records);
  EXPECT_GT(continued.telemetry.expansions, 0U);
}

TEST(PersistentDStarLitePlanner3DTest,
     FeasibilityFirstFrontierSurvivesBoundedContinuationSlices) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 32, 24, 5});
  for (int z = 0; z < 5; ++z) {
    for (int y = 0; y <= 20; ++y) {
      ASSERT_TRUE(
          occupancy->setState(GridIndex3D{14, y, z}, ObservedVoxelState::kOccupied));
    }
  }
  PersistentPlannerConfig3D config = testConfig();
  config.maximum_adaptive_lattice_level = 0U;
  config.feasibility_first_enabled = true;
  config.maximum_feasibility_expansions_per_update = 64U;
  config.maximum_expansions_per_update = 1U;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{2.5, 4.5, 2.5};
  const Point3 goal{29.5, 4.5, 2.5};

  PlannerUpdate3D result;
  for (std::size_t slice = 0U;
       slice < 128U && !result.telemetry.feasibility_route_found; ++slice) {
    result = planner.plan(request(start, goal, world(occupancy, 1U)));
  }

  ASSERT_TRUE(result.telemetry.feasibility_route_found)
      << "records=" << result.telemetry.records
      << " open=" << result.telemetry.open_entries;
  ASSERT_TRUE(result.publishable());
  EXPECT_EQ(candidate(result).source,
            SpatialRouteCandidateSource3D::kFeasibilitySearch);
  EXPECT_TRUE(result.telemetry.search_state_reused);
  EXPECT_GT(candidate(result).points.size(), 2U);
  expectRawValid(candidate(result).points, *occupancy,
                 planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     FeasibilityFirstSearchReplacesAnIncumbentBlockedByNewRawEvidence) {
  auto initial_occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 22, 12, 5});
  PersistentPlannerConfig3D config = testConfig();
  config.feasibility_first_enabled = true;
  PersistentDStarLitePlanner3D planner{config};
  const Point3 start{1.5, 5.5, 2.5};
  const Point3 goal{20.5, 5.5, 2.5};
  const PlannerUpdate3D initial =
      planner.plan(request(start, goal, world(initial_occupancy, 1U)));
  ASSERT_TRUE(initial.publishable());
  ASSERT_TRUE(initial.telemetry.feasibility_route_found);

  auto changed = std::make_shared<ObservedOccupancyGrid3D>(*initial_occupancy);
  const GridIndex3D obstacle{11, 5, 2};
  ASSERT_TRUE(changed->setState(obstacle, ObservedVoxelState::kOccupied));
  const PlannerUpdate3D replacement = planner.plan(
      request(start, goal,
              world(changed, 2U, {ObservedOccupancyGrid3D::chunkIndex(obstacle)})));

  ASSERT_TRUE(replacement.publishable());
  EXPECT_FALSE(replacement.telemetry.incumbent_retained);
  EXPECT_TRUE(replacement.telemetry.feasibility_attempted);
  EXPECT_TRUE(replacement.telemetry.feasibility_route_found);
  EXPECT_GT(candidate(replacement).points.size(), candidate(initial).points.size());
  expectRawValid(candidate(replacement).points, *changed,
                 planner.config().physical_footprint);
}

} // namespace

TEST(PersistentDStarLitePlanner3DTest,
     TheFeasibilitySearchLeavesHalfTheBudgetToAPersistentSearchWithWork) {
  using std::chrono::milliseconds;
  EXPECT_EQ(feasibilitySearchBudget3D(milliseconds{150}, milliseconds{140}, true),
            milliseconds{75});
  EXPECT_EQ(feasibilitySearchBudget3D(milliseconds{150}, milliseconds{140}, false),
            milliseconds{140});
  EXPECT_EQ(feasibilitySearchBudget3D(milliseconds{150}, milliseconds{40}, true),
            milliseconds{40});
  EXPECT_EQ(feasibilitySearchBudget3D(milliseconds{-5}, milliseconds{140}, true),
            milliseconds{0});
  EXPECT_EQ(feasibilitySearchBudget3D(milliseconds{150}, milliseconds{-1}, false),
            milliseconds{0});
}

} // namespace drone_city_nav
