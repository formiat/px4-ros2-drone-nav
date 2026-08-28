#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
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
    EXPECT_TRUE(
        validateObservedSweptFootprint(occupancy, path[index - 1U], FootprintBodyAxis{},
                                       path[index], FootprintBodyAxis{}, footprint,
                                       ObservedSpaceValidationPolicy::kAllowUnknown)
            .accepted());
  }
}

TEST(PersistentDStarLitePlanner3DTest,
     SolvesOneWorldFixedTwentySixConnectedMissionToTheExactGoal) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 12, 8});
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{1.5, 1.5, 1.5};
  const Point3 goal{10.5, 10.5, 5.5};

  const PersistentPlannerResult3D result =
      planner.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(result.executable());
  ASSERT_EQ(result.status, PersistentPlannerStatus3D::kReachedMissionGoal);
  EXPECT_TRUE(result.search_complete);
  EXPECT_FALSE(result.search_state_reused);
  EXPECT_EQ(result.search_generation, 1U);
  EXPECT_DOUBLE_EQ(result.points.front().x, start.x);
  EXPECT_DOUBLE_EQ(result.points.front().y, start.y);
  EXPECT_DOUBLE_EQ(result.points.front().z, start.z);
  EXPECT_DOUBLE_EQ(result.points.back().x, goal.x);
  EXPECT_DOUBLE_EQ(result.points.back().y, goal.y);
  EXPECT_DOUBLE_EQ(result.points.back().z, goal.z);
  EXPECT_GT(result.estimated_translation_time_s, 0.0);
  EXPECT_GE(result.estimated_execution_time_s, result.estimated_translation_time_s);
  expectRawValid(result.points, *occupancy, planner.config().physical_footprint);
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

  const PersistentPlannerResult3D adaptive_result =
      adaptive.plan(request(start, goal, world(occupancy, 1U)));
  const PersistentPlannerResult3D fixed_result =
      fixed.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(adaptive_result.executable());
  ASSERT_TRUE(fixed_result.executable());
  expectSamePath(adaptive_result.points, fixed_result.points);
  EXPECT_DOUBLE_EQ(adaptive_result.path_length_m, fixed_result.path_length_m);
  EXPECT_GT(adaptive_result.adaptive_edge_queries, 0U);
  EXPECT_GT(adaptive_result.adaptive_edges_in_extracted_path, 0U);
  EXPECT_EQ(adaptive_result.maximum_queried_lattice_level, 2U);
  EXPECT_EQ(fixed_result.adaptive_edge_queries, 0U);
  EXPECT_EQ(fixed_result.maximum_queried_lattice_level, 0U);
}

TEST(PersistentDStarLitePlanner3DTest,
     FreeUnknownRelabelPreservesPathCostAndIncrementalSearchState) {
  auto first = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 12, 8, 6});
  static_cast<void>(first->setState(GridIndex3D{3, 3, 2}, ObservedVoxelState::kFree));
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{1.5, 3.5, 2.5};
  const Point3 goal{10.5, 3.5, 2.5};
  const PersistentPlannerResult3D initial =
      planner.plan(request(start, goal, world(first, 1U)));
  ASSERT_TRUE(initial.executable());

  auto relabeled = std::make_shared<ObservedOccupancyGrid3D>(*first);
  static_cast<void>(
      relabeled->setState(GridIndex3D{3, 3, 2}, ObservedVoxelState::kUnknown));
  static_cast<void>(
      relabeled->setState(GridIndex3D{8, 4, 2}, ObservedVoxelState::kFree));
  const PersistentPlannerResult3D updated = planner.plan(
      request(start, goal,
              world(relabeled, 2U,
                    {ObservedOccupancyGrid3D::chunkIndex(GridIndex3D{3, 3, 2})})));

  ASSERT_TRUE(updated.executable());
  EXPECT_TRUE(updated.search_state_reused);
  EXPECT_TRUE(updated.occupied_world_unchanged);
  EXPECT_EQ(updated.changed_occupied_voxels, 0U);
  EXPECT_EQ(updated.search_generation, initial.search_generation);
  EXPECT_DOUBLE_EQ(updated.path_length_m, initial.path_length_m);
  EXPECT_DOUBLE_EQ(updated.estimated_translation_time_s,
                   initial.estimated_translation_time_s);
  expectSamePath(updated.points, initial.points);
}

TEST(PersistentDStarLitePlanner3DTest,
     RepairsTheResidentSearchAfterAnOccupiedCellAppearsOnItsPath) {
  auto initial_occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 14, 10, 6});
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{1.5, 5.5, 2.5};
  const Point3 goal{12.5, 5.5, 2.5};
  const PersistentPlannerResult3D initial =
      planner.plan(request(start, goal, world(initial_occupancy, 1U)));
  ASSERT_TRUE(initial.executable());

  auto changed = std::make_shared<ObservedOccupancyGrid3D>(*initial_occupancy);
  const GridIndex3D obstacle{7, 5, 2};
  ASSERT_TRUE(changed->setState(obstacle, ObservedVoxelState::kOccupied));
  const PersistentPlannerResult3D repaired = planner.plan(
      request(start, goal,
              world(changed, 2U, {ObservedOccupancyGrid3D::chunkIndex(obstacle)})));

  ASSERT_TRUE(repaired.executable());
  EXPECT_TRUE(repaired.search_state_reused);
  EXPECT_EQ(repaired.changed_occupied_voxels, 1U);
  EXPECT_GT(repaired.affected_lattice_states, 0U);
  EXPECT_GT(repaired.adaptive_edge_queries, 0U);
  EXPECT_EQ(repaired.search_generation, initial.search_generation);
  EXPECT_EQ(repaired.repair_generation, 1U);
  EXPECT_GT(repaired.path_length_m, initial.path_length_m);
  expectRawValid(repaired.points, *changed, planner.config().physical_footprint);
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
  PersistentPlannerResult3D initial =
      planner.plan(request(start, goal, world(initial_occupancy, 1U)));
  for (std::size_t attempt = 0U; attempt < 5000U && !initial.executable(); ++attempt) {
    initial = planner.plan(request(start, goal, world(initial_occupancy, 1U)));
  }
  ASSERT_TRUE(initial.executable());

  auto changed = std::make_shared<ObservedOccupancyGrid3D>(*initial_occupancy);
  const GridIndex3D obstacle{7, 5, 2};
  ASSERT_TRUE(changed->setState(obstacle, ObservedVoxelState::kOccupied));
  PersistentPlannerResult3D repaired = planner.plan(
      request(start, goal,
              world(changed, 2U, {ObservedOccupancyGrid3D::chunkIndex(obstacle)})));

  EXPECT_TRUE(repaired.search_state_reused);
  EXPECT_EQ(repaired.search_generation, initial.search_generation);
  EXPECT_EQ(repaired.repair_generation, 1U);
  EXPECT_GT(repaired.affected_lattice_states, 1U);
  EXPECT_EQ(repaired.repair_lattice_states_processed, 1U);
  EXPECT_TRUE(repaired.repair_pending);
  EXPECT_GT(repaired.repair_lattice_states_pending, 0U);
  EXPECT_EQ(repaired.expansions, 0U);

  for (std::size_t attempt = 0U; attempt < 5000U && !repaired.executable(); ++attempt) {
    repaired = planner.plan(
        request(start, goal,
                world(changed, 2U, {ObservedOccupancyGrid3D::chunkIndex(obstacle)})));
  }

  ASSERT_TRUE(repaired.executable());
  EXPECT_FALSE(repaired.repair_pending);
  EXPECT_EQ(repaired.repair_lattice_states_pending, 0U);
  expectRawValid(repaired.points, *changed, planner.config().physical_footprint);
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
  const PersistentPlannerResult3D initial =
      planner.plan(request(start, goal, world(blocked, 1U)));
  ASSERT_EQ(initial.status, PersistentPlannerStatus3D::kNoRoute);

  auto opened = std::make_shared<ObservedOccupancyGrid3D>(*blocked);
  const GridIndex3D opening{7, 5, 2};
  ASSERT_TRUE(opened->setState(opening, ObservedVoxelState::kUnknown));
  const PersistentPlannerResult3D repaired = planner.plan(request(
      start, goal, world(opened, 2U, {ObservedOccupancyGrid3D::chunkIndex(opening)})));

  ASSERT_TRUE(repaired.executable());
  EXPECT_TRUE(repaired.search_state_reused);
  EXPECT_EQ(repaired.changed_occupied_voxels, 1U);
  EXPECT_GT(repaired.affected_lattice_states, 0U);
  EXPECT_EQ(repaired.search_generation, initial.search_generation);
  EXPECT_EQ(repaired.repair_generation, 1U);
  expectRawValid(repaired.points, *opened, planner.config().physical_footprint);
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
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 start{2.5, 5.5, 5.5};
  const Point3 goal{11.5, 5.5, 5.5};

  const PersistentPlannerResult3D result =
      planner.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(result.executable());
  EXPECT_LT(std::ranges::min_element(result.points, {}, &Point3::z)->z, start.z);
  expectRawValid(result.points, *occupancy, planner.config().physical_footprint);
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

  const PersistentPlannerResult3D timed =
      timed_planner.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(timed.executable());
  EXPECT_TRUE(timed.execution_time_search_complete);
  EXPECT_TRUE(timed.search_complete);
  EXPECT_GT(timed.execution_time_search_expansions, 0U);
  EXPECT_GT(timed.execution_time_search_objective_s, 0.0);
  EXPECT_TRUE(std::ranges::any_of(timed.points,
                                  [](const Point3& point) { return point.y > 6.5; }));
  EXPECT_GT(timed.estimated_stationary_turn_time_s, 0.0);

  PersistentPlannerConfig3D translation_only_config = timed_config;
  translation_only_config.minimum_continuous_turn_alignment = -1.0;
  PersistentDStarLitePlanner3D translation_only_planner{translation_only_config};
  const PersistentPlannerResult3D translation_only =
      translation_only_planner.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(translation_only.executable());
  EXPECT_LT(translation_only.path_length_m, timed.path_length_m);
  EXPECT_FALSE(std::ranges::any_of(translation_only.points,
                                   [](const Point3& point) { return point.y > 6.5; }));
  expectRawValid(timed.points, *occupancy, timed_planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     MovingStartReusesTheSameBackwardSearchAndReturnsACurrentConnector) {
  auto occupancy = std::make_shared<ObservedOccupancyGrid3D>(
      GridBounds3D{0.0, 0.0, 0.0, 1.0, 16, 8, 6});
  PersistentDStarLitePlanner3D planner{testConfig()};
  const Point3 first_start{1.5, 3.5, 2.5};
  const Point3 moved_start{5.5, 3.5, 2.5};
  const Point3 goal{14.5, 3.5, 2.5};
  const PersistentPlannerResult3D initial =
      planner.plan(request(first_start, goal, world(occupancy, 1U)));
  ASSERT_TRUE(initial.executable());

  const PersistentPlannerResult3D moved =
      planner.plan(request(moved_start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(moved.executable());
  EXPECT_TRUE(moved.search_state_reused);
  EXPECT_EQ(moved.search_generation, initial.search_generation);
  EXPECT_DOUBLE_EQ(moved.points.front().x, moved_start.x);
  EXPECT_DOUBLE_EQ(moved.points.front().y, moved_start.y);
  EXPECT_DOUBLE_EQ(moved.points.front().z, moved_start.z);
  EXPECT_LT(moved.path_length_m, initial.path_length_m);
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

  const PersistentPlannerResult3D initial =
      planner.plan(request(start, goal, std::move(initial_world)));
  ASSERT_EQ(initial.status, PersistentPlannerStatus3D::kSearchInProgress);
  ASSERT_FALSE(initial.search_state_reused);

  ProprioceptiveFreeSpaceSeed3D moved_seed = initial_seed;
  moved_seed.position.x += 0.25;
  LaunchSupportContact3D moved_support = initial_support;
  moved_support.seed = moved_seed;
  ASSERT_TRUE(launchSupportContactValid3D(moved_support));
  PersistentPlannerWorld3D refreshed_world = world(occupancy, 1U);
  refreshed_world.proprioceptive_free_space_seed = moved_seed;
  refreshed_world.launch_support_contact = moved_support;
  const PersistentPlannerResult3D refreshed =
      planner.plan(request(start, goal, std::move(refreshed_world)));

  EXPECT_TRUE(refreshed.search_state_reused);
  EXPECT_TRUE(refreshed.occupied_world_unchanged);
  EXPECT_EQ(refreshed.search_generation, initial.search_generation);
  EXPECT_GE(refreshed.records, initial.records);
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

  PersistentPlannerResult3D result =
      planner.plan(request(start, goal, world(occupancy, 1U)));
  ASSERT_EQ(result.status, PersistentPlannerStatus3D::kSearchInProgress);
  EXPECT_TRUE(result.points.empty());
  const std::uint64_t generation = result.search_generation;
  for (std::size_t attempt = 0U; attempt < 5000U && !result.executable(); ++attempt) {
    result = planner.plan(request(start, goal, world(occupancy, 1U)));
  }

  ASSERT_TRUE(result.executable())
      << "spatial_expansions=" << result.expansions
      << " time_expansions=" << result.execution_time_search_expansions
      << " time_records=" << result.execution_time_search_records;
  EXPECT_TRUE(result.search_state_reused);
  EXPECT_EQ(result.search_generation, generation);
  EXPECT_FALSE(result.execution_time_search_complete);
  EXPECT_FALSE(result.search_complete);
  EXPECT_EQ(result.shortcut_checks, 0U);
  expectRawValid(result.points, *occupancy, planner.config().physical_footprint);
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

  PersistentPlannerResult3D result = planner.plan(request(
      Point3{54.0, 54.0, 18.0}, Point3{216.0, 378.0, 18.0}, world(occupancy, 1U)));
  std::size_t slices = 1U;
  for (; slices < 30U && !result.executable(); ++slices) {
    result = planner.plan(request(Point3{54.0, 54.0, 18.0}, Point3{216.0, 378.0, 18.0},
                                  world(occupancy, 1U)));
  }

  EXPECT_TRUE(result.executable())
      << "slices=" << slices << " records=" << result.records
      << " open=" << result.open_entries << " expansions=" << result.expansions
      << " time_expansions=" << result.execution_time_search_expansions
      << " search_ms=" << result.search_ms;
  EXPECT_TRUE(result.feasibility_attempted);
  EXPECT_TRUE(result.feasibility_route_found);
  EXPECT_GT(result.feasibility_expansions, 0U);
  EXPECT_FALSE(result.execution_time_search_complete);
  EXPECT_FALSE(result.search_complete);
  expectRawValid(result.points, *occupancy, planner.config().physical_footprint);
}

TEST(PersistentDStarLitePlanner3DTest,
     FeasibilityFirstSearchDetoursBeforePublishingACompleteRawValidRoute) {
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
  ASSERT_FALSE(
      validateObservedSweptFootprint(*occupancy, start, FootprintBodyAxis{}, goal,
                                     FootprintBodyAxis{}, config.physical_footprint,
                                     ObservedSpaceValidationPolicy::kAllowUnknown)
          .accepted());

  PersistentDStarLitePlanner3D planner{config};
  const PersistentPlannerResult3D result =
      planner.plan(request(start, goal, world(occupancy, 1U)));

  ASSERT_TRUE(result.executable());
  EXPECT_TRUE(result.feasibility_attempted);
  EXPECT_TRUE(result.feasibility_route_found);
  EXPECT_GT(result.feasibility_expansions, 1U);
  EXPECT_GT(result.points.size(), 2U);
  expectRawValid(result.points, *occupancy, planner.config().physical_footprint);
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

  const PersistentPlannerResult3D initial =
      planner.plan(request(start, goal, world(occupancy, 1U)));
  ASSERT_EQ(initial.status, PersistentPlannerStatus3D::kSearchInProgress);
  EXPECT_TRUE(initial.feasibility_attempted);
  EXPECT_FALSE(initial.feasibility_route_found);
  EXPECT_EQ(initial.feasibility_expansions, 1U);
  EXPECT_GT(initial.expansions, 0U);

  const PersistentPlannerResult3D continued =
      planner.plan(request(start, goal, world(occupancy, 1U)));
  EXPECT_TRUE(continued.search_state_reused);
  EXPECT_EQ(continued.search_generation, initial.search_generation);
  EXPECT_GE(continued.records, initial.records);
  EXPECT_GT(continued.expansions, 0U);
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
  const PersistentPlannerResult3D initial =
      planner.plan(request(start, goal, world(initial_occupancy, 1U)));
  ASSERT_TRUE(initial.executable());
  ASSERT_TRUE(initial.feasibility_route_found);

  auto changed = std::make_shared<ObservedOccupancyGrid3D>(*initial_occupancy);
  const GridIndex3D obstacle{11, 5, 2};
  ASSERT_TRUE(changed->setState(obstacle, ObservedVoxelState::kOccupied));
  const PersistentPlannerResult3D replacement = planner.plan(
      request(start, goal,
              world(changed, 2U, {ObservedOccupancyGrid3D::chunkIndex(obstacle)})));

  ASSERT_TRUE(replacement.executable());
  EXPECT_FALSE(replacement.incumbent_retained);
  EXPECT_TRUE(replacement.feasibility_attempted);
  EXPECT_TRUE(replacement.feasibility_route_found);
  EXPECT_GT(replacement.points.size(), initial.points.size());
  expectRawValid(replacement.points, *changed, planner.config().physical_footprint);
}

} // namespace
} // namespace drone_city_nav
