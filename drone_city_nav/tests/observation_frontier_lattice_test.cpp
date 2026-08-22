#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>

namespace drone_city_nav {
namespace {

struct OpeningFixture {
  ObservedOccupancyGrid3D occupancy;
  Point3 start;
  Point3 goal;
};

void fillBox(ObservedOccupancyGrid3D& occupancy, const int minimum_x,
             const int maximum_x, const int minimum_y, const int maximum_y,
             const int minimum_z, const int maximum_z, const ObservedVoxelState state) {
  for (int z = minimum_z; z <= maximum_z; ++z) {
    for (int y = minimum_y; y <= maximum_y; ++y) {
      for (int x = minimum_x; x <= maximum_x; ++x) {
        ASSERT_TRUE(occupancy.setState(GridIndex3D{x, y, z}, state));
      }
    }
  }
}

[[nodiscard]] OpeningFixture makeForwardOpeningFixture(const double start_z,
                                                       const int opening_minimum_y,
                                                       const int opening_maximum_y,
                                                       const int opening_minimum_z,
                                                       const int opening_maximum_z) {
  OpeningFixture fixture{
      .occupancy =
          ObservedOccupancyGrid3D{GridBounds3D{0.0, 0.0, 0.0, 1.0, 32, 24, 24}},
      .start = Point3{3.5, 10.5, start_z},
      .goal = Point3{26.5, 10.5, start_z},
  };
  fillBox(fixture.occupancy, 0, 14, 0, 23, 0, 23, ObservedVoxelState::kOccupied);
  fillBox(fixture.occupancy, 1, 11, 2, 21, 2, 21, ObservedVoxelState::kFree);
  fillBox(fixture.occupancy, 12, 14, opening_minimum_y, opening_maximum_y,
          opening_minimum_z, opening_maximum_z, ObservedVoxelState::kFree);
  return fixture;
}

[[nodiscard]] OpeningFixture makeBackwardOpeningFixture() {
  OpeningFixture fixture{
      .occupancy =
          ObservedOccupancyGrid3D{GridBounds3D{0.0, 0.0, 0.0, 1.0, 32, 24, 24}},
      .start = Point3{18.5, 10.5, 12.5},
      .goal = Point3{27.5, 10.5, 12.5},
  };
  fillBox(fixture.occupancy, 5, 31, 0, 23, 0, 23, ObservedVoxelState::kOccupied);
  fillBox(fixture.occupancy, 8, 20, 2, 21, 2, 21, ObservedVoxelState::kFree);
  fillBox(fixture.occupancy, 5, 7, 8, 12, 10, 15, ObservedVoxelState::kFree);
  fillBox(fixture.occupancy, 25, 30, 8, 12, 10, 15, ObservedVoxelState::kFree);
  return fixture;
}

[[nodiscard]] RiskAwareLattice3DConfig makeConfig() {
  RiskAwareLattice3DConfig config;
  config.horizontal_step_m = 1.0;
  config.vertical_step_m = 1.0;
  config.sample_step_m = 0.5;
  config.planning_goal_distance_m = 40.0;
  config.preferred_distance_m = 0.0;
  config.critical_distance_m = 0.0;
  config.maximum_topology_search_groups = 0U;
  config.maximum_expansions = 100000U;
  config.maximum_search_time_ms = 2000.0;
  config.frontier_minimum_endpoint_displacement_m = 1.0;
  config.observation_frontier_maximum_evaluations = 100000U;
  config.observation_frontier_evaluation_stride = 1U;
  config.observation_frontier_information_gain_weight = 3.0;
  config.observation_frontier_goal_progress_weight = 0.1;
  config.observation_frontier_path_cost_weight = 0.1;
  config.observation_frontier_clearance_weight = 0.0;
  config.physical_footprint_radius_m = 0.25;
  config.physical_footprint_lower_extent_m = 0.25;
  config.physical_footprint_upper_extent_m = 0.25;
  config.physical_footprint_samples = 8U;
  config.sensor_observability.footprint.radius_m = 0.25;
  config.sensor_observability.footprint.lower_extent_m = 0.25;
  config.sensor_observability.footprint.upper_extent_m = 0.25;
  config.sensor_observability.footprint.perimeter_samples = 8U;
  config.sensor_observability.footprint.radial_rings = 2U;
  config.sensor_observability.footprint.axial_samples = 3U;
  config.sensor_observability.maximum_observation_range_m = 6.0;
  config.sensor_observability.minimum_known_free_ray_m = 1.0;
  config.sensor_observability.minimum_supporting_rays = 2U;
  config.sensor_observability.minimum_information_gain_voxels = 2U;
  return config;
}

[[nodiscard]] RiskAwareLattice3DResult
planFixture(const OpeningFixture& fixture,
            const RiskAwareLattice3DConfig& config = makeConfig()) {
  const ObservedEsdf3D field =
      buildObservedEsdf3D(fixture.occupancy, fixture.occupancy.bounds(), 40.0);
  const Lattice3DExplorationContext exploration{
      .observed_occupancy = field.local_occupancy.get(),
      .map_revision = 41U,
      .strategic_directive = std::nullopt,
  };
  return planRiskAwareLattice3D(field.grid, field.distances_m, fixture.start,
                                Vec3{fixture.goal.x - fixture.start.x,
                                     fixture.goal.y - fixture.start.y,
                                     fixture.goal.z - fixture.start.z},
                                fixture.goal, {}, config, nullptr, &exploration);
}

void expectObservationRoute(const RiskAwareLattice3DResult& result,
                            const Point3& start) {
  ASSERT_EQ(result.status, Lattice3DStatus::kViableFrontier)
      << "frontiers=" << result.frontier_candidates_considered
      << " searches=" << result.frontier_searches
      << " sampled_free=" << result.frontier_sampled_free_voxels
      << " boundary_candidates=" << result.frontier_boundary_candidates
      << " evaluated=" << result.frontier_evaluated_candidates
      << " rejected(raw=" << result.successor_diagnostics.lattice_rejected_raw_collision
      << ", unknown=" << result.successor_diagnostics.lattice_rejected_unknown_space
      << ", outside=" << result.successor_diagnostics.lattice_rejected_outside_grid
      << ", risk=" << result.successor_diagnostics.lattice_rejected_risk_stage << ")";
  ASSERT_EQ(result.route_purpose, Lattice3DRoutePurpose::kObservationFrontier);
  ASSERT_TRUE(result.observation_frontier.has_value());
  ASSERT_GE(result.points.size(), 2U);
  const ObservationFrontier frontier =
      result.observation_frontier.value_or(ObservationFrontier{});
  EXPECT_EQ(frontier.supporting_map_revision, 41U);
  EXPECT_GT(frontier.information_gain_voxels, 0U);
  EXPECT_NEAR(result.frontier_endpoint_displacement_m,
              distance3D(start, result.points.back()), 1.0e-9);
  EXPECT_GE(result.frontier_endpoint_displacement_m, 1.0);
}

TEST(ObservationFrontierLatticeTest, DescendsFromHighStartTowardLowerObservedOpening) {
  const OpeningFixture fixture = makeForwardOpeningFixture(18.5, 8, 12, 4, 8);

  const RiskAwareLattice3DResult result = planFixture(fixture);

  expectObservationRoute(result, fixture.start);
  EXPECT_LT(result.points.back().z, 10.0);
  EXPECT_GT(result.points.back().x, 11.0);
}

TEST(ObservationFrontierLatticeTest, ClimbsFromLowStartTowardHigherObservedOpening) {
  const OpeningFixture fixture = makeForwardOpeningFixture(5.5, 8, 12, 15, 20);

  const RiskAwareLattice3DResult result = planFixture(fixture);

  expectObservationRoute(result, fixture.start);
  EXPECT_GT(result.points.back().z, 14.0);
  EXPECT_GT(result.points.back().x, 11.0);
}

TEST(ObservationFrontierLatticeTest,
     NormalPolicyReachesGoalThroughObservedOpeningOutsideDirectGoalLine) {
  OpeningFixture fixture = makeForwardOpeningFixture(10.5, 15, 19, 8, 13);
  fixture.start.y = 6.5;
  fixture.goal.y = 6.5;

  const RiskAwareLattice3DResult result = planFixture(fixture);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  EXPECT_TRUE(std::ranges::any_of(result.points, [](const Point3& point) {
    return point.x > 11.0 && point.x < 15.0 && point.y > 13.0;
  }));
}

TEST(ObservationFrontierLatticeTest,
     StrictPolicyMovesSidewaysTowardObservedOpeningOutsideDirectGoalLine) {
  OpeningFixture fixture = makeForwardOpeningFixture(10.5, 15, 19, 8, 13);
  fixture.start.y = 6.5;
  fixture.goal.y = 6.5;
  RiskAwareLattice3DConfig config = makeConfig();
  config.require_known_free_space = true;

  const RiskAwareLattice3DResult result = planFixture(fixture, config);

  expectObservationRoute(result, fixture.start);
  EXPECT_GT(result.points.back().y, 13.0);
  EXPECT_GT(result.points.back().x, 11.0);
}

TEST(ObservationFrontierLatticeTest,
     AcceptsObservationRouteWhoseEndpointIsFartherFromGoal) {
  const OpeningFixture fixture = makeBackwardOpeningFixture();

  const RiskAwareLattice3DResult result = planFixture(fixture);

  expectObservationRoute(result, fixture.start);
  EXPECT_LT(result.points.back().x, fixture.start.x);
  EXPECT_LT(result.achieved_progress_m, 0.0);
}

TEST(ObservationFrontierLatticeTest,
     StrategicBacktrackSuppressesCompetingFrontierSelection) {
  const OpeningFixture fixture = makeForwardOpeningFixture(12.5, 8, 12, 10, 15);
  const ObservedEsdf3D field =
      buildObservedEsdf3D(fixture.occupancy, fixture.occupancy.bounds(), 40.0);
  const Point3 backtrack_goal{3.5, 18.5, 12.5};
  const Lattice3DExplorationContext exploration{
      .observed_occupancy = field.local_occupancy.get(),
      .map_revision = 42U,
      .strategic_directive =
          Lattice3DStrategicDirective{
              .planning_goal = backtrack_goal,
              .preferred_direction = Vec3{0.0, 8.0, 0.0},
              .route_purpose = Lattice3DRoutePurpose::kTopologicalBacktrack,
              .observation_frontier = std::nullopt,
              .selection_score = 7.0,
              .reaches_mission_goal = false,
          },
  };

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      field.grid, field.distances_m, fixture.start, {1.0, 0.0, 0.0}, fixture.goal, {},
      makeConfig(), nullptr, &exploration);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  EXPECT_EQ(result.route_purpose, Lattice3DRoutePurpose::kTopologicalBacktrack);
  EXPECT_DOUBLE_EQ(result.planning_goal.x, backtrack_goal.x);
  EXPECT_DOUBLE_EQ(result.planning_goal.y, backtrack_goal.y);
  EXPECT_DOUBLE_EQ(result.planning_goal.z, backtrack_goal.z);
  EXPECT_FALSE(result.reached_mission_goal);
  EXPECT_FALSE(result.observation_frontier.has_value());
  EXPECT_EQ(result.frontier_searches, 0U);
  ASSERT_FALSE(result.points.empty());
  EXPECT_NEAR(result.points.back().y, backtrack_goal.y, 1.0e-6);
}

TEST(ObservationFrontierLatticeTest,
     StrategicLocalMissionTargetDoesNotClaimFinalMissionArrival) {
  const OpeningFixture fixture = makeForwardOpeningFixture(12.5, 8, 12, 10, 15);
  const ObservedEsdf3D field =
      buildObservedEsdf3D(fixture.occupancy, fixture.occupancy.bounds(), 40.0);
  const Point3 local_goal{9.5, 10.5, 12.5};
  const Lattice3DExplorationContext exploration{
      .observed_occupancy = field.local_occupancy.get(),
      .map_revision = 43U,
      .strategic_directive =
          Lattice3DStrategicDirective{
              .planning_goal = local_goal,
              .preferred_direction = Vec3{6.0, 0.0, 0.0},
              .route_purpose = Lattice3DRoutePurpose::kMissionTransit,
              .observation_frontier = std::nullopt,
              .selection_score = 1.0,
              .reaches_mission_goal = false,
          },
  };

  const RiskAwareLattice3DResult result = planRiskAwareLattice3D(
      field.grid, field.distances_m, fixture.start, {1.0, 0.0, 0.0}, fixture.goal, {},
      makeConfig(), nullptr, &exploration);

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  EXPECT_EQ(result.route_purpose, Lattice3DRoutePurpose::kMissionTransit);
  EXPECT_FALSE(result.reached_mission_goal);
  EXPECT_NEAR(result.points.back().x, local_goal.x, 1.0e-6);
}

} // namespace
} // namespace drone_city_nav
