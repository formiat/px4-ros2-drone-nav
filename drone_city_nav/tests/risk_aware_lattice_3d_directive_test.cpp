#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <optional>
#include <ranges>
#include <utility>

namespace drone_city_nav {
namespace {

struct OpenField3D {
  DistanceField3D distances;
  mppi::EsdfGrid grid;
};

[[nodiscard]] OpenField3D makeOpenField() {
  const OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 24, 16, 8}};
  DistanceField3D distances = DistanceField3D::build(occupancy, 20.0);
  const GridBounds3D& bounds = distances.bounds();
  const mppi::EsdfGrid grid{bounds.width_cells,
                            bounds.height_cells,
                            static_cast<float>(bounds.resolution_m),
                            static_cast<float>(bounds.origin_x),
                            static_cast<float>(bounds.origin_y),
                            bounds.depth_cells,
                            static_cast<float>(bounds.origin_z)};
  return OpenField3D{
      .distances = std::move(distances),
      .grid = grid,
  };
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
  config.maximum_search_time_ms = 1000.0;
  config.physical_footprint_radius_m = 0.25;
  config.physical_footprint_lower_extent_m = 0.25;
  config.physical_footprint_upper_extent_m = 0.25;
  return config;
}

[[nodiscard]] RiskAwareLattice3DResult
planDirective(const OpenField3D& field, const Lattice3DStrategicDirective& directive,
              const Point3& mission_goal) {
  const Point3 start{3.5, 3.5, 3.5};
  return planRiskAwareLattice3D(field.grid, field.distances.distancesM(), start,
                                directive.preferred_direction, mission_goal, {},
                                makeConfig(), nullptr, &directive);
}

TEST(RiskAwareLattice3DDirectiveTest,
     StrategicBacktrackOwnsTheLocalTargetAndRoutePurpose) {
  const OpenField3D field = makeOpenField();
  const Point3 local_target{3.5, 12.5, 3.5};
  const Lattice3DStrategicDirective directive{
      .planning_goal = local_target,
      .preferred_direction = Vec3{0.0, 1.0, 0.0},
      .route_purpose = Lattice3DRoutePurpose::kTopologicalBacktrack,
      .observation_frontier = std::nullopt,
      .selection_score = 7.0,
      .reaches_mission_goal = false,
  };

  const RiskAwareLattice3DResult result =
      planDirective(field, directive, {20.5, 3.5, 3.5});

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  EXPECT_EQ(result.route_purpose, Lattice3DRoutePurpose::kTopologicalBacktrack);
  EXPECT_NEAR(result.planning_goal.x, local_target.x, 1.0e-9);
  EXPECT_NEAR(result.planning_goal.y, local_target.y, 1.0e-9);
  EXPECT_FALSE(result.reached_mission_goal);
  EXPECT_FALSE(result.observation_frontier.has_value());
}

TEST(RiskAwareLattice3DDirectiveTest,
     LocalMissionSegmentDoesNotClaimFinalMissionArrival) {
  const OpenField3D field = makeOpenField();
  const Point3 local_target{10.5, 3.5, 3.5};
  const Lattice3DStrategicDirective directive{
      .planning_goal = local_target,
      .preferred_direction = Vec3{1.0, 0.0, 0.0},
      .route_purpose = Lattice3DRoutePurpose::kMissionTransit,
      .observation_frontier = std::nullopt,
      .selection_score = 1.0,
      .reaches_mission_goal = false,
  };

  const RiskAwareLattice3DResult result =
      planDirective(field, directive, {20.5, 3.5, 3.5});

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  EXPECT_EQ(result.route_purpose, Lattice3DRoutePurpose::kMissionTransit);
  EXPECT_FALSE(result.reached_mission_goal);
  ASSERT_FALSE(result.points.empty());
  EXPECT_NEAR(result.points.back().x, local_target.x, 1.0e-6);
}

TEST(RiskAwareLattice3DDirectiveTest,
     StrategicFrontierMetadataSurvivesLocalMaterialization) {
  const OpenField3D field = makeOpenField();
  const Point3 local_target{10.5, 3.5, 3.5};
  const ObservationFrontier frontier{
      .id = ObservationFrontierId{77U},
      .observation_pose = local_target,
      .supporting_map_revision = 42U,
  };
  const Lattice3DStrategicDirective directive{
      .planning_goal = local_target,
      .preferred_direction = Vec3{1.0, 0.0, 0.0},
      .route_purpose = Lattice3DRoutePurpose::kObservationFrontier,
      .observation_frontier = frontier,
      .selection_score = 4.5,
      .reaches_mission_goal = false,
  };

  const RiskAwareLattice3DResult result =
      planDirective(field, directive, {20.5, 3.5, 3.5});

  ASSERT_EQ(result.status, Lattice3DStatus::kReachedPlanningGoal);
  EXPECT_EQ(result.route_purpose, Lattice3DRoutePurpose::kObservationFrontier);
  if (!result.observation_frontier.has_value()) {
    FAIL() << "the strategic frontier metadata was not preserved";
  }
  EXPECT_EQ(result.observation_frontier.value().id, frontier.id);
  EXPECT_DOUBLE_EQ(result.frontier_selection_score, directive.selection_score);
  EXPECT_FALSE(result.reached_mission_goal);
}

TEST(RiskAwareLattice3DDirectiveTest,
     DirectedSoftTabuPenalizesOnlyTheRecordedApproachDirection) {
  const OpenField3D field = makeOpenField();
  const Point3 start{3.5, 3.5, 3.5};
  const Point3 local_target{10.5, 3.5, 3.5};
  const Lattice3DStrategicDirective directive{
      .planning_goal = local_target,
      .preferred_direction = Vec3{1.0, 0.0, 0.0},
      .route_purpose = Lattice3DRoutePurpose::kMissionTransit,
      .observation_frontier = std::nullopt,
      .reaches_mission_goal = false,
  };
  const Lattice3DSoftTabuEntry same_direction{
      .point = {6.5, 3.5, 3.5},
      .approach_heading_rad = 0.0,
      .radius_m = 1.0,
      .heading_tolerance_rad = 0.2,
      .penalty_cost = 40.0,
  };
  const Lattice3DSoftTabuEntry reverse_direction{
      .point = same_direction.point,
      .approach_heading_rad = std::numbers::pi,
      .radius_m = same_direction.radius_m,
      .heading_tolerance_rad = same_direction.heading_tolerance_rad,
      .penalty_cost = same_direction.penalty_cost,
  };

  const RiskAwareLattice3DResult penalized = planRiskAwareLattice3D(
      field.grid, field.distances.distancesM(), start, directive.preferred_direction,
      local_target, {}, makeConfig(), nullptr, &directive,
      std::span<const Lattice3DSoftTabuEntry>{&same_direction, 1U});
  const RiskAwareLattice3DResult reverse = planRiskAwareLattice3D(
      field.grid, field.distances.distancesM(), start, directive.preferred_direction,
      local_target, {}, makeConfig(), nullptr, &directive,
      std::span<const Lattice3DSoftTabuEntry>{&reverse_direction, 1U});

  ASSERT_EQ(penalized.status, Lattice3DStatus::kReachedPlanningGoal);
  ASSERT_EQ(reverse.status, Lattice3DStatus::kReachedPlanningGoal);
  EXPECT_GT(penalized.successor_diagnostics.soft_tabu_penalties_applied, 0U);
  const auto leaves_direct_line = [](const RiskAwareLattice3DResult& result) {
    return std::any_of(
        result.points.begin(), result.points.end(), [](const Point3& point) {
          return std::abs(point.y - 3.5) > 0.25 || std::abs(point.z - 3.5) > 0.25;
        });
  };
  EXPECT_TRUE(leaves_direct_line(penalized));
  EXPECT_FALSE(leaves_direct_line(reverse));
}

} // namespace
} // namespace drone_city_nav
