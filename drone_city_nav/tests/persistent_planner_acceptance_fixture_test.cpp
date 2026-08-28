#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/route_compiler_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <vector>

#include "persistent_planner_acceptance_fixture.hpp"

namespace drone_city_nav::test {
namespace {

[[nodiscard]] SweptFootprintConfig acceptanceFootprint() noexcept {
  return SweptFootprintConfig{
      .radius_m = 0.20,
      .lower_extent_m = 0.20,
      .upper_extent_m = 0.20,
      .perimeter_samples = 8U,
      .radial_rings = 1U,
      .axial_samples = 3U,
      .sweep_step_m = 0.20,
  };
}

[[nodiscard]] PersistentPlannerConfig3D acceptancePlannerConfig() noexcept {
  PersistentPlannerConfig3D config;
  config.horizontal_step_m = 1.0;
  config.vertical_step_m = 1.0;
  config.time_model.maximum_horizontal_speed_mps = 4.0;
  config.time_model.maximum_vertical_speed_mps = 2.0;
  config.time_model.maximum_horizontal_acceleration_mps2 = 4.0;
  config.time_model.maximum_vertical_acceleration_mps2 = 3.0;
  config.time_model.maximum_control_jerk_mps3 = 12.0;
  config.goal_tolerance_m = 0.01;
  config.maximum_compute_time_ms = 2000.0;
  config.maximum_expansions_per_update = 500000U;
  config.maximum_extracted_path_nodes = 4096U;
  config.maximum_shortcut_checks = 4096U;
  config.physical_footprint = acceptanceFootprint();
  config.flight_envelope.minimum_target_z_m = 0.5;
  config.flight_envelope.maximum_target_z_m = 11.5;
  return config;
}

[[nodiscard]] PersistentPlannerWorld3D
acceptanceWorld(std::shared_ptr<const ObservedOccupancyGrid3D> occupancy,
                const std::uint64_t revision,
                std::vector<OccupancyChunkIndex3D> dirty_chunks = {}) {
  const OccupancyGrid3D occupied = occupancy->occupiedSnapshot();
  return PersistentPlannerWorld3D{
      .observed_occupancy = std::move(occupancy),
      .static_occupancy = nullptr,
      .proprioceptive_free_space_seed = std::nullopt,
      .launch_support_contact = std::nullopt,
      .dirty_chunks = std::move(dirty_chunks),
      .producer_instance_id = 0xA11CEU,
      .revision = revision,
      .occupied_fingerprint = occupied.contentFingerprint(),
      .full_reset = false,
  };
}

[[nodiscard]] PersistentPlannerResult3D
planMission(PersistentDStarLitePlanner3D& planner,
            const PersistentPlannerAcceptanceMission& mission,
            std::shared_ptr<const ObservedOccupancyGrid3D> occupancy,
            const std::uint64_t revision = 1U,
            std::vector<OccupancyChunkIndex3D> dirty_chunks = {}) {
  return planner.plan(PersistentPlannerRequest3D{
      .start = mission.start,
      .velocity = {},
      .mission_goal = mission.goal,
      .mission_epoch = 11U,
      .world = acceptanceWorld(std::move(occupancy), revision, std::move(dirty_chunks)),
  });
}

void expectRawSafe(const std::span<const Point3> path,
                   const ObservedOccupancyGrid3D& occupancy) {
  ASSERT_GE(path.size(), 2U);
  const SweptFootprintConfig footprint = acceptanceFootprint();
  for (std::size_t index = 1U; index < path.size(); ++index) {
    EXPECT_TRUE(
        validateObservedSweptFootprint(occupancy, path[index - 1U], FootprintBodyAxis{},
                                       path[index], FootprintBodyAxis{}, footprint,
                                       ObservedSpaceValidationPolicy::kAllowUnknown)
            .accepted())
        << "segment=" << index;
  }
}

[[nodiscard]] bool pathsDiffer(const std::span<const Point3> first,
                               const std::span<const Point3> second) noexcept {
  if (first.size() != second.size()) {
    return true;
  }
  for (std::size_t index = 0U; index < first.size(); ++index) {
    if (distance3D(first[index], second[index]) > 1.0e-9) {
      return true;
    }
  }
  return false;
}

[[nodiscard]] RouteCompilationResult3D
compileMissionRoute(const PersistentPlannerResult3D& plan,
                    const ObservedOccupancyGrid3D& occupancy) {
  std::vector<RouteSample3D> route = sampleRoute3D(plan.points, 0.5, 4.0);
  const std::uint64_t fingerprint = routeFingerprint(route);
  RouteCompilerConfig3D compiler;
  compiler.unconstrained_speed_mps = 4.0;
  compiler.constrained_speed_mps = 2.0;
  compiler.speed_policy.cruise_speed_mps = 4.0;
  compiler.speed_policy.absolute_speed_limit_mps = 6.0;
  compiler.dynamics.maximum_horizontal_speed_mps = 6.0F;
  compiler.dynamics.maximum_vertical_speed_mps = 3.0F;
  compiler.physical_footprint = acceptanceFootprint();
  compiler.tracking_error_tube.response_time_s = 0.10;
  const OccupancyGrid3D occupied = occupancy.occupiedSnapshot();
  return compileExecutionRoute3D(RouteCompilerInput3D{
      .route = std::move(route),
      .constrained_spans = {},
      .passage_volumes = {},
      .cooperative_passage_assignments = {},
      .selected_passage_traversal_ids = {},
      .passage_volume_config = {},
      .endpoint_semantics = RouteEndpointSemantics3D::kMissionStop,
      .materialized_route_fingerprint = fingerprint,
      .tracking_world =
          TrackingErrorTubeWorld3D{
              .observed_occupancy = &occupancy,
              .occupancy = nullptr,
              .occupied_content_fingerprint = occupied.contentFingerprint(),
              .occupancy_policy = TrackingErrorTubeOccupancyPolicy3D::kRawOccupiedOnly,
              .free_space_seed = nullptr,
              .launch_support_contact = nullptr,
          },
      .config = compiler,
  });
}

void expectRequiredMotion(const PersistentPlannerAcceptanceMission& mission,
                          const PersistentPlannerResult3D& plan,
                          const std::optional<GridBounds3D>& local_cache) {
  ASSERT_GE(plan.points.size(), 2U);
  const auto [minimum_z, maximum_z] =
      std::ranges::minmax_element(plan.points, {}, &Point3::z);
  const auto minimum_y = std::ranges::min_element(plan.points, {}, &Point3::y);
  switch (mission.required_motion) {
    case RequiredRouteMotion::kNone:
      return;
    case RequiredRouteMotion::kDescend:
      EXPECT_LT(minimum_z->z, mission.start.z - 2.0);
      return;
    case RequiredRouteMotion::kClimb:
      EXPECT_GT(maximum_z->z, mission.start.z + 2.0);
      return;
    case RequiredRouteMotion::kLateral:
      EXPECT_LT(minimum_y->y, mission.start.y - 2.0);
      return;
    case RequiredRouteMotion::kVertical: {
      EXPECT_GT(maximum_z->z - minimum_z->z, 6.0);
      const double maximum_horizontal_offset_m = std::ranges::max(
          plan.points | std::views::transform([&](const Point3& point) {
            return std::hypot(point.x - mission.start.x, point.y - mission.start.y);
          }));
      EXPECT_LT(maximum_horizontal_offset_m, 1.0e-9);
      return;
    }
    case RequiredRouteMotion::kInclined:
      EXPECT_GT(maximum_z->z - minimum_z->z, 5.0);
      EXPECT_GT(std::hypot(mission.goal.x - mission.start.x,
                           mission.goal.y - mission.start.y),
                15.0);
      return;
    case RequiredRouteMotion::kInitiallyAwayFromGoal: {
      const double initial_distance_m = distance3D(mission.start, mission.goal);
      const double maximum_distance_m = std::ranges::max(
          plan.points | std::views::transform([&](const Point3& point) {
            return distance3D(point, mission.goal);
          }));
      EXPECT_GT(maximum_distance_m, initial_distance_m + 0.5);
      return;
    }
    case RequiredRouteMotion::kBeyondLocalDistanceCache: {
      const GridBounds3D cache = local_cache.value_or(GridBounds3D{});
      ASSERT_TRUE(local_cache.has_value());
      const double cache_max_x =
          cache.origin_x + cache.resolution_m * cache.width_cells;
      EXPECT_TRUE(std::ranges::any_of(plan.points, [cache_max_x](const Point3& point) {
        return point.x > cache_max_x;
      }));
      return;
    }
  }
}

TEST(PersistentPlannerAcceptanceFixture,
     EveryGenericScenarioPlansRawSafeAndCompilesForExecution) {
  const std::vector<PersistentPlannerAcceptanceKind> kinds =
      persistentPlannerAcceptanceKinds();
  ASSERT_EQ(kinds.size(), 12U);

  for (const PersistentPlannerAcceptanceKind kind : kinds) {
    const PersistentPlannerAcceptanceFixture fixture =
        buildPersistentPlannerAcceptanceFixture(kind);
    ASSERT_NE(fixture.occupancy, nullptr) << fixture.name;
    ASSERT_FALSE(fixture.missions.empty()) << fixture.name;
    for (const PersistentPlannerAcceptanceMission& mission : fixture.missions) {
      PersistentDStarLitePlanner3D planner{acceptancePlannerConfig()};
      const PersistentPlannerResult3D result =
          planMission(planner, mission, fixture.occupancy);
      ASSERT_TRUE(result.executable())
          << fixture.name << " status=" << persistentPlannerStatus3DName(result.status)
          << " expansions=" << result.expansions << " search_ms=" << result.search_ms;
      EXPECT_EQ(result.status, PersistentPlannerStatus3D::kReachedMissionGoal)
          << fixture.name;
      EXPECT_TRUE(result.search_complete) << fixture.name;
      EXPECT_DOUBLE_EQ(result.points.front().x, mission.start.x) << fixture.name;
      EXPECT_DOUBLE_EQ(result.points.back().x, mission.goal.x) << fixture.name;
      expectRawSafe(result.points, *fixture.occupancy);
      expectRequiredMotion(mission, result, fixture.local_distance_cache_bounds);

      const RouteCompilationResult3D compilation =
          compileMissionRoute(result, *fixture.occupancy);
      ASSERT_TRUE(compilation.compiled())
          << fixture.name << " reason="
          << executionRouteGeometryFailureReasonName3D(compilation.validation.reason);
      ASSERT_NE(compilation.geometry, nullptr);
      EXPECT_TRUE(trackingErrorTubeProfile3DMatchesWorld(
          *compilation.geometry->route, *compilation.tracking_error_tube,
          TrackingErrorTubeWorld3D{
              .observed_occupancy = fixture.occupancy.get(),
              .occupancy = nullptr,
              .occupied_content_fingerprint =
                  fixture.occupancy->occupiedSnapshot().contentFingerprint(),
              .occupancy_policy = TrackingErrorTubeOccupancyPolicy3D::kRawOccupiedOnly,
              .free_space_seed = nullptr,
              .launch_support_contact = nullptr,
          }))
          << fixture.name;
    }
  }
}

TEST(PersistentPlannerAcceptanceFixture,
     LoopRepairsResidentSearchAndUsesTheOtherRawSafeBranch) {
  const PersistentPlannerAcceptanceFixture fixture =
      buildPersistentPlannerAcceptanceFixture(PersistentPlannerAcceptanceKind::kLoop);
  ASSERT_EQ(fixture.missions.size(), 1U);
  const PersistentPlannerAcceptanceMission& mission = fixture.missions.front();
  PersistentDStarLitePlanner3D planner{acceptancePlannerConfig()};
  const PersistentPlannerResult3D initial =
      planMission(planner, mission, fixture.occupancy);
  ASSERT_TRUE(initial.executable());
  ASSERT_GE(initial.points.size(), 3U);

  auto changed = std::make_shared<ObservedOccupancyGrid3D>(*fixture.occupancy);
  const Point3 blocked_point = initial.points[initial.points.size() / 2U];
  const std::optional<GridIndex3D> blocked_cell = changed->worldToCell(blocked_point);
  const GridIndex3D changed_cell = blocked_cell.value_or(GridIndex3D{-1, -1, -1});
  ASSERT_TRUE(blocked_cell.has_value());
  ASSERT_TRUE(changed->setState(changed_cell, ObservedVoxelState::kOccupied));
  const PersistentPlannerResult3D repaired =
      planMission(planner, mission, changed, 2U,
                  {ObservedOccupancyGrid3D::chunkIndex(changed_cell)});

  ASSERT_TRUE(repaired.executable()) << persistentPlannerStatus3DName(repaired.status);
  EXPECT_TRUE(repaired.search_state_reused);
  EXPECT_EQ(repaired.search_generation, initial.search_generation);
  EXPECT_EQ(repaired.changed_occupied_voxels, 1U);
  EXPECT_GT(repaired.affected_lattice_states, 0U);
  EXPECT_TRUE(pathsDiffer(repaired.points, initial.points));
  expectRawSafe(repaired.points, *changed);
}

} // namespace
} // namespace drone_city_nav::test
