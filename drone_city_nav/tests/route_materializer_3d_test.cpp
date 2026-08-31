#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "production_mppi_route_world.hpp"
#include "route_materializer_3d.hpp"

namespace drone_city_nav {
namespace {

struct MaterializerFixture3D {
  std::shared_ptr<const WorldSnapshot3D> world;
  std::shared_ptr<const PersistentPlannerWorld3D> planner_world;
  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
};

[[nodiscard]] RoutePlannerConfig3D plannerConfig() {
  RoutePlannerConfig3D config;
  config.planner.minimum_horizontal_step_m = 1.0;
  config.planner.minimum_vertical_step_m = 1.0;
  config.planner.maximum_adaptive_lattice_level = 0U;
  config.planner.time_model.maximum_horizontal_speed_mps = 5.0;
  config.planner.time_model.maximum_vertical_speed_mps = 2.0;
  config.planner.goal_tolerance_m = 0.01;
  config.planner.feasibility_first_enabled = false;
  config.planner.maximum_compute_time_ms = 1000.0;
  config.planner.maximum_expansions_per_update = 100000U;
  config.planner.physical_footprint.radius_m = 0.0;
  config.planner.physical_footprint.lower_extent_m = 0.0;
  config.planner.physical_footprint.upper_extent_m = 0.0;
  config.planner.physical_footprint.perimeter_samples = 0U;
  config.planner.physical_footprint.radial_rings = 0U;
  config.planner.physical_footprint.axial_samples = 0U;
  config.planner.physical_footprint.sweep_step_m = 0.2;
  config.planner.flight_envelope.minimum_target_z_m = 0.0;
  config.planner.flight_envelope.maximum_target_z_m = 20.0;
  config.route_sampling_step_m = 0.5;
  config.cruise_speed_mps = 5.0;
  return config;
}

[[nodiscard]] RouteMaterializerConfig3D materializerConfig() {
  RouteMaterializerConfig3D config;
  config.route_geometry.enabled = false;
  config.physical_footprint.radius_m = 0.0;
  config.physical_footprint.lower_extent_m = 0.0;
  config.physical_footprint.upper_extent_m = 0.0;
  config.physical_footprint.perimeter_samples = 0U;
  config.physical_footprint.radial_rings = 0U;
  config.physical_footprint.axial_samples = 0U;
  config.flight_envelope.minimum_target_z_m = 0.0;
  config.flight_envelope.maximum_target_z_m = 20.0;
  config.critical_distance_m = 1.5;
  config.preferred_distance_m = 4.0;
  config.worker_pool = nullptr;
  config.cooperative_traffic_enabled = false;
  return config;
}

[[nodiscard]] MaterializerFixture3D
fixture(const StaticRouteSearchRequestIdentity request =
            StaticRouteSearchRequestIdentity{
                .kind = StaticRouteSearchRequestKind::kInitial,
                .base_route_generation = 0U,
            },
        const RouteReleaseReason3D release_reason = RouteReleaseReason3D::kNone,
        const bool invalid_derived_distances = false) {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 16, 16, 8};
  auto occupancy = std::make_shared<OccupancyGrid3D>(bounds, 71U);
  const std::uint64_t revision = occupancy->fingerprint();
  const std::uint64_t occupied_fingerprint = occupancy->contentFingerprint();
  auto world = std::make_shared<WorldSnapshot3D>();
  world->revision = revision;
  world->source_occupied_fingerprint = occupied_fingerprint;
  world->raw_occupied_fingerprint = occupied_fingerprint;
  world->grid = EsdfGrid3D{
      .width = bounds.width_cells,
      .height = bounds.height_cells,
      .resolution_m = static_cast<float>(bounds.resolution_m),
      .origin_x_m = static_cast<float>(bounds.origin_x),
      .origin_y_m = static_cast<float>(bounds.origin_y),
      .depth = bounds.depth_cells,
      .origin_z_m = static_cast<float>(bounds.origin_z),
      .outside_is_unknown = true,
  };
  const std::size_t voxel_count = static_cast<std::size_t>(bounds.width_cells) *
                                  static_cast<std::size_t>(bounds.height_cells) *
                                  static_cast<std::size_t>(bounds.depth_cells);
  const float distance_value =
      invalid_derived_distances ? std::numeric_limits<float>::quiet_NaN() : 20.0F;
  world->distances_m =
      std::make_shared<const std::vector<float>>(voxel_count, distance_value);
  world->static_occupancy = occupancy;
  world->local_world_generation = LocalWorldGeneration{
      .generation = 3U,
      .raw_map = {.producer_instance_id = 0U,
                  .base_snapshot_revision = revision,
                  .revision = revision},
      .pose_revision = 5U,
      .esdf_revision = revision,
      .gpu_esdf_revision = revision,
  };
  const std::shared_ptr<const PersistentPlannerWorld3D> planner_world =
      captureResidentPlannerWorld3D(*world);
  std::shared_ptr<const WorldSnapshot3D> immutable_world = std::move(world);
  const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
      makePlannerSearchTransaction3D(immutable_world, planner_world,
                                     StaticRouteObjective{
                                         .goal = {12.5, 12.5, 4.5},
                                         .mission_epoch = 7U,
                                         .available = true,
                                     },
                                     request, std::nullopt, release_reason);
  return MaterializerFixture3D{
      .world = std::move(immutable_world),
      .planner_world = planner_world,
      .transaction = transaction,
  };
}

[[nodiscard]] RoutePlannerVehicleState3D vehicleState() {
  return RoutePlannerVehicleState3D{
      .position = Point3{1.5, 1.5, 1.5},
      .velocity = Vec3{0.0, 0.0, 0.0},
      .valid = true,
  };
}

[[nodiscard]] RouteSearchCandidate3D
plannerCandidate(const std::shared_ptr<const PlannerSearchTransaction3D>& transaction) {
  if (transaction == nullptr) {
    throw std::invalid_argument{"planner transaction is unavailable"};
  }
  RoutePlanner3D planner{plannerConfig()};
  RoutePlannerUpdate3D update = planner.update(*transaction, vehicleState());
  if (!update.improved_incumbent.has_value()) {
    throw std::runtime_error{"planner did not produce a materializer fixture"};
  }
  return std::move(update.improved_incumbent).value_or(RouteSearchCandidate3D{});
}

[[nodiscard]] RouteMaterializationRequest3D
requestFor(const MaterializerFixture3D& input, RouteSearchCandidate3D candidate,
           const std::uint64_t candidate_generation = 8U) {
  return RouteMaterializationRequest3D{
      .transaction = input.transaction,
      .world_telemetry = {.build_ms = 12.5},
      .current_position = vehicleState().position,
      .candidate = std::move(candidate),
      .candidate_generation = candidate_generation,
      .active_route = nullptr,
      .activation_raw_world = nullptr,
  };
}

TEST(RouteMaterializer3DTest, RejectsInvalidConfigurationAndRequest) {
  RouteMaterializerConfig3D invalid = materializerConfig();
  invalid.preferred_distance_m = invalid.critical_distance_m - 1.0;
  EXPECT_THROW(static_cast<void>(RouteMaterializer3D{invalid}), std::invalid_argument);

  RouteMaterializer3D materializer{materializerConfig()};
  const ProductionRouteMaterialization3D result = materializer.materialize({});
  EXPECT_EQ(result.validation.status, StaticRouteCandidateStatus::kInvalidInput);
  EXPECT_FALSE(result.validation.accepted);
  EXPECT_EQ(result.route.route, nullptr);
}

TEST(RouteMaterializer3DTest,
     MaterializesOneImmutablePlannerCandidateWithoutNodeState) {
  const MaterializerFixture3D input = fixture();
  ASSERT_NE(input.transaction, nullptr);
  RouteMaterializer3D materializer{materializerConfig()};

  const ProductionRouteMaterialization3D result =
      materializer.materialize(requestFor(input, plannerCandidate(input.transaction)));

  EXPECT_TRUE(result.validation.accepted);
  EXPECT_EQ(result.route.world, input.world);
  EXPECT_EQ(result.route.objective.mission_epoch, 7U);
  EXPECT_EQ(result.route.candidate_generation, 8U);
  ASSERT_NE(result.route.route, nullptr);
  EXPECT_GE(result.route.route->size(), 2U);
  EXPECT_NE(result.route.fingerprint, 0U);
  EXPECT_TRUE(result.route.initial_projection.valid);
  EXPECT_TRUE(result.telemetry.planner.invoked);
  EXPECT_DOUBLE_EQ(result.telemetry.world_build.build_ms, 12.5);
  EXPECT_FALSE(result.geometry_optimization_fallback.has_value());
}

TEST(RouteMaterializer3DTest,
     InvalidDerivedDistanceEvidenceRemainsNeutralForRawSafeRoute) {
  const MaterializerFixture3D input = fixture(
      StaticRouteSearchRequestIdentity{
          .kind = StaticRouteSearchRequestKind::kInitial,
          .base_route_generation = 0U,
      },
      RouteReleaseReason3D::kNone, true);
  ASSERT_NE(input.transaction, nullptr);
  RouteMaterializer3D materializer{materializerConfig()};

  const ProductionRouteMaterialization3D result =
      materializer.materialize(requestFor(input, plannerCandidate(input.transaction)));

  ASSERT_TRUE(result.validation.accepted);
  ASSERT_NE(result.route.route, nullptr);
  for (const RouteSample3D& sample : *result.route.route) {
    EXPECT_EQ(sample.required_risk_tier, RouteRiskTier3D::kPreferred);
  }
}

TEST(RouteMaterializer3DTest, OverlapCandidateRequiresTheExactCertifiedActiveRoute) {
  const MaterializerFixture3D input = fixture(
      StaticRouteSearchRequestIdentity{
          .kind = StaticRouteSearchRequestKind::kReplan,
          .base_route_generation = 4U,
      },
      RouteReleaseReason3D::kBlocked);
  ASSERT_NE(input.transaction, nullptr);
  RouteSearchCandidate3D candidate = plannerCandidate(input.transaction);
  candidate.search_base_route_instance_id = RouteInstanceId3D{.value = 91U};
  candidate.search_base_stitch_station_m = 3.0;
  RouteMaterializer3D materializer{materializerConfig()};

  const ProductionRouteMaterialization3D result =
      materializer.materialize(requestFor(input, std::move(candidate)));

  EXPECT_EQ(result.validation.status, StaticRouteCandidateStatus::kInvalidPassageSpan);
  EXPECT_FALSE(result.validation.accepted);
  EXPECT_EQ(result.replacement_policy,
            StaticRouteReplacementPolicy::kAllowSafetyReplan);
}

} // namespace
} // namespace drone_city_nav
