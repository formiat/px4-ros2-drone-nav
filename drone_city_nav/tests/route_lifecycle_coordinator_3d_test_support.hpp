#pragma once

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "execution_route_snapshot_3d_plan_test_support.hpp"
#include "execution_supervisor_horizon_3d_test_support.hpp"
#include "production_mppi_route_world.hpp"
#include "route_lifecycle_coordinator_3d.hpp"

namespace drone_city_nav {
namespace {

// The fixture clock stands at kFixtureStampNs, so a search requested at
// kFixtureSearchRequestStampNs has a measurable lead time when it activates.
inline constexpr std::int64_t kFixtureStampNs{100'000'000};
inline constexpr std::int64_t kFixtureSearchRequestStampNs{75'000'000};
inline constexpr double kFixtureSearchLatencyMs{25.0};

struct LifecycleFixture3D {
  std::shared_ptr<const WorldSnapshot3D> world;
  std::shared_ptr<const PersistentPlannerWorld3D> planner_world;
  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
  std::shared_ptr<const ProductionNavigationObjective> objective;
  ProductionMppiNavigation navigation{};
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
  config.planner.maximum_no_route_compute_time_ms = 1000.0;
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
  return config;
}

[[nodiscard]] RouteActivationCoordinatorConfig3D activationConfig() {
  RouteActivationCoordinatorConfig3D config;
  MotionDynamicsConfig3D dynamics;
  MotionAltitudeEnvelopeConfig3D altitude_envelope;
  config.route_tracking.maximum_cross_track_m = 2.0;
  config.route_extension.minimum_remaining_m = 1.0;
  config.route_extension.required_certified_overlap_m = 1.0;
  config.cruise_speed_mps = 5.0;
  config.maximum_control_feedback_age_ms = 1000.0;
  config.flight_envelope.minimum_target_z_m = 0.0;
  config.flight_envelope.maximum_target_z_m = 20.0;
  config.trajectory_compiler.trajectory.unconstrained_speed_mps =
      config.cruise_speed_mps;
  config.trajectory_compiler.trajectory.physical_footprint = config.physical_footprint;
  config.trajectory_compiler.trajectory.time_model.maximum_horizontal_speed_mps = 5.0;
  config.trajectory_compiler.trajectory.time_model.maximum_vertical_speed_mps = 2.0;
  config.trajectory_compiler.passage_volume.flight_envelope = config.flight_envelope;
  config.trajectory_compiler.passage_volume.footprint = config.physical_footprint;
  dynamics.linear_drag_1ps = 0.0F;
  dynamics.maximum_control_jerk_mps3 = 100.0F;
  config.route_risk = RouteRiskPolicy3D{
      .critical_distance_m = 1.0,
      .preferred_distance_m = 6.0,
  };
  config.dynamic_handoff_validator = [](const DynamicHandoffRequest3D& request) {
    return DynamicHandoffResult3D{
        .status = request.candidate_trajectory != nullptr &&
                          request.derived_distances_m != nullptr
                      ? DynamicHandoffStatus3D::kAccepted
                      : DynamicHandoffStatus3D::kInvalidInput,
    };
  };
  config.validation_policy = VersionedExecutionValidationPolicy3D::capture(
      config.flight_envelope, dynamics, altitude_envelope, config.physical_footprint,
      1000.0, 1000.0, 1000.0, true, false, true);
  return config;
}

[[nodiscard]] LifecycleFixture3D fixture() {
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 16, 16, 8};
  auto occupancy = std::make_shared<OccupancyGrid3D>(bounds, 71U);
  const std::uint64_t revision = occupancy->fingerprint();
  const std::uint64_t occupied_fingerprint = occupancy->contentFingerprint();
  auto world = std::make_shared<WorldSnapshot3D>();
  world->revision = revision;
  world->source_occupied_fingerprint = revision;
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
  world->distances_m = std::make_shared<const std::vector<float>>(voxel_count, 20.0F);
  world->static_occupancy = occupancy;
  world->local_world_generation = LocalWorldGeneration{
      .generation = 3U,
      .raw_map =
          RawMapVersion{
              .producer_instance_id = 0U,
              .base_snapshot_revision = revision,
              .revision = revision,
          },
      .pose_revision = 5U,
      .esdf_revision = revision,
      .gpu_esdf_revision = revision,
  };
  const Point3 start{1.5, 1.5, 1.5};
  const Point3 goal{8.5, 1.5, 1.5};
  const StaticRouteObjective route_objective{
      .goal = goal,
      .mission_epoch = 7U,
      .sample_sequence = 1U,
      .available = true,
  };
  const std::shared_ptr<const PersistentPlannerWorld3D> planner_world =
      captureResidentPlannerWorld3D(*world);
  std::shared_ptr<const WorldSnapshot3D> immutable_world = std::move(world);
  const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
      makePlannerSearchTransaction3D(immutable_world, planner_world, route_objective,
                                     StaticRouteSearchRequestIdentity{
                                         .kind = StaticRouteSearchRequestKind::kInitial,
                                         .base_route_generation = 0U,
                                     },
                                     std::nullopt, RouteReleaseReason3D::kNone,
                                     kFixtureSearchRequestStampNs);
  auto objective = std::make_shared<const ProductionNavigationObjective>(
      ProductionNavigationObjective{
          .goal = goal,
          .tracking = std::nullopt,
          .mission_epoch = route_objective.mission_epoch,
          .sample_sequence = route_objective.sample_sequence,
      });
  return LifecycleFixture3D{
      .world = std::move(immutable_world),
      .planner_world = planner_world,
      .transaction = transaction,
      .objective = std::move(objective),
      .navigation =
          ProductionMppiNavigation{
              .state =
                  MotionState3D{
                      .x = static_cast<float>(start.x),
                      .y = static_cast<float>(start.y),
                      .z = static_cast<float>(start.z),
                  },
              .receive_stamp_ns = 100,
              .source_timestamp_us = 90U,
              .revision = 11U,
              .valid = true,
          },
  };
}

[[nodiscard]] RoutePlannerVehicleState3D vehicleState(const LifecycleFixture3D& input) {
  return RoutePlannerVehicleState3D{
      .position = Point3{input.navigation.state.x, input.navigation.state.y,
                         input.navigation.state.z},
      .velocity = {},
      .valid = true,
  };
}

[[nodiscard]] ProductionRouteActivationSnapshot3D
activationSnapshot(const LifecycleFixture3D& input,
                   const ExecutionSupervisor3D& supervisor) {
  const RouteExecutionManagerSnapshot3D execution = supervisor.snapshot();
  return ProductionRouteActivationSnapshot3D{
      .resident_world = input.world,
      .execution_authority = execution.authority,
      .pending_route = execution.pending,
      .navigation = input.navigation,
      .objective = input.objective,
      .raw_world = nullptr,
      .minimum_tracking_route_mission_epoch = 0U,
      .minimum_tracking_route_sample_sequence = 0U,
      .stamp_ns = 100,
  };
}

[[nodiscard]] RouteLifecycleCoordinatorConfig3D
lifecycleConfig(const LifecycleFixture3D& input, ExecutionSupervisor3D& supervisor,
                std::size_t& commit_count) {
  RouteLifecycleCoordinatorConfig3D config;
  config.planner = plannerConfig();
  config.materializer = materializerConfig();
  config.activation = activationConfig();
  config.extension = config.planner.extension;
  config.flight_envelope = config.materializer.flight_envelope;
  config.static_route_lookahead_m = 100.0;
  config.tracking_world_refresh_margin_m = 1.0;
  config.vehicle_state_provider = [&input]() { return vehicleState(input); };
  config.activation_snapshot_provider = [&input, &supervisor]() {
    return activationSnapshot(input, supervisor);
  };
  config.activation_commit_boundary =
      [&input, &commit_count](PreparedRouteActivation3D prepared,
                              const RouteActivationCommitOperation3D& commit) {
        ++commit_count;
        return commit(std::move(prepared),
                      RouteActivationCommitContext3D{
                          .resident_world = input.world,
                          .objective = input.objective,
                          .raw_world = nullptr,
                          .minimum_tracking_route_mission_epoch = 0U,
                          .minimum_tracking_route_sample_sequence = 0U,
                      });
      };
  config.tracking_context_provider = [&input]() {
    return RouteLifecycleTrackingContext3D{.objective = input.objective};
  };
  config.replan_snapshot_provider = [&input]() {
    return RouteLifecycleReplanSnapshot3D{
        .navigation = input.navigation,
        .objective = input.objective,
        .resident_world = input.world,
        .resident_planner_world = input.planner_world,
        .latest_raw_world = nullptr,
        .world_telemetry = {},
        .committed_route_generation = 0U,
        .blocked_raw_revision = 0U,
        .minimum_route_mission_epoch = 0U,
        .minimum_route_sample_sequence = 0U,
        .stamp_ns = 100,
    };
  };
  config.world_refresh_requester = [](const std::uint64_t generation,
                                      const RouteLifecycleWorldRefreshPurpose3D) {
    return RouteLifecycleWorldRefreshResult3D{
        .sequence = 1U,
        .base_route_generation = generation,
    };
  };
  config.stamp_provider = []() { return kFixtureStampNs; };
  config.update_handler = [](RouteLifecycleUpdate3D) {};
  config.rejection_handler = [](const RoutePlanningRejection3D&) {};
  return config;
}

void bindResidentReplanSnapshot(RouteLifecycleCoordinatorConfig3D& config,
                                const LifecycleFixture3D& input,
                                ExecutionSupervisor3D& supervisor,
                                const std::int64_t stamp_ns) {
  config.replan_snapshot_provider = [&input, &supervisor, stamp_ns]() {
    const std::shared_ptr<const ExecutionPlan3D> execution = supervisor.plan();
    return RouteLifecycleReplanSnapshot3D{
        .navigation = input.navigation,
        .objective = input.objective,
        .resident_world = input.world,
        .resident_planner_world = input.planner_world,
        .latest_raw_world = nullptr,
        .world_telemetry = {},
        .committed_route_generation =
            execution != nullptr ? execution->routeGenerationHighWater() : 0U,
        .blocked_raw_revision = 0U,
        .minimum_route_mission_epoch = 0U,
        .minimum_route_sample_sequence = 0U,
        .stamp_ns = stamp_ns,
    };
  };
}

void activatePending(ExecutionSupervisor3D& supervisor) {
  const std::shared_ptr<const PendingCertifiedRoute3D> pending = supervisor.pending();
  const std::shared_ptr<const CommittedExecutionAuthority3D> authority =
      supervisor.authority();
  ASSERT_NE(pending, nullptr);
  ASSERT_NE(authority, nullptr);
  const std::shared_ptr<const ExecutionPlan3D> plan = authority->plan();
  ASSERT_NE(plan, nullptr);
  FiniteExecutionPlan3D finite = SnapshotFixture3D::finitePlanForRoute(
      *plan, pending->route, FiniteExecutionKind3D::kNominal, 100U);
  const ExecutionRouteTransitionResult3D transition =
      ::drone_city_nav::activateCertifiedRoute3D(*plan, plan->version, pending->route,
                                                 std::move(finite));
  ASSERT_TRUE(transition.applied());
  ASSERT_NE(transition.next, nullptr);
  const ExecutionHorizonCommitResult3D committed = commitExecutionHorizonForTest(
      supervisor, ExecutionHorizonTestTransaction3D{
                      .kind = ExecutionHorizonCommitKind3D::kPendingTransition,
                      .expected_authority = authority,
                      .expected_plan = plan,
                      .transition = transition,
                      .expected_pending = pending,
                      .owner = SnapshotFixture3D::committedOwner(*transition.next),
                      .input = SnapshotFixture3D::committedInput(*transition.next),
                  });
  ASSERT_EQ(committed.status, ExecutionHorizonCommitStatus3D::kCommitted);
}

void planAndActivateInitialRoute(RouteLifecycleCoordinator3D& coordinator,
                                 ExecutionSupervisor3D& supervisor,
                                 const LifecycleFixture3D& input) {
  RoutePlanner3D planner{plannerConfig()};
  const RoutePlannerVehicleState3D vehicle_state = vehicleState(input);
  RoutePlannerUpdate3D planner_update =
      planner.update(*input.transaction, vehicle_state);
  ASSERT_TRUE(planner_update.improved_incumbent.has_value());
  const RouteLifecycleUpdate3D activation =
      coordinator.advance(RoutePlanningUpdateEvent3D{
          .request =
              RoutePlanningRequest3D{
                  .transaction = input.transaction,
                  .continuation_session = nullptr,
              },
          .vehicle_state = vehicle_state,
          .update = std::move(planner_update),
      });
  ASSERT_TRUE(activation.activation.admission.certified_pending);
  activatePending(supervisor);
  ASSERT_NE(supervisor.plan(), nullptr);
  ASSERT_NE(supervisor.plan()->route(), nullptr);
}

} // namespace
} // namespace drone_city_nav
