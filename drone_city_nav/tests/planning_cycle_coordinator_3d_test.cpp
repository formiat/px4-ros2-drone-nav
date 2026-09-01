#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "planning_cycle_coordinator_3d.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] ExecutionStateProvenance3D authoritativeStateProvenance() {
  const ExecutionStateFieldProvenance3D source{
      ExecutionStateFieldProvenance3D::kSourceSample};
  return ExecutionStateProvenance3D{
      .x = source,
      .y = source,
      .z = source,
      .vx = source,
      .vy = source,
      .vz = source,
      .yaw = source,
      .yaw_rate = source,
  };
}

[[nodiscard]] std::shared_ptr<const VersionedExecutionInput3D> executionInput() {
  return VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
      .capture_sequence = 11U,
      .pose_revision = 13U,
      .pose_source_timestamp_us = 17U,
      .pose_receive_stamp_ns = 5'010'000'000,
      .effective_stamp_ns = 5'020'000'000,
      .state = MotionState3D{.x = 2.0F, .y = 3.0F, .z = 4.0F, .yaw = 0.5F},
      .full_state_authoritative = true,
      .state_provenance = authoritativeStateProvenance(),
      .previous_control_source =
          ExecutionPreviousControlEvidenceSource3D::kOffboardFeedback,
      .previous_control_source_producer_instance_id = 17U,
      .previous_control_source_sequence = 19U,
      .previous_control_source_stamp_ns = 5'015'000'000,
      .previous_control_receive_stamp_ns = 5'016'000'000,
  });
}

[[nodiscard]] WorldSnapshot3D worldSnapshot() {
  WorldSnapshot3D world;
  world.revision = 23U;
  world.grid = EsdfGrid3D{
      .width = 8,
      .height = 8,
      .resolution_m = 1.0F,
      .origin_x_m = 0.0F,
      .origin_y_m = 0.0F,
      .depth = 8,
      .origin_z_m = 0.0F,
      .outside_is_unknown = false,
  };
  world.distances_m = std::make_shared<const std::vector<float>>(8U * 8U * 8U, 20.0F);
  world.local_world_generation.gpu_esdf_revision = world.revision;
  return world;
}

[[nodiscard]] PlanningCycleCoordinatorConfig3D coordinatorConfig() {
  PlanningCycleCoordinatorConfig3D config;
  config.route_execution.flight_envelope =
      FlightEnvelopeConfig{.minimum_target_z_m = 1.0, .maximum_target_z_m = 32.0};
  config.flight_envelope = config.route_execution.flight_envelope;
  config.horizon_steps = 8U;
  config.dynamics.dt_s = 0.1F;
  config.tracking_capture_radius_m = 5.0;
  config.route_constraint_diagnostics_distance_m = 30.0;
  return config;
}

[[nodiscard]] PlanningCycleRequest3D
requestFor(const WorldSnapshot3D& world,
           const ProductionNavigationObjective& objective) {
  PlanningCycleRequest3D request;
  request.world = &world;
  request.objective = &objective;
  request.navigation.state = mppi::State{.x = 2.0F, .y = 3.0F, .z = 4.0F, .yaw = 0.5F};
  request.execution_input = executionInput();
  request.mission_goal = objective.goal;
  request.minimum_tracking_sample_sequence = objective.sample_sequence;
  request.world_revision = world.revision;
  request.now_ns = 5'020'000'000;
  request.control_feedback_fresh = true;
  request.terminal_hold_enabled = true;
  request.use_static_map = true;
  return request;
}

TEST(PlanningCycleCoordinator3DTest,
     InvalidRequestReturnsTypedDispositionWithoutMutatingExecution) {
  ExecutionSupervisor3D supervisor;
  PlanningCycleCoordinator3D coordinator{supervisor, coordinatorConfig()};
  const std::shared_ptr<const CommittedExecutionAuthority3D> before =
      supervisor.authority();

  const PlanningCycleOutcome3D result = coordinator.prepare({});

  EXPECT_EQ(result.status, PlanningCycleStatus3D::kInvalidRequest);
  EXPECT_STREQ(planningCycleStatus3DName(result.status), "invalid_request");
  EXPECT_EQ(supervisor.authority(), before);
  EXPECT_TRUE(result.effects.route_execution.empty());
}

TEST(PlanningCycleCoordinator3DTest,
     NoActiveRouteProducesOneTypedStationaryControllerCycle) {
  ExecutionSupervisor3D supervisor;
  PlanningCycleCoordinator3D coordinator{supervisor, coordinatorConfig()};
  const WorldSnapshot3D world = worldSnapshot();
  ProductionNavigationObjective objective;
  objective.goal = Point3{7.0, 8.0, 9.0};
  objective.mission_epoch = 29U;
  objective.sample_sequence = 31U;

  const PlanningCycleOutcome3D result =
      coordinator.prepare(requestFor(world, objective));

  ASSERT_TRUE(result.ready());
  EXPECT_FALSE(result.route.usable);
  EXPECT_EQ(result.route.execution_status, RouteExecutionStatus3D::kNoActiveRoute);
  EXPECT_EQ(result.controller.planning_state,
            ProductionMppiPlanningState::kNoExecutableRouteHold);
  EXPECT_EQ(result.controller.request.mode, MppiControllerMode3D::kStationaryHold);
  EXPECT_FLOAT_EQ(result.controller.request.input.target.x, 2.0F);
  EXPECT_FLOAT_EQ(result.controller.request.input.target.y, 3.0F);
  EXPECT_FLOAT_EQ(result.controller.request.input.target.z, 4.0F);
  EXPECT_EQ(result.controller.target_source, "no_executable_route_no_active_route");
  EXPECT_TRUE(result.effects.request_pending_successor);
  EXPECT_TRUE(result.effects.request_route_extension);
}

TEST(PlanningCycleCoordinator3DTest,
     MissionCommandHoldOverridesRouteFallbackInCoordinatorPolicy) {
  ExecutionSupervisor3D supervisor;
  PlanningCycleCoordinator3D coordinator{supervisor, coordinatorConfig()};
  const WorldSnapshot3D world = worldSnapshot();
  ProductionNavigationObjective objective;
  objective.goal = Point3{7.0, 8.0, 9.0};
  objective.mission_epoch = 37U;
  objective.sample_sequence = 41U;
  objective.immediate_hold = true;

  const PlanningCycleOutcome3D result =
      coordinator.prepare(requestFor(world, objective));

  ASSERT_TRUE(result.ready());
  EXPECT_EQ(result.controller.planning_state,
            ProductionMppiPlanningState::kMissionCommandPositionHold);
  EXPECT_FLOAT_EQ(result.controller.request.input.target.x, 7.0F);
  EXPECT_FLOAT_EQ(result.controller.request.input.target.y, 8.0F);
  EXPECT_FLOAT_EQ(result.controller.request.input.target.z, 9.0F);
  EXPECT_FLOAT_EQ(result.controller.request.input.reference_speed_mps, 0.0F);
  EXPECT_EQ(result.controller.target_source, "mission_command_position_hold");
}

TEST(PlanningCycleCoordinator3DTest,
     NonCooperativeTrackInfluenceReachesTheControllerRequest) {
  ExecutionSupervisor3D supervisor;
  PlanningCycleCoordinatorConfig3D config = coordinatorConfig();
  config.noncooperative_avoidance_enabled = true;
  PlanningCycleCoordinator3D coordinator{supervisor, config};
  const WorldSnapshot3D world = worldSnapshot();
  ProductionNavigationObjective objective;
  objective.goal = Point3{7.0, 8.0, 9.0};
  objective.mission_epoch = 43U;
  objective.sample_sequence = 47U;
  PlanningCycleRequest3D request = requestFor(world, objective);
  request.noncooperative_tracks = ProductionMppiNonCooperativeTracks{
      .tracks = {NonCooperativeAircraftTrack{
          .local_track_id = 53U,
          .position = Point3{7.0, 3.0, 4.0},
          .velocity = Vec3{},
          .measurement_stamp_ns = request.now_ns,
          .position_valid = true,
          .velocity_valid = true,
      }},
      .source_scan_sequence = 59U,
      .receive_stamp_ns = request.now_ns,
  };

  const PlanningCycleOutcome3D result = coordinator.prepare(request);

  ASSERT_TRUE(result.ready());
  EXPECT_TRUE(result.controller.noncooperative.enabled);
  EXPECT_TRUE(
      result.controller.noncooperative.avoidance.influence.cost_influence_active);
  EXPECT_TRUE(
      result.controller.noncooperative.avoidance.influence.evasive_maneuver_active);
  ASSERT_EQ(result.controller.request.input.dynamic_aircraft.size(), 1U);
  ASSERT_TRUE(result.controller.request.input.dynamic_aircraft_cost_policy.has_value());
  const mppi::DynamicAircraftCostPolicy cost_policy =
      result.controller.request.input.dynamic_aircraft_cost_policy.value_or(
          mppi::DynamicAircraftCostPolicy{});
  EXPECT_FLOAT_EQ(
      cost_policy.strong_weight,
      static_cast<float>(config.noncooperative_avoidance.strong_cost_weight));
  EXPECT_TRUE(result.controller.request.input.noncooperative_acquisition.has_value());
  EXPECT_TRUE(result.controller.request.input.noncooperative_avoidance_active);
}

TEST(PlanningCycleCoordinator3DTest, NamesEveryCycleDisposition) {
  EXPECT_STREQ(planningCycleStatus3DName(PlanningCycleStatus3D::kReady), "ready");
  EXPECT_STREQ(planningCycleStatus3DName(PlanningCycleStatus3D::kInvalidRequest),
               "invalid_request");
  EXPECT_STREQ(
      planningCycleStatus3DName(PlanningCycleStatus3D::kTrackingHandoffRetained),
      "tracking_handoff_retained");
}

} // namespace
} // namespace drone_city_nav
