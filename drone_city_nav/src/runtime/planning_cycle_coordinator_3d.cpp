#include "planning_cycle_coordinator_3d.hpp"

#include "drone_city_nav/cooperative_mppi_adapter.hpp"
#include "drone_city_nav/esdf_query.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/intercept_guidance.hpp"
#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/pending_certified_route_3d.hpp"
#include "drone_city_nav/route_execution_contract_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <utility>

#include "production_mppi_raw_world.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] mppi::DeterministicCandidateKind planningDeterministicCandidate(
    const bool direct_tracking_interception,
    const ProductionMppiPlanningState planning_state, const bool route_usable,
    const bool route_projection_valid, const bool route_hold) noexcept {
  if (direct_tracking_interception) {
    return mppi::DeterministicCandidateKind::kTargetDirectedReacquisition;
  }
  if (planning_state == ProductionMppiPlanningState::kPlanned && route_usable &&
      route_projection_valid && !route_hold) {
    return mppi::DeterministicCandidateKind::kRouteDirectedCruise;
  }
  return mppi::DeterministicCandidateKind::kDisabled;
}

[[nodiscard]] std::uint64_t planningRawRevision(
    const bool use_static_map, const std::uint64_t esdf_revision,
    const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world) noexcept {
  if (use_static_map) {
    return esdf_revision;
  }
  return latest_raw_world != nullptr ? latest_raw_world->version().revision
                                     : esdf_revision;
}

[[nodiscard]] mppi::State
selectTarget(const std::span<const RouteSample3D> route,
             const std::span<const mppi::RouteSample3D> controller_route,
             const Point3& mission_goal, const double current_station_m,
             const double lookahead_m, std::string& target_source,
             double& target_station_m) {
  mppi::State target{static_cast<float>(mission_goal.x),
                     static_cast<float>(mission_goal.y),
                     static_cast<float>(mission_goal.z)};
  target_source = "mission_goal_direct";
  target_station_m = 0.0;
  const double desired_station_m = current_station_m + std::max(0.0, lookahead_m);
  if (!route.empty()) {
    const RouteSample3D sample = sampleRoute3DAtStation(route, desired_station_m);
    target.x = static_cast<float>(sample.position.x);
    target.y = static_cast<float>(sample.position.y);
    target.z = static_cast<float>(sample.position.z);
    target_station_m = sample.station_m;
    target_source = "persistent_route_3d";
    return target;
  }
  if (!controller_route.empty()) {
    const float desired_station = static_cast<float>(desired_station_m);
    const auto selected = std::ranges::lower_bound(controller_route, desired_station,
                                                   {}, &mppi::RouteSample3D::station_m);
    const mppi::RouteSample3D& sample =
        selected == controller_route.end() ? controller_route.back() : *selected;
    target.x = sample.x_m;
    target.y = sample.y_m;
    target.z = sample.z_m;
    target_station_m = sample.station_m;
    target_source = "persistent_route_3d";
  }
  return target;
}

[[nodiscard]] ProductionMppiCooperativeUpdate prepareCooperativeUpdate(
    const PlanningCycleCoordinatorConfig3D& config,
    const std::span<const CooperativePassageAssignment> passage_assignments,
    const ConstrainedRouteObservation& route_observation,
    const std::optional<ProductionMppiCooperativeCommand>& command,
    const std::int64_t now_ns, const double planned_speed_mps) {
  ProductionMppiCooperativeUpdate result;
  if (!config.cooperative_traffic_enabled) {
    return result;
  }

  const CooperativePassageAssignment* assignment = nullptr;
  if (route_observation.span_available &&
      route_observation.span_index < passage_assignments.size()) {
    const CooperativePassageAssignment& candidate =
        passage_assignments[route_observation.span_index];
    if (candidate.span_index == route_observation.span_index &&
        candidate.route_generation == route_observation.route_generation &&
        candidate.passage_traversal_id == route_observation.passage_traversal_id) {
      assignment = std::addressof(candidate);
    }
  }
  if (assignment != nullptr) {
    result.passage =
        makeCooperativePassageUse(route_observation, *assignment, now_ns,
                                  planned_speed_mps, config.cooperative_timing);
  }
  if (!command.has_value()) {
    return result;
  }

  result.command_generation = command->data.command_generation;
  result.command_age_ms =
      now_ns >= command->data.stamp_ns
          ? static_cast<double>(now_ns - command->data.stamp_ns) / 1.0e6
          : -1.0;
  result.mppi = adaptCooperativeMppiCommand(command->data, config.vehicle_id, now_ns,
                                            config.horizon_steps, config.dynamics.dt_s);
  result.yield = evaluateCooperativePassageYield(
      command->data, result.passage, route_observation, config.vehicle_id, now_ns,
      route_observation.actual_horizontal_speed_mps, config.cooperative_yield);
  return result;
}

[[nodiscard]] ProductionMppiNonCooperativeUpdate prepareNonCooperativeUpdate(
    NonCooperativeCollisionAvoidance* const avoidance,
    const PlanningCycleCoordinatorConfig3D& config, const mppi::State& ownship,
    const ProductionMppiNonCooperativeTracks& tracks, const std::int64_t now_ns) {
  ProductionMppiNonCooperativeUpdate result{
      .source_scan_sequence = tracks.source_scan_sequence,
      .transport_age_ms = tracks.receive_stamp_ns > 0
                              ? static_cast<double>(std::max<std::int64_t>(
                                    0, now_ns - tracks.receive_stamp_ns)) /
                                    1.0e6
                              : -1.0,
      .enabled = config.noncooperative_avoidance_enabled,
  };
  if (!config.noncooperative_avoidance_enabled || avoidance == nullptr) {
    return result;
  }
  result.avoidance = avoidance->update(NonCooperativeAvoidanceInput{
      .ownship = ownship,
      .tracks = tracks.tracks,
      .now_ns = now_ns,
      .horizon_steps = config.horizon_steps,
      .step_s = config.dynamics.dt_s,
  });
  return result;
}

[[nodiscard]] std::optional<PassageGeometryProximity3D>
observePassageGeometry(const WorldSnapshot3D& world, const Point3& actual_position,
                       std::vector<PassageGeometryObservation>& observations) {
  if (world.topology_passage_traversals == nullptr) {
    return std::nullopt;
  }
  observations.reserve(world.topology_passage_traversals->size());
  const PassageTraversalEdge* nearest = nullptr;
  RouteProjection3D nearest_projection;
  double nearest_entry_distance_m = std::numeric_limits<double>::infinity();
  for (const PassageTraversalEdge& passage : *world.topology_passage_traversals) {
    const RouteProjection3D projection =
        projectOntoRoute3D(passage.centerline, actual_position);
    const double entry_distance_m = distance3D(actual_position, passage.entry);
    if (entry_distance_m < nearest_entry_distance_m) {
      nearest = std::addressof(passage);
      nearest_projection = projection;
      nearest_entry_distance_m = entry_distance_m;
    }
    observations.push_back(PassageGeometryObservation{
        .passage_traversal_id = passage.id,
        .within_corridor =
            projection.valid && projection.distance_m <= passage.minimum_clearance_m,
        .station_m = projection.station_m,
        .traversal_length_m =
            passage.centerline.empty() ? 0.0 : passage.centerline.back().station_m,
        .cross_track_error_m = projection.distance_m,
    });
  }
  if (nearest == nullptr || nearest_entry_distance_m >= 8.0) {
    return std::nullopt;
  }
  return PassageGeometryProximity3D{
      .passage_traversal_id = nearest->id,
      .entry_distance_m = nearest_entry_distance_m,
      .projection_station_m = nearest_projection.station_m,
      .projection_cross_track_m = nearest_projection.distance_m,
      .minimum_clearance_m = nearest->minimum_clearance_m,
      .projection_valid = nearest_projection.valid,
      .within_corridor = nearest_projection.valid &&
                         nearest_projection.distance_m <= nearest->minimum_clearance_m,
  };
}

} // namespace

bool PlanningCycleRequest3D::valid() const noexcept {
  // A stationary capture rearm input carries an assumed-zero previous control
  // instead of a nominal one; the cycle it prepares is the goal hold, which
  // plans no motion from it.
  return world != nullptr && world->distances_m != nullptr &&
         execution_input != nullptr && execution_input->valid() &&
         (execution_input->nominalStateAuthoritative() ||
          execution_input->stationaryCaptureStateAuthoritative()) &&
         now_ns > 0;
}

const char* planningCycleStatus3DName(const PlanningCycleStatus3D status) noexcept {
  switch (status) {
    case PlanningCycleStatus3D::kReady:
      return "ready";
    case PlanningCycleStatus3D::kInvalidRequest:
      return "invalid_request";
    case PlanningCycleStatus3D::kTrackingHandoffRetained:
      return "tracking_handoff_retained";
  }
  return "invalid_request";
}

PlanningCycleCoordinator3D::PlanningCycleCoordinator3D(
    ExecutionSupervisor3D& execution_supervisor,
    PlanningCycleCoordinatorConfig3D config)
    : execution_supervisor_{execution_supervisor},
      config_{std::move(config)},
      route_execution_selector_{execution_supervisor_, config_.route_execution},
      liveness_supervisor_{config_.liveness},
      goal_capture_latch_{config_.goal_capture},
      direct_tracking_maneuver_lifecycle_{config_.direct_tracking} {
  if (config_.horizon_steps == 0U || !(config_.dynamics.dt_s > 0.0F) ||
      !(config_.tracking_capture_radius_m > 0.0) ||
      !(config_.route_constraint_diagnostics_distance_m >= 0.0) ||
      (config_.cooperative_traffic_enabled &&
       config_.noncooperative_avoidance_enabled)) {
    throw std::invalid_argument{"invalid planning cycle coordinator configuration"};
  }
  if (config_.route_progress.has_value()) {
    route_progress_tracker_ =
        std::make_unique<RouteProgressTracker3D>(*config_.route_progress);
  }
  if (config_.noncooperative_avoidance_enabled) {
    noncooperative_avoidance_ = std::make_unique<NonCooperativeCollisionAvoidance>(
        config_.noncooperative_avoidance);
  }
}

bool PlanningCycleCoordinator3D::goalCaptureLatchedFor(
    const Point3& mission_goal) const noexcept {
  return goal_capture_latch_.latchedFor(mission_goal);
}

PlanningCycleOutcome3D
PlanningCycleCoordinator3D::prepare(const PlanningCycleRequest3D& request) {
  PlanningCycleOutcome3D output;
  if (!request.valid()) {
    return output;
  }

  RouteExecutionSelectorResult3D route_selection =
      route_execution_selector_.select(RouteExecutionSelectorRequest3D{
          .world = request.world,
          .objective = request.objective,
          .navigation = request.navigation,
          .execution_input = request.execution_input,
          .latest_raw_world = request.latest_raw_world,
          .latest_lidar_evidence = request.latest_lidar_evidence,
          .validation_stamp_ns = request.now_ns,
          .minimum_tracking_sample_sequence = request.minimum_tracking_sample_sequence,
          .physically_invalidated_through_generation =
              request.physically_invalidated_through_generation,
          .direct_tracking_identity = request.direct_tracking_identity,
          .observed_3d_world = request.observed_3d_world,
      });
  output.effects.route_execution = std::move(route_selection.effects);
  output.route.execution = std::move(route_selection.selection);
  const PendingCertifiedRouteRecoveryResult3D pending_recovery =
      recoverPendingCertifiedRouteLiveness3D(
          execution_supervisor_, output.route.execution.pending_route,
          PendingCertifiedRouteRecoveryObservation3D{
              .direct_tracking_requested =
                  output.route.execution.direct_tracking_identity.has_value(),
              // A stationary hold owns the wire but executes no route; the
              // successor search it waits for must not be suppressed by it.
              .execution_owner_available =
                  output.route.execution.execution_owner_available &&
                  !output.route.execution.stationary_hold_owner,
              .pending_activation = output.route.execution.pending_activation,
          });
  output.effects.request_pending_successor = pending_recovery.request_successor;
  if (output.route.execution.tracking_error_tube_handoff_active) {
    output.status = PlanningCycleStatus3D::kTrackingHandoffRetained;
    return output;
  }

  const CertifiedRouteSuffix3D* const activated_route =
      output.route.execution.route.get();
  const std::shared_ptr<const CompiledTrajectory3D> route_geometry =
      activated_route != nullptr ? activated_route->geometry : nullptr;
  const std::shared_ptr<const RouteDecorations3D> route_decorations =
      activated_route != nullptr ? activated_route->decorations : nullptr;
  output.route.generation =
      activated_route != nullptr ? activated_route->identity.generation : 0U;
  const bool route_reaches_mission_goal =
      activated_route != nullptr &&
      activated_route->identity.proposal.reaches_mission_goal;
  RouteEndpointSemantics3D route_endpoint_semantics =
      RouteEndpointSemantics3D::kLocalStop;
  if (activated_route != nullptr) {
    route_endpoint_semantics = activated_route->planned_endpoint_semantics;
  } else if (route_reaches_mission_goal) {
    route_endpoint_semantics = request.terminal_hold_enabled
                                   ? RouteEndpointSemantics3D::kMissionStop
                                   : RouteEndpointSemantics3D::kContinuation;
  }

  const std::shared_ptr<const std::vector<RouteSample3D>> execution_route =
      route_geometry != nullptr ? route_geometry->route : nullptr;
  output.route.controller_route = trajectory_reference_adapter_.adapt(route_geometry);
  const std::shared_ptr<const std::vector<ConstrainedRouteSpan>> constrained_spans =
      route_geometry != nullptr ? route_geometry->constrained_spans : nullptr;
  const std::shared_ptr<const std::vector<CooperativePassageAssignment>>
      passage_assignments_owner =
          route_decorations != nullptr
              ? route_decorations->cooperative_passage_assignments
              : nullptr;
  output.route.selected_passage_traversal_ids =
      route_decorations != nullptr ? route_decorations->selected_passage_traversal_ids
                                   : nullptr;
  output.route.usable = output.route.execution.route_usable;
  output.route.execution_status = output.route.execution.status;
  output.route.projection = output.route.execution.projection;
  if (output.route.projection.valid) {
    output.route.projection.station_m = output.route.execution.station_m;
    output.route.projection.remaining_m = std::max(
        0.0, output.route.projection.total_length_m - output.route.execution.station_m);
  }
  output.effects.request_static_tracking_world_refresh =
      request.use_static_map && request.objective != nullptr &&
      request.objective->continuous_tracking;
  output.effects.request_route_extension =
      request.use_static_map || request.observed_3d_world;

  const std::span<const RouteSample3D> route =
      output.route.usable && execution_route != nullptr
          ? std::span<const RouteSample3D>{*execution_route}
          : std::span<const RouteSample3D>{};
  const std::span<const ConstrainedRouteSpan> spans =
      output.route.usable && constrained_spans != nullptr
          ? std::span<const ConstrainedRouteSpan>{*constrained_spans}
          : std::span<const ConstrainedRouteSpan>{};
  const Point3 actual_position{request.navigation.state.x, request.navigation.state.y,
                               request.navigation.state.z};
  const ConstrainedRouteObservation route_constraint = observeConstrainedRoute(
      route, spans, output.route.generation, output.route.projection.station_m,
      actual_position,
      Vec3{request.navigation.state.vx, request.navigation.state.vy,
           request.navigation.state.vz},
      config_.route_envelope, config_.route_constraint_diagnostics_distance_m);
  output.effects.passage_traversal_events = passage_traversal_evidence_tracker_.update(
      route_constraint, actual_position, request.now_ns);
  std::vector<PassageGeometryObservation> passage_geometry_observations;
  output.effects.passage_geometry_proximity = observePassageGeometry(
      *request.world, actual_position, passage_geometry_observations);
  output.effects.passage_geometry_events = passage_geometry_evidence_tracker_.update(
      passage_geometry_observations, actual_position, request.now_ns,
      PassageGeometryEvidenceConfig{});

  const ConstrainedRouteControl route_control = constrained_route_coordinator_.update(
      route_constraint, config_.speed_policy.cruise_speed_mps,
      config_.constrained_route_control);
  output.controller.goal_capture =
      request.terminal_hold_enabled
          ? goal_capture_latch_.update(MissionGoalCaptureObservation{
                .mission_goal = request.mission_goal,
                .state = request.navigation.state,
                .terminal_route_available = route_reaches_mission_goal,
            })
          : MissionGoalCaptureResult{};
  output.route.local_stop_is_terminal =
      output.route.usable && output.route.projection.valid &&
      route_endpoint_semantics == RouteEndpointSemantics3D::kLocalStop;
  output.controller.speed_policy = evaluateMppiSpeedPolicy(
      config_.speed_policy,
      MppiSpeedPolicyInput{
          .state = request.navigation.state,
          .mission_goal = request.mission_goal,
          .route = route,
          .route_endpoint_remaining_m =
              output.route.usable && output.route.projection.valid &&
                      routeEndpointHasTerminalStop3D(route_endpoint_semantics)
                  ? std::optional<double>{output.route.projection.remaining_m}
                  : std::nullopt,
          .route_constraint_speed_limit_mps =
              route_control.active
                  ? std::optional<double>{route_control.speed_limit_mps}
                  : std::nullopt,
          .blocked_route_remaining_m =
              output.route.usable && output.route.projection.valid &&
                      output.route.execution.raw_blocked_station_m.has_value()
                  ? std::optional<double>{std::max(
                        0.0, *output.route.execution.raw_blocked_station_m -
                                 output.route.projection.station_m)}
                  : std::nullopt,
          .executed_horizon_clearance_m =
              request.previous_result != nullptr &&
                      !request.previous_result->horizon.empty() &&
                      request.previous_result->post_update_classification.executable
                  ? std::optional<double>{static_cast<double>(
                        request.previous_result->minimum_esdf_distance_m)}
                  : std::nullopt,
          .route_endpoint_semantics = route_endpoint_semantics,
          .terminal_goal_limit_enabled = request.terminal_hold_enabled,
      });
  const std::span<const CooperativePassageAssignment> passage_assignments =
      passage_assignments_owner != nullptr
          ? std::span<const CooperativePassageAssignment>{*passage_assignments_owner}
          : std::span<const CooperativePassageAssignment>{};
  output.controller.cooperative = prepareCooperativeUpdate(
      config_, passage_assignments, route_constraint, request.cooperative_command,
      request.now_ns, output.controller.speed_policy.reference_speed_mps);
  output.controller.noncooperative = prepareNonCooperativeUpdate(
      noncooperative_avoidance_.get(), config_, request.navigation.state,
      request.noncooperative_tracks, request.now_ns);
  const bool noncooperative_cost_influence_active =
      output.controller.noncooperative.enabled &&
      output.controller.noncooperative.avoidance.influence.cost_influence_active;
  const bool noncooperative_evasive_maneuver_active =
      output.controller.noncooperative.enabled &&
      output.controller.noncooperative.avoidance.influence.evasive_maneuver_active;
  if (output.controller.cooperative.yield.active) {
    output.controller.speed_policy.reference_speed_mps =
        std::min(output.controller.speed_policy.reference_speed_mps,
                 output.controller.cooperative.yield.maximum_speed_mps);
    output.controller.speed_policy.target_lookahead_m =
        std::min(output.controller.speed_policy.target_lookahead_m,
                 std::max(0.0, output.controller.cooperative.yield.hold_station_m -
                                   output.route.projection.station_m));
  }

  const bool route_execution_blocked = !request.direct_tracking_interception &&
                                       request.objective != nullptr &&
                                       !output.route.usable;
  double target_station_m = 0.0;
  mppi::State target;
  if (request.direct_tracking_interception) {
    target = mppi::State{
        .x = static_cast<float>(request.mission_goal.x),
        .y = static_cast<float>(request.mission_goal.y),
        .z = static_cast<float>(request.mission_goal.z),
        .yaw = request.navigation.state.yaw,
    };
    output.controller.target_source =
        request.objective != nullptr && request.objective->tracking.has_value() &&
                request.objective->tracking->predicted_intercept_path_clear
            ? "tracking_direct_full_prediction"
            : "tracking_direct_shortened_prediction";
  } else if (route_execution_blocked) {
    const Point3& hold = output.route.execution.hold_position;
    target = mppi::State{
        .x = static_cast<float>(hold.x),
        .y = static_cast<float>(hold.y),
        .z = static_cast<float>(hold.z),
        .yaw = request.navigation.state.yaw,
    };
    output.controller.target_source = request.objective->continuous_tracking
                                          ? "tracking_no_executable_route_"
                                          : "no_executable_route_";
    output.controller.target_source +=
        routeExecutionStatus3DName(output.route.execution_status);
  } else {
    const std::span<const mppi::RouteSample3D> controller_route =
        output.route.controller_route != nullptr
            ? std::span<const mppi::RouteSample3D>{*output.route.controller_route}
            : std::span<const mppi::RouteSample3D>{};
    target = selectTarget(route, controller_route, request.mission_goal,
                          output.route.execution.station_m,
                          output.controller.speed_policy.target_lookahead_m,
                          output.controller.target_source, target_station_m);
  }
  if (route_control.active) {
    target.z = static_cast<float>(route_control.reference_z_m);
    if (route_control.hold_xy) {
      output.controller.target_source = "passage_vertical_alignment_hold";
      target.x = request.navigation.state.x;
      target.y = request.navigation.state.y;
      output.controller.speed_policy.reference_speed_mps = 0.0;
      output.controller.speed_policy.target_lookahead_m = 0.0;
    } else if (route_control.vertical_ready) {
      output.controller.target_source = "passage_traversal";
    } else {
      output.controller.target_source = "passage_vertical_alignment";
    }
  }
  if (output.controller.cooperative.yield.active && output.route.usable &&
      output.route.projection.valid && execution_route != nullptr &&
      !execution_route->empty() && !route_control.hold_xy) {
    const RouteSample3D hold_sample = sampleRoute3DAtStation(
        *execution_route, output.controller.cooperative.yield.hold_station_m);
    target.x = static_cast<float>(hold_sample.position.x);
    target.y = static_cast<float>(hold_sample.position.y);
    target.z = static_cast<float>(hold_sample.position.z);
    target_station_m = hold_sample.station_m;
    output.controller.target_source = output.controller.cooperative.yield.hold_at_entry
                                          ? "cooperative_passage_yield_hold"
                                          : "cooperative_passage_yield_deceleration";
  }

  output.controller.planning_state = ProductionMppiPlanningState::kPlanned;
  if (request.objective != nullptr && request.objective->immediate_hold) {
    output.controller.planning_state =
        ProductionMppiPlanningState::kMissionCommandPositionHold;
    target = mppi::State{
        .x = static_cast<float>(request.mission_goal.x),
        .y = static_cast<float>(request.mission_goal.y),
        .z = static_cast<float>(request.mission_goal.z),
        .yaw = request.navigation.state.yaw,
    };
    output.controller.speed_policy.reference_speed_mps = 0.0;
    output.controller.speed_policy.target_lookahead_m = 0.0;
    output.controller.target_source = "mission_command_position_hold";
  } else if (output.controller.goal_capture.latched) {
    output.controller.planning_state =
        ProductionMppiPlanningState::kMissionGoalPositionHold;
    target = mppi::State{
        .x = static_cast<float>(request.mission_goal.x),
        .y = static_cast<float>(request.mission_goal.y),
        .z = static_cast<float>(request.mission_goal.z),
        .yaw = request.navigation.state.yaw,
    };
    output.controller.speed_policy.reference_speed_mps = 0.0;
    output.controller.speed_policy.target_lookahead_m = 0.0;
    output.controller.target_source = "mission_goal_position_hold";
  } else if (route_execution_blocked) {
    output.controller.planning_state =
        ProductionMppiPlanningState::kNoExecutableRouteHold;
    output.controller.speed_policy.reference_speed_mps = 0.0;
    output.controller.speed_policy.target_lookahead_m = 0.0;
  } else if (output.controller.cooperative.yield.active &&
             output.controller.cooperative.yield.hold_at_entry &&
             !output.controller.cooperative.mppi.avoidance_active &&
             !route_control.hold_xy) {
    output.controller.planning_state =
        ProductionMppiPlanningState::kCooperativePassageYieldHold;
    output.controller.speed_policy.reference_speed_mps = 0.0;
    output.controller.speed_policy.target_lookahead_m = 0.0;
    output.controller.target_source = "cooperative_passage_yield_hold";
  }

  if (!request.direct_tracking_interception) {
    output.controller.liveness = liveness_supervisor_.evaluate(MppiLivenessObservation{
        .stamp_ns = request.now_ns,
        .actual_state = request.navigation.state,
        .controller_active =
            request.control_feedback_fresh &&
            output.controller.planning_state == ProductionMppiPlanningState::kPlanned &&
            !route_control.hold_xy && output.route.projection.valid,
        .predicted_head_progress_m = request.previous_result != nullptr
                                         ? request.previous_result->head_progress_m
                                         : 0.0,
        .predicted_terminal_progress_m =
            request.previous_result != nullptr
                ? request.previous_result->terminal_progress_m
                : 0.0,
        .route_generation = output.route.generation,
        .route_station_m = output.route.projection.station_m,
        .route_station_valid = output.route.projection.valid,
    });
  }
  if (route_progress_tracker_ != nullptr && !request.direct_tracking_interception) {
    output.controller.route_progress =
        route_progress_tracker_->evaluate(RouteProgressObservation3D{
            .stamp_ns = request.now_ns,
            .route_generation = output.route.projection.valid &&
                                        output.controller.planning_state ==
                                            ProductionMppiPlanningState::kPlanned
                                    ? output.route.generation
                                    : 0U,
            .station_m = output.route.projection.station_m,
            .predicted_head_progress_m = request.previous_result != nullptr
                                             ? request.previous_result->head_progress_m
                                             : 0.0,
            .cross_track_m = output.route.projection.cross_track_m,
            .recovery_active = output.controller.liveness.recovery_active,
            .controller_active = request.control_feedback_fresh &&
                                 output.controller.planning_state ==
                                     ProductionMppiPlanningState::kPlanned &&
                                 !route_control.hold_xy,
        });
    output.effects.request_stalled_route_release =
        output.controller.route_progress.stalled &&
        config_.route_progress_replan_enabled;
  }

  output.controller.route_required_risk_tier = mppi::RiskTier::kPreferred;
  if (output.route.usable && output.route.controller_route != nullptr &&
      output.route.projection.valid) {
    const double horizon_distance_m =
        std::max(target_station_m - output.route.projection.station_m,
                 output.controller.speed_policy.reference_speed_mps *
                     static_cast<double>(config_.horizon_steps) *
                     static_cast<double>(config_.dynamics.dt_s));
    output.controller.route_required_risk_tier = mppi::maximumRequiredRiskTier(
        *output.route.controller_route,
        static_cast<float>(output.route.projection.station_m),
        static_cast<float>(output.route.projection.station_m +
                           std::max(0.0, horizon_distance_m)));
  }

  std::optional<mppi::MovingTargetReference> moving_target;
  const ProductionTrackingObjective* tracking =
      request.objective != nullptr && request.objective->tracking.has_value()
          ? std::addressof(request.objective->tracking.value())
          : nullptr;
  if (request.objective != nullptr && request.objective->continuous_tracking &&
      tracking != nullptr) {
    const double observation_age_s =
        static_cast<double>(std::max<std::int64_t>(
            0, request.now_ns - tracking->observation_stamp_ns)) *
        1.0e-9;
    const TargetVerticalPrediction vertical_prediction = predictTargetVerticalMotion(
        tracking->observed_position.z, tracking->observed_velocity.z, observation_age_s,
        config_.dynamics.maximum_vertical_acceleration_mps2, config_.flight_envelope);
    const float minimum_z =
        static_cast<float>(config_.flight_envelope.minimum_target_z_m);
    const float maximum_z = std::nextafter(
        static_cast<float>(config_.flight_envelope.maximum_target_z_m), minimum_z);
    if (vertical_prediction.valid && std::isfinite(minimum_z) &&
        std::isfinite(maximum_z) && maximum_z > minimum_z) {
      moving_target = mppi::MovingTargetReference{
          .state =
              mppi::State{
                  .x = static_cast<float>(tracking->observed_position.x +
                                          tracking->observed_velocity.x *
                                              observation_age_s),
                  .y = static_cast<float>(tracking->observed_position.y +
                                          tracking->observed_velocity.y *
                                              observation_age_s),
                  .z = mppi::clampMovingTargetAltitude(
                      static_cast<float>(vertical_prediction.z_m), minimum_z,
                      maximum_z),
                  .vx = static_cast<float>(tracking->observed_velocity.x),
                  .vy = static_cast<float>(tracking->observed_velocity.y),
                  .vz = static_cast<float>(vertical_prediction.velocity_mps),
              },
          .capture_radius_m = static_cast<float>(config_.tracking_capture_radius_m),
          .vertical_deceleration_mps2 =
              config_.dynamics.maximum_vertical_acceleration_mps2,
          .minimum_z_m = minimum_z,
          .maximum_z_m = maximum_z,
          .bounded_vertical_motion = true,
      };
    }
  }
  if (request.direct_tracking_interception && moving_target.has_value()) {
    output.controller.direct_tracking_maneuver =
        direct_tracking_maneuver_lifecycle_.update(DirectTrackingManeuverObservation{
            .interceptor_position = actual_position,
            .interceptor_velocity =
                Vec3{request.navigation.state.vx, request.navigation.state.vy,
                     request.navigation.state.vz},
            .target_position = Point3{moving_target->state.x, moving_target->state.y,
                                      moving_target->state.z},
            .target_velocity = Vec3{moving_target->state.vx, moving_target->state.vy,
                                    moving_target->state.vz},
            .stamp_ns = request.now_ns,
            .line_of_sight_generation = request.line_of_sight_generation,
            .active = output.controller.planning_state ==
                      ProductionMppiPlanningState::kPlanned,
        });
  } else {
    output.controller.direct_tracking_maneuver =
        direct_tracking_maneuver_lifecycle_.update({});
  }

  const EsdfQueryResult current_clearance = queryConservativeEsdf3D(
      request.world->grid, *request.world->distances_m, request.navigation.state.x,
      request.navigation.state.y, request.navigation.state.z);
  const double tracking_age_ms =
      tracking != nullptr && tracking->observation_stamp_ns > 0
          ? static_cast<double>(std::max<std::int64_t>(
                0, request.now_ns - tracking->observation_stamp_ns)) /
                1.0e6
          : std::numeric_limits<double>::infinity();
  output.controller.rollout_budget = selectMppiRolloutBudget(
      config_.rollout_budget,
      MppiRolloutBudgetObservation{
          .static_world = request.use_static_map,
          .route_available = request.direct_tracking_interception ||
                             (output.route.usable && output.route.projection.valid),
          .direct_tracking = request.direct_tracking_interception,
          .clearance_valid = current_clearance.status == EsdfQueryStatus::kValid,
          .clearance_m = current_clearance.clearance_m,
          .world_age_ms = request.observation_age_ms,
          .tracking_age_ms = tracking_age_ms,
          .required_risk_tier = request.direct_tracking_interception
                                    ? mppi::RiskTier::kPreferred
                                    : output.controller.route_required_risk_tier,
      });
  const mppi::DeterministicCandidateKind deterministic_candidate =
      planningDeterministicCandidate(
          request.direct_tracking_interception, output.controller.planning_state,
          output.route.usable, output.route.projection.valid, route_control.hold_xy);
  output.controller.request =
      MppiControllerRequest3D{
          .input =
              mppi::MppiTickInput{
                  .initial_state = request.execution_input->state(),
                  .target = target,
                  .pose_revision = request.execution_input->poseRevision(),
                  .obstacle_revision = planningRawRevision(request.use_static_map,
                                                           request.world->revision,
                                                           request.latest_raw_world),
                  .expected_esdf_revision =
                      request.world->local_world_generation.gpu_esdf_revision,
                  .planning_stamp_ns = request.now_ns,
                  .previous_applied_control =
                      request.execution_input->previousControl(),
                  .reference_speed_mps =
                      output.controller.speed_policy.enabled
                          ? static_cast<float>(
                                output.controller.speed_policy.reference_speed_mps)
                          : -1.0F,
                  .moving_target = moving_target,
                  .route =
                      output.controller.planning_state ==
                                  ProductionMppiPlanningState::kPlanned &&
                              output.route.usable &&
                              output.route.controller_route != nullptr &&
                              output.route.projection.valid && !route_control.hold_xy
                          ? std::optional<mppi::RouteReference>{mppi::RouteReference{
                                .points = output.route.controller_route,
                                .generation = output.route.generation,
                                .initial_station_m = static_cast<float>(
                                    output.route.projection.station_m),
                                .terminal_cross_track_tolerance_m =
                                    routeCrossTrackTolerance3D(
                                        config_.route_cross_track_constraints_enabled),
                            }}
                          : std::nullopt,
                  .dynamic_aircraft =
                      noncooperative_cost_influence_active
                          ? output.controller.noncooperative.avoidance.trajectories
                          : output.controller.cooperative
                                .mppi.dynamic_aircraft,
                  .dynamic_aircraft_cost_policy =
                      noncooperative_cost_influence_active
                          ? std::optional<mppi::
                                              DynamicAircraftCostPolicy>{output
                                                                             .controller
                                                                             .noncooperative
                                                                             .avoidance
                                                                             .cost_policy}
                          : std::nullopt,
                  .cooperative_maneuver = output.controller.cooperative.mppi.maneuver,
                  .cooperative_acquisition =
                      output.controller.cooperative.mppi.acquisition,
                  .noncooperative_acquisition =
                      noncooperative_evasive_maneuver_active
                          ? output.controller.noncooperative.avoidance.acquisition
                          : std::nullopt,
                  .active_rollouts = output.controller.rollout_budget.active_rollouts,
                  .deterministic_candidate = deterministic_candidate,
                  .prefer_route_directed_candidate =
                      !config_.stochastic_trajectory_selection_enabled,
                  .force_route_directed_candidate =
                      output.controller.liveness
                          .recovery_active ||
                      output.controller.route_progress.local_reseed_requested,
                  .cooperative_avoidance_active =
                      output.controller.cooperative.mppi.avoidance_active,
                  .noncooperative_avoidance_active =
                      noncooperative_evasive_maneuver_active,
              },
          .nominal_reseed =
              MppiNominalReseedObservation{
                  .route_generation = request.direct_tracking_interception
                                          ? request.effective_route_generation
                                          : output.route.generation,
                  .local_liveness_generation =
                      output.controller.liveness.reseed_generation,
                  .route_liveness_generation =
                      output.controller.route_progress.local_reseed_generation,
                  .direct_tracking_maneuver_generation =
                      output.controller.direct_tracking_maneuver.reseed_generation,
              },
          .tick_started = request.tick_started,
          .world_revision = request.world_revision,
          .mode = output.controller.planning_state ==
                              ProductionMppiPlanningState::kMissionGoalPositionHold ||
                          output.controller.planning_state ==
                              ProductionMppiPlanningState::kNoExecutableRouteHold
                      ? MppiControllerMode3D::kStationaryHold
                      : MppiControllerMode3D::kPlan,
      };
  output.status = PlanningCycleStatus3D::kReady;
  return output;
}

} // namespace drone_city_nav
