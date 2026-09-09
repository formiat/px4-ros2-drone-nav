#include "drone_city_nav/cooperative_traffic_ros.hpp"
#include "drone_city_nav/execution_horizon_commit_3d.hpp"
#include "drone_city_nav/navigation_state_prediction.hpp"

#include <chrono>
#include <cinttypes>
#include <limits>
#include <memory>
#include <utility>

#include "mppi_controller_3d.hpp"
#include "planning_cycle_coordinator_3d.hpp"
#include "production_mppi_node.hpp"
#include "production_mppi_node_planning_tick_context.hpp"
#include "production_mppi_node_planning_tick_finalize.hpp"
#include "production_mppi_node_planning_tick_rearm.hpp"
#include "raw_world_ingress_ros_3d.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] const char* residentObstacleSource(
    const ProductionMppiResidentObstacleDisposition disposition) noexcept {
  switch (disposition) {
    case ProductionMppiResidentObstacleDisposition::kRouteSuffixReplacementRequired:
      return "resident_route_suffix_persistent_raw";
    case ProductionMppiResidentObstacleDisposition::
        kPersistentRawFiniteExecutionInvalidated:
      return "active_finite_trajectory_persistent_raw";
    case ProductionMppiResidentObstacleDisposition::
        kLatestLidarFiniteExecutionInvalidated:
      return "active_finite_trajectory_latest_lidar";
    case ProductionMppiResidentObstacleDisposition::kClear:
      return "none";
  }
  return "none";
}

} // namespace

void ProductionMppiNode::planningTick() {
  if (!mppi_controller_) {
    return;
  }
  const std::int64_t tick_entry_ns = get_clock()->now().nanoseconds();
  if (last_planning_tick_entry_ns_ > 0 &&
      tick_entry_ns > last_planning_tick_entry_ns_) {
    last_planning_tick_period_ns_ = tick_entry_ns - last_planning_tick_entry_ns_;
  }
  last_planning_tick_entry_ns_ = tick_entry_ns;
  const std::shared_ptr<const ProductionNavigationObjectiveState> objective_state =
      navigationObjectiveState();
  const std::shared_ptr<const ProductionNavigationObjective> objective =
      objective_state != nullptr ? objective_state->objective : nullptr;
  const bool tracking_objective_available =
      objective != nullptr && objective->tracking.has_value();
  const ProductionTrackingObjective tracking_objective =
      objective != nullptr ? objective->tracking.value_or(ProductionTrackingObjective{})
                           : ProductionTrackingObjective{};
  const Point3 mission_goal = objective ? objective->goal : mission_goal_;
  const bool terminal_hold_enabled = !objective || !objective->continuous_tracking;
  const bool direct_tracking_interception =
      objective && objective->continuous_tracking && tracking_objective_available &&
      tracking_objective.direct_interception_active;
  const bool observed_3d_world = !config_.world.use_static_map;
  if (handleRequestedExecutionRevocation(tick_entry_ns)) {
    // A callback-requested epoch is a hard barrier; retain it until revoke
    // publication has linearized with the exact snapshot.
    return;
  }
  const std::uint64_t line_of_sight_generation =
      tracking_objective_available ? tracking_objective.line_of_sight_generation : 0U;
  const std::optional<DirectTrackingOwnerIdentity3D> direct_tracking_identity =
      makeDirectTrackingOwnerIdentity(objective.get(), direct_tracking_interception,
                                      line_of_sight_generation);
  const std::uint64_t effective_route_generation = directTrackingRouteGeneration(
      direct_tracking_interception, line_of_sight_generation);
  const std::uint64_t required_route_sample =
      objective_state != nullptr ? objective_state->requiredTrackingSampleSequence()
                                 : 0U;
  const auto snapshot_started = std::chrono::steady_clock::now();
  ProductionMppiNavigation navigation;
  ProductionMppiVehicleStatus vehicle_status;
  ProductionMppiPredictionError prediction;
  std::shared_ptr<const CommittedExecutionAuthority3D> execution_authority;
  AppliedControlEvidence3D applied_control;
  ExecutionOwnerIdentity3D execution_horizon_owner;
  std::uint64_t applied_control_discontinuity_generation{0U};
  bool applied_control_discontinuity_generation_valid{false};
  OffboardSessionAdmissionState offboard_session;
  std::int64_t offboard_session_receive_stamp_ns{0};
  ProductionMppiHorizonAcknowledgement horizon_acknowledgement;
  ProductionMppiHorizonPublicationRecord horizon_publication;
  bool vehicle_status_epoch_stable{false};
  std::optional<ProductionMppiCooperativeCommand> cooperative_command;
  ProductionMppiNonCooperativeTracks noncooperative_tracks;
  LatestObservation latest_observation;
  std::uint64_t memory_sequence{0U};
  bool raw_world_identity_conflicted{false};
  RawWorldIngressSnapshot3D world_input;
  {
    const auto lock = evidence_boundary_.input();
    navigation = navigation_;
    navigation.valid = navigation.valid && !navigation_revision_exhausted_ &&
                       !navigation_frame_reset_unresolved_;
    vehicle_status = vehicle_status_;
    prediction = latest_prediction_error_;
    execution_authority = execution_supervisor_.authority();
    if (execution_authority != nullptr && execution_authority->valid()) {
      applied_control = execution_authority->control();
      execution_horizon_owner = execution_authority->owner();
    }
    applied_control_discontinuity_generation =
        applied_control_discontinuity_generation_;
    applied_control_discontinuity_generation_valid =
        !applied_control_discontinuity_generation_exhausted_;
    offboard_session = offboard_session_admission_;
    offboard_session_receive_stamp_ns = offboard_session_receive_stamp_ns_;
    horizon_acknowledgement = latest_horizon_acknowledgement_;
    horizon_publication = latest_horizon_publication_;
    vehicle_status_epoch_stable =
        !vehicle_status_epoch_probation_ && !vehicle_status_revision_exhausted_;
    cooperative_command = cooperative_command_;
    noncooperative_tracks = noncooperative_tracks_;
    world_input = raw_world_ingress_->snapshot();
    latest_observation = world_input.latest_observation;
    memory_sequence = latest_observation.sequence;
    raw_world_identity_conflicted = world_input.raw_world_identity_conflicted;
  }
  const WorldPipelineResidentSnapshot3D resident_world =
      world_pipeline_->residentSnapshot();
  const double capture_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - snapshot_started)
                                .count();
  const std::shared_ptr<const WorldSnapshot3D>& world = resident_world.world;
  const ProductionWorldBuildTelemetry3D& world_build = resident_world.telemetry;
  const std::shared_ptr<const ProductionRouteActivationResult3D> route_pipeline =
      latest_route_pipeline_event_.load(std::memory_order_acquire);
  const std::shared_ptr<const ProductionMppiRawWorld3D> latest_raw_world_3d =
      world_input.latest_raw_world;
  const bool latest_lidar_evidence_identity_conflicted =
      latest_lidar_evidence_identity_conflicted_.load(std::memory_order_acquire);
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence =
      latest_lidar_evidence_identity_conflicted
          ? nullptr
          : latest_lidar_evidence_.load(std::memory_order_acquire);
  const std::shared_ptr<const ExecutionPlan3D> execution_snapshot =
      execution_authority != nullptr ? execution_authority->plan() : nullptr;
  // Timestamp the immutable planning view only after all callback-owned inputs
  // have been captured. A concurrently published evidence value may have a
  // receive stamp later than tick entry, but never later than this boundary.
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  // The capture gate is opened with the exact stamp the mission observation
  // carries; a second clock read between the two would break its continuity
  // on every tick.
  if (mission_waypoint_capture_gate_) {
    mission_waypoint_capture_gate_->beginTick(now_ns);
  }
  const double pose_age_ms =
      static_cast<double>(now_ns - navigation.receive_stamp_ns) / 1.0e6;
  double esdf_age_ms = std::numeric_limits<double>::infinity();
  if (world) {
    esdf_age_ms = config_.world.use_static_map
                      ? 0.0
                      : static_cast<double>(now_ns - world->ready_stamp_ns) / 1.0e6;
  }
  double observation_age_ms = std::numeric_limits<double>::infinity();
  if (config_.world.use_static_map) {
    observation_age_ms = 0.0;
  } else if (world && !raw_world_identity_conflicted) {
    if (latest_raw_world_3d != nullptr &&
        latest_raw_world_3d->version().producer_instance_id ==
            world->producer_instance_id) {
      observation_age_ms = committedRawWorldAgeMs(latest_raw_world_3d.get(), now_ns);
    }
  }
  const double control_feedback_age_ms =
      applied_control.valid
          ? static_cast<double>(now_ns - applied_control.receive_stamp_ns) / 1.0e6
          : std::numeric_limits<double>::infinity();
  const bool world_current =
      world_ready_.load(std::memory_order_acquire) && world &&
      observation_age_ms >= 0.0 &&
      observation_age_ms <= config_.world.maximum_esdf_age_ms +
                                config_.execution.stale_esdf_execution_window_ms;
  const NavigationHealthAssessment navigation_health =
      updateNavigationHealth(objective, execution_authority, world_current, now_ns);
  if (navigation_health.terminal &&
      config_.planning.optional_constraints.nonphysical_execution_revocation_enabled) {
    publishFailClosedExecutionRevocation(
        terminalExecutionReason(navigation_health.failure), now_ns);
    return;
  }
  if (!vehicleStatusAuthoritativeForExecution(
          vehicle_status, vehicle_status_epoch_stable, now_ns,
          config_.execution.maximum_vehicle_status_age_ms)) {
    if (execution_horizon_owner.valid) {
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
      static_cast<void>(handleRequestedExecutionRevocation(now_ns));
    }
    return;
  }
  const bool goal_capture_latched =
      planning_cycle_coordinator_ != nullptr && objective && terminal_hold_enabled &&
      planning_cycle_coordinator_->goalCaptureLatchedFor(mission_goal);
  const MissionWaypointUpdate early_waypoint_update = updateMissionWaypoint(
      objective_state, navigation, vehicle_status, execution_authority,
      applied_control_discontinuity_generation,
      applied_control_discontinuity_generation_valid, vehicle_status_epoch_stable,
      goal_capture_latched, now_ns);
  if (early_waypoint_update.waypoint_completed) {
    // The planner published the aggregate acknowledgement before replacing the
    // objective. Replan the newly active leg on the next tick.
    return;
  }
  const bool matching_goal_capture_attempt =
      goal_capture_latched && execution_horizon_owner.valid &&
      execution_horizon_owner.execution_mode ==
          ExecutionAuthorityMode3D::kPositionHold &&
      execution_horizon_owner.execution_reason ==
          ExecutionAuthorityReason3D::kGoalCapture &&
      execution_horizon_owner.stationary_position_hold &&
      now_ns >= execution_horizon_owner.valid_from_ns &&
      now_ns < execution_horizon_owner.valid_until_ns &&
      distance3D(execution_horizon_owner.route_target, mission_goal) <=
          config_.execution.mission_waypoint_capture_gate.target_match_tolerance_m &&
      distance3D(execution_horizon_owner.stationary_hold_position, mission_goal) <=
          config_.execution.mission_waypoint_capture_gate.goal_radius_m;
  if (matching_goal_capture_attempt) {
    // Freeze one committed lease/identity for the full acknowledgement attempt.
    // The offboard process repeats feedback for this exact tuple; superseding it
    // every planning tick would make continuous exact witnessing impossible.
    return;
  }
  if (!navigation.valid || pose_age_ms < 0.0) {
    publishFailClosedExecutionRevocation(
        ProductionMppiExecutionReason::kNoExecutableHorizon, now_ns);
    return;
  }
  bool pose_predicted = false;
  if (pose_age_ms > config_.execution.maximum_pose_age_ms) {
    const NavigationStatePredictionResult predicted = predictNavigationState(
        navigation.state, pose_age_ms / 1000.0,
        config_.execution.maximum_pose_prediction_age_ms / 1000.0);
    if (!predicted.valid) {
      publishFailClosedExecutionRevocation(
          ProductionMppiExecutionReason::kNoExecutableHorizon, now_ns);
      return;
    }
    navigation.state = predicted.state;
    pose_predicted = predicted.predicted;
  }
  if (!world || observation_age_ms < 0.0 ||
      observation_age_ms > config_.world.maximum_esdf_age_ms +
                               config_.execution.stale_esdf_execution_window_ms) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "PRODUCTION_MPPI_UNAVAILABLE_WORLD action=wait_for_world "
                         "observation_age_ms=%.1f esdf_content_age_ms=%.1f "
                         "maximum_execution_age_ms=%.1f",
                         observation_age_ms, esdf_age_ms,
                         config_.world.maximum_esdf_age_ms +
                             config_.execution.stale_esdf_execution_window_ms);
    publishFailClosedExecutionRevocation(
        ProductionMppiExecutionReason::kUnavailableWorld, now_ns);
    return;
  }
  if (!worldGenerationAvailableForPlanning(*world, now_ns)) {
    return;
  }
  if (!mppi_controller_->ready()) {
    publishFailClosedExecutionRevocation(
        ProductionMppiExecutionReason::kNoExecutableHorizon, now_ns);
    return;
  }
  const char* const owner_witness_failure = appliedControlAuthorityFailure3D(
      applied_control, execution_horizon_owner, now_ns,
      config_.execution.maximum_control_feedback_age_ms);
  const ProductionMppiHorizonSupersessionDecision supersession =
      assessPlannedHorizonSupersession(ProductionMppiHorizonSupersessionCheck{
          .owner = &execution_horizon_owner,
          .acknowledgement = &horizon_acknowledgement,
          .owner_witnessed = owner_witness_failure == nullptr,
          .now_ns = now_ns,
          .owner_publication_stamp_ns =
              horizon_publication.sequence == execution_horizon_owner.sequence
                  ? horizon_publication.publication_stamp_ns
                  : 0,
          .acknowledgement_grace_ns =
              config_.execution.horizon_acknowledgement_grace_ns,
          .maximum_acknowledgement_age_ns = static_cast<std::int64_t>(
              config_.execution.maximum_control_feedback_age_ms * 1.0e6),
      });
  switch (supersession) {
    case ProductionMppiHorizonSupersessionDecision::kDeferredAwaitingAcknowledgement:
      horizon_supersession_deferrals_.fetch_add(1U, std::memory_order_relaxed);
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "EXECUTION_HORIZON_SUPERSESSION deferred=true "
          "reason=awaiting_acknowledgement witness_failure=%s "
          "producer=%" PRIu64 " sequence=%" PRIu64 " acknowledged_sequence=%" PRIu64
          " acknowledgement_age_ms=%.1f owner_age_ms=%.1f",
          owner_witness_failure != nullptr ? owner_witness_failure : "none",
          execution_horizon_owner.producer_instance_id,
          execution_horizon_owner.sequence,
          horizon_acknowledgement.valid ? horizon_acknowledgement.horizon_sequence : 0U,
          horizon_acknowledgement.valid
              ? static_cast<double>(now_ns - horizon_acknowledgement.receive_stamp_ns) *
                    1.0e-6
              : -1.0,
          static_cast<double>(now_ns - execution_horizon_owner.valid_from_ns) * 1.0e-6);
      return;
    case ProductionMppiHorizonSupersessionDecision::kAllowedAcknowledgementGraceElapsed:
      horizon_supersession_grace_replacements_.fetch_add(1U, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "EXECUTION_HORIZON_SUPERSESSION allowed=true "
          "reason=acknowledgement_grace_elapsed witness_failure=%s "
          "producer=%" PRIu64 " sequence=%" PRIu64 " acknowledged_sequence=%" PRIu64,
          owner_witness_failure != nullptr ? owner_witness_failure : "none",
          execution_horizon_owner.producer_instance_id,
          execution_horizon_owner.sequence,
          horizon_acknowledgement.valid ? horizon_acknowledgement.horizon_sequence
                                        : 0U);
      break;
    case ProductionMppiHorizonSupersessionDecision::kRejectedOwnerNotCurrent:
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "EXECUTION_HORIZON_SUPERSESSION rejected=true reason=owner_not_current "
          "action=%s "
          "producer=%" PRIu64 " sequence=%" PRIu64,
          config_.planning.optional_constraints.nonphysical_execution_revocation_enabled
              ? "revoke"
              : "replace_expired_owner",
          execution_horizon_owner.producer_instance_id,
          execution_horizon_owner.sequence);
      if (config_.planning.optional_constraints
              .nonphysical_execution_revocation_enabled) {
        publishFailClosedExecutionRevocation(
            ProductionMppiExecutionReason::kNoExecutableHorizon, now_ns);
        return;
      }
      break;
    case ProductionMppiHorizonSupersessionDecision::kAllowedNoPlannedOwner:
    case ProductionMppiHorizonSupersessionDecision::kAllowedWitnessedOwner:
    case ProductionMppiHorizonSupersessionDecision::kAllowedAcknowledgedPredecessor:
      break;
  }
  if (execution_input_capture_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                          "PRODUCTION_MPPI_UNAVAILABLE_CONTROL "
                          "action=execution_input_sequence_exhausted");
    publishFailClosedExecutionRevocation(
        ProductionMppiExecutionReason::kNoExecutableHorizon, now_ns);
    return;
  }
  const std::uint64_t execution_input_sequence = ++execution_input_capture_sequence_;
  const char* const stationary_capture_rearm_failure =
      stationaryCaptureRearmIneligibilityForPlanningTick(
          ProductionMppiStationaryCaptureRearmContext{
              .objective = objective.get(),
              .mission_waypoint_sequence = mission_waypoint_sequence_.get(),
              .navigation = &navigation,
              .vehicle_status = &vehicle_status,
              .execution_authority = execution_authority,
              .offboard_session = &offboard_session,
              .world = world.get(),
              .latest_raw_world_3d = latest_raw_world_3d,
              .latest_lidar_evidence = latest_lidar_evidence,
              .validation_policy = config_.execution.validation_policy,
              .static_occupancy_3d = world->static_occupancy,
              .capture_gate_config = config_.execution.mission_waypoint_capture_gate,
              .mission_goal = mission_goal,
              .now_ns = now_ns,
              .offboard_session_receive_stamp_ns = offboard_session_receive_stamp_ns,
              .maximum_pose_age_ms = config_.execution.maximum_pose_age_ms,
              .maximum_control_feedback_age_ms =
                  config_.execution.maximum_control_feedback_age_ms,
              .maximum_observation_age_ms =
                  config_.world.maximum_esdf_age_ms +
                  config_.execution.stale_esdf_execution_window_ms,
              .observation_age_ms = observation_age_ms,
              .vehicle_status_epoch_stable = vehicle_status_epoch_stable,
              .terminal_hold_enabled = terminal_hold_enabled,
              .goal_capture_latched = goal_capture_latched,
              .use_static_map = config_.world.use_static_map,
              .observed_3d_world = observed_3d_world,
          });
  const bool stationary_capture_rearm = stationary_capture_rearm_failure == nullptr;
  if (goal_capture_latched && !stationary_capture_rearm) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "STATIONARY_CAPTURE_REARM eligible=false reason=%s",
                         stationary_capture_rearm_failure);
  }
  const auto execution_input_started = std::chrono::steady_clock::now();
  const ProductionMppiExecutionInputPreparation execution_input_preparation =
      prepareExecutionInputForPlanningTick(
          navigation, execution_authority, execution_input_sequence, now_ns,
          config_.execution.maximum_control_feedback_age_ms, pose_predicted,
          stationary_capture_rearm);
  const double execution_input_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                execution_input_started)
          .count();
  if (!execution_input_preparation.previous_control_available ||
      tick_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "PRODUCTION_MPPI_UNAVAILABLE_CONTROL action=wait_for_authoritative_control "
        "feedback_fresh=%s measured_control=%s published_control=%s",
        execution_input_preparation.control_feedback_fresh ? "true" : "false",
        execution_input_preparation.measured_control_available ? "true" : "false",
        execution_input_preparation.previous_control_available ? "true" : "false");
    publishFailClosedExecutionRevocation(
        ProductionMppiExecutionReason::kNoExecutableHorizon, now_ns);
    return;
  }
  const std::shared_ptr<const VersionedExecutionInput3D> execution_input =
      execution_input_preparation.execution_input;
  if (execution_input == nullptr) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "PRODUCTION_MPPI_UNAVAILABLE_CONTROL action=reject_invalid_execution_input "
        "pose_revision=%" PRIu64 " control_source=%s",
        navigation.revision,
        productionMppiPreviousControlSourceName(
            execution_input_preparation.previous_control_source));
    publishFailClosedExecutionRevocation(
        ProductionMppiExecutionReason::kNoExecutableHorizon, now_ns);
    return;
  }
  if (planning_cycle_coordinator_ == nullptr) {
    publishFailClosedExecutionRevocation(
        ProductionMppiExecutionReason::kNoExecutableHorizon, now_ns);
    return;
  }
  const auto cycle_prepare_started = std::chrono::steady_clock::now();
  PlanningCycleOutcome3D planning =
      planning_cycle_coordinator_->prepare(PlanningCycleRequest3D{
          .world = world.get(),
          .objective = objective.get(),
          .previous_result = previous_result_.has_value()
                                 ? std::addressof(previous_result_.value())
                                 : nullptr,
          .navigation = navigation,
          .execution_input = execution_input,
          .latest_raw_world = latest_raw_world_3d,
          .latest_lidar_evidence = latest_lidar_evidence,
          .cooperative_command = cooperative_command,
          .noncooperative_tracks = noncooperative_tracks,
          .direct_tracking_identity = direct_tracking_identity,
          .mission_goal = mission_goal,
          .tick_started = snapshot_started,
          .minimum_tracking_sample_sequence = required_route_sample,
          .physically_invalidated_through_generation =
              physical_trajectory_replan_route_generation_.load(
                  std::memory_order_acquire),
          .effective_route_generation = effective_route_generation,
          .line_of_sight_generation = line_of_sight_generation,
          .world_revision = world->revision,
          .now_ns = now_ns,
          .observation_age_ms = observation_age_ms,
          .control_feedback_fresh = execution_input_preparation.control_feedback_fresh,
          .terminal_hold_enabled = terminal_hold_enabled,
          .direct_tracking_interception = direct_tracking_interception,
          .use_static_map = config_.world.use_static_map,
          .observed_3d_world = observed_3d_world,
      });
  const double cycle_prepare_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                cycle_prepare_started)
          .count();
  {
    // The block the replan snapshot stitches short of: the nearer of the
    // persistent raw and the latest lidar blocks on the resident route.
    std::optional<double> blocked_station_m =
        planning.route.execution.raw_blocked_station_m;
    if (planning.route.execution.latest_lidar_blocked_station_m.has_value() &&
        (!blocked_station_m.has_value() ||
         *planning.route.execution.latest_lidar_blocked_station_m <
             *blocked_station_m)) {
      blocked_station_m = planning.route.execution.latest_lidar_blocked_station_m;
    }
    observed_route_blocked_station_m_.store(
        blocked_station_m.value_or(std::numeric_limits<double>::quiet_NaN()),
        std::memory_order_release);
  }
  for (const RouteExecutionSelectorEffect3D& effect :
       planning.effects.route_execution) {
    switch (effect.kind) {
      case RouteExecutionSelectorEffectKind3D::kRequestRouteRelease:
        requestRouteRelease(effect.release_reason, effect.route_generation);
        break;
      case RouteExecutionSelectorEffectKind3D::kHandlePhysicalTrajectoryCollision:
        handlePhysicalTrajectoryCollision(
            effect.route_generation, effect.observed_raw_world,
            residentObstacleSource(effect.obstacle_disposition),
            ProductionMppiPhysicalTrajectoryAuthority::kResidentOwner);
        break;
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ROUTE_EXECUTION3D route_generation=%" PRIu64
        " status=%.*s effect=%s obstacle_source=%s pending=%.*s",
        effect.route_generation,
        static_cast<int>(
            routeExecutionStatus3DName(planning.route.execution.status).size()),
        routeExecutionStatus3DName(planning.route.execution.status).data(),
        effect.kind == RouteExecutionSelectorEffectKind3D::kRequestRouteRelease
            ? "request_route_release"
            : "handle_physical_trajectory_collision",
        residentObstacleSource(effect.obstacle_disposition),
        static_cast<int>(
            pendingRouteEligibility3DName(planning.route.execution.pending_eligibility)
                .size()),
        pendingRouteEligibility3DName(planning.route.execution.pending_eligibility)
            .data());
  }
  reportUnadoptedPendingRoute(planning.route.execution);
  if (planning.effects.request_pending_successor) {
    requestRouteRelease(RouteReleaseReason3D::kNoActiveRoute, 0U);
  }
  if (planning.status == PlanningCycleStatus3D::kTrackingHandoffRetained) {
    // The resident connector remains the sole finite execution owner until its
    // bounded handoff completes.
    return;
  }
  if (!planning.ready()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                          "PRODUCTION_MPPI_PLANNING_CYCLE status=%s action=fail_closed",
                          planningCycleStatus3DName(planning.status));
    publishFailClosedExecutionRevocation(
        ProductionMppiExecutionReason::kNoExecutableHorizon, now_ns);
    return;
  }

  if (planning.effects.request_static_tracking_world_refresh && objective) {
    maybeRequestStaticTrackingWorldRefresh(world, navigation, objective, now_ns);
  }
  if (planning.effects.request_route_extension) {
    maybeRequestStaticRouteExtensionFromExecution(
        world, world_build, planning.route.execution, navigation, now_ns);
  }
  if (planning.effects.request_stalled_route_release) {
    requestRouteRelease(RouteReleaseReason3D::kStalled, planning.route.generation);
  }

  for (const PassageTraversalEvidenceEvent& event :
       planning.effects.passage_traversal_events) {
    RCLCPP_INFO(get_logger(),
                "PASSAGE_TRAVERSAL_EVENT vehicle_id='%s' sequence=%" PRIu64
                " status=%s reason=%s passage='%s' route_generation=%" PRIu64
                " span_index=%zu observations=%zu duration_s=%.3f station_m=%.2f "
                "span_station_m=(%.2f,%.2f) position=(%.2f,%.2f,%.2f) "
                "maximum_cross_track_m=%.3f maximum_vertical_error_m=%.3f "
                "vertical_window_preserved=%s",
                config_.planning.vehicle_id.c_str(), event.sequence,
                passageTraversalEvidenceStatusName(event.status).data(),
                passageTraversalEvidenceReasonName(event.reason).data(),
                event.passage_traversal_id.c_str(), event.route_generation,
                event.span_index, event.traversal_observation_count, event.duration_s,
                event.station_m, event.begin_station_m, event.end_station_m,
                event.actual_position.x, event.actual_position.y,
                event.actual_position.z, event.maximum_cross_track_error_m,
                event.maximum_absolute_vertical_error_m,
                event.vertical_window_preserved ? "true" : "false");
  }
  if (planning.effects.passage_geometry_proximity.has_value()) {
    const PassageGeometryProximity3D& proximity =
        planning.effects.passage_geometry_proximity.value();
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "PASSAGE_GEOMETRY_PROXIMITY vehicle_id='%s' passage='%s' "
        "entry_distance_m=%.2f projection_station_m=%.2f "
        "projection_cross_track_m=%.2f minimum_clearance_m=%.2f "
        "within_corridor=%s",
        config_.planning.vehicle_id.c_str(), proximity.passage_traversal_id.c_str(),
        proximity.entry_distance_m, proximity.projection_station_m,
        proximity.projection_cross_track_m, proximity.minimum_clearance_m,
        proximity.within_corridor ? "true" : "false");
  }
  for (const PassageGeometryEvidenceEvent& event :
       planning.effects.passage_geometry_events) {
    RCLCPP_INFO(get_logger(),
                "PASSAGE_GEOMETRY_EVENT vehicle_id='%s' sequence=%" PRIu64
                " status=%s reason=%s passage='%s' observations=%zu "
                "duration_s=%.3f station_m=%.2f traversal_length_m=%.2f "
                "maximum_station_m=%.2f position=(%.2f,%.2f,%.2f) "
                "maximum_cross_track_m=%.3f",
                config_.planning.vehicle_id.c_str(), event.sequence,
                passageTraversalEvidenceStatusName(event.status).data(),
                passageTraversalEvidenceReasonName(event.reason).data(),
                event.passage_traversal_id.c_str(), event.observation_count,
                event.duration_s, event.station_m, event.traversal_length_m,
                event.maximum_station_m, event.actual_position.x,
                event.actual_position.y, event.actual_position.z,
                event.maximum_cross_track_error_m);
  }
  if (cooperative_passage_state_pub_) {
    cooperative_passage_state_pub_->publish(
        cooperativePassageIntentMessage(planning.controller.cooperative.passage));
  }
  logNonCooperativeUpdate(planning.controller.noncooperative);

  const double snapshot_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - snapshot_started)
                                 .count();
  const auto controller_started = std::chrono::steady_clock::now();
  std::optional<MppiControllerResult3D> controller_tick =
      runPlanningController(ProductionMppiControllerTick{
          .world = *world,
          .request = std::move(planning.controller.request),
          .route_generation = planning.route.generation,
          .now_ns = now_ns,
          .route_cross_track_m = planning.route.projection.cross_track_m,
          .direct_tracking_interception = direct_tracking_interception,
      });
  const double controller_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                controller_started)
          .count();
  if (!controller_tick.has_value()) {
    return;
  }
  mppi::MppiTickInput& input = controller_tick->input;
  mppi::MppiTickResult& result = controller_tick->result;
  const MppiEligibleRolloutUpdate& no_eligible_recovery =
      controller_tick->no_eligible_recovery;
  finalizePlanningTick(ProductionMppiPlanningTickFinalization{
      .input = input,
      .result = result,
      .world = world,
      .world_build = world_build,
      .route_pipeline = route_pipeline,
      .route_execution = planning.route.execution,
      .execution_input = execution_input,
      .latest_lidar_evidence = latest_lidar_evidence,
      .offboard_session = offboard_session,
      .offboard_session_receive_stamp_ns = offboard_session_receive_stamp_ns,
      .navigation = navigation,
      .execution_mppi_route = planning.route.controller_route,
      .execution_selected_passage_traversal_ids =
          planning.route.selected_passage_traversal_ids,
      .objective = objective,
      .prediction = prediction,
      .liveness = planning.controller.liveness,
      .direct_tracking_maneuver = planning.controller.direct_tracking_maneuver,
      .speed_policy = planning.controller.speed_policy,
      .route_progress = planning.controller.route_progress,
      .no_eligible_recovery = no_eligible_recovery,
      .goal_capture = planning.controller.goal_capture,
      .rollout_budget = planning.controller.rollout_budget,
      .cooperative = planning.controller.cooperative,
      .noncooperative = planning.controller.noncooperative,
      .route_projection = planning.route.projection,
      .mission_goal = mission_goal,
      .target_source = planning.controller.target_source,
      .route_generation = planning.route.generation,
      .memory_sequence = memory_sequence,
      .now_ns = now_ns,
      .pose_age_ms = pose_age_ms,
      .esdf_age_ms = esdf_age_ms,
      .observation_age_ms = observation_age_ms,
      .control_feedback_age_ms = control_feedback_age_ms,
      .snapshot_ms = snapshot_ms,
      .capture_ms = capture_ms,
      .execution_input_ms = execution_input_ms,
      .cycle_prepare_ms = cycle_prepare_ms,
      .controller_ms = controller_ms,
      .tick_started = snapshot_started,
      .route_execution_status = planning.route.execution_status,
      .planning_state = planning.controller.planning_state,
      .previous_control_source = execution_input_preparation.previous_control_source,
      .route_required_risk_tier = planning.controller.route_required_risk_tier,
      .executed_horizon_clearance = planning.controller.executed_horizon_clearance,
      .route_usable = planning.route.usable,
      .direct_tracking_interception = direct_tracking_interception,
      .local_route_stop_is_terminal = planning.route.local_stop_is_terminal,
      .pose_predicted = pose_predicted,
  });
}

void ProductionMppiNode::reportUnadoptedPendingRoute(
    const ProductionRouteExecutionSelection3D& execution) {
  if (execution.pending_route == nullptr || execution.pending_activation) {
    return;
  }
  if (execution.pending_eligibility == PendingRouteEligibility3D::kEligible) {
    // The plan may take this successor and did not take it: the refresh
    // against the current world produced nothing, or the splice onto the
    // resident route is not ready. That is the state a vehicle waits in with a
    // route already certified, and naming the eligibility alone said the
    // successor was fine.
    const std::string_view splice =
        routeSpliceReadinessStatus3DName(execution.pending_splice_readiness);
    const std::string_view refresh =
        routeCertificationStatus3DName(execution.pending_refresh_status);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "PENDING_ROUTE3D adopted=false eligibility=eligible refreshed=%s "
        "refresh_status=%.*s splice=%.*s "
        "pending_generation=%" PRIu64 " base_generation=%" PRIu64,
        execution.pending_refresh_available ? "true" : "false",
        static_cast<int>(refresh.size()), refresh.data(),
        static_cast<int>(splice.size()), splice.data(),
        execution.pending_route->route.identity.generation,
        execution.pending_route->base_route_generation);
    return;
  }
  // A successor is sealed and the plan cannot take it. Without this the hold
  // that follows names no reason at all: one recorded flight held for one and
  // three quarter seconds with a certified successor published.
  const std::string_view eligibility =
      pendingRouteEligibility3DName(execution.pending_eligibility);
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "PENDING_ROUTE3D adopted=false eligibility=%.*s pending_generation=%" PRIu64
      " base_generation=%" PRIu64 " plan_generation=%" PRIu64
      " base_owner_epoch=%" PRIu64,
      static_cast<int>(eligibility.size()), eligibility.data(),
      execution.pending_route->route.identity.generation,
      execution.pending_route->base_route_generation,
      execution.source_snapshot != nullptr
          ? execution.source_snapshot->routeGenerationHighWater()
          : 0U,
      execution.pending_route->base_execution_owner_epoch);
}

} // namespace drone_city_nav
