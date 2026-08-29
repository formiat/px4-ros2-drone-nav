#include "drone_city_nav/esdf_query.hpp"
#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/navigation_state_prediction.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <memory>
#include <span>

#include "production_mppi_node.hpp"
#include "production_mppi_node_planning_tick_context.hpp"
#include "production_mppi_node_planning_tick_finalize.hpp"
#include "production_mppi_node_planning_tick_rearm.hpp"
#include "production_mppi_route_helpers.hpp"

namespace drone_city_nav {

void ProductionMppiNode::planningTick() {
  if (!engine_) {
    return;
  }
  const std::int64_t tick_entry_ns = get_clock()->now().nanoseconds();
  if (mission_waypoint_capture_gate_) {
    mission_waypoint_capture_gate_->beginTick(tick_entry_ns);
  }
  const std::shared_ptr<const ProductionNavigationObjective> objective =
      navigationObjective();
  const ProductionTrackingObjective* tracking_objective =
      objective && objective->tracking.has_value() ? &objective->tracking.value()
                                                   : nullptr;
  const Point3 mission_goal = objective ? objective->goal : mission_goal_;
  const bool terminal_hold_enabled = !objective || !objective->continuous_tracking;
  const bool direct_tracking_interception =
      objective && objective->continuous_tracking && tracking_objective != nullptr &&
      tracking_objective->direct_interception_active;
  const bool observed_3d_world = !use_static_map_;
  if (handleRequestedExecutionRevocation(tick_entry_ns)) {
    // A callback-requested epoch is a hard barrier; retain it until revoke
    // publication has linearized with the exact snapshot.
    return;
  }
  const std::uint64_t line_of_sight_generation =
      tracking_objective != nullptr ? tracking_objective->line_of_sight_generation : 0U;
  const std::optional<DirectTrackingOwnerIdentity3D> direct_tracking_identity =
      makeDirectTrackingOwnerIdentity(objective.get(), direct_tracking_interception,
                                      line_of_sight_generation);
  const std::uint64_t effective_route_generation = directTrackingRouteGeneration(
      direct_tracking_interception, line_of_sight_generation);
  const std::uint64_t required_route_epoch =
      minimum_tracking_route_mission_epoch_.load(std::memory_order_acquire);
  const std::uint64_t required_route_sample =
      objective && objective->mission_epoch == required_route_epoch
          ? minimum_tracking_route_sample_sequence_.load(std::memory_order_acquire)
          : 0U;
  const auto snapshot_started = std::chrono::steady_clock::now();
  ProductionMppiNavigation navigation;
  ProductionMppiVehicleStatus vehicle_status;
  ProductionMppiPredictionError prediction;
  ProductionMppiAppliedControl applied_control;
  ProductionMppiExecutionHorizonOwner execution_horizon_owner;
  std::uint64_t applied_control_discontinuity_generation{0U};
  bool applied_control_discontinuity_generation_valid{false};
  OffboardSessionAdmissionState offboard_session;
  std::int64_t offboard_session_receive_stamp_ns{0};
  bool vehicle_status_epoch_stable{false};
  std::optional<ProductionMppiCooperativeCommand> cooperative_command;
  ProductionMppiNonCooperativeTracks noncooperative_tracks;
  LatestObservation latest_observation;
  std::uint64_t memory_sequence{0U};
  bool raw_world_identity_conflicted{false};
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
    navigation.valid = navigation.valid && !navigation_revision_exhausted_ &&
                       !navigation_frame_reset_unresolved_;
    vehicle_status = vehicle_status_;
    prediction = latest_prediction_error_;
    applied_control = applied_control_;
    execution_horizon_owner = execution_horizon_owner_;
    applied_control_discontinuity_generation =
        applied_control_discontinuity_generation_;
    applied_control_discontinuity_generation_valid =
        !applied_control_discontinuity_generation_exhausted_;
    offboard_session = offboard_session_admission_;
    offboard_session_receive_stamp_ns = offboard_session_receive_stamp_ns_;
    vehicle_status_epoch_stable =
        !vehicle_status_epoch_probation_ && !vehicle_status_revision_exhausted_;
    cooperative_command = cooperative_command_;
    noncooperative_tracks = noncooperative_tracks_;
    latest_observation = latest_observation_tracker_.latest();
    memory_sequence = latest_observation.sequence;
    raw_world_identity_conflicted = raw_world_identity_conflicted_;
  }
  std::optional<ProductionMppiPreparedEsdf> esdf;
  {
    const std::scoped_lock lock{world_generation_publication_mutex_, esdf_state_mutex_};
    esdf = prepared_esdf_;
  }
  const std::shared_ptr<const ProductionMppiRawWorld3D> latest_raw_world_3d =
      latest_raw_world_3d_.load(std::memory_order_acquire);
  const bool latest_lidar_evidence_identity_conflicted =
      latest_lidar_evidence_identity_conflicted_.load(std::memory_order_acquire);
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence =
      latest_lidar_evidence_identity_conflicted
          ? nullptr
          : latest_lidar_evidence_.load(std::memory_order_acquire);
  const std::shared_ptr<const ExecutionRouteSnapshot3D> execution_snapshot =
      execution_route_store_.snapshot();
  // Timestamp the immutable planning view only after all callback-owned inputs
  // have been captured. A concurrently published evidence value may have a
  // receive stamp later than tick entry, but never later than this boundary.
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  const double pose_age_ms =
      static_cast<double>(now_ns - navigation.receive_stamp_ns) / 1.0e6;
  double esdf_age_ms = std::numeric_limits<double>::infinity();
  if (esdf.has_value()) {
    esdf_age_ms = use_static_map_
                      ? 0.0
                      : static_cast<double>(now_ns - esdf->ready_stamp_ns) / 1.0e6;
  }
  double observation_age_ms = std::numeric_limits<double>::infinity();
  if (use_static_map_) {
    observation_age_ms = 0.0;
  } else if (esdf.has_value() && !raw_world_identity_conflicted) {
    if (latest_raw_world_3d != nullptr &&
        latest_raw_world_3d->version.producer_instance_id ==
            esdf->producer_instance_id) {
      observation_age_ms = committedRawWorldAgeMs(latest_raw_world_3d.get(), now_ns);
    }
  }
  const double control_feedback_age_ms =
      applied_control.valid
          ? static_cast<double>(now_ns - applied_control.receive_stamp_ns) / 1.0e6
          : std::numeric_limits<double>::infinity();
  const bool world_current =
      world_ready_.load(std::memory_order_acquire) && esdf.has_value() &&
      observation_age_ms >= 0.0 &&
      observation_age_ms <= maximum_esdf_age_ms_ + stale_esdf_execution_window_ms_;
  const NavigationHealthAssessment navigation_health =
      updateNavigationHealth(objective, applied_control, execution_horizon_owner,
                             execution_snapshot, world_current, now_ns);
  if (navigation_health.terminal &&
      optional_constraints_.nonphysical_execution_revocation_enabled) {
    publishFailClosedExecutionRevocation(
        terminalExecutionReason(navigation_health.failure), now_ns);
    return;
  }
  if (!vehicleStatusAuthoritativeForExecution(vehicle_status,
                                              vehicle_status_epoch_stable, now_ns,
                                              maximum_vehicle_status_age_ms_)) {
    if (execution_horizon_owner.valid) {
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
      static_cast<void>(handleRequestedExecutionRevocation(now_ns));
    }
    return;
  }
  const bool goal_capture_latched =
      mission_goal_capture_latch_ && objective && terminal_hold_enabled &&
      mission_goal_capture_latch_->latchedFor(mission_goal);
  const MissionWaypointUpdate early_waypoint_update = updateMissionWaypoint(
      objective, navigation, vehicle_status, applied_control, execution_horizon_owner,
      applied_control_discontinuity_generation,
      applied_control_discontinuity_generation_valid, vehicle_status_epoch_stable,
      goal_capture_latched, now_ns);
  if (early_waypoint_update.waypoint_completed) {
    // The planner published the aggregate acknowledgement before replacing the
    // objective. Replan the newly active leg on the next tick.
    return;
  }
  if (mission_goal_capture_attempt_invalidated_) {
    // Preserve the exact wire owner until the queued revocation linearizes.
    // A replacement capture lease may be committed only on a later tick.
    return;
  }
  const bool matching_goal_capture_attempt =
      goal_capture_latched && execution_horizon_owner.valid &&
      execution_horizon_owner.execution_mode ==
          msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD &&
      execution_horizon_owner.execution_reason ==
          msg::MppiTrajectoryHorizon::EXECUTION_REASON_GOAL_CAPTURE &&
      execution_horizon_owner.stationary_position_hold &&
      now_ns >= execution_horizon_owner.valid_from_ns &&
      now_ns < execution_horizon_owner.valid_until_ns &&
      distance3D(execution_horizon_owner.route_target, mission_goal) <=
          mission_waypoint_capture_gate_config_.target_match_tolerance_m &&
      distance3D(execution_horizon_owner.stationary_hold_position, mission_goal) <=
          mission_waypoint_capture_gate_config_.target_match_tolerance_m;
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
  if (pose_age_ms > maximum_pose_age_ms_) {
    const NavigationStatePredictionResult predicted =
        predictNavigationState(navigation.state, pose_age_ms / 1000.0,
                               maximum_pose_prediction_age_ms_ / 1000.0);
    if (!predicted.valid) {
      publishFailClosedExecutionRevocation(
          ProductionMppiExecutionReason::kNoExecutableHorizon, now_ns);
      return;
    }
    navigation.state = predicted.state;
    pose_predicted = predicted.predicted;
  }
  if (!esdf.has_value() || observation_age_ms < 0.0 ||
      observation_age_ms > maximum_esdf_age_ms_ + stale_esdf_execution_window_ms_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "PRODUCTION_MPPI_UNAVAILABLE_WORLD action=wait_for_world "
                         "observation_age_ms=%.1f esdf_content_age_ms=%.1f "
                         "maximum_execution_age_ms=%.1f",
                         observation_age_ms, esdf_age_ms,
                         maximum_esdf_age_ms_ + stale_esdf_execution_window_ms_);
    publishFailClosedExecutionRevocation(
        ProductionMppiExecutionReason::kUnavailableWorld, now_ns);
    return;
  }
  if (!worldGenerationAvailableForPlanning(*esdf, now_ns)) {
    return;
  }
  if (!engine_->ready()) {
    publishFailClosedExecutionRevocation(
        ProductionMppiExecutionReason::kNoExecutableHorizon, now_ns);
    return;
  }
  const bool planned_owner_witnessed =
      appliedControlAuthoritativeForExecution(applied_control, execution_horizon_owner,
                                              now_ns, maximum_control_feedback_age_ms_);
  switch (assessPlannedHorizonSupersession(execution_horizon_owner,
                                           planned_owner_witnessed, now_ns)) {
    case ProductionMppiHorizonSupersessionDecision::kDeferredAwaitingOwnerWitness:
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "EXECUTION_HORIZON_SUPERSESSION deferred=true reason=awaiting_owner_witness "
          "producer=%" PRIu64 " sequence=%" PRIu64,
          execution_horizon_owner.producer_instance_id,
          execution_horizon_owner.sequence);
      return;
    case ProductionMppiHorizonSupersessionDecision::kRejectedOwnerNotCurrent:
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "EXECUTION_HORIZON_SUPERSESSION rejected=true reason=owner_not_current "
          "action=%s "
          "producer=%" PRIu64 " sequence=%" PRIu64,
          optional_constraints_.nonphysical_execution_revocation_enabled
              ? "revoke"
              : "replace_expired_owner",
          execution_horizon_owner.producer_instance_id,
          execution_horizon_owner.sequence);
      if (optional_constraints_.nonphysical_execution_revocation_enabled) {
        publishFailClosedExecutionRevocation(
            ProductionMppiExecutionReason::kNoExecutableHorizon, now_ns);
        return;
      }
      break;
    case ProductionMppiHorizonSupersessionDecision::kAllowedNoPlannedOwner:
    case ProductionMppiHorizonSupersessionDecision::kAllowedWitnessedOwner:
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
  const bool stationary_capture_rearm = stationaryCaptureRearmEligibleForPlanningTick(
      ProductionMppiStationaryCaptureRearmContext{
          .objective = objective.get(),
          .mission_waypoint_sequence = mission_waypoint_sequence_.get(),
          .navigation = &navigation,
          .vehicle_status = &vehicle_status,
          .applied_control = &applied_control,
          .execution_horizon_owner = &execution_horizon_owner,
          .offboard_session = &offboard_session,
          .esdf = std::addressof(*esdf),
          .latest_raw_world_3d = latest_raw_world_3d,
          .latest_lidar_evidence = latest_lidar_evidence,
          .execution_snapshot = execution_snapshot,
          .validation_policy = execution_validation_policy_,
          .static_occupancy_3d = static_occupancy_3d_,
          .capture_gate_config = mission_waypoint_capture_gate_config_,
          .mission_goal = mission_goal,
          .now_ns = now_ns,
          .offboard_session_receive_stamp_ns = offboard_session_receive_stamp_ns,
          .maximum_pose_age_ms = maximum_pose_age_ms_,
          .maximum_control_feedback_age_ms = maximum_control_feedback_age_ms_,
          .maximum_esdf_age_ms = maximum_esdf_age_ms_,
          .observation_age_ms = observation_age_ms,
          .vehicle_status_epoch_stable = vehicle_status_epoch_stable,
          .terminal_hold_enabled = terminal_hold_enabled,
          .goal_capture_latched = goal_capture_latched,
          .use_static_map = use_static_map_,
          .observed_3d_world = observed_3d_world,
      });
  const ProductionMppiExecutionInputPreparation execution_input_preparation =
      prepareExecutionInputForPlanningTick(
          navigation, applied_control, execution_horizon_owner,
          execution_input_sequence, now_ns, maximum_control_feedback_age_ms_,
          pose_predicted, stationary_capture_rearm);
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
  ProductionRouteExecutionSelection3D route_execution;
  bool route_usable{false};
  RouteExecutionStatus3D route_execution_status{RouteExecutionStatus3D::kNoActiveRoute};
  RouteProgressProjection3D measured_route_projection;
  Point3 route_hold_position{
      navigation.state.x,
      navigation.state.y,
      clampToFlightEnvelope(navigation.state.z, flight_envelope_config_)
          .value_or(flight_envelope_config_.minimum_target_z_m),
  };
  route_execution = resolveRouteExecution3D(
      *esdf, objective.get(), navigation, execution_input, latest_raw_world_3d,
      latest_lidar_evidence, now_ns, required_route_sample, direct_tracking_identity,
      observed_3d_world);
  const PendingCertifiedRouteRecoveryResult3D pending_recovery =
      recoverPendingCertifiedRouteLiveness3D(
          pending_certified_route_mailbox_, route_execution.pending_route,
          PendingCertifiedRouteRecoveryObservation3D{
              .direct_tracking_requested =
                  route_execution.direct_tracking_identity.has_value(),
              .execution_owner_available = route_execution.execution_owner_available,
              .pending_activation = route_execution.pending_activation,
          });
  if (pending_recovery.request_successor) {
    requestRouteRelease(RouteReleaseReason3D::kNoActiveRoute, 0U);
  }
  if (route_execution.tracking_error_tube_handoff_active) {
    // The controller is still consuming the exact finite connector certified
    // in the resident immutable plan. Replanning would reset its execution
    // clock and turn a bounded handoff into an open-ended exemption.
    return;
  }
  route_usable = route_execution.route_usable;
  route_execution_status = route_execution.status;
  measured_route_projection = route_execution.projection;
  route_hold_position = route_execution.hold_position;
  const CertifiedRouteSuffix3D* const activated_route = route_execution.route.get();
  const ProductionRouteGeometry3D* const route_geometry =
      activated_route != nullptr ? activated_route->geometry.get() : nullptr;
  const std::uint64_t route_generation = activated_route != nullptr
                                             ? activated_route->identity.generation
                                             : esdf->route_generation;
  const bool route_reaches_mission_goal =
      activated_route != nullptr
          ? activated_route->identity.proposal.reaches_mission_goal
          : esdf->route_reaches_mission_goal;
  RouteEndpointSemantics3D route_endpoint_semantics =
      RouteEndpointSemantics3D::kLocalStop;
  if (activated_route != nullptr) {
    route_endpoint_semantics = activated_route->planned_endpoint_semantics;
  } else if (route_reaches_mission_goal) {
    route_endpoint_semantics = terminal_hold_enabled
                                   ? RouteEndpointSemantics3D::kMissionStop
                                   : RouteEndpointSemantics3D::kContinuation;
  }
  const std::shared_ptr<const std::vector<RouteSample3D>> execution_route =
      route_geometry != nullptr ? route_geometry->route : esdf->route_3d;
  const std::shared_ptr<const std::vector<mppi::RouteSample3D>> execution_mppi_route =
      route_geometry != nullptr ? route_geometry->mppi_route : esdf->mppi_route;
  const std::shared_ptr<const std::vector<ConstrainedRouteSpan>>
      execution_constrained_spans =
          route_geometry != nullptr ? route_geometry->constrained_spans
                                    : esdf->constrained_spans;
  const std::shared_ptr<const std::vector<CooperativePassageAssignment>>
      execution_cooperative_passage_assignments =
          route_geometry != nullptr ? route_geometry->cooperative_passage_assignments
                                    : esdf->cooperative_passage_assignments;
  const std::shared_ptr<const std::vector<PassageTraversalId>>
      execution_selected_passage_traversal_ids =
          route_geometry != nullptr ? route_geometry->selected_passage_traversal_ids
                                    : esdf->selected_passage_traversal_ids;
  const bool route_execution_blocked =
      !direct_tracking_interception && objective && !route_usable;
  const double route_station_m = route_execution.station_m;
  RouteProgressProjection3D route_projection = measured_route_projection;
  if (route_projection.valid) {
    route_projection.station_m = route_station_m;
    route_projection.remaining_m =
        std::max(0.0, route_projection.total_length_m - route_station_m);
  }
  if (use_static_map_ && objective && objective->continuous_tracking) {
    maybeRequestStaticTrackingWorldRefresh(*esdf, navigation, *objective, now_ns);
  }
  if (use_static_map_ || observed_3d_world) {
    maybeRequestStaticRouteExtensionFromExecution(*esdf, route_execution, navigation,
                                                  now_ns);
  }
  const std::span<const RouteSample3D> route_3d =
      route_usable && execution_route ? std::span<const RouteSample3D>{*execution_route}
                                      : std::span<const RouteSample3D>{};
  const std::span<const ConstrainedRouteSpan> constrained_spans =
      route_usable && execution_constrained_spans
          ? std::span<const ConstrainedRouteSpan>{*execution_constrained_spans}
          : std::span<const ConstrainedRouteSpan>{};
  const ConstrainedRouteObservation route_constraint = observeConstrainedRoute(
      route_3d, constrained_spans, route_generation, route_projection.station_m,
      Point3{navigation.state.x, navigation.state.y, navigation.state.z},
      Vec3{navigation.state.vx, navigation.state.vy, navigation.state.vz},
      route_envelope_config_, route_constraint_diagnostics_distance_m_);
  const Point3 actual_position{navigation.state.x, navigation.state.y,
                               navigation.state.z};
  for (const PassageTraversalEvidenceEvent& event :
       passage_traversal_evidence_tracker_.update(route_constraint, actual_position,
                                                  now_ns)) {
    RCLCPP_INFO(get_logger(),
                "PASSAGE_TRAVERSAL_EVENT vehicle_id='%s' sequence=%" PRIu64
                " status=%s reason=%s passage='%s' route_generation=%" PRIu64
                " span_index=%zu observations=%zu duration_s=%.3f station_m=%.2f "
                "span_station_m=(%.2f,%.2f) position=(%.2f,%.2f,%.2f) "
                "maximum_cross_track_m=%.3f maximum_vertical_error_m=%.3f "
                "vertical_window_preserved=%s",
                vehicle_id_.c_str(), event.sequence,
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
  std::vector<PassageGeometryObservation> passage_geometry_observations;
  const PassageTraversalEdge* nearest_passage_entry = nullptr;
  RouteProjection3D nearest_passage_projection;
  double nearest_passage_entry_distance_m = std::numeric_limits<double>::infinity();
  if (esdf->passage_traversals) {
    passage_geometry_observations.reserve(esdf->passage_traversals->size());
    for (const PassageTraversalEdge& passage : *esdf->passage_traversals) {
      const RouteProjection3D projection =
          projectOntoRoute3D(passage.centerline, actual_position);
      const double entry_distance_m = distance3D(actual_position, passage.entry);
      if (entry_distance_m < nearest_passage_entry_distance_m) {
        nearest_passage_entry = &passage;
        nearest_passage_projection = projection;
        nearest_passage_entry_distance_m = entry_distance_m;
      }
      const double traversal_length_m =
          passage.centerline.empty() ? 0.0 : passage.centerline.back().station_m;
      passage_geometry_observations.push_back(PassageGeometryObservation{
          .passage_traversal_id = passage.id,
          .within_corridor =
              projection.valid && projection.distance_m <= passage.minimum_clearance_m,
          .station_m = projection.station_m,
          .traversal_length_m = traversal_length_m,
          .cross_track_error_m = projection.distance_m,
      });
    }
  }
  if (nearest_passage_entry != nullptr && nearest_passage_entry_distance_m < 8.0) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "PASSAGE_GEOMETRY_PROXIMITY vehicle_id='%s' passage='%s' "
                         "entry_distance_m=%.2f projection_station_m=%.2f "
                         "projection_cross_track_m=%.2f minimum_clearance_m=%.2f "
                         "within_corridor=%s",
                         vehicle_id_.c_str(), nearest_passage_entry->id.c_str(),
                         nearest_passage_entry_distance_m,
                         nearest_passage_projection.station_m,
                         nearest_passage_projection.distance_m,
                         nearest_passage_entry->minimum_clearance_m,
                         nearest_passage_projection.valid &&
                                 nearest_passage_projection.distance_m <=
                                     nearest_passage_entry->minimum_clearance_m
                             ? "true"
                             : "false");
  }
  for (const PassageGeometryEvidenceEvent& event :
       passage_geometry_evidence_tracker_.update(passage_geometry_observations,
                                                 actual_position, now_ns,
                                                 PassageGeometryEvidenceConfig{})) {
    RCLCPP_INFO(get_logger(),
                "PASSAGE_GEOMETRY_EVENT vehicle_id='%s' sequence=%" PRIu64
                " status=%s reason=%s passage='%s' observations=%zu "
                "duration_s=%.3f station_m=%.2f traversal_length_m=%.2f "
                "maximum_station_m=%.2f position=(%.2f,%.2f,%.2f) "
                "maximum_cross_track_m=%.3f",
                vehicle_id_.c_str(), event.sequence,
                passageTraversalEvidenceStatusName(event.status).data(),
                passageTraversalEvidenceReasonName(event.reason).data(),
                event.passage_traversal_id.c_str(), event.observation_count,
                event.duration_s, event.station_m, event.traversal_length_m,
                event.maximum_station_m, event.actual_position.x,
                event.actual_position.y, event.actual_position.z,
                event.maximum_cross_track_error_m);
  }
  const ConstrainedRouteControl route_control = constrained_route_coordinator_.update(
      route_constraint, speed_policy_config_.cruise_speed_mps,
      constrained_route_control_config_);
  const MissionGoalCaptureResult goal_capture =
      mission_goal_capture_latch_ && terminal_hold_enabled
          ? mission_goal_capture_latch_->update(MissionGoalCaptureObservation{
                .mission_goal = mission_goal,
                .state = navigation.state,
                .terminal_route_available = route_reaches_mission_goal,
            })
          : MissionGoalCaptureResult{};
  const bool local_route_stop_is_terminal =
      route_usable && route_projection.valid &&
      route_endpoint_semantics == RouteEndpointSemantics3D::kLocalStop;
  MppiSpeedPolicyResult speed_policy = evaluateMppiSpeedPolicy(
      speed_policy_config_,
      MppiSpeedPolicyInput{
          .state = navigation.state,
          .mission_goal = mission_goal,
          .route = route_3d,
          .route_endpoint_remaining_m =
              route_usable && route_projection.valid &&
                      routeEndpointHasTerminalStop3D(route_endpoint_semantics)
                  ? std::optional<double>{route_projection.remaining_m}
                  : std::nullopt,
          .route_constraint_speed_limit_mps =
              route_control.active
                  ? std::optional<double>{route_control.speed_limit_mps}
                  : std::nullopt,
          .route_endpoint_semantics = route_endpoint_semantics,
          .terminal_goal_limit_enabled = terminal_hold_enabled,
      });
  const std::span<const CooperativePassageAssignment> passage_assignments =
      execution_cooperative_passage_assignments
          ? std::span<
                const CooperativePassageAssignment>{*execution_cooperative_passage_assignments}
          : std::span<const CooperativePassageAssignment>{};
  const ProductionMppiCooperativeUpdate cooperative =
      prepareCooperativeTick(passage_assignments, route_constraint, cooperative_command,
                             now_ns, speed_policy.reference_speed_mps);
  const ProductionMppiNonCooperativeUpdate noncooperative =
      prepareNonCooperativeTick(navigation.state, noncooperative_tracks, now_ns);
  const bool noncooperative_cost_influence_active =
      noncooperative.enabled &&
      noncooperative.avoidance.influence.cost_influence_active;
  const bool noncooperative_evasive_maneuver_active =
      noncooperative.enabled &&
      noncooperative.avoidance.influence.evasive_maneuver_active;
  if (cooperative.yield.active) {
    speed_policy.reference_speed_mps =
        std::min(speed_policy.reference_speed_mps, cooperative.yield.maximum_speed_mps);
    speed_policy.target_lookahead_m = std::min(
        speed_policy.target_lookahead_m,
        std::max(0.0, cooperative.yield.hold_station_m - route_projection.station_m));
  }
  std::string target_source;
  double target_station_m = 0.0;
  mppi::State target;
  if (direct_tracking_interception) {
    target = mppi::State{
        .x = static_cast<float>(mission_goal.x),
        .y = static_cast<float>(mission_goal.y),
        .z = static_cast<float>(mission_goal.z),
        .yaw = navigation.state.yaw,
    };
    target_source = tracking_objective != nullptr &&
                            tracking_objective->predicted_intercept_path_clear
                        ? "tracking_direct_full_prediction"
                        : "tracking_direct_shortened_prediction";
  } else if (route_execution_blocked) {
    target = mppi::State{
        .x = static_cast<float>(route_hold_position.x),
        .y = static_cast<float>(route_hold_position.y),
        .z = static_cast<float>(route_hold_position.z),
        .yaw = navigation.state.yaw,
    };
    target_source = objective->continuous_tracking ? "tracking_no_executable_route_"
                                                   : "no_executable_route_";
    target_source += routeExecutionStatus3DName(route_execution_status);
  } else {
    const std::span<const mppi::RouteSample3D> mppi_route =
        execution_mppi_route
            ? std::span<const mppi::RouteSample3D>{*execution_mppi_route}
            : std::span<const mppi::RouteSample3D>{};
    target =
        selectTarget(route_3d, mppi_route, route_station_m,
                     speed_policy.target_lookahead_m, target_source, target_station_m);
  }
  if (route_control.active) {
    target.z = static_cast<float>(route_control.reference_z_m);
    if (route_control.hold_xy) {
      target_source = "passage_vertical_alignment_hold";
    } else if (route_control.vertical_ready) {
      target_source = "passage_traversal";
    } else {
      target_source = "passage_vertical_alignment";
    }
    if (route_control.hold_xy) {
      target.x = navigation.state.x;
      target.y = navigation.state.y;
      speed_policy.reference_speed_mps = 0.0;
      speed_policy.target_lookahead_m = 0.0;
    }
  }
  if (cooperative.yield.active && route_usable && route_projection.valid &&
      execution_route && !execution_route->empty() && !route_control.hold_xy) {
    const RouteSample3D hold_sample =
        sampleRoute3DAtStation(*execution_route, cooperative.yield.hold_station_m);
    target.x = static_cast<float>(hold_sample.position.x);
    target.y = static_cast<float>(hold_sample.position.y);
    target.z = static_cast<float>(hold_sample.position.z);
    target_station_m = hold_sample.station_m;
    target_source = cooperative.yield.hold_at_entry
                        ? "cooperative_passage_yield_hold"
                        : "cooperative_passage_yield_deceleration";
  }
  ProductionMppiPlanningState planning_state = ProductionMppiPlanningState::kPlanned;
  if (objective && objective->immediate_hold) {
    planning_state = ProductionMppiPlanningState::kMissionCommandPositionHold;
    target = mppi::State{
        .x = static_cast<float>(mission_goal.x),
        .y = static_cast<float>(mission_goal.y),
        .z = static_cast<float>(mission_goal.z),
        .yaw = navigation.state.yaw,
    };
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
    target_source = "mission_command_position_hold";
  } else if (goal_capture.latched) {
    planning_state = ProductionMppiPlanningState::kMissionGoalPositionHold;
    target = mppi::State{
        .x = static_cast<float>(mission_goal.x),
        .y = static_cast<float>(mission_goal.y),
        .z = static_cast<float>(mission_goal.z),
        .yaw = navigation.state.yaw,
    };
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
    target_source = "mission_goal_position_hold";
  } else if (route_execution_blocked) {
    planning_state = ProductionMppiPlanningState::kNoExecutableRouteHold;
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
  } else if (cooperative.yield.active && cooperative.yield.hold_at_entry &&
             !cooperative.mppi.avoidance_active && !route_control.hold_xy) {
    planning_state = ProductionMppiPlanningState::kCooperativePassageYieldHold;
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
    target_source = "cooperative_passage_yield_hold";
  }
  MppiLivenessResult liveness;
  if (liveness_supervisor_ && !direct_tracking_interception) {
    liveness = liveness_supervisor_->evaluate(MppiLivenessObservation{
        .stamp_ns = now_ns,
        .actual_state = navigation.state,
        .controller_active = execution_input_preparation.control_feedback_fresh &&
                             planning_state == ProductionMppiPlanningState::kPlanned &&
                             !route_control.hold_xy && route_projection.valid,
        .predicted_head_progress_m =
            previous_result_.has_value() ? previous_result_->head_progress_m : 0.0,
        .predicted_terminal_progress_m =
            previous_result_.has_value() ? previous_result_->terminal_progress_m : 0.0,
        .route_generation = route_generation,
        .route_station_m = route_projection.station_m,
        .route_station_valid = route_projection.valid,
    });
  }
  RouteProgressUpdate3D route_progress;
  if (route_progress_tracker_ && !direct_tracking_interception) {
    const RouteProgressProjection3D& projection = route_projection;
    route_progress = route_progress_tracker_->evaluate(RouteProgressObservation3D{
        .stamp_ns = now_ns,
        .route_generation =
            projection.valid && planning_state == ProductionMppiPlanningState::kPlanned
                ? route_generation
                : 0U,
        .station_m = projection.station_m,
        .predicted_head_progress_m =
            previous_result_.has_value() ? previous_result_->head_progress_m : 0.0,
        .cross_track_m = projection.cross_track_m,
        .recovery_active = liveness.recovery_active,
        .controller_active = execution_input_preparation.control_feedback_fresh &&
                             planning_state == ProductionMppiPlanningState::kPlanned &&
                             !route_control.hold_xy,
    });
    if (route_progress.stalled && optional_constraints_.route_progress_replan_enabled) {
      requestRouteRelease(RouteReleaseReason3D::kStalled, route_generation);
    }
  }
  mppi::RiskTier route_required_risk_tier = mppi::RiskTier::kPreferred;
  if (route_usable && execution_mppi_route && route_projection.valid) {
    const double horizon_distance_m = std::max(
        target_station_m - route_projection.station_m,
        speed_policy.reference_speed_mps * static_cast<double>(mppi_config_.steps) *
            static_cast<double>(mppi_config_.dynamics.dt_s));
    route_required_risk_tier = mppi::maximumRequiredRiskTier(
        *execution_mppi_route, static_cast<float>(route_projection.station_m),
        static_cast<float>(route_projection.station_m +
                           std::max(0.0, horizon_distance_m)));
  }
  std::optional<mppi::MovingTargetReference> moving_target;
  if (objective && objective->continuous_tracking && objective->tracking.has_value()) {
    const ProductionTrackingObjective& tracking = objective->tracking.value();
    const double observation_age_s = static_cast<double>(std::max<std::int64_t>(
                                         0, now_ns - tracking.observation_stamp_ns)) *
                                     1.0e-9;
    const TargetVerticalPrediction vertical_prediction = predictTargetVerticalMotion(
        tracking.observed_position.z, tracking.observed_velocity.z, observation_age_s,
        mppi_config_.dynamics.maximum_vertical_acceleration_mps2,
        flight_envelope_config_);
    const float minimum_z =
        static_cast<float>(flight_envelope_config_.minimum_target_z_m);
    const float maximum_z = std::nextafter(
        static_cast<float>(flight_envelope_config_.maximum_target_z_m), minimum_z);
    if (vertical_prediction.valid && std::isfinite(minimum_z) &&
        std::isfinite(maximum_z) && maximum_z > minimum_z) {
      const float bounded_vertical_z = mppi::clampMovingTargetAltitude(
          static_cast<float>(vertical_prediction.z_m), minimum_z, maximum_z);
      moving_target = mppi::MovingTargetReference{
          .state =
              mppi::State{
                  .x = static_cast<float>(tracking.observed_position.x +
                                          tracking.observed_velocity.x *
                                              observation_age_s),
                  .y = static_cast<float>(tracking.observed_position.y +
                                          tracking.observed_velocity.y *
                                              observation_age_s),
                  .z = bounded_vertical_z,
                  .vx = static_cast<float>(tracking.observed_velocity.x),
                  .vy = static_cast<float>(tracking.observed_velocity.y),
                  .vz = static_cast<float>(vertical_prediction.velocity_mps),
              },
          .capture_radius_m = static_cast<float>(tracking_capture_radius_m_),
          .vertical_deceleration_mps2 =
              mppi_config_.dynamics.maximum_vertical_acceleration_mps2,
          .minimum_z_m = minimum_z,
          .maximum_z_m = maximum_z,
          .bounded_vertical_motion = true,
      };
    }
  }
  DirectTrackingManeuverUpdate direct_tracking_maneuver;
  if (direct_tracking_interception && moving_target.has_value()) {
    direct_tracking_maneuver =
        direct_tracking_maneuver_lifecycle_.update(DirectTrackingManeuverObservation{
            .interceptor_position =
                Point3{navigation.state.x, navigation.state.y, navigation.state.z},
            .interceptor_velocity =
                Vec3{navigation.state.vx, navigation.state.vy, navigation.state.vz},
            .target_position = Point3{moving_target->state.x, moving_target->state.y,
                                      moving_target->state.z},
            .target_velocity = Vec3{moving_target->state.vx, moving_target->state.vy,
                                    moving_target->state.vz},
            .stamp_ns = now_ns,
            .line_of_sight_generation = line_of_sight_generation,
            .active = planning_state == ProductionMppiPlanningState::kPlanned,
        });
  } else {
    direct_tracking_maneuver = direct_tracking_maneuver_lifecycle_.update({});
  }
  const MppiNominalReseedUpdate nominal_reseed =
      nominal_reseed_tracker_.update(MppiNominalReseedObservation{
          .route_generation = direct_tracking_interception ? effective_route_generation
                                                           : route_generation,
          .local_liveness_generation = liveness.reseed_generation,
          .route_liveness_generation = route_progress.local_reseed_generation,
          .direct_tracking_maneuver_generation =
              direct_tracking_maneuver.reseed_generation,
      });
  const EsdfQueryResult current_clearance =
      queryConservativeEsdf3D(esdf->grid, *esdf->distances_m, navigation.state.x,
                              navigation.state.y, navigation.state.z);
  const double tracking_age_ms =
      tracking_objective != nullptr && tracking_objective->observation_stamp_ns > 0
          ? static_cast<double>(std::max<std::int64_t>(
                0, now_ns - tracking_objective->observation_stamp_ns)) /
                1.0e6
          : std::numeric_limits<double>::infinity();
  const MppiRolloutBudgetDecision rollout_budget = selectMppiRolloutBudget(
      rollout_budget_config_,
      MppiRolloutBudgetObservation{
          .static_world = use_static_map_,
          .route_available =
              direct_tracking_interception || (route_usable && route_projection.valid),
          .direct_tracking = direct_tracking_interception,
          .clearance_valid = current_clearance.status == EsdfQueryStatus::kValid,
          .clearance_m = current_clearance.clearance_m,
          .world_age_ms = observation_age_ms,
          .tracking_age_ms = tracking_age_ms,
          .required_risk_tier = direct_tracking_interception
                                    ? mppi::RiskTier::kPreferred
                                    : route_required_risk_tier,
      });
  const mppi::DeterministicCandidateKind deterministic_candidate =
      planningDeterministicCandidate(direct_tracking_interception, planning_state,
                                     route_usable, route_projection.valid,
                                     route_control.hold_xy);
  mppi::MppiTickInput input{
      .initial_state = execution_input->state(),
      .target = target,
      .pose_revision = execution_input->poseRevision(),
      .obstacle_revision =
          planningRawRevision(use_static_map_, esdf->revision, latest_raw_world_3d),
      .expected_esdf_revision = esdf->local_world_generation.gpu_esdf_revision,
      .planning_stamp_ns = now_ns,
      .previous_applied_control = execution_input->previousControl(),
      .nominal_reseed_generation = nominal_reseed.generation,
      .reference_speed_mps = speed_policy.enabled
                                 ? static_cast<float>(speed_policy.reference_speed_mps)
                                 : -1.0F,
      .moving_target = moving_target,
      .route =
          planning_state == ProductionMppiPlanningState::kPlanned && route_usable &&
                  execution_mppi_route && route_projection.valid &&
                  !route_control.hold_xy
              ? std::optional<mppi::RouteReference>{mppi::RouteReference{
                    .points = execution_mppi_route,
                    .generation = route_generation,
                    .initial_station_m = static_cast<float>(route_projection.station_m),
                    .terminal_cross_track_tolerance_m = routeCrossTrackTolerance3D(
                        optional_constraints_.route_cross_track_constraints_enabled),
                }}
              : std::nullopt,
      .dynamic_aircraft = noncooperative_cost_influence_active
                              ? noncooperative.avoidance.trajectories
                              : cooperative.mppi.dynamic_aircraft,
      .dynamic_aircraft_cost_policy =
          noncooperative_cost_influence_active
              ? std::optional<mppi::DynamicAircraftCostPolicy>{noncooperative.avoidance
                                                                   .cost_policy}
              : std::nullopt,
      .cooperative_maneuver = cooperative.mppi.maneuver,
      .cooperative_acquisition = cooperative.mppi.acquisition,
      .noncooperative_acquisition = noncooperative_evasive_maneuver_active
                                        ? noncooperative.avoidance.acquisition
                                        : std::nullopt,
      .active_rollouts = rollout_budget.active_rollouts,
      .deterministic_candidate = deterministic_candidate,
      .prefer_route_directed_candidate =
          !optional_constraints_.stochastic_trajectory_selection_enabled ||
          liveness.recovery_active || route_progress.local_reseed_requested,
      .cooperative_avoidance_active = cooperative.mppi.avoidance_active,
      .noncooperative_avoidance_active = noncooperative_evasive_maneuver_active,
  };
  const double snapshot_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - snapshot_started)
                                 .count();
  std::optional<ProductionMppiControllerTickResult> controller_tick =
      runPlanningController(ProductionMppiControllerTick{
          .esdf = *esdf,
          .input = input,
          .nominal_reseed = nominal_reseed,
          .target = target,
          .snapshot_started = snapshot_started,
          .route_generation = route_generation,
          .now_ns = now_ns,
          .route_cross_track_m = route_projection.cross_track_m,
          .planning_state = planning_state,
          .direct_tracking_interception = direct_tracking_interception,
      });
  if (!controller_tick.has_value()) {
    return;
  }
  mppi::MppiTickResult& result = controller_tick->result;
  const MppiEligibleRolloutUpdate& no_eligible_recovery =
      controller_tick->no_eligible_recovery;
  finalizePlanningTick(ProductionMppiPlanningTickFinalization{
      .input = input,
      .result = result,
      .esdf = *esdf,
      .route_execution = route_execution,
      .execution_input = execution_input,
      .latest_lidar_evidence = latest_lidar_evidence,
      .offboard_session = offboard_session,
      .offboard_session_receive_stamp_ns = offboard_session_receive_stamp_ns,
      .navigation = navigation,
      .execution_mppi_route = execution_mppi_route,
      .execution_selected_passage_traversal_ids =
          execution_selected_passage_traversal_ids,
      .objective = objective,
      .prediction = prediction,
      .liveness = liveness,
      .direct_tracking_maneuver = direct_tracking_maneuver,
      .speed_policy = speed_policy,
      .route_progress = route_progress,
      .no_eligible_recovery = no_eligible_recovery,
      .goal_capture = goal_capture,
      .rollout_budget = rollout_budget,
      .cooperative = cooperative,
      .noncooperative = noncooperative,
      .route_projection = route_projection,
      .mission_goal = mission_goal,
      .target_source = target_source,
      .route_generation = route_generation,
      .memory_sequence = memory_sequence,
      .now_ns = now_ns,
      .pose_age_ms = pose_age_ms,
      .esdf_age_ms = esdf_age_ms,
      .observation_age_ms = observation_age_ms,
      .control_feedback_age_ms = control_feedback_age_ms,
      .snapshot_ms = snapshot_ms,
      .route_execution_status = route_execution_status,
      .planning_state = planning_state,
      .previous_control_source = execution_input_preparation.previous_control_source,
      .route_required_risk_tier = route_required_risk_tier,
      .route_usable = route_usable,
      .direct_tracking_interception = direct_tracking_interception,
      .local_route_stop_is_terminal = local_route_stop_is_terminal,
      .pose_predicted = pose_predicted,
  });
}

} // namespace drone_city_nav
