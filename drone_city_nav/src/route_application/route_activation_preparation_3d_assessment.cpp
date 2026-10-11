#include "drone_city_nav/execution_horizon_commit_3d.hpp"
#include "drone_city_nav/execution_plan_3d.hpp"
#include "drone_city_nav/execution_route_certification_3d.hpp"
#include "drone_city_nav/route_risk_annotation_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <utility>

#include "production_mppi_route_helpers.hpp"
#include "production_mppi_route_world.hpp"
#include "route_activation_preparation_3d.hpp"
#include "route_activation_preparation_3d_stages.hpp"

namespace drone_city_nav::route_activation_preparation_detail {

RouteActivationPreparationState3D
assessAdmission(RouteActivationPreparationState3D state,
                const RouteActivationCoordinatorConfig3D& config) {
  const PlannerSearchTransaction3D& transaction = *state.transaction;
  const ProductionRouteActivationSnapshot3D& snapshot = state.prepared.snapshot;
  ProductionRouteActivationResult3D& result = state.prepared.result;
  MaterializedRoute3D& candidate = result.materialized;
  RouteAdmissionReport3D& report = result.admission;
  const std::uint64_t candidate_generation = candidate.candidate_generation;

  if (report.candidate_validation.accepted && candidate.route != nullptr &&
      candidate.route->size() >= 2U) {
    const Point3 snapshot_position{snapshot.navigation.state.x,
                                   snapshot.navigation.state.y,
                                   snapshot.navigation.state.z};
    const RouteProjection3D reserve_projection = projectOntoRoute3DWithinStationWindow(
        *candidate.route, snapshot_position, candidate.route->front().station_m,
        candidate.route->back().station_m);
    StaticRoutePlanningLatencyStats latency = state.planning_latency;
    if (latency.sample_count == 0U) {
      latency.planning_p95_ms = std::max(0.0, result.telemetry.route_search_ms);
      latency.planning_p99_ms = std::max(
          latency.planning_p95_ms, 1000.0 * config.route_extension.maximum_latency_s);
      latency.build_and_planning_p99_ms = latency.planning_p99_ms;
    }
    const ProductionMppiForwardAcceleration3D forward_acceleration =
        productionMppiForwardAcceleration3D(snapshot.navigation);
    const StaticRouteExtensionDecision reserve_decision = evaluateStaticRouteExtension(
        config.route_extension,
        StaticRouteExtensionObservation{
            .route_generation = candidate_generation,
            .route_station_m = reserve_projection.station_m,
            .route_remaining_m = reserve_projection.remaining_m,
            .horizontal_speed_mps =
                std::hypot(snapshot.navigation.state.vx, snapshot.navigation.state.vy),
            .forward_acceleration_mps2 = forward_acceleration.horizontal_mps2,
            .vertical_speed_mps = std::abs(snapshot.navigation.state.vz),
            .forward_vertical_acceleration_mps2 = forward_acceleration.vertical_mps2,
            .planning_latency_p95_ms = latency.planning_p95_ms,
            .planning_latency_p99_ms = latency.planning_p99_ms,
            .build_and_planning_latency_p99_ms = latency.build_and_planning_p99_ms,
            .route_reaches_mission_goal = candidate.reaches_mission_goal,
        });
    const RouteEndpointSemantics3D endpoint_semantics = routeEndpointSemantics3D(
        candidate.reaches_mission_goal, !transaction.objective.continuous_tracking);
    report.certified_reserve = assessCertifiedRouteReserve3D(
        reserve_decision,
        reserve_projection.valid ? reserve_projection.remaining_m
                                 : std::numeric_limits<double>::quiet_NaN(),
        endpoint_semantics);
    if (!report.certified_reserve.accepted()) {
      report.candidate_validation = StaticRouteCandidateValidation{
          .status =
              report.certified_reserve.status == CertifiedRouteReserveStatus3D::kInvalid
                  ? StaticRouteCandidateStatus::kInvalidCertifiedReserve
                  : StaticRouteCandidateStatus::kInsufficientCertifiedReserve};
    }
  }

  NavigationWorldCertificate3D validated_world_certificate =
      navigationWorldCertificate3D(*candidate.world);
  SegmentEvidence3D activation_evidence = candidate.segment_evidence;
  if (candidate.route != nullptr && candidate.route->size() >= 2U) {
    const Point3 snapshot_position{snapshot.navigation.state.x,
                                   snapshot.navigation.state.y,
                                   snapshot.navigation.state.z};
    activation_evidence.route_length_m = 0.0;
    for (std::size_t index = 1U; index < candidate.route->size(); ++index) {
      activation_evidence.route_length_m += distance3D(
          (*candidate.route)[index - 1U].position, (*candidate.route)[index].position);
    }
    activation_evidence.objective_cost =
        result.trajectory != nullptr && result.trajectory->time_profile.valid()
            ? result.trajectory->time_profile.travel_time_s
            : std::numeric_limits<double>::infinity();
    activation_evidence.endpoint_displacement_m =
        distance3D(snapshot_position, candidate.route->back().position);
    activation_evidence.mission_progress_m =
        distance3D(snapshot_position, candidate.intent.mission_target) -
        distance3D(candidate.route->back().position, candidate.intent.mission_target);
    activation_evidence.net_coordinate_progress_m = routeNetCoordinateProgress3D(
        snapshot_position, candidate.route->back().position,
        candidate.intent.mission_target);
  }
  activation_evidence.physical_executable =
      candidate.planner_executable && report.candidate_validation.accepted;
  const MaterializedRouteProposal3D activation_identity{
      .planned_world = state.planned_world_certificate,
      .validated_world = validated_world_certificate,
      .objective = transaction.objective,
      .intent = candidate.intent,
      .evidence = activation_evidence,
      .route_fingerprint = candidate.fingerprint,
      .route_sample_count = candidate.route != nullptr ? candidate.route->size() : 0U,
      .reaches_mission_goal = candidate.reaches_mission_goal,
      .activation_eligible = activation_evidence.physical_executable,
  };

  const std::uint64_t required_objective_sample =
      snapshot.objective != nullptr && snapshot.objective->mission_epoch ==
                                           snapshot.minimum_tracking_route_mission_epoch
          ? snapshot.minimum_tracking_route_sample_sequence
          : 0U;
  report.required_objective_sample = required_objective_sample;
  state.departure_seed = candidate.world->proprioceptive_free_space_seed;
  if (state.departure_seed.has_value()) {
    state.departure_seed->position =
        Point3{snapshot.navigation.state.x, snapshot.navigation.state.y,
               snapshot.navigation.state.z};
    state.departure_seed->departure_chain =
        candidate.route != nullptr
            ? departureChain3D(*candidate.route, candidate.departure_end_station_m)
            : std::vector<Point3>{};
  }
  report.assessment = assessRouteActivation3D(
      activation_identity,
      candidate.route != nullptr ? std::span<const RouteSample3D>{*candidate.route}
                                 : std::span<const RouteSample3D>{},
      RouteActivationObservation3D{
          .resident_world = snapshot.resident_world != nullptr
                                ? navigationWorldCertificate3D(*snapshot.resident_world)
                                : NavigationWorldCertificate3D{},
          .current_objective = snapshot.objective != nullptr
                                   ? makeStaticRouteObjective(*snapshot.objective)
                                   : StaticRouteObjective{},
          .minimum_tracking_sample_sequence = required_objective_sample,
          .position = {snapshot.navigation.state.x, snapshot.navigation.state.y,
                       snapshot.navigation.state.z},
          .maximum_cross_track_m = config.route_tracking.maximum_cross_track_m,
          .latest_raw_occupancy = snapshot.raw_world != nullptr
                                      ? &snapshot.raw_world->occupancy()
                                      : nullptr,
          .latest_raw_producer_instance_id =
              snapshot.raw_world != nullptr
                  ? snapshot.raw_world->version().producer_instance_id
                  : 0U,
          .latest_raw_revision = snapshot.raw_world != nullptr
                                     ? snapshot.raw_world->version().revision
                                     : 0U,
          .footprint = activationFootprint(config),
          .launch_support_contact =
              optionalAddress(candidate.world->launch_support_contact),
          .proprioceptive_free_space_seed = optionalAddress(state.departure_seed),
          .flight_envelope = config.flight_envelope,
          .raw_validation_required = state.raw_validation_required,
      });
  report.world_compatible = report.assessment.publication.compatible();
  report.objective_matches = report.assessment.objective_matches;
  if (state.raw_validation_required && !report.assessment.raw_world_compatible &&
      report.world_compatible && report.objective_matches) {
    report.candidate_validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kRawWorldUnavailable};
  } else if (report.assessment.raw_validation.status ==
             RawRouteSuffixStatus3D::kRawCollision) {
    report.candidate_validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kRawCollision,
        .failure_segment_index = report.assessment.raw_validation.failure_route_segment,
        .failure_point = report.assessment.raw_validation.failure_point,
    };
  }

  const bool raw_route_valid = !state.raw_validation_required ||
                               (report.assessment.raw_world_compatible &&
                                report.assessment.raw_validation.accepted() &&
                                report.assessment.raw_validation.connector_validated &&
                                report.assessment.raw_validation.suffix_validated);
  activation_evidence.validated_through_revision =
      report.assessment.raw_validated_through_revision;
  activation_evidence.physical_executable = candidate.planner_executable &&
                                            report.candidate_validation.accepted &&
                                            raw_route_valid;
  if (activation_evidence.physical_executable) {
    activation_evidence.status = SegmentEvidenceStatus3D::kValid;
  } else if (report.candidate_validation.status ==
             StaticRouteCandidateStatus::kRawCollision) {
    activation_evidence.status = SegmentEvidenceStatus3D::kRawCollision;
    activation_evidence.failure_segment_index =
        report.candidate_validation.failure_segment_index;
    activation_evidence.failure_point = report.candidate_validation.failure_point;
  } else if (state.raw_validation_required && !report.assessment.raw_world_compatible) {
    activation_evidence.status = SegmentEvidenceStatus3D::kInvalidWorld;
  }
  candidate.segment_evidence = activation_evidence;
  validated_world_certificate.raw_validated_through_revision =
      report.assessment.raw_validated_through_revision;

  result.proposal = ProductionMaterializedRouteProposal3D{
      .identity =
          MaterializedRouteProposal3D{
              .planned_world = state.planned_world_certificate,
              .validated_world = validated_world_certificate,
              .objective = transaction.objective,
              .intent = candidate.intent,
              .evidence = activation_evidence,
              .route_fingerprint = candidate.fingerprint,
              .route_sample_count =
                  candidate.route != nullptr ? candidate.route->size() : 0U,
              .reaches_mission_goal = candidate.reaches_mission_goal,
              .activation_eligible = activation_evidence.physical_executable,
          },
      .trajectory = result.trajectory,
      .decorations = result.decorations,
  };

  if (result.proposal.trajectory == nullptr &&
      report.trajectory_validation.reason ==
          CompiledTrajectoryFailureReason3D::kNotAttempted) {
    report.trajectory_validation = {CompiledTrajectoryFailureReason3D::kMissingRoute,
                                    0U};
  } else if (result.proposal.trajectory != nullptr &&
             result.proposal.trajectory->route != nullptr) {
    report.trajectory_validation =
        validateCompiledTrajectorySamples3D(*result.proposal.trajectory->route);
    const ActivatedRouteIdentity3D candidate_identity{
        .generation = candidate_generation,
        .proposal = result.proposal.identity,
    };
    if (report.trajectory_validation.valid() &&
        (!compiledTrajectoryValid3D(*result.proposal.trajectory, candidate_identity) ||
         result.proposal.decorations == nullptr ||
         !routeDecorationsValid3D(*result.proposal.decorations,
                                  *result.proposal.trajectory, candidate_generation) ||
         result.proposal.trajectory->exact_initial_state !=
             exactVehicleState3D(snapshot.navigation))) {
      report.trajectory_validation.reason =
          CompiledTrajectoryFailureReason3D::kDerivedResourceMismatch;
      report.decoration_validation.reason =
          RouteDecorationFailureReason3D::kDerivedResourceMismatch;
    }
  }

  const AppliedControlEvidence3D captured_control =
      snapshot.execution_authority != nullptr ? snapshot.execution_authority->control()
                                              : AppliedControlEvidence3D{};
  const ExecutionOwnerIdentity3D captured_owner =
      snapshot.execution_authority != nullptr ? snapshot.execution_authority->owner()
                                              : ExecutionOwnerIdentity3D{};
  const bool handoff_control_fresh = appliedControlAuthoritativeForExecution3D(
      captured_control, captured_owner, snapshot.stamp_ns,
      config.maximum_control_feedback_age_ms);
  if (result.proposal.identity.activation_eligible && report.assessment.accepted() &&
      result.proposal.trajectory != nullptr &&
      candidate.world->distances_m != nullptr && snapshot.navigation.valid) {
    report.handoff = config.dynamic_handoff_validator(DynamicHandoffRequest3D{
        .current_state = snapshot.navigation.state,
        .previous_applied_control =
            handoff_control_fresh ? captured_control.control : MotionControl3D{},
        .candidate_trajectory = result.proposal.trajectory,
        .reference_speed_mps = static_cast<float>(config.cruise_speed_mps),
        .maximum_cross_track_m =
            static_cast<float>(config.route_tracking.maximum_cross_track_m),
        .terminal_cross_track_tolerance_m =
            static_cast<float>(kFiniteExecutionRouteCrossTrackToleranceM3D),
        .grid = candidate.world->grid,
        .derived_distances_m = candidate.world->distances_m,
    });
  }

  result.proposal.identity.activation_eligible =
      result.proposal.identity.activation_eligible && report.assessment.accepted() &&
      report.handoff.accepted() && result.proposal.trajectory != nullptr &&
      result.proposal.decorations != nullptr && result.proposal.trajectory->route &&
      result.proposal.trajectory->constrained_spans && report.world_compatible &&
      report.objective_matches && report.trajectory_validation.valid();

  if (report.candidate_validation.accepted && !report.world_compatible) {
    report.activation_status = StaticRouteActivationStatus::kWorldPublicationRejected;
  } else if (report.candidate_validation.accepted && !report.objective_matches) {
    report.activation_status = StaticRouteActivationStatus::kStaleObjective;
  } else if (report.candidate_validation.accepted &&
             !report.compiledTrajectoryValid()) {
    report.activation_status = StaticRouteActivationStatus::kInvalidExecutionGeometry;
  } else if (report.candidate_validation.accepted &&
             !report.readyForArbitration(result.proposal)) {
    report.activation_status = StaticRouteActivationStatus::kDynamicHandoffRejected;
  }
  return state;
}

RouteActivationPreparationState3D
assessReplacement(RouteActivationPreparationState3D state,
                  const RouteActivationCoordinatorConfig3D& config) {
  const PlannerSearchTransaction3D& transaction = *state.transaction;
  const ProductionRouteActivationSnapshot3D& snapshot = state.prepared.snapshot;
  ProductionRouteActivationResult3D& result = state.prepared.result;
  MaterializedRoute3D& candidate = result.materialized;
  RouteAdmissionReport3D& report = result.admission;
  const ProductionMaterializedRouteProposal3D& materialized_proposal = result.proposal;
  const std::uint64_t candidate_generation = candidate.candidate_generation;

  report.commit_assessment_performed = true;
  report.generation_assessed = true;
  const std::shared_ptr<const ExecutionPlan3D>& current_execution =
      state.prepared.execution_base;
  const CertifiedRouteSuffix3D* const current_route =
      current_execution != nullptr ? current_execution->route() : nullptr;
  state.base_generation =
      current_execution != nullptr ? current_execution->routeGenerationHighWater() : 0U;
  const bool request_requires_base =
      transaction.extension() || transaction.replacement();
  const bool base_generation_matches =
      !request_requires_base ||
      transaction.request.base_route_generation == state.base_generation;
  const bool allocation_generation_matches =
      state.base_generation != std::numeric_limits<std::uint64_t>::max() &&
      candidate_generation == state.base_generation + 1U;
  report.generation_matches = base_generation_matches && allocation_generation_matches;
  report.certification_execution_base_current =
      sameExecutionRouteBase3D(state.prepared.execution_base, current_execution);
  const ActivatedRouteIdentity3D* const active_identity =
      current_route != nullptr ? std::addressof(current_route->identity) : nullptr;
  // A resident route released as blocked or diverged is one the vehicle
  // cannot follow any more, and one released as stalled is one it has stood
  // on instead of flying: neither is a base a successor has to improve on,
  // so the first raw-valid successor replaces it.
  const bool safety_replan_requested =
      transaction.replacement() &&
      (transaction.release_reason == RouteReleaseReason3D::kBlocked ||
       transaction.release_reason == RouteReleaseReason3D::kDiverged ||
       transaction.release_reason == RouteReleaseReason3D::kStalled);
  state.overlap_search =
      candidate.provenance.required_splice_base_route_instance_id.valid();
  const PendingCertifiedRoute3D* const captured_pending =
      snapshot.pending_route != nullptr && current_execution != nullptr &&
              pendingCertifiedRouteEligible3D(*snapshot.pending_route,
                                              *current_execution)
          ? snapshot.pending_route.get()
          : nullptr;
  const CertifiedRouteSuffix3D* const improvement_resident =
      captured_pending != nullptr ? std::addressof(captured_pending->route)
                                  : current_route;
  report.successor_compared_to_pending = captured_pending != nullptr;
  // A resident route the vehicle is not following is no base a successor has
  // to improve on either. A route the plan keeps while the vehicle brakes on a
  // continuation stop is still measured against, and one recorded flight
  // refused a successor for being two hundredths of a second slower while it
  // had nothing to fly at all and was holding without an executable horizon. A
  // pending certified route is different -- the vehicle is about to fly it --
  // so it stays a base worth improving on.
  const bool improvement_base_under_execution =
      captured_pending != nullptr ||
      (current_execution != nullptr &&
       current_execution->phase() == ExecutionRoutePhase3D::kFollowing);
  if (improvement_resident != nullptr) {
    const std::optional<ActiveIntent3D> resident_intent =
        activeIntent3D(improvement_resident->identity.proposal);
    const std::optional<ActiveIntent3D> candidate_intent =
        activeIntent3D(materialized_proposal.identity);
    const bool same_intent =
        resident_intent.has_value() && candidate_intent.has_value() &&
        sameActiveIntent3D(
            resident_intent.value(),   // NOLINT(bugprone-unchecked-optional-access)
            candidate_intent.value()); // NOLINT(bugprone-unchecked-optional-access)
    const bool point_to_point_intent =
        same_intent && !resident_intent.value_or(ActiveIntent3D{}).continuous_tracking;
    report.successor_improvement_required =
        point_to_point_intent && !safety_replan_requested &&
        improvement_base_under_execution &&
        improvement_resident->identity.proposal.reaches_mission_goal &&
        materialized_proposal.identity.reaches_mission_goal;
    if (report.successor_improvement_required &&
        improvement_resident->geometry != nullptr &&
        improvement_resident->geometry->route != nullptr &&
        materialized_proposal.trajectory != nullptr) {
      const Point3 current_position{snapshot.navigation.state.x,
                                    snapshot.navigation.state.y,
                                    snapshot.navigation.state.z};
      const RouteProjection3D resident_projection =
          projectOntoRoute3DWithinStationWindow(
              *improvement_resident->geometry->route, current_position,
              improvement_resident->progress.station_m,
              improvement_resident->geometry->route->back().station_m);
      report.successor_improvement = assessRouteSuccessorImprovement3D(
          *improvement_resident->geometry, resident_projection.station_m,
          *materialized_proposal.trajectory, report.assessment.projection.station_m,
          config.successor_improvement);
    }
  }
  // A route released as blocked is blocked at one station; its remaining time
  // is still what the vehicle was about to spend. A replacement searched from
  // the vehicle that would cost that time twice over is the first route the
  // search found, not the best it can find: one recorded flight committed to a
  // 247 m loop in place of a 66 m route the moment the search produced it,
  // watched the search shorten the loop to 95 m over the next minute while it
  // flew the loop west, and ran out of mission time. Such a replacement waits
  // out the grace its extra cost earns while the vehicle still flies the
  // blocked route's validated prefix. The search converging on its world does
  // not end the wait: one recorded flight was handed a converged 195 m loop
  // six seconds into a 36 s grace, flew it, and never got back. Standing at
  // the block does end it: measured over the urban flights r247 to r260 (43
  // rests under a deferred replacement, 97 s), the candidate shortened by
  // 58 s while the vehicle stood, 39 s less than the standing itself cost
  // before the stop and the restart are counted; a vehicle at rest takes the
  // replacement it has, and a better one replaces it under the successor
  // improvement rule. A route released as stalled is past that
  // wait: the vehicle stood through whatever grace its block earned, and the
  // release is what ends the standing, so its successor is not weighed
  // against it. One recorded flight held a successor a tenth of a second
  // slower than the route the vehicle was standing on, and the next ones
  // for a second and a half more.
  report.blocked_replacement_resident_available =
      current_route != nullptr && current_route->geometry != nullptr &&
      current_route->geometry->route != nullptr;
  if (safety_replan_requested &&
      transaction.release_reason != RouteReleaseReason3D::kStalled &&
      !state.overlap_search && report.blocked_replacement_resident_available &&
      materialized_proposal.trajectory != nullptr &&
      current_route->identity.proposal.reaches_mission_goal &&
      materialized_proposal.identity.reaches_mission_goal) {
    const Point3 current_position{snapshot.navigation.state.x,
                                  snapshot.navigation.state.y,
                                  snapshot.navigation.state.z};
    const RouteProjection3D blocked_projection = projectOntoRoute3DWithinStationWindow(
        *current_route->geometry->route, current_position,
        current_route->progress.station_m,
        current_route->geometry->route->back().station_m);
    const RouteSuccessorImprovementAssessment3D against_blocked =
        assessRouteSuccessorImprovement3D(
            *current_route->geometry, blocked_projection.station_m,
            *materialized_proposal.trajectory, report.assessment.projection.station_m,
            config.successor_improvement);
    report.blocked_replacement = against_blocked;
    report.blocked_replacement_assessed = true;
    report.blocked_replacement_held_s = state.blocked_replacement_held_s;
    report.blocked_replacement_grace_s =
        kBlockedReplacementGraceLossRatio *
        std::max(0.0, against_blocked.candidate_remaining_time_s -
                          against_blocked.resident_remaining_time_s);
    const MotionState3D& vehicle = snapshot.navigation.state;
    const bool vehicle_at_rest =
        std::hypot(std::hypot(vehicle.vx, vehicle.vy), vehicle.vz) <=
        kStationaryExecutionHoldSpeedToleranceMps;
    report.blocked_replacement_deferred =
        against_blocked.resident_remaining_time_s > 0.0 && !vehicle_at_rest &&
        state.blocked_replacement_held_s < report.blocked_replacement_grace_s;
  }
  // A deferred replacement is drafted for nobody: with a pending draft the
  // commit publishes it as certified pending before any admission verdict is
  // read, and the first recorded deferral was activated that way the moment
  // the rule held it.
  state.successor_improvement_cleared = (!report.successor_improvement_required ||
                                         report.successor_improvement.accepted()) &&
                                        !report.blocked_replacement_deferred;
  report.replacement = assessRouteProposalReplacement3D(
      active_identity, materialized_proposal.identity,
      RouteProposalReplacementObservation3D{
          .safety_replan_requested = safety_replan_requested,
          .continuity_preserving_successor =
              candidate.provenance.required_splice_base_route_instance_id.valid(),
          .materially_improved_point_to_point_successor =
              report.successor_improvement_required &&
              report.successor_improvement.accepted()});
  return state;
}

} // namespace drone_city_nav::route_activation_preparation_detail
