#include "drone_city_nav/certified_route_splice_3d.hpp"
#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/execution_route_transitions_3d.hpp"
#include "drone_city_nav/motion_altitude_envelope_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/occupied_collision_oracle_3d.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <numeric>
#include <ranges>
#include <span>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "execution_route_snapshot_3d_internal.hpp"

namespace drone_city_nav::execution_route_snapshot_3d_internal {

namespace {

[[nodiscard]] ExecutionRouteTransitionResult3D replaceCertifiedRouteImpl(
    const ExecutionPlan3D& current, const ExecutionRouteTransitionGuard3D& guard,
    CertifiedRouteSuffix3D successor, FiniteExecutionPlan3D successor_execution,
    const CertifiedRouteSplice3D* const splice) {
  const ExecutionRouteTransitionStatus3D guard_status = checkGuard(current, guard);
  if (guard_status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(guard_status);
  }
  const CertifiedRouteSuffix3D* const current_route = routePointer(current);
  if (current_route == nullptr) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCurrentSnapshot);
  }
  const bool same_active_intent = sameActiveIntent3D(current_route->owner.active_intent,
                                                     successor.owner.active_intent);
  if ((splice != nullptr && !same_active_intent) ||
      (!same_active_intent && successor.owner.id == current_route->owner.id)) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate,
                             ExecutionRouteTransitionDetail3D::kActiveIntentConflict);
  }
  if (same_active_intent) {
    successor.owner = current_route->owner;
  }
  const bool route_owner_changes = successor.owner.id != current_route->owner.id;
  const bool splice_binding_mismatch =
      splice != nullptr && distance3D(successor.progress.last_observed_position,
                                      current_route->progress.last_observed_position) >
                               kExecutionBindingToleranceM;
  if (!successor.valid() ||
      current_route->identity.generation == std::numeric_limits<std::uint64_t>::max() ||
      current.execution_owner_epoch == std::numeric_limits<std::uint64_t>::max() ||
      successor.identity.generation != current_route->identity.generation + 1U ||
      splice_binding_mismatch) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kInvalidCandidate,
        ExecutionRouteTransitionDetail3D::kSuccessorIdentityMismatch);
  }
  const bool replacement_phase_allowed =
      executionRouteAcceptsCertifiedReplacement3D(current);
  const bool suspended_without_execution =
      splice == nullptr &&
      current.phase() == ExecutionRoutePhase3D::kAwaitingSuccessor &&
      current.finiteExecution() == nullptr;
  if (!replacement_phase_allowed ||
      (current.finiteExecution() == nullptr && !suspended_without_execution) ||
      successor_execution.command_horizon.execution_input == nullptr) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  if (splice != nullptr) {
    const MotionState3D& splice_state =
        successor_execution.command_horizon.execution_input->state();
    const RouteSpliceReadiness3D splice_readiness = assessRouteSpliceReadiness3D(
        *splice, *current_route, successor,
        Point3{splice_state.x, splice_state.y, splice_state.z});
    if (!splice_readiness.ready()) {
      return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate,
                               ExecutionRouteTransitionDetail3D::kSpliceNotReady);
    }
  }
  ExecutionRouteTransitionDetail3D successor_evidence_regression =
      ExecutionRouteTransitionDetail3D::kNone;
  if (current.finiteExecution() != nullptr) {
    successor_evidence_regression =
        successorEvidenceRegression(*current_route, *current.finiteExecution(),
                                    successor, successor_execution.command_horizon);
  } else if (current_route->progress.execution_input == nullptr) {
    successor_evidence_regression =
        ExecutionRouteTransitionDetail3D::kResidentProgressInputMissing;
  } else if (!executionInputNotOlder(
                 *successor_execution.command_horizon.execution_input,
                 *current_route->progress.execution_input)) {
    successor_evidence_regression =
        ExecutionRouteTransitionDetail3D::kSuccessorExecutionInputOlder;
  } else {
    successor_evidence_regression = successorRouteEvidenceRegression(
        *current_route, successor, successor_execution.command_horizon);
  }
  if (successor_execution.command_horizon.kind != FiniteExecutionKind3D::kNominal) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  if (successor_evidence_regression != ExecutionRouteTransitionDetail3D::kNone) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kCertificateRegression,
                             successor_evidence_regression);
  }

  bindProgressToExecutionInput(
      successor.progress, successor_execution.command_horizon.execution_input,
      successor_execution.command_horizon.begin_route_station_m);
  if (!successor.valid() || !successor_execution.validFor(successor) ||
      !candidateFiniteExecutionValid(successor_execution.command_horizon, current,
                                     &successor, true)) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  ExecutionPlan3D next = current;
  ++next.version;
  const std::uint64_t successor_generation = successor.identity.generation;
  next.state = FollowingPlan3D{
      .route = std::move(successor),
      .execution = std::move(successor_execution),
  };
  if (route_owner_changes) {
    ++next.execution_owner_epoch;
  }
  next.route_generation_high_water = successor_generation;
  return finishTransition(current, std::move(next));
}

} // namespace

ExecutionRouteTransitionResult3D applyActivateCertifiedRouteCommand3D(
    const ExecutionPlan3D& current, const std::uint64_t expected_snapshot_version,
    CertifiedRouteSuffix3D candidate, FiniteExecutionPlan3D candidate_execution) {
  const ExecutionRouteTransitionStatus3D status =
      checkCurrentAndVersion(current, expected_snapshot_version);
  if (status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(status);
  }
  if (!candidate.valid() ||
      current.routeGenerationHighWater() == std::numeric_limits<std::uint64_t>::max() ||
      current.execution_owner_epoch == std::numeric_limits<std::uint64_t>::max() ||
      candidate.identity.generation != current.routeGenerationHighWater() + 1U ||
      candidate_execution.command_horizon.kind != FiniteExecutionKind3D::kNominal ||
      candidate_execution.command_horizon.execution_input == nullptr) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kInvalidCandidate,
        ExecutionRouteTransitionDetail3D::kActivationIdentityMismatch);
  }
  const bool empty_owner =
      (current.phase() == ExecutionRoutePhase3D::kAwaitingSuccessor ||
       current.phase() == ExecutionRoutePhase3D::kRevoked) &&
      current.route() == nullptr && current.finiteExecution() == nullptr &&
      current.directTrackingExecution() == nullptr &&
      current.stationaryHold() == nullptr;
  const bool stationary_owner = current.phase() == ExecutionRoutePhase3D::kStopped &&
                                current.stationaryHold() != nullptr;
  // A stop is a finite trajectory, never a state that withholds movement: the
  // moment a route is certified from the vehicle, it takes the vehicle back,
  // whether the stop has already been flown to rest or is still braking.
  const bool stopping_owner = current.phase() == ExecutionRoutePhase3D::kStopping &&
                              current.stopExecution() != nullptr;
  if ((!empty_owner && !stationary_owner && !stopping_owner) ||
      (stationary_owner &&
       !routeExecutionEvidenceNotOlderThanHold(candidate_execution.command_horizon,
                                               *current.stationaryHold())) ||
      (stopping_owner &&
       !routeExecutionEvidenceNotOlderThanStop(candidate_execution.command_horizon,
                                               *current.stopExecution()))) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  bindProgressToExecutionInput(
      candidate.progress, candidate_execution.command_horizon.execution_input,
      candidate_execution.command_horizon.begin_route_station_m);
  if (!candidate.valid() || !candidate_execution.validFor(candidate) ||
      !candidateFiniteExecutionValid(candidate_execution.command_horizon, current,
                                     &candidate, true)) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kInvalidCandidate,
        ExecutionRouteTransitionDetail3D::kActivationBindingInvalid);
  }
  ExecutionPlan3D next = current;
  ++next.version;
  const std::uint64_t candidate_generation = candidate.identity.generation;
  next.state = FollowingPlan3D{
      .route = std::move(candidate),
      .execution = std::move(candidate_execution),
  };
  ++next.execution_owner_epoch;
  next.route_generation_high_water = candidate_generation;
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D applyAdvanceCertifiedRouteCommand3D(
    const ExecutionPlan3D& current, const ExecutionRouteTransitionGuard3D& guard,
    RouteExecutionObservation3D observation,
    std::shared_ptr<const VersionedExecutionInput3D> execution_input,
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world) {
  const ExecutionRouteTransitionStatus3D guard_status = checkGuard(current, guard);
  if (guard_status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(guard_status);
  }
  const CertifiedRouteSuffix3D* const current_route = routePointer(current);
  if (current_route == nullptr) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCurrentSnapshot);
  }
  const CertifiedRouteSuffix3D& route = *current_route;
  if (route.progress.execution_input == nullptr || execution_input == nullptr ||
      !execution_input->valid() || !execution_input->nominalStateAuthoritative() ||
      !executionInputFreshAt(*execution_input, *route.validation_policy,
                             execution_input->effectiveStampNs())) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kInvalidCandidate,
        ExecutionRouteTransitionDetail3D::kProgressExecutionInputStale);
  }
  const ExecutionInputProgressRelation3D progress_relation =
      executionInputProgressRelation(*execution_input, *route.progress.execution_input);
  if (progress_relation == ExecutionInputProgressRelation3D::kInvalid) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kNonMonotonicProgress);
  }
  if (progress_relation == ExecutionInputProgressRelation3D::kReplay) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kNoChange);
  }
  observation.position = executionInputPosition(*execution_input);
  observation.flight_envelope = route.validation_policy->flightEnvelope();
  const CertificateView3D old_certificate = certificateView(route.certificate);
  if (old_certificate.observed_raw) {
    if (observed_raw_world == nullptr || !observed_raw_world->valid() ||
        observed_raw_world->version().producer_instance_id !=
            old_certificate.producer_instance_id ||
        observed_raw_world->version().revision <
            old_certificate.validated_through_revision ||
        (observed_raw_world->version().revision ==
             old_certificate.validated_through_revision &&
         observed_raw_world->contentFingerprint() !=
             old_certificate.world_content_fingerprint)) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kInvalidCandidate,
          ExecutionRouteTransitionDetail3D::kProgressWorldOlderThanCertificate);
    }
    observation.latest_raw_occupancy = &observed_raw_world->occupancy();
    observation.latest_raw_producer_instance_id =
        observed_raw_world->version().producer_instance_id;
    observation.latest_raw_revision = observed_raw_world->version().revision;
    observation.launch_support_contact =
        observed_raw_world->launchSupportContact().has_value()
            ? &*observed_raw_world->launchSupportContact()
            : nullptr;
    observation.proprioceptive_free_space_seed =
        observed_raw_world->proprioceptiveFreeSpaceSeed().has_value()
            ? &*observed_raw_world->proprioceptiveFreeSpaceSeed()
            : nullptr;
    if (validationPolicyFingerprint(observation.footprint,
                                    observation.launch_support_contact) !=
        old_certificate.validation_policy_fingerprint) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kInvalidCandidate,
          ExecutionRouteTransitionDetail3D::kProgressValidationPolicyChanged);
    }
    const bool derived_geometry_world_changed =
        observed_raw_world->occupiedContentFingerprint() !=
        old_certificate.geometry_derivation_occupancy_content_fingerprint;
    if (derived_geometry_world_changed &&
        !trackingErrorTubeProfile3DMatchesWorld(
            *route.geometry->route, *route.geometry->tracking_error_tube,
            TrackingErrorTubeWorld3D{
                .observed_occupancy = &observed_raw_world->occupancy(),
                .occupied_content_fingerprint =
                    observed_raw_world->occupiedContentFingerprint(),
                .launch_support_contact =
                    observed_raw_world->launchSupportContact().has_value()
                        ? &*observed_raw_world->launchSupportContact()
                        : nullptr,
                .proprioceptive_free_space_seed =
                    observed_raw_world->proprioceptiveFreeSpaceSeed().has_value()
                        ? &*observed_raw_world->proprioceptiveFreeSpaceSeed()
                        : nullptr,
            })) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kExecutionAssessmentRejected);
    }
    if (!route.geometry->constrained_spans->empty() &&
        (!sameFootprintConfig(route.decorations->passage_volume_config.footprint,
                              observation.footprint) ||
         (derived_geometry_world_changed &&
          !canonicalPassageGeometryMatchesObservedWorld(
              *route.geometry, *route.decorations, *observed_raw_world,
              route.decorations->passage_volume_config)))) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kInvalidCandidate,
          ExecutionRouteTransitionDetail3D::kProgressPassageGeometryChanged);
    }
  } else {
    if (observed_raw_world != nullptr) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kInvalidCandidate,
          ExecutionRouteTransitionDetail3D::kProgressUnexpectedObservedWorld);
    }
    observation.latest_raw_occupancy = nullptr;
    observation.latest_raw_producer_instance_id = 0U;
    observation.latest_raw_revision = 0U;
    observation.launch_support_contact = nullptr;
    observation.proprioceptive_free_space_seed = nullptr;
    if (validationPolicyFingerprint(observation.footprint, nullptr) !=
        old_certificate.validation_policy_fingerprint) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kInvalidCandidate,
          ExecutionRouteTransitionDetail3D::kProgressValidationPolicyChanged);
    }
  }
  observation.previously_validated_through_raw_revision =
      old_certificate.validated_through_revision;
  observation.minimum_station_m = route.progress.station_m;
  const double observed_travel_m =
      distance3D(route.progress.last_observed_position, observation.position);
  if (!std::isfinite(observed_travel_m)) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kInvalidCandidate,
        ExecutionRouteTransitionDetail3D::kProgressObservedTravelInvalid);
  }
  observation.maximum_station_m = std::min(
      route.endStationM(), route.progress.station_m +
                               kMaximumStationCreditPerTravel * observed_travel_m +
                               kStationToleranceM);
  const std::array<MotionState3D, 2U> observed_path{
      route.progress.execution_input->state(), execution_input->state()};
  const bool acquiring_certified_tracking_tube =
      route.validation_policy->routeTrackingTubeConstraintsEnabled() &&
      certifiedTrackingTubeHandoffPending(current, route);
  if (acquiring_certified_tracking_tube) {
    const TrackingErrorTubeHandoffAssessment3D handoff =
        assessCertifiedTrackingTubeHandoff3D(current, route, *execution_input);
    if (!handoff.active() &&
        handoff.status !=
            TrackingErrorTubeHandoffStatus3D::kReferenceAcquiredRouteTube) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kExecutionAssessmentRejected);
    }
  }
  const std::optional<double> cross_track_limit =
      route.validation_policy->routeCrossTrackConstraintsEnabled()
          ? std::optional<double>{observation.maximum_cross_track_m}
          : std::nullopt;
  const RouteAdherenceAssessment3D observed_adherence = validateFiniteRouteAdherence(
      *route.geometry, *route.decorations, observed_path, route.progress.station_m,
      old_certificate.suffix_start_station_m, old_certificate.certified_end_station_m,
      cross_track_limit, cross_track_limit, observation.footprint.sweep_step_m,
      acquiring_certified_tracking_tube,
      route.validation_policy->routeTrackingTubeConstraintsEnabled());
  if (!observed_adherence.accepted) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kExecutionAssessmentRejected);
  }
  // The body stands upright: it is the body at every tilt the dynamics reach.
  constexpr FootprintBodyAxis previous_route_axis{};
  constexpr FootprintBodyAxis current_execution_axis{};
  const OccupiedCollisionOracle3D collision_oracle{OccupiedCollisionWorld3D{
      .observed_occupancy = old_certificate.observed_raw
                                ? std::addressof(observed_raw_world->occupancy())
                                : nullptr,
      .static_occupancy = !old_certificate.observed_raw
                              ? std::addressof(route.static_world->occupancy())
                              : nullptr,
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = observation.launch_support_contact,
      .proprioceptive_free_space_seed = observation.proprioceptive_free_space_seed,
      .footprint = observation.footprint,
      .flight_envelope = route.validation_policy->flightEnvelope(),
  }};
  if (!collision_oracle
           .validateSegment(route.progress.last_observed_position, previous_route_axis,
                            observation.position, current_execution_axis)
           .clear()) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kExecutionAssessmentRejected);
  }
  const RouteExecutionAssessment3D assessment =
      assessRouteExecution3D(&route.identity, *route.geometry->route, observation);
  if (!assessment.usable()) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kExecutionAssessmentRejected);
  }
  if (!assessment.projection.valid || !std::isfinite(assessment.projection.station_m) ||
      assessment.projection.station_m + kStationToleranceM < route.progress.station_m ||
      assessment.projection.station_m > route.endStationM() + kStationToleranceM ||
      !nearlyEqual(assessment.projection.station_m, observed_adherence.stop.station_m,
                   kStationToleranceM)) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kNonMonotonicProgress);
  }

  ExecutionPlan3D next = current;
  CertifiedRouteSuffix3D* const advanced_route = routePointer(next);
  if (advanced_route == nullptr) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCurrentSnapshot);
  }
  CertifiedRouteSuffix3D& advanced = *advanced_route;
  bindProgressToExecutionInput(
      advanced.progress, execution_input,
      std::max(route.progress.station_m, assessment.projection.station_m));
  if (old_certificate.observed_raw && assessment.raw_validation.suffix_validated) {
    // Only older raw evidence regresses the certificate. A validation that
    // starts at an earlier station than the certified suffix covers more of
    // the route on the current evidence, not less: a hovering vehicle drifts
    // a few centimetres back along its route, and refusing every transition
    // until it drifts forward again would hold it there indefinitely.
    if (assessment.validated_through_raw_revision <
        old_certificate.validated_through_revision) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kCertificateRegression,
          ExecutionRouteTransitionDetail3D::kProgressRawRevisionOlder);
    }
    ObservedRawRouteCertificate3D& renewed =
        std::get<ObservedRawRouteCertificate3D>(advanced.certificate);
    renewed.validated_through_revision = assessment.validated_through_raw_revision;
    renewed.observed_world_content_fingerprint =
        observed_raw_world->contentFingerprint();
    renewed.geometry_derivation_occupancy_content_fingerprint =
        observed_raw_world->occupiedContentFingerprint();
    renewed.suffix_start_station_m = assessment.raw_validation.validated_from_station_m;
    advanced.observed_raw_world = std::move(observed_raw_world);
  }
  if (FiniteExecutionState3D* const finite_execution = finiteExecutionPointer(next)) {
    finite_execution->revalidation_required = true;
  }
  if (FiniteExecutionState3D* const braking_fallback = brakingFallbackPointer(next)) {
    braking_fallback->revalidation_required = true;
  }
  ++next.version;
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D
applyReplaceFiniteExecutionPlanCommand3D(const ExecutionPlan3D& current,
                                         const ExecutionRouteTransitionGuard3D& guard,
                                         FiniteExecutionPlan3D execution) {
  const ExecutionRouteTransitionStatus3D guard_status = checkGuard(current, guard);
  if (guard_status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(guard_status);
  }
  const CertifiedRouteSuffix3D* const current_route = routePointer(current);
  if (current_route == nullptr) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCurrentSnapshot);
  }
  if (execution.command_horizon.execution_input == nullptr ||
      current_route->progress.execution_input == nullptr ||
      executionInputProgressRelation(*execution.command_horizon.execution_input,
                                     *current_route->progress.execution_input) ==
          ExecutionInputProgressRelation3D::kInvalid ||
      current.phase() == ExecutionRoutePhase3D::kStopped ||
      finiteExecutionValidatedAgainstNewerRawWorld(execution.command_horizon) ||
      (current.phase() == ExecutionRoutePhase3D::kAwaitingSuccessor &&
       execution.command_horizon.kind != FiniteExecutionKind3D::kNominal)) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }

  ExecutionPlan3D next = current;
  CertifiedRouteSuffix3D* const rebound_route = routePointer(next);
  if (rebound_route == nullptr) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCurrentSnapshot);
  }
  bindProgressToExecutionInput(
      rebound_route->progress, execution.command_horizon.execution_input,
      std::max(current_route->progress.station_m,
               execution.command_horizon.begin_route_station_m));
  if (!execution.validFor(*rebound_route) ||
      !candidateFiniteExecutionValid(execution.command_horizon, current, rebound_route,
                                     true)) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  ++next.version;
  CertifiedRouteSuffix3D rebound = std::move(*rebound_route);
  if (const auto* const awaiting = std::get_if<AwaitingSuccessorPlan3D>(&current.state);
      awaiting != nullptr &&
      std::holds_alternative<ContinuationStopPlan3D>(awaiting->owner)) {
    next.state = AwaitingSuccessorPlan3D{
        .owner =
            ContinuationStopPlan3D{
                .route = std::move(rebound),
                .execution = std::move(execution),
            },
    };
  } else {
    next.state = FollowingPlan3D{
        .route = std::move(rebound),
        .execution = std::move(execution),
    };
  }
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D
applyCompleteCertifiedRouteCommand3D(const ExecutionPlan3D& current,
                                     const ExecutionRouteTransitionGuard3D& guard,
                                     const RouteLifecycleEvent3D& event) {
  const ExecutionRouteTransitionStatus3D guard_status = checkGuard(current, guard);
  if (guard_status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(guard_status);
  }
  const CertifiedRouteSuffix3D* const current_route = routePointer(current);
  if (current_route == nullptr) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCurrentSnapshot);
  }
  if (event.generation != current_route->identity.generation) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kRouteGenerationMismatch);
  }
  if (!knownRouteLifecycleEventKind(event.kind)) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate,
                             ExecutionRouteTransitionDetail3D::kLifecycleEventUnknown);
  }
  // Only completion retires a route in place. Every other lifecycle event says
  // the vehicle should no longer be executing this path, which is answered by
  // a stop derived from the vehicle, not by a transition of the route.
  if (event.kind != RouteLifecycleEventKind3D::kCompleted) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kNoChange);
  }
  if (current.phase() == ExecutionRoutePhase3D::kAwaitingSuccessor ||
      current.phase() == ExecutionRoutePhase3D::kStopped) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kNoChange);
  }
  if (current.phase() != ExecutionRoutePhase3D::kFollowing ||
      current_route->remainingM() > kCompletionStationToleranceM ||
      (current.finiteExecution() != nullptr &&
       (current.finiteExecution()->kind != FiniteExecutionKind3D::kNominal ||
        current.finiteExecution()->revalidation_required))) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  if (current_route->planned_endpoint_semantics !=
          RouteEndpointSemantics3D::kContinuation &&
      (current.finiteExecution() == nullptr ||
       current.finiteExecution()->stop_boundary.station_m +
               kCompletionStationToleranceM <
           current_route->endStationM())) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  if (current.version == std::numeric_limits<std::uint64_t>::max()) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kVersionExhausted);
  }
  ExecutionPlan3D next = current;
  CertifiedRouteSuffix3D* const completed_route = routePointer(next);
  const FiniteExecutionState3D* const completed_execution = next.finiteExecution();
  const FiniteExecutionState3D* const completed_braking = next.brakingFallback();
  if (completed_route == nullptr || completed_execution == nullptr ||
      completed_braking == nullptr) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  CertifiedRouteSuffix3D route = std::move(*completed_route);
  FiniteExecutionPlan3D execution{
      .command_horizon = *completed_execution,
      .braking_tail = *completed_braking,
  };
  if (route.planned_endpoint_semantics == RouteEndpointSemantics3D::kContinuation) {
    next.state = AwaitingSuccessorPlan3D{
        .owner =
            ContinuationStopPlan3D{
                .route = std::move(route),
                .execution = std::move(execution),
            },
    };
  } else {
    next.state = StationaryHoldPlan3D{
        .owner =
            CertifiedTerminalHoldPlan3D{
                .route = std::move(route),
                .execution = std::move(execution),
            },
    };
  }
  ++next.version;
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D applyReplaceCertifiedRouteCommand3D(
    const ExecutionPlan3D& current, const ExecutionRouteTransitionGuard3D& guard,
    CertifiedRouteSuffix3D successor, FiniteExecutionPlan3D successor_execution,
    const CertifiedRouteSplice3D& splice) {
  return replaceCertifiedRouteImpl(current, guard, std::move(successor),
                                   std::move(successor_execution),
                                   std::addressof(splice));
}

ExecutionRouteTransitionResult3D applyReplaceCertifiedRouteAtHandoffCommand3D(
    const ExecutionPlan3D& current, const ExecutionRouteTransitionGuard3D& guard,
    CertifiedRouteSuffix3D successor, FiniteExecutionPlan3D successor_execution) {
  return replaceCertifiedRouteImpl(current, guard, std::move(successor),
                                   std::move(successor_execution), nullptr);
}

ExecutionRouteTransitionResult3D applyTransferToDirectTrackingCommand3D(
    const ExecutionPlan3D& current, const std::uint64_t expected_snapshot_version,
    DirectTrackingFiniteExecution3D direct_execution) {
  const ExecutionRouteTransitionStatus3D status =
      checkCurrentAndVersion(current, expected_snapshot_version);
  if (status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(status);
  }
  const bool route_owner = current.finiteExecution() != nullptr;
  const bool stationary_owner = current.stationaryHold() != nullptr;
  const bool empty_owner =
      (current.phase() == ExecutionRoutePhase3D::kAwaitingSuccessor ||
       current.phase() == ExecutionRoutePhase3D::kRevoked) &&
      current.route() == nullptr && current.finiteExecution() == nullptr &&
      current.directTrackingExecution() == nullptr &&
      current.stationaryHold() == nullptr;
  if (current.phase() == ExecutionRoutePhase3D::kDirectTracking ||
      current.execution_owner_epoch == std::numeric_limits<std::uint64_t>::max() ||
      !direct_execution.valid() ||
      direct_execution.source_snapshot_version != current.version ||
      (!route_owner && !stationary_owner && !empty_owner) ||
      (route_owner && !directTrackingEvidenceNotOlderThanRoute(
                          direct_execution, *current.finiteExecution())) ||
      (stationary_owner && !directTrackingEvidenceNotOlderThanHold(
                               direct_execution, *current.stationaryHold()))) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  ExecutionPlan3D next = current;
  ++next.version;
  next.route_generation_high_water = current.routeGenerationHighWater();
  next.state = DirectTrackingPlan3D{.execution = std::move(direct_execution)};
  ++next.execution_owner_epoch;
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D applyReplaceDirectTrackingExecutionCommand3D(
    const ExecutionPlan3D& current, const std::uint64_t expected_snapshot_version,
    DirectTrackingFiniteExecution3D direct_execution) {
  const ExecutionRouteTransitionStatus3D status =
      checkCurrentAndVersion(current, expected_snapshot_version);
  if (status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(status);
  }
  if (current.phase() != ExecutionRoutePhase3D::kDirectTracking ||
      current.directTrackingExecution() == nullptr || !direct_execution.valid() ||
      direct_execution.source_snapshot_version != current.version ||
      !directTrackingExecutionNotOlder(direct_execution,
                                       *current.directTrackingExecution())) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  ExecutionPlan3D next = current;
  ++next.version;
  next.state = DirectTrackingPlan3D{.execution = std::move(direct_execution)};
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D applyTransferDirectTrackingToCertifiedRouteCommand3D(
    const ExecutionPlan3D& current, const std::uint64_t expected_snapshot_version,
    CertifiedRouteSuffix3D successor, FiniteExecutionPlan3D successor_execution) {
  const ExecutionRouteTransitionStatus3D status =
      checkCurrentAndVersion(current, expected_snapshot_version);
  if (status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(status);
  }
  if (current.phase() != ExecutionRoutePhase3D::kDirectTracking ||
      current.directTrackingExecution() == nullptr || !successor.valid() ||
      current.routeGenerationHighWater() == std::numeric_limits<std::uint64_t>::max() ||
      current.execution_owner_epoch == std::numeric_limits<std::uint64_t>::max() ||
      successor.identity.generation != current.routeGenerationHighWater() + 1U ||
      successor_execution.command_horizon.kind != FiniteExecutionKind3D::kNominal ||
      successor_execution.command_horizon.execution_input == nullptr ||
      !candidateFiniteExecutionValid(successor_execution.command_horizon, current,
                                     &successor, true) ||
      !routeExecutionEvidenceNotOlderThanDirect(successor_execution.command_horizon,
                                                *current.directTrackingExecution())) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  bindProgressToExecutionInput(
      successor.progress, successor_execution.command_horizon.execution_input,
      successor_execution.command_horizon.begin_route_station_m);
  if (!successor.valid() || !successor_execution.validFor(successor) ||
      !candidateFiniteExecutionValid(successor_execution.command_horizon, current,
                                     &successor, true)) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  ExecutionPlan3D next = current;
  ++next.version;
  const std::uint64_t successor_generation = successor.identity.generation;
  next.state = FollowingPlan3D{
      .route = std::move(successor),
      .execution = std::move(successor_execution),
  };
  ++next.execution_owner_epoch;
  next.route_generation_high_water = successor_generation;
  return finishTransition(current, std::move(next));
}

} // namespace drone_city_nav::execution_route_snapshot_3d_internal
