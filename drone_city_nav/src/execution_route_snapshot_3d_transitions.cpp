#include "drone_city_nav/certified_route_splice_3d.hpp"
#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/mppi/mppi_altitude_envelope.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"

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

namespace drone_city_nav {

using namespace execution_route_snapshot_3d_internal;

namespace {

[[nodiscard]] ExecutionRouteTransitionResult3D
replaceCertifiedRouteImpl(const ExecutionRouteSnapshot3D& current,
                          const ExecutionRouteTransitionGuard3D& guard,
                          CertifiedRouteSuffix3D successor,
                          FiniteExecutionPlan3D successor_execution,
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
      (splice == nullptr && same_active_intent) ||
      (!same_active_intent && successor.owner.id == current_route->owner.id)) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  }
  if (same_active_intent) {
    successor.owner = current_route->owner;
  }
  const bool route_owner_changes = successor.owner.id != current_route->owner.id;
  if (!successor.valid() ||
      current_route->identity.generation == std::numeric_limits<std::uint64_t>::max() ||
      current.execution_owner_epoch == std::numeric_limits<std::uint64_t>::max() ||
      successor.identity.generation != current_route->identity.generation + 1U ||
      distance3D(successor.progress.last_observed_position,
                 current_route->progress.last_observed_position) >
          kExecutionBindingToleranceM) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  }
  const bool replacement_phase_allowed =
      current.phase == ExecutionRoutePhase3D::kFollowing ||
      current.phase == ExecutionRoutePhase3D::kAwaitingSuccessor ||
      current.phase == ExecutionRoutePhase3D::kBraking ||
      (current.phase == ExecutionRoutePhase3D::kStopped &&
       current_route->planned_endpoint_semantics ==
           RouteEndpointSemantics3D::kObservationStop);
  if (!replacement_phase_allowed || !current.finite_execution.has_value() ||
      successor_execution.command_horizon.execution_input == nullptr) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  if (splice != nullptr) {
    const mppi::State& splice_state =
        successor_execution.command_horizon.execution_input->state();
    const RouteSpliceReadiness3D splice_readiness = assessRouteSpliceReadiness3D(
        *splice, *current_route, successor,
        Point3{splice_state.x, splice_state.y, splice_state.z});
    if (!splice_readiness.ready()) {
      return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
    }
  }
  if (successor_execution.command_horizon.kind != FiniteExecutionKind3D::kNominal ||
      !successorEvidenceNotOlder(*current_route, *current.finite_execution, successor,
                                 successor_execution.command_horizon)) {
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
  ExecutionRouteSnapshot3D next = current;
  ++next.version;
  next.phase = ExecutionRoutePhase3D::kFollowing;
  next.route = std::move(successor);
  next.finite_execution = std::move(successor_execution.command_horizon);
  next.braking_fallback = std::move(successor_execution.braking_tail);
  next.direct_tracking_execution.reset();
  next.stationary_hold.reset();
  if (route_owner_changes) {
    ++next.execution_owner_epoch;
  }
  next.route_generation_high_water = next.route->identity.generation;
  return finishTransition(current, std::move(next));
}

} // namespace

ExecutionRouteTransitionResult3D
activateCertifiedRoute3D(const ExecutionRouteSnapshot3D& current,
                         const std::uint64_t expected_snapshot_version,
                         CertifiedRouteSuffix3D candidate,
                         FiniteExecutionPlan3D candidate_execution) {
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
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  }
  const bool empty_owner =
      (current.phase == ExecutionRoutePhase3D::kAwaitingSuccessor ||
       current.phase == ExecutionRoutePhase3D::kRevoked) &&
      !current.route.has_value() && !current.finite_execution.has_value() &&
      !current.braking_fallback.has_value() &&
      !current.direct_tracking_execution.has_value() &&
      !current.stationary_hold.has_value();
  const bool stationary_owner =
      current.phase == ExecutionRoutePhase3D::kStopped && !current.route.has_value() &&
      !current.finite_execution.has_value() && !current.braking_fallback.has_value() &&
      !current.direct_tracking_execution.has_value() &&
      current.stationary_hold.has_value();
  if ((!empty_owner && !stationary_owner) ||
      (stationary_owner &&
       !routeExecutionEvidenceNotOlderThanHold(candidate_execution.command_horizon,
                                               *current.stationary_hold))) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  bindProgressToExecutionInput(
      candidate.progress, candidate_execution.command_horizon.execution_input,
      candidate_execution.command_horizon.begin_route_station_m);
  if (!candidate.valid() || !candidate_execution.validFor(candidate) ||
      !candidateFiniteExecutionValid(candidate_execution.command_horizon, current,
                                     &candidate, true)) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  }
  ExecutionRouteSnapshot3D next = current;
  ++next.version;
  next.phase = ExecutionRoutePhase3D::kFollowing;
  next.route = std::move(candidate);
  next.finite_execution = std::move(candidate_execution.command_horizon);
  next.braking_fallback = std::move(candidate_execution.braking_tail);
  next.direct_tracking_execution.reset();
  next.stationary_hold.reset();
  ++next.execution_owner_epoch;
  next.route_generation_high_water = next.route->identity.generation;
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D advanceCertifiedRoute3D(
    const ExecutionRouteSnapshot3D& current,
    const ExecutionRouteTransitionGuard3D& guard,
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
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
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
      return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
    }
    observation.latest_raw_occupancy = &observed_raw_world->occupancy();
    observation.latest_raw_producer_instance_id =
        observed_raw_world->version().producer_instance_id;
    observation.latest_raw_revision = observed_raw_world->version().revision;
    observation.proprioceptive_free_space_seed =
        observed_raw_world->proprioceptiveFreeSpaceSeed().has_value()
            ? &*observed_raw_world->proprioceptiveFreeSpaceSeed()
            : nullptr;
    observation.launch_support_contact =
        observed_raw_world->launchSupportContact().has_value()
            ? &*observed_raw_world->launchSupportContact()
            : nullptr;
    if (validationPolicyFingerprint(observation.footprint,
                                    ObservedSpaceValidationPolicy::kAllowUnknown,
                                    observation.proprioceptive_free_space_seed,
                                    observation.launch_support_contact) !=
        old_certificate.validation_policy_fingerprint) {
      return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
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
                .free_space_seed =
                    observed_raw_world->proprioceptiveFreeSpaceSeed().has_value()
                        ? &*observed_raw_world->proprioceptiveFreeSpaceSeed()
                        : nullptr,
                .launch_support_contact =
                    observed_raw_world->launchSupportContact().has_value()
                        ? &*observed_raw_world->launchSupportContact()
                        : nullptr,
            })) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kExecutionAssessmentRejected);
    }
    if (!route.geometry->constrained_spans->empty() &&
        (!sameFootprintConfig(route.geometry->passage_volume_config.footprint,
                              observation.footprint) ||
         (derived_geometry_world_changed &&
          !canonicalPassageGeometryMatchesObservedWorld(
              *route.geometry, *observed_raw_world,
              route.geometry->passage_volume_config)))) {
      return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
    }
  } else {
    if (observed_raw_world != nullptr) {
      return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
    }
    observation.latest_raw_occupancy = nullptr;
    observation.latest_raw_producer_instance_id = 0U;
    observation.latest_raw_revision = 0U;
    observation.proprioceptive_free_space_seed = nullptr;
    observation.launch_support_contact = nullptr;
    if (validationPolicyFingerprint(
            observation.footprint, ObservedSpaceValidationPolicy::kRequireKnownFree,
            nullptr, nullptr) != old_certificate.validation_policy_fingerprint) {
      return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
    }
  }
  observation.previously_validated_through_raw_revision =
      old_certificate.validated_through_revision;
  observation.minimum_station_m = route.progress.station_m;
  const double observed_travel_m =
      distance3D(route.progress.last_observed_position, observation.position);
  if (!std::isfinite(observed_travel_m)) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  }
  observation.maximum_station_m = std::min(
      route.endStationM(), route.progress.station_m +
                               kMaximumStationCreditPerTravel * observed_travel_m +
                               kStationToleranceM);
  const std::array<mppi::State, 2U> observed_path{
      route.progress.execution_input->state(), execution_input->state()};
  const bool acquiring_certified_tracking_tube =
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
  const RouteAdherenceAssessment3D observed_adherence = validateFiniteRouteAdherence(
      *route.geometry, observed_path, route.progress.station_m,
      old_certificate.suffix_start_station_m, old_certificate.certified_end_station_m,
      observation.maximum_cross_track_m, observation.maximum_cross_track_m,
      observation.footprint.sweep_step_m, acquiring_certified_tracking_tube);
  if (!observed_adherence.accepted) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kExecutionAssessmentRejected);
  }
  const mppi::Control& previous_route_control =
      route.progress.execution_input->previousControl();
  const mppi::Control& current_execution_control = execution_input->previousControl();
  const FootprintBodyAxis previous_route_axis = bodyAxisFromWorldAcceleration(Vec3{
      previous_route_control.ax, previous_route_control.ay, previous_route_control.az});
  const FootprintBodyAxis current_execution_axis = bodyAxisFromWorldAcceleration(
      Vec3{current_execution_control.ax, current_execution_control.ay,
           current_execution_control.az});
  const bool observed_segment_world_safe =
      old_certificate.observed_raw
          ? validateObservedSweptFootprint(
                observed_raw_world->occupancy(), route.progress.last_observed_position,
                previous_route_axis, observation.position, current_execution_axis,
                observation.footprint, ObservedSpaceValidationPolicy::kAllowUnknown,
                observation.proprioceptive_free_space_seed,
                observation.launch_support_contact)
                .accepted()
          : validateKnownStaticSweptFootprint(
                route.static_world->occupancy(), route.progress.last_observed_position,
                previous_route_axis, observation.position, current_execution_axis,
                observation.footprint)
                .accepted();
  if (!observed_segment_world_safe) {
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

  ExecutionRouteSnapshot3D next = current;
  CertifiedRouteSuffix3D* const advanced_route = routePointer(next);
  if (advanced_route == nullptr) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCurrentSnapshot);
  }
  CertifiedRouteSuffix3D& advanced = *advanced_route;
  bindProgressToExecutionInput(
      advanced.progress, execution_input,
      std::max(route.progress.station_m, assessment.projection.station_m));
  if (old_certificate.observed_raw && assessment.raw_validation.suffix_validated) {
    if (assessment.validated_through_raw_revision <
            old_certificate.validated_through_revision ||
        assessment.raw_validation.validated_from_station_m + kStationToleranceM <
            old_certificate.suffix_start_station_m) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kCertificateRegression);
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
  if (next.finite_execution.has_value()) {
    next.finite_execution->revalidation_required = true;
  }
  if (next.braking_fallback.has_value()) {
    next.braking_fallback->revalidation_required = true;
  }
  ++next.version;
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D
replaceFiniteExecutionPlan3D(const ExecutionRouteSnapshot3D& current,
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
      current.phase == ExecutionRoutePhase3D::kStopped ||
      finiteExecutionValidatedAgainstNewerRawWorld(execution.command_horizon) ||
      (current.phase == ExecutionRoutePhase3D::kAwaitingSuccessor &&
       execution.command_horizon.kind != FiniteExecutionKind3D::kNominal) ||
      (current.phase == ExecutionRoutePhase3D::kBraking &&
       execution.command_horizon.kind == FiniteExecutionKind3D::kNominal)) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }

  ExecutionRouteSnapshot3D next = current;
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
  next.finite_execution = std::move(execution.command_horizon);
  next.braking_fallback = std::move(execution.braking_tail);
  next.stationary_hold.reset();
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D
retireCertifiedRoute3D(const ExecutionRouteSnapshot3D& current,
                       const ExecutionRouteTransitionGuard3D& guard,
                       const RouteLifecycleEvent3D& event,
                       std::optional<FiniteExecutionState3D> retained_safe_execution) {
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
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  }
  if (event.kind == RouteLifecycleEventKind3D::kControlCandidateRejected) {
    if (retained_safe_execution.has_value()) {
      return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
    }
    return transitionFailure(ExecutionRouteTransitionStatus3D::kNoChange);
  }

  if (event.kind == RouteLifecycleEventKind3D::kCompleted) {
    if (current.phase == ExecutionRoutePhase3D::kAwaitingSuccessor ||
        current.phase == ExecutionRoutePhase3D::kStopped) {
      return transitionFailure(ExecutionRouteTransitionStatus3D::kNoChange);
    }
    if (current.phase != ExecutionRoutePhase3D::kFollowing ||
        current_route->remainingM() > kCompletionStationToleranceM ||
        (current.finite_execution.has_value() &&
         (current.finite_execution->kind != FiniteExecutionKind3D::kNominal ||
          current.finite_execution->revalidation_required))) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
    }
    if (current_route->planned_endpoint_semantics !=
            RouteEndpointSemantics3D::kContinuation &&
        (!current.finite_execution.has_value() ||
         current.finite_execution->stop_boundary.station_m +
                 kCompletionStationToleranceM <
             current_route->endStationM())) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
    }
  }
  if (current.phase == ExecutionRoutePhase3D::kStopped) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kNoChange);
  }
  if (current.phase == ExecutionRoutePhase3D::kBraking &&
      !retained_safe_execution.has_value() &&
      (event.kind == RouteLifecycleEventKind3D::kObjectiveSuperseded ||
       event.kind == RouteLifecycleEventKind3D::kCrossTrackExceeded ||
       event.kind == RouteLifecycleEventKind3D::kTrackingTubeExceeded)) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kNoChange);
  }
  if (!retained_safe_execution.has_value() &&
      (event.kind == RouteLifecycleEventKind3D::kObjectiveSuperseded ||
       event.kind == RouteLifecycleEventKind3D::kCrossTrackExceeded ||
       event.kind == RouteLifecycleEventKind3D::kTrackingTubeExceeded)) {
    retained_safe_execution = current.braking_fallback;
  }

  const auto* const current_raw_certificate =
      std::get_if<ObservedRawRouteCertificate3D>(&current_route->certificate);
  if (event.kind == RouteLifecycleEventKind3D::kRawInvalidated) {
    if (current_raw_certificate == nullptr || event.raw_producer_instance_id == 0U ||
        event.raw_revision == 0U ||
        event.raw_producer_instance_id !=
            current_raw_certificate->producer_instance_id) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
    }
    if (event.raw_revision <= current_raw_certificate->validated_through_revision) {
      return transitionFailure(ExecutionRouteTransitionStatus3D::kNoChange);
    }
    if (current.phase == ExecutionRoutePhase3D::kBraking &&
        !retained_safe_execution.has_value()) {
      const auto* const existing_raw_lineage =
          current.finite_execution.has_value()
              ? std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(
                    &current.finite_execution->validation_proof.lineage)
              : nullptr;
      if (existing_raw_lineage != nullptr &&
          existing_raw_lineage->producer_instance_id ==
              event.raw_producer_instance_id &&
          existing_raw_lineage->validated_through_raw_revision >= event.raw_revision) {
        return transitionFailure(ExecutionRouteTransitionStatus3D::kNoChange);
      }
    }
  }

  const bool has_retained_safe_execution = retained_safe_execution.has_value();
  if (event.kind != RouteLifecycleEventKind3D::kRawInvalidated &&
      has_retained_safe_execution &&
      finiteExecutionValidatedAgainstNewerRawWorld(*retained_safe_execution)) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  if (event.kind == RouteLifecycleEventKind3D::kCompleted &&
      current_route->planned_endpoint_semantics ==
          RouteEndpointSemantics3D::kContinuation &&
      has_retained_safe_execution) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  }

  ExecutionRouteSnapshot3D next = current;
  if (retained_safe_execution.has_value()) {
    CertifiedRouteSuffix3D* const rebound_route = routePointer(next);
    if (rebound_route == nullptr) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
    }
    if (event.kind == RouteLifecycleEventKind3D::kRawInvalidated) {
      if (current_route->progress.execution_input == nullptr ||
          retained_safe_execution->execution_input == nullptr ||
          executionInputProgressRelation(*retained_safe_execution->execution_input,
                                         *current_route->progress.execution_input) !=
              ExecutionInputProgressRelation3D::kStrictlyNewer ||
          retained_safe_execution->horizon == nullptr ||
          retained_safe_execution->horizon->states.empty() ||
          !rawInvalidationProofMatchesEvent(*retained_safe_execution, event) ||
          retained_safe_execution->begin_route_station_m + kStationToleranceM <
              current_route->progress.station_m ||
          retained_safe_execution->begin_route_station_m >
              current_route->endStationM() + kStationToleranceM) {
        return transitionFailure(
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
      }
      const Point3 rebound_position =
          executionInputPosition(*retained_safe_execution->execution_input);
      const RouteProjection3D rebound_projection =
          projectOntoRoute3DWithinStationWindow(
              *current_route->geometry->route, rebound_position,
              std::max(
                  certificateView(current_route->certificate).suffix_start_station_m,
                  retained_safe_execution->begin_route_station_m -
                      kExecutionBindingToleranceM),
              std::min(current_route->endStationM(),
                       retained_safe_execution->begin_route_station_m +
                           kExecutionBindingToleranceM));
      if (!rebound_projection.valid ||
          rebound_projection.distance_m > kExecutionBindingToleranceM ||
          std::abs(rebound_projection.station_m -
                   retained_safe_execution->begin_route_station_m) >
              kExecutionBindingToleranceM) {
        return transitionFailure(
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
      }
      bindProgressToExecutionInput(rebound_route->progress,
                                   retained_safe_execution->execution_input,
                                   retained_safe_execution->begin_route_station_m);
    } else {
      if (current_route->progress.execution_input == nullptr ||
          retained_safe_execution->execution_input == nullptr ||
          executionInputProgressRelation(*retained_safe_execution->execution_input,
                                         *current_route->progress.execution_input) ==
              ExecutionInputProgressRelation3D::kInvalid) {
        return transitionFailure(
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
      }
      bindProgressToExecutionInput(
          rebound_route->progress, retained_safe_execution->execution_input,
          std::max(current_route->progress.station_m,
                   retained_safe_execution->begin_route_station_m));
    }
    const bool activates_resident_fallback =
        current.braking_fallback.has_value() &&
        finiteExecutionArtifactFingerprint(*retained_safe_execution) ==
            finiteExecutionArtifactFingerprint(*current.braking_fallback);
    if (retained_safe_execution->kind != FiniteExecutionKind3D::kEmergencyBrakeTail ||
        (!activates_resident_fallback &&
         !candidateFiniteExecutionValid(*retained_safe_execution, current,
                                        rebound_route, true))) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
    }
    next.finite_execution = *retained_safe_execution;
    next.braking_fallback = std::move(retained_safe_execution);
    next.stationary_hold.reset();
  }

  switch (event.kind) {
    case RouteLifecycleEventKind3D::kCompleted:
      if (next.route->planned_endpoint_semantics ==
          RouteEndpointSemantics3D::kContinuation) {
        next.phase = ExecutionRoutePhase3D::kAwaitingSuccessor;
      } else {
        if (!next.finite_execution.has_value()) {
          return transitionFailure(
              ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
        }
        next.phase = ExecutionRoutePhase3D::kStopped;
      }
      break;
    case RouteLifecycleEventKind3D::kRawInvalidated: {
      const auto* const retained_raw_lineage =
          next.finite_execution.has_value()
              ? std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(
                    &next.finite_execution->validation_proof.lineage)
              : nullptr;
      if (!next.finite_execution.has_value() || retained_raw_lineage == nullptr ||
          retained_raw_lineage->producer_instance_id !=
              event.raw_producer_instance_id ||
          retained_raw_lineage->validated_through_raw_revision != event.raw_revision ||
          next.finite_execution->kind != FiniteExecutionKind3D::kEmergencyBrakeTail) {
        return transitionFailure(
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
      }
      next.phase = ExecutionRoutePhase3D::kBraking;
      break;
    }
    case RouteLifecycleEventKind3D::kObjectiveSuperseded:
    case RouteLifecycleEventKind3D::kCrossTrackExceeded:
    case RouteLifecycleEventKind3D::kTrackingTubeExceeded:
      if (!next.finite_execution.has_value() ||
          next.finite_execution->kind == FiniteExecutionKind3D::kNominal) {
        return transitionFailure(
            ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
      }
      next.phase = ExecutionRoutePhase3D::kBraking;
      break;
    case RouteLifecycleEventKind3D::kControlCandidateRejected:
      return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  }
  ++next.version;
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D replaceCertifiedRoute3D(
    const ExecutionRouteSnapshot3D& current,
    const ExecutionRouteTransitionGuard3D& guard, CertifiedRouteSuffix3D successor,
    FiniteExecutionPlan3D successor_execution, const CertifiedRouteSplice3D& splice) {
  return replaceCertifiedRouteImpl(current, guard, std::move(successor),
                                   std::move(successor_execution),
                                   std::addressof(splice));
}

ExecutionRouteTransitionResult3D
replaceCertifiedRouteAtHandoff3D(const ExecutionRouteSnapshot3D& current,
                                 const ExecutionRouteTransitionGuard3D& guard,
                                 CertifiedRouteSuffix3D successor,
                                 FiniteExecutionPlan3D successor_execution) {
  return replaceCertifiedRouteImpl(current, guard, std::move(successor),
                                   std::move(successor_execution), nullptr);
}

ExecutionRouteTransitionResult3D
transferToDirectTracking3D(const ExecutionRouteSnapshot3D& current,
                           const std::uint64_t expected_snapshot_version,
                           DirectTrackingFiniteExecution3D direct_execution) {
  const ExecutionRouteTransitionStatus3D status =
      checkCurrentAndVersion(current, expected_snapshot_version);
  if (status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(status);
  }
  const bool route_owner = current.finite_execution.has_value();
  const bool stationary_owner = current.stationary_hold.has_value();
  const bool empty_owner =
      (current.phase == ExecutionRoutePhase3D::kAwaitingSuccessor ||
       current.phase == ExecutionRoutePhase3D::kRevoked) &&
      !current.route.has_value() && !current.finite_execution.has_value() &&
      !current.braking_fallback.has_value() &&
      !current.direct_tracking_execution.has_value() &&
      !current.stationary_hold.has_value();
  if (current.phase == ExecutionRoutePhase3D::kDirectTracking ||
      current.execution_owner_epoch == std::numeric_limits<std::uint64_t>::max() ||
      !direct_execution.valid() ||
      direct_execution.source_snapshot_version != current.version ||
      (!route_owner && !stationary_owner && !empty_owner) ||
      (route_owner && !directTrackingEvidenceNotOlderThanRoute(
                          direct_execution, *current.finite_execution)) ||
      (stationary_owner && !directTrackingEvidenceNotOlderThanHold(
                               direct_execution, *current.stationary_hold))) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  ExecutionRouteSnapshot3D next = current;
  ++next.version;
  next.phase = ExecutionRoutePhase3D::kDirectTracking;
  next.route_generation_high_water = current.routeGenerationHighWater();
  next.route.reset();
  next.finite_execution.reset();
  next.braking_fallback.reset();
  next.stationary_hold.reset();
  next.direct_tracking_execution = std::move(direct_execution);
  ++next.execution_owner_epoch;
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D
replaceDirectTrackingExecution3D(const ExecutionRouteSnapshot3D& current,
                                 const std::uint64_t expected_snapshot_version,
                                 DirectTrackingFiniteExecution3D direct_execution) {
  const ExecutionRouteTransitionStatus3D status =
      checkCurrentAndVersion(current, expected_snapshot_version);
  if (status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(status);
  }
  if (current.phase != ExecutionRoutePhase3D::kDirectTracking ||
      !current.direct_tracking_execution.has_value() || !direct_execution.valid() ||
      direct_execution.source_snapshot_version != current.version ||
      !directTrackingExecutionNotOlder(direct_execution,
                                       *current.direct_tracking_execution)) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  ExecutionRouteSnapshot3D next = current;
  ++next.version;
  next.direct_tracking_execution = std::move(direct_execution);
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D composeExecutionPlanTransition3D(
    const ExecutionRouteSnapshot3D& resident,
    const ExecutionRouteTransitionResult3D& prepared_progress,
    const ExecutionRouteTransitionResult3D& prepared_execution_plan) {
  if (!prepared_progress.applied() || prepared_progress.predecessor != &resident ||
      prepared_progress.next == nullptr || !prepared_execution_plan.applied() ||
      prepared_execution_plan.predecessor != prepared_progress.next.get() ||
      prepared_execution_plan.next == nullptr ||
      !prepared_execution_plan.next->publishable()) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  }
  return finishTransition(resident, *prepared_execution_plan.next);
}

ExecutionRouteTransitionResult3D
transferDirectTrackingToCertifiedRoute3D(const ExecutionRouteSnapshot3D& current,
                                         const std::uint64_t expected_snapshot_version,
                                         CertifiedRouteSuffix3D successor,
                                         FiniteExecutionPlan3D successor_execution) {
  const ExecutionRouteTransitionStatus3D status =
      checkCurrentAndVersion(current, expected_snapshot_version);
  if (status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(status);
  }
  if (current.phase != ExecutionRoutePhase3D::kDirectTracking ||
      !current.direct_tracking_execution.has_value() || !successor.valid() ||
      current.routeGenerationHighWater() == std::numeric_limits<std::uint64_t>::max() ||
      current.execution_owner_epoch == std::numeric_limits<std::uint64_t>::max() ||
      successor.identity.generation != current.routeGenerationHighWater() + 1U ||
      successor_execution.command_horizon.kind != FiniteExecutionKind3D::kNominal ||
      successor_execution.command_horizon.execution_input == nullptr ||
      !candidateFiniteExecutionValid(successor_execution.command_horizon, current,
                                     &successor, true) ||
      !routeExecutionEvidenceNotOlderThanDirect(successor_execution.command_horizon,
                                                *current.direct_tracking_execution)) {
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
  ExecutionRouteSnapshot3D next = current;
  ++next.version;
  next.phase = ExecutionRoutePhase3D::kFollowing;
  next.route = std::move(successor);
  next.finite_execution = std::move(successor_execution.command_horizon);
  next.braking_fallback = std::move(successor_execution.braking_tail);
  next.direct_tracking_execution.reset();
  next.stationary_hold.reset();
  ++next.execution_owner_epoch;
  next.route_generation_high_water = next.route->identity.generation;
  return finishTransition(current, std::move(next));
}

} // namespace drone_city_nav
