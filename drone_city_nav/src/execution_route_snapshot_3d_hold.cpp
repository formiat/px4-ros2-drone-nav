#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/mppi/mppi_altitude_envelope.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include "execution_route_snapshot_3d_internal.hpp"

namespace drone_city_nav {

using namespace execution_route_snapshot_3d_internal;

namespace execution_route_snapshot_3d_internal {

bool stationaryHoldRawSafe(
    const Point3& position, const VersionedExecutionInput3D& execution_input,
    const VersionedObservedRawWorld3D* const observed_raw_world,
    const VersionedStaticWorld3D* const static_world,
    const VersionedExecutionValidationPolicy3D& validation_policy,
    const VersionedLatestLidarEvidence3D& latest_lidar_evidence) noexcept {
  if ((observed_raw_world == nullptr) == (static_world == nullptr)) {
    return false;
  }
  const mppi::Control& control = execution_input.previousControl();
  const FootprintBodyAxis axis =
      bodyAxisFromWorldAcceleration(Vec3{control.ax, control.ay, control.az});
  SweptFootprintResult world_validation;
  if (static_world != nullptr) {
    world_validation = validateKnownStaticSweptFootprint(
        static_world->occupancy(), position, axis, position, axis,
        validation_policy.sweptFootprint());
  } else {
    world_validation = validateObservedSweptFootprint(
        observed_raw_world->occupancy(), position, axis, position, axis,
        validation_policy.sweptFootprint(),
        ObservedSpaceValidationPolicy::kAllowUnknown,
        observed_raw_world->proprioceptiveFreeSpaceSeed().has_value()
            ? std::addressof(*observed_raw_world->proprioceptiveFreeSpaceSeed())
            : nullptr,
        observed_raw_world->launchSupportContact().has_value()
            ? std::addressof(*observed_raw_world->launchSupportContact())
            : nullptr);
  }
  if (!world_validation.accepted()) {
    return false;
  }
  return validateRawPointCloudSweptFootprint(
             latest_lidar_evidence.hitPointsMapM(), position, axis, position, axis,
             validation_policy.sweptFootprint(),
             observed_raw_world != nullptr &&
                     observed_raw_world->launchSupportContact().has_value()
                 ? std::addressof(*observed_raw_world->launchSupportContact())
                 : nullptr)
      .accepted();
}

} // namespace execution_route_snapshot_3d_internal

namespace {

[[nodiscard]] bool
holdWorldNotOlder(const StationaryExecutionHoldCertification3D& certification,
                  const VersionedObservedRawWorld3D* const previous_observed,
                  const VersionedStaticWorld3D* const previous_static) noexcept {
  if ((certification.observed_raw_world == nullptr) ==
      (certification.static_world == nullptr)) {
    return false;
  }
  if (previous_observed != nullptr) {
    return certification.observed_raw_world != nullptr &&
           certification.observed_raw_world->valid() &&
           certification.observed_raw_world->version().producer_instance_id ==
               previous_observed->version().producer_instance_id &&
           certification.observed_raw_world->version().revision >=
               previous_observed->version().revision &&
           (certification.observed_raw_world->version().revision !=
                previous_observed->version().revision ||
            certification.observed_raw_world->contentFingerprint() ==
                previous_observed->contentFingerprint() ||
            certification.observed_raw_world->sharesObservationOwner(
                *previous_observed));
  }
  return previous_static != nullptr && certification.observed_raw_world == nullptr &&
         certification.static_world != nullptr && certification.static_world->valid() &&
         staticWorldNotOlder(*certification.static_world, *previous_static);
}

[[nodiscard]] bool
stationaryHoldPointSafe(const StationaryExecutionHoldCertification3D& certification,
                        const bool stationary_capture_rearm) noexcept {
  if (certification.execution_input == nullptr ||
      !certification.execution_input->valid() ||
      (stationary_capture_rearm
           ? !certification.execution_input->stationaryCaptureStateAuthoritative()
           : !certification.execution_input->nominalStateAuthoritative()) ||
      certification.validation_policy == nullptr ||
      !certification.validation_policy->valid() ||
      certification.latest_lidar_evidence == nullptr ||
      !certification.latest_lidar_evidence->valid() ||
      !executionInputFreshAt(*certification.execution_input,
                             *certification.validation_policy,
                             certification.execution_input->effectiveStampNs()) ||
      !latestLidarEvidenceFreshAt(*certification.latest_lidar_evidence,
                                  *certification.validation_policy,
                                  certification.execution_input->effectiveStampNs()) ||
      !finitePoint(certification.position)) {
    return false;
  }
  const mppi::State& state = certification.execution_input->state();
  const mppi::Control& control = certification.execution_input->previousControl();
  const Point3 actual_position{state.x, state.y, state.z};
  if (distance3D(certification.position, actual_position) >
          kStationaryExecutionHoldPositionToleranceM ||
      std::hypot(std::hypot(state.vx, state.vy), state.vz) >
          kStationaryExecutionHoldSpeedToleranceMps ||
      std::abs(state.yaw_rate) > kStationaryExecutionHoldYawRateToleranceRadps ||
      !insideFlightEnvelope(certification.position,
                            certification.validation_policy->flightEnvelope()) ||
      !mppi::altitudeEnvelopeDynamicallyRecoverable(
          state, control, certification.validation_policy->dynamics(),
          certification.validation_policy->altitudeEnvelope())) {
    return false;
  }
  return stationaryHoldRawSafe(
      certification.position, *certification.execution_input,
      certification.observed_raw_world.get(), certification.static_world.get(),
      *certification.validation_policy, *certification.latest_lidar_evidence);
}

[[nodiscard]] ExecutionRouteSnapshot3D
makeStationaryHoldSnapshot(const ExecutionRouteSnapshot3D& current,
                           StationaryExecutionHoldCertification3D certification,
                           const StationaryExecutionHoldOrigin3D origin,
                           const std::uint64_t source_trajectory_revision,
                           const std::uint64_t hold_id) {
  ExecutionRouteSnapshot3D next = current;
  ++next.version;
  next.phase = ExecutionRoutePhase3D::kStopped;
  next.route_generation_high_water = current.routeGenerationHighWater();
  next.route.reset();
  next.finite_execution.reset();
  next.braking_fallback.reset();
  next.direct_tracking_execution.reset();
  next.stationary_hold = StationaryExecutionHold3D{
      .hold_id = hold_id,
      .source_trajectory_revision = source_trajectory_revision,
      .origin = origin,
      .position = certification.position,
      .terminal_execution_input = std::move(certification.execution_input),
      .observed_raw_world = std::move(certification.observed_raw_world),
      .static_world = std::move(certification.static_world),
      .validation_policy = std::move(certification.validation_policy),
      .latest_lidar_evidence = std::move(certification.latest_lidar_evidence),
  };
  return next;
}

} // namespace

ExecutionRouteTransitionResult3D
transferToExecutionHold3D(const ExecutionRouteSnapshot3D& current,
                          const std::uint64_t expected_snapshot_version,
                          StationaryExecutionHoldCertification3D certification) {
  const ExecutionRouteTransitionStatus3D status =
      checkCurrentAndVersion(current, expected_snapshot_version);
  if (status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(status);
  }
  const FiniteExecutionState3D* const route_execution =
      current.finite_execution.has_value() ? &*current.finite_execution : nullptr;
  const DirectTrackingFiniteExecution3D* const direct_execution =
      current.direct_tracking_execution.has_value()
          ? &*current.direct_tracking_execution
          : nullptr;
  const StationaryExecutionHold3D* const resident_hold =
      current.stationary_hold.has_value() ? &*current.stationary_hold : nullptr;
  if ((route_execution != nullptr) + (direct_execution != nullptr) +
          (resident_hold != nullptr) !=
      1) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  const mppi::FiniteHorizon* const source_horizon =
      route_execution != nullptr    ? route_execution->horizon.get()
      : direct_execution != nullptr ? direct_execution->horizon.get()
                                    : nullptr;
  const std::shared_ptr<const VersionedExecutionInput3D> source_input =
      route_execution != nullptr    ? route_execution->execution_input
      : direct_execution != nullptr ? direct_execution->execution_input
                                    : resident_hold->terminal_execution_input;
  const VersionedObservedRawWorld3D* const source_observed =
      route_execution != nullptr    ? route_execution->observed_raw_world.get()
      : direct_execution != nullptr ? direct_execution->observed_raw_world.get()
                                    : resident_hold->observed_raw_world.get();
  const VersionedStaticWorld3D* const source_static =
      route_execution != nullptr    ? route_execution->static_world.get()
      : direct_execution != nullptr ? direct_execution->static_world.get()
                                    : resident_hold->static_world.get();
  const VersionedLatestLidarEvidence3D* const source_lidar =
      route_execution != nullptr    ? route_execution->latest_lidar_evidence.get()
      : direct_execution != nullptr ? direct_execution->latest_lidar_evidence.get()
                                    : resident_hold->latest_lidar_evidence.get();
  const VersionedExecutionValidationPolicy3D* const source_policy =
      route_execution != nullptr    ? route_execution->validation_policy.get()
      : direct_execution != nullptr ? direct_execution->validation_policy.get()
                                    : resident_hold->validation_policy.get();
  if (source_input == nullptr || source_lidar == nullptr || source_policy == nullptr ||
      certification.execution_input == nullptr ||
      executionInputProgressRelation(*certification.execution_input, *source_input) ==
          ExecutionInputProgressRelation3D::kInvalid ||
      certification.validation_policy == nullptr ||
      certification.validation_policy->contentFingerprint() !=
          source_policy->contentFingerprint() ||
      certification.latest_lidar_evidence == nullptr ||
      !latestLidarEvidenceNotOlder(*certification.latest_lidar_evidence,
                                   *source_lidar) ||
      !holdWorldNotOlder(certification, source_observed, source_static) ||
      !stationaryHoldPointSafe(certification, false)) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  }

  std::uint64_t source_trajectory_revision{0U};
  std::uint64_t hold_id{0U};
  StationaryExecutionHoldOrigin3D origin{
      StationaryExecutionHoldOrigin3D::kTerminalExecution};
  if (resident_hold != nullptr) {
    if (distance3D(resident_hold->position, certification.position) >
        kStationToleranceM) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
    }
    source_trajectory_revision = resident_hold->source_trajectory_revision;
    hold_id = resident_hold->hold_id;
    origin = resident_hold->origin;
    const bool same_evidence =
        certification.observed_raw_world == resident_hold->observed_raw_world &&
        certification.static_world == resident_hold->static_world &&
        certification.validation_policy->policyId() ==
            resident_hold->validation_policy->policyId() &&
        certification.latest_lidar_evidence->evidenceId() ==
            resident_hold->latest_lidar_evidence->evidenceId() &&
        certification.latest_lidar_evidence->contentFingerprint() ==
            resident_hold->latest_lidar_evidence->contentFingerprint();
    if (same_evidence) {
      return transitionFailure(ExecutionRouteTransitionStatus3D::kNoChange);
    }
  } else {
    if (current.execution_owner_epoch == std::numeric_limits<std::uint64_t>::max() ||
        source_horizon == nullptr || source_horizon->states.empty()) {
      return transitionFailure(ExecutionRouteTransitionStatus3D::kVersionExhausted);
    }
    const std::int64_t valid_until_ns = route_execution != nullptr
                                            ? route_execution->valid_until_ns
                                            : direct_execution->valid_until_ns;
    const mppi::State& terminal = source_horizon->states.back();
    if (certification.execution_input->effectiveStampNs() < valid_until_ns ||
        distance3D(certification.position, Point3{terminal.x, terminal.y, terminal.z}) >
            kStationaryExecutionHoldPositionToleranceM) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
    }
    source_trajectory_revision = route_execution != nullptr
                                     ? route_execution->trajectory_revision
                                     : direct_execution->trajectory_revision;
    hold_id = current.execution_owner_epoch + 1U;
  }

  ExecutionRouteSnapshot3D next = makeStationaryHoldSnapshot(
      current, std::move(certification), origin, source_trajectory_revision, hold_id);
  if (resident_hold == nullptr) {
    ++next.execution_owner_epoch;
  }
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D
armStationaryCaptureHold3D(const ExecutionRouteSnapshot3D& current,
                           const std::uint64_t expected_snapshot_version,
                           StationaryExecutionHoldCertification3D certification) {
  const ExecutionRouteTransitionStatus3D status =
      checkCurrentAndVersion(current, expected_snapshot_version);
  if (status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(status);
  }
  if (current.phase != ExecutionRoutePhase3D::kRevoked || current.route.has_value() ||
      current.finite_execution.has_value() || current.braking_fallback.has_value() ||
      current.direct_tracking_execution.has_value() ||
      current.stationary_hold.has_value()) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  if (current.execution_owner_epoch == std::numeric_limits<std::uint64_t>::max()) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kVersionExhausted);
  }
  if (!stationaryHoldPointSafe(certification, true)) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate);
  }
  const std::uint64_t hold_id = current.execution_owner_epoch + 1U;
  ExecutionRouteSnapshot3D next = makeStationaryHoldSnapshot(
      current, std::move(certification),
      StationaryExecutionHoldOrigin3D::kStationaryCaptureRearm, 0U, hold_id);
  ++next.execution_owner_epoch;
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D
revokeExecution3D(const ExecutionRouteSnapshot3D& current,
                  const std::uint64_t expected_snapshot_version) {
  const ExecutionRouteTransitionStatus3D status =
      checkCurrentAndVersion(current, expected_snapshot_version);
  if (status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(status);
  }
  if (current.phase == ExecutionRoutePhase3D::kRevoked) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kNoChange);
  }
  if (current.execution_owner_epoch == std::numeric_limits<std::uint64_t>::max()) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kVersionExhausted);
  }
  ExecutionRouteSnapshot3D next = current;
  ++next.version;
  ++next.execution_owner_epoch;
  next.phase = ExecutionRoutePhase3D::kRevoked;
  next.route_generation_high_water = current.routeGenerationHighWater();
  next.route.reset();
  next.finite_execution.reset();
  next.braking_fallback.reset();
  next.direct_tracking_execution.reset();
  next.stationary_hold.reset();
  return finishTransition(current, std::move(next));
}

} // namespace drone_city_nav
