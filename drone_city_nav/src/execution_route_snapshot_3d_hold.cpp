#include "drone_city_nav/execution_route_transitions_3d.hpp"
#include "drone_city_nav/finite_motion_horizon_3d.hpp"
#include "drone_city_nav/motion_altitude_envelope_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/occupied_collision_oracle_3d.hpp"
#include "drone_city_nav/proprioceptive_contact_seed_3d.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
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
  const MotionControl3D& control = execution_input.previousControl();
  const FootprintBodyAxis axis =
      bodyAxisFromWorldAcceleration(Vec3{control.ax, control.ay, control.az});
  const std::optional<LaunchSupportContact3D>* const launch_support_owner =
      observed_raw_world != nullptr
          ? std::addressof(observed_raw_world->launchSupportContact())
          : nullptr;
  const LaunchSupportContact3D* const launch_support =
      launch_support_owner != nullptr && launch_support_owner->has_value()
          ? std::addressof(**launch_support_owner)
          : nullptr;
  const std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_seed =
      proprioceptiveContactSeed3D(Point3{execution_input.state().x,
                                         execution_input.state().y,
                                         execution_input.state().z},
                                  control, validation_policy.sweptFootprint(),
                                  observed_raw_world != nullptr
                                      ? std::addressof(observed_raw_world->occupancy())
                                      : nullptr);
  const OccupiedCollisionOracle3D oracle{OccupiedCollisionWorld3D{
      .observed_occupancy = observed_raw_world != nullptr
                                ? std::addressof(observed_raw_world->occupancy())
                                : nullptr,
      .static_occupancy =
          static_world != nullptr ? std::addressof(static_world->occupancy()) : nullptr,
      .planar_occupancy = nullptr,
      .raw_point_cloud = latest_lidar_evidence.indexedHitPoints(),
      .launch_support_contact = launch_support,
      .proprioceptive_free_space_seed = proprioceptive_seed.has_value()
                                            ? std::addressof(*proprioceptive_seed)
                                            : nullptr,
      .footprint = validation_policy.sweptFootprint(),
      .flight_envelope = validation_policy.flightEnvelope(),
  }};
  return oracle.validatePoint(position, axis).clear();
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
  const MotionState3D& state = certification.execution_input->state();
  const MotionControl3D& control = certification.execution_input->previousControl();
  const Point3 actual_position{state.x, state.y, state.z};
  if (distance3D(certification.position, actual_position) >
          kStationaryExecutionHoldPositionToleranceM ||
      std::hypot(std::hypot(state.vx, state.vy), state.vz) >
          kStationaryExecutionHoldSpeedToleranceMps ||
      std::abs(state.yaw_rate) > kStationaryExecutionHoldYawRateToleranceRadps ||
      !insideFlightEnvelope(certification.position,
                            certification.validation_policy->flightEnvelope()) ||
      !motionAltitudeEnvelopeDynamicallyRecoverable3D(
          state, control, certification.validation_policy->dynamics(),
          certification.validation_policy->altitudeEnvelope())) {
    return false;
  }
  return stationaryHoldRawSafe(
      certification.position, *certification.execution_input,
      certification.observed_raw_world.get(), certification.static_world.get(),
      *certification.validation_policy, *certification.latest_lidar_evidence);
}

// A finite execution may hand its ownership to a stationary hold once its
// remaining lease commands nothing but rest at the terminal state: either the
// lease has ended, or every state scheduled at or after `stamp_ns` already
// rests there. The hold then replaces an owner that would only keep the
// vehicle still, so no commanded motion is truncated.
[[nodiscard]] bool finiteExecutionLeaseRestsAt(const FiniteMotionHorizon3D& horizon,
                                               const std::int64_t valid_from_ns,
                                               const std::int64_t valid_until_ns,
                                               const std::int64_t control_interval_ns,
                                               const std::int64_t stamp_ns) noexcept {
  if (stamp_ns >= valid_until_ns) {
    return true;
  }
  if (stamp_ns < valid_from_ns || control_interval_ns <= 0) {
    return false;
  }
  const std::int64_t elapsed_ns = stamp_ns - valid_from_ns;
  const std::int64_t first_state_index =
      elapsed_ns / control_interval_ns +
      static_cast<std::int64_t>(elapsed_ns % control_interval_ns != 0);
  return finiteMotionHorizonRestsFromState3D(
      horizon, static_cast<std::size_t>(first_state_index),
      kStationaryExecutionHoldPositionToleranceM,
      kStationaryExecutionHoldSpeedToleranceMps);
}

[[nodiscard]] ExecutionPlan3D
makeStationaryHoldSnapshot(const ExecutionPlan3D& current,
                           StationaryExecutionHoldCertification3D certification,
                           const StationaryExecutionHoldOrigin3D origin,
                           const std::uint64_t source_trajectory_revision,
                           const std::uint64_t hold_id) {
  ExecutionPlan3D next = current;
  ++next.version;
  next.route_generation_high_water = current.routeGenerationHighWater();
  next.state = StationaryHoldPlan3D{
      .owner =
          StationaryExecutionHold3D{
              .hold_id = hold_id,
              .source_trajectory_revision = source_trajectory_revision,
              .origin = origin,
              .position = certification.position,
              .terminal_execution_input = std::move(certification.execution_input),
              .observed_raw_world = std::move(certification.observed_raw_world),
              .static_world = std::move(certification.static_world),
              .validation_policy = std::move(certification.validation_policy),
              .latest_lidar_evidence = std::move(certification.latest_lidar_evidence),
          },
  };
  return next;
}

} // namespace

ExecutionRouteTransitionResult3D
execution_route_snapshot_3d_internal::applyTransferToExecutionHoldCommand3D(
    const ExecutionPlan3D& current, const std::uint64_t expected_snapshot_version,
    StationaryExecutionHoldCertification3D certification) {
  const ExecutionRouteTransitionStatus3D status =
      checkCurrentAndVersion(current, expected_snapshot_version);
  if (status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(status);
  }
  const FiniteExecutionState3D* const route_execution = current.finiteExecution();
  const DirectTrackingFiniteExecution3D* const direct_execution =
      current.directTrackingExecution();
  // A flown stop is the ordinary way a moving vehicle reaches rest, so it is a
  // hold source like any other terminal execution. Without this the plan would
  // have no way out of the stop it just completed.
  const StopExecution3D* const stop_execution = current.stopExecution();
  const StationaryExecutionHold3D* const resident_hold = current.stationaryHold();
  const std::size_t source_count =
      static_cast<std::size_t>(route_execution != nullptr) +
      static_cast<std::size_t>(direct_execution != nullptr) +
      static_cast<std::size_t>(stop_execution != nullptr) +
      static_cast<std::size_t>(resident_hold != nullptr);
  if (source_count != 1U) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  const FiniteMotionHorizon3D* source_horizon{nullptr};
  std::shared_ptr<const VersionedExecutionInput3D> source_input;
  const VersionedObservedRawWorld3D* source_observed{nullptr};
  const VersionedStaticWorld3D* source_static{nullptr};
  const VersionedLatestLidarEvidence3D* source_lidar{nullptr};
  const VersionedExecutionValidationPolicy3D* source_policy{nullptr};
  if (route_execution != nullptr) {
    source_horizon = route_execution->horizon.get();
    source_input = route_execution->execution_input;
    source_observed = route_execution->observed_raw_world.get();
    source_static = route_execution->static_world.get();
    source_lidar = route_execution->latest_lidar_evidence.get();
    source_policy = route_execution->validation_policy.get();
  } else if (direct_execution != nullptr) {
    source_horizon = direct_execution->horizon.get();
    source_input = direct_execution->execution_input;
    source_observed = direct_execution->observed_raw_world.get();
    source_static = direct_execution->static_world.get();
    source_lidar = direct_execution->latest_lidar_evidence.get();
    source_policy = direct_execution->validation_policy.get();
  } else if (stop_execution != nullptr) {
    source_horizon = stop_execution->horizon.get();
    source_input = stop_execution->execution_input;
    source_observed = stop_execution->observed_raw_world.get();
    source_static = stop_execution->static_world.get();
    source_lidar = stop_execution->latest_lidar_evidence.get();
    source_policy = stop_execution->validation_policy.get();
  } else {
    source_input = resident_hold->terminal_execution_input;
    source_observed = resident_hold->observed_raw_world.get();
    source_static = resident_hold->static_world.get();
    source_lidar = resident_hold->latest_lidar_evidence.get();
    source_policy = resident_hold->validation_policy.get();
  }
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
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate,
                             ExecutionRouteTransitionDetail3D::kHoldCertificationStale);
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
        certification.execution_input == resident_hold->terminal_execution_input &&
        certification.position.x == resident_hold->position.x &&
        certification.position.y == resident_hold->position.y &&
        certification.position.z == resident_hold->position.z &&
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

    // The one terminal execution the hold is taken over from, whatever kind of
    // execution owned the vehicle.
    struct TerminalExecutionLease3D {
      std::int64_t valid_from_ns{0};
      std::int64_t valid_until_ns{0};
      std::int64_t control_interval_ns{0};
      std::uint64_t trajectory_revision{0U};
    };

    const TerminalExecutionLease3D lease = [&]() -> TerminalExecutionLease3D {
      if (route_execution != nullptr) {
        return {route_execution->valid_from_ns, route_execution->valid_until_ns,
                route_execution->control_interval_ns,
                route_execution->trajectory_revision};
      }
      if (direct_execution != nullptr) {
        return {direct_execution->valid_from_ns, direct_execution->valid_until_ns,
                direct_execution->control_interval_ns,
                direct_execution->trajectory_revision};
      }
      return {stop_execution->valid_from_ns, stop_execution->valid_until_ns,
              stop_execution->control_interval_ns, stop_execution->trajectory_revision};
    }();
    // A route or a tracking horizon is handed over at the rest it commanded.
    // A stop's rest point is a prediction from the vehicle's dynamics, and
    // the vehicle rests wherever braking actually left it; the hold pins that
    // measured position, which the certification has already been checked
    // against, once the stop's remaining lease commands nothing but rest.
    const MotionState3D& terminal = source_horizon->states.back();
    if ((stop_execution == nullptr &&
         distance3D(certification.position,
                    Point3{terminal.x, terminal.y, terminal.z}) >
             kStationaryExecutionHoldPositionToleranceM) ||
        !finiteExecutionLeaseRestsAt(
            *source_horizon, lease.valid_from_ns, lease.valid_until_ns,
            lease.control_interval_ns,
            certification.execution_input->effectiveStampNs())) {
      return transitionFailure(
          ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
    }
    source_trajectory_revision = lease.trajectory_revision;
    hold_id = current.execution_owner_epoch + 1U;
  }

  ExecutionPlan3D next = makeStationaryHoldSnapshot(
      current, std::move(certification), origin, source_trajectory_revision, hold_id);
  if (resident_hold == nullptr) {
    ++next.execution_owner_epoch;
  }
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D
execution_route_snapshot_3d_internal::applyArmStationaryCaptureHoldCommand3D(
    const ExecutionPlan3D& current, const std::uint64_t expected_snapshot_version,
    StationaryExecutionHoldCertification3D certification) {
  const ExecutionRouteTransitionStatus3D status =
      checkCurrentAndVersion(current, expected_snapshot_version);
  if (status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(status);
  }
  if (current.phase() != ExecutionRoutePhase3D::kRevoked ||
      current.route() != nullptr || current.finiteExecution() != nullptr ||
      current.directTrackingExecution() != nullptr ||
      current.stationaryHold() != nullptr) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  if (current.execution_owner_epoch == std::numeric_limits<std::uint64_t>::max()) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kVersionExhausted);
  }
  if (!stationaryHoldPointSafe(certification, true)) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCandidate,
                             ExecutionRouteTransitionDetail3D::kHoldPointUnsafe);
  }
  const std::uint64_t hold_id = current.execution_owner_epoch + 1U;
  ExecutionPlan3D next = makeStationaryHoldSnapshot(
      current, std::move(certification),
      StationaryExecutionHoldOrigin3D::kStationaryCaptureRearm, 0U, hold_id);
  ++next.execution_owner_epoch;
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D
execution_route_snapshot_3d_internal::applyRevokeExecutionCommand3D(
    const ExecutionPlan3D& current, const std::uint64_t expected_snapshot_version) {
  const ExecutionRouteTransitionStatus3D status =
      checkCurrentAndVersion(current, expected_snapshot_version);
  if (status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(status);
  }
  if (current.phase() == ExecutionRoutePhase3D::kRevoked) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kNoChange);
  }
  if (current.execution_owner_epoch == std::numeric_limits<std::uint64_t>::max()) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kVersionExhausted);
  }
  ExecutionPlan3D next = current;
  ++next.version;
  ++next.execution_owner_epoch;
  next.route_generation_high_water = current.routeGenerationHighWater();
  next.state = RevokedPlan3D{};
  return finishTransition(current, std::move(next));
}

ExecutionRouteTransitionResult3D
execution_route_snapshot_3d_internal::applySuspendFiniteExecutionCommand3D(
    const ExecutionPlan3D& current, const std::uint64_t expected_snapshot_version) {
  const ExecutionRouteTransitionStatus3D status =
      checkCurrentAndVersion(current, expected_snapshot_version);
  if (status != ExecutionRouteTransitionStatus3D::kApplied) {
    return transitionFailure(status);
  }
  if (current.phase() == ExecutionRoutePhase3D::kAwaitingSuccessor &&
      current.route() != nullptr && current.finiteExecution() == nullptr) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kNoChange);
  }
  const bool suspendable_route_phase =
      current.phase() == ExecutionRoutePhase3D::kFollowing;
  if (!suspendable_route_phase || current.route() == nullptr ||
      current.finiteExecution() == nullptr || current.brakingFallback() == nullptr ||
      current.execution_owner_epoch == std::numeric_limits<std::uint64_t>::max()) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kFiniteExecutionConflict);
  }
  ExecutionPlan3D next = current;
  CertifiedRouteSuffix3D* const suspended_route = routePointer(next);
  if (suspended_route == nullptr) {
    return transitionFailure(ExecutionRouteTransitionStatus3D::kInvalidCurrentSnapshot);
  }
  ++next.version;
  ++next.execution_owner_epoch;
  next.state = AwaitingSuccessorPlan3D{
      .owner = SuspendedRoutePlan3D{.route = std::move(*suspended_route)},
  };
  return finishTransition(current, std::move(next));
}

} // namespace drone_city_nav
