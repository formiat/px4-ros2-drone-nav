#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/execution_route_snapshot_3d.hpp"
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

namespace execution_route_snapshot_3d_internal {

template<typename T>
[[nodiscard]] const T* optionalAddress(const std::optional<T>& value) noexcept {
  if (!value.has_value()) {
    return nullptr;
  }
  return std::addressof(value.value());
}

[[nodiscard]] static FiniteExecutionCertificationResult3D rejectedFiniteExecution(
    const FiniteExecutionCertificationStatus3D status,
    const RouteAdherenceAssessment3D* const route_adherence = nullptr) noexcept {
  return {
      .status = status,
      .execution = std::nullopt,
      .route_adherence_status =
          route_adherence != nullptr
              ? route_adherence->status
              : FiniteExecutionRouteAdherenceStatus3D::kNotEvaluated,
      .route_adherence_failure_state_index =
          route_adherence != nullptr ? route_adherence->failure_state_index : 0U,
      .route_adherence_failure_distance_m =
          route_adherence != nullptr ? route_adherence->failure_distance_m : -1.0,
  };
}

[[nodiscard]] std::optional<RouteAdherenceAssessment3D>
validateExecutionProgressConnector(
    const CertifiedRouteSuffix3D& route, const Point3& execution_position,
    const std::shared_ptr<const VersionedExecutionInput3D>& execution_input,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& observed_raw_world,
    const std::span<const Point3> latest_lidar_obstacle_points) {
  if (route.progress.execution_input == nullptr || execution_input == nullptr) {
    return std::nullopt;
  }
  const double connector_travel_m =
      distance3D(route.progress.last_observed_position, execution_position);
  if (!std::isfinite(connector_travel_m)) {
    return std::nullopt;
  }
  const CertificateView3D certificate_view = certificateView(route.certificate);
  const double connector_maximum_station_m = std::min(
      certificate_view.certified_end_station_m,
      route.progress.station_m + kMaximumStationCreditPerTravel * connector_travel_m +
          kStationToleranceM);
  const std::array<mppi::State, 2U> connector_states{
      mppi::State{.x = static_cast<float>(route.progress.last_observed_position.x),
                  .y = static_cast<float>(route.progress.last_observed_position.y),
                  .z = static_cast<float>(route.progress.last_observed_position.z)},
      mppi::State{.x = static_cast<float>(execution_position.x),
                  .y = static_cast<float>(execution_position.y),
                  .z = static_cast<float>(execution_position.z)},
  };
  const std::optional<double> cross_track_limit =
      route.validation_policy->routeCrossTrackConstraintsEnabled()
          ? std::optional<double>{kMaximumRouteCrossTrackM}
          : std::nullopt;
  RouteAdherenceAssessment3D adherence = validateFiniteRouteAdherence(
      *route.geometry, connector_states, route.progress.station_m,
      certificate_view.suffix_start_station_m, connector_maximum_station_m,
      cross_track_limit, cross_track_limit,
      route.validation_policy->sweptFootprint().sweep_step_m, false,
      route.validation_policy->routeTrackingTubeConstraintsEnabled());
  if (!adherence.accepted) {
    return std::nullopt;
  }

  const mppi::Control& previous_route_control =
      route.progress.execution_input->previousControl();
  const mppi::Control& current_execution_control = execution_input->previousControl();
  const FootprintBodyAxis previous_route_axis = bodyAxisFromWorldAcceleration(Vec3{
      previous_route_control.ax, previous_route_control.ay, previous_route_control.az});
  const FootprintBodyAxis current_execution_axis = bodyAxisFromWorldAcceleration(
      Vec3{current_execution_control.ax, current_execution_control.ay,
           current_execution_control.az});
  const auto* const raw_certificate =
      std::get_if<ObservedRawRouteCertificate3D>(&route.certificate);
  const ProprioceptiveFreeSpaceSeed3D* const free_space_seed =
      raw_certificate != nullptr && observed_raw_world != nullptr
          ? optionalAddress(observed_raw_world->proprioceptiveFreeSpaceSeed())
          : nullptr;
  const LaunchSupportContact3D* const launch_support_contact =
      raw_certificate != nullptr && observed_raw_world != nullptr
          ? optionalAddress(observed_raw_world->launchSupportContact())
          : nullptr;
  const bool world_safe =
      raw_certificate != nullptr
          ? observed_raw_world != nullptr &&
                validateObservedSweptFootprint(
                    observed_raw_world->occupancy(),
                    route.progress.last_observed_position, previous_route_axis,
                    execution_position, current_execution_axis,
                    route.validation_policy->sweptFootprint(),
                    ObservedSpaceValidationPolicy::kAllowUnknown, free_space_seed,
                    launch_support_contact)
                    .accepted()
          : route.static_world != nullptr &&
                validateKnownStaticSweptFootprint(
                    route.static_world->occupancy(),
                    route.progress.last_observed_position, previous_route_axis,
                    execution_position, current_execution_axis,
                    route.validation_policy->sweptFootprint())
                    .accepted();
  if (!world_safe ||
      (!latest_lidar_obstacle_points.empty() &&
       !validateRawPointCloudSweptFootprint(
            latest_lidar_obstacle_points, route.progress.last_observed_position,
            previous_route_axis, execution_position, current_execution_axis,
            route.validation_policy->sweptFootprint(), launch_support_contact)
            .accepted())) {
    return std::nullopt;
  }
  return adherence;
}

[[nodiscard]] RouteAdherenceAssessment3D
brakingRouteOwnershipBinding(const CertifiedRouteSuffix3D& route,
                             const std::span<const mppi::State> states,
                             const double station_m) noexcept {
  RouteAdherenceAssessment3D binding;
  if (route.geometry == nullptr || route.geometry->route == nullptr || states.empty() ||
      !std::isfinite(station_m) ||
      station_m < certificateView(route.certificate).suffix_start_station_m -
                      kStationToleranceM ||
      station_m > route.endStationM() + kStationToleranceM) {
    binding.status = FiniteExecutionRouteAdherenceStatus3D::kInvalidInput;
    return binding;
  }
  const RouteSample3D owner_sample =
      sampleRoute3DAtStation(*route.geometry->route, station_m);
  const auto projection = [&](const mppi::State& state) {
    const Point3 position{state.x, state.y, state.z};
    return RouteProjection3D{
        .valid = true,
        .station_m = station_m,
        .remaining_m = std::max(0.0, route.endStationM() - station_m),
        .distance_m = distance3D(position, owner_sample.position),
        .point = owner_sample.position,
    };
  };
  binding.begin = projection(states.front());
  binding.stop = projection(states.back());
  if (!std::isfinite(binding.begin.distance_m) ||
      !std::isfinite(binding.stop.distance_m)) {
    binding.status = FiniteExecutionRouteAdherenceStatus3D::kInvalidInput;
    return binding;
  }
  binding.status = FiniteExecutionRouteAdherenceStatus3D::kAccepted;
  binding.accepted = true;
  return binding;
}

[[nodiscard]] FiniteExecutionCertificationResult3D
certifyFiniteExecutionAgainstOwnedWorld3D(
    const ExecutionRouteSnapshot3D& current, const CertifiedRouteSuffix3D& target_route,
    FiniteExecutionCertification3D certification,
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_validation_world,
    const RouteLifecycleEvent3D* const lifecycle_event,
    const std::optional<double> atomic_plan_begin_station_m = std::nullopt) {
  const CertifiedRouteSuffix3D* const current_route = routePointer(current);
  const bool certifies_raw_invalidation =
      lifecycle_event != nullptr &&
      lifecycle_event->kind == RouteLifecycleEventKind3D::kRawInvalidated;
  const bool certifies_lifecycle_braking =
      lifecycle_event != nullptr &&
      (lifecycle_event->kind == RouteLifecycleEventKind3D::kObjectiveSuperseded ||
       lifecycle_event->kind == RouteLifecycleEventKind3D::kCrossTrackExceeded ||
       lifecycle_event->kind == RouteLifecycleEventKind3D::kTrackingTubeExceeded);
  const bool certifies_braking_execution =
      certification.kind == FiniteExecutionKind3D::kEmergencyBrakeTail;
  if (!current.valid() || !target_route.valid() ||
      certification.trajectory_revision == 0U ||
      certification.execution_input == nullptr ||
      !certification.execution_input->valid() ||
      !certification.execution_input->nominalStateAuthoritative() ||
      certification.valid_from_ns <= 0 ||
      certification.execution_input->effectiveStampNs() !=
          certification.valid_from_ns ||
      (current.finite_execution.has_value() &&
       certification.trajectory_revision <=
           current.finite_execution->trajectory_revision) ||
      (current.direct_tracking_execution.has_value() &&
       certification.trajectory_revision <=
           current.direct_tracking_execution->trajectory_revision)) {
    return rejectedFiniteExecution(FiniteExecutionCertificationStatus3D::kInvalidInput);
  }

  const bool targets_current_route =
      current_route != nullptr &&
      target_route.route_instance_id == current_route->route_instance_id;
  const bool targets_successor_route =
      current_route != nullptr &&
      current_route->identity.generation != std::numeric_limits<std::uint64_t>::max() &&
      target_route.identity.generation == current_route->identity.generation + 1U;
  const bool targets_direct_successor =
      current.phase == ExecutionRoutePhase3D::kDirectTracking &&
      current.direct_tracking_execution.has_value() && current_route == nullptr &&
      current.routeGenerationHighWater() != std::numeric_limits<std::uint64_t>::max() &&
      target_route.identity.generation == current.routeGenerationHighWater() + 1U;
  const bool targets_initial_route =
      current_route == nullptr && !current.finite_execution.has_value() &&
      !current.direct_tracking_execution.has_value() &&
      current.routeGenerationHighWater() != std::numeric_limits<std::uint64_t>::max() &&
      target_route.identity.generation == current.routeGenerationHighWater() + 1U;
  if (!targets_current_route && !targets_successor_route && !targets_direct_successor &&
      !targets_initial_route) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kTargetRelationRejected);
  }
  if (lifecycle_event != nullptr && !targets_current_route) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kTargetRelationRejected);
  }
  if (targets_current_route) {
    if (target_route.progress.execution_input == nullptr) {
      return rejectedFiniteExecution(
          FiniteExecutionCertificationStatus3D::kProgressRelationRejected);
    }
    const ExecutionInputProgressRelation3D progress_relation =
        executionInputProgressRelation(*certification.execution_input,
                                       *target_route.progress.execution_input);
    if ((certifies_raw_invalidation &&
         progress_relation != ExecutionInputProgressRelation3D::kStrictlyNewer) ||
        (certifies_lifecycle_braking &&
         progress_relation == ExecutionInputProgressRelation3D::kInvalid)) {
      return rejectedFiniteExecution(
          FiniteExecutionCertificationStatus3D::kProgressRelationRejected);
    }
  }

  const auto* const static_certificate =
      std::get_if<StaticRouteCertificate3D>(&target_route.certificate);
  const auto* const raw_certificate =
      std::get_if<ObservedRawRouteCertificate3D>(&target_route.certificate);
  const bool static_mode = static_certificate != nullptr;
  const bool raw_mode = raw_certificate != nullptr;
  const std::shared_ptr<const VersionedExecutionValidationPolicy3D>& policy =
      target_route.validation_policy;
  if (certifies_raw_invalidation) {
    const auto* const previous_raw_lineage =
        current.finite_execution.has_value()
            ? std::get_if<ObservedRawFiniteExecutionValidationLineage3D>(
                  &current.finite_execution->validation_proof.lineage)
            : nullptr;
    if (!raw_mode || lifecycle_event->generation != target_route.identity.generation ||
        lifecycle_event->raw_producer_instance_id == 0U ||
        lifecycle_event->raw_producer_instance_id !=
            raw_certificate->producer_instance_id ||
        lifecycle_event->raw_revision <= raw_certificate->validated_through_revision ||
        observed_raw_validation_world == nullptr ||
        !observed_raw_validation_world->valid() ||
        observed_raw_validation_world->version().producer_instance_id !=
            lifecycle_event->raw_producer_instance_id ||
        observed_raw_validation_world->version().revision !=
            lifecycle_event->raw_revision ||
        certification.kind != FiniteExecutionKind3D::kEmergencyBrakeTail ||
        (previous_raw_lineage != nullptr &&
         (lifecycle_event->raw_revision <
              previous_raw_lineage->validated_through_raw_revision ||
          (lifecycle_event->raw_revision ==
               previous_raw_lineage->validated_through_raw_revision &&
           observed_raw_validation_world->contentFingerprint() !=
               previous_raw_lineage->observed_world_content_fingerprint))) ||
        !canonicalPassageGeometryMatchesObservedWorld(
            *target_route.geometry, *observed_raw_validation_world,
            target_route.geometry->passage_volume_config)) {
      return rejectedFiniteExecution(
          FiniteExecutionCertificationStatus3D::kRawInvalidationContractRejected);
    }
  }
  if (lifecycle_event != nullptr && !certifies_raw_invalidation &&
      (!certifies_lifecycle_braking ||
       lifecycle_event->generation != target_route.identity.generation ||
       lifecycle_event->raw_producer_instance_id != 0U ||
       lifecycle_event->raw_revision != 0U ||
       certification.kind != FiniteExecutionKind3D::kEmergencyBrakeTail)) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kLifecycleBrakingContractRejected);
  }
  if ((!static_mode && !raw_mode) || policy == nullptr || !policy->valid() ||
      policy->contentFingerprint() != certificateView(target_route.certificate)
                                          .execution_validation_policy_fingerprint ||
      !executionInputFreshAt(*certification.execution_input, *policy,
                             certification.valid_from_ns) ||
      certification.latest_lidar_evidence == nullptr ||
      !latestLidarEvidenceFreshAt(*certification.latest_lidar_evidence, *policy,
                                  certification.valid_from_ns) ||
      (static_mode && (!staticWorldMatchesCertificate(target_route.static_world,
                                                      *static_certificate) ||
                       observed_raw_validation_world != nullptr)) ||
      (raw_mode && ((!certifies_raw_invalidation &&
                     !rawWorldMatchesCertificate(observed_raw_validation_world,
                                                 *raw_certificate, true)) ||
                    target_route.static_world != nullptr))) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kEvidenceContractRejected);
  }

  const ProprioceptiveFreeSpaceSeed3D* const free_space_seed =
      raw_mode ? optionalAddress(
                     observed_raw_validation_world->proprioceptiveFreeSpaceSeed())
               : nullptr;
  const LaunchSupportContact3D* const launch_support_contact =
      raw_mode ? optionalAddress(observed_raw_validation_world->launchSupportContact())
               : nullptr;
  const ObservedSpaceValidationPolicy observed_policy =
      raw_mode ? ObservedSpaceValidationPolicy::kAllowUnknown
               : ObservedSpaceValidationPolicy::kRequireKnownFree;
  const std::uint64_t execution_collision_policy_fingerprint =
      validationPolicyFingerprint(policy->sweptFootprint(), observed_policy,
                                  free_space_seed, launch_support_contact);
  if (execution_collision_policy_fingerprint == 0U) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kCollisionPolicyInvalid);
  }

  const mppi::FiniteHorizon& validated_horizon = certification.horizon;
  const std::int64_t control_interval_ns =
      mppi::finitePathControlIntervalNanoseconds(policy->dynamics().dt_s);
  if (validated_horizon.controls.empty() || validated_horizon.states.empty() ||
      validated_horizon.states.size() != validated_horizon.controls.size() + 1U ||
      control_interval_ns <= 0 ||
      !finiteHorizonDynamicallyConsistent(
          validated_horizon, certification.execution_input->previousControl(),
          policy->dynamics()) ||
      validated_horizon.controls.size() >
          static_cast<std::uint64_t>(
              (std::numeric_limits<std::int64_t>::max() - certification.valid_from_ns) /
              control_interval_ns)) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kHorizonContractRejected);
  }

  const CertificateView3D certificate_view = certificateView(target_route.certificate);
  const mppi::State& initial_state = validated_horizon.states.front();
  const Point3 initial_state_position{initial_state.x, initial_state.y,
                                      initial_state.z};
  if (!finiteStateNearlyEqual(initial_state, certification.execution_input->state())) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kInitialStateMismatch);
  }
  const std::span<const Point3> latest_lidar_obstacle_points =
      certification.latest_lidar_evidence != nullptr
          ? std::span<const Point3>{certification.latest_lidar_evidence
                                        ->hitPointsMapM()}
          : std::span<const Point3>{};
  double execution_begin_station_m = target_route.progress.station_m;
  if (certifies_raw_invalidation) {
    const std::optional<RouteAdherenceAssessment3D> connector_adherence =
        validateExecutionProgressConnector(
            target_route, initial_state_position, certification.execution_input,
            observed_raw_validation_world, latest_lidar_obstacle_points);
    if (!connector_adherence.has_value()) {
      return rejectedFiniteExecution(
          FiniteExecutionCertificationStatus3D::kRawInvalidationConnectorRejected);
    }
    execution_begin_station_m = connector_adherence->stop.station_m;
  } else if (certifies_lifecycle_braking) {
    // Once route following is explicitly terminated, the stop is owned by the
    // route lifecycle but is not a route-following maneuver. Do not manufacture
    // progress or reject the only physical execution owner because the vehicle
    // is already outside the old behavioral corridor.
    execution_begin_station_m = target_route.progress.station_m;
  } else if (target_route.progress.execution_input != nullptr &&
             distance3D(initial_state_position,
                        target_route.progress.last_observed_position) >
                 kExecutionBindingToleranceM) {
    const std::optional<RouteAdherenceAssessment3D> connector_adherence =
        validateExecutionProgressConnector(
            target_route, initial_state_position, certification.execution_input,
            observed_raw_validation_world, latest_lidar_obstacle_points);
    if (!connector_adherence.has_value()) {
      return rejectedFiniteExecution(
          FiniteExecutionCertificationStatus3D::kExecutionBindingRejected);
    }
    execution_begin_station_m = connector_adherence->stop.station_m;
  } else if (distance3D(initial_state_position,
                        target_route.progress.last_observed_position) >
             kExecutionBindingToleranceM) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kExecutionBindingRejected);
  }
  if (atomic_plan_begin_station_m.has_value()) {
    if (!std::isfinite(*atomic_plan_begin_station_m) ||
        std::abs(*atomic_plan_begin_station_m - execution_begin_station_m) >
            kExecutionBindingToleranceM) {
      return rejectedFiniteExecution(
          FiniteExecutionCertificationStatus3D::kExecutionBindingRejected);
    }
    // Both artifacts in an atomic plan must use one bit-identical station
    // binding. Independent projections can differ by floating-point noise and
    // must not make the resident braking tail invalid after publication.
    execution_begin_station_m = *atomic_plan_begin_station_m;
  }
  // Route-following paths preserve station order and constrained-passage
  // geometry. The generic centerline corridor is an opt-in behavioral
  // constraint because physical safety is independently certified below. A
  // lifecycle brake is intentionally bound only to the immutable route owner.
  const std::optional<double> cross_track_limit =
      policy->routeCrossTrackConstraintsEnabled() &&
              certification.kind != FiniteExecutionKind3D::kEmergencyBrakeTail
          ? std::optional<double>{kMaximumRouteCrossTrackM}
          : std::nullopt;
  const RouteAdherenceAssessment3D route_adherence =
      certifies_braking_execution
          ? brakingRouteOwnershipBinding(target_route, validated_horizon.states,
                                         execution_begin_station_m)
          : validateFiniteRouteAdherence(
                *target_route.geometry, validated_horizon.states,
                execution_begin_station_m, certificate_view.suffix_start_station_m,
                certificate_view.certified_end_station_m, cross_track_limit,
                cross_track_limit, policy->sweptFootprint().sweep_step_m,
                targets_initial_route || targets_direct_successor ||
                    (targets_current_route &&
                     certifiedTrackingTubeHandoffPending(current, target_route)),
                policy->routeTrackingTubeConstraintsEnabled());
  if (!route_adherence.accepted) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kRouteAdherenceRejected,
        &route_adherence);
  }
  const RouteProjection3D& begin_projection = route_adherence.begin;
  const RouteProjection3D& stop_projection = route_adherence.stop;
  const std::optional<FiniteRouteTerminalBoundary3D> terminal_boundary =
      canonicalFiniteRouteTerminalBoundary(target_route, begin_projection.station_m);
  if (!terminal_boundary.has_value() ||
      !terminal_boundary->valid(target_route.endStationM())) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kTerminalBoundaryInvalid);
  }

  mppi::FiniteExecutionPathWorld validation_world{
      .flight_envelope = &policy->flightEnvelope(),
      .dynamics = &policy->dynamics(),
      .altitude_envelope = &policy->altitudeEnvelope(),
      .footprint = &policy->sweptFootprint(),
      .static_occupancy =
          static_mode ? &target_route.static_world->occupancy() : nullptr,
      .observed_occupancy =
          raw_mode ? &observed_raw_validation_world->occupancy() : nullptr,
      .require_known_free_space = static_mode,
      .proprioceptive_free_space_seed = free_space_seed,
      .launch_support_contact = launch_support_contact,
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points = latest_lidar_obstacle_points,
      .terminal_boundary = std::nullopt,
  };
  // A braking tail is a physical safety maneuver, not a route-following attempt
  // to capture the endpoint. It remains bound to the immutable route owner and
  // station, while dynamics, swept occupancy, and latest lidar stay mandatory.
  validation_world.terminal_boundary =
      certifies_braking_execution
          ? std::nullopt
          : makeValidationTerminalBoundary(terminal_boundary, target_route);
  if (policy->routeTrackingTubeConstraintsEnabled() && !certifies_braking_execution &&
      !validateTrackingTubeHandoffClearance(
          target_route, validated_horizon, begin_projection.station_m,
          certification.execution_input->previousControl(), validation_world)) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kTrackingTubeHandoffRejected);
  }
  const std::vector<mppi::TimedExecutionPathPoint> validation_points =
      timedExecutionPathPoints(validated_horizon,
                               certification.execution_input->previousControl(),
                               control_interval_ns);
  const mppi::FiniteExecutionPathValidation validation =
      mppi::validateCompleteFiniteExecutionPath(
          validation_points, certification.execution_input->previousControl(),
          validation_world);
  if (!validation.accepted()) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kPathValidationRejected);
  }

  const std::uint64_t expected_static_occupancy_fingerprint =
      static_certificate == nullptr
          ? 0U
          : expectedStaticOccupancyFingerprint(*static_certificate);
  if (static_certificate != nullptr && validation_world.static_occupancy != nullptr &&
      (expected_static_occupancy_fingerprint == 0U ||
       validation_world.static_occupancy->fingerprint() == 0U ||
       validation_world.static_occupancy->fingerprint() !=
           expected_static_occupancy_fingerprint)) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kStaticWorldFingerprintMismatch);
  }

  FiniteExecutionValidationLineage3D validation_lineage{
      StaticFiniteExecutionValidationLineage3D{}};
  if (raw_mode) {
    const std::uint64_t policy_fingerprint =
        validationPolicyFingerprint(*validation_world.footprint, observed_policy,
                                    validation_world.proprioceptive_free_space_seed,
                                    validation_world.launch_support_contact);
    if (policy_fingerprint == 0U ||
        observed_raw_validation_world->version().producer_instance_id !=
            raw_certificate->producer_instance_id ||
        (!certifies_raw_invalidation &&
         observed_raw_validation_world->version().revision !=
             raw_certificate->validated_through_revision)) {
      return rejectedFiniteExecution(
          FiniteExecutionCertificationStatus3D::kRawValidationLineageRejected);
    }
    validation_lineage = ObservedRawFiniteExecutionValidationLineage3D{
        .producer_instance_id =
            observed_raw_validation_world->version().producer_instance_id,
        .validated_through_raw_revision =
            observed_raw_validation_world->version().revision,
        .validation_policy_fingerprint = policy_fingerprint,
        .observed_world_content_fingerprint =
            observed_raw_validation_world->contentFingerprint(),
    };
  } else {
    validation_lineage = StaticFiniteExecutionValidationLineage3D{
        .world_certificate = target_route.static_world->certificate(),
        .static_occupancy_content_fingerprint =
            target_route.static_world->contentFingerprint(),
        .validation_policy_fingerprint = execution_collision_policy_fingerprint,
    };
  }
  if (!validationLineageValidForCertificate(validation_lineage,
                                            target_route.certificate)) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kValidationLineageRejected);
  }
  const std::uint64_t validation_contract_fingerprint = validationContractFingerprint(
      validation_world, certification.execution_input->previousControl(),
      ValidationContractOwners3D{
          .observed_raw_world =
              raw_mode ? observed_raw_validation_world.get() : nullptr,
          .static_world = static_mode ? target_route.static_world.get() : nullptr,
          .latest_lidar_evidence = certification.latest_lidar_evidence.get(),
      });
  if (validation_contract_fingerprint == 0U) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kValidationContractInvalid);
  }

  const std::int64_t valid_until_ns =
      certification.valid_from_ns +
      static_cast<std::int64_t>(validated_horizon.controls.size()) *
          control_interval_ns;
  const mppi::State& terminal_state = validated_horizon.states.back();
  const Point3 terminal_position{terminal_state.x, terminal_state.y, terminal_state.z};
  const std::uint64_t validated_raw_revision = rawValidatedRevision(validation_lineage);
  FiniteExecutionState3D execution{
      .trajectory_revision = certification.trajectory_revision,
      .source_snapshot_version = current.version,
      .source_navigation_revision = certification.execution_input->poseRevision(),
      .source_route_instance_id = target_route.route_instance_id,
      .source_route_generation = target_route.identity.generation,
      .source_geometry_revision = target_route.geometry->executable_geometry_revision,
      .source_physical_route_fingerprint =
          target_route.geometry->physical_route_fingerprint,
      .certificate = target_route.certificate,
      .horizon =
          std::make_shared<const mppi::FiniteHorizon>(std::move(certification.horizon)),
      .observed_raw_world = std::move(observed_raw_validation_world),
      .static_world = target_route.static_world,
      .validation_policy = policy,
      .execution_input = std::move(certification.execution_input),
      .latest_lidar_evidence = std::move(certification.latest_lidar_evidence),
      .terminal_boundary = terminal_boundary,
      .stop_boundary =
          CertifiedStopBoundary3D{
              .route_instance_id = target_route.route_instance_id,
              .route_generation = target_route.identity.generation,
              .geometry_revision = target_route.geometry->executable_geometry_revision,
              .physical_route_fingerprint =
                  target_route.geometry->physical_route_fingerprint,
              .trajectory_revision = certification.trajectory_revision,
              .raw_validated_through_revision = validated_raw_revision,
              .station_m = stop_projection.station_m,
              .position_tolerance_m = kFiniteStopPositionToleranceM,
              .position = terminal_position,
          },
      .begin_route_station_m = begin_projection.station_m,
      .valid_from_ns = certification.valid_from_ns,
      .valid_until_ns = valid_until_ns,
      .control_interval_ns = control_interval_ns,
      .kind = certification.kind,
      .validation_proof =
          FiniteExecutionValidationProof3D{
              .validation_contract_fingerprint = validation_contract_fingerprint,
              .lineage = validation_lineage,
          },
      .revalidation_required = false,
  };
  execution.validation_proof.artifact_fingerprint =
      finiteExecutionArtifactFingerprint(execution);
  CertifiedRouteSuffix3D rebound_route = target_route;
  bindProgressToExecutionInput(rebound_route.progress, execution.execution_input,
                               execution.begin_route_station_m);
  if (!execution.validFor(certifies_raw_invalidation ? nullptr : &rebound_route)) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kInvalidArtifact);
  }
  return {
      .status = FiniteExecutionCertificationStatus3D::kCertified,
      .execution = std::move(execution),
      .route_adherence_status = FiniteExecutionRouteAdherenceStatus3D::kAccepted,
      .route_adherence_failure_state_index = 0U,
      .route_adherence_failure_distance_m = -1.0,
  };
}

} // namespace execution_route_snapshot_3d_internal

mppi::FiniteExecutionPathValidation
validateRemainingFiniteExecutionAgainstObservedWorld3D(
    const FiniteExecutionState3D& execution,
    const VersionedExecutionInput3D& current_input,
    const VersionedObservedRawWorld3D& current_world,
    const std::int64_t validation_stamp_ns) noexcept {
  if (execution.horizon == nullptr || execution.validation_policy == nullptr ||
      execution.execution_input == nullptr || execution.observed_raw_world == nullptr ||
      execution.static_world != nullptr || !current_input.valid() ||
      !current_input.nominalStateAuthoritative() || !current_world.valid() ||
      validation_stamp_ns <= 0 ||
      current_input.effectiveStampNs() != validation_stamp_ns ||
      current_world.version().producer_instance_id !=
          execution.observed_raw_world->version().producer_instance_id ||
      current_world.version().revision <
          execution.observed_raw_world->version().revision) {
    return {};
  }
  const std::vector<mppi::TimedExecutionPathPoint> points = timedExecutionPathPoints(
      *execution.horizon, execution.execution_input->previousControl(),
      execution.control_interval_ns);
  if (points.empty()) {
    return {};
  }
  const ProprioceptiveFreeSpaceSeed3D* const free_space_seed =
      current_world.proprioceptiveFreeSpaceSeed().has_value()
          ? std::addressof(*current_world.proprioceptiveFreeSpaceSeed())
          : nullptr;
  const LaunchSupportContact3D* const launch_support_contact =
      current_world.launchSupportContact().has_value()
          ? std::addressof(*current_world.launchSupportContact())
          : nullptr;
  const mppi::FiniteExecutionPathWorld validation_world{
      .flight_envelope = &execution.validation_policy->flightEnvelope(),
      .dynamics = &execution.validation_policy->dynamics(),
      .altitude_envelope = &execution.validation_policy->altitudeEnvelope(),
      .footprint = &execution.validation_policy->sweptFootprint(),
      .static_occupancy = nullptr,
      .observed_occupancy = &current_world.occupancy(),
      .require_known_free_space = false,
      .proprioceptive_free_space_seed = free_space_seed,
      .launch_support_contact = launch_support_contact,
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points = {},
      .terminal_boundary = std::nullopt,
  };
  return mppi::validateFiniteExecutionTrajectoryContinuation(
      points, execution.valid_from_ns, execution.valid_until_ns, validation_stamp_ns,
      current_input.state(), current_input.previousControl(), validation_world);
}

std::optional<FiniteExecutionState3D>
certifyFiniteExecution3D(const ExecutionRouteSnapshot3D& current,
                         const CertifiedRouteSuffix3D& target_route,
                         FiniteExecutionCertification3D certification) {
  return certifyFiniteExecution3DDetailed(current, target_route,
                                          std::move(certification))
      .execution;
}

FiniteExecutionCertificationResult3D
certifyFiniteExecution3DDetailed(const ExecutionRouteSnapshot3D& current,
                                 const CertifiedRouteSuffix3D& target_route,
                                 FiniteExecutionCertification3D certification) {
  return certifyFiniteExecutionAgainstOwnedWorld3D(
      current, target_route, std::move(certification), target_route.observed_raw_world,
      nullptr, std::nullopt);
}

FiniteExecutionPlanCertificationResult3D
certifyFiniteExecutionPlan3DDetailed(const ExecutionRouteSnapshot3D& current,
                                     const CertifiedRouteSuffix3D& target_route,
                                     FiniteExecutionPlanCertification3D certification) {
  FiniteExecutionPlanCertificationResult3D result;
  if (certification.command_horizon.kind != FiniteExecutionKind3D::kNominal &&
      certification.command_horizon.kind != FiniteExecutionKind3D::kRetained) {
    return result;
  }
  FiniteExecutionCertification3D braking_certification{
      .trajectory_revision = certification.command_horizon.trajectory_revision,
      .horizon = std::move(certification.braking_tail),
      .execution_input = certification.command_horizon.execution_input,
      .latest_lidar_evidence = certification.command_horizon.latest_lidar_evidence,
      .valid_from_ns = certification.command_horizon.valid_from_ns,
      .kind = FiniteExecutionKind3D::kEmergencyBrakeTail,
  };
  result.command_horizon = certifyFiniteExecution3DDetailed(
      current, target_route, std::move(certification.command_horizon));
  if (!result.command_horizon.certified() ||
      !result.command_horizon.execution.has_value()) {
    return result;
  }
  result.braking_tail = certifyFiniteExecutionAgainstOwnedWorld3D(
      current, target_route, std::move(braking_certification),
      target_route.observed_raw_world, nullptr,
      result.command_horizon.execution->begin_route_station_m);
  if (!result.braking_tail.certified() || !result.braking_tail.execution.has_value()) {
    return result;
  }
  FiniteExecutionPlan3D plan{
      .command_horizon = result.command_horizon.execution.value(),
      .braking_tail = result.braking_tail.execution.value(),
  };
  CertifiedRouteSuffix3D rebound_route = target_route;
  bindProgressToExecutionInput(rebound_route.progress,
                               plan.command_horizon.execution_input,
                               plan.command_horizon.begin_route_station_m);
  if (!rebound_route.valid() || !plan.validFor(rebound_route)) {
    return result;
  }
  result.plan = std::move(plan);
  return result;
}

std::optional<FiniteExecutionState3D>
certifyFiniteExecution3D(const ExecutionRouteSnapshot3D& current,
                         FiniteExecutionCertification3D certification) {
  const CertifiedRouteSuffix3D* const route = routePointer(current);
  return route == nullptr
             ? std::nullopt
             : certifyFiniteExecution3D(current, *route, std::move(certification));
}

std::optional<FiniteExecutionState3D> certifyRawInvalidatedFiniteExecution3D(
    const ExecutionRouteSnapshot3D& current,
    RawInvalidatedFiniteExecutionCertification3D certification) {
  return certifyRawInvalidatedFiniteExecution3DDetailed(current,
                                                        std::move(certification))
      .execution;
}

FiniteExecutionCertificationResult3D certifyRawInvalidatedFiniteExecution3DDetailed(
    const ExecutionRouteSnapshot3D& current,
    RawInvalidatedFiniteExecutionCertification3D certification) {
  const CertifiedRouteSuffix3D* const route = routePointer(current);
  if (route == nullptr) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kTargetRelationRejected);
  }
  const RouteLifecycleEvent3D invalidation = certification.invalidation;
  return certifyFiniteExecutionAgainstOwnedWorld3D(
      current, *route, std::move(certification.finite_execution),
      std::move(certification.invalidating_observed_raw_world), &invalidation,
      std::nullopt);
}

std::optional<FiniteExecutionState3D> certifyLifecycleBrakingFiniteExecution3D(
    const ExecutionRouteSnapshot3D& current,
    LifecycleBrakingFiniteExecutionCertification3D certification) {
  return certifyLifecycleBrakingFiniteExecution3DDetailed(current,
                                                          std::move(certification))
      .execution;
}

FiniteExecutionCertificationResult3D certifyLifecycleBrakingFiniteExecution3DDetailed(
    const ExecutionRouteSnapshot3D& current,
    LifecycleBrakingFiniteExecutionCertification3D certification) {
  const CertifiedRouteSuffix3D* const route = routePointer(current);
  if (route == nullptr) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kTargetRelationRejected);
  }
  const RouteLifecycleEvent3D lifecycle_event = certification.lifecycle_event;
  return certifyFiniteExecutionAgainstOwnedWorld3D(
      current, *route, std::move(certification.finite_execution),
      route->observed_raw_world, &lifecycle_event, std::nullopt);
}

std::optional<DirectTrackingFiniteExecution3D>
certifyDirectTrackingExecution3D(const ExecutionRouteSnapshot3D& current,
                                 DirectTrackingExecutionCertification3D certification) {
  const bool raw_mode = certification.observed_raw_world != nullptr;
  const bool static_mode = certification.static_world != nullptr;
  if (!current.valid() || !certification.identity.valid() ||
      certification.trajectory_revision == 0U || !finitePoint(certification.target) ||
      raw_mode == static_mode || certification.validation_policy == nullptr ||
      !certification.validation_policy->valid() ||
      certification.execution_input == nullptr ||
      !certification.execution_input->valid() ||
      !certification.execution_input->nominalStateAuthoritative() ||
      certification.latest_lidar_evidence == nullptr ||
      certification.valid_from_ns <= 0 ||
      certification.execution_input->effectiveStampNs() !=
          certification.valid_from_ns ||
      !executionInputFreshAt(*certification.execution_input,
                             *certification.validation_policy,
                             certification.valid_from_ns) ||
      !latestLidarEvidenceFreshAt(*certification.latest_lidar_evidence,
                                  *certification.validation_policy,
                                  certification.valid_from_ns) ||
      (certification.kind != FiniteExecutionKind3D::kNominal &&
       certification.kind != FiniteExecutionKind3D::kRetained) ||
      (raw_mode && !certification.observed_raw_world->valid()) ||
      (static_mode && !certification.static_world->valid()) ||
      (current.finite_execution.has_value() &&
       certification.trajectory_revision <=
           current.finite_execution->trajectory_revision) ||
      (current.direct_tracking_execution.has_value() &&
       certification.trajectory_revision <=
           current.direct_tracking_execution->trajectory_revision)) {
    return std::nullopt;
  }

  const mppi::FiniteHorizon& horizon = certification.horizon;
  const std::int64_t control_interval_ns = mppi::finitePathControlIntervalNanoseconds(
      certification.validation_policy->dynamics().dt_s);
  if (horizon.controls.empty() ||
      horizon.states.size() != horizon.controls.size() + 1U ||
      control_interval_ns <= 0 || !mppi::finiteHorizonHasTerminalRestState(horizon) ||
      !finiteHorizonDynamicallyConsistent(
          horizon, certification.execution_input->previousControl(),
          certification.validation_policy->dynamics()) ||
      !finiteStateNearlyEqual(horizon.states.front(),
                              certification.execution_input->state()) ||
      horizon.controls.size() >
          static_cast<std::uint64_t>(
              (std::numeric_limits<std::int64_t>::max() - certification.valid_from_ns) /
              control_interval_ns)) {
    return std::nullopt;
  }

  const ProprioceptiveFreeSpaceSeed3D* const free_space_seed =
      raw_mode ? optionalAddress(
                     certification.observed_raw_world->proprioceptiveFreeSpaceSeed())
               : nullptr;
  const LaunchSupportContact3D* const launch_support_contact =
      raw_mode
          ? optionalAddress(certification.observed_raw_world->launchSupportContact())
          : nullptr;
  const std::span<const Point3> latest_lidar_obstacle_points{
      certification.latest_lidar_evidence->hitPointsMapM()};
  mppi::FiniteExecutionPathWorld validation_world{
      .flight_envelope = &certification.validation_policy->flightEnvelope(),
      .dynamics = &certification.validation_policy->dynamics(),
      .altitude_envelope = &certification.validation_policy->altitudeEnvelope(),
      .footprint = &certification.validation_policy->sweptFootprint(),
      .static_occupancy =
          static_mode ? &certification.static_world->occupancy() : nullptr,
      .observed_occupancy =
          raw_mode ? &certification.observed_raw_world->occupancy() : nullptr,
      .require_known_free_space = static_mode,
      .proprioceptive_free_space_seed = free_space_seed,
      .launch_support_contact = launch_support_contact,
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points = latest_lidar_obstacle_points,
      .terminal_boundary = std::nullopt,
  };
  const std::vector<mppi::TimedExecutionPathPoint> validation_points =
      timedExecutionPathPoints(horizon,
                               certification.execution_input->previousControl(),
                               control_interval_ns);
  if (!mppi::validateCompleteFiniteExecutionPath(
           validation_points, certification.execution_input->previousControl(),
           validation_world)
           .accepted()) {
    return std::nullopt;
  }

  const ObservedSpaceValidationPolicy observed_policy =
      raw_mode ? ObservedSpaceValidationPolicy::kAllowUnknown
               : ObservedSpaceValidationPolicy::kRequireKnownFree;
  const std::uint64_t collision_policy_fingerprint = validationPolicyFingerprint(
      certification.validation_policy->sweptFootprint(), observed_policy,
      free_space_seed, launch_support_contact);
  const std::uint64_t validation_contract_fingerprint = validationContractFingerprint(
      validation_world, certification.execution_input->previousControl(),
      ValidationContractOwners3D{
          .observed_raw_world = certification.observed_raw_world.get(),
          .static_world = certification.static_world.get(),
          .latest_lidar_evidence = certification.latest_lidar_evidence.get(),
      });
  if (collision_policy_fingerprint == 0U || validation_contract_fingerprint == 0U) {
    return std::nullopt;
  }
  FiniteExecutionValidationLineage3D lineage{
      StaticFiniteExecutionValidationLineage3D{}};
  if (raw_mode) {
    lineage = ObservedRawFiniteExecutionValidationLineage3D{
        .producer_instance_id =
            certification.observed_raw_world->version().producer_instance_id,
        .validated_through_raw_revision =
            certification.observed_raw_world->version().revision,
        .validation_policy_fingerprint = collision_policy_fingerprint,
        .observed_world_content_fingerprint =
            certification.observed_raw_world->contentFingerprint(),
    };
  } else {
    lineage = StaticFiniteExecutionValidationLineage3D{
        .world_certificate = certification.static_world->certificate(),
        .static_occupancy_content_fingerprint =
            certification.static_world->contentFingerprint(),
        .validation_policy_fingerprint = collision_policy_fingerprint,
    };
  }

  const std::int64_t valid_until_ns =
      certification.valid_from_ns +
      static_cast<std::int64_t>(horizon.controls.size()) * control_interval_ns;
  DirectTrackingFiniteExecution3D execution{
      .identity = certification.identity,
      .trajectory_revision = certification.trajectory_revision,
      .source_snapshot_version = current.version,
      .source_navigation_revision = certification.execution_input->poseRevision(),
      .target = certification.target,
      .horizon =
          std::make_shared<const mppi::FiniteHorizon>(std::move(certification.horizon)),
      .observed_raw_world = std::move(certification.observed_raw_world),
      .static_world = std::move(certification.static_world),
      .validation_policy = std::move(certification.validation_policy),
      .execution_input = std::move(certification.execution_input),
      .latest_lidar_evidence = std::move(certification.latest_lidar_evidence),
      .valid_from_ns = certification.valid_from_ns,
      .valid_until_ns = valid_until_ns,
      .control_interval_ns = control_interval_ns,
      .kind = certification.kind,
      .validation_proof =
          FiniteExecutionValidationProof3D{
              .validation_contract_fingerprint = validation_contract_fingerprint,
              .lineage = lineage,
          },
  };
  execution.validation_proof.artifact_fingerprint =
      directTrackingExecutionArtifactFingerprint(execution);
  return execution.valid()
             ? std::optional<DirectTrackingFiniteExecution3D>{std::move(execution)}
             : std::nullopt;
}

} // namespace drone_city_nav
