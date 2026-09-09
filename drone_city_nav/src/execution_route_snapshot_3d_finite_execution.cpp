#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/execution_route_certification_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/occupied_collision_oracle_3d.hpp"
#include "drone_city_nav/proprioceptive_contact_seed_3d.hpp"
#include "drone_city_nav/trajectory_control_reference_3d.hpp"

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

std::optional<RouteAdherenceAssessment3D> validateExecutionProgressConnector(
    const CertifiedRouteSuffix3D& route, const Point3& execution_position,
    const std::shared_ptr<const VersionedExecutionInput3D>& execution_input,
    const std::shared_ptr<const VersionedObservedRawWorld3D>& observed_raw_world,
    const IndexedPointCloudView3D& latest_lidar_obstacle_points) {
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
          route.validation_policy->routeStationCreditSlackM() + kStationToleranceM);
  const std::array<MotionState3D, 2U> connector_states{
      MotionState3D{.x = static_cast<float>(route.progress.last_observed_position.x),
                    .y = static_cast<float>(route.progress.last_observed_position.y),
                    .z = static_cast<float>(route.progress.last_observed_position.z)},
      MotionState3D{.x = static_cast<float>(execution_position.x),
                    .y = static_cast<float>(execution_position.y),
                    .z = static_cast<float>(execution_position.z)},
  };
  const std::optional<double> cross_track_limit =
      route.validation_policy->routeCrossTrackConstraintsEnabled()
          ? std::optional<double>{kMaximumRouteCrossTrackM}
          : std::nullopt;
  RouteAdherenceAssessment3D adherence = validateFiniteRouteAdherence(
      *route.geometry, *route.decorations, connector_states, route.progress.station_m,
      certificate_view.suffix_start_station_m, connector_maximum_station_m,
      cross_track_limit, cross_track_limit,
      route.validation_policy->sweptFootprint().sweep_step_m, false,
      route.validation_policy->routeTrackingTubeConstraintsEnabled());
  if (!adherence.accepted) {
    return std::nullopt;
  }

  const MotionControl3D& current_execution_control = execution_input->previousControl();
  // The body stands upright: it is the body at every tilt the dynamics reach.
  constexpr FootprintBodyAxis previous_route_axis{};
  constexpr FootprintBodyAxis current_execution_axis{};
  const auto* const raw_certificate =
      std::get_if<ObservedRawRouteCertificate3D>(&route.certificate);
  const LaunchSupportContact3D* const launch_support_contact =
      raw_certificate != nullptr && observed_raw_world != nullptr
          ? optionalAddress(observed_raw_world->launchSupportContact())
          : nullptr;
  const std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_seed =
      seedWithRouteDeparture3D(
          proprioceptiveContactSeed3D(
              executionInputPosition(*execution_input), current_execution_control,
              route.validation_policy->sweptFootprint(),
              raw_certificate != nullptr && observed_raw_world != nullptr
                  ? std::addressof(observed_raw_world->occupancy())
                  : nullptr),
          &route);
  const OccupiedCollisionOracle3D collision_oracle{OccupiedCollisionWorld3D{
      .observed_occupancy = raw_certificate != nullptr && observed_raw_world != nullptr
                                ? std::addressof(observed_raw_world->occupancy())
                                : nullptr,
      .static_occupancy = raw_certificate == nullptr && route.static_world != nullptr
                              ? std::addressof(route.static_world->occupancy())
                              : nullptr,
      .planar_occupancy = nullptr,
      .raw_point_cloud = latest_lidar_obstacle_points,
      .launch_support_contact = launch_support_contact,
      .proprioceptive_free_space_seed = optionalAddress(proprioceptive_seed),
      .footprint = route.validation_policy->sweptFootprint(),
      .flight_envelope = route.validation_policy->flightEnvelope(),
  }};
  if ((raw_certificate != nullptr && observed_raw_world == nullptr) ||
      (raw_certificate == nullptr && route.static_world == nullptr) ||
      !collision_oracle
           .validateSegment(route.progress.last_observed_position, previous_route_axis,
                            execution_position, current_execution_axis)
           .clear()) {
    return std::nullopt;
  }
  return adherence;
}

[[nodiscard]] std::optional<double>
unboundSuccessorExecutionStation(const CertifiedRouteSuffix3D& route,
                                 const Point3& execution_position) noexcept {
  if (route.geometry == nullptr || route.geometry->route == nullptr ||
      route.geometry->route->empty()) {
    return std::nullopt;
  }
  const double planning_to_execution_travel_m =
      distance3D(route.progress.last_observed_position, execution_position);
  if (!std::isfinite(planning_to_execution_travel_m)) {
    return std::nullopt;
  }
  const CertificateView3D certificate_view = certificateView(route.certificate);
  const double maximum_station_m = std::min(
      certificate_view.certified_end_station_m,
      route.progress.station_m +
          kMaximumStationCreditPerTravel * planning_to_execution_travel_m +
          route.validation_policy->routeStationCreditSlackM() + kStationToleranceM);
  const RouteProjection3D projection = projectOntoRoute3DWithinStationWindow(
      *route.geometry->route, execution_position, route.progress.station_m,
      maximum_station_m);
  return projection.valid ? std::optional<double>{projection.station_m} : std::nullopt;
}

[[nodiscard]] RouteAdherenceAssessment3D
brakingRouteOwnershipBinding(const CertifiedRouteSuffix3D& route,
                             const std::span<const MotionState3D> states,
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
  const auto projection = [&](const MotionState3D& state) {
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
    const ExecutionPlan3D& current, const CertifiedRouteSuffix3D& target_route,
    FiniteExecutionCertification3D certification,
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_validation_world,
    const std::optional<double> atomic_plan_begin_station_m = std::nullopt) {
  const CertifiedRouteSuffix3D* const current_route = routePointer(current);
  const FiniteExecutionState3D* const current_execution = current.finiteExecution();
  const DirectTrackingFiniteExecution3D* const current_direct_execution =
      current.directTrackingExecution();
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
      (current_execution != nullptr &&
       certification.trajectory_revision <= current_execution->trajectory_revision) ||
      (current_direct_execution != nullptr &&
       certification.trajectory_revision <=
           current_direct_execution->trajectory_revision)) {
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
      current.phase() == ExecutionRoutePhase3D::kDirectTracking &&
      current_direct_execution != nullptr && current_route == nullptr &&
      current.routeGenerationHighWater() != std::numeric_limits<std::uint64_t>::max() &&
      target_route.identity.generation == current.routeGenerationHighWater() + 1U;
  const bool targets_initial_route =
      current_route == nullptr && current_execution == nullptr &&
      current_direct_execution == nullptr &&
      current.routeGenerationHighWater() != std::numeric_limits<std::uint64_t>::max() &&
      target_route.identity.generation == current.routeGenerationHighWater() + 1U;
  if (!targets_current_route && !targets_successor_route && !targets_direct_successor &&
      !targets_initial_route) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kTargetRelationRejected);
  }
  if (targets_current_route && target_route.progress.execution_input == nullptr) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kProgressRelationRejected);
  }

  const auto* const static_certificate =
      std::get_if<StaticRouteCertificate3D>(&target_route.certificate);
  const auto* const raw_certificate =
      std::get_if<ObservedRawRouteCertificate3D>(&target_route.certificate);
  const bool static_mode = static_certificate != nullptr;
  const bool raw_mode = raw_certificate != nullptr;
  const std::shared_ptr<const VersionedExecutionValidationPolicy3D>& policy =
      target_route.validation_policy;
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
      (raw_mode && (!rawWorldMatchesCertificate(observed_raw_validation_world,
                                                *raw_certificate, true) ||
                    target_route.static_world != nullptr))) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kEvidenceContractRejected);
  }

  const LaunchSupportContact3D* const launch_support_contact =
      raw_mode ? optionalAddress(observed_raw_validation_world->launchSupportContact())
               : nullptr;
  const std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_seed =
      seedWithRouteDeparture3D(
          proprioceptiveContactSeed3D(
              executionInputPosition(*certification.execution_input),
              certification.execution_input->previousControl(),
              policy->sweptFootprint(),
              raw_mode ? std::addressof(observed_raw_validation_world->occupancy())
                       : nullptr),
          &target_route);
  const std::uint64_t execution_collision_policy_fingerprint =
      validationPolicyFingerprint(policy->sweptFootprint(), launch_support_contact);
  if (execution_collision_policy_fingerprint == 0U) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kCollisionPolicyInvalid);
  }

  const FiniteMotionHorizon3D& validated_horizon = certification.horizon;
  const std::int64_t control_interval_ns =
      finitePathControlIntervalNanoseconds3D(policy->dynamics().dt_s);
  const MotionDynamicsConsistency3D horizon_dynamics =
      finiteMotionHorizonDynamicsConsistency3D(
          validated_horizon, certification.execution_input->previousControl(),
          policy->dynamics());
  if (validated_horizon.controls.empty() || validated_horizon.states.empty() ||
      validated_horizon.states.size() != validated_horizon.controls.size() + 1U ||
      control_interval_ns <= 0 ||
      horizon_dynamics != MotionDynamicsConsistency3D::kConsistent ||
      validated_horizon.controls.size() >
          static_cast<std::uint64_t>(
              (std::numeric_limits<std::int64_t>::max() - certification.valid_from_ns) /
              control_interval_ns)) {
    FiniteExecutionCertificationResult3D rejection = rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kHorizonContractRejected);
    rejection.dynamics_consistency = horizon_dynamics;
    return rejection;
  }

  const CertificateView3D certificate_view = certificateView(target_route.certificate);
  const MotionState3D& initial_state = validated_horizon.states.front();
  const Point3 initial_state_position{initial_state.x, initial_state.y,
                                      initial_state.z};
  if (!finiteStateNearlyEqual(initial_state, certification.execution_input->state())) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kInitialStateMismatch);
  }
  const IndexedPointCloudView3D latest_lidar_obstacle_points =
      certification.latest_lidar_evidence != nullptr
          ? certification.latest_lidar_evidence->indexedHitPoints()
          : IndexedPointCloudView3D{};
  double execution_begin_station_m = target_route.progress.station_m;
  if (target_route.progress.execution_input == nullptr) {
    // An asynchronous successor has not owned execution yet. Motion from its
    // planning pose to this state was owned by the preceding finite trajectory,
    // so establish only the new route's bounded forward station. The command
    // and braking horizons beginning at this exact state remain subject to the
    // complete swept-world and latest-lidar validation below.
    if (!targets_successor_route && !targets_direct_successor &&
        !targets_initial_route) {
      return rejectedFiniteExecution(
          FiniteExecutionCertificationStatus3D::kExecutionBindingRejected);
    }
    const std::optional<double> unbound_station =
        unboundSuccessorExecutionStation(target_route, initial_state_position);
    if (!unbound_station.has_value()) {
      return rejectedFiniteExecution(
          FiniteExecutionCertificationStatus3D::kExecutionBindingRejected);
    }
    execution_begin_station_m = *unbound_station;
  } else if (distance3D(initial_state_position,
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
                *target_route.geometry, *target_route.decorations,
                validated_horizon.states, execution_begin_station_m,
                certificate_view.suffix_start_station_m,
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

  const std::shared_ptr<const std::vector<ControlRouteSample3D>> mppi_reference =
      certifies_braking_execution
          ? nullptr
          : adaptTrajectoryControlReference3D(*target_route.geometry);
  if (!certifies_braking_execution && mppi_reference == nullptr) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kTerminalBoundaryInvalid);
  }
  FiniteExecutionPathWorld3D validation_world{
      .flight_envelope = &policy->flightEnvelope(),
      .dynamics = &policy->dynamics(),
      .altitude_envelope = &policy->altitudeEnvelope(),
      .footprint = &policy->sweptFootprint(),
      .static_occupancy =
          static_mode ? &target_route.static_world->occupancy() : nullptr,
      .observed_occupancy =
          raw_mode ? &observed_raw_validation_world->occupancy() : nullptr,
      .launch_support_contact = launch_support_contact,
      .proprioceptive_free_space_seed = optionalAddress(proprioceptive_seed),
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
          : makeValidationTerminalBoundary(terminal_boundary, target_route,
                                           *mppi_reference);
  if (policy->routeTrackingTubeConstraintsEnabled() && !certifies_braking_execution &&
      !validateTrackingTubeHandoffClearance(target_route, validated_horizon,
                                            begin_projection.station_m,
                                            validation_world)) {
    return rejectedFiniteExecution(
        FiniteExecutionCertificationStatus3D::kTrackingTubeHandoffRejected);
  }
  const std::vector<TimedExecutionPathPoint3D> validation_points =
      timedExecutionPathPoints(validated_horizon,
                               certification.execution_input->previousControl(),
                               control_interval_ns);
  const FiniteExecutionPathValidation3D validation =
      validateCompleteFiniteExecutionPath3D(
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
    const std::uint64_t policy_fingerprint = validationPolicyFingerprint(
        *validation_world.footprint, validation_world.launch_support_contact);
    if (policy_fingerprint == 0U ||
        observed_raw_validation_world->version().producer_instance_id !=
            raw_certificate->producer_instance_id ||
        observed_raw_validation_world->version().revision !=
            raw_certificate->validated_through_revision) {
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
        .observed_occupancy_content_fingerprint =
            observed_raw_validation_world->occupiedContentFingerprint(),
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
  const MotionState3D& terminal_state = validated_horizon.states.back();
  const Point3 terminal_position{terminal_state.x, terminal_state.y, terminal_state.z};
  const std::uint64_t validated_raw_revision = rawValidatedRevision(validation_lineage);
  FiniteExecutionState3D execution{
      .trajectory_revision = certification.trajectory_revision,
      .source_snapshot_version = current.version,
      .source_navigation_revision = certification.execution_input->poseRevision(),
      .source_route_instance_id = target_route.route_instance_id,
      .source_route_generation = target_route.identity.generation,
      .source_geometry_revision = target_route.geometry->compiled_trajectory_revision,
      .source_physical_route_fingerprint =
          target_route.geometry->physical_route_fingerprint,
      .certificate = target_route.certificate,
      .horizon = std::make_shared<const FiniteMotionHorizon3D>(
          std::move(certification.horizon)),
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
              .geometry_revision = target_route.geometry->compiled_trajectory_revision,
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
  if (!execution.validFor(&rebound_route)) {
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

std::optional<FiniteExecutionState3D>
certifyFiniteExecution3D(const ExecutionPlan3D& current,
                         const CertifiedRouteSuffix3D& target_route,
                         FiniteExecutionCertification3D certification) {
  return certifyFiniteExecution3DDetailed(current, target_route,
                                          std::move(certification))
      .execution;
}

FiniteExecutionCertificationResult3D
certifyFiniteExecution3DDetailed(const ExecutionPlan3D& current,
                                 const CertifiedRouteSuffix3D& target_route,
                                 FiniteExecutionCertification3D certification) {
  return certifyFiniteExecutionAgainstOwnedWorld3D(
      current, target_route, std::move(certification), target_route.observed_raw_world,
      std::nullopt);
}

FiniteExecutionPlanCertificationResult3D
certifyFiniteExecutionPlan3DDetailed(const ExecutionPlan3D& current,
                                     const CertifiedRouteSuffix3D& target_route,
                                     FiniteExecutionPlanCertification3D certification) {
  const std::array<FiniteMotionHorizon3D, 1U> tails{
      std::move(certification.braking_tail)};
  return certifyFiniteExecutionPlan3DDetailed(
      current, target_route, std::move(certification.command_horizon), tails);
}

FiniteExecutionPlanCertificationResult3D certifyFiniteExecutionPlan3DDetailed(
    const ExecutionPlan3D& current, const CertifiedRouteSuffix3D& target_route,
    FiniteExecutionCertification3D command_horizon,
    const std::span<const FiniteMotionHorizon3D> braking_tails) {
  FiniteExecutionPlanCertificationResult3D result;
  if ((command_horizon.kind != FiniteExecutionKind3D::kNominal &&
       command_horizon.kind != FiniteExecutionKind3D::kRetained) ||
      braking_tails.empty()) {
    return result;
  }
  const std::uint64_t trajectory_revision = command_horizon.trajectory_revision;
  const std::shared_ptr<const VersionedExecutionInput3D> execution_input =
      command_horizon.execution_input;
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence =
      command_horizon.latest_lidar_evidence;
  const std::int64_t valid_from_ns = command_horizon.valid_from_ns;
  result.command_horizon = certifyFiniteExecution3DDetailed(current, target_route,
                                                            std::move(command_horizon));
  if (!result.command_horizon.certified() ||
      !result.command_horizon.execution.has_value()) {
    return result;
  }
  for (const FiniteMotionHorizon3D& tail : braking_tails) {
    result.braking_tail = certifyFiniteExecutionAgainstOwnedWorld3D(
        current, target_route,
        FiniteExecutionCertification3D{
            .trajectory_revision = trajectory_revision,
            .horizon = tail,
            .execution_input = execution_input,
            .latest_lidar_evidence = latest_lidar_evidence,
            .valid_from_ns = valid_from_ns,
            .kind = FiniteExecutionKind3D::kEmergencyBrakeTail,
        },
        target_route.observed_raw_world,
        result.command_horizon.execution->begin_route_station_m);
    if (!result.braking_tail.certified() ||
        !result.braking_tail.execution.has_value()) {
      continue;
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
      continue;
    }
    result.plan = std::move(plan);
    return result;
  }
  return result;
}

std::optional<FiniteExecutionState3D>
certifyFiniteExecution3D(const ExecutionPlan3D& current,
                         FiniteExecutionCertification3D certification) {
  const CertifiedRouteSuffix3D* const route = routePointer(current);
  return route == nullptr
             ? std::nullopt
             : certifyFiniteExecution3D(current, *route, std::move(certification));
}

std::optional<DirectTrackingFiniteExecution3D>
certifyDirectTrackingExecution3D(const ExecutionPlan3D& current,
                                 DirectTrackingExecutionCertification3D certification) {
  const bool raw_mode = certification.observed_raw_world != nullptr;
  const bool static_mode = certification.static_world != nullptr;
  const FiniteExecutionState3D* const current_execution = current.finiteExecution();
  const DirectTrackingFiniteExecution3D* const current_direct_execution =
      current.directTrackingExecution();
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
      (current_execution != nullptr &&
       certification.trajectory_revision <= current_execution->trajectory_revision) ||
      (current_direct_execution != nullptr &&
       certification.trajectory_revision <=
           current_direct_execution->trajectory_revision)) {
    return std::nullopt;
  }

  const FiniteMotionHorizon3D& horizon = certification.horizon;
  const std::int64_t control_interval_ns = finitePathControlIntervalNanoseconds3D(
      certification.validation_policy->dynamics().dt_s);
  if (horizon.controls.empty() ||
      horizon.states.size() != horizon.controls.size() + 1U ||
      control_interval_ns <= 0 || !finiteMotionHorizonHasTerminalRestState3D(horizon) ||
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

  const LaunchSupportContact3D* const launch_support_contact =
      raw_mode
          ? optionalAddress(certification.observed_raw_world->launchSupportContact())
          : nullptr;
  const std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_seed =
      proprioceptiveContactSeed3D(
          executionInputPosition(*certification.execution_input),
          certification.execution_input->previousControl(),
          certification.validation_policy->sweptFootprint(),
          raw_mode ? std::addressof(certification.observed_raw_world->occupancy())
                   : nullptr);
  const IndexedPointCloudView3D latest_lidar_obstacle_points =
      certification.latest_lidar_evidence->indexedHitPoints();
  FiniteExecutionPathWorld3D validation_world{
      .flight_envelope = &certification.validation_policy->flightEnvelope(),
      .dynamics = &certification.validation_policy->dynamics(),
      .altitude_envelope = &certification.validation_policy->altitudeEnvelope(),
      .footprint = &certification.validation_policy->sweptFootprint(),
      .static_occupancy =
          static_mode ? &certification.static_world->occupancy() : nullptr,
      .observed_occupancy =
          raw_mode ? &certification.observed_raw_world->occupancy() : nullptr,
      .launch_support_contact = launch_support_contact,
      .proprioceptive_free_space_seed = optionalAddress(proprioceptive_seed),
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points = latest_lidar_obstacle_points,
      .terminal_boundary = std::nullopt,
  };
  const std::vector<TimedExecutionPathPoint3D> validation_points =
      timedExecutionPathPoints(horizon,
                               certification.execution_input->previousControl(),
                               control_interval_ns);
  if (!validateCompleteFiniteExecutionPath3D(
           validation_points, certification.execution_input->previousControl(),
           validation_world)
           .accepted()) {
    return std::nullopt;
  }

  const std::uint64_t collision_policy_fingerprint = validationPolicyFingerprint(
      certification.validation_policy->sweptFootprint(), launch_support_contact);
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
        .observed_occupancy_content_fingerprint =
            certification.observed_raw_world->occupiedContentFingerprint(),
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
      .horizon = std::make_shared<const FiniteMotionHorizon3D>(
          std::move(certification.horizon)),
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
