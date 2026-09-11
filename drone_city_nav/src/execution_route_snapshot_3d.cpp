#include "drone_city_nav/execution_horizon_timing.hpp"
#include "drone_city_nav/execution_route_certification_3d.hpp"
#include "drone_city_nav/motion_altitude_envelope_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"
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

[[nodiscard]] bool endpointSpeedProfileMatchesSemantics3D(
    const CompiledTrajectory3D& trajectory,
    const MaterializedRouteProposal3D& proposal) noexcept {
  if (trajectory.route == nullptr || trajectory.route->empty()) {
    return false;
  }
  const RouteEndpointSemantics3D semantics = routeEndpointSemantics3D(
      proposal.reaches_mission_goal, !proposal.objective.continuous_tracking);
  constexpr double kTerminalSpeedToleranceMps{1.0e-4};
  const double terminal_speed_mps = trajectory.route->back().reference_speed_mps;
  return trajectory.endpoint_semantics == semantics &&
         (routeEndpointHasTerminalStop3D(semantics)
              ? std::abs(terminal_speed_mps) <= kTerminalSpeedToleranceMps
              : terminal_speed_mps > kTerminalSpeedToleranceMps);
}

} // namespace

bool CertifiedRouteProgress3D::valid() const noexcept {
  if (route_generation == 0U || geometry_revision == 0U || !std::isfinite(station_m) ||
      station_m < 0.0 || !finitePoint(last_observed_position)) {
    return false;
  }
  if (execution_input == nullptr) {
    return true;
  }
  const Point3 owned_position = executionInputPosition(*execution_input);
  return execution_input->valid() && execution_input->nominalStateAuthoritative() &&
         last_observed_position.x == owned_position.x &&
         last_observed_position.y == owned_position.y &&
         last_observed_position.z == owned_position.z;
}

bool StaticRouteCertificate3D::validFor(
    const RouteInstanceId3D expected_route_instance_id,
    const ActivatedRouteIdentity3D& identity,
    const std::uint64_t expected_geometry_revision,
    const std::uint64_t expected_physical_route_fingerprint,
    const double route_end_station_m) const noexcept {
  return route_instance_id.valid() && route_instance_id == expected_route_instance_id &&
         route_generation == identity.generation && route_generation != 0U &&
         geometry_revision == expected_geometry_revision && geometry_revision != 0U &&
         physical_route_fingerprint == expected_physical_route_fingerprint &&
         physical_route_fingerprint != 0U &&
         static_occupancy_content_fingerprint != 0U &&
         validation_policy_fingerprint != 0U &&
         execution_validation_policy_fingerprint != 0U &&
         route_decorations_revision != 0U && passage_volume_config_fingerprint != 0U &&
         geometry_derivation_occupancy_content_fingerprint ==
             static_occupancy_content_fingerprint &&
         sameWorldCertificate(world_certificate, identity.proposal.validated_world) &&
         validStationInterval(suffix_start_station_m, certified_end_station_m,
                              route_end_station_m);
}

bool ObservedRawRouteCertificate3D::validFor(
    const RouteInstanceId3D expected_route_instance_id,
    const ActivatedRouteIdentity3D& identity,
    const std::uint64_t expected_geometry_revision,
    const std::uint64_t expected_physical_route_fingerprint,
    const double route_end_station_m) const noexcept {
  return route_instance_id.valid() && route_instance_id == expected_route_instance_id &&
         route_generation == identity.generation && route_generation != 0U &&
         geometry_revision == expected_geometry_revision && geometry_revision != 0U &&
         physical_route_fingerprint == expected_physical_route_fingerprint &&
         physical_route_fingerprint != 0U &&
         producer_instance_id ==
             identity.proposal.validated_world.producer_instance_id &&
         producer_instance_id != 0U &&
         validated_through_revision >=
             identity.proposal.validated_world.raw_validated_through_revision &&
         validation_policy_fingerprint != 0U &&
         execution_validation_policy_fingerprint != 0U &&
         observed_world_content_fingerprint != 0U && route_decorations_revision != 0U &&
         passage_volume_config_fingerprint != 0U &&
         geometry_derivation_occupancy_content_fingerprint != 0U &&
         validStationInterval(suffix_start_station_m, certified_end_station_m,
                              route_end_station_m);
}

bool compiledTrajectoryValid3D(const CompiledTrajectory3D& geometry,
                               const ActivatedRouteIdentity3D& identity) noexcept {
  return compiledTrajectoryResourcesValid3D(geometry, identity.generation) &&
         identity.generation != 0U && identity.proposal.route_fingerprint != 0U &&
         geometry.materialized_route_fingerprint ==
             identity.proposal.route_fingerprint &&
         geometry.route->size() == identity.proposal.route_sample_count &&
         geometry.compiled_trajectory_revision != 0U &&
         geometry.compiled_trajectory_revision ==
             compiledTrajectoryRevision3D(geometry) &&
         endpointSpeedProfileMatchesSemantics3D(geometry, identity.proposal);
}

bool CertifiedRouteSuffix3D::valid() const noexcept {
  const std::optional<ActiveIntent3D> proposal_intent =
      activeIntent3D(identity.proposal);
  if (!route_instance_id.valid() || !owner.valid() || !proposal_intent.has_value() ||
      !sameActiveIntent3D(owner.active_intent, *proposal_intent) ||
      geometry == nullptr || decorations == nullptr || !progress.valid() ||
      (parent_route_instance_id.has_value() &&
       (!parent_route_instance_id->valid() ||
        *parent_route_instance_id == route_instance_id)) ||
      validation_policy == nullptr || !validation_policy->valid() ||
      (progress.execution_input != nullptr &&
       !executionInputFreshAt(*progress.execution_input, *validation_policy,
                              progress.execution_input->effectiveStampNs())) ||
      !footprintConservativelyContains(decorations->passage_volume_config.footprint,
                                       validation_policy->sweptFootprint()) ||
      !footprintConservativelyContains(
          geometry->tracking_error_tube->physical_footprint,
          validation_policy->sweptFootprint()) ||
      progress.route_generation != identity.generation ||
      progress.geometry_revision != geometry->compiled_trajectory_revision ||
      !compiledTrajectoryValid3D(*geometry, identity) || continuity_id == 0U ||
      !routeDecorationsValid3D(*decorations, *geometry, identity.generation) ||
      continuity_id !=
          routeContinuityId3D(identity.proposal.intent, continuity_lineage) ||
      planned_endpoint_semantics !=
          routeEndpointSemantics3D(identity.proposal.reaches_mission_goal,
                                   !identity.proposal.objective.continuous_tracking)) {
    return false;
  }
  const double route_end_station_m = endStationM();
  const auto* const static_certificate =
      std::get_if<StaticRouteCertificate3D>(&certificate);
  const auto* const raw_certificate =
      std::get_if<ObservedRawRouteCertificate3D>(&certificate);
  const bool certificate_valid =
      static_certificate != nullptr
          ? static_certificate->validFor(
                route_instance_id, identity, geometry->compiled_trajectory_revision,
                geometry->physical_route_fingerprint, route_end_station_m) &&
                observed_raw_world == nullptr &&
                staticWorldMatchesCertificate(static_world, *static_certificate)
          : raw_certificate != nullptr &&
                raw_certificate->validFor(
                    route_instance_id, identity, geometry->compiled_trajectory_revision,
                    geometry->physical_route_fingerprint, route_end_station_m) &&
                static_world == nullptr &&
                rawWorldMatchesCertificate(observed_raw_world, *raw_certificate, true);
  if (!certificate_valid) {
    return false;
  }
  const CertificateView3D certificate_view = certificateView(certificate);
  const std::uint64_t route_decorations_revision =
      routeDecorationsRevision3D(*decorations);
  const std::uint64_t passage_config_fingerprint =
      passageVolumeConfigFingerprint(decorations->passage_volume_config);
  return certificate_view.physical_route_fingerprint ==
             geometry->physical_route_fingerprint &&
         certificate_view.execution_validation_policy_fingerprint ==
             validation_policy->contentFingerprint() &&
         route_decorations_revision != 0U &&
         certificate_view.route_decorations_revision == route_decorations_revision &&
         passage_config_fingerprint != 0U &&
         certificate_view.passage_volume_config_fingerprint ==
             passage_config_fingerprint &&
         certificate_view.geometry_derivation_occupancy_content_fingerprint ==
             (certificate_view.observed_raw
                  ? observed_raw_world->occupiedContentFingerprint()
                  : static_world->contentFingerprint()) &&
         progress.station_m + kStationToleranceM >=
             certificate_view.suffix_start_station_m &&
         progress.station_m <=
             certificate_view.certified_end_station_m + kStationToleranceM;
}

double CertifiedRouteSuffix3D::endStationM() const noexcept {
  return geometry != nullptr && geometry->route != nullptr && !geometry->route->empty()
             ? geometry->route->back().station_m
             : 0.0;
}

double CertifiedRouteSuffix3D::remainingM() const noexcept {
  return std::max(0.0, endStationM() - progress.station_m);
}

bool FiniteRouteTerminalBoundary3D::valid(
    const double route_end_station_m) const noexcept {
  const double forward_norm =
      std::sqrt(forward.x * forward.x + forward.y * forward.y + forward.z * forward.z);
  return finitePoint(endpoint) && finiteVector(forward) &&
         std::isfinite(forward_norm) && forward_norm > kStationToleranceM &&
         std::isfinite(tolerance_m) && tolerance_m > 0.0 &&
         std::isfinite(activation_distance_m) && activation_distance_m >= 0.0 &&
         std::isfinite(maximum_cross_track_m) && maximum_cross_track_m > 0.0 &&
         std::isfinite(initial_route_station_m) &&
         std::isfinite(activation_route_station_m) && initial_route_station_m >= 0.0 &&
         activation_route_station_m + kStationToleranceM >= initial_route_station_m &&
         activation_route_station_m <= route_end_station_m + kStationToleranceM;
}

bool FiniteExecutionState3D::validFor(
    const CertifiedRouteSuffix3D* const route) const noexcept {
  const bool horizon_counts_valid =
      horizon != nullptr &&
      horizon->nominal_prefix_control_count <= horizon->controls.size() &&
      horizon->arrival_control_count <= horizon->controls.size() &&
      horizon->nominal_prefix_control_count + horizon->arrival_control_count ==
          horizon->controls.size();
  const bool duration_fits =
      horizon != nullptr && control_interval_ns > 0 && valid_from_ns > 0 &&
      horizon->controls.size() <=
          static_cast<std::uint64_t>(
              (std::numeric_limits<std::int64_t>::max() - valid_from_ns) /
              control_interval_ns);
  const std::int64_t expected_valid_until_ns =
      duration_fits
          ? valid_from_ns + static_cast<std::int64_t>(horizon->controls.size()) *
                                control_interval_ns
          : 0;
  if (!knownFiniteExecutionKind(kind) || trajectory_revision == 0U ||
      source_snapshot_version == 0U || source_navigation_revision == 0U ||
      !source_route_instance_id.valid() || source_route_generation == 0U ||
      source_geometry_revision == 0U || source_physical_route_fingerprint == 0U ||
      horizon == nullptr || horizon->controls.empty() ||
      horizon->states.size() != horizon->controls.size() + 1U ||
      !horizon_counts_valid || !duration_fits ||
      !finiteMotionHorizonHasTerminalRestState3D(*horizon) || valid_from_ns < 0 ||
      valid_until_ns != expected_valid_until_ns ||
      !std::isfinite(begin_route_station_m) || begin_route_station_m < 0.0 ||
      !certificateValidForSource(certificate, source_route_instance_id,
                                 source_route_generation, source_geometry_revision,
                                 source_physical_route_fingerprint)) {
    return false;
  }
  // A braking tail follows the command horizon for its nominal prefix and
  // brakes to rest from there: the earliest stop along the horizon the world
  // admitted. A prefix of zero is the stop that begins at once.
  if (kind == FiniteExecutionKind3D::kEmergencyBrakeTail &&
      horizon->nominal_prefix_control_count + horizon->arrival_control_count !=
          horizon->controls.size()) {
    return false;
  }
  if (!std::all_of(horizon->states.begin(), horizon->states.end(), finiteState) ||
      !std::all_of(horizon->controls.begin(), horizon->controls.end(), finiteControl) ||
      !terminalStopBoundaryValid(stop_boundary, *this) ||
      validation_proof.artifact_fingerprint == 0U ||
      validation_proof.validation_contract_fingerprint == 0U ||
      !validationLineageValidForCertificate(validation_proof.lineage, certificate) ||
      !finiteWorldOwnerMatchesProof(*this) || execution_input == nullptr ||
      !finiteStateNearlyEqual(horizon->states.front(), execution_input->state()) ||
      stop_boundary.position_tolerance_m != kFiniteStopPositionToleranceM ||
      validation_proof.artifact_fingerprint !=
          finiteExecutionArtifactFingerprint(*this)) {
    return false;
  }
  const CertificateView3D certificate_view = certificateView(certificate);
  if (begin_route_station_m + kStationToleranceM <
          certificate_view.suffix_start_station_m ||
      begin_route_station_m >
          certificate_view.certified_end_station_m + kStationToleranceM ||
      !terminal_boundary.has_value() ||
      !terminal_boundary->valid(certificate_view.certified_end_station_m)) {
    return false;
  }
  if (route == nullptr) {
    return kind != FiniteExecutionKind3D::kNominal;
  }
  if (!route->valid() || source_route_instance_id != route->route_instance_id ||
      source_route_generation != route->identity.generation ||
      validation_policy == nullptr ||
      validation_policy->contentFingerprint() !=
          route->validation_policy->contentFingerprint() ||
      source_geometry_revision != route->geometry->compiled_trajectory_revision ||
      source_physical_route_fingerprint !=
          route->geometry->physical_route_fingerprint ||
      begin_route_station_m > route->progress.station_m + kExecutionBindingToleranceM ||
      (!revalidation_required &&
       stop_boundary.station_m + kStationToleranceM < route->progress.station_m) ||
      !(revalidation_required
            ? certificateEligibleForRevalidation(certificate, route->certificate)
            : certificateNotNewerThan(certificate, route->certificate))) {
    return false;
  }
  const RouteSample3D expected_stop =
      sampleRoute3DAtStation(*route->geometry->route, stop_boundary.station_m);
  // A strict route corridor is optional because the complete finite horizon is
  // independently swept against its immutable world and latest lidar evidence.
  if (kind != FiniteExecutionKind3D::kEmergencyBrakeTail &&
      validation_policy->routeCrossTrackConstraintsEnabled() &&
      distance3D(stop_boundary.position, expected_stop.position) >
          kMaximumRouteCrossTrackM) {
    return false;
  }
  return sameTerminalBoundary(terminal_boundary, canonicalFiniteRouteTerminalBoundary(
                                                     *route, begin_route_station_m));
}

bool FiniteExecutionPlan3D::validFor(
    const CertifiedRouteSuffix3D& route) const noexcept {
  const bool shared_binding =
      command_horizon.trajectory_revision == braking_tail.trajectory_revision &&
      command_horizon.source_snapshot_version == braking_tail.source_snapshot_version &&
      command_horizon.source_navigation_revision ==
          braking_tail.source_navigation_revision &&
      command_horizon.source_route_instance_id ==
          braking_tail.source_route_instance_id &&
      command_horizon.source_route_generation == braking_tail.source_route_generation &&
      command_horizon.source_geometry_revision ==
          braking_tail.source_geometry_revision &&
      command_horizon.source_physical_route_fingerprint ==
          braking_tail.source_physical_route_fingerprint &&
      command_horizon.execution_input == braking_tail.execution_input &&
      command_horizon.latest_lidar_evidence == braking_tail.latest_lidar_evidence &&
      command_horizon.observed_raw_world == braking_tail.observed_raw_world &&
      command_horizon.static_world == braking_tail.static_world &&
      command_horizon.validation_policy == braking_tail.validation_policy &&
      command_horizon.valid_from_ns == braking_tail.valid_from_ns &&
      command_horizon.control_interval_ns == braking_tail.control_interval_ns &&
      command_horizon.revalidation_required == braking_tail.revalidation_required &&
      sameCertificate(command_horizon.certificate, braking_tail.certificate) &&
      std::abs(command_horizon.begin_route_station_m -
               braking_tail.begin_route_station_m) <= kStationToleranceM;
  return (command_horizon.kind == FiniteExecutionKind3D::kNominal ||
          command_horizon.kind == FiniteExecutionKind3D::kRetained) &&
         braking_tail.kind == FiniteExecutionKind3D::kEmergencyBrakeTail &&
         shared_binding && command_horizon.validFor(&route) &&
         braking_tail.validFor(&route) &&
         braking_tail.valid_until_ns <= command_horizon.valid_until_ns &&
         braking_tail.stop_boundary.station_m <=
             command_horizon.stop_boundary.station_m + kStationToleranceM;
}

bool DirectTrackingOwnerIdentity3D::valid() const noexcept {
  return mission_epoch != 0U && assignment_generation != 0U &&
         target_detection_id != 0U && target_track_id != 0U &&
         objective_sample_sequence != 0U && line_of_sight_generation != 0U;
}

bool DirectTrackingFiniteExecution3D::valid() const noexcept {
  const bool horizon_counts_valid =
      horizon != nullptr &&
      horizon->nominal_prefix_control_count <= horizon->controls.size() &&
      horizon->arrival_control_count <= horizon->controls.size() &&
      horizon->nominal_prefix_control_count + horizon->arrival_control_count ==
          horizon->controls.size();
  const bool duration_fits =
      horizon != nullptr && control_interval_ns > 0 && valid_from_ns > 0 &&
      horizon->controls.size() <=
          static_cast<std::uint64_t>(
              (std::numeric_limits<std::int64_t>::max() - valid_from_ns) /
              control_interval_ns);
  const std::int64_t expected_valid_until_ns =
      duration_fits
          ? valid_from_ns + static_cast<std::int64_t>(horizon->controls.size()) *
                                control_interval_ns
          : 0;
  return identity.valid() && trajectory_revision != 0U &&
         source_snapshot_version != 0U && source_navigation_revision != 0U &&
         finitePoint(target) && horizon != nullptr && !horizon->controls.empty() &&
         horizon->states.size() == horizon->controls.size() + 1U &&
         horizon_counts_valid && duration_fits &&
         finiteMotionHorizonHasTerminalRestState3D(*horizon) &&
         valid_until_ns == expected_valid_until_ns &&
         (kind == FiniteExecutionKind3D::kNominal ||
          kind == FiniteExecutionKind3D::kRetained) &&
         std::all_of(horizon->states.begin(), horizon->states.end(), finiteState) &&
         std::all_of(horizon->controls.begin(), horizon->controls.end(),
                     finiteControl) &&
         execution_input != nullptr &&
         finiteStateNearlyEqual(horizon->states.front(), execution_input->state()) &&
         validation_proof.artifact_fingerprint != 0U &&
         validation_proof.validation_contract_fingerprint != 0U &&
         directTrackingWorldOwnerMatchesProof(*this) &&
         validation_proof.artifact_fingerprint ==
             directTrackingExecutionArtifactFingerprint(*this);
}

bool StationaryExecutionHold3D::valid() const noexcept {
  const bool terminal_origin =
      origin == StationaryExecutionHoldOrigin3D::kTerminalExecution;
  const bool capture_origin =
      origin == StationaryExecutionHoldOrigin3D::kStationaryCaptureRearm;
  if (hold_id == 0U || (!terminal_origin && !capture_origin) ||
      (terminal_origin != (source_trajectory_revision != 0U)) ||
      !finitePoint(position) || terminal_execution_input == nullptr ||
      !terminal_execution_input->valid() ||
      (terminal_origin
           ? !terminal_execution_input->nominalStateAuthoritative()
           : !(terminal_execution_input->stationaryCaptureStateAuthoritative() ||
               terminal_execution_input->nominalStateAuthoritative())) ||
      validation_policy == nullptr || !validation_policy->valid() ||
      latest_lidar_evidence == nullptr || !latest_lidar_evidence->valid() ||
      !executionInputFreshAt(*terminal_execution_input, *validation_policy,
                             terminal_execution_input->effectiveStampNs()) ||
      !latestLidarEvidenceFreshAt(*latest_lidar_evidence, *validation_policy,
                                  terminal_execution_input->effectiveStampNs()) ||
      (observed_raw_world == nullptr) == (static_world == nullptr) ||
      (observed_raw_world != nullptr && !observed_raw_world->valid()) ||
      (static_world != nullptr && !static_world->valid())) {
    return false;
  }
  const MotionState3D& state = terminal_execution_input->state();
  const MotionControl3D& control = terminal_execution_input->previousControl();
  const Point3 actual_position{state.x, state.y, state.z};
  return insideFlightEnvelope(position, validation_policy->flightEnvelope()) &&
         motionAltitudeEnvelopeDynamicallyRecoverable3D(
             state, control, validation_policy->dynamics(),
             validation_policy->altitudeEnvelope()) &&
         distance3D(position, actual_position) <=
             kStationaryExecutionHoldPositionToleranceM &&
         std::hypot(std::hypot(state.vx, state.vy), state.vz) <=
             kStationaryExecutionHoldSpeedToleranceMps &&
         std::abs(state.yaw_rate) <= kStationaryExecutionHoldYawRateToleranceRadps &&
         stationaryHoldRawSafe(position, *terminal_execution_input,
                               observed_raw_world.get(), static_world.get(),
                               *validation_policy, *latest_lidar_evidence);
}

ExecutionRoutePhase3D ExecutionPlan3D::phase() const noexcept {
  if (std::holds_alternative<FollowingPlan3D>(state)) {
    return ExecutionRoutePhase3D::kFollowing;
  }
  if (std::holds_alternative<DirectTrackingPlan3D>(state)) {
    return ExecutionRoutePhase3D::kDirectTracking;
  }
  if (std::holds_alternative<StopPlan3D>(state)) {
    return ExecutionRoutePhase3D::kStopping;
  }
  if (std::holds_alternative<StationaryHoldPlan3D>(state)) {
    return ExecutionRoutePhase3D::kStopped;
  }
  if (std::holds_alternative<AwaitingSuccessorPlan3D>(state)) {
    return ExecutionRoutePhase3D::kAwaitingSuccessor;
  }
  return ExecutionRoutePhase3D::kRevoked;
}

const CertifiedRouteSuffix3D* ExecutionPlan3D::route() const noexcept {
  if (const auto* following = std::get_if<FollowingPlan3D>(&state)) {
    return std::addressof(following->route);
  }
  if (const auto* stationary = std::get_if<StationaryHoldPlan3D>(&state)) {
    const auto* certified =
        std::get_if<CertifiedTerminalHoldPlan3D>(&stationary->owner);
    return certified != nullptr ? std::addressof(certified->route) : nullptr;
  }
  const auto* awaiting = std::get_if<AwaitingSuccessorPlan3D>(&state);
  if (awaiting == nullptr) {
    return nullptr;
  }
  if (const auto* suspended = std::get_if<SuspendedRoutePlan3D>(&awaiting->owner)) {
    return std::addressof(suspended->route);
  }
  const auto* continuation = std::get_if<ContinuationStopPlan3D>(&awaiting->owner);
  return continuation != nullptr ? std::addressof(continuation->route) : nullptr;
}

const FiniteExecutionState3D* ExecutionPlan3D::finiteExecution() const noexcept {
  if (const auto* following = std::get_if<FollowingPlan3D>(&state)) {
    return std::addressof(following->execution.command_horizon);
  }
  if (const auto* stationary = std::get_if<StationaryHoldPlan3D>(&state)) {
    const auto* certified =
        std::get_if<CertifiedTerminalHoldPlan3D>(&stationary->owner);
    return certified != nullptr ? std::addressof(certified->execution.command_horizon)
                                : nullptr;
  }
  const auto* awaiting = std::get_if<AwaitingSuccessorPlan3D>(&state);
  const auto* continuation = awaiting != nullptr
                                 ? std::get_if<ContinuationStopPlan3D>(&awaiting->owner)
                                 : nullptr;
  return continuation != nullptr
             ? std::addressof(continuation->execution.command_horizon)
             : nullptr;
}

const FiniteExecutionState3D* ExecutionPlan3D::brakingFallback() const noexcept {
  if (const auto* following = std::get_if<FollowingPlan3D>(&state)) {
    return std::addressof(following->execution.braking_tail);
  }
  if (const auto* stationary = std::get_if<StationaryHoldPlan3D>(&state)) {
    const auto* certified =
        std::get_if<CertifiedTerminalHoldPlan3D>(&stationary->owner);
    return certified != nullptr ? std::addressof(certified->execution.braking_tail)
                                : nullptr;
  }
  const auto* awaiting = std::get_if<AwaitingSuccessorPlan3D>(&state);
  const auto* continuation = awaiting != nullptr
                                 ? std::get_if<ContinuationStopPlan3D>(&awaiting->owner)
                                 : nullptr;
  return continuation != nullptr ? std::addressof(continuation->execution.braking_tail)
                                 : nullptr;
}

const DirectTrackingFiniteExecution3D*
ExecutionPlan3D::directTrackingExecution() const noexcept {
  const auto* direct = std::get_if<DirectTrackingPlan3D>(&state);
  return direct != nullptr ? std::addressof(direct->execution) : nullptr;
}

const StopExecution3D* ExecutionPlan3D::stopExecution() const noexcept {
  const auto* stopping = std::get_if<StopPlan3D>(&state);
  return stopping != nullptr ? std::addressof(stopping->execution) : nullptr;
}

const CertifiedRouteSuffix3D* ExecutionPlan3D::suspendedRoute() const noexcept {
  const auto* stopping = std::get_if<StopPlan3D>(&state);
  return stopping != nullptr && stopping->suspended_route.valid()
             ? std::addressof(stopping->suspended_route)
             : nullptr;
}

const StationaryExecutionHold3D* ExecutionPlan3D::stationaryHold() const noexcept {
  const auto* stationary = std::get_if<StationaryHoldPlan3D>(&state);
  return stationary != nullptr
             ? std::get_if<StationaryExecutionHold3D>(&stationary->owner)
             : nullptr;
}

std::uint64_t ExecutionPlan3D::ownerTrajectoryRevision() const noexcept {
  std::uint64_t revision{0U};
  if (const FiniteExecutionState3D* const finite = finiteExecution()) {
    revision = std::max(revision, finite->trajectory_revision);
  }
  if (const FiniteExecutionState3D* const braking = brakingFallback()) {
    revision = std::max(revision, braking->trajectory_revision);
  }
  if (const DirectTrackingFiniteExecution3D* const direct = directTrackingExecution()) {
    revision = std::max(revision, direct->trajectory_revision);
  }
  if (const StopExecution3D* const stop = stopExecution()) {
    revision = std::max(revision, stop->trajectory_revision);
  }
  if (const StationaryExecutionHold3D* const hold = stationaryHold()) {
    revision = std::max(revision, hold->source_trajectory_revision);
  }
  return revision;
}

bool ExecutionPlan3D::valid() const noexcept {
  if (version == 0U || execution_owner_epoch == 0U || state.valueless_by_exception()) {
    return false;
  }
  const CertifiedRouteSuffix3D* const owned_route = route();
  if (owned_route != nullptr &&
      (!owned_route->valid() || owned_route->progress.execution_input == nullptr ||
       (route_generation_high_water != 0U &&
        route_generation_high_water < owned_route->identity.generation))) {
    return false;
  }

  const auto route_execution_valid =
      [this](const CertifiedRouteSuffix3D& candidate_route,
             const FiniteExecutionPlan3D& execution) noexcept {
        if (!execution.validFor(candidate_route) ||
            execution.command_horizon.source_snapshot_version >= version ||
            execution.braking_tail.source_snapshot_version >= version) {
          return false;
        }
        const ExecutionInputProgressRelation3D command_relation =
            executionInputProgressRelation(*candidate_route.progress.execution_input,
                                           *execution.command_horizon.execution_input);
        const ExecutionInputProgressRelation3D braking_relation =
            executionInputProgressRelation(*candidate_route.progress.execution_input,
                                           *execution.braking_tail.execution_input);
        return command_relation != ExecutionInputProgressRelation3D::kInvalid &&
               braking_relation != ExecutionInputProgressRelation3D::kInvalid &&
               (execution.command_horizon.revalidation_required ||
                command_relation == ExecutionInputProgressRelation3D::kReplay) &&
               (execution.braking_tail.revalidation_required ||
                braking_relation == ExecutionInputProgressRelation3D::kReplay);
      };
  const auto certified_endpoint_stop =
      [&route_execution_valid](const CertifiedRouteSuffix3D& candidate_route,
                               const FiniteExecutionPlan3D& execution) noexcept {
        return candidate_route.remainingM() <= kCompletionStationToleranceM &&
               execution.command_horizon.kind == FiniteExecutionKind3D::kNominal &&
               route_execution_valid(candidate_route, execution) &&
               !execution.command_horizon.revalidation_required &&
               execution.command_horizon.terminal_boundary.has_value() &&
               execution.command_horizon.stop_boundary.station_m +
                       kCompletionStationToleranceM >=
                   candidate_route.endStationM();
      };

  if (const auto* plan = std::get_if<FollowingPlan3D>(&state)) {
    return route_execution_valid(plan->route, plan->execution) &&
           (plan->execution.command_horizon.kind == FiniteExecutionKind3D::kNominal ||
            plan->execution.command_horizon.kind == FiniteExecutionKind3D::kRetained) &&
           !finiteExecutionValidatedAgainstNewerRawWorld(
               plan->execution.command_horizon);
  }
  if (const auto* plan = std::get_if<DirectTrackingPlan3D>(&state)) {
    return plan->execution.valid() && plan->execution.source_snapshot_version < version;
  }
  if (const auto* plan = std::get_if<StopPlan3D>(&state)) {
    return plan->execution.valid() && plan->execution.source_snapshot_version < version;
  }
  if (const auto* plan = std::get_if<StationaryHoldPlan3D>(&state)) {
    if (plan->owner.valueless_by_exception()) {
      return false;
    }
    if (const auto* hold = std::get_if<StationaryExecutionHold3D>(&plan->owner)) {
      return hold->valid() && hold->hold_id == execution_owner_epoch;
    }
    const auto* certified = std::get_if<CertifiedTerminalHoldPlan3D>(&plan->owner);
    return certified != nullptr &&
           certified->route.planned_endpoint_semantics !=
               RouteEndpointSemantics3D::kContinuation &&
           certified_endpoint_stop(certified->route, certified->execution);
  }
  if (const auto* plan = std::get_if<AwaitingSuccessorPlan3D>(&state)) {
    if (plan->owner.valueless_by_exception()) {
      return false;
    }
    if (std::holds_alternative<EmptyAwaitingSuccessorPlan3D>(plan->owner)) {
      return true;
    }
    if (const auto* suspended = std::get_if<SuspendedRoutePlan3D>(&plan->owner)) {
      return suspended->route.valid() &&
             suspended->route.progress.execution_input != nullptr;
    }
    const auto* continuation = std::get_if<ContinuationStopPlan3D>(&plan->owner);
    return continuation != nullptr &&
           continuation->route.planned_endpoint_semantics ==
               RouteEndpointSemantics3D::kContinuation &&
           certified_endpoint_stop(continuation->route, continuation->execution);
  }
  return std::holds_alternative<RevokedPlan3D>(state);
}

bool ExecutionPlan3D::publishable() const noexcept {
  if (!valid()) {
    return false;
  }
  // A stop owns the vehicle on its own: it has no command/braking pair to
  // agree with, and nothing about it can require revalidation before it is
  // published, because it is what the vehicle falls back to.
  if (std::holds_alternative<StopPlan3D>(state)) {
    return true;
  }
  const FiniteExecutionState3D* const command = finiteExecution();
  const FiniteExecutionState3D* const braking = brakingFallback();
  if (command != nullptr || braking != nullptr) {
    return command != nullptr && braking != nullptr &&
           !command->revalidation_required && !braking->revalidation_required;
  }
  const auto* awaiting = std::get_if<AwaitingSuccessorPlan3D>(&state);
  return awaiting == nullptr ||
         !std::holds_alternative<SuspendedRoutePlan3D>(awaiting->owner);
}

std::uint64_t ExecutionPlan3D::routeGenerationHighWater() const noexcept {
  const CertifiedRouteSuffix3D* const owned_route = route();
  return owned_route != nullptr
             ? std::max(route_generation_high_water, owned_route->identity.generation)
             : route_generation_high_water;
}

} // namespace drone_city_nav
