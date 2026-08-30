#include "production_mppi_route_activation.hpp"

#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/mppi/route_risk_adapter_3d.hpp"
#include "drone_city_nav/mppi/trajectory_reference_adapter_3d.hpp"
#include "drone_city_nav/trajectory_compiler_3d.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <utility>

#include "production_mppi_route_helpers.hpp"
#include "production_mppi_route_materialization.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
namespace {

template<typename T>
[[nodiscard]] const T* optionalAddress(const std::optional<T>& value) noexcept {
  if (!value.has_value()) {
    return nullptr;
  }
  return std::addressof(value.value());
}

[[nodiscard]] bool sameRawMapVersion(const RawMapVersion& first,
                                     const RawMapVersion& second) noexcept {
  return first.producer_instance_id == second.producer_instance_id &&
         first.base_snapshot_revision == second.base_snapshot_revision &&
         first.revision == second.revision;
}

[[nodiscard]] bool
rawWorldExecutionOwnerExact(const ProductionMppiRawWorld3D& raw_world) noexcept {
  return raw_world.version.valid() && raw_world.occupancy != nullptr &&
         raw_world.execution_owner != nullptr && raw_world.execution_owner->valid() &&
         sameRawMapVersion(raw_world.version, raw_world.execution_owner->version()) &&
         std::addressof(raw_world.execution_owner->occupancy()) ==
             raw_world.occupancy.get();
}

[[nodiscard]] StaticRouteCandidateStatus candidateStatusFromRiskAssignment(
    const RouteRiskTierAssignmentStatus3D status) noexcept {
  switch (status) {
    case RouteRiskTierAssignmentStatus3D::kAccepted:
      return StaticRouteCandidateStatus::kAccepted;
    case RouteRiskTierAssignmentStatus3D::kInvalidInput:
      return StaticRouteCandidateStatus::kInvalidInput;
  }
  return StaticRouteCandidateStatus::kInvalidInput;
}

[[nodiscard]] bool sameActivationWorld(const WorldSnapshot3D& current,
                                       const WorldSnapshot3D& captured) noexcept {
  return productionWorldGenerationCoherent(current) &&
         productionWorldGenerationCoherent(captured) &&
         current.local_world_generation.sameSnapshot(captured.local_world_generation);
}

[[nodiscard]] bool exclusiveExecutionHold(const ExecutionPlan3D& snapshot) noexcept {
  return snapshot.phase() == ExecutionRoutePhase3D::kStopped &&
         snapshot.stationaryHold() != nullptr && snapshot.route() == nullptr &&
         snapshot.finiteExecution() == nullptr &&
         snapshot.directTrackingExecution() == nullptr;
}

[[nodiscard]] bool
sameExecutionRouteBase(const std::shared_ptr<const ExecutionPlan3D>& first,
                       const std::shared_ptr<const ExecutionPlan3D>& second) noexcept {
  if (first == nullptr || second == nullptr || !first->valid() || !second->valid()) {
    return false;
  }
  if (first->execution_owner_epoch != second->execution_owner_epoch ||
      (first->route() != nullptr) != (second->route() != nullptr) ||
      (first->directTrackingExecution() != nullptr) !=
          (second->directTrackingExecution() != nullptr) ||
      (first->stationaryHold() != nullptr) != (second->stationaryHold() != nullptr) ||
      first->routeGenerationHighWater() != second->routeGenerationHighWater()) {
    return false;
  }
  const StationaryExecutionHold3D* const first_hold = first->stationaryHold();
  const StationaryExecutionHold3D* const second_hold = second->stationaryHold();
  if (exclusiveExecutionHold(*first)) {
    return exclusiveExecutionHold(*second) && first_hold != nullptr &&
           second_hold != nullptr && first_hold->hold_id == second_hold->hold_id;
  }
  const DirectTrackingFiniteExecution3D* const first_direct =
      first->directTrackingExecution();
  const DirectTrackingFiniteExecution3D* const second_direct =
      second->directTrackingExecution();
  if (first_direct != nullptr) {
    if (second_direct == nullptr) {
      return false;
    }
    const DirectTrackingOwnerIdentity3D& left = first_direct->identity;
    const DirectTrackingOwnerIdentity3D& right = second_direct->identity;
    return left.mission_epoch == right.mission_epoch &&
           left.assignment_generation == right.assignment_generation &&
           left.target_detection_id == right.target_detection_id &&
           left.target_track_id == right.target_track_id &&
           left.objective_sample_sequence == right.objective_sample_sequence &&
           left.line_of_sight_generation == right.line_of_sight_generation;
  }
  const CertifiedRouteSuffix3D* const first_route = first->route();
  const CertifiedRouteSuffix3D* const second_route = second->route();
  if (first_route == nullptr) {
    return first->phase() == second->phase();
  }
  return second_route != nullptr &&
         first_route->identity.generation == second_route->identity.generation &&
         first_route->geometry != nullptr && second_route->geometry != nullptr &&
         first_route->geometry->compiled_trajectory_revision ==
             second_route->geometry->compiled_trajectory_revision &&
         first_route->continuity_id == second_route->continuity_id;
}

[[nodiscard]] PendingExecutionBaseKind3D
pendingExecutionBaseKind(const ExecutionPlan3D& snapshot,
                         const bool route_splice_required) noexcept {
  if (snapshot.phase() == ExecutionRoutePhase3D::kRevoked) {
    return PendingExecutionBaseKind3D::kRevoked;
  }
  if (exclusiveExecutionHold(snapshot)) {
    return PendingExecutionBaseKind3D::kStationaryHold;
  }
  if (snapshot.directTrackingExecution() != nullptr) {
    return PendingExecutionBaseKind3D::kDirectTracking;
  }
  if (snapshot.route() != nullptr) {
    return route_splice_required ? PendingExecutionBaseKind3D::kRoute
                                 : PendingExecutionBaseKind3D::kRouteHandoff;
  }
  return PendingExecutionBaseKind3D::kEmpty;
}

[[nodiscard]] std::optional<std::uint64_t>
nextPendingPublicationSequence(std::atomic<std::uint64_t>& sequence) noexcept {
  std::uint64_t current = sequence.load(std::memory_order_relaxed);
  while (current != std::numeric_limits<std::uint64_t>::max()) {
    if (sequence.compare_exchange_weak(current, current + 1U, std::memory_order_relaxed,
                                       std::memory_order_relaxed)) {
      return current + 1U;
    }
  }
  return std::nullopt;
}

[[nodiscard]] VehicleState3D
exactVehicleState3D(const ProductionMppiNavigation& navigation) noexcept {
  return VehicleState3D{
      .identity =
          VehicleStateIdentity3D{
              .revision = navigation.revision,
              .source_timestamp_us = navigation.source_timestamp_us,
              .receive_stamp_ns = navigation.receive_stamp_ns,
          },
      .position = Point3{navigation.state.x, navigation.state.y, navigation.state.z},
      .velocity = Vec3{navigation.state.vx, navigation.state.vy, navigation.state.vz},
      .yaw_rad = navigation.state.yaw,
      .yaw_rate_radps = navigation.state.yaw_rate,
  };
}

} // namespace

bool RouteAdmissionReport3D::compiledTrajectoryValid() const noexcept {
  return trajectory_validation.valid();
}

bool pendingRoutePublicationBaseCurrent3D(
    const PendingRoutePublicationCurrentness3D& currentness) noexcept {
  return currentness.resident_world_current && currentness.objective_current &&
         currentness.execution_base_current && currentness.candidate_world_coherent;
}

bool RouteAdmissionReport3D::readyForArbitration(
    const ProductionMaterializedRouteProposal3D& proposal) const noexcept {
  if (proposal.trajectory == nullptr) {
    return false;
  }
  const CompiledTrajectory3D& trajectory = *proposal.trajectory;
  return proposal.identity.activation_eligible && assessment.accepted() &&
         handoff.accepted && trajectory.route && trajectory.constrained_spans &&
         trajectory.passage_volumes && trajectory.cooperative_passage_assignments &&
         trajectory.selected_passage_traversal_ids &&
         trajectory.materialized_route_fingerprint ==
             proposal.identity.route_fingerprint &&
         trajectory.compiled_trajectory_revision != 0U &&
         trajectory.compiled_trajectory_revision ==
             compiledTrajectoryRevision3D(trajectory) &&
         trajectory_validation.valid() && world_compatible && objective_matches;
}

ProductionRouteActivationSnapshot3D
ProductionMppiNode::captureRouteActivationSnapshot3D() {
  ProductionRouteActivationSnapshot3D snapshot;
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_,
                                world_generation_publication_mutex_, input_mutex_,
                                esdf_state_mutex_};
    snapshot.execution_authority = route_execution_manager_.authority();
    snapshot.raw_world = latest_raw_world_3d_.load(std::memory_order_acquire);
    snapshot.resident_world = resident_world_;
    snapshot.navigation = navigation_;
    snapshot.objective = navigationObjective();
  }
  snapshot.minimum_tracking_route_mission_epoch =
      minimum_tracking_route_mission_epoch_.load(std::memory_order_acquire);
  snapshot.minimum_tracking_route_sample_sequence =
      minimum_tracking_route_sample_sequence_.load(std::memory_order_acquire);
  snapshot.stamp_ns = get_clock()->now().nanoseconds();
  return snapshot;
}

ProductionRouteActivationResult3D ProductionMppiNode::prepareRouteActivation3D(
    const PlannerSearchTransaction3D& transaction,
    ProductionRouteMaterialization3D materialization,
    const NavigationWorldCertificate3D planned_world_certificate,
    StaticRouteCandidateValidation validation,
    const StaticRouteReplacementPolicy replacement_policy, const Point3& mission_goal,
    const std::uint64_t candidate_generation,
    const ProductionRouteActivationSnapshot3D& snapshot) {
  const std::shared_ptr<const ExecutionPlan3D> captured_execution =
      snapshot.execution_authority != nullptr ? snapshot.execution_authority->plan()
                                              : nullptr;
  const AppliedControlEvidence3D captured_control =
      snapshot.execution_authority != nullptr ? snapshot.execution_authority->control()
                                              : AppliedControlEvidence3D{};
  const ExecutionOwnerIdentity3D captured_owner =
      snapshot.execution_authority != nullptr ? snapshot.execution_authority->owner()
                                              : ExecutionOwnerIdentity3D{};
  ProductionRouteActivationResult3D result;
  result.materialized = std::move(materialization.route);
  result.telemetry = materialization.telemetry;
  RouteAdmissionReport3D& report = result.admission;
  report.candidate_validation = validation;
  report.activation_status =
      result.materialized.planner_executable
          ? StaticRouteActivationStatus::kCandidateValidationRejected
          : StaticRouteActivationStatus::kCandidateNotExecutable;
  MaterializedRoute3D& candidate = result.materialized;
  candidate.candidate_generation = candidate_generation;
  const bool raw_validation_required = candidate.world->observed_occupancy != nullptr;

  report.snapshot_pose_revision = snapshot.navigation.revision;
  report.snapshot_raw_revision =
      snapshot.raw_world ? snapshot.raw_world->version.revision : 0U;

  const MaterializedRouteProposal3D publication_proposal{
      .planned_world = planned_world_certificate,
      .validated_world = navigationWorldCertificate3D(*candidate.world),
  };
  const RoutePublicationAssessment3D publication =
      snapshot.resident_world ? assessRoutePublication3D(publication_proposal,
                                                         navigationWorldCertificate3D(
                                                             *snapshot.resident_world))
                              : RoutePublicationAssessment3D{};
  report.world_compatible = publication.compatible();
  const std::shared_ptr<const VersionedObservedRawWorld3D> activation_raw_owner =
      snapshot.raw_world != nullptr && rawWorldExecutionOwnerExact(*snapshot.raw_world)
          ? snapshot.raw_world->execution_owner
          : nullptr;

  if (report.candidate_validation.accepted && report.world_compatible &&
      snapshot.resident_world && candidate.route && candidate.constrained_spans &&
      candidate.passage_volumes && candidate.cooperative_passage_assignments &&
      candidate.selected_passage_traversal_ids &&
      snapshot.resident_world->distances_m &&
      snapshot.resident_world->local_world_generation.generation !=
          candidate.world->local_world_generation.generation) {
    auto rebased_route = std::make_shared<std::vector<RouteSample3D>>(*candidate.route);
    const RouteRiskTierAssignmentResult3D risk_assignment =
        assignRouteRiskTiersFromMppiEsdf3D(*rebased_route,
                                           snapshot.resident_world->grid,
                                           *snapshot.resident_world->distances_m,
                                           mppi_config_.risk.critical_distance_m,
                                           mppi_config_.risk.preferred_distance_m);
    if (!risk_assignment.accepted()) {
      report.candidate_validation = StaticRouteCandidateValidation{
          .status = candidateStatusFromRiskAssignment(risk_assignment.status),
          .failure_segment_index = risk_assignment.failure_sample_index,
          .failure_point = risk_assignment.failure_point,
      };
    } else {
      const CertifiedRouteSuffix3D* const resident_route =
          captured_execution ? captured_execution->route() : nullptr;
      report.candidate_validation = validateStaticRouteCandidate(
          resident_route != nullptr && resident_route->geometry != nullptr &&
                  resident_route->geometry->route != nullptr
              ? std::span<const RouteSample3D>{*resident_route->geometry->route}
              : std::span<const RouteSample3D>{},
          *rebased_route, mission_goal,
          static_route_extension_config_.minimum_endpoint_improvement_m,
          candidate.reaches_mission_goal, flight_envelope_config_, replacement_policy);
    }
    if (report.candidate_validation.accepted &&
        !validateConstrainedRouteSpans(*rebased_route, *candidate.constrained_spans)) {
      report.candidate_validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
    }
    if (report.candidate_validation.accepted) {
      candidate.world = snapshot.resident_world;
      candidate.route = rebased_route;
      candidate.initial_projection = projectOntoRouteProgress3D(
          *candidate.route,
          Point3{snapshot.navigation.state.x, snapshot.navigation.state.y,
                 snapshot.navigation.state.z});
      report.observed_world_rebased =
          snapshot.resident_world->observed_occupancy != nullptr;
      report.publication_world_advanced = true;
    }
  }

  report.tracking_profile_source_occupied_fingerprint =
      candidate.world->observed_raw_world_owner != nullptr
          ? candidate.world->observed_raw_world_owner->occupiedContentFingerprint()
          : 0U;
  report.tracking_profile_activation_occupied_fingerprint =
      activation_raw_owner != nullptr
          ? activation_raw_owner->occupiedContentFingerprint()
          : 0U;
  const bool spatial_route_available =
      candidate.route != nullptr && candidate.constrained_spans != nullptr &&
      candidate.passage_volumes != nullptr &&
      candidate.cooperative_passage_assignments != nullptr &&
      candidate.selected_passage_traversal_ids != nullptr;
  const bool activation_tracking_world_available =
      !raw_validation_required || activation_raw_owner != nullptr;
  // Spatial search and materialization are snapshot-bound. Compile their
  // immutable executable sidecars exactly once, against the raw snapshot that
  // is also used for activation validation below.
  if (report.candidate_validation.accepted && spatial_route_available &&
      activation_tracking_world_available) {
    report.trajectory_compile_attempted = true;
    const RouteEndpointSemantics3D endpoint_semantics = routeEndpointSemantics3D(
        candidate.reaches_mission_goal, !transaction.objective.continuous_tracking);
    const TrackingErrorTubeWorld3D tracking_world =
        raw_validation_required
            ? TrackingErrorTubeWorld3D{
                  .observed_occupancy = &activation_raw_owner->occupancy(),
                  .occupied_content_fingerprint =
                      activation_raw_owner->occupiedContentFingerprint(),
                  .launch_support_contact =
                      optionalAddress(candidate.world->launch_support_contact),
              }
            : trackingErrorTubeWorld3D(*candidate.world);
    TrajectoryCompilationResult3D compilation =
        TrajectoryCompiler3D::compile(TrajectoryCompilerInput3D{
            .exact_initial_state = exactVehicleState3D(snapshot.navigation),
            .route_generation = candidate.candidate_generation,
            .route = *candidate.route,
            .constrained_spans = *candidate.constrained_spans,
            .passage_volumes = *candidate.passage_volumes,
            .cooperative_passage_assignments =
                *candidate.cooperative_passage_assignments,
            .selected_passage_traversal_ids = *candidate.selected_passage_traversal_ids,
            .passage_volume_config = cooperative_passage_volume_config_,
            .endpoint_semantics = endpoint_semantics,
            .materialized_route_fingerprint = candidate.fingerprint,
            .tracking_world = tracking_world,
            .config = trajectoryCompilerConfig3D(),
        });
    report.trajectory_compiled = compilation.compiled();
    report.trajectory_validation = compilation.validation;
    result.stop_turn_count = compilation.stop_turn_count;
    result.trajectory =
        compilation.compiled() ? std::move(compilation.trajectory) : nullptr;
  } else if (report.candidate_validation.accepted && raw_validation_required &&
             activation_raw_owner == nullptr) {
    report.candidate_validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kRawWorldUnavailable};
  }
  if (candidate.route) {
    candidate.initial_projection = projectOntoRouteProgress3D(
        *candidate.route,
        Point3{snapshot.navigation.state.x, snapshot.navigation.state.y,
               snapshot.navigation.state.z});
  }

  if (report.candidate_validation.accepted && candidate.route &&
      candidate.route->size() >= 2U) {
    const Point3 snapshot_position{snapshot.navigation.state.x,
                                   snapshot.navigation.state.y,
                                   snapshot.navigation.state.z};
    const RouteProjection3D reserve_projection = projectOntoRoute3DWithinStationWindow(
        *candidate.route, snapshot_position, candidate.route->front().station_m,
        candidate.route->back().station_m);
    StaticRoutePlanningLatencyStats latency;
    {
      const std::scoped_lock lock{static_route_extension_mutex_};
      latency = static_route_planning_latency_tracker_.stats();
    }
    if (latency.sample_count == 0U) {
      latency.planning_p95_ms = std::max(0.0, result.telemetry.route_search_ms);
      latency.planning_p99_ms =
          std::max(latency.planning_p95_ms,
                   1000.0 * static_route_extension_config_.maximum_latency_s);
      latency.build_and_planning_p99_ms = latency.planning_p99_ms;
    }
    const ProductionMppiForwardAcceleration3D forward_acceleration =
        productionMppiForwardAcceleration3D(snapshot.navigation);
    const StaticRouteExtensionDecision reserve_decision = evaluateStaticRouteExtension(
        static_route_extension_config_,
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
    const CertifiedRouteReserveAssessment3D reserve = assessCertifiedRouteReserve3D(
        reserve_decision,
        reserve_projection.valid ? reserve_projection.remaining_m
                                 : std::numeric_limits<double>::quiet_NaN(),
        endpoint_semantics);
    report.certified_reserve = reserve;
    if (!reserve.accepted()) {
      report.candidate_validation = StaticRouteCandidateValidation{
          .status = reserve.status == CertifiedRouteReserveStatus3D::kInvalid
                        ? StaticRouteCandidateStatus::kInvalidCertifiedReserve
                        : StaticRouteCandidateStatus::kInsufficientCertifiedReserve};
    }
  }

  NavigationWorldCertificate3D validated_world_certificate =
      navigationWorldCertificate3D(*candidate.world);
  SegmentEvidence3D activation_evidence = candidate.segment_evidence;
  if (candidate.route && candidate.route->size() >= 2U) {
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
      .planned_world = planned_world_certificate,
      .validated_world = validated_world_certificate,
      .objective = transaction.objective,
      .intent = candidate.intent,
      .evidence = activation_evidence,
      .route_fingerprint = candidate.fingerprint,
      .route_sample_count = candidate.route ? candidate.route->size() : 0U,
      .reaches_mission_goal = candidate.reaches_mission_goal,
      .activation_eligible = activation_evidence.physical_executable,
  };

  const std::uint64_t required_objective_sample =
      snapshot.objective && snapshot.objective->mission_epoch ==
                                snapshot.minimum_tracking_route_mission_epoch
          ? snapshot.minimum_tracking_route_sample_sequence
          : 0U;
  report.required_objective_sample = required_objective_sample;
  report.assessment = assessRouteActivation3D(
      activation_identity,
      candidate.route ? std::span<const RouteSample3D>{*candidate.route}
                      : std::span<const RouteSample3D>{},
      RouteActivationObservation3D{
          .resident_world = snapshot.resident_world
                                ? navigationWorldCertificate3D(*snapshot.resident_world)
                                : NavigationWorldCertificate3D{},
          .current_objective = snapshot.objective
                                   ? makeStaticRouteObjective(*snapshot.objective)
                                   : StaticRouteObjective{},
          .minimum_tracking_sample_sequence = required_objective_sample,
          .position = {snapshot.navigation.state.x, snapshot.navigation.state.y,
                       snapshot.navigation.state.z},
          .maximum_cross_track_m = route_tracking_policy_.maximum_cross_track_m,
          .latest_raw_occupancy = snapshot.raw_world && snapshot.raw_world->occupancy
                                      ? snapshot.raw_world->occupancy.get()
                                      : nullptr,
          .latest_raw_producer_instance_id =
              snapshot.raw_world ? snapshot.raw_world->version.producer_instance_id
                                 : 0U,
          .latest_raw_revision =
              snapshot.raw_world ? snapshot.raw_world->version.revision : 0U,
          .footprint =
              SweptFootprintConfig{
                  .radius_m = physical_footprint_config_.radius_m,
                  .lower_extent_m = physical_footprint_config_.lower_extent_m,
                  .upper_extent_m = physical_footprint_config_.upper_extent_m,
                  .perimeter_samples = physical_footprint_config_.perimeter_samples,
                  .radial_rings = physical_footprint_config_.radial_rings,
                  .axial_samples = physical_footprint_config_.axial_samples,
                  .sweep_step_m = physical_footprint_config_.sweep_step_m},
          .launch_support_contact =
              optionalAddress(candidate.world->launch_support_contact),
          .flight_envelope = flight_envelope_config_,
          .raw_validation_required = raw_validation_required,
      });
  report.world_compatible = report.assessment.publication.compatible();
  report.objective_matches = report.assessment.objective_matches;
  if (raw_validation_required && !report.assessment.raw_world_compatible &&
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

  const bool raw_route_valid = !raw_validation_required ||
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
  } else if (raw_validation_required && !report.assessment.raw_world_compatible) {
    activation_evidence.status = SegmentEvidenceStatus3D::kInvalidWorld;
  }
  candidate.segment_evidence = activation_evidence;
  validated_world_certificate.raw_validated_through_revision =
      report.assessment.raw_validated_through_revision;

  result.proposal = ProductionMaterializedRouteProposal3D{
      .identity =
          MaterializedRouteProposal3D{
              .planned_world = planned_world_certificate,
              .validated_world = validated_world_certificate,
              .objective = transaction.objective,
              .intent = candidate.intent,
              .evidence = activation_evidence,
              .route_fingerprint = candidate.fingerprint,
              .route_sample_count = candidate.route ? candidate.route->size() : 0U,
              .reaches_mission_goal = candidate.reaches_mission_goal,
              .activation_eligible = activation_evidence.physical_executable,
          },
      .trajectory = result.trajectory,
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
         result.proposal.trajectory->exact_initial_state !=
             exactVehicleState3D(snapshot.navigation))) {
      report.trajectory_validation.reason =
          CompiledTrajectoryFailureReason3D::kDerivedResourceMismatch;
    }
  }

  const bool handoff_control_fresh = appliedControlAuthoritativeForExecution(
      captured_control, captured_owner, snapshot.stamp_ns,
      maximum_control_feedback_age_ms_);
  const std::shared_ptr<const std::vector<mppi::RouteSample3D>> mppi_reference =
      result.proposal.trajectory != nullptr
          ? mppi::adaptTrajectoryReference3D(*result.proposal.trajectory)
          : nullptr;
  if (result.proposal.identity.activation_eligible && report.assessment.accepted() &&
      mppi_reference && candidate.world->distances_m && snapshot.navigation.valid) {
    report.handoff = mppi::validateStaticRouteHandoff(
        snapshot.navigation.state,
        handoff_control_fresh ? captured_control.control : mppi::Control{},
        *mppi_reference, static_cast<float>(speed_policy_config_.cruise_speed_mps),
        static_cast<float>(route_tracking_policy_.maximum_cross_track_m),
        static_cast<float>(kFiniteExecutionRouteCrossTrackToleranceM3D), mppi_config_,
        candidate.world->grid, *candidate.world->distances_m);
  }

  result.proposal.identity.activation_eligible =
      result.proposal.identity.activation_eligible && report.assessment.accepted() &&
      report.handoff.accepted && result.proposal.trajectory != nullptr &&
      result.proposal.trajectory->route &&
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
  return result;
}

void ProductionMppiNode::commitRouteActivation3D(
    const PlannerSearchTransaction3D& transaction,
    const ProductionRouteActivationSnapshot3D& snapshot,
    const std::uint64_t candidate_generation,
    ProductionRouteActivationResult3D& result) {
  MaterializedRoute3D& candidate = result.materialized;
  RouteAdmissionReport3D& report = result.admission;
  const ProductionMaterializedRouteProposal3D& materialized_proposal = result.proposal;
  const bool raw_validation_required = candidate.world->observed_occupancy != nullptr;
  report.commit_assessment_performed = true;
  report.generation_assessed = true;

  const std::shared_ptr<const ExecutionPlan3D> current_execution =
      route_execution_manager_.plan();
  const CertifiedRouteSuffix3D* const current_route =
      current_execution != nullptr ? current_execution->route() : nullptr;
  const DirectTrackingFiniteExecution3D* const current_direct =
      current_execution != nullptr ? current_execution->directTrackingExecution()
                                   : nullptr;
  const std::uint64_t base_generation =
      current_execution != nullptr ? current_execution->routeGenerationHighWater() : 0U;
  const bool request_requires_base =
      transaction.extension() || transaction.replacement();
  const bool base_generation_matches =
      !request_requires_base ||
      transaction.request.base_route_generation == base_generation;
  const bool allocation_generation_matches =
      base_generation != std::numeric_limits<std::uint64_t>::max() &&
      candidate_generation == base_generation + 1U;
  report.generation_matches = base_generation_matches && allocation_generation_matches;
  report.certification_execution_base_current = sameExecutionRouteBase(
      snapshot.execution_authority != nullptr ? snapshot.execution_authority->plan()
                                              : nullptr,
      current_execution);
  const ActivatedRouteIdentity3D* const active_identity =
      current_route != nullptr ? std::addressof(current_route->identity) : nullptr;
  report.replacement = assessRouteProposalReplacement3D(
      active_identity, materialized_proposal.identity,
      RouteProposalReplacementObservation3D{
          .safety_replan_requested =
              transaction.replacement() &&
              transaction.release_reason == RouteReleaseReason3D::kBlocked,
          .continuity_preserving_successor =
              candidate.provenance.required_splice_base_route_instance_id.valid()});

  const ActivatedRouteIdentity3D candidate_identity{
      .generation = candidate_generation,
      .proposal = materialized_proposal.identity,
  };
  const bool compiled_trajectory_valid =
      candidate.candidate_generation == candidate_generation &&
      report.trajectory_validation.valid() &&
      materialized_proposal.trajectory != nullptr &&
      compiledTrajectoryValid3D(*materialized_proposal.trajectory, candidate_identity);
  if (!compiled_trajectory_valid) {
    RCLCPP_WARN(
        get_logger(),
        "COMPILED_TRAJECTORY valid=false reason=%s sample_index=%zu "
        "route_generation=%" PRIu64,
        compiledTrajectoryFailureReason3DName(report.trajectory_validation.reason),
        report.trajectory_validation.sample_index, candidate_generation);
  }
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_owner;
  std::shared_ptr<const VersionedStaticWorld3D> static_owner;
  if (raw_validation_required && snapshot.raw_world != nullptr &&
      snapshot.raw_world->occupancy != nullptr &&
      snapshot.raw_world->execution_owner != nullptr &&
      snapshot.raw_world->execution_owner->valid() &&
      std::addressof(snapshot.raw_world->execution_owner->occupancy()) ==
          snapshot.raw_world->occupancy.get() &&
      snapshot.raw_world->execution_owner->version().producer_instance_id ==
          snapshot.raw_world->version.producer_instance_id &&
      snapshot.raw_world->execution_owner->version().base_snapshot_revision ==
          snapshot.raw_world->version.base_snapshot_revision &&
      snapshot.raw_world->execution_owner->version().revision ==
          snapshot.raw_world->version.revision) {
    observed_owner = snapshot.raw_world->execution_owner->deriveRouteEvidence(
        candidate.world->proprioceptive_free_space_seed,
        candidate.world->launch_support_contact);
  } else if (!raw_validation_required && static_occupancy_3d_ != nullptr) {
    static_owner = VersionedStaticWorld3D::captureOwned(
        materialized_proposal.identity.validated_world, static_occupancy_3d_);
  }
  const RouteContinuityLineage3D continuity_lineage{
      .mission_epoch = materialized_proposal.identity.objective.mission_epoch,
      .assignment_generation =
          materialized_proposal.identity.objective.assignment_generation,
      .target_detection_id =
          materialized_proposal.identity.objective.target_detection_id,
      .target_track_id = materialized_proposal.identity.objective.target_track_id,
  };
  const std::optional<RouteOwnerIdentity3D> retained_route_owner =
      current_route != nullptr &&
              candidate.provenance.required_splice_base_route_instance_id ==
                  current_route->route_instance_id
          ? std::optional<RouteOwnerIdentity3D>{current_route->owner}
          : std::nullopt;
  const std::optional<CertifiedRouteSuffix3D> certified_route =
      report.readyForArbitration(result.proposal) && report.generation_matches &&
              report.certification_execution_base_current &&
              report.replacement.replacementAllowed() && compiled_trajectory_valid
          ? certifyExecutionRoute3D(ExecutionRouteActivation3D{
                .route_generation = candidate_generation,
                .proposal = materialized_proposal.identity,
                .geometry = materialized_proposal.trajectory,
                .observation =
                    RouteActivationObservation3D{
                        .resident_world =
                            snapshot.resident_world
                                ? navigationWorldCertificate3D(*snapshot.resident_world)
                                : NavigationWorldCertificate3D{},
                        .current_objective =
                            snapshot.objective
                                ? makeStaticRouteObjective(*snapshot.objective)
                                : StaticRouteObjective{},
                        .minimum_tracking_sample_sequence =
                            report.required_objective_sample,
                        .position = {snapshot.navigation.state.x,
                                     snapshot.navigation.state.y,
                                     snapshot.navigation.state.z},
                        .maximum_cross_track_m =
                            route_tracking_policy_.maximum_cross_track_m,
                        .footprint =
                            SweptFootprintConfig{
                                .radius_m = physical_footprint_config_.radius_m,
                                .lower_extent_m =
                                    physical_footprint_config_.lower_extent_m,
                                .upper_extent_m =
                                    physical_footprint_config_.upper_extent_m,
                                .perimeter_samples =
                                    physical_footprint_config_.perimeter_samples,
                                .radial_rings = physical_footprint_config_.radial_rings,
                                .axial_samples =
                                    physical_footprint_config_.axial_samples,
                                .sweep_step_m =
                                    physical_footprint_config_.sweep_step_m},
                        .flight_envelope = flight_envelope_config_,
                        .raw_validation_required = raw_validation_required,
                    },
                .passage_volume_config = cooperative_passage_volume_config_,
                .continuity_lineage = continuity_lineage,
                .observed_raw_world = observed_owner,
                .static_world = static_owner,
                .validation_policy = execution_validation_policy_,
                .retained_route_owner = retained_route_owner,
            })
          : std::nullopt;
  report.route_certified = certified_route.has_value();

  const bool overlap_search =
      candidate.provenance.required_splice_base_route_instance_id.valid();
  const bool overlap_base_matches =
      current_route != nullptr && overlap_search &&
      current_route->route_instance_id ==
          candidate.provenance.required_splice_base_route_instance_id;
  if (certified_route.has_value() && overlap_base_matches) {
    report.splice = certifyRouteSplice3D(*current_route, certified_route.value(),
                                         Point3{snapshot.navigation.state.x,
                                                snapshot.navigation.state.y,
                                                snapshot.navigation.state.z},
                                         certified_route_splice_config_);
  }
  const bool splice_ready =
      !overlap_search || (overlap_base_matches && report.splice.certified());

  const std::optional<std::uint64_t> publication_sequence =
      certified_route.has_value() && splice_ready
          ? nextPendingPublicationSequence(pending_certified_route_sequence_)
          : std::nullopt;
  const auto pending =
      certified_route.has_value() && publication_sequence.has_value()
          ? std::make_shared<const PendingCertifiedRoute3D>(PendingCertifiedRoute3D{
                .publication_sequence = *publication_sequence,
                .base_execution_owner_epoch =
                    current_execution != nullptr
                        ? current_execution->execution_owner_epoch
                        : 0U,
                .base_kind =
                    current_execution != nullptr
                        ? pendingExecutionBaseKind(*current_execution, overlap_search)
                        : PendingExecutionBaseKind3D::kEmpty,
                .base_route_generation = base_generation,
                .base_geometry_revision =
                    current_route != nullptr && current_route->geometry != nullptr
                        ? current_route->geometry->compiled_trajectory_revision
                        : 0U,
                .base_continuity_id =
                    current_route != nullptr ? current_route->continuity_id : 0U,
                .base_direct_tracking_identity =
                    current_direct != nullptr
                        ? std::optional<DirectTrackingOwnerIdentity3D>{current_direct
                                                                           ->identity}
                        : std::nullopt,
                .route_splice = overlap_search ? report.splice.splice : std::nullopt,
                .route = certified_route.value(),
            })
          : nullptr;

  bool published_pending{false};
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_,
                                world_generation_publication_mutex_, esdf_state_mutex_};
    const bool resident_world_current =
        resident_world_ && snapshot.resident_world &&
        sameActivationWorld(*resident_world_, *snapshot.resident_world);
    const bool objective_current =
        navigationObjective() == snapshot.objective &&
        minimum_tracking_route_mission_epoch_.load(std::memory_order_acquire) ==
            snapshot.minimum_tracking_route_mission_epoch &&
        minimum_tracking_route_sample_sequence_.load(std::memory_order_acquire) ==
            snapshot.minimum_tracking_route_sample_sequence;
    const bool raw_world_current =
        !raw_validation_required ||
        latest_raw_world_3d_.load(std::memory_order_acquire) == snapshot.raw_world;
    const bool execution_base_current =
        sameExecutionRouteBase(current_execution, route_execution_manager_.plan());
    const bool candidate_world_coherent =
        productionWorldGenerationCoherent(*candidate.world);
    report.resident_world_snapshot_current = resident_world_current;
    report.objective_snapshot_current = objective_current;
    report.raw_snapshot_current = raw_world_current;
    report.execution_base_snapshot_current = execution_base_current;
    report.candidate_world_coherent = candidate_world_coherent;
    report.snapshot_current =
        pendingRoutePublicationBaseCurrent3D(PendingRoutePublicationCurrentness3D{
            .resident_world_current = resident_world_current,
            .objective_current = objective_current,
            .raw_world_current = raw_world_current,
            .execution_base_current = execution_base_current,
            .candidate_world_coherent = candidate_world_coherent,
        });
    if (pending != nullptr && report.snapshot_current) {
      published_pending = route_execution_manager_.publishPending(pending);
    }
    if (published_pending) {
      report.activation_status = StaticRouteActivationStatus::kCertifiedPending;
      report.generation_matches = true;
      report.world_compatible = true;
      report.certified_pending = true;
    }
  }
  if (!published_pending && report.candidate_validation.accepted &&
      !report.replacement.replacementAllowed()) {
    report.activation_status =
        StaticRouteActivationStatus::kEquivalentActiveSegmentRetained;
  } else if (!published_pending && report.candidate_validation.accepted &&
             !report.world_compatible) {
    report.activation_status = StaticRouteActivationStatus::kWorldPublicationRejected;
  } else if (!published_pending && report.candidate_validation.accepted &&
             !report.snapshot_current) {
    report.activation_status =
        StaticRouteActivationStatus::kActivationSnapshotSuperseded;
  } else if (!published_pending && report.candidate_validation.accepted &&
             !report.generation_matches) {
    report.activation_status = StaticRouteActivationStatus::kStaleRouteGeneration;
  } else if (!published_pending && report.candidate_validation.accepted &&
             !report.objective_matches) {
    report.activation_status = StaticRouteActivationStatus::kStaleObjective;
  } else if (!published_pending && report.candidate_validation.accepted &&
             !compiled_trajectory_valid) {
    report.activation_status = StaticRouteActivationStatus::kInvalidExecutionGeometry;
  } else if (!published_pending && report.candidate_validation.accepted &&
             overlap_search && certified_route.has_value() &&
             !report.splice.certified()) {
    report.activation_status = StaticRouteActivationStatus::kCertifiedSpliceRejected;
  } else if (!published_pending && report.candidate_validation.accepted &&
             (!report.assessment.accepted() || !report.handoff.accepted ||
              !certified_route.has_value())) {
    report.activation_status = StaticRouteActivationStatus::kDynamicHandoffRejected;
  }
  latest_route_pipeline_event_.store(
      std::make_shared<const ProductionRouteActivationResult3D>(result),
      std::memory_order_release);
}

} // namespace drone_city_nav
