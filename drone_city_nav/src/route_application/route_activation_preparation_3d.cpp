#include "route_activation_preparation_3d.hpp"

#include "drone_city_nav/execution_horizon_commit_3d.hpp"
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

namespace drone_city_nav {
namespace {

template<typename T>
[[nodiscard]] const T* optionalAddress(const std::optional<T>& value) noexcept {
  if (!value.has_value()) {
    return nullptr;
  }
  return std::addressof(value.value());
}

[[nodiscard]] StaticRouteCandidateStatus
candidateStatusFromRiskAssignment(const RouteRiskAnnotationStatus3D status) noexcept {
  switch (status) {
    case RouteRiskAnnotationStatus3D::kAccepted:
      return StaticRouteCandidateStatus::kAccepted;
    case RouteRiskAnnotationStatus3D::kInvalidInput:
      return StaticRouteCandidateStatus::kInvalidInput;
  }
  return StaticRouteCandidateStatus::kInvalidInput;
}

[[nodiscard]] bool exclusiveExecutionHold(const ExecutionPlan3D& snapshot) noexcept {
  return snapshot.phase() == ExecutionRoutePhase3D::kStopped &&
         snapshot.stationaryHold() != nullptr && snapshot.route() == nullptr &&
         snapshot.finiteExecution() == nullptr &&
         snapshot.directTrackingExecution() == nullptr;
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
  if (snapshot.stopExecution() != nullptr) {
    return PendingExecutionBaseKind3D::kStop;
  }
  if (snapshot.route() != nullptr) {
    return route_splice_required ? PendingExecutionBaseKind3D::kRoute
                                 : PendingExecutionBaseKind3D::kRouteHandoff;
  }
  return PendingExecutionBaseKind3D::kEmpty;
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

[[nodiscard]] SweptFootprintConfig
activationFootprint(const RouteActivationCoordinatorConfig3D& config) noexcept {
  return SweptFootprintConfig{
      .radius_m = config.physical_footprint.radius_m,
      .lower_extent_m = config.physical_footprint.lower_extent_m,
      .upper_extent_m = config.physical_footprint.upper_extent_m,
      .perimeter_samples = config.physical_footprint.perimeter_samples,
      .radial_rings = config.physical_footprint.radial_rings,
      .axial_samples = config.physical_footprint.axial_samples,
      .sweep_step_m = config.physical_footprint.sweep_step_m,
  };
}

struct RouteActivationPreparationState3D final {
  RouteActivationPreparationState3D() = default;
  RouteActivationPreparationState3D(const RouteActivationPreparationState3D&) = delete;
  RouteActivationPreparationState3D&
  operator=(const RouteActivationPreparationState3D&) = delete;
  RouteActivationPreparationState3D(RouteActivationPreparationState3D&&) = default;
  RouteActivationPreparationState3D&
  operator=(RouteActivationPreparationState3D&&) = default;
  ~RouteActivationPreparationState3D() = default;

  PreparedRouteActivation3D prepared{};
  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
  StaticRouteReplacementPolicy replacement_policy{
      StaticRouteReplacementPolicy::kRequireEndpointImprovement};
  StaticRoutePlanningLatencyStats planning_latency{};
  NavigationWorldCertificate3D planned_world_certificate{};
  std::shared_ptr<const VersionedObservedRawWorld3D> activation_raw_owner;
  std::optional<CertifiedRouteSuffix3D> certified_route;
  std::uint64_t base_generation{0U};
  bool raw_validation_required{false};
  bool overlap_search{false};
  bool overlap_base_matches{false};
  bool splice_ready{false};
  bool successor_improvement_cleared{false};
};

[[nodiscard]] RouteActivationPreparationState3D
beginPreparation(RouteActivationPreparationRequest3D request) {
  const bool request_valid = request.valid();
  RouteActivationPreparationState3D state;
  state.replacement_policy = request.materialization.replacement_policy;
  state.planning_latency = request.planning_latency;
  state.prepared.snapshot = std::move(request.snapshot);
  state.prepared.execution_base =
      state.prepared.snapshot.execution_authority != nullptr
          ? state.prepared.snapshot.execution_authority->plan()
          : nullptr;
  state.prepared.result.materialized = std::move(request.materialization.route);
  state.prepared.result.telemetry = request.materialization.telemetry;
  RouteAdmissionReport3D& report = state.prepared.result.admission;
  report.candidate_validation = request.materialization.validation;
  report.activation_status =
      state.prepared.result.materialized.planner_executable
          ? StaticRouteActivationStatus::kCandidateValidationRejected
          : StaticRouteActivationStatus::kCandidateNotExecutable;
  if (!request_valid) {
    report.candidate_validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kInvalidInput,
    };
    return state;
  }

  state.transaction = std::move(request.transaction);
  state.planned_world_certificate =
      navigationWorldCertificate3D(*state.transaction->world);
  state.raw_validation_required =
      state.prepared.result.materialized.world->observed_occupancy != nullptr;
  report.snapshot_pose_revision = state.prepared.snapshot.navigation.revision;
  report.snapshot_raw_revision =
      state.prepared.snapshot.raw_world != nullptr
          ? state.prepared.snapshot.raw_world->version().revision
          : 0U;
  state.activation_raw_owner =
      state.prepared.snapshot.raw_world != nullptr &&
              state.prepared.snapshot.raw_world->valid()
          ? state.prepared.snapshot.raw_world->authoritativeOwner()
          : nullptr;
  return state;
}

[[nodiscard]] RouteActivationPreparationState3D
rebaseAndValidate(RouteActivationPreparationState3D state,
                  const RouteActivationCoordinatorConfig3D& config) {
  const PlannerSearchTransaction3D& transaction = *state.transaction;
  const ProductionRouteActivationSnapshot3D& snapshot = state.prepared.snapshot;
  const std::shared_ptr<const ExecutionPlan3D>& captured_execution =
      state.prepared.execution_base;
  MaterializedRoute3D& candidate = state.prepared.result.materialized;
  RouteAdmissionReport3D& report = state.prepared.result.admission;

  const MaterializedRouteProposal3D publication_proposal{
      .planned_world = state.planned_world_certificate,
      .validated_world = navigationWorldCertificate3D(*candidate.world),
  };
  const RoutePublicationAssessment3D publication =
      snapshot.resident_world != nullptr
          ? assessRoutePublication3D(
                publication_proposal,
                navigationWorldCertificate3D(*snapshot.resident_world))
          : RoutePublicationAssessment3D{};
  report.world_compatible = publication.compatible();

  if (report.candidate_validation.accepted && report.world_compatible &&
      snapshot.resident_world != nullptr && candidate.route != nullptr &&
      candidate.constrained_spans != nullptr && candidate.passage_volumes != nullptr &&
      candidate.cooperative_passage_assignments != nullptr &&
      candidate.selected_passage_traversal_ids != nullptr &&
      snapshot.resident_world->distances_m != nullptr &&
      snapshot.resident_world->local_world_generation.generation !=
          candidate.world->local_world_generation.generation) {
    auto rebased_route = std::make_shared<std::vector<RouteSample3D>>(*candidate.route);
    const RouteRiskAnnotationResult3D risk_assignment =
        annotateRouteRiskTiersFromDerivedEsdf3D(*rebased_route,
                                                snapshot.resident_world->grid,
                                                *snapshot.resident_world->distances_m,
                                                config.route_risk.critical_distance_m,
                                                config.route_risk.preferred_distance_m);
    if (!risk_assignment.accepted()) {
      report.candidate_validation = StaticRouteCandidateValidation{
          .status = candidateStatusFromRiskAssignment(risk_assignment.status),
          .failure_segment_index = risk_assignment.failure_sample_index,
          .failure_point = risk_assignment.failure_point,
      };
    } else {
      const CertifiedRouteSuffix3D* const resident_route =
          captured_execution != nullptr ? captured_execution->route() : nullptr;
      report.candidate_validation = validateStaticRouteCandidate(
          resident_route != nullptr && resident_route->geometry != nullptr &&
                  resident_route->geometry->route != nullptr
              ? std::span<const RouteSample3D>{*resident_route->geometry->route}
              : std::span<const RouteSample3D>{},
          *rebased_route, transaction.objective.goal,
          config.route_extension.minimum_endpoint_improvement_m,
          candidate.reaches_mission_goal, config.flight_envelope,
          state.replacement_policy);
    }
    if (report.candidate_validation.accepted &&
        !validateConstrainedRouteSpans(*rebased_route, *candidate.constrained_spans)) {
      report.candidate_validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
    }
    if (report.candidate_validation.accepted) {
      candidate.world = snapshot.resident_world;
      candidate.route = std::move(rebased_route);
      candidate.initial_projection = projectOntoRouteProgress3D(
          *candidate.route,
          Point3{snapshot.navigation.state.x, snapshot.navigation.state.y,
                 snapshot.navigation.state.z});
      report.observed_world_rebased =
          snapshot.resident_world->observed_occupancy != nullptr;
      report.publication_world_advanced = true;
    }
  }
  return state;
}

[[nodiscard]] RouteActivationPreparationState3D
compile(RouteActivationPreparationState3D state,
        const RouteTrajectoryCompiler3D& trajectory_compiler) {
  const PlannerSearchTransaction3D& transaction = *state.transaction;
  const ProductionRouteActivationSnapshot3D& snapshot = state.prepared.snapshot;
  ProductionRouteActivationResult3D& result = state.prepared.result;
  MaterializedRoute3D& candidate = result.materialized;
  RouteAdmissionReport3D& report = result.admission;

  report.tracking_profile_source_occupied_fingerprint =
      candidate.world->observed_raw_world_owner != nullptr
          ? candidate.world->observed_raw_world_owner->occupiedContentFingerprint()
          : 0U;
  report.tracking_profile_activation_occupied_fingerprint =
      state.activation_raw_owner != nullptr
          ? state.activation_raw_owner->occupiedContentFingerprint()
          : 0U;
  const bool spatial_route_available =
      candidate.route != nullptr && candidate.constrained_spans != nullptr &&
      candidate.passage_volumes != nullptr &&
      candidate.cooperative_passage_assignments != nullptr &&
      candidate.selected_passage_traversal_ids != nullptr;
  const bool activation_tracking_world_available =
      !state.raw_validation_required || state.activation_raw_owner != nullptr;
  if (report.candidate_validation.accepted && spatial_route_available &&
      activation_tracking_world_available) {
    report.trajectory_compile_attempted = true;
    const RouteEndpointSemantics3D endpoint_semantics = routeEndpointSemantics3D(
        candidate.reaches_mission_goal, !transaction.objective.continuous_tracking);
    RouteTrajectoryCompilationResult3D compilation =
        trajectory_compiler.compile(RouteTrajectoryCompilationRequest3D{
            .materialized = candidate,
            .exact_initial_state = exactVehicleState3D(snapshot.navigation),
            .endpoint_semantics = endpoint_semantics,
            .observed_raw_world = state.activation_raw_owner,
        });
    const bool compilation_succeeded = compilation.compiled();
    report.trajectory_compiled = compilation_succeeded;
    report.trajectory_validation = compilation.validation;
    report.decoration_validation = compilation.decoration_validation;
    result.stop_turn_count = compilation.stop_turn_count;
    result.trajectory =
        compilation_succeeded ? std::move(compilation.trajectory) : nullptr;
    result.decorations =
        compilation_succeeded ? std::move(compilation.decorations) : nullptr;
  } else if (report.candidate_validation.accepted && state.raw_validation_required &&
             state.activation_raw_owner == nullptr) {
    report.candidate_validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kRawWorldUnavailable};
  }
  if (candidate.route != nullptr) {
    candidate.initial_projection = projectOntoRouteProgress3D(
        *candidate.route,
        Point3{snapshot.navigation.state.x, snapshot.navigation.state.y,
               snapshot.navigation.state.z});
  }
  return state;
}

[[nodiscard]] RouteActivationPreparationState3D
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
          .proprioceptive_free_space_seed =
              optionalAddress(candidate.world->proprioceptive_free_space_seed),
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

[[nodiscard]] RouteActivationPreparationState3D
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
  // cannot follow any more: it is no base a successor has to improve on, so
  // the first raw-valid successor replaces it.
  const bool safety_replan_requested =
      transaction.replacement() &&
      (transaction.release_reason == RouteReleaseReason3D::kBlocked ||
       transaction.release_reason == RouteReleaseReason3D::kDiverged);
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
  state.successor_improvement_cleared =
      !report.successor_improvement_required || report.successor_improvement.accepted();
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

[[nodiscard]] RouteActivationPreparationState3D
certify(RouteActivationPreparationState3D state,
        const RouteActivationCoordinatorConfig3D& config) {
  const ProductionRouteActivationSnapshot3D& snapshot = state.prepared.snapshot;
  ProductionRouteActivationResult3D& result = state.prepared.result;
  MaterializedRoute3D& candidate = result.materialized;
  RouteAdmissionReport3D& report = result.admission;
  const ProductionMaterializedRouteProposal3D& materialized_proposal = result.proposal;
  const std::uint64_t candidate_generation = candidate.candidate_generation;
  const std::shared_ptr<const ExecutionPlan3D>& current_execution =
      state.prepared.execution_base;
  const CertifiedRouteSuffix3D* const current_route =
      current_execution != nullptr ? current_execution->route() : nullptr;

  const ActivatedRouteIdentity3D candidate_identity{
      .generation = candidate_generation,
      .proposal = materialized_proposal.identity,
  };
  const bool compiled_trajectory_valid =
      candidate.candidate_generation == candidate_generation &&
      report.trajectory_validation.valid() && report.decoration_validation.valid() &&
      materialized_proposal.trajectory != nullptr &&
      materialized_proposal.decorations != nullptr &&
      compiledTrajectoryValid3D(*materialized_proposal.trajectory,
                                candidate_identity) &&
      routeDecorationsValid3D(*materialized_proposal.decorations,
                              *materialized_proposal.trajectory, candidate_generation);
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_owner;
  std::shared_ptr<const VersionedStaticWorld3D> static_owner;
  if (state.raw_validation_required && snapshot.raw_world != nullptr) {
    observed_owner = snapshot.raw_world->deriveRouteEvidence(
        candidate.world->proprioceptive_free_space_seed,
        candidate.world->launch_support_contact);
  } else if (!state.raw_validation_required &&
             candidate.world->static_occupancy != nullptr) {
    static_owner = VersionedStaticWorld3D::captureOwned(
        materialized_proposal.identity.validated_world,
        candidate.world->static_occupancy);
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
  state.certified_route =
      report.readyForArbitration(result.proposal) && report.generation_matches &&
              report.certification_execution_base_current &&
              report.replacement.replacementAllowed() &&
              state.successor_improvement_cleared && compiled_trajectory_valid
          ? certifyExecutionRoute3D(ExecutionRouteActivation3D{
                .route_generation = candidate_generation,
                .proposal = materialized_proposal.identity,
                .geometry = materialized_proposal.trajectory,
                .decorations = materialized_proposal.decorations,
                .observation =
                    RouteActivationObservation3D{
                        .resident_world =
                            snapshot.resident_world != nullptr
                                ? navigationWorldCertificate3D(*snapshot.resident_world)
                                : NavigationWorldCertificate3D{},
                        .current_objective =
                            snapshot.objective != nullptr
                                ? makeStaticRouteObjective(*snapshot.objective)
                                : StaticRouteObjective{},
                        .minimum_tracking_sample_sequence =
                            report.required_objective_sample,
                        .position = {snapshot.navigation.state.x,
                                     snapshot.navigation.state.y,
                                     snapshot.navigation.state.z},
                        .maximum_cross_track_m =
                            config.route_tracking.maximum_cross_track_m,
                        .footprint = activationFootprint(config),
                        .flight_envelope = config.flight_envelope,
                        .raw_validation_required = state.raw_validation_required,
                    },
                .continuity_lineage = continuity_lineage,
                .observed_raw_world = observed_owner,
                .static_world = static_owner,
                .validation_policy = config.validation_policy,
                .retained_route_owner = retained_route_owner,
            })
          : std::nullopt;
  report.route_certified = state.certified_route.has_value();

  state.overlap_base_matches =
      current_route != nullptr && state.overlap_search &&
      current_route->route_instance_id ==
          candidate.provenance.required_splice_base_route_instance_id;
  if (state.certified_route.has_value() && state.overlap_base_matches) {
    report.splice = certifyRouteSplice3D(*current_route, state.certified_route.value(),
                                         Point3{snapshot.navigation.state.x,
                                                snapshot.navigation.state.y,
                                                snapshot.navigation.state.z},
                                         config.certified_splice);
  }
  state.splice_ready = !state.overlap_search ||
                       (state.overlap_base_matches && report.splice.certified());
  return state;
}

[[nodiscard]] PreparedRouteActivation3D
makePendingDraft(RouteActivationPreparationState3D state) {
  ProductionRouteActivationResult3D& result = state.prepared.result;
  const RouteAdmissionReport3D& report = result.admission;
  const std::shared_ptr<const ExecutionPlan3D>& current_execution =
      state.prepared.execution_base;
  const CertifiedRouteSuffix3D* const current_route =
      current_execution != nullptr ? current_execution->route() : nullptr;
  const DirectTrackingFiniteExecution3D* const current_direct =
      current_execution != nullptr ? current_execution->directTrackingExecution()
                                   : nullptr;

  state.prepared.pending_draft =
      state.certified_route.has_value() && state.splice_ready
          ? std::optional<PendingCertifiedRoute3D>{PendingCertifiedRoute3D{
                .publication_sequence = 0U,
                .base_execution_owner_epoch =
                    current_execution != nullptr
                        ? current_execution->execution_owner_epoch
                        : 0U,
                .base_kind = current_execution != nullptr
                                 ? pendingExecutionBaseKind(*current_execution,
                                                            state.overlap_search)
                                 : PendingExecutionBaseKind3D::kEmpty,
                .base_route_generation = state.base_generation,
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
                .route_splice =
                    state.overlap_search ? report.splice.splice : std::nullopt,
                .route = state.certified_route.value(),
            }}
          : std::nullopt;
  return std::move(state.prepared);
}

} // namespace

PreparedRouteActivation3D
prepareRouteActivation3D(RouteActivationPreparationRequest3D request,
                         const RouteActivationCoordinatorConfig3D& config,
                         const RouteTrajectoryCompiler3D& trajectory_compiler) {
  RouteActivationPreparationState3D state = beginPreparation(std::move(request));
  if (state.transaction == nullptr) {
    return std::move(state.prepared);
  }
  state = rebaseAndValidate(std::move(state), config);
  state = compile(std::move(state), trajectory_compiler);
  state = assessAdmission(std::move(state), config);
  state = assessReplacement(std::move(state), config);
  state = certify(std::move(state), config);
  return makePendingDraft(std::move(state));
}

} // namespace drone_city_nav
