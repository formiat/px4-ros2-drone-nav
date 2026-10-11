#include "route_activation_preparation_3d.hpp"

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
#include "route_activation_preparation_3d_stages.hpp"

namespace drone_city_nav {
namespace {

using route_activation_preparation_detail::activationFootprint;
using route_activation_preparation_detail::assessAdmission;
using route_activation_preparation_detail::assessReplacement;
using route_activation_preparation_detail::exactVehicleState3D;
using route_activation_preparation_detail::optionalAddress;
using route_activation_preparation_detail::RouteActivationPreparationState3D;

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
         snapshot.finiteExecution() == nullptr;
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
  if (snapshot.stopExecution() != nullptr) {
    return PendingExecutionBaseKind3D::kStop;
  }
  if (snapshot.route() != nullptr) {
    return route_splice_required ? PendingExecutionBaseKind3D::kRoute
                                 : PendingExecutionBaseKind3D::kRouteHandoff;
  }
  return PendingExecutionBaseKind3D::kEmpty;
}

[[nodiscard]] RouteActivationPreparationState3D
beginPreparation(RouteActivationPreparationRequest3D request) {
  const bool request_valid = request.valid();
  RouteActivationPreparationState3D state;
  state.replacement_policy = request.materialization.replacement_policy;
  state.blocked_replacement_held_s = request.blocked_replacement_held_s;
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
          ? certifyExecutionRoute3D(
                ExecutionRouteActivation3D{
                    .route_generation = candidate_generation,
                    .proposal = materialized_proposal.identity,
                    .geometry = materialized_proposal.trajectory,
                    .decorations = materialized_proposal.decorations,
                    .observation =
                        RouteActivationObservation3D{
                            .resident_world = snapshot.resident_world != nullptr
                                                  ? navigationWorldCertificate3D(
                                                        *snapshot.resident_world)
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
                },
                &report.route_certification)
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
