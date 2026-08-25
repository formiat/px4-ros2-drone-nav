#include "production_mppi_route_activation.hpp"

#include "drone_city_nav/execution_route_snapshot_3d.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <utility>

#include "production_mppi_route_helpers.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] StaticRouteCandidateStatus
candidateStatusFromRiskAssignment(const RouteRiskTierAssignmentStatus status) noexcept {
  if (status == RouteRiskTierAssignmentStatus::kRawCollision) {
    return StaticRouteCandidateStatus::kRawCollision;
  }
  if (status == RouteRiskTierAssignmentStatus::kOutsideGrid ||
      status == RouteRiskTierAssignmentStatus::kUnknownSpace) {
    return StaticRouteCandidateStatus::kOutsideEsdf;
  }
  return StaticRouteCandidateStatus::kInvalidEsdf;
}

[[nodiscard]] bool
sameActivationWorld(const ProductionMppiPreparedEsdf& current,
                    const ProductionMppiPreparedEsdf& captured) noexcept {
  return productionWorldGenerationCoherent(current) &&
         productionWorldGenerationCoherent(captured) &&
         current.local_world_generation.sameSnapshot(captured.local_world_generation);
}

[[nodiscard]] bool
exclusiveExecutionHold(const ExecutionRouteSnapshot3D& snapshot) noexcept {
  return snapshot.phase == ExecutionRoutePhase3D::kStopped &&
         snapshot.stationary_hold.has_value() && !snapshot.route.has_value() &&
         !snapshot.finite_execution.has_value() &&
         !snapshot.direct_tracking_execution.has_value();
}

[[nodiscard]] bool sameExecutionRouteBase(
    const std::shared_ptr<const ExecutionRouteSnapshot3D>& first,
    const std::shared_ptr<const ExecutionRouteSnapshot3D>& second) noexcept {
  if (first == nullptr || second == nullptr || !first->valid() || !second->valid()) {
    return false;
  }
  if (first->execution_owner_epoch != second->execution_owner_epoch ||
      first->route.has_value() != second->route.has_value() ||
      first->direct_tracking_execution.has_value() !=
          second->direct_tracking_execution.has_value() ||
      first->stationary_hold.has_value() != second->stationary_hold.has_value() ||
      first->routeGenerationHighWater() != second->routeGenerationHighWater()) {
    return false;
  }
  if (exclusiveExecutionHold(*first)) {
    return exclusiveExecutionHold(*second) &&
           first->stationary_hold->hold_id == second->stationary_hold->hold_id;
  }
  if (first->direct_tracking_execution.has_value()) {
    const DirectTrackingOwnerIdentity3D& left =
        first->direct_tracking_execution->identity;
    const DirectTrackingOwnerIdentity3D& right =
        second->direct_tracking_execution->identity;
    return left.mission_epoch == right.mission_epoch &&
           left.assignment_generation == right.assignment_generation &&
           left.target_detection_id == right.target_detection_id &&
           left.target_track_id == right.target_track_id &&
           left.objective_sample_sequence == right.objective_sample_sequence &&
           left.line_of_sight_generation == right.line_of_sight_generation;
  }
  if (!first->route.has_value()) {
    return first->phase == second->phase;
  }
  return first->route->identity.generation == second->route->identity.generation &&
         first->route->geometry != nullptr && second->route->geometry != nullptr &&
         first->route->geometry->executable_geometry_revision ==
             second->route->geometry->executable_geometry_revision &&
         first->route->continuity_id == second->route->continuity_id;
}

[[nodiscard]] PendingExecutionBaseKind3D
pendingExecutionBaseKind(const ExecutionRouteSnapshot3D& snapshot) noexcept {
  if (snapshot.phase == ExecutionRoutePhase3D::kRevoked) {
    return PendingExecutionBaseKind3D::kRevoked;
  }
  if (exclusiveExecutionHold(snapshot)) {
    return PendingExecutionBaseKind3D::kStationaryHold;
  }
  if (snapshot.direct_tracking_execution.has_value()) {
    return PendingExecutionBaseKind3D::kDirectTracking;
  }
  if (snapshot.route.has_value()) {
    return PendingExecutionBaseKind3D::kRoute;
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

} // namespace

bool ProductionRouteActivationResult3D::executionGeometryValid() const noexcept {
  const ProductionRouteGeometry3D& geometry = proposal.geometry;
  const ActivatedRouteIdentity3D candidate_identity{
      .generation = candidate_generation,
      .proposal = proposal.identity,
  };
  return candidate_generation != 0U &&
         executionRouteGeometryValid3D(geometry, candidate_identity);
}

bool ProductionRouteActivationResult3D::readyForArbitration() const noexcept {
  const ProductionRouteGeometry3D& geometry = proposal.geometry;
  return proposal.identity.activation_eligible && assessment.accepted() &&
         handoff.accepted && geometry.mppi_route && geometry.route &&
         geometry.route_2d_projection && geometry.constrained_spans &&
         geometry.passage_volumes && geometry.cooperative_passage_assignments &&
         geometry.selected_passage_traversal_ids &&
         geometry.materialized_route_fingerprint ==
             proposal.identity.route_fingerprint &&
         geometry.executable_geometry_revision != 0U &&
         geometry.executable_geometry_revision ==
             executionRouteGeometryRevision3D(geometry) &&
         executionGeometryValid() && world_compatible && objective_matches;
}

ProductionRouteActivationSnapshot3D
ProductionMppiNode::captureRouteActivationSnapshot3D() {
  ProductionRouteActivationSnapshot3D snapshot;
  {
    const std::scoped_lock evidence_lock{execution_evidence_commit_mutex_};
    snapshot.execution_snapshot = execution_route_store_.snapshot();
    snapshot.raw_world = latest_raw_world_3d_.load(std::memory_order_acquire);
  }
  {
    const std::scoped_lock lock{world_generation_publication_mutex_, input_mutex_,
                                esdf_state_mutex_};
    snapshot.resident_world = prepared_esdf_;
    snapshot.navigation = navigation_;
    snapshot.applied_control = applied_control_;
    snapshot.execution_horizon_owner = execution_horizon_owner_;
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
    const ProductionMppiPreparedEsdf& search_world, ProductionMppiPreparedEsdf prepared,
    const NavigationWorldCertificate3D planned_world_certificate,
    StaticRouteCandidateValidation validation,
    const StaticRouteReplacementPolicy replacement_policy, const Point3& mission_goal,
    const std::uint64_t candidate_generation,
    const ProductionRouteActivationSnapshot3D& snapshot) {
  ProductionRouteActivationResult3D result{
      .prepared = std::move(prepared),
      .validation = validation,
      .candidate_generation = candidate_generation,
  };
  result.activation_status =
      result.prepared.lattice_executable
          ? StaticRouteActivationStatus::kCandidateValidationRejected
          : StaticRouteActivationStatus::kCandidateNotExecutable;
  ProductionMppiPreparedEsdf& candidate = result.prepared;

  result.snapshot_pose_revision = snapshot.navigation.revision;
  result.snapshot_raw_revision =
      snapshot.raw_world ? snapshot.raw_world->version.revision : 0U;

  const MaterializedRouteProposal3D publication_proposal{
      .planned_world = planned_world_certificate,
      .validated_world = navigationWorldCertificate3D(candidate),
  };
  const RoutePublicationAssessment3D publication =
      snapshot.resident_world ? assessRoutePublication3D(publication_proposal,
                                                         navigationWorldCertificate3D(
                                                             *snapshot.resident_world))
                              : RoutePublicationAssessment3D{};
  result.world_compatible = publication.compatible();

  if (result.validation.accepted && result.world_compatible &&
      snapshot.resident_world && candidate.route_3d && candidate.constrained_spans &&
      snapshot.resident_world->distances_m &&
      snapshot.resident_world->local_world_generation.generation !=
          candidate.local_world_generation.generation) {
    auto rebased_route =
        std::make_shared<std::vector<RouteSample3D>>(*candidate.route_3d);
    const RouteRiskTierAssignmentResult risk_assignment = assignRouteRiskTiers(
        *rebased_route, snapshot.resident_world->grid,
        *snapshot.resident_world->distances_m, mppi_config_.risk.critical_distance_m,
        mppi_config_.risk.preferred_distance_m,
        lattice_3d_config_.require_known_free_space);
    if (!risk_assignment.accepted()) {
      result.validation = StaticRouteCandidateValidation{
          .status = candidateStatusFromRiskAssignment(risk_assignment.status),
          .failure_segment_index = risk_assignment.failure_sample_index,
          .failure_point = risk_assignment.failure_point,
      };
    } else {
      result.validation = validateStaticRouteCandidate(
          snapshot.resident_world->route_3d
              ? std::span<const RouteSample3D>{*snapshot.resident_world->route_3d}
              : std::span<const RouteSample3D>{},
          *rebased_route, snapshot.resident_world->grid,
          *snapshot.resident_world->distances_m, mission_goal,
          static_route_extension_config_.minimum_endpoint_improvement_m,
          candidate.global_guide_reaches_mission_goal,
          lattice_3d_config_.flight_envelope, replacement_policy,
          SweptFootprintConfig{
              .radius_m = lattice_3d_config_.physical_footprint_radius_m,
              .lower_extent_m = lattice_3d_config_.physical_footprint_lower_extent_m,
              .upper_extent_m = lattice_3d_config_.physical_footprint_upper_extent_m,
              .perimeter_samples = physical_footprint_config_.perimeter_samples,
              .radial_rings = physical_footprint_config_.radial_rings,
              .axial_samples = physical_footprint_config_.axial_samples,
              .sweep_step_m = physical_footprint_config_.sweep_step_m});
    }
    if (result.validation.accepted &&
        !validateConstrainedRouteSpans(*rebased_route, *candidate.constrained_spans,
                                       snapshot.resident_world->grid,
                                       *snapshot.resident_world->distances_m)) {
      result.validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kInvalidPassageSpan};
    }
    if (result.validation.accepted) {
      adoptWorldResources(candidate, *snapshot.resident_world);
      candidate.route_3d = rebased_route;
      candidate.route_2d_projection = projectRouteTo2D(*rebased_route);
      const RouteEndpointSemantics3D endpoint_semantics = routeEndpointSemantics3D(
          candidate.route_intent,
          candidate.route_segment_evidence.reaches_intent_target,
          candidate.global_guide_reaches_mission_goal,
          !search_world.search_objective.continuous_tracking);
      candidate.mppi_route = makeMppiRoute3D(
          *rebased_route, *candidate.constrained_spans,
          speed_policy_config_.cruise_speed_mps, constrained_route_speed_limit_mps_,
          endpoint_semantics, speed_policy_config_);
      candidate.global_guide_projection = projectOntoGlobalGuide(
          *candidate.route_2d_projection,
          Point2{snapshot.navigation.state.x, snapshot.navigation.state.y});
      result.observed_world_rebased =
          snapshot.resident_world->observed_occupancy != nullptr;
      result.publication_world_advanced = true;
    }
  }
  if (candidate.route_2d_projection) {
    candidate.global_guide_projection = projectOntoGlobalGuide(
        *candidate.route_2d_projection,
        Point2{snapshot.navigation.state.x, snapshot.navigation.state.y});
  }

  NavigationWorldCertificate3D validated_world_certificate =
      navigationWorldCertificate3D(candidate);
  SegmentEvidence3D activation_evidence = candidate.route_segment_evidence;
  if (candidate.route_3d && candidate.route_3d->size() >= 2U) {
    const Point3 snapshot_position{snapshot.navigation.state.x,
                                   snapshot.navigation.state.y,
                                   snapshot.navigation.state.z};
    activation_evidence.route_length_m = 0.0;
    for (std::size_t index = 1U; index < candidate.route_3d->size(); ++index) {
      activation_evidence.route_length_m +=
          distance3D((*candidate.route_3d)[index - 1U].position,
                     (*candidate.route_3d)[index].position);
    }
    activation_evidence.endpoint_displacement_m =
        distance3D(snapshot_position, candidate.route_3d->back().position);
    activation_evidence.mission_progress_m =
        distance3D(snapshot_position, candidate.route_intent.mission_target) -
        distance3D(candidate.route_3d->back().position,
                   candidate.route_intent.mission_target);
  }
  activation_evidence.physical_executable =
      candidate.lattice_executable && result.validation.accepted;
  const MaterializedRouteProposal3D activation_identity{
      .planned_world = planned_world_certificate,
      .validated_world = validated_world_certificate,
      .objective = search_world.search_objective,
      .intent = candidate.route_intent,
      .evidence = activation_evidence,
      .route_fingerprint = candidate.route_fingerprint,
      .route_sample_count = candidate.route_3d ? candidate.route_3d->size() : 0U,
      .reaches_mission_goal = candidate.global_guide_reaches_mission_goal,
      .activation_eligible = activation_evidence.physical_executable,
  };

  const bool raw_validation_required = candidate.observed_occupancy != nullptr;
  const std::uint64_t required_objective_sample =
      snapshot.objective && snapshot.objective->mission_epoch ==
                                snapshot.minimum_tracking_route_mission_epoch
          ? snapshot.minimum_tracking_route_sample_sequence
          : 0U;
  result.required_objective_sample = required_objective_sample;
  result.assessment = assessRouteActivation3D(
      activation_identity,
      candidate.route_3d ? std::span<const RouteSample3D>{*candidate.route_3d}
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
          .maximum_cross_track_m = active_guide_config_.maximum_cross_track_m,
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
                  .radius_m = lattice_3d_config_.physical_footprint_radius_m,
                  .lower_extent_m =
                      lattice_3d_config_.physical_footprint_lower_extent_m,
                  .upper_extent_m =
                      lattice_3d_config_.physical_footprint_upper_extent_m,
                  .perimeter_samples = physical_footprint_config_.perimeter_samples,
                  .radial_rings = physical_footprint_config_.radial_rings,
                  .axial_samples = physical_footprint_config_.axial_samples,
                  .sweep_step_m = physical_footprint_config_.sweep_step_m},
          .proprioceptive_free_space_seed =
              candidate.proprioceptive_free_space_seed
                  ? std::addressof(*candidate.proprioceptive_free_space_seed)
                  : nullptr,
          .launch_support_contact =
              candidate.launch_support_contact
                  ? std::addressof(*candidate.launch_support_contact)
                  : nullptr,
          .raw_validation_required = raw_validation_required,
      });
  result.world_compatible = result.assessment.publication.compatible();
  result.objective_matches = result.assessment.objective_matches;
  if (raw_validation_required && !result.assessment.raw_world_compatible &&
      result.world_compatible && result.objective_matches) {
    result.validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kInvalidEsdf};
  } else if (result.assessment.raw_validation.status ==
             RawRouteSuffixStatus3D::kRawCollision) {
    result.validation = StaticRouteCandidateValidation{
        .status = StaticRouteCandidateStatus::kRawCollision,
        .failure_segment_index = result.assessment.raw_validation.failure_route_segment,
        .failure_point = result.assessment.raw_validation.failure_point,
    };
  }

  const bool raw_route_valid = !raw_validation_required ||
                               (result.assessment.raw_world_compatible &&
                                result.assessment.raw_validation.accepted() &&
                                result.assessment.raw_validation.connector_validated &&
                                result.assessment.raw_validation.suffix_validated);
  activation_evidence.validated_through_revision =
      result.assessment.raw_validated_through_revision;
  activation_evidence.physical_executable =
      candidate.lattice_executable && result.validation.accepted && raw_route_valid;
  if (activation_evidence.physical_executable) {
    activation_evidence.status = SegmentEvidenceStatus3D::kValid;
  } else if (result.validation.status == StaticRouteCandidateStatus::kRawCollision) {
    activation_evidence.status = SegmentEvidenceStatus3D::kRawCollision;
    activation_evidence.raw_collision = true;
    activation_evidence.failure_segment_index = result.validation.failure_segment_index;
    activation_evidence.failure_point = result.validation.failure_point;
  } else if (raw_validation_required && !result.assessment.raw_world_compatible) {
    activation_evidence.status = SegmentEvidenceStatus3D::kInvalidWorld;
  }
  candidate.route_segment_evidence = activation_evidence;
  candidate.static_route_candidate_status = result.validation.status;
  validated_world_certificate.raw_validated_through_revision =
      result.assessment.raw_validated_through_revision;

  ProductionRouteGeometry3D execution_geometry{
      .mppi_route = candidate.mppi_route,
      .route = candidate.route_3d,
      .route_2d_projection = candidate.route_2d_projection,
      .constrained_spans = candidate.constrained_spans,
      .passage_volumes = candidate.passage_volumes,
      .cooperative_passage_assignments = candidate.cooperative_passage_assignments,
      .selected_passage_traversal_ids = candidate.selected_passage_traversal_ids,
      .passage_volume_config = cooperative_passage_volume_config_,
      .route_purpose = candidate.lattice_3d_route_purpose,
      .observation_frontier = candidate.lattice_3d_observation_frontier,
      .materialized_route_fingerprint = candidate.route_fingerprint,
      .physical_route_fingerprint =
          candidate.route_3d ? routeFingerprint(*candidate.route_3d) : 0U,
  };
  execution_geometry.executable_geometry_revision =
      executionRouteGeometryRevision3D(execution_geometry);

  result.proposal = ProductionMaterializedRouteProposal3D{
      .identity =
          MaterializedRouteProposal3D{
              .planned_world = planned_world_certificate,
              .validated_world = validated_world_certificate,
              .objective = search_world.search_objective,
              .intent = candidate.route_intent,
              .evidence = activation_evidence,
              .route_fingerprint = candidate.route_fingerprint,
              .route_sample_count =
                  candidate.route_3d ? candidate.route_3d->size() : 0U,
              .reaches_mission_goal = candidate.global_guide_reaches_mission_goal,
              .activation_eligible = activation_evidence.physical_executable,
          },
      .geometry = std::move(execution_geometry),
  };

  const bool handoff_control_fresh = appliedControlAuthoritativeForExecution(
      snapshot.applied_control, snapshot.execution_horizon_owner, snapshot.stamp_ns,
      maximum_control_feedback_age_ms_);
  if (result.proposal.identity.activation_eligible && result.assessment.accepted() &&
      candidate.mppi_route && candidate.distances_m && snapshot.navigation.valid) {
    result.handoff = mppi::validateStaticRouteHandoff(
        snapshot.navigation.state,
        handoff_control_fresh ? snapshot.applied_control.control : mppi::Control{},
        *candidate.mppi_route,
        static_cast<float>(speed_policy_config_.cruise_speed_mps),
        static_cast<float>(active_guide_config_.maximum_cross_track_m), mppi_config_,
        candidate.grid, *candidate.distances_m);
  }

  result.proposal.identity.activation_eligible =
      result.proposal.identity.activation_eligible && result.assessment.accepted() &&
      result.handoff.accepted && result.proposal.geometry.route &&
      result.proposal.geometry.constrained_spans && result.world_compatible &&
      result.objective_matches;

  if (result.validation.accepted && !result.world_compatible) {
    result.activation_status = StaticRouteActivationStatus::kWorldPublicationRejected;
  } else if (result.validation.accepted && !result.objective_matches) {
    result.activation_status = StaticRouteActivationStatus::kStaleObjective;
  } else if (result.validation.accepted && !result.executionGeometryValid()) {
    result.activation_status = StaticRouteActivationStatus::kInvalidExecutionGeometry;
  } else if (result.validation.accepted && !result.readyForArbitration()) {
    result.activation_status = StaticRouteActivationStatus::kDynamicHandoffRejected;
  }
  candidate.static_route_activation_status = result.activation_status;
  candidate.static_route_publication_status = result.assessment.publication.status;
  candidate.static_route_world_compatible = result.world_compatible;
  return result;
}

void ProductionMppiNode::commitRouteActivation3D(
    const ProductionMppiPreparedEsdf& search_world,
    const ProductionRouteActivationSnapshot3D& snapshot,
    const std::uint64_t candidate_generation,
    const RouteStrategyArbitrationDecision3D& strategy_decision,
    ProductionRouteActivationResult3D& result) {
  ProductionMppiPreparedEsdf& candidate = result.prepared;
  const ProductionMaterializedRouteProposal3D& materialized_proposal = result.proposal;
  const bool raw_validation_required = candidate.observed_occupancy != nullptr;

  const std::shared_ptr<const ExecutionRouteSnapshot3D> current_execution =
      execution_route_store_.snapshot();
  const std::uint64_t base_generation =
      current_execution != nullptr ? current_execution->routeGenerationHighWater() : 0U;
  const std::uint64_t required_base_generation =
      search_world.static_route_replan_request
          ? search_world.static_route_replan_base_generation
          : search_world.static_route_extension_base_generation;
  const bool base_generation_matches = (!search_world.static_route_extension_request &&
                                        !search_world.static_route_replan_request) ||
                                       required_base_generation == base_generation;
  const bool allocation_generation_matches =
      base_generation != std::numeric_limits<std::uint64_t>::max() &&
      candidate_generation == base_generation + 1U;
  result.generation_matches = base_generation_matches && allocation_generation_matches;
  result.snapshot_current =
      sameExecutionRouteBase(snapshot.execution_snapshot, current_execution);
  const ActivatedRouteIdentity3D* const active_identity =
      current_execution != nullptr && current_execution->route.has_value()
          ? std::addressof(current_execution->route->identity)
          : nullptr;
  result.replacement = assessRouteProposalReplacement3D(
      active_identity, materialized_proposal.identity,
      RouteProposalReplacementObservation3D{
          .safety_replan_requested = search_world.static_route_replan_request &&
                                     search_world.static_route_replan_reason ==
                                         GlobalGuideReleaseReason::kBlocked});

  const ActivatedRouteIdentity3D candidate_identity{
      .generation = candidate_generation,
      .proposal = materialized_proposal.identity,
  };
  const bool execution_geometry_valid =
      result.candidate_generation == candidate_generation &&
      executionRouteGeometryValid3D(materialized_proposal.geometry, candidate_identity);
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
        candidate.proprioceptive_free_space_seed, candidate.launch_support_contact);
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
  const std::optional<CertifiedRouteSuffix3D> certified_route =
      result.readyForArbitration() && result.generation_matches &&
              result.snapshot_current && result.replacement.replacementAllowed() &&
              execution_geometry_valid
          ? certifyExecutionRoute3D(ExecutionRouteActivation3D{
                .route_generation = candidate_generation,
                .proposal = materialized_proposal.identity,
                .geometry = std::make_shared<const ProductionRouteGeometry3D>(
                    materialized_proposal.geometry),
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
                            result.required_objective_sample,
                        .position = {snapshot.navigation.state.x,
                                     snapshot.navigation.state.y,
                                     snapshot.navigation.state.z},
                        .maximum_cross_track_m =
                            active_guide_config_.maximum_cross_track_m,
                        .footprint =
                            SweptFootprintConfig{
                                .radius_m =
                                    lattice_3d_config_.physical_footprint_radius_m,
                                .lower_extent_m =
                                    lattice_3d_config_
                                        .physical_footprint_lower_extent_m,
                                .upper_extent_m =
                                    lattice_3d_config_
                                        .physical_footprint_upper_extent_m,
                                .perimeter_samples =
                                    physical_footprint_config_.perimeter_samples,
                                .radial_rings = physical_footprint_config_.radial_rings,
                                .axial_samples =
                                    physical_footprint_config_.axial_samples,
                                .sweep_step_m =
                                    physical_footprint_config_.sweep_step_m},
                        .raw_validation_required = raw_validation_required,
                    },
                .passage_volume_config = cooperative_passage_volume_config_,
                .continuity_lineage = continuity_lineage,
                .observed_raw_world = observed_owner,
                .static_world = static_owner,
                .validation_policy = execution_validation_policy_,
            })
          : std::nullopt;

  const bool route_base =
      current_execution != nullptr && current_execution->route.has_value();
  if (certified_route.has_value() && route_base) {
    result.splice = certifyRouteSplice3D(*current_execution->route, *certified_route,
                                         Point3{snapshot.navigation.state.x,
                                                snapshot.navigation.state.y,
                                                snapshot.navigation.state.z},
                                         certified_route_splice_config_);
  }
  const bool splice_ready = !route_base || result.splice.certified();

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
                .base_kind = current_execution != nullptr
                                 ? pendingExecutionBaseKind(*current_execution)
                                 : PendingExecutionBaseKind3D::kEmpty,
                .base_route_generation = base_generation,
                .base_geometry_revision =
                    current_execution != nullptr && current_execution->route.has_value()
                        ? current_execution->route->geometry
                              ->executable_geometry_revision
                        : 0U,
                .base_continuity_id =
                    current_execution != nullptr && current_execution->route.has_value()
                        ? current_execution->route->continuity_id
                        : 0U,
                .base_direct_tracking_identity =
                    current_execution != nullptr &&
                            current_execution->direct_tracking_execution.has_value()
                        ? std::optional<
                              DirectTrackingOwnerIdentity3D>{current_execution
                                                                 ->direct_tracking_execution
                                                                 ->identity}
                        : std::nullopt,
                .route_splice = route_base ? result.splice.splice : std::nullopt,
                .strategy_decision = strategy_decision,
                .route = *certified_route,
            })
          : nullptr;

  bool published_pending{false};
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_,
                                world_generation_publication_mutex_, esdf_state_mutex_};
    const bool resident_world_current =
        prepared_esdf_ && snapshot.resident_world &&
        sameActivationWorld(*prepared_esdf_, *snapshot.resident_world);
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
        sameExecutionRouteBase(current_execution, execution_route_store_.snapshot());
    const bool candidate_world_coherent = productionWorldGenerationCoherent(candidate);
    result.snapshot_current = resident_world_current && objective_current &&
                              raw_world_current && execution_base_current &&
                              candidate_world_coherent;
    if (pending != nullptr && result.snapshot_current) {
      published_pending = pending_certified_route_mailbox_.publish(pending);
    }
    if (published_pending) {
      result.activation_status = StaticRouteActivationStatus::kCertifiedPending;
      candidate.static_route_generation_matches = true;
      candidate.static_route_world_compatible = true;
      candidate.global_guide_generation = candidate_generation;
      candidate.route_objective = search_world.search_objective;
      candidate.global_guide_release_reason = GlobalGuideReleaseReason::kNone;
      candidate.static_route_extension_request = false;
      candidate.static_route_extension_base_generation = 0U;
      candidate.static_route_replan_request = false;
      candidate.static_route_replan_base_generation = 0U;
      candidate.static_route_replan_reason = GlobalGuideReleaseReason::kNone;
      prepared_esdf_ = candidate;
      result.certified_pending = true;
    }
  }
  if (!published_pending && result.validation.accepted &&
      !result.replacement.replacementAllowed()) {
    result.activation_status =
        StaticRouteActivationStatus::kEquivalentActiveSegmentRetained;
  } else if (!published_pending && result.validation.accepted &&
             !result.world_compatible) {
    result.activation_status = StaticRouteActivationStatus::kWorldPublicationRejected;
  } else if (!published_pending && result.validation.accepted &&
             !result.snapshot_current) {
    result.activation_status =
        StaticRouteActivationStatus::kActivationSnapshotSuperseded;
  } else if (!published_pending && result.validation.accepted &&
             !result.generation_matches) {
    result.activation_status = StaticRouteActivationStatus::kStaleRouteGeneration;
  } else if (!published_pending && result.validation.accepted &&
             !result.objective_matches) {
    result.activation_status = StaticRouteActivationStatus::kStaleObjective;
  } else if (!published_pending && result.validation.accepted &&
             !execution_geometry_valid) {
    result.activation_status = StaticRouteActivationStatus::kInvalidExecutionGeometry;
  } else if (!published_pending && result.validation.accepted && route_base &&
             certified_route.has_value() && !result.splice.certified()) {
    result.activation_status = StaticRouteActivationStatus::kCertifiedSpliceRejected;
  } else if (!published_pending && result.validation.accepted &&
             (!result.assessment.accepted() || !result.handoff.accepted ||
              !certified_route.has_value())) {
    result.activation_status = StaticRouteActivationStatus::kDynamicHandoffRejected;
  }
  candidate.static_route_activation_status = result.activation_status;
  candidate.static_route_publication_status = result.assessment.publication.status;
  candidate.static_route_world_compatible = result.world_compatible;
  candidate.static_route_generation_matches = result.generation_matches;
}

} // namespace drone_city_nav
