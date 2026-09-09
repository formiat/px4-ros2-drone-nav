#include "route_activation_coordinator_3d.hpp"

#include <stdexcept>
#include <utility>

#include "production_mppi_route_world.hpp"
#include "route_activation_preparation_3d.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool sameActivationWorld(const WorldSnapshot3D& current,
                                       const WorldSnapshot3D& captured) noexcept {
  return productionWorldGenerationCoherent(current) &&
         productionWorldGenerationCoherent(captured) &&
         current.local_world_generation.sameSnapshot(captured.local_world_generation);
}

} // namespace

bool RouteAdmissionReport3D::compiledTrajectoryValid() const noexcept {
  return trajectory_validation.valid() && decoration_validation.valid();
}

bool RouteActivationPreparationRequest3D::valid() const noexcept {
  return transaction != nullptr && transaction->valid() &&
         materialization.route.world != nullptr &&
         materialization.route.candidate_generation != 0U;
}

RouteActivationCoordinator3D::RouteActivationCoordinator3D(
    const RouteActivationCoordinatorConfig3D& config)
    : config_{config},
      trajectory_compiler_{config.trajectory_compiler} {
  if (!config_.successor_improvement.valid() || !config_.route_risk.valid() ||
      !config_.dynamic_handoff_validator) {
    throw std::invalid_argument{"invalid route activation coordinator configuration"};
  }
}

bool pendingRoutePublicationBaseCurrent3D(
    const PendingRoutePublicationCurrentness3D& currentness) noexcept {
  return currentness.resident_world_current && currentness.objective_current &&
         currentness.execution_base_current && currentness.pending_current &&
         currentness.candidate_world_coherent;
}

bool RouteAdmissionReport3D::readyForArbitration(
    const ProductionMaterializedRouteProposal3D& proposal) const noexcept {
  if (proposal.trajectory == nullptr || proposal.decorations == nullptr) {
    return false;
  }
  const CompiledTrajectory3D& trajectory = *proposal.trajectory;
  return proposal.identity.activation_eligible && assessment.accepted() &&
         handoff.accepted() && trajectory.route && trajectory.constrained_spans &&
         trajectory.materialized_route_fingerprint ==
             proposal.identity.route_fingerprint &&
         trajectory.compiled_trajectory_revision != 0U &&
         trajectory.compiled_trajectory_revision ==
             compiledTrajectoryRevision3D(trajectory) &&
         routeDecorationsValid3D(*proposal.decorations, trajectory,
                                 proposal.decorations->route_generation) &&
         trajectory_validation.valid() && decoration_validation.valid() &&
         world_compatible && objective_matches;
}

PreparedRouteActivation3D RouteActivationCoordinator3D::prepare(
    RouteActivationPreparationRequest3D request) const {
  return prepareRouteActivation3D(std::move(request), config_, trajectory_compiler_);
}

RouteActivationCommitResult3D RouteActivationCoordinator3D::commit(
    PreparedRouteActivation3D prepared, const RouteActivationCommitContext3D& context,
    ExecutionSupervisor3D& execution_supervisor) const {
  RouteActivationCommitResult3D committed{
      .result = std::move(prepared.result),
  };
  ProductionRouteActivationResult3D& result = committed.result;
  MaterializedRoute3D& candidate = result.materialized;
  RouteAdmissionReport3D& report = result.admission;
  if (candidate.world == nullptr) {
    report.activation_status = StaticRouteActivationStatus::kActivationCommitRejected;
    return committed;
  }
  const ProductionRouteActivationSnapshot3D& snapshot = prepared.snapshot;
  const bool raw_validation_required = candidate.world->observed_occupancy != nullptr;
  bool published_pending{false};
  const bool resident_world_current =
      context.resident_world != nullptr && snapshot.resident_world != nullptr &&
      sameActivationWorld(*context.resident_world, *snapshot.resident_world);
  const bool objective_current = context.objective == snapshot.objective &&
                                 context.minimum_tracking_route_mission_epoch ==
                                     snapshot.minimum_tracking_route_mission_epoch &&
                                 context.minimum_tracking_route_sample_sequence ==
                                     snapshot.minimum_tracking_route_sample_sequence;
  const bool raw_world_current =
      !raw_validation_required || context.raw_world == snapshot.raw_world;
  bool execution_base_current =
      sameExecutionRouteBase3D(prepared.execution_base, execution_supervisor.plan());
  bool pending_current = snapshot.pending_route == execution_supervisor.pending();
  const bool candidate_world_coherent =
      productionWorldGenerationCoherent(*candidate.world);
  report.resident_world_snapshot_current = resident_world_current;
  report.objective_snapshot_current = objective_current;
  report.raw_snapshot_current = raw_world_current;
  report.execution_base_snapshot_current = execution_base_current;
  report.pending_snapshot_current = pending_current;
  report.candidate_world_coherent = candidate_world_coherent;
  report.snapshot_current =
      pendingRoutePublicationBaseCurrent3D(PendingRoutePublicationCurrentness3D{
          .resident_world_current = resident_world_current,
          .objective_current = objective_current,
          .raw_world_current = raw_world_current,
          .execution_base_current = execution_base_current,
          .pending_current = pending_current,
          .candidate_world_coherent = candidate_world_coherent,
      });
  if (prepared.pending_draft.has_value() && report.snapshot_current) {
    const PendingRoutePublicationResult3D publication =
        snapshot.pending_route != nullptr
            ? execution_supervisor.replacePendingForCurrentBase(
                  prepared.execution_base, snapshot.pending_route,
                  std::move(*prepared.pending_draft))
            : execution_supervisor.publishPendingForCurrentBase(
                  prepared.execution_base, std::move(*prepared.pending_draft));
    report.pending_publication_status = publication.status;
    prepared.pending_draft.reset();
    execution_base_current =
        publication.status != PendingRoutePublicationStatus3D::kStaleExecutionBase;
    pending_current =
        publication.status != PendingRoutePublicationStatus3D::kPendingChanged &&
        publication.status != PendingRoutePublicationStatus3D::kPendingOccupied;
    report.execution_base_snapshot_current = execution_base_current;
    report.pending_snapshot_current = pending_current;
    report.snapshot_current =
        pendingRoutePublicationBaseCurrent3D(PendingRoutePublicationCurrentness3D{
            .resident_world_current = resident_world_current,
            .objective_current = objective_current,
            .raw_world_current = raw_world_current,
            .execution_base_current = execution_base_current,
            .pending_current = pending_current,
            .candidate_world_coherent = candidate_world_coherent,
        });
    published_pending = publication.published();
  }
  if (published_pending) {
    report.activation_status = StaticRouteActivationStatus::kCertifiedPending;
    report.generation_matches = true;
    report.world_compatible = true;
    report.certified_pending = true;
  }
  const bool overlap_search =
      candidate.provenance.required_splice_base_route_instance_id.valid();
  const bool compiled_trajectory_valid =
      candidate.candidate_generation != 0U && report.trajectory_validation.valid() &&
      report.decoration_validation.valid() && result.proposal.trajectory != nullptr &&
      result.proposal.decorations != nullptr &&
      compiledTrajectoryValid3D(*result.proposal.trajectory,
                                ActivatedRouteIdentity3D{
                                    .generation = candidate.candidate_generation,
                                    .proposal = result.proposal.identity,
                                }) &&
      routeDecorationsValid3D(*result.proposal.decorations, *result.proposal.trajectory,
                              candidate.candidate_generation);
  if (!published_pending && report.candidate_validation.accepted &&
      report.successor_improvement_required &&
      !report.successor_improvement.accepted()) {
    report.activation_status =
        StaticRouteActivationStatus::kInsufficientSuccessorImprovement;
  } else if (!published_pending && report.candidate_validation.accepted &&
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
             report.pending_publication_status.has_value()) {
    report.activation_status = StaticRouteActivationStatus::kActivationCommitRejected;
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
             overlap_search && report.route_certified && !report.splice.certified()) {
    report.activation_status = StaticRouteActivationStatus::kCertifiedSpliceRejected;
  } else if (!published_pending && report.candidate_validation.accepted &&
             (!report.assessment.accepted() || !report.handoff.accepted())) {
    report.activation_status = StaticRouteActivationStatus::kDynamicHandoffRejected;
  } else if (!published_pending && report.candidate_validation.accepted &&
             !report.route_certified) {
    report.activation_status = StaticRouteActivationStatus::kRouteCertificationRejected;
  }
  return committed;
}

} // namespace drone_city_nav
