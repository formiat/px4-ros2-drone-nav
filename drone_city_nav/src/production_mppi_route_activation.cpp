#include "production_mppi_route_activation.hpp"

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

struct ProductionRouteActivationSnapshot3D {
  std::optional<ProductionMppiPreparedEsdf> resident_world;
  ProductionMppiNavigation navigation{};
  ProductionMppiAppliedControl applied_control{};
  std::shared_ptr<const ProductionNavigationObjective> objective;
  std::shared_ptr<const ProductionMppiRawWorld3D> raw_world;
  std::uint64_t minimum_tracking_route_mission_epoch{0U};
  std::uint64_t minimum_tracking_route_sample_sequence{0U};
  std::int64_t stamp_ns{0};
};

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
  return current.local_world_generation.generation ==
             captured.local_world_generation.generation &&
         current.producer_instance_id == captured.producer_instance_id &&
         current.revision == captured.revision &&
         current.source_raw_revision == captured.source_raw_revision;
}

} // namespace

ProductionRouteActivationResult3D ProductionMppiNode::finalizeRouteActivation3D(
    const ProductionMppiPreparedEsdf& search_world, ProductionMppiPreparedEsdf prepared,
    const NavigationWorldCertificate3D planned_world_certificate,
    StaticRouteCandidateValidation validation,
    const StaticRouteReplacementPolicy replacement_policy, const Point3& mission_goal,
    const std::uint64_t candidate_generation) {
  ProductionRouteActivationResult3D result{
      .prepared = std::move(prepared),
      .validation = validation,
  };
  result.activation_status =
      result.prepared.lattice_executable
          ? StaticRouteActivationStatus::kCandidateValidationRejected
          : StaticRouteActivationStatus::kCandidateNotExecutable;
  ProductionMppiPreparedEsdf& candidate = result.prepared;

  ProductionRouteActivationSnapshot3D snapshot;
  {
    const std::scoped_lock lock{input_mutex_, esdf_state_mutex_};
    snapshot.resident_world = prepared_esdf_;
    snapshot.navigation = navigation_;
    snapshot.applied_control = applied_control_;
    snapshot.objective = navigationObjective();
    snapshot.raw_world = latest_raw_world_3d_.load(std::memory_order_acquire);
    snapshot.minimum_tracking_route_mission_epoch =
        minimum_tracking_route_mission_epoch_.load(std::memory_order_acquire);
    snapshot.minimum_tracking_route_sample_sequence =
        minimum_tracking_route_sample_sequence_.load(std::memory_order_acquire);
    snapshot.stamp_ns = get_clock()->now().nanoseconds();
  }
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
      candidate.mppi_route =
          makeMppiRoute3D(*rebased_route, *candidate.constrained_spans,
                          speed_policy_config_.cruise_speed_mps,
                          constrained_route_speed_limit_mps_, speed_policy_config_);
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

  const ProductionMaterializedRouteProposal3D materialized_proposal{
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
      .geometry =
          ProductionRouteGeometry3D{
              .mppi_route = candidate.mppi_route,
              .route = candidate.route_3d,
              .route_2d_projection = candidate.route_2d_projection,
              .constrained_spans = candidate.constrained_spans,
              .passage_volumes = candidate.passage_volumes,
              .cooperative_passage_assignments =
                  candidate.cooperative_passage_assignments,
              .selected_passage_traversal_ids =
                  candidate.selected_passage_traversal_ids,
              .route_purpose = candidate.lattice_3d_route_purpose,
              .observation_frontier = candidate.lattice_3d_observation_frontier,
          },
  };

  const bool handoff_control_fresh =
      snapshot.applied_control.valid && snapshot.applied_control.receive_stamp_ns > 0 &&
      snapshot.stamp_ns >= snapshot.applied_control.receive_stamp_ns &&
      static_cast<double>(snapshot.stamp_ns -
                          snapshot.applied_control.receive_stamp_ns) *
              1.0e-6 <=
          maximum_control_feedback_age_ms_;
  if (materialized_proposal.identity.activation_eligible &&
      result.assessment.accepted() && candidate.mppi_route && candidate.distances_m &&
      snapshot.navigation.valid) {
    result.handoff = mppi::validateStaticRouteHandoff(
        snapshot.navigation.state,
        handoff_control_fresh ? snapshot.applied_control.control : mppi::Control{},
        *candidate.mppi_route,
        static_cast<float>(speed_policy_config_.cruise_speed_mps),
        static_cast<float>(active_guide_config_.maximum_cross_track_m), mppi_config_,
        candidate.grid, *candidate.distances_m);
  }

  {
    const std::scoped_lock lock{esdf_state_mutex_, route_supervisor_mutex_};
    const std::uint64_t required_base_generation =
        search_world.static_route_replan_request
            ? search_world.static_route_replan_base_generation
            : search_world.static_route_extension_base_generation;
    const bool base_generation_matches =
        (!search_world.static_route_extension_request &&
         !search_world.static_route_replan_request) ||
        (prepared_esdf_ &&
         prepared_esdf_->global_guide_generation == required_base_generation);
    const bool allocation_generation_matches =
        candidate_generation != 0U &&
        route_supervisor_.lastAllocatedGeneration() !=
            std::numeric_limits<std::uint64_t>::max() &&
        route_supervisor_.lastAllocatedGeneration() + 1U == candidate_generation;
    result.generation_matches =
        base_generation_matches && allocation_generation_matches;
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
    result.snapshot_current =
        resident_world_current && objective_current && raw_world_current;
    result.replacement = assessRouteProposalReplacement3D(
        route_supervisor_.activeRoute(), materialized_proposal.identity,
        RouteProposalReplacementObservation3D{
            .safety_replan_requested = search_world.static_route_replan_request &&
                                       search_world.static_route_replan_reason ==
                                           GlobalGuideReleaseReason::kBlocked});

    std::optional<ActivatedRouteIdentity3D> activated_identity;
    if (materialized_proposal.identity.activation_eligible &&
        result.assessment.accepted() && result.handoff.accepted &&
        materialized_proposal.geometry.route &&
        materialized_proposal.geometry.constrained_spans && result.world_compatible &&
        result.generation_matches && result.objective_matches &&
        result.snapshot_current && result.replacement.replacementAllowed()) {
      const std::optional<std::uint64_t> activated_generation =
          route_supervisor_.activate(materialized_proposal.identity);
      if (activated_generation == candidate_generation &&
          route_supervisor_.activeRoute() != nullptr) {
        activated_identity = *route_supervisor_.activeRoute();
      }
    }
    if (activated_identity.has_value()) {
      result.activation_status = StaticRouteActivationStatus::kActivated;
      candidate.static_route_activation_status = result.activation_status;
      candidate.static_route_generation_matches = true;
      candidate.static_route_publication_status = result.assessment.publication.status;
      candidate.static_route_world_compatible = true;
      candidate.global_guide_generation = activated_identity->generation;
      candidate.route_objective = search_world.search_objective;
      candidate.activated_route_3d =
          std::make_shared<const ProductionActivatedRoute3D>(ProductionActivatedRoute3D{
              .identity = *activated_identity,
              .geometry = materialized_proposal.geometry,
          });
      candidate.global_guide_release_reason = GlobalGuideReleaseReason::kNone;
      candidate.static_route_extension_request = false;
      candidate.static_route_extension_base_generation = 0U;
      candidate.static_route_replan_request = false;
      candidate.static_route_replan_base_generation = 0U;
      candidate.static_route_replan_reason = GlobalGuideReleaseReason::kNone;
      prepared_esdf_ = candidate;
      result.activated = true;
    } else if (result.validation.accepted && !result.replacement.replacementAllowed()) {
      result.activation_status =
          StaticRouteActivationStatus::kEquivalentActiveSegmentRetained;
    } else if (result.validation.accepted && !result.world_compatible) {
      result.activation_status = StaticRouteActivationStatus::kWorldPublicationRejected;
    } else if (result.validation.accepted && !result.snapshot_current) {
      result.activation_status =
          StaticRouteActivationStatus::kActivationSnapshotSuperseded;
    } else if (result.validation.accepted && !result.generation_matches) {
      result.activation_status = StaticRouteActivationStatus::kStaleRouteGeneration;
    } else if (result.validation.accepted && !result.objective_matches) {
      result.activation_status = StaticRouteActivationStatus::kStaleObjective;
    } else if (result.validation.accepted &&
               (!result.assessment.accepted() || !result.handoff.accepted)) {
      result.activation_status = StaticRouteActivationStatus::kDynamicHandoffRejected;
    }
    candidate.static_route_activation_status = result.activation_status;
    candidate.static_route_publication_status = result.assessment.publication.status;
    candidate.static_route_world_compatible = result.world_compatible;
    candidate.static_route_generation_matches = result.generation_matches;
  }
  return result;
}

} // namespace drone_city_nav
