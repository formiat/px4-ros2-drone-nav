#include "drone_city_nav/mppi/static_route_handoff.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "production_mppi_node.hpp"
#include "production_mppi_route_activation.hpp"
#include "production_mppi_route_helpers.hpp"
#include "production_mppi_route_materialization.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {

void ProductionMppiNode::processGuideSearch3D(
    const ProductionMppiPreparedEsdf& world,
    const ProductionMppiNavigation& navigation) {
  const auto planning_started = std::chrono::steady_clock::now();
  const Point3 mission_goal =
      world.search_objective.available ? world.search_objective.goal : mission_goal_;
  const NavigationWorldCertificate3D planned_world_certificate =
      navigationWorldCertificate3D(world);
  const std::shared_ptr<const ProductionMppiRawWorld3D> latest_raw_world_3d =
      latest_raw_world_3d_.load(std::memory_order_acquire);
  const Point3 search_start{navigation.state.x, navigation.state.y, navigation.state.z};
  const RouteSegmentCompletionAssessment3D active_route_completion =
      assessActiveRouteCompletion3D(world, search_start);
  const bool active_observation_segment_completed =
      world.lattice_3d_route_purpose == Lattice3DRoutePurpose::kObservationFrontier &&
      world.lattice_3d_observation_frontier && active_route_completion.captured;
  if (topological_navigation_3d_ && active_observation_segment_completed) {
    if (world.route_intent.valid && world.route_intent.segment_reaches_intent_target) {
      topological_navigation_3d_->completeObservationFrontier(
          *world.lattice_3d_observation_frontier, world.revision);
      RCLCPP_INFO(get_logger(),
                  "INCREMENTAL_TOPOLOGY3D_FRONTIER_COMPLETED frontier_id=%" PRIu64
                  " route_endpoint_reached=true intent_id=%" PRIu64
                  " endpoint_distance_m=%.3f route_remaining_m=%.3f "
                  "capture_radius_m=%.3f",
                  world.lattice_3d_observation_frontier->id.value,
                  world.route_intent.id, active_route_completion.endpoint_distance_m,
                  active_route_completion.projection.remaining_m,
                  topological_lattice_adapter_3d_config_.segment_capture_radius_m);
    } else {
      RCLCPP_INFO(get_logger(),
                  "INCREMENTAL_TOPOLOGY3D_SEGMENT_COMPLETED frontier_id=%" PRIu64
                  " intent_id=%" PRIu64
                  " intent_target_reached=false endpoint_distance_m=%.3f "
                  "route_remaining_m=%.3f capture_radius_m=%.3f",
                  world.lattice_3d_observation_frontier->id.value,
                  world.route_intent.id, active_route_completion.endpoint_distance_m,
                  active_route_completion.projection.remaining_m,
                  topological_lattice_adapter_3d_config_.segment_capture_radius_m);
    }
    requestGuideRelease(GlobalGuideReleaseReason::kExhausted,
                        world.global_guide_generation);
  }

  ProductionRouteCandidateSet3D candidate_set =
      generateRouteCandidates3D(world, navigation, mission_goal, latest_raw_world_3d);
  const std::uint64_t candidate_generation = nextRouteGeneration3D();

  std::vector<ProductionRouteMaterialization3D> materializations;
  materializations.reserve(candidate_set.candidates.size());
  for (const ProductionRouteSearchCandidate3D& candidate : candidate_set.candidates) {
    materializations.push_back(materializeRouteCandidate3D(
        world, navigation, mission_goal, candidate, candidate_generation,
        active_observation_segment_completed));
  }

  const ProductionRouteActivationSnapshot3D activation_snapshot =
      captureRouteActivationSnapshot3D();
  std::vector<ProductionRouteActivationResult3D> activations;
  std::vector<RouteProposal3D> final_proposals;
  activations.reserve(materializations.size());
  final_proposals.reserve(materializations.size());
  for (ProductionRouteMaterialization3D& materialization : materializations) {
    activations.push_back(prepareRouteActivation3D(
        world, std::move(materialization.prepared), planned_world_certificate,
        materialization.validation, materialization.replacement_policy, mission_goal,
        candidate_generation, activation_snapshot));
    const ProductionRouteActivationResult3D& activation = activations.back();
    final_proposals.push_back(RouteProposal3D{
        .intent = activation.proposal.identity.intent,
        .evidence = activation.proposal.identity.evidence,
        .route_fingerprint = activation.proposal.identity.route_fingerprint,
        .activation_eligible = activation.readyForArbitration(),
    });
  }

  const RouteStrategyArbitrationObservation3D strategy_observation{
      .position = search_start,
      .mission_target = mission_goal,
      .world_revision = world.revision,
      .active_intent = world.route_intent.valid
                           ? std::optional<RouteIntent3D>{world.route_intent}
                           : std::nullopt,
      .active_intent_completed = active_route_completion.captured &&
                                 world.route_intent.valid &&
                                 world.route_intent.segment_reaches_intent_target,
  };
  const RouteStrategyArbitrationDecision3D strategy_decision =
      route_strategy_arbitrator_3d_.evaluate(
          final_proposals, route_proposal_selection_3d_config_, strategy_observation);
  const RouteProposalSelection3D& proposal_selection = strategy_decision.selection;
  for (std::size_t index = 0U; index < final_proposals.size(); ++index) {
    const RouteProposal3D& proposal = final_proposals[index];
    const ProductionRouteActivationResult3D& activation = activations[index];
    RCLCPP_INFO(
        get_logger(),
        "ROUTE_PROPOSAL3D stage=arbitrated revision=%" PRIu64
        " index=%zu selected=%s intent_id=%" PRIu64 " strategic_plan_id=%" PRIu64
        " source=%s purpose=%s lease_reason=%s return_lineage=%" PRIu64
        " return_anchor=%" PRIu64 " "
        "physical=%s activation_eligible=%s validation=%.*s handoff=%s "
        "raw_connector_validated=%s raw_suffix_validated=%s "
        "route_length_m=%.2f mission_progress_m=%.2f objective=%.3f",
        world.revision, index,
        proposal_selection.selected_index == std::optional<std::size_t>{index}
            ? "true"
            : "false",
        proposal.intent.id, proposal.intent.strategic_plan_id,
        routeIntentSource3DName(proposal.intent.source),
        routeIntentPurpose3DName(proposal.intent.purpose),
        routeStrategyLeaseReason3DName(proposal.intent.lease_reason),
        proposal.intent.return_lineage.id,
        proposal.intent.return_lineage.return_anchor_identity,
        proposal.evidence.physical_executable ? "true" : "false",
        proposal.activation_eligible ? "true" : "false",
        static_cast<int>(
            staticRouteCandidateStatusName(activation.validation.status).size()),
        staticRouteCandidateStatusName(activation.validation.status).data(),
        mppi::staticRouteHandoffStatusName(activation.handoff.status),
        activation.assessment.raw_validation.connector_validated ? "true" : "false",
        activation.assessment.raw_validation.suffix_validated ? "true" : "false",
        proposal.evidence.route_length_m, proposal.evidence.mission_progress_m,
        proposal.evidence.objective_cost);
    ProductionRouteSearchCandidate3D& candidate = candidate_set.candidates[index];
    if (candidate.topology &&
        activation.validation.status == StaticRouteCandidateStatus::kRawCollision) {
      rejectIncrementalTopologyRoute3D(
          *candidate.topology,
          candidate.evidence.status == SegmentEvidenceStatus3D::kRawCollision
              ? ProductionIncrementalTopologyRejectionReason3D::
                    kSegmentEvidenceRawCollision
              : ProductionIncrementalTopologyRejectionReason3D::
                    kMaterializedRouteRawCollision);
    }
  }

  ProductionRouteActivationResult3D empty_activation{.prepared = world};
  ProductionRouteMaterialization3D empty_materialization{.prepared = world};
  ProductionRouteSearchCandidate3D empty_candidate;
  const std::size_t diagnostic_index = proposal_selection.selected_index.value_or(0U);
  const bool candidate_available = diagnostic_index < candidate_set.candidates.size();
  ProductionRouteActivationResult3D& activation =
      candidate_available ? activations[diagnostic_index] : empty_activation;
  ProductionRouteMaterialization3D& materialization =
      candidate_available ? materializations[diagnostic_index] : empty_materialization;
  ProductionRouteSearchCandidate3D& selected_candidate =
      candidate_available ? candidate_set.candidates[diagnostic_index]
                          : empty_candidate;

  activation.prepared.route_proposal_selection_reason = proposal_selection.reason;
  activation.prepared.route_proposal_candidate_count =
      proposal_selection.considered_candidates;
  activation.prepared.route_proposal_eligible_count =
      proposal_selection.eligible_candidates;
  activation.prepared.global_guide_search_ms = candidate_set.search_ms;
  if (proposal_selection.selected_index.has_value()) {
    commitRouteActivation3D(world, activation_snapshot, candidate_generation,
                            activation);
  }
  const double route_planning_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                planning_started)
          .count();
  {
    const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
    static_route_planning_latency_tracker_.record(route_planning_ms, world.build_ms);
  }

  RiskAwareLattice3DResult& lattice = selected_candidate.lattice;
  ProductionIncrementalTopologySearch3D empty_topology;
  const bool topology_route_used =
      candidate_available && selected_candidate.topology.has_value();
  ProductionIncrementalTopologySearch3D& topology =
      topology_route_used ? *selected_candidate.topology : empty_topology;
  const Vec3 preferred_direction =
      candidate_available ? selected_candidate.directive.preferred_direction
                          : candidate_set.preferred_direction;
  const double search_ms = candidate_set.search_ms;
  const std::vector<SelectedPassageTraversal>& route_traversals =
      lattice.selected_passage_traversals;
  const StaticRouteReplacementPolicy replacement_policy =
      materialization.replacement_policy;
  const ObservationRouteReplacementDecision& observation_replacement =
      materialization.observation_replacement;
  ProductionMppiPreparedEsdf prepared = std::move(activation.prepared);
  StaticRouteCandidateValidation validation = activation.validation;
  const StaticRouteActivationStatus activation_status = activation.activation_status;
  const RoutePublicationStatus3D publication_status =
      activation.assessment.publication.status;
  const bool world_compatible = activation.world_compatible;
  const bool publication_world_advanced = activation.publication_world_advanced;
  const bool generation_matches = activation.generation_matches;
  const bool objective_matches = activation.objective_matches;
  const bool activation_snapshot_current = activation.snapshot_current;
  const std::uint64_t required_objective_sample = activation.required_objective_sample;
  const bool observed_world_rebased = activation.observed_world_rebased;
  const bool certified_pending = activation.certified_pending;
  const mppi::StaticRouteHandoffResult& handoff = activation.handoff;

  bool strategy_outcome_recorded{false};
  if (certified_pending) {
    const std::scoped_lock strategy_lock{execution_evidence_commit_mutex_};
    pending_route_strategy_decision_ = strategy_decision;
    strategy_outcome_recorded = true;
  } else {
    strategy_outcome_recorded =
        route_strategy_arbitrator_3d_.recordOutcome(strategy_decision, false);
  }
  const RouteStrategyArbitrationState3D& strategy_state =
      route_strategy_arbitrator_3d_.state();
  const RouteStrategyLease3D* const strategy_lease =
      strategy_state.lease ? std::addressof(*strategy_state.lease) : nullptr;
  const RetiredRouteStrategyLineage3D* const retired_lineage =
      strategy_state.retired_lineage ? std::addressof(*strategy_state.retired_lineage)
                                     : nullptr;
  RCLCPP_INFO(
      get_logger(),
      "ROUTE_STRATEGY_ARBITRATION3D sequence=%" PRIu64
      " action=%.*s selection_committed=%s outcome_recorded=%s "
      "lease_id=%" PRIu64 " lease_kind=%.*s lease_reason=%s "
      "lease_plan_id=%" PRIu64 " lease_target=%" PRIu64 " lease_return_lineage=%" PRIu64
      " lease_budget_remaining_m=%.2f "
      "lease_travelled_m=%.2f direct_advantage_confirmations=%zu "
      "retired_kind=%.*s retired_target=%" PRIu64 " retired_return_lineage=%" PRIu64,
      strategy_decision.sequence,
      static_cast<int>(
          routeStrategyArbitrationAction3DName(strategy_decision.action).size()),
      routeStrategyArbitrationAction3DName(strategy_decision.action).data(),
      certified_pending ? "true" : "false",
      strategy_outcome_recorded ? "true" : "false",
      strategy_lease != nullptr ? strategy_lease->lease_id : 0U,
      static_cast<int>(routeStrategyKind3DName(strategy_lease != nullptr
                                                   ? strategy_lease->kind
                                                   : RouteStrategyKind3D::kNone)
                           .size()),
      routeStrategyKind3DName(strategy_lease != nullptr ? strategy_lease->kind
                                                        : RouteStrategyKind3D::kNone)
          .data(),
      routeStrategyLeaseReason3DName(strategy_lease != nullptr
                                         ? strategy_lease->reason
                                         : RouteStrategyLeaseReason3D::kNone),
      strategy_lease != nullptr ? strategy_lease->strategic_plan_id : 0U,
      strategy_lease != nullptr ? strategy_lease->target_identity : 0U,
      strategy_lease != nullptr ? strategy_lease->return_lineage.id : 0U,
      strategy_lease != nullptr ? strategy_lease->remainingBudgetM() : 0.0,
      strategy_lease != nullptr ? strategy_lease->travelled_m : 0.0,
      strategy_lease != nullptr ? strategy_lease->direct_advantage_confirmations : 0U,
      static_cast<int>(routeStrategyKind3DName(retired_lineage != nullptr
                                                   ? retired_lineage->kind
                                                   : RouteStrategyKind3D::kNone)
                           .size()),
      routeStrategyKind3DName(retired_lineage != nullptr ? retired_lineage->kind
                                                         : RouteStrategyKind3D::kNone)
          .data(),
      retired_lineage != nullptr ? retired_lineage->target_identity : 0U,
      retired_lineage != nullptr ? retired_lineage->return_lineage_id : 0U);

  if (certified_pending) {
    if (topology_route_used) {
      commitIncrementalTopologyRoute3D(topology);
    } else if (topological_navigation_3d_ &&
               topological_navigation_3d_->supersedeAcceptedPlan()) {
      RCLCPP_INFO(
          get_logger(),
          "INCREMENTAL_TOPOLOGY3D_PLAN_SUPERSEDED replacement=non_topology_route");
    }
  }
  if (certified_pending && prepared.cooperative_passage_assignments) {
    for (const CooperativePassageAssignment& assignment :
         *prepared.cooperative_passage_assignments) {
      RCLCPP_INFO(
          get_logger(),
          "COOPERATIVE_PASSAGE_ROUTE route_generation=%" PRIu64
          " span_index=%zu passage='%s' offset_interval_m=[%.2f,%.2f] "
          "secondary_interval_m=[%.2f,%.2f] cross_sections=%zu "
          "raw_volume=%s requested_offset_m=%.2f applied_offset_m=%.2f "
          "status=%s volume_build_ms=%.2f volume_resource_reused=%s",
          assignment.route_generation, assignment.span_index,
          assignment.passage_traversal_id.c_str(), assignment.minimum_lateral_offset_m,
          assignment.maximum_lateral_offset_m, assignment.minimum_secondary_offset_m,
          assignment.maximum_secondary_offset_m, assignment.passage_cross_section_count,
          assignment.passage_volume_raw_validated ? "true" : "false",
          assignment.requested_lateral_offset_m, assignment.applied_lateral_offset_m,
          cooperativePassageRouteStatusName(assignment.status),
          prepared.passage_volume_build_ms,
          prepared.passage_volume_resource_reused ? "true" : "false");
    }
  }
  const char* const route_space =
      world.observed_occupancy ? "observed_occupancy_3d" : "static_occupancy_3d";
  const char* const topology_acceleration =
      world.topological_graph ? "incremental_topological_graph" : "unavailable";
  if (topology_route_used) {
    logIncrementalTopologyRoute3D(topology, lattice, validation, activation_status,
                                  certified_pending);
  }
  const ObservationFrontier* const observation_frontier =
      lattice.observation_frontier ? &*lattice.observation_frontier : nullptr;
  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_GUIDE3D revision=%" PRIu64
      " certified_pending=%s activation_status=%.*s publication_status=%.*s "
      "world_compatible=%s activation_snapshot_current=%s "
      "snapshot_pose_revision=%" PRIu64 " snapshot_raw_revision=%" PRIu64 " "
      "generation_matches=%s objective_matches=%s route_generation=%" PRIu64
      " route_space=%s observed_world_rebased=%s publication_world_advanced=%s "
      "route_purpose=%s "
      "topology_acceleration=%s "
      "observation_frontier_id=%" PRIu64 " observation_frontier_revision=%" PRIu64
      " observation_frontier_rays=%zu observation_frontier_gain=%zu "
      "observation_frontier_score=%.3f "
      "observation_replacement=%.*s observation_score_improvement=%.3f "
      "extension=%s replan=%s "
      "base_generation=%" PRIu64 " replan_reason=%s"
      " replacement_policy=%.*s validation=%.*s endpoint_improvement_m=%.2f "
      "validation_failure_segment=%zu validation_failure=(%.2f,%.2f,%.2f) "
      "raw_connector_validated=%s raw_suffix_validated=%s "
      "raw_validated_from_station_m=%.2f "
      "handoff=%s handoff_cross_track_m=%.2f handoff_minimum_clearance_m=%.2f "
      "handoff_planning_exposure_m=%.2f handoff_critical_exposure_m=%.2f "
      "splice=%.*s splice_overlap_m=%.2f splice_max_separation_m=%.2f "
      "splice_min_tangent_alignment=%.3f "
      "status=%s termination=%s "
      "points=%zu samples=%zu spans=%zu expansions=%zu expansion_limit=%zu "
      "deadline_ms=%.2f "
      "risk_stage=%s start=(%.2f,%.2f,%.2f) planning_goal=(%.2f,%.2f,%.2f) "
      "endpoint=(%.2f,%.2f,%.2f) direction=(%.3f,%.3f,%.3f) "
      "achieved_progress_m=%.2f minimum_clearance_m=%.2f stale_pops=%zu "
      "open_peak=%zu records_peak=%zu terminal_successors=%zu "
      "continuation_states=%zu continuation_depth_m=%.2f "
      "lattice_successor_generated=%zu lattice_successor_accepted=%zu "
      "lattice_successor_reject_edge=%zu lattice_successor_reject_zero=%zu "
      "lattice_successor_reject_outside_roi=%zu "
      "lattice_successor_reject_unknown_space=%zu "
      "lattice_successor_reject_envelope=%zu "
      "lattice_successor_reject_invalid=%zu "
      "lattice_successor_reject_collision=%zu lattice_successor_reject_risk=%zu "
      "lattice_successor_reject_cost=%zu "
      "passage_successor_generated=%zu passage_successor_accepted=%zu "
      "passage_successor_rejected=%zu passage_successor_reject_connection=%zu "
      "passage_successor_reject_outside_roi=%zu "
      "passage_successor_reject_unknown_space=%zu "
      "passage_successor_reject_envelope=%zu "
      "passage_successor_reject_invalid=%zu "
      "passage_successor_reject_collision=%zu passage_successor_reject_risk=%zu "
      "passage_successor_reject_cost=%zu "
      "successor_search_batches=%zu successor_search_candidates=%zu "
      "successor_search_batch_max=%zu successor_search_worker_ms=%.3f "
      "successor_continuation_batches=%zu "
      "successor_continuation_candidates=%zu "
      "successor_continuation_batch_max=%zu "
      "successor_continuation_worker_ms=%.3f "
      "objective=%.3f route_length_m=%.2f travel_time_s=%.2f "
      "vertical_alignment_time_s=%.2f planning_exposure_m=%.2f "
      "critical_exposure_m=%.2f selected_passage_traversals=%zu search_ms=%.2f "
      "route_planning_ms=%.2f "
      "topology_searches=%zu parallel_topology_searches=%zu "
      "topology_search_worker_ms=%.2f "
      "continuation_ms=%.2f validation_ms=%.2f smoothing_ms=%.2f "
      "shortcut_validation_ms=%.2f corner_validation_ms=%.2f "
      "shortcut_candidates=%zu parallel_shortcut_candidates=%zu "
      "corner_candidates=%zu parallel_corner_candidates=%zu "
      "shortcuts=%zu smoothed_corners=%zu route_fingerprint=%" PRIu64,
      prepared.revision, certified_pending ? "true" : "false",
      static_cast<int>(staticRouteActivationStatusName(activation_status).size()),
      staticRouteActivationStatusName(activation_status).data(),
      static_cast<int>(routePublicationStatus3DName(publication_status).size()),
      routePublicationStatus3DName(publication_status).data(),
      world_compatible ? "true" : "false",
      activation_snapshot_current ? "true" : "false", activation.snapshot_pose_revision,
      activation.snapshot_raw_revision, generation_matches ? "true" : "false",
      objective_matches ? "true" : "false", prepared.global_guide_generation,
      route_space, observed_world_rebased ? "true" : "false",
      publication_world_advanced ? "true" : "false",
      lattice3DRoutePurposeName(lattice.route_purpose), topology_acceleration,
      observation_frontier ? observation_frontier->id.value : 0U,
      observation_frontier ? observation_frontier->supporting_map_revision : 0U,
      observation_frontier ? observation_frontier->supporting_rays : 0U,
      observation_frontier ? observation_frontier->information_gain_voxels : 0U,
      lattice.frontier_selection_score,
      static_cast<int>(
          observationRouteReplacementStatusName(observation_replacement.status).size()),
      observationRouteReplacementStatusName(observation_replacement.status).data(),
      observation_replacement.score_improvement,
      world.static_route_extension_request ? "true" : "false",
      world.static_route_replan_request ? "true" : "false",
      world.static_route_replan_request ? world.static_route_replan_base_generation
                                        : world.static_route_extension_base_generation,
      globalGuideReleaseReasonName(world.static_route_replan_reason),
      static_cast<int>(staticRouteReplacementPolicyName(replacement_policy).size()),
      staticRouteReplacementPolicyName(replacement_policy).data(),
      static_cast<int>(staticRouteCandidateStatusName(validation.status).size()),
      staticRouteCandidateStatusName(validation.status).data(),
      validation.endpoint_improvement_m, validation.failure_segment_index,
      validation.failure_point.x, validation.failure_point.y,
      validation.failure_point.z,
      activation.assessment.raw_validation.connector_validated ? "true" : "false",
      activation.assessment.raw_validation.suffix_validated ? "true" : "false",
      activation.assessment.raw_validation.validated_from_station_m,
      mppi::staticRouteHandoffStatusName(handoff.status), handoff.cross_track_m,
      handoff.minimum_clearance_m, handoff.planning_exposure_m,
      handoff.critical_exposure_m,
      static_cast<int>(
          routeSpliceCertificationStatus3DName(activation.splice.status).size()),
      routeSpliceCertificationStatus3DName(activation.splice.status).data(),
      activation.splice.splice.has_value()
          ? activation.splice.splice->required_overlap_m
          : 0.0,
      activation.splice.measured_maximum_position_separation_m,
      activation.splice.measured_minimum_tangent_alignment,
      lattice3DStatusName(lattice.status),
      lattice3DSearchTerminationName(lattice.termination), lattice.points.size(),
      lattice.route.size(),
      prepared.constrained_spans ? prepared.constrained_spans->size() : 0U,
      lattice.expansions, lattice_3d_config_.maximum_expansions,
      lattice_3d_config_.maximum_search_time_ms,
      lattice3DRiskStageName(lattice.risk_stage), search_start.x, search_start.y,
      search_start.z, lattice.planning_goal.x, lattice.planning_goal.y,
      lattice.planning_goal.z, prepared.planning_candidate_endpoint.x,
      prepared.planning_candidate_endpoint.y, prepared.planning_candidate_endpoint.z,
      preferred_direction.x, preferred_direction.y, preferred_direction.z,
      lattice.achieved_progress_m, lattice.minimum_clearance_m,
      lattice.stale_queue_pops, lattice.open_peak, lattice.records_peak,
      lattice.terminal_successor_count, lattice.continuation_reachable_states,
      lattice.continuation_reachable_depth_m,
      lattice.successor_diagnostics.lattice_generated,
      lattice.successor_diagnostics.lattice_accepted,
      lattice.successor_diagnostics.lattice_rejected_edge,
      lattice.successor_diagnostics.lattice_rejected_zero_length,
      lattice.successor_diagnostics.lattice_rejected_outside_grid,
      lattice.successor_diagnostics.lattice_rejected_unknown_space,
      lattice.successor_diagnostics.lattice_rejected_flight_envelope,
      lattice.successor_diagnostics.lattice_rejected_invalid_esdf,
      lattice.successor_diagnostics.lattice_rejected_raw_collision,
      lattice.successor_diagnostics.lattice_rejected_risk_stage,
      lattice.successor_diagnostics.lattice_rejected_no_cost_improvement,
      lattice.successor_diagnostics.passage_generated,
      lattice.successor_diagnostics.passage_accepted,
      lattice.successor_diagnostics.passage_rejected,
      lattice.successor_diagnostics.passage_rejected_connection_distance,
      lattice.successor_diagnostics.passage_rejected_outside_grid,
      lattice.successor_diagnostics.passage_rejected_unknown_space,
      lattice.successor_diagnostics.passage_rejected_flight_envelope,
      lattice.successor_diagnostics.passage_rejected_invalid_esdf,
      lattice.successor_diagnostics.passage_rejected_raw_collision,
      lattice.successor_diagnostics.passage_rejected_risk_stage,
      lattice.successor_diagnostics.passage_rejected_no_cost_improvement,
      lattice.successor_profiling.search.collection_calls,
      lattice.successor_profiling.search.candidates,
      lattice.successor_profiling.search.maximum_candidates,
      lattice.successor_profiling.search.worker_ms,
      lattice.successor_profiling.continuation.collection_calls,
      lattice.successor_profiling.continuation.candidates,
      lattice.successor_profiling.continuation.maximum_candidates,
      lattice.successor_profiling.continuation.worker_ms, lattice.objective_cost,
      lattice.route_length_m, lattice.estimated_travel_time_s,
      lattice.vertical_alignment_time_s, lattice.planning_exposure_m,
      lattice.critical_exposure_m, route_traversals.size(), search_ms,
      route_planning_ms, lattice.topology_searches, lattice.parallel_topology_searches,
      lattice.topology_search_worker_ms, prepared.continuation_validation_ms,
      prepared.candidate_validation_ms, prepared.route_smoothing_ms,
      prepared.route_shortcut_validation_ms, prepared.route_corner_validation_ms,
      prepared.route_shortcut_candidates, prepared.route_parallel_shortcut_candidates,
      prepared.route_corner_candidates, prepared.route_parallel_corner_candidates,
      prepared.route_shortcuts_applied, prepared.route_corners_smoothed,
      prepared.route_fingerprint);
  for (const Lattice3DTopologyCandidate& candidate : lattice.topology_candidates) {
    RCLCPP_INFO(get_logger(),
                "PRODUCTION_MPPI_TOPOLOGY_CANDIDATE revision=%" PRIu64
                " topology=%s risk_stage=%s status=%s termination=%s "
                "achieved_progress_m=%.2f minimum_clearance_m=%.2f "
                "expansions=%zu stale_pops=%zu open_peak=%zu records_peak=%zu "
                "terminal_successors=%zu continuation_states=%zu "
                "continuation_depth_m=%.2f objective=%.3f "
                "route_length_m=%.2f travel_time_s=%.2f "
                "vertical_alignment_time_s=%.2f planning_exposure_m=%.2f "
                "critical_exposure_m=%.2f turn_cost=%.3f rank=%zu selected=%s "
                "reason=%s",
                prepared.revision, candidate.topology.c_str(),
                lattice3DRiskStageName(candidate.risk_stage),
                lattice3DStatusName(candidate.status),
                lattice3DSearchTerminationName(candidate.termination),
                candidate.achieved_progress_m, candidate.minimum_clearance_m,
                candidate.expansions, candidate.stale_queue_pops, candidate.open_peak,
                candidate.records_peak, candidate.terminal_successor_count,
                candidate.continuation_reachable_states,
                candidate.continuation_reachable_depth_m, candidate.objective_cost,
                candidate.route_length_m, candidate.estimated_travel_time_s,
                candidate.vertical_alignment_time_s, candidate.planning_exposure_m,
                candidate.critical_exposure_m, candidate.turn_cost,
                candidate.candidate_rank, candidate.selected ? "true" : "false",
                candidate.decision_reason.c_str());
  }
  const bool initial_route_search = !world.static_route_extension_request &&
                                    !world.static_route_replan_request &&
                                    world.global_guide_generation == 0U;
  const bool same_observation_frontier_retained =
      lattice.route_purpose == Lattice3DRoutePurpose::kObservationFrontier &&
      observation_replacement.status ==
          ObservationRouteReplacementStatus::kSameFrontierRetained;
  if (world.static_route_replan_request || initial_route_search) {
    const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
    if (certified_pending) {
      static_route_failed_search_latch_.clear();
    } else if (initial_route_search ||
               (world_compatible && generation_matches && objective_matches &&
                !same_observation_frontier_retained)) {
      const std::uint64_t failed_generation =
          world.static_route_replan_request ? world.static_route_replan_base_generation
                                            : 0U;
      static_route_failed_search_latch_.recordFailure(StaticRouteSearchContext{
          .base_route_generation = failed_generation,
          .search_start = search_start,
          .objective = world.search_objective,
          .minimum_tracking_sample_sequence = required_objective_sample,
          .stamp_ns = get_clock()->now().nanoseconds(),
      });
      RCLCPP_INFO(
          get_logger(),
          "STATIC_ROUTE_SEARCH_OUTCOME status=failed_latched "
          "generation=%" PRIu64 " initial=%s activation_status=%.*s "
          "candidate_status=%.*s start=(%.2f,%.2f,%.2f)",
          failed_generation, initial_route_search ? "true" : "false",
          static_cast<int>(staticRouteActivationStatusName(activation_status).size()),
          staticRouteActivationStatusName(activation_status).data(),
          static_cast<int>(staticRouteCandidateStatusName(validation.status).size()),
          staticRouteCandidateStatusName(validation.status).data(), search_start.x,
          search_start.y, search_start.z);
    }
  } else if (certified_pending) {
    const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
    static_route_failed_search_latch_.clear();
  }
  finishStaticRouteSearch(world, certified_pending);
  const std::shared_ptr<const ProductionNavigationObjective> current_objective =
      navigationObjective();
  if (current_objective && current_objective->continuous_tracking) {
    const std::uint64_t required_epoch =
        minimum_tracking_route_mission_epoch_.load(std::memory_order_acquire);
    const std::uint64_t required_sample =
        current_objective->mission_epoch == required_epoch
            ? minimum_tracking_route_sample_sequence_.load(std::memory_order_acquire)
            : 0U;
    StaticRouteObjective resident_route_objective;
    {
      const std::scoped_lock lock{world_generation_publication_mutex_,
                                  esdf_state_mutex_};
      if (prepared_esdf_ && productionWorldGenerationCoherent(*prepared_esdf_)) {
        resident_route_objective = prepared_esdf_->route_objective;
      }
    }
    if (!staticRouteObjectiveMatches(
            resident_route_objective, makeStaticRouteObjective(*current_objective),
            required_sample, std::numeric_limits<double>::infinity())) {
      RCLCPP_INFO(get_logger(),
                  "STATIC_ROUTE_SHADOW status=followup_required "
                  "required_epoch=%" PRIu64 " required_sample=%" PRIu64
                  " resident_epoch=%" PRIu64 " resident_sample=%" PRIu64,
                  required_epoch, required_sample,
                  resident_route_objective.mission_epoch,
                  resident_route_objective.sample_sequence);
      requestGuideRelease(GlobalGuideReleaseReason::kObjectiveChanged);
    }
  }
}

} // namespace drone_city_nav
