#include <chrono>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "production_mppi_node.hpp"
#include "production_mppi_route_helpers.hpp"

namespace drone_city_nav {

void ProductionMppiNode::processStaticGuideSearch(
    const ProductionMppiPreparedEsdf& world,
    const ProductionMppiNavigation& navigation) {
  const Point3 mission_goal =
      world.search_objective.available ? world.search_objective.goal : mission_goal_;
  const Point3 search_start{navigation.state.x, navigation.state.y, navigation.state.z};
  Vec3 preferred_direction{static_cast<double>(navigation.state.vx),
                           static_cast<double>(navigation.state.vy),
                           static_cast<double>(navigation.state.vz)};
  if (std::hypot(preferred_direction.x, preferred_direction.y) < 0.5) {
    preferred_direction =
        Vec3{mission_goal.x - navigation.state.x, mission_goal.y - navigation.state.y,
             mission_goal.z - navigation.state.z};
  }
  const auto search_started = std::chrono::steady_clock::now();
  const std::span<const ConstrainedFreeSpaceEdge> channel_edges =
      world.channel_edges
          ? std::span<const ConstrainedFreeSpaceEdge>{*world.channel_edges}
          : std::span<const ConstrainedFreeSpaceEdge>{};
  const RiskAwareLattice3DResult lattice = planRiskAwareLattice3D(
      world.grid, *world.distances_m, search_start, preferred_direction, mission_goal,
      channel_edges, lattice_3d_config_, planning_worker_pool_.get());
  const double search_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - search_started)
                               .count();
  ProductionMppiPreparedEsdf prepared = world;
  prepared.global_guide_search_ms = search_ms;
  prepared.planning_search_kind = ProductionPlanningSearchKind::kLattice3D;
  prepared.planning_search_start = search_start;
  prepared.planning_search_goal = lattice.planning_goal;
  prepared.planning_candidate_endpoint =
      lattice.points.empty() ? search_start : lattice.points.back();
  prepared.planning_search_direction = preferred_direction;
  prepared.planning_candidate_points = lattice.points.size();
  prepared.planning_candidate_samples = lattice.route.size();
  prepared.lattice_search_performed = true;
  prepared.lattice_executable =
      lattice.status == Lattice3DStatus::kReachedPlanningGoal ||
      lattice.status == Lattice3DStatus::kViableFrontier;
  prepared.global_guide_expansions = lattice.expansions;
  prepared.lattice_3d_status = lattice.status;
  prepared.lattice_3d_risk_stage = lattice.risk_stage;
  prepared.lattice_3d_termination = lattice.termination;
  prepared.lattice_3d_minimum_clearance_m = lattice.minimum_clearance_m;
  prepared.lattice_3d_successor_diagnostics = lattice.successor_diagnostics;
  prepared.lattice_3d_successor_profiling = lattice.successor_profiling;
  prepared.lattice_search_session_complete =
      lattice.status != Lattice3DStatus::kSearchIncomplete;
  prepared.lattice_search_revision = world.revision;
  prepared.lattice_validation_revision = world.revision;
  prepared.lattice_planning_goal_reached =
      lattice.status == Lattice3DStatus::kReachedPlanningGoal;
  prepared.lattice_achieved_progress_m = lattice.achieved_progress_m;
  prepared.lattice_guide_length_m = lattice.route_length_m;
  prepared.lattice_remaining_goal_distance_m =
      distance3D(prepared.planning_candidate_endpoint, lattice.planning_goal);
  prepared.lattice_terminal_successor_count = lattice.terminal_successor_count;
  prepared.lattice_stale_queue_pops = lattice.stale_queue_pops;
  prepared.lattice_open_peak = lattice.open_peak;
  prepared.lattice_records_peak = lattice.records_peak;
  prepared.lattice_continuation_reachable_states =
      lattice.continuation_reachable_states;
  prepared.lattice_reachable_depth_m = lattice.continuation_reachable_depth_m;
  prepared.global_guide_cost = lattice.objective_cost;
  prepared.global_guide_reaches_mission_goal = lattice.reached_mission_goal;
  prepared.topology_candidates = lattice.topology_candidates;
  prepared.topology_objective_cost = lattice.objective_cost;
  prepared.topology_route_length_m = lattice.route_length_m;
  prepared.topology_travel_time_s = lattice.estimated_travel_time_s;
  prepared.topology_vertical_alignment_time_s = lattice.vertical_alignment_time_s;
  prepared.topology_planning_exposure_m = lattice.planning_exposure_m;
  prepared.topology_critical_exposure_m = lattice.critical_exposure_m;
  prepared.continuation_validation_ms = lattice.continuation_validation_ms;
  prepared.route_fingerprint = lattice.route_fingerprint;

  StaticRouteCandidateValidation validation{.status =
                                                StaticRouteCandidateStatus::kEmpty};
  if (prepared.lattice_executable) {
    const auto validation_started = std::chrono::steady_clock::now();
    auto mutable_route = std::make_shared<std::vector<RouteSample3D>>(lattice.route);
    const std::uint64_t candidate_generation = static_route_generation_ + 1U;
    std::vector<ConstrainedRouteSpan> initial_spans =
        makeConstrainedRouteSpans(*mutable_route, lattice.selected_channels,
                                  candidate_generation, route_envelope_config_);
    const auto smoothing_started = std::chrono::steady_clock::now();
    StaticRouteGeometryResult geometry = optimizeStaticRouteGeometry(
        *mutable_route, initial_spans, world.grid, *world.distances_m,
        SweptFootprintConfig{
            .radius_m = lattice_3d_config_.physical_footprint_radius_m,
            .lower_extent_m = lattice_3d_config_.physical_footprint_lower_extent_m,
            .upper_extent_m = lattice_3d_config_.physical_footprint_upper_extent_m,
            .perimeter_samples = safety_config_.physical_footprint_samples,
            .radial_rings = safety_config_.physical_footprint_radial_rings,
            .axial_samples = safety_config_.physical_footprint_axial_samples,
            .sweep_step_m = safety_config_.swept_validation_step_m},
        static_route_geometry_config_, route_envelope_config_,
        planning_worker_pool_.get());
    prepared.route_smoothing_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  smoothing_started)
            .count();
    prepared.route_shortcuts_applied = geometry.shortcuts_applied;
    prepared.route_corners_smoothed = geometry.corners_smoothed;
    prepared.route_shortcut_candidates = geometry.shortcut_candidates;
    prepared.route_parallel_shortcut_candidates = geometry.parallel_shortcut_candidates;
    prepared.route_corner_candidates = geometry.corner_candidates;
    prepared.route_parallel_corner_candidates = geometry.parallel_corner_candidates;
    prepared.route_shortcut_validation_ms = geometry.shortcut_validation_ms;
    prepared.route_corner_validation_ms = geometry.corner_validation_ms;
    if (geometry.route.size() >= 2U) {
      *mutable_route = std::move(geometry.route);
    }
    for (ConstrainedRouteSpan& span : geometry.constrained_spans) {
      span.route_generation = candidate_generation;
    }
    std::vector<CooperativeChannelLaneAssignment> channel_assignments;
    bool cooperative_route_valid = true;
    if (cooperative_traffic_enabled_ && static_occupancy_3d_) {
      const std::span<const ChannelLaneSet> lane_sets =
          world.channel_lane_sets
              ? std::span<const ChannelLaneSet>{*world.channel_lane_sets}
              : std::span<const ChannelLaneSet>{};
      CooperativeChannelRouteResult cooperative_route = applyCooperativeChannelLanes(
          *mutable_route, geometry.constrained_spans, lane_sets, *static_occupancy_3d_,
          cooperative_channel_route_config_);
      cooperative_route_valid = cooperative_route.valid;
      if (cooperative_route.valid) {
        *mutable_route = std::move(cooperative_route.route);
        geometry.constrained_spans = std::move(cooperative_route.constrained_spans);
      }
      channel_assignments = std::move(cooperative_route.assignments);
    }
    auto mutable_spans = std::make_shared<std::vector<ConstrainedRouteSpan>>(
        std::move(geometry.constrained_spans));
    if (!cooperative_route_valid) {
      validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kInvalidChannelSpan};
    } else if (assignRouteRiskTiers(*mutable_route, world.grid, *world.distances_m,
                                    mppi_config_.risk.critical_distance_m,
                                    mppi_config_.risk.preferred_distance_m)) {
      validation = validateStaticRouteCandidate(
          world.route_3d ? std::span<const RouteSample3D>{*world.route_3d}
                         : std::span<const RouteSample3D>{},
          *mutable_route, world.grid, *world.distances_m, mission_goal,
          static_route_extension_config_.minimum_endpoint_improvement_m,
          lattice.reached_mission_goal, lattice_3d_config_.flight_envelope,
          world.static_route_extension_request || world.static_route_replan_request,
          SweptFootprintConfig{
              .radius_m = lattice_3d_config_.physical_footprint_radius_m,
              .lower_extent_m = lattice_3d_config_.physical_footprint_lower_extent_m,
              .upper_extent_m = lattice_3d_config_.physical_footprint_upper_extent_m,
              .perimeter_samples = safety_config_.physical_footprint_samples,
              .radial_rings = safety_config_.physical_footprint_radial_rings,
              .axial_samples = safety_config_.physical_footprint_axial_samples,
              .sweep_step_m = safety_config_.swept_validation_step_m});
    } else {
      validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kInvalidEsdf};
    }
    const std::shared_ptr<const std::vector<RouteSample3D>> route = mutable_route;
    const std::shared_ptr<const std::vector<ConstrainedRouteSpan>> spans =
        mutable_spans;
    if (validation.accepted && spans->size() != initial_spans.size()) {
      validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kInvalidChannelSpan};
    }
    if (validation.accepted && !validateConstrainedRouteSpans(
                                   *route, *spans, world.grid, *world.distances_m)) {
      validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kInvalidChannelSpan};
    }
    const bool protected_suffix =
        (world.static_route_extension_request || world.static_route_replan_request) &&
        world.route_3d && world.constrained_spans &&
        staticRouteHasProtectedConstrainedSuffix(
            *world.route_3d, *world.constrained_spans, search_start,
            static_route_extension_config_.protected_departure_m);
    if (validation.accepted && protected_suffix) {
      validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kProtectedConstrainedSuffix};
    }
    std::vector<std::string> selected_channel_ids;
    selected_channel_ids.reserve(lattice.selected_channels.size());
    for (const SelectedChannelTraversal& traversal : lattice.selected_channels) {
      selected_channel_ids.push_back(traversal.channel_id);
    }
    prepared.route_3d = route;
    prepared.route_2d_projection = projectRouteTo2D(*route);
    prepared.constrained_spans = spans;
    prepared.cooperative_channel_assignments =
        std::make_shared<const std::vector<CooperativeChannelLaneAssignment>>(
            std::move(channel_assignments));
    prepared.selected_channel_ids = std::make_shared<const std::vector<std::string>>(
        std::move(selected_channel_ids));
    prepared.mppi_route =
        makeMppiRoute3D(*route, *spans, speed_policy_config_.cruise_speed_mps,
                        constrained_route_speed_limit_mps_);
    prepared.global_guide_projection = projectOntoGlobalGuide(
        *prepared.route_2d_projection, Point2{navigation.state.x, navigation.state.y});
    prepared.route_fingerprint = routeFingerprint(*route, lattice.selected_channels);
    prepared.candidate_validation_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  validation_started)
            .count();
  }

  prepared.static_route_candidate_status = validation.status;
  StaticRouteActivationStatus activation_status =
      prepared.lattice_executable
          ? StaticRouteActivationStatus::kCandidateValidationRejected
          : StaticRouteActivationStatus::kCandidateNotExecutable;
  bool revision_matches = false;
  bool generation_matches = false;
  const std::shared_ptr<const ProductionNavigationObjective> activation_objective =
      navigationObjective();
  const std::uint64_t required_objective_epoch =
      minimum_tracking_route_mission_epoch_.load(std::memory_order_acquire);
  const std::uint64_t required_objective_sample =
      activation_objective &&
              activation_objective->mission_epoch == required_objective_epoch
          ? minimum_tracking_route_sample_sequence_.load(std::memory_order_acquire)
          : 0U;
  const bool objective_matches =
      activation_objective &&
      staticRouteObjectiveMatches(
          world.search_objective, makeStaticRouteObjective(*activation_objective),
          required_objective_sample, std::numeric_limits<double>::infinity());
  const StaticRouteCandidate route_candidate{
      .search_revision = prepared.revision,
      .base_route_generation = world.static_route_replan_request
                                   ? world.static_route_replan_base_generation
                                   : world.static_route_extension_base_generation,
      .candidate_route_generation = static_route_generation_ + 1U,
      .fingerprint = prepared.route_fingerprint,
      .executable = prepared.lattice_executable,
      .reaches_mission_goal = prepared.global_guide_reaches_mission_goal,
      .validation = validation,
      .route = prepared.route_3d,
      .constrained_spans = prepared.constrained_spans,
  };

  bool activated = false;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    const std::uint64_t required_base_generation =
        world.static_route_replan_request
            ? world.static_route_replan_base_generation
            : world.static_route_extension_base_generation;
    generation_matches =
        (!world.static_route_extension_request && !world.static_route_replan_request) ||
        (prepared_esdf_ &&
         prepared_esdf_->global_guide_generation == required_base_generation);
    revision_matches = prepared_esdf_ && prepared_esdf_->revision == prepared.revision;
    if (route_candidate.executable && route_candidate.validation.accepted &&
        route_candidate.route && route_candidate.constrained_spans &&
        revision_matches && generation_matches && objective_matches) {
      activation_status = StaticRouteActivationStatus::kActivated;
      prepared.static_route_activation_status = activation_status;
      prepared.static_route_revision_matches = true;
      prepared.static_route_generation_matches = true;
      prepared.global_guide_generation = ++static_route_generation_;
      prepared.route_objective = world.search_objective;
      prepared.global_guide_release_reason = GlobalGuideReleaseReason::kNone;
      prepared.static_route_extension_request = false;
      prepared.static_route_extension_base_generation = 0U;
      prepared.static_route_replan_request = false;
      prepared.static_route_replan_base_generation = 0U;
      prepared.static_route_replan_reason = GlobalGuideReleaseReason::kNone;
      prepared_esdf_ = prepared;
      activated = true;
    } else if (route_candidate.validation.accepted && !revision_matches) {
      activation_status = StaticRouteActivationStatus::kStaleWorldRevision;
    } else if (route_candidate.validation.accepted && !generation_matches) {
      activation_status = StaticRouteActivationStatus::kStaleRouteGeneration;
    } else if (route_candidate.validation.accepted && !objective_matches) {
      activation_status = StaticRouteActivationStatus::kStaleObjective;
    }
  }
  if (activated && prepared.cooperative_channel_assignments) {
    for (const CooperativeChannelLaneAssignment& assignment :
         *prepared.cooperative_channel_assignments) {
      RCLCPP_INFO(get_logger(),
                  "COOPERATIVE_CHANNEL_ROUTE route_generation=%" PRIu64
                  " span_index=%zu channel='%s' lane=%zu/%zu offset_m=%.2f status=%s",
                  assignment.route_generation, assignment.span_index,
                  assignment.channel_id.c_str(), assignment.lane_index,
                  assignment.lane_count, assignment.lateral_offset_m,
                  cooperativeChannelLaneRouteStatusName(assignment.status));
    }
  }
  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_GUIDE3D revision=%" PRIu64
      " activated=%s activation_status=%.*s revision_matches=%s "
      "generation_matches=%s objective_matches=%s extension=%s replan=%s "
      "base_generation=%" PRIu64 " replan_reason=%s"
      " validation=%.*s endpoint_improvement_m=%.2f status=%s termination=%s "
      "points=%zu samples=%zu spans=%zu expansions=%zu expansion_limit=%zu "
      "deadline_ms=%.2f "
      "risk_stage=%s start=(%.2f,%.2f,%.2f) planning_goal=(%.2f,%.2f,%.2f) "
      "endpoint=(%.2f,%.2f,%.2f) direction=(%.3f,%.3f,%.3f) "
      "achieved_progress_m=%.2f minimum_clearance_m=%.2f stale_pops=%zu "
      "open_peak=%zu records_peak=%zu terminal_successors=%zu "
      "continuation_states=%zu continuation_depth_m=%.2f "
      "lattice_successor_generated=%zu lattice_successor_accepted=%zu "
      "lattice_successor_reject_edge=%zu lattice_successor_reject_zero=%zu "
      "lattice_successor_reject_grid=%zu lattice_successor_reject_envelope=%zu "
      "lattice_successor_reject_invalid=%zu "
      "lattice_successor_reject_collision=%zu lattice_successor_reject_risk=%zu "
      "lattice_successor_reject_cost=%zu "
      "channel_successor_generated=%zu channel_successor_accepted=%zu "
      "channel_successor_rejected=%zu channel_successor_reject_connection=%zu "
      "channel_successor_reject_grid=%zu channel_successor_reject_envelope=%zu "
      "channel_successor_reject_invalid=%zu "
      "channel_successor_reject_collision=%zu channel_successor_reject_risk=%zu "
      "channel_successor_reject_cost=%zu "
      "successor_search_batches=%zu successor_search_candidates=%zu "
      "successor_search_batch_max=%zu successor_search_worker_ms=%.3f "
      "successor_continuation_batches=%zu "
      "successor_continuation_candidates=%zu "
      "successor_continuation_batch_max=%zu "
      "successor_continuation_worker_ms=%.3f "
      "objective=%.3f route_length_m=%.2f travel_time_s=%.2f "
      "vertical_alignment_time_s=%.2f planning_exposure_m=%.2f "
      "critical_exposure_m=%.2f selected_channels=%zu search_ms=%.2f "
      "topology_searches=%zu parallel_topology_searches=%zu "
      "topology_search_worker_ms=%.2f "
      "continuation_ms=%.2f validation_ms=%.2f smoothing_ms=%.2f "
      "shortcut_validation_ms=%.2f corner_validation_ms=%.2f "
      "shortcut_candidates=%zu parallel_shortcut_candidates=%zu "
      "corner_candidates=%zu parallel_corner_candidates=%zu "
      "shortcuts=%zu smoothed_corners=%zu route_fingerprint=%" PRIu64,
      prepared.revision, activated ? "true" : "false",
      static_cast<int>(staticRouteActivationStatusName(activation_status).size()),
      staticRouteActivationStatusName(activation_status).data(),
      revision_matches ? "true" : "false", generation_matches ? "true" : "false",
      objective_matches ? "true" : "false",
      world.static_route_extension_request ? "true" : "false",
      world.static_route_replan_request ? "true" : "false",
      world.static_route_replan_request ? world.static_route_replan_base_generation
                                        : world.static_route_extension_base_generation,
      globalGuideReleaseReasonName(world.static_route_replan_reason),
      static_cast<int>(staticRouteCandidateStatusName(validation.status).size()),
      staticRouteCandidateStatusName(validation.status).data(),
      validation.endpoint_improvement_m, lattice3DStatusName(lattice.status),
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
      lattice.successor_diagnostics.lattice_rejected_flight_envelope,
      lattice.successor_diagnostics.lattice_rejected_invalid_esdf,
      lattice.successor_diagnostics.lattice_rejected_raw_collision,
      lattice.successor_diagnostics.lattice_rejected_risk_stage,
      lattice.successor_diagnostics.lattice_rejected_no_cost_improvement,
      lattice.successor_diagnostics.channel_generated,
      lattice.successor_diagnostics.channel_accepted,
      lattice.successor_diagnostics.channel_rejected,
      lattice.successor_diagnostics.channel_rejected_connection_distance,
      lattice.successor_diagnostics.channel_rejected_outside_grid,
      lattice.successor_diagnostics.channel_rejected_flight_envelope,
      lattice.successor_diagnostics.channel_rejected_invalid_esdf,
      lattice.successor_diagnostics.channel_rejected_raw_collision,
      lattice.successor_diagnostics.channel_rejected_risk_stage,
      lattice.successor_diagnostics.channel_rejected_no_cost_improvement,
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
      lattice.critical_exposure_m, lattice.selected_channels.size(), search_ms,
      lattice.topology_searches, lattice.parallel_topology_searches,
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
  if (world.static_route_extension_request) {
    finishStaticRouteExtension(world.static_route_extension_base_generation);
  }
  const bool initial_route_search = !world.static_route_extension_request &&
                                    !world.static_route_replan_request &&
                                    world.global_guide_generation == 0U;
  if (world.static_route_replan_request || initial_route_search) {
    const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
    if (activated) {
      static_route_failed_search_latch_.clear();
    } else if (initial_route_search ||
               (revision_matches && generation_matches && objective_matches)) {
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
    if (world.static_route_replan_request) {
      static_route_replan_gate_.finish(world.static_route_replan_base_generation);
    }
  } else if (activated) {
    const std::scoped_lock lifecycle_lock{static_route_extension_mutex_};
    static_route_failed_search_latch_.clear();
  }
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
      const std::scoped_lock lock{esdf_state_mutex_};
      if (prepared_esdf_) {
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
