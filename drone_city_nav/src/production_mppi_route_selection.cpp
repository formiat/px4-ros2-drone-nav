#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool latticeExecutable(const RiskAwareLattice3DResult& lattice) noexcept {
  return lattice.status == Lattice3DStatus::kReachedPlanningGoal ||
         lattice.status == Lattice3DStatus::kViableFrontier;
}

[[nodiscard]] RouteIntentPurpose3D
intentPurpose(const Lattice3DRoutePurpose purpose) noexcept {
  switch (purpose) {
    case Lattice3DRoutePurpose::kMissionTransit:
      return RouteIntentPurpose3D::kMissionTransit;
    case Lattice3DRoutePurpose::kLaunchDeparture:
      return RouteIntentPurpose3D::kLaunchDeparture;
    case Lattice3DRoutePurpose::kObservationFrontier:
      return RouteIntentPurpose3D::kObservationFrontier;
    case Lattice3DRoutePurpose::kTopologicalBacktrack:
      return RouteIntentPurpose3D::kTopologicalBacktrack;
  }
  return RouteIntentPurpose3D::kMissionTransit;
}

[[nodiscard]] Point3
intentTarget(const ProductionIncrementalTopologySearch3D& topology,
             const Lattice3DStrategicDirective& directive) noexcept {
  if (!topology.plan.guidance_points.empty()) {
    return topology.plan.guidance_points.back();
  }
  return directive.planning_goal;
}

[[nodiscard]] std::uint64_t
intentTargetIdentity(const ProductionIncrementalTopologySearch3D& topology) noexcept {
  return topology.plan.selected_frontier ? topology.plan.selected_frontier->id.value
                                         : topology.plan.target_node.value;
}

[[nodiscard]] RouteStrategyLeaseReason3D
strategyLeaseReason(const IncrementalTopologicalPlan3D& plan) noexcept {
  switch (plan.purpose) {
    case IncrementalTopologicalRoutePurpose3D::kMissionTransit:
      return RouteStrategyLeaseReason3D::kMissionTopologyContinuation;
    case IncrementalTopologicalRoutePurpose3D::kObservationFrontier:
      return RouteStrategyLeaseReason3D::kObservationFrontier;
    case IncrementalTopologicalRoutePurpose3D::kTopologicalBacktrack:
      switch (plan.backtrack_reason) {
        case TopologicalBacktrackReason3D::kConfirmedTerminal:
          return RouteStrategyLeaseReason3D::kBacktrackConfirmedTerminal;
        case TopologicalBacktrackReason3D::kNoReachableFrontier:
          return RouteStrategyLeaseReason3D::kBacktrackNoReachableFrontier;
        case TopologicalBacktrackReason3D::kAllReachableBranchesExplored:
          return RouteStrategyLeaseReason3D::kBacktrackAllReachableBranchesExplored;
        case TopologicalBacktrackReason3D::kNone:
          return RouteStrategyLeaseReason3D::kNone;
      }
      return RouteStrategyLeaseReason3D::kNone;
  }
  return RouteStrategyLeaseReason3D::kNone;
}

[[nodiscard]] SegmentEvidenceWorld3D
evidenceWorld(const ProductionMppiPreparedEsdf& world,
              const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world,
              const RiskAwareLattice3DConfig& lattice_config,
              const SweptFootprintConfig& footprint) noexcept {
  return SegmentEvidenceWorld3D{
      .grid = &world.grid,
      .esdf_m = world.distances_m ? std::span<const float>{*world.distances_m}
                                  : std::span<const float>{},
      .latest_observed_occupancy = latest_raw_world && latest_raw_world->occupancy
                                       ? latest_raw_world->occupancy.get()
                                       : nullptr,
      .proprioceptive_free_space_seed =
          world.proprioceptive_free_space_seed
              ? std::addressof(*world.proprioceptive_free_space_seed)
              : nullptr,
      .launch_support_contact = world.launch_support_contact
                                    ? std::addressof(*world.launch_support_contact)
                                    : nullptr,
      .footprint = footprint,
      .flight_envelope = lattice_config.flight_envelope,
      .validated_through_revision = latest_raw_world
                                        ? latest_raw_world->version.revision
                                        : world.source_raw_revision,
      .require_known_free_space = lattice_config.require_known_free_space,
  };
}

} // namespace

ProductionRouteCandidateSet3D ProductionMppiNode::generateRouteCandidates3D(
    const ProductionMppiPreparedEsdf& world, const ProductionMppiNavigation& navigation,
    const Point3& mission_goal,
    const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world) {
  const auto search_started = std::chrono::steady_clock::now();
  const Point3 search_start{navigation.state.x, navigation.state.y, navigation.state.z};
  Vec3 preferred_direction{static_cast<double>(navigation.state.vx),
                           static_cast<double>(navigation.state.vy),
                           static_cast<double>(navigation.state.vz)};
  if (std::hypot(preferred_direction.x, preferred_direction.y) < 0.5) {
    preferred_direction =
        Vec3{mission_goal.x - navigation.state.x, mission_goal.y - navigation.state.y,
             mission_goal.z - navigation.state.z};
  }
  const SweptFootprintConfig footprint{
      .radius_m = lattice_3d_config_.physical_footprint_radius_m,
      .lower_extent_m = lattice_3d_config_.physical_footprint_lower_extent_m,
      .upper_extent_m = lattice_3d_config_.physical_footprint_upper_extent_m,
      .perimeter_samples = physical_footprint_config_.perimeter_samples,
      .radial_rings = physical_footprint_config_.radial_rings,
      .axial_samples = physical_footprint_config_.axial_samples,
      .sweep_step_m = physical_footprint_config_.sweep_step_m,
  };
  const SegmentEvidenceWorld3D evidence_world =
      evidenceWorld(world, latest_raw_world, lattice_3d_config_, footprint);
  std::vector<ProductionRouteSearchCandidate3D> candidates;
  candidates.reserve(2U);

  const auto plan_lattice =
      [&](const Lattice3DStrategicDirective& directive,
          const std::optional<std::chrono::steady_clock::time_point> deadline =
              std::nullopt) {
        RiskAwareLattice3DConfig search_config = lattice_3d_config_;
        if (directive.route_purpose == Lattice3DRoutePurpose::kLaunchDeparture) {
          search_config.goal_tolerance_m = std::min(
              search_config.goal_tolerance_m, 0.5 * search_config.vertical_step_m);
        }
        if (deadline.has_value()) {
          const double remaining_ms = std::chrono::duration<double, std::milli>(
                                          *deadline - std::chrono::steady_clock::now())
                                          .count();
          if (!(remaining_ms > 0.0)) {
            RCLCPP_INFO(get_logger(),
                        "ROUTE_PROPOSAL_SEARCH3D stage=skipped reason=control_deadline "
                        "revision=%" PRIu64 " purpose=%s",
                        world.revision,
                        lattice3DRoutePurposeName(directive.route_purpose));
            return std::optional<RiskAwareLattice3DResult>{};
          }
          search_config.maximum_search_time_ms =
              std::min(search_config.maximum_search_time_ms, remaining_ms);
        }
        RCLCPP_INFO(get_logger(),
                    "ROUTE_PROPOSAL_SEARCH3D stage=begin revision=%" PRIu64
                    " purpose=%s target=(%.2f,%.2f,%.2f)",
                    world.revision, lattice3DRoutePurposeName(directive.route_purpose),
                    directive.planning_goal.x, directive.planning_goal.y,
                    directive.planning_goal.z);
        RiskAwareLattice3DResult result = planRiskAwareLattice3D(
            world.grid, *world.distances_m, search_start, directive.preferred_direction,
            mission_goal, std::span<const PassageTraversalEdge>{}, search_config,
            planning_worker_pool_.get(), &directive);
        RCLCPP_INFO(get_logger(),
                    "ROUTE_PROPOSAL_SEARCH3D stage=complete revision=%" PRIu64
                    " purpose=%s status=%s points=%zu",
                    world.revision, lattice3DRoutePurposeName(directive.route_purpose),
                    lattice3DStatusName(result.status), result.points.size());
        return std::optional<RiskAwareLattice3DResult>{std::move(result)};
      };
  const auto add_candidate =
      [&](RouteIntent3D intent, const Lattice3DStrategicDirective& directive,
          RiskAwareLattice3DResult lattice,
          std::optional<ProductionIncrementalTopologySearch3D> topology =
              std::nullopt) {
        const bool reaches_segment =
            lattice.status == Lattice3DStatus::kReachedPlanningGoal;
        SegmentEvidence3D evidence = evaluateSegmentEvidence3D(
            intent, lattice.route, search_start, latticeExecutable(lattice),
            reaches_segment, lattice.reached_mission_goal, lattice.objective_cost,
            evidence_world);
        candidates.push_back(ProductionRouteSearchCandidate3D{
            .intent = intent,
            .evidence = evidence,
            .lattice = std::move(lattice),
            .topology = std::move(topology),
            .directive = directive,
        });
      };

  bool direct_mission_candidate{false};
  if (world.launch_support_resolution_pending) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "LAUNCH_SUPPORT_CONTACT state=route_pending");
  } else if (world.launch_support_contact && world.observed_occupancy) {
    const LaunchSupportDeparture3D departure = planLaunchSupportDeparture3D(
        *world.observed_occupancy, search_start, *world.launch_support_contact,
        lattice_3d_config_.vertical_step_m);
    RCLCPP_INFO(get_logger(),
                "LAUNCH_SUPPORT_DEPARTURE executable=%s validation=%s "
                "axial_departure_m=%.3f start=(%.3f,%.3f,%.3f) "
                "target=(%.3f,%.3f,%.3f)",
                departure.executable ? "true" : "false",
                sweptFootprintStatusName(departure.validation.status),
                departure.axial_departure_m, search_start.x, search_start.y,
                search_start.z, departure.target.x, departure.target.y,
                departure.target.z);
    if (departure.executable) {
      const FootprintBodyAxis& axis = world.launch_support_contact->seed.body_axis;
      const Lattice3DStrategicDirective directive{
          .planning_goal = departure.target,
          .preferred_direction = {axis.x, axis.y, axis.z},
          .route_purpose = Lattice3DRoutePurpose::kLaunchDeparture,
          .observation_frontier = std::nullopt,
          .selection_score = 0.0,
          .reaches_mission_goal = false,
      };
      RouteIntent3D intent{
          .planned_on_revision = world.revision,
          .mission_target = mission_goal,
          .intent_target = departure.target,
          .segment_target = departure.target,
          .source = RouteIntentSource3D::kLaunchDeparture,
          .purpose = RouteIntentPurpose3D::kLaunchDeparture,
          .segment_reaches_intent_target = true,
          .valid = true,
      };
      intent.id = makeRouteIntentId3D(intent.source, intent.purpose,
                                      intent.mission_target, intent.intent_target);
      if (auto lattice = plan_lattice(directive)) {
        add_candidate(intent, directive, std::move(*lattice));
      }
    }
  } else {
    direct_mission_candidate = true;
    const Point3 planning_goal = staticRoutePlanningGoal(
        search_start, mission_goal, lattice_3d_config_.planning_goal_distance_m);
    const Lattice3DStrategicDirective directive{
        .planning_goal = planning_goal,
        .preferred_direction = preferred_direction,
        .route_purpose = Lattice3DRoutePurpose::kMissionTransit,
        .observation_frontier = std::nullopt,
        .selection_score = 0.0,
        .reaches_mission_goal = distance3D(planning_goal, mission_goal) <= 1.0e-6,
    };
    RouteIntent3D intent{
        .planned_on_revision = world.revision,
        .mission_target = mission_goal,
        .intent_target = planning_goal,
        .segment_target = planning_goal,
        .source = RouteIntentSource3D::kDirect,
        .purpose = RouteIntentPurpose3D::kMissionTransit,
        .segment_reaches_intent_target = true,
        .intent_reaches_mission_target = directive.reaches_mission_goal,
        .valid = true,
    };
    intent.id = makeRouteIntentId3D(intent.source, intent.purpose,
                                    intent.mission_target, intent.intent_target);
    if (auto lattice = plan_lattice(directive)) {
      add_candidate(intent, directive, std::move(*lattice));
    }
  }

  const bool direct_candidate_executable =
      !candidates.empty() && candidates.front().evidence.physical_executable &&
      latticeExecutable(candidates.front().lattice);
  const double direct_selection_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                search_started)
          .count();
  const bool topology_budget_available =
      direct_selection_ms <= topological_strategy_budget_ms_;
  const auto topology_deadline =
      search_started +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double, std::milli>{topological_strategy_budget_ms_});
  if (direct_mission_candidate &&
      (!direct_candidate_executable || topology_budget_available)) {
    ProductionIncrementalTopologySearch3D topology = selectIncrementalTopologyRoute3D(
        world, search_start, mission_goal, topology_deadline);
    RCLCPP_INFO(get_logger(),
                "INCREMENTAL_TOPOLOGY3D_SEARCH graph_revision=%" PRIu64
                " strategic_plan_id=%" PRIu64
                " graph_nodes=%zu graph_edges=%zu status=%s purpose=%s "
                "start_node=%" PRIu64 " target_node=%" PRIu64 " goal_node=%" PRIu64
                " route_nodes=%zu route_edges=%zu reaches_mission_goal=%s "
                "continued_active_plan=%s "
                "reachable_mission_continuations=%zu "
                "maximum_mission_continuation_goal_progress_m=%.2f "
                "reachable_frontiers=%zu goal_progress_m=%.2f directive_available=%s "
                "no_executable_route_age_ms=%.2f",
                topology.plan.planned_on_revision, topology.plan.strategic_plan_id,
                topology.graph_node_count, topology.graph_edge_count,
                incrementalTopologicalPlanStatus3DName(topology.plan.status),
                incrementalTopologicalRoutePurpose3DName(topology.plan.purpose),
                topology.plan.start_node.value, topology.plan.target_node.value,
                topology.plan.goal_node.has_value() ? topology.plan.goal_node->value
                                                    : 0U,
                topology.plan.route_nodes.size(), topology.plan.route_steps.size(),
                topology.plan.reaches_mission_goal ? "true" : "false",
                topology.plan.continued_from_active_plan ? "true" : "false",
                topology.plan.reachable_mission_continuation_count,
                topology.plan.maximum_reachable_mission_continuation_goal_progress_m,
                topology.plan.reachable_frontier_count, topology.plan.goal_progress_m,
                topology.directive.has_value() ? "true" : "false",
                topology.no_executable_route_age_ms);
    if (topology.directive) {
      const Lattice3DStrategicDirective directive = topology.directive->lattice;
      const Point3 target = intentTarget(topology, directive);
      const std::uint64_t target_identity = intentTargetIdentity(topology);
      RouteIntent3D intent{
          .strategic_plan_id = topology.plan.strategic_plan_id,
          .planned_on_revision = world.revision,
          .source_graph_revision = topology.plan.planned_on_revision,
          .target_identity = target_identity,
          .return_lineage = makeRouteStrategyReturnLineage3D(
              topology.plan.strategic_plan_id, topology.plan.topology_lineage_id,
              topology.plan.planned_on_revision, topology.plan.start_node.value,
              target_identity, mission_goal),
          .mission_target = mission_goal,
          .intent_target = target,
          .segment_target = directive.planning_goal,
          .source = RouteIntentSource3D::kTopology,
          .purpose = intentPurpose(directive.route_purpose),
          .lease_reason = strategyLeaseReason(topology.plan),
          .graph_step_count = topology.plan.route_steps.size(),
          .strategic_continuation_available =
              topology.plan.guidance_points.size() >= 2U,
          .strategic_mission_continuation =
              topology.plan.purpose ==
                  IncrementalTopologicalRoutePurpose3D::kMissionTransit &&
              (topology.plan.status ==
                   IncrementalTopologicalPlanStatus3D::kMissionRoute ||
               topology.plan.status ==
                   IncrementalTopologicalPlanStatus3D::kMissionContinuationRoute),
          .segment_reaches_intent_target =
              topology.directive->reaches_topological_target,
          .intent_reaches_mission_target = topology.plan.reaches_mission_goal,
          .valid = true,
      };
      intent.id =
          makeRouteIntentId3D(intent.source, intent.purpose, intent.mission_target,
                              intent.intent_target, intent.target_identity);
      if (auto lattice = plan_lattice(directive, topology_deadline)) {
        add_candidate(intent, directive, std::move(*lattice), std::move(topology));
      } else {
        RCLCPP_INFO(get_logger(),
                    "INCREMENTAL_TOPOLOGY3D_SEARCH status=deferred_control_deadline "
                    "graph_revision=%" PRIu64 " strategic_plan_id=%" PRIu64,
                    topology.plan.planned_on_revision, topology.plan.strategic_plan_id);
      }
    }
  } else if (direct_mission_candidate) {
    RCLCPP_INFO(get_logger(),
                "INCREMENTAL_TOPOLOGY3D_SEARCH status=deferred_control_budget "
                "direct_selection_ms=%.2f budget_ms=%.2f direct_executable=%s",
                direct_selection_ms, topological_strategy_budget_ms_,
                direct_candidate_executable ? "true" : "false");
  }

  for (std::size_t index = 0U; index < candidates.size(); ++index) {
    const ProductionRouteSearchCandidate3D& candidate = candidates[index];
    RCLCPP_INFO(
        get_logger(),
        "ROUTE_PROPOSAL3D stage=generated revision=%" PRIu64 " index=%zu "
        "intent_id=%" PRIu64 " strategic_plan_id=%" PRIu64
        " source=%s purpose=%s planned_on=%" PRIu64 " validated_through=%" PRIu64
        " status=%s physical=%s "
        "segment_target=%s intent_target=%s mission_target=%s strategic=%s "
        "strategic_mission=%s "
        "unknown=%s known_clearance=%s minimum_known_clearance_m=%.3f "
        "route_length_m=%.2f endpoint_displacement_m=%.2f "
        "mission_progress_m=%.2f objective=%.3f",
        world.revision, index, candidate.intent.id, candidate.intent.strategic_plan_id,
        routeIntentSource3DName(candidate.intent.source),
        routeIntentPurpose3DName(candidate.intent.purpose),
        candidate.evidence.planned_on_revision,
        candidate.evidence.validated_through_revision,
        segmentEvidenceStatus3DName(candidate.evidence.status),
        candidate.evidence.physical_executable ? "true" : "false",
        candidate.evidence.reaches_segment_target ? "true" : "false",
        candidate.evidence.reaches_intent_target ? "true" : "false",
        candidate.evidence.reaches_mission_target ? "true" : "false",
        candidate.intent.strategic_continuation_available ? "true" : "false",
        candidate.intent.strategic_mission_continuation ? "true" : "false",
        candidate.evidence.unknown_exposure ? "true" : "false",
        candidate.evidence.known_clearance_observed ? "true" : "false",
        candidate.evidence.minimum_known_clearance_m, candidate.evidence.route_length_m,
        candidate.evidence.endpoint_displacement_m,
        candidate.evidence.mission_progress_m, candidate.evidence.objective_cost);
  }
  return ProductionRouteCandidateSet3D{
      .candidates = std::move(candidates),
      .preferred_direction = preferred_direction,
      .search_ms = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - search_started)
                       .count(),
  };
}

} // namespace drone_city_nav
