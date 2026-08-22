#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] int checkedPositiveIntParameter(const std::int64_t value,
                                              const char* const name) {
  if (value <= 0 || value > std::numeric_limits<int>::max()) {
    throw std::invalid_argument{std::string{name} + " must fit a positive int"};
  }
  return static_cast<int>(value);
}

[[nodiscard]] std::size_t checkedPositiveSizeParameter(const std::int64_t value,
                                                       const char* const name) {
  if (value <= 0) {
    throw std::invalid_argument{std::string{name} + " must be positive"};
  }
  return static_cast<std::size_t>(value);
}

[[nodiscard]] std::size_t checkedNonnegativeSizeParameter(const std::int64_t value,
                                                          const char* const name) {
  if (value < 0) {
    throw std::invalid_argument{std::string{name} + " must be nonnegative"};
  }
  return static_cast<std::size_t>(value);
}

[[nodiscard]] std::uint64_t
nodeIdValue(const std::optional<IncrementalTopologyNodeId>& node) noexcept {
  return node.has_value() ? node->value : 0U;
}

} // namespace

void ProductionMppiNode::topologyWorker(const std::stop_token stop_token) {
  while (!stop_token.stop_requested()) {
    std::shared_ptr<const ProductionMppiRawWorld3D> raw_world;
    {
      std::unique_lock lock{topology_queue_mutex_};
      topology_queue_condition_.wait(
          lock, stop_token, [this]() { return pending_topology_world_3d_ != nullptr; });
      if (stop_token.stop_requested()) {
        return;
      }
      raw_world = std::exchange(pending_topology_world_3d_, nullptr);
    }
    if (raw_world) {
      processObservedTopology3D(*raw_world);
    }
  }
}

void ProductionMppiNode::processObservedTopology3D(
    const ProductionMppiRawWorld3D& raw_world) {
  if (!topological_navigation_3d_ || !raw_world.occupancy) {
    return;
  }

  ProductionMppiNavigation navigation;
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
  }
  const std::shared_ptr<const ProductionNavigationObjective> objective =
      navigationObjective();
  const std::optional<IncrementalTopologyBuildPriority3D> priority =
      navigation.valid && objective
          ? std::optional<IncrementalTopologyBuildPriority3D>{{
                .position = {navigation.state.x, navigation.state.y,
                             navigation.state.z},
                .target = objective->goal,
            }}
          : std::nullopt;

  RCLCPP_INFO(get_logger(),
              "INCREMENTAL_TOPOLOGY3D_UPDATE_START raw_revision=%" PRIu64
              " full_reset=%s dirty_chunks=%zu",
              raw_world.revision, raw_world.full_reset ? "true" : "false",
              raw_world.dirty_chunks.size());
  const auto started = std::chrono::steady_clock::now();
  const IncrementalTopologicalWorldUpdate3D update =
      topological_navigation_3d_->updateObserved(
          *raw_world.occupancy, raw_world.producer_instance_id, raw_world.revision,
          raw_world.dirty_chunks, raw_world.full_reset, priority);
  const double update_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - started)
                               .count();
  {
    const std::scoped_lock lock{topology_state_mutex_};
    latest_observed_topological_graph_ = update.snapshot;
    latest_observed_topological_producer_instance_id_ = raw_world.producer_instance_id;
    latest_observed_topological_graph_update_ = update.graph;
  }
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    if (prepared_esdf_ &&
        prepared_esdf_->producer_instance_id == raw_world.producer_instance_id) {
      prepared_esdf_->topological_graph = update.snapshot;
      prepared_esdf_->topological_graph_update = update.graph;
    }
  }
  RCLCPP_INFO(
      get_logger(),
      "INCREMENTAL_TOPOLOGY3D_UPDATE revision=%" PRIu64
      " full_reset=%s dirty_chunks=%zu discovered_dirty_blocks=%zu "
      "rebuilt_blocks=%zu pending_blocks=%zu refined_blocks=%zu "
      "base_resolution_m=%.3f coarse_resolution_m=%.3f "
      "refined_resolution_m=%.3f retained_nodes=%zu created_nodes=%zu "
      "retired_nodes=%zu nodes=%zu edges=%zu dirty_discovery_ms=%.2f "
      "rebuild_ms=%.2f update_ms=%.2f",
      update.graph.revision, update.graph.full_reset ? "true" : "false",
      update.graph.requested_dirty_chunks, update.graph.discovered_dirty_blocks,
      update.graph.rebuilt_blocks, update.graph.pending_blocks,
      update.graph.adaptively_refined_blocks,
      raw_world.occupancy->bounds().resolution_m,
      raw_world.occupancy->bounds().resolution_m *
          static_cast<double>(topological_graph_3d_config_.coarse_sample_stride_cells),
      raw_world.occupancy->bounds().resolution_m *
          static_cast<double>(topological_graph_3d_config_.refined_sample_stride_cells),
      update.graph.retained_node_ids, update.graph.created_nodes,
      update.graph.retired_nodes, update.graph.node_count, update.graph.edge_count,
      update.graph.dirty_block_discovery_ms, update.graph.graph_rebuild_ms, update_ms);
}

void ProductionMppiNode::configureIncrementalTopology3D() {
  topological_graph_3d_config_.block_size_cells = checkedPositiveIntParameter(
      declare_parameter<std::int64_t>("topological_graph_3d_block_size_cells", 16),
      "topological_graph_3d_block_size_cells");
  topological_graph_3d_config_.coarse_sample_stride_cells = checkedPositiveIntParameter(
      declare_parameter<std::int64_t>("topological_graph_3d_coarse_sample_stride_cells",
                                      2),
      "topological_graph_3d_coarse_sample_stride_cells");
  topological_graph_3d_config_.refined_sample_stride_cells =
      checkedPositiveIntParameter(
          declare_parameter<std::int64_t>(
              "topological_graph_3d_refined_sample_stride_cells", 1),
          "topological_graph_3d_refined_sample_stride_cells");
  topological_graph_3d_config_.maximum_observed_blocks_per_update =
      checkedPositiveSizeParameter(
          declare_parameter<std::int64_t>(
              "topological_graph_3d_maximum_observed_blocks_per_update", 16),
          "topological_graph_3d_maximum_observed_blocks_per_update");
  topological_graph_3d_config_.minimum_oldest_blocks_per_update =
      checkedNonnegativeSizeParameter(
          declare_parameter<std::int64_t>(
              "topological_graph_3d_minimum_oldest_blocks_per_update", 4),
          "topological_graph_3d_minimum_oldest_blocks_per_update");
  topological_graph_3d_config_.footprint = physical_footprint_config_;

  topological_planner_3d_config_.maximum_start_anchor_distance_m =
      declare_parameter<double>("topological_planner_3d_start_anchor_distance_m", 20.0);
  topological_planner_3d_config_.require_known_free_space =
      require_known_free_space_for_goal_;
  topological_planner_3d_config_.maximum_goal_anchor_distance_m =
      declare_parameter<double>("topological_planner_3d_goal_anchor_distance_m", 8.0);
  topological_planner_3d_config_.path_cost_weight =
      declare_parameter<double>("topological_planner_3d_path_cost_weight", 1.0);
  topological_planner_3d_config_.information_gain_reward =
      declare_parameter<double>("topological_planner_3d_information_gain_reward", 1.0);
  topological_planner_3d_config_.clearance_reward =
      declare_parameter<double>("topological_planner_3d_clearance_reward", 0.5);
  topological_planner_3d_config_.goal_progress_reward =
      declare_parameter<double>("topological_planner_3d_goal_progress_reward", 3.0);
  topological_planner_3d_config_.directed_traversal_penalty =
      declare_parameter<double>("topological_planner_3d_traversal_penalty", 6.0);
  topological_planner_3d_config_.repeated_distance_penalty = declare_parameter<double>(
      "topological_planner_3d_repeated_distance_penalty", 0.25);
  topological_planner_3d_config_.dead_end_penalty =
      declare_parameter<double>("topological_planner_3d_dead_end_penalty", 100.0);
  topological_planner_3d_config_.frontier_selection_penalty = declare_parameter<double>(
      "topological_planner_3d_frontier_selection_penalty", 8.0);
  topological_planner_3d_config_.frontier_completion_penalty =
      declare_parameter<double>("topological_planner_3d_frontier_completion_penalty",
                                48.0);
  topological_planner_3d_config_.coverage_penalty_weight =
      declare_parameter<double>("topological_planner_3d_coverage_penalty_weight", 1.0);
  topological_planner_3d_config_.maximum_fresh_frontier_anchor_distance_m =
      declare_parameter<double>(
          "topological_planner_3d_maximum_fresh_frontier_anchor_distance_m",
          lattice_3d_config_.sensor_observability.maximum_observation_range_m);
  topological_planner_3d_config_.minimum_observation_target_displacement_m =
      declare_parameter<double>(
          "topological_planner_3d_minimum_observation_target_displacement_m", 2.0);
  topological_planner_3d_config_.maximum_fresh_frontier_evaluations =
      checkedPositiveSizeParameter(
          declare_parameter<std::int64_t>(
              "topological_planner_3d_maximum_fresh_frontier_evaluations", 16),
          "topological_planner_3d_maximum_fresh_frontier_evaluations");
  topological_backtracking_enabled_ =
      declare_parameter<bool>("topological_backtracking_enabled", false);

  topological_memory_3d_config_.coverage_resolution_m =
      declare_parameter<double>("topological_memory_3d_coverage_resolution_m", 2.0);
  topological_memory_3d_config_.coverage_influence_radius_m = declare_parameter<double>(
      "topological_memory_3d_coverage_influence_radius_m", 4.0);
  topological_memory_3d_config_.visit_penalty_weight =
      declare_parameter<double>("topological_memory_3d_visit_penalty_weight", 2.0);
  topological_memory_3d_config_.observation_penalty_weight = declare_parameter<double>(
      "topological_memory_3d_observation_penalty_weight", 0.25);
  topological_memory_3d_config_.revision_decay =
      declare_parameter<double>("topological_memory_3d_revision_decay", 0.0);
  topological_memory_3d_config_.maximum_observed_transition_m =
      declare_parameter<double>("topological_memory_3d_maximum_observed_transition_m",
                                4.0);
  topological_memory_3d_config_.maximum_trail_nodes = checkedPositiveSizeParameter(
      declare_parameter<std::int64_t>("topological_memory_3d_maximum_trail_nodes",
                                      4096),
      "topological_memory_3d_maximum_trail_nodes");

  topological_lattice_adapter_3d_config_.maximum_lookahead_m =
      declare_parameter<double>("topological_lattice_3d_maximum_lookahead_m", 30.0);
  topological_lattice_adapter_3d_config_.minimum_target_displacement_m =
      declare_parameter<double>("topological_lattice_3d_minimum_target_displacement_m",
                                0.25);
  const double observation_rate_hz =
      declare_parameter<double>("topological_observation_rate_hz", 5.0);
  if (!incrementalTopologicalLatticeAdapter3DConfigIsValid(
          topological_lattice_adapter_3d_config_) ||
      !std::isfinite(observation_rate_hz) || observation_rate_hz <= 0.0) {
    throw std::invalid_argument{"invalid incremental topology runtime configuration"};
  }
  topological_observation_period_ns_ =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>{1.0 / observation_rate_hz})
          .count();
  if (topological_observation_period_ns_ <= 0) {
    throw std::invalid_argument{"topological observation period must be positive"};
  }

  topological_navigation_3d_ = std::make_unique<IncrementalTopologicalNavigation3D>(
      topological_graph_3d_config_, topological_planner_3d_config_,
      topological_memory_3d_config_, lattice_3d_config_.sensor_observability);
}

void ProductionMppiNode::initializeStaticTopology3D() {
  if (!static_occupancy_3d_ || !topological_navigation_3d_) {
    throw std::logic_error{
        "static topology requires initialized occupancy and planner"};
  }

  const auto started = std::chrono::steady_clock::now();
  const IncrementalTopologicalWorldUpdate3D update =
      topological_navigation_3d_->resetStatic(*static_occupancy_3d_,
                                              static_occupancy_3d_->fingerprint());
  const double build_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - started)
                              .count();
  RCLCPP_INFO(get_logger(),
              "INCREMENTAL_TOPOLOGY3D_STATIC revision=%" PRIu64
              " nodes=%zu edges=%zu rebuilt_blocks=%zu refined_blocks=%zu "
              "base_resolution_m=%.3f coarse_resolution_m=%.3f "
              "refined_resolution_m=%.3f sampled_cells=%zu build_ms=%.2f",
              update.graph.revision, update.graph.node_count, update.graph.edge_count,
              update.graph.rebuilt_blocks, update.graph.adaptively_refined_blocks,
              static_occupancy_3d_->bounds().resolution_m,
              static_occupancy_3d_->bounds().resolution_m *
                  topological_graph_3d_config_.coarse_sample_stride_cells,
              static_occupancy_3d_->bounds().resolution_m *
                  topological_graph_3d_config_.refined_sample_stride_cells,
              update.graph.sampled_navigable_cells, build_ms);
}

ProductionIncrementalTopologySearch3D
ProductionMppiNode::selectIncrementalTopologyRoute3D(
    const ProductionMppiPreparedEsdf& world, const Point3& position,
    const Point3& mission_goal) {
  ProductionIncrementalTopologySearch3D result;
  if (!topological_navigation_3d_ || !world.topological_graph) {
    return result;
  }

  result.graph_node_count = world.topological_graph->nodes().size();
  result.graph_edge_count = world.topological_graph->edges().size();
  result.observation = topological_navigation_3d_->observePosition(
      world.topological_graph, position, world.observed_occupancy.get());
  result.plan = world.observed_occupancy
                    ? topological_navigation_3d_->planObserved(
                          world.topological_graph, *world.observed_occupancy, position,
                          mission_goal)
                    : topological_navigation_3d_->plan(world.topological_graph,
                                                       position, mission_goal);
  result.directive = makeIncrementalTopologicalLatticeDirective3D(
      result.plan, position, topological_lattice_adapter_3d_config_);
  const bool topological_backtracking_route =
      result.plan.purpose ==
          IncrementalTopologicalRoutePurpose3D::kTopologicalBacktrack ||
      result.plan.goal_progress_m < -1.0e-6;
  if (!topological_backtracking_enabled_ && topological_backtracking_route) {
    result.directive.reset();
    RCLCPP_INFO(get_logger(),
                "INCREMENTAL_TOPOLOGY3D_BACKTRACK status=disabled purpose=%s "
                "goal_progress_m=%.2f",
                incrementalTopologicalRoutePurpose3DName(result.plan.purpose),
                result.plan.goal_progress_m);
  }

  const auto now = std::chrono::steady_clock::now();
  if (result.directive.has_value()) {
    topological_no_executable_route_since_ = {};
  } else {
    if (topological_no_executable_route_since_ ==
        std::chrono::steady_clock::time_point{}) {
      topological_no_executable_route_since_ = now;
    }
    result.no_executable_route_age_ms =
        std::chrono::duration<double, std::milli>(
            now - topological_no_executable_route_since_)
            .count();
  }
  return result;
}

void ProductionMppiNode::commitIncrementalTopologyRoute3D(
    ProductionIncrementalTopologySearch3D& search) {
  if (!topological_navigation_3d_ || !search.directive.has_value()) {
    return;
  }
  search.commit = topological_navigation_3d_->commitAcceptedPlan(search.plan);
}

void ProductionMppiNode::logIncrementalTopologyRoute3D(
    const ProductionIncrementalTopologySearch3D& search,
    const RiskAwareLattice3DResult& lattice,
    const StaticRouteCandidateValidation& validation,
    const StaticRouteActivationStatus activation_status, const bool activated) {
  const ObservationFrontier* selected_frontier{nullptr};
  if (search.plan.selected_frontier.has_value()) {
    selected_frontier = std::addressof(search.plan.selected_frontier.value());
  }
  const TopologicalDeadEndConclusion3D* dead_end{nullptr};
  if (search.plan.dead_end_conclusion.has_value()) {
    dead_end = std::addressof(search.plan.dead_end_conclusion.value());
  }
  const IncrementalTopologicalLatticeDirective3D* directive{nullptr};
  if (search.directive.has_value()) {
    directive = std::addressof(search.directive.value());
  }
  const bool lattice_executable =
      lattice.status == Lattice3DStatus::kReachedPlanningGoal ||
      lattice.status == Lattice3DStatus::kViableFrontier;
  const Point3 directive_target =
      directive != nullptr ? directive->lattice.planning_goal : Point3{};

  RCLCPP_INFO(
      get_logger(),
      "INCREMENTAL_TOPOLOGICAL_PLAN3D graph_revision=%" PRIu64
      " graph_nodes=%zu graph_edges=%zu status=%s purpose=%s "
      "start_node=%" PRIu64 " target_node=%" PRIu64 " goal_node=%" PRIu64
      " route_nodes=%zu route_edges=%zu selected_frontier_id=%" PRIu64
      " frontier_boundary=(%.2f,%.2f,%.2f)"
      " frontier_direction=(%.3f,%.3f,%.3f) frontier_gain=%zu"
      " frontier_required_gain=%zu"
      " reachable_frontiers=%zu selection_score=%.3f goal_progress_m=%.2f "
      "frontier_selection_count=%zu frontier_completion_count=%zu "
      "goal_directed_reachable_frontiers=%zu "
      "maximum_goal_progress_frontier_id=%" PRIu64
      " maximum_reachable_frontier_goal_progress_m=%.2f "
      "fresh_frontier_candidates=%zu fresh_frontier_evaluated=%zu "
      "fresh_frontier_discovered=%zu "
      "fresh_frontier_sample_fingerprint=%" PRIu64 " "
      "fresh_frontier_status_accepted=%zu "
      "fresh_frontier_status_outside_map=%zu "
      "fresh_frontier_status_footprint_not_observed=%zu "
      "fresh_frontier_status_raw_collision=%zu "
      "fresh_frontier_status_no_unknown_boundary=%zu "
      "fresh_frontier_status_insufficient_ray_support=%zu "
      "fresh_frontier_status_insufficient_information_gain=%zu "
      "fresh_frontier_budget_exhausted=%s "
      "coverage_penalty=%.3f directed_traversals=%zu "
      "repeated_edge_distance_m=%.2f "
      "backtrack_reason=%s dead_end_edge_id=%" PRIu64 " dead_end_from=%" PRIu64
      " dead_end_to=%" PRIu64 " dead_end_revision=%" PRIu64
      " no_executable_route_age_ms=%.2f "
      "observation_previous_node=%" PRIu64 " observation_current_node=%" PRIu64
      " observation_traversed_edges=%zu observation_coverage_cells=%zu "
      "observation_trail_reset=%s directive_available=%s "
      "directive_source_segment=%zu directive_source_station_m=%.2f "
      "directive_target_station_m=%.2f directive_projection_distance_m=%.2f "
      "directive_target=(%.2f,%.2f,%.2f) directive_reaches_target=%s "
      "lattice_status=%s lattice_purpose=%s lattice_executable=%s "
      "candidate_validation=%.*s activation=%.*s activated=%s "
      "commit_accepted=%s commit_route_nodes=%zu commit_frontier=%s "
      "commit_replaced_frontier_coverage=%s commit_dead_end=%s",
      search.plan.planned_on_revision, search.graph_node_count, search.graph_edge_count,
      incrementalTopologicalPlanStatus3DName(search.plan.status),
      incrementalTopologicalRoutePurpose3DName(search.plan.purpose),
      search.plan.start_node.value, search.plan.target_node.value,
      nodeIdValue(search.plan.goal_node), search.plan.route_nodes.size(),
      search.plan.route_steps.size(),
      selected_frontier != nullptr ? selected_frontier->id.value : 0U,
      selected_frontier != nullptr ? selected_frontier->boundary_centroid.x : 0.0,
      selected_frontier != nullptr ? selected_frontier->boundary_centroid.y : 0.0,
      selected_frontier != nullptr ? selected_frontier->boundary_centroid.z : 0.0,
      selected_frontier != nullptr ? selected_frontier->observation_direction.x : 0.0,
      selected_frontier != nullptr ? selected_frontier->observation_direction.y : 0.0,
      selected_frontier != nullptr ? selected_frontier->observation_direction.z : 0.0,
      selected_frontier != nullptr ? selected_frontier->information_gain_voxels : 0U,
      selected_frontier != nullptr ? selected_frontier->required_information_gain_voxels
                                   : 0U,
      search.plan.reachable_frontier_count, search.plan.selection_score,
      search.plan.goal_progress_m, search.plan.selected_frontier_selection_count,
      search.plan.selected_frontier_completion_count,
      search.plan.goal_directed_reachable_frontier_count,
      search.plan.maximum_goal_progress_frontier_id.value,
      search.plan.maximum_reachable_frontier_goal_progress_m,
      search.plan.fresh_frontier_candidate_count,
      search.plan.fresh_frontier_evaluated_count,
      search.plan.fresh_frontier_discovered_count,
      search.plan.fresh_frontier_sample_fingerprint,
      search.plan.fresh_frontier_status_counts[static_cast<std::size_t>(
          ObservationFrontierStatus::kAccepted)],
      search.plan.fresh_frontier_status_counts[static_cast<std::size_t>(
          ObservationFrontierStatus::kOutsideMap)],
      search.plan.fresh_frontier_status_counts[static_cast<std::size_t>(
          ObservationFrontierStatus::kFootprintNotObserved)],
      search.plan.fresh_frontier_status_counts[static_cast<std::size_t>(
          ObservationFrontierStatus::kRawCollision)],
      search.plan.fresh_frontier_status_counts[static_cast<std::size_t>(
          ObservationFrontierStatus::kNoUnknownBoundary)],
      search.plan.fresh_frontier_status_counts[static_cast<std::size_t>(
          ObservationFrontierStatus::kInsufficientRaySupport)],
      search.plan.fresh_frontier_status_counts[static_cast<std::size_t>(
          ObservationFrontierStatus::kInsufficientInformationGain)],
      search.plan.fresh_frontier_budget_exhausted ? "true" : "false",
      search.plan.coverage_penalty, search.plan.directed_traversal_count,
      search.plan.repeated_edge_distance_m,
      topologicalBacktrackReason3DName(search.plan.backtrack_reason),
      dead_end != nullptr ? dead_end->attempted_direction.edge_id.value : 0U,
      dead_end != nullptr ? dead_end->attempted_direction.from.value : 0U,
      dead_end != nullptr ? dead_end->attempted_direction.to.value : 0U,
      dead_end != nullptr ? dead_end->validated_through_revision : 0U,
      search.no_executable_route_age_ms, nodeIdValue(search.observation.previous_node),
      nodeIdValue(search.observation.current_node), search.observation.traversed_edges,
      search.observation.coverage_cells,
      search.observation.trail_reset ? "true" : "false",
      directive != nullptr ? "true" : "false",
      directive != nullptr ? directive->source_segment_index : 0U,
      directive != nullptr ? directive->source_station_m : 0.0,
      directive != nullptr ? directive->target_station_m : 0.0,
      directive != nullptr ? directive->projection_distance_m : 0.0, directive_target.x,
      directive_target.y, directive_target.z,
      directive != nullptr && directive->reaches_topological_target ? "true" : "false",
      lattice3DStatusName(lattice.status),
      lattice3DRoutePurposeName(lattice.route_purpose),
      lattice_executable ? "true" : "false",
      static_cast<int>(staticRouteCandidateStatusName(validation.status).size()),
      staticRouteCandidateStatusName(validation.status).data(),
      static_cast<int>(staticRouteActivationStatusName(activation_status).size()),
      staticRouteActivationStatusName(activation_status).data(),
      activated ? "true" : "false", search.commit.accepted ? "true" : "false",
      search.commit.active_route_nodes,
      search.commit.frontier_selection_recorded ? "true" : "false",
      search.commit.replaced_frontier_coverage_recorded ? "true" : "false",
      search.commit.dead_end_recorded ? "true" : "false");

  for (std::size_t index = 0U; index < search.plan.route_steps.size(); ++index) {
    const TopologicalRouteStep3D& step = search.plan.route_steps[index];
    RCLCPP_INFO(get_logger(),
                "INCREMENTAL_TOPOLOGICAL_ROUTE_EDGE3D graph_revision=%" PRIu64
                " step=%zu regional_edge_id=%" PRIu64 " from=%" PRIu64 " to=%" PRIu64
                " source_edges=%zu length_m=%.2f "
                "repeated_distance_m=%.2f traversal_count=%zu "
                "validated_through_revision=%" PRIu64,
                search.plan.planned_on_revision, index, step.regional_edge_id.value,
                step.from.value, step.to.value, step.directed_source_edges.size(),
                step.length_m, step.repeated_distance_m, step.traversal_count,
                step.validated_through_revision);
    for (std::size_t source_index = 0U;
         source_index < step.directed_source_edges.size(); ++source_index) {
      const DirectedTopologyEdge3D& source = step.directed_source_edges[source_index];
      RCLCPP_INFO(get_logger(),
                  "INCREMENTAL_TOPOLOGICAL_SOURCE_EDGE3D graph_revision=%" PRIu64
                  " step=%zu source_index=%zu edge_id=%" PRIu64 " from=%" PRIu64
                  " to=%" PRIu64,
                  search.plan.planned_on_revision, index, source_index,
                  source.edge_id.value, source.from.value, source.to.value);
    }
  }
}

void ProductionMppiNode::maybeObserveIncrementalTopology3D(
    const ProductionMppiPreparedEsdf& world, const ProductionMppiNavigation& navigation,
    const std::int64_t now_ns) {
  const std::shared_ptr<const IncrementalTopologyGraph3DSnapshot>& graph =
      world.topological_graph;
  if (!topological_navigation_3d_ || !graph || !navigation.valid || now_ns <= 0) {
    return;
  }
  std::uint64_t previous_graph_revision =
      last_topological_observation_graph_revision_.load(std::memory_order_relaxed);
  while (graph->revision() > previous_graph_revision &&
         !last_topological_observation_graph_revision_.compare_exchange_weak(
             previous_graph_revision, graph->revision(), std::memory_order_acq_rel,
             std::memory_order_relaxed)) {
  }
  if (graph->revision() < previous_graph_revision) {
    return;
  }
  std::int64_t previous_stamp =
      last_topological_observation_stamp_ns_.load(std::memory_order_relaxed);
  if (previous_stamp > 0 &&
      now_ns - previous_stamp < topological_observation_period_ns_) {
    return;
  }
  if (!last_topological_observation_stamp_ns_.compare_exchange_strong(
          previous_stamp, now_ns, std::memory_order_acq_rel,
          std::memory_order_relaxed)) {
    return;
  }

  const IncrementalTopologicalNavigationObservation3D observation =
      topological_navigation_3d_->observePosition(
          graph, Point3{navigation.state.x, navigation.state.y, navigation.state.z},
          world.observed_occupancy.get());
  if (observation.previous_node != observation.current_node ||
      observation.traversed_edges > 0U || observation.trail_reset) {
    RCLCPP_INFO(get_logger(),
                "INCREMENTAL_TOPOLOGY3D_OBSERVATION graph_revision=%" PRIu64
                " previous_node=%" PRIu64 " current_node=%" PRIu64
                " traversed_edges=%zu coverage_cells=%zu trail_reset=%s",
                observation.graph_revision, nodeIdValue(observation.previous_node),
                nodeIdValue(observation.current_node), observation.traversed_edges,
                observation.coverage_cells, observation.trail_reset ? "true" : "false");
  }
}

} // namespace drone_city_nav
