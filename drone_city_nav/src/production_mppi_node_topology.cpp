#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

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

} // namespace

void ProductionMppiNode::configureIncrementalTopology3D() {
  topological_graph_3d_config_.tile_size_cells = checkedPositiveIntParameter(
      declare_parameter<std::int64_t>("topological_graph_3d_tile_size_cells", 8),
      "topological_graph_3d_tile_size_cells");
  topological_graph_3d_config_.sample_stride_cells = checkedPositiveIntParameter(
      declare_parameter<std::int64_t>("topological_graph_3d_sample_stride_cells", 2),
      "topological_graph_3d_sample_stride_cells");
  topological_graph_3d_config_.maximum_frontier_evaluations_per_component =
      checkedPositiveSizeParameter(
          declare_parameter<std::int64_t>(
              "topological_graph_3d_maximum_frontier_evaluations_per_component", 128),
          "topological_graph_3d_maximum_frontier_evaluations_per_component");
  topological_graph_3d_config_.footprint = physical_footprint_config_;
  topological_graph_3d_config_.observability = lattice_3d_config_.sensor_observability;

  topological_planner_3d_config_.maximum_start_anchor_distance_m =
      declare_parameter<double>("topological_planner_3d_start_anchor_distance_m", 8.0);
  topological_planner_3d_config_.maximum_goal_anchor_distance_m =
      declare_parameter<double>("topological_planner_3d_goal_anchor_distance_m", 8.0);
  topological_planner_3d_config_.path_cost_weight =
      declare_parameter<double>("topological_planner_3d_path_cost_weight", 1.0);
  topological_planner_3d_config_.information_gain_reward =
      declare_parameter<double>("topological_planner_3d_information_gain_reward", 3.0);
  topological_planner_3d_config_.clearance_reward =
      declare_parameter<double>("topological_planner_3d_clearance_reward", 0.5);
  topological_planner_3d_config_.goal_progress_reward =
      declare_parameter<double>("topological_planner_3d_goal_progress_reward", 0.25);
  topological_planner_3d_config_.directed_traversal_penalty =
      declare_parameter<double>("topological_planner_3d_traversal_penalty", 6.0);
  topological_planner_3d_config_.repeated_distance_penalty = declare_parameter<double>(
      "topological_planner_3d_repeated_distance_penalty", 0.25);
  topological_planner_3d_config_.frontier_selection_penalty = declare_parameter<double>(
      "topological_planner_3d_frontier_selection_penalty", 8.0);
  topological_planner_3d_config_.coverage_penalty_weight =
      declare_parameter<double>("topological_planner_3d_coverage_penalty_weight", 1.0);

  topological_memory_3d_config_.coverage_resolution_m =
      declare_parameter<double>("topological_memory_3d_coverage_resolution_m", 2.0);
  topological_memory_3d_config_.visit_penalty_weight =
      declare_parameter<double>("topological_memory_3d_visit_penalty_weight", 2.0);
  topological_memory_3d_config_.observation_penalty_weight = declare_parameter<double>(
      "topological_memory_3d_observation_penalty_weight", 0.25);
  topological_memory_3d_config_.revision_decay =
      declare_parameter<double>("topological_memory_3d_revision_decay", 0.02);
  topological_memory_3d_config_.maximum_trail_nodes = checkedPositiveSizeParameter(
      declare_parameter<std::int64_t>("topological_memory_3d_maximum_trail_nodes",
                                      4096),
      "topological_memory_3d_maximum_trail_nodes");

  topological_navigation_3d_ = std::make_unique<IncrementalTopologicalNavigation3D>(
      topological_graph_3d_config_, topological_planner_3d_config_,
      topological_memory_3d_config_);
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
              " nodes=%zu edges=%zu rebuilt_tiles=%zu build_ms=%.2f",
              update.graph.revision, update.graph.node_count, update.graph.edge_count,
              update.graph.rebuilt_tiles, build_ms);
}

} // namespace drone_city_nav
