#include <algorithm>
#include <cstdint>
#include <numbers>
#include <stdexcept>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::configureOptionalNavigationConstraints() {
  optional_constraints_ = ProductionNavigationOptionalConstraints{
      .reject_invalid_esdf_routes =
          declare_parameter<bool>("reject_invalid_esdf_routes", false),
      .clearance_costs_enabled =
          declare_parameter<bool>("clearance_costs_enabled", false),
      .route_shape_costs_enabled =
          declare_parameter<bool>("route_shape_costs_enabled", false),
      .frontier_viability_enabled =
          declare_parameter<bool>("frontier_viability_constraints_enabled", false),
      .topological_history_costs_enabled =
          declare_parameter<bool>("topological_history_costs_enabled", false),
      .route_proposal_precedence_enabled =
          declare_parameter<bool>("route_proposal_precedence_enabled", false),
      .route_strategy_leases_enabled =
          declare_parameter<bool>("route_strategy_leases_enabled", false),
      .static_route_geometry_optimization_enabled =
          declare_parameter<bool>("static_route_geometry_optimization_enabled", false),
      .stochastic_trajectory_selection_enabled =
          declare_parameter<bool>("stochastic_trajectory_selection_enabled", false),
      .observation_frontier_stops_enabled =
          declare_parameter<bool>("observation_frontier_stops_enabled", false),
      .topological_segment_boundaries_enabled =
          declare_parameter<bool>("topological_segment_boundaries_enabled", false),
      .route_replacement_progress_enabled =
          declare_parameter<bool>("route_replacement_progress_enabled", false),
      .clearance_tier_constraints_enabled =
          declare_parameter<bool>("clearance_tier_constraints_enabled", false),
  };
}

void ProductionMppiNode::configureStaticRouteGeometry() {
  static_route_geometry_config_.sample_step_m = lattice_3d_config_.sample_step_m;
  static_route_geometry_config_.enabled =
      optional_constraints_.static_route_geometry_optimization_enabled;
  static_route_geometry_config_.maximum_shortcut_length_m =
      declare_parameter<double>("static_route_maximum_shortcut_length_m", 30.0);
  static_route_geometry_config_.sparse_deviation_tolerance_m =
      declare_parameter<double>("static_route_sparse_deviation_tolerance_m", 0.05);
  static_route_geometry_config_.maximum_shortcut_turn_increase_rad =
      declare_parameter<double>("static_route_maximum_shortcut_turn_increase_rad",
                                0.35);
  static_route_geometry_config_.shortcut_validation_batch_size =
      static_cast<std::size_t>(std::max<std::int64_t>(
          0, declare_parameter<std::int64_t>(
                 "static_route_shortcut_validation_batch_size", 4)));
  static_route_geometry_config_.corner_smoothing_distance_m =
      declare_parameter<double>("static_route_corner_smoothing_distance_m", 2.0);
  static_route_geometry_config_.corner_curve_samples = static_cast<std::size_t>(
      declare_parameter<std::int64_t>("static_route_corner_curve_samples", 4));
}

void ProductionMppiNode::configureSensorObservability() {
  SensorObservabilityConfig& config = lattice_3d_config_.sensor_observability;
  config.maximum_observation_range_m = declare_parameter<double>(
      "global_lattice_3d_observation_frontier_maximum_range_m", 8.0);
  const double minimum_known_free_ray_m = declare_parameter<double>(
      "global_lattice_3d_observation_frontier_minimum_known_free_ray_m", 1.0);
  config.minimum_known_free_ray_m =
      optional_constraints_.frontier_viability_enabled ? minimum_known_free_ray_m : 0.0;
  config.frontier_identity_resolution_m = declare_parameter<double>(
      "global_lattice_3d_observation_frontier_identity_resolution_m",
      config.maximum_observation_range_m);
  config.horizontal_min_angle_rad =
      declare_parameter<double>("lidar_3d_horizontal_min_angle_rad", -std::numbers::pi);
  config.horizontal_max_angle_rad =
      declare_parameter<double>("lidar_3d_horizontal_max_angle_rad", std::numbers::pi);
  config.vertical_min_angle_rad =
      declare_parameter<double>("lidar_3d_vertical_min_angle_rad", -1.3962634015954636);
  config.vertical_max_angle_rad =
      declare_parameter<double>("lidar_3d_vertical_max_angle_rad", 1.3962634015954636);
  config.directional_cluster_half_angle_rad = declare_parameter<double>(
      "global_lattice_3d_observation_frontier_directional_cluster_half_angle_rad",
      std::numbers::pi / 4.0);
  const std::int64_t horizontal_samples =
      declare_parameter<std::int64_t>("lidar_3d_horizontal_samples", 240);
  const std::int64_t vertical_samples =
      declare_parameter<std::int64_t>("lidar_3d_vertical_samples", 17);
  const std::int64_t minimum_supporting_rays = declare_parameter<std::int64_t>(
      "global_lattice_3d_observation_frontier_minimum_supporting_rays", 2);
  const std::int64_t minimum_information_gain_voxels = declare_parameter<std::int64_t>(
      "global_lattice_3d_observation_frontier_minimum_information_gain_voxels", 4);
  if (minimum_supporting_rays <= 0 || minimum_information_gain_voxels <= 0 ||
      horizontal_samples <= 1 || vertical_samples <= 1) {
    throw std::invalid_argument{"invalid observation frontier count configuration"};
  }
  config.minimum_supporting_rays =
      optional_constraints_.frontier_viability_enabled
          ? static_cast<std::size_t>(minimum_supporting_rays)
          : 1U;
  config.minimum_information_gain_voxels =
      optional_constraints_.frontier_viability_enabled
          ? static_cast<std::size_t>(minimum_information_gain_voxels)
          : 1U;
  const double minimum_observation_pose_advance_m = declare_parameter<double>(
      "global_lattice_3d_observation_frontier_minimum_pose_advance_m", 2.0);
  config.minimum_observation_pose_advance_m =
      optional_constraints_.frontier_viability_enabled
          ? minimum_observation_pose_advance_m
          : 1.0e-3;
  config.horizontal_samples = static_cast<std::size_t>(horizontal_samples);
  config.vertical_samples = static_cast<std::size_t>(vertical_samples);
}

} // namespace drone_city_nav
