#include <algorithm>
#include <cstdint>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::configureOptionalNavigationConstraints() {
  optional_constraints_ = ProductionNavigationOptionalConstraints{
      .clearance_costs_enabled =
          declare_parameter<bool>("clearance_costs_enabled", false),
      .static_route_shortcut_optimization_enabled =
          declare_parameter<bool>("static_route_shortcut_optimization_enabled", false),
      .stochastic_trajectory_selection_enabled =
          declare_parameter<bool>("stochastic_trajectory_selection_enabled", false),
      .route_cross_track_constraints_enabled = declare_parameter<bool>(
          "execution_route_cross_track_constraints_enabled", false),
      .route_tracking_tube_constraints_enabled = declare_parameter<bool>(
          "execution_route_tracking_tube_constraints_enabled", false),
      .route_progress_replan_enabled =
          declare_parameter<bool>("route_progress_replan_enabled", false),
      .no_eligible_route_replan_enabled =
          declare_parameter<bool>("no_eligible_route_replan_enabled", false),
      .latest_lidar_freshness_required =
          declare_parameter<bool>("execution_latest_lidar_freshness_required", false),
      .nonphysical_execution_revocation_enabled =
          declare_parameter<bool>("execution_nonphysical_revocation_enabled", false),
  };
}

void ProductionMppiNode::configureStaticRouteGeometry() {
  static_route_geometry_config_.sample_step_m = route_sampling_step_m_;
  static_route_geometry_config_.enabled = true;
  static_route_geometry_config_.shortcut_optimization_enabled =
      optional_constraints_.static_route_shortcut_optimization_enabled;
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

} // namespace drone_city_nav
