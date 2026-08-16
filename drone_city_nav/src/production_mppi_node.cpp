#include "production_mppi_node.hpp"

#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <filesystem>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdexcept>

namespace drone_city_nav {
ProductionMppiNode::ProductionMppiNode(const rclcpp::NodeOptions& options)
    : Node{"production_mppi_node", options} {
  tick_rate_hz_ = declare_parameter<double>("tick_rate_hz", 50.0);
  rviz_rate_hz_ = declare_parameter<double>("rviz_rate_hz", 10.0);
  diagnostics_info_rate_hz_ =
      declare_parameter<double>("diagnostics_info_rate_hz", 5.0);
  diagnostics_file_rate_hz_ =
      declare_parameter<double>("diagnostics_file_rate_hz", 5.0);
  diagnostics_flush_period_s_ =
      declare_parameter<double>("diagnostics_flush_period_s", 1.0);
  const std::int64_t diagnostics_error_ring_capacity =
      declare_parameter<std::int64_t>("diagnostics_error_ring_capacity", 25);
  if (diagnostics_error_ring_capacity < 1 || diagnostics_error_ring_capacity > 1000) {
    throw std::invalid_argument{"diagnostics error ring capacity must be in [1, 1000]"};
  }
  diagnostics_error_ring_capacity_ =
      static_cast<std::size_t>(diagnostics_error_ring_capacity);
  deadline_ms_ = declare_parameter<double>("deadline_ms", 20.0);
  maximum_pose_age_ms_ = declare_parameter<double>("maximum_pose_age_ms", 150.0);
  maximum_pose_prediction_age_ms_ =
      declare_parameter<double>("maximum_pose_prediction_age_ms", 1000.0);
  maximum_esdf_age_ms_ = declare_parameter<double>("maximum_esdf_age_ms", 1000.0);
  maximum_control_feedback_age_ms_ =
      declare_parameter<double>("maximum_control_feedback_age_ms", 200.0);
  latest_lidar_obstacle_maximum_age_ms_ =
      declare_parameter<double>("latest_lidar_obstacle_maximum_age_ms", 250.0);
  const std::int64_t planner_worker_count =
      declare_parameter<std::int64_t>("planner_worker_count", 4);
  if (planner_worker_count < 1 || planner_worker_count > 8) {
    throw std::invalid_argument{"planner worker count must be in [1, 8]"};
  }
  planner_worker_count_ = static_cast<std::size_t>(planner_worker_count);
  planning_tick_phase_offset_s_ =
      declare_parameter<double>("planning_tick_phase_offset_s", 0.0);
  use_static_map_ = declare_parameter<bool>("use_static_map", true);
  no_static_guide_lookahead_m_ =
      declare_parameter<double>("no_static_guide_lookahead_m", 30.0);
  no_static_esdf_update_rate_hz_ =
      declare_parameter<double>("no_static_esdf_update_rate_hz", 2.5);
  no_static_esdf_half_extent_m_ =
      declare_parameter<double>("no_static_esdf_half_extent_m", 100.0);
  no_static_esdf_recenter_margin_m_ =
      declare_parameter<double>("no_static_esdf_recenter_margin_m", 70.0);
  constrained_route_speed_limit_mps_ = static_cast<float>(
      declare_parameter<double>("constrained_route_speed_limit_mps", 10.0));
  route_constraint_diagnostics_distance_m_ =
      declare_parameter<double>("route_constraint_diagnostics_distance_m", 30.0);
  target_mode_ = declare_parameter<std::string>("target_mode", "active_route_guide");
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  diagnostics_output_dir_ =
      declare_parameter<std::string>("diagnostics_output_dir", "log/mppi");
  px4_map_transform_ = Px4MapFrameTransform{
      .map_origin = Point3{declare_parameter<double>("px4_local_origin_x_m", 54.0),
                           declare_parameter<double>("px4_local_origin_y_m", 54.0),
                           declare_parameter<double>("px4_local_origin_z_m", 0.0)},
      .m00 = declare_parameter<double>("px4_to_map_m00", 1.0),
      .m01 = declare_parameter<double>("px4_to_map_m01", 0.0),
      .m10 = declare_parameter<double>("px4_to_map_m10", 0.0),
      .m11 = declare_parameter<double>("px4_to_map_m11", 1.0),
  };
  px4_map_transform_.validate();
  mission_start_.x = declare_parameter<double>("start_x_m", 54.0);
  mission_start_.y = declare_parameter<double>("start_y_m", 54.0);
  mission_start_.z = declare_parameter<double>("start_z_m", 0.0);
  mission_goal_.x = declare_parameter<double>("goal_x_m", 216.0);
  mission_goal_.y = declare_parameter<double>("goal_y_m", 378.0);
  mission_goal_.z = declare_parameter<double>("goal_z_m", 18.0);
  flight_envelope_config_.minimum_target_z_m =
      declare_parameter<double>("minimum_target_z_m", 1.0);
  flight_envelope_config_.maximum_target_z_m =
      declare_parameter<double>("maximum_target_z_m", 32.0);
  mppi_config_.altitude_envelope = mppi::AltitudeEnvelopeConfig{
      .minimum_z_m = static_cast<float>(flight_envelope_config_.minimum_target_z_m),
      .maximum_z_m = static_cast<float>(flight_envelope_config_.maximum_target_z_m),
  };
  dynamic_objective_replan_distance_m_ =
      declare_parameter<double>("dynamic_objective_replan_distance_m", 5.0);
  dynamic_objective_replan_period_s_ =
      declare_parameter<double>("dynamic_objective_replan_period_s", 0.25);
  tracking_objective_ray_sample_spacing_m_ =
      declare_parameter<double>("tracking_objective_ray_sample_spacing_m", 0.25);
  tracking_capture_radius_m_ =
      declare_parameter<double>("tracking_capture_radius_m", 5.0);
  static_tracking_esdf_refresh_margin_m_ =
      declare_parameter<double>("static_tracking_esdf_refresh_margin_m", 15.0);
  direct_tracking_maneuver_lifecycle_ =
      DirectTrackingManeuverLifecycle{DirectTrackingManeuverConfig{
          .bearing_change_threshold_rad = declare_parameter<double>(
              "direct_tracking_reseed_bearing_change_rad", 0.5235987755982988),
          .minimum_closing_speed_mps = declare_parameter<double>(
              "direct_tracking_minimum_closing_speed_mps", 0.5),
          .closing_recovery_speed_mps = declare_parameter<double>(
              "direct_tracking_closing_recovery_speed_mps", 1.5),
          .no_closing_duration_s = declare_parameter<double>(
              "direct_tracking_no_closing_reseed_delay_s", 1.0),
          .minimum_reseed_interval_s = declare_parameter<double>(
              "direct_tracking_minimum_reseed_interval_s", 0.5),
      }};
  if (!(dynamic_objective_replan_distance_m_ > 0.0) ||
      !(dynamic_objective_replan_period_s_ > 0.0) ||
      !(tracking_objective_ray_sample_spacing_m_ > 0.0) ||
      !(tracking_capture_radius_m_ > 0.0) ||
      !(static_tracking_esdf_refresh_margin_m_ >= 0.0) ||
      !insideFlightEnvelope(mission_goal_, flight_envelope_config_)) {
    throw std::invalid_argument{"invalid navigation objective configuration"};
  }
  navigation_objective_.store(std::make_shared<const ProductionNavigationObjective>(
                                  ProductionNavigationObjective{
                                      .goal = mission_goal_,
                                      .tracking = std::nullopt,
                                  }),
                              std::memory_order_release);
  objective_replan_anchor_ = mission_goal_;
  mission_goal_capture_config_.capture_radius_m =
      declare_parameter<double>("mission_goal_capture_radius_m", 2.0);
  mppi_config_.rollouts =
      static_cast<std::size_t>(declare_parameter<int>("rollouts", 8192));
  rollout_budget_config_.full_rollouts = mppi_config_.rollouts;
  rollout_budget_config_.open_static_rollouts =
      static_cast<std::size_t>(declare_parameter<int>("open_static_rollouts", 6144));
  rollout_budget_config_.direct_tracking_rollouts = static_cast<std::size_t>(
      declare_parameter<int>("direct_tracking_rollouts", 4096));
  rollout_budget_config_.minimum_reduced_clearance_m = static_cast<float>(
      declare_parameter<double>("adaptive_rollout_minimum_clearance_m", 8.0));
  rollout_budget_config_.maximum_world_age_ms =
      declare_parameter<double>("adaptive_rollout_maximum_world_age_ms", 250.0);
  rollout_budget_config_.maximum_tracking_age_ms =
      declare_parameter<double>("adaptive_rollout_maximum_tracking_age_ms", 250.0);
  if (mppi_config_.rollouts == 0U ||
      rollout_budget_config_.direct_tracking_rollouts == 0U ||
      rollout_budget_config_.open_static_rollouts == 0U ||
      rollout_budget_config_.direct_tracking_rollouts >
          rollout_budget_config_.open_static_rollouts ||
      rollout_budget_config_.open_static_rollouts > mppi_config_.rollouts ||
      !(rollout_budget_config_.minimum_reduced_clearance_m > 0.0F) ||
      !(rollout_budget_config_.maximum_world_age_ms > 0.0) ||
      !(rollout_budget_config_.maximum_tracking_age_ms > 0.0)) {
    throw std::invalid_argument{"invalid MPPI rollout budget"};
  }
  mppi_config_.dynamics.dt_s =
      static_cast<float>(declare_parameter<double>("dt_s", 0.05));
  const double static_horizon_duration_s =
      declare_parameter<double>("static_horizon_duration_s", 6.0);
  const double no_static_horizon_duration_s =
      declare_parameter<double>("no_static_horizon_duration_s", 4.0);
  const double active_horizon_duration_s =
      use_static_map_ ? static_horizon_duration_s : no_static_horizon_duration_s;
  const double static_stale_esdf_execution_window_s = declare_parameter<double>(
      "static_stale_esdf_execution_window_s", static_horizon_duration_s);
  const double no_static_stale_esdf_execution_window_s = declare_parameter<double>(
      "no_static_stale_esdf_execution_window_s", no_static_horizon_duration_s);
  stale_esdf_execution_window_ms_ =
      1000.0 * (use_static_map_ ? static_stale_esdf_execution_window_s
                                : no_static_stale_esdf_execution_window_s);
  mppi_config_.steps = static_cast<std::size_t>(
      std::ceil(active_horizon_duration_s / mppi_config_.dynamics.dt_s));
  MppiSpeedPolicyConfig static_speed_policy_config;
  static_speed_policy_config.horizon_duration_s = static_horizon_duration_s;
  static_speed_policy_config.cruise_speed_mps =
      declare_parameter<double>("static_cruise_speed_mps", 20.0);
  static_speed_policy_config.absolute_speed_limit_mps =
      declare_parameter<double>("static_absolute_speed_limit_mps", 20.0);
  static_speed_policy_config.maximum_lateral_acceleration_mps2 =
      declare_parameter<double>("static_maximum_lateral_acceleration_mps2", 5.0);
  static_speed_policy_config.stopping_capability
      .maximum_commanded_horizontal_deceleration_mps2 =
      declare_parameter<double>("static_maximum_braking_acceleration_mps2", 8.0);
  static_speed_policy_config.stopping_capability
      .guaranteed_horizontal_deceleration_mps2 =
      declare_parameter<double>("static_guaranteed_stopping_deceleration_mps2", 4.0);
  static_speed_policy_config.stopping_capability.guaranteed_vertical_deceleration_mps2 =
      declare_parameter<double>("guaranteed_vertical_stopping_deceleration_mps2", 2.0);
  static_speed_policy_config.stopping_capability.reaction_latency_s =
      declare_parameter<double>("static_speed_reaction_latency_s", 0.10);
  static_speed_policy_config.observation_distance_m =
      declare_parameter<double>("static_observation_distance_m", 30.0);
  static_speed_policy_config.observation_margin_m =
      declare_parameter<double>("static_observation_margin_m", 3.0);
  static_speed_policy_config.goal_margin_m =
      declare_parameter<double>("static_goal_braking_margin_m", 2.0);
  static_speed_policy_config.curvature_preview_distance_m =
      declare_parameter<double>("static_curvature_preview_distance_m", 100.0);
  static_speed_policy_config.curvature_measurement_window_m =
      declare_parameter<double>("route_curvature_measurement_window_m", 5.0);
  static_speed_policy_config.minimum_target_lookahead_m =
      declare_parameter<double>("static_minimum_target_lookahead_m", 30.0);
  static_speed_policy_config.maximum_target_lookahead_m =
      declare_parameter<double>("static_maximum_target_lookahead_m", 100.0);
  const double static_maximum_horizontal_acceleration_mps2 =
      declare_parameter<double>("static_maximum_horizontal_acceleration_mps2", 8.0);
  const double static_maximum_control_jerk_mps3 =
      declare_parameter<double>("static_maximum_control_jerk_mps3", 20.0);
  MppiSpeedPolicyConfig no_static_speed_policy_config = static_speed_policy_config;
  no_static_speed_policy_config.horizon_duration_s = no_static_horizon_duration_s;
  no_static_speed_policy_config.cruise_speed_mps =
      declare_parameter<double>("no_static_cruise_speed_mps", 10.0);
  no_static_speed_policy_config.absolute_speed_limit_mps =
      declare_parameter<double>("no_static_absolute_speed_limit_mps", 10.0);
  const double no_static_maximum_horizontal_acceleration_mps2 =
      declare_parameter<double>("no_static_maximum_horizontal_acceleration_mps2", 4.0);
  const double no_static_maximum_control_jerk_mps3 =
      declare_parameter<double>("no_static_maximum_control_jerk_mps3", 12.0);
  no_static_speed_policy_config.maximum_lateral_acceleration_mps2 =
      no_static_maximum_horizontal_acceleration_mps2;
  no_static_speed_policy_config.stopping_capability
      .maximum_commanded_horizontal_deceleration_mps2 =
      no_static_maximum_horizontal_acceleration_mps2;
  no_static_speed_policy_config.stopping_capability
      .guaranteed_horizontal_deceleration_mps2 =
      no_static_maximum_horizontal_acceleration_mps2;
  no_static_speed_policy_config.curvature_preview_distance_m =
      declare_parameter<double>("no_static_curvature_preview_distance_m", 60.0);
  no_static_speed_policy_config.minimum_target_lookahead_m =
      no_static_guide_lookahead_m_;
  no_static_speed_policy_config.maximum_target_lookahead_m =
      no_static_guide_lookahead_m_;
  speed_policy_config_ =
      use_static_map_ ? static_speed_policy_config : no_static_speed_policy_config;
  mppi_config_.dynamics.maximum_horizontal_speed_mps =
      static_cast<float>(speed_policy_config_.absolute_speed_limit_mps);
  mppi_config_.dynamics.maximum_horizontal_acceleration_mps2 = static_cast<float>(
      use_static_map_ ? static_maximum_horizontal_acceleration_mps2
                      : no_static_maximum_horizontal_acceleration_mps2);
  finite_horizon_config_ =
      mppi::makeFiniteHorizonConfig(speed_policy_config_.stopping_capability);
  mppi_config_.dynamics.maximum_control_jerk_mps3 =
      static_cast<float>(use_static_map_ ? static_maximum_control_jerk_mps3
                                         : no_static_maximum_control_jerk_mps3);
  mppi_config_.dynamics.maximum_vertical_acceleration_mps2 = static_cast<float>(
      declare_parameter<double>("maximum_vertical_acceleration_mps2", 4.0));
  mppi_config_.altitude_envelope.guaranteed_vertical_deceleration_mps2 =
      static_cast<float>(speed_policy_config_.stopping_capability
                             .guaranteed_vertical_deceleration_mps2);
  mppi_config_.altitude_envelope.reaction_latency_s =
      static_cast<float>(speed_policy_config_.stopping_capability.reaction_latency_s);
  mppi_config_.dynamics.maximum_vertical_speed_mps =
      static_cast<float>(declare_parameter<double>("maximum_vertical_speed_mps", 5.0));
  constrained_route_control_config_.maximum_vertical_acceleration_mps2 =
      mppi_config_.dynamics.maximum_vertical_acceleration_mps2;
  constrained_route_control_config_.maximum_vertical_speed_mps =
      mppi_config_.dynamics.maximum_vertical_speed_mps;
  constrained_route_control_config_.alignment_distance_buffer_m =
      declare_parameter<double>("constrained_route_alignment_distance_buffer_m", 5.0);
  constrained_route_control_config_.stationary_hold_distance_m =
      declare_parameter<double>("constrained_route_stationary_hold_distance_m", 2.0);
  constrained_route_control_config_.vertical_capture_margin_m =
      declare_parameter<double>("constrained_route_vertical_capture_margin_m", 0.5);
  constrained_route_control_config_.vertical_capture_speed_mps =
      declare_parameter<double>("constrained_route_vertical_capture_speed_mps", 0.75);
  physical_footprint_config_.radius_m =
      declare_parameter<double>("physical_footprint_radius_m", 0.82);
  physical_footprint_config_.lower_extent_m =
      declare_parameter<double>("physical_footprint_lower_extent_m", 0.23);
  physical_footprint_config_.upper_extent_m =
      declare_parameter<double>("physical_footprint_upper_extent_m", 0.35);
  physical_footprint_config_.perimeter_samples = static_cast<std::size_t>(
      declare_parameter<std::int64_t>("physical_footprint_samples", 12));
  physical_footprint_config_.radial_rings = static_cast<std::size_t>(
      declare_parameter<std::int64_t>("physical_footprint_radial_rings", 2));
  physical_footprint_config_.axial_samples = static_cast<std::size_t>(
      declare_parameter<std::int64_t>("physical_footprint_axial_samples", 3));
  const double static_route_tracking_margin_m =
      declare_parameter<double>("static_route_tracking_margin_m", 0.75);
  if (!(static_route_tracking_margin_m >= 0.0)) {
    throw std::invalid_argument{"static route tracking margin must be non-negative"};
  }
  mppi_config_.footprint = mppi::FootprintConfig{
      .radius_m = static_cast<float>(physical_footprint_config_.radius_m),
      .lower_extent_m = static_cast<float>(physical_footprint_config_.lower_extent_m),
      .upper_extent_m = static_cast<float>(physical_footprint_config_.upper_extent_m),
      .perimeter_samples =
          static_cast<std::uint32_t>(physical_footprint_config_.perimeter_samples),
      .radial_rings =
          static_cast<std::uint32_t>(physical_footprint_config_.radial_rings),
      .axial_samples =
          static_cast<std::uint32_t>(physical_footprint_config_.axial_samples),
  };
  mppi_config_.footprint.clearance_broad_phase_enabled =
      declare_parameter<bool>("mppi_footprint_clearance_broad_phase_enabled", true);
  mppi_config_.costs.head_progress_horizon_s =
      static_cast<float>(declare_parameter<double>("head_progress_horizon_s", 0.4));
  mppi_config_.costs.head_progress_weight =
      static_cast<float>(declare_parameter<double>("head_progress_weight", 8.0));
  mppi_config_.costs.planning_exposure_weight =
      static_cast<float>(declare_parameter<double>("planning_exposure_weight", 2.0));
  mppi_config_.costs.critical_exposure_weight =
      static_cast<float>(declare_parameter<double>("critical_exposure_weight", 20.0));
  mppi_config_.costs.critical_clearance_proximity_weight = static_cast<float>(
      declare_parameter<double>("critical_clearance_proximity_weight", 400.0));
  mppi_config_.costs.obstacle_approach_weight =
      static_cast<float>(declare_parameter<double>("obstacle_approach_weight", 40.0));
  mppi_config_.horizon_sampling.full_rate_duration_s = static_cast<float>(
      declare_parameter<double>("far_horizon_full_rate_duration_s", 2.0));
  mppi_config_.horizon_sampling.far_cost_stride = static_cast<std::uint32_t>(
      declare_parameter<std::int64_t>("far_horizon_cost_stride", 2));
  const double static_speed_tracking_weight =
      declare_parameter<double>("static_speed_tracking_weight", 1.0);
  const double no_static_speed_tracking_weight =
      declare_parameter<double>("no_static_speed_tracking_weight", 1.0);
  mppi_config_.costs.speed_tracking_weight = static_cast<float>(
      use_static_map_ ? static_speed_tracking_weight : no_static_speed_tracking_weight);
  mppi_config_.risk.critical_distance_m =
      static_cast<float>(declare_parameter<double>("critical_distance_m", 1.0));
  mppi_config_.risk.preferred_distance_m =
      static_cast<float>(declare_parameter<double>("preferred_distance_m", 6.0));
  mppi_config_.risk.obstacle_approach_response_time_s = static_cast<float>(
      declare_parameter<double>("obstacle_approach_response_time_s", 0.25));
  mppi_config_.risk.obstacle_approach_deceleration_mps2 = static_cast<float>(
      declare_parameter<double>("obstacle_approach_deceleration_mps2", 4.0));
  mppi_config_.seed = static_cast<std::uint64_t>(declare_parameter<int>("seed", 42));
  lattice_config_.heading_bins = static_cast<int>(
      declare_parameter<std::int64_t>("global_lattice_heading_bins", 16));
  lattice_config_.primitive_length_m =
      declare_parameter<double>("global_lattice_primitive_length_m", 4.0);
  lattice_config_.short_primitive_length_m =
      declare_parameter<double>("global_lattice_short_primitive_length_m", 2.0);
  const double static_lattice_distance =
      declare_parameter<double>("static_global_lattice_window_m", 180.0);
  const double no_static_lattice_distance =
      declare_parameter<double>("no_static_global_lattice_window_m", 60.0);
  lattice_config_.receding_goal_distance_m =
      use_static_map_ ? static_lattice_distance : no_static_lattice_distance;
  const std::int64_t static_lattice_expansions = declare_parameter<std::int64_t>(
      "static_global_lattice_maximum_expansions", 120000);
  const std::int64_t no_static_lattice_expansions = declare_parameter<std::int64_t>(
      "no_static_global_lattice_maximum_expansions", 60000);
  lattice_config_.maximum_expansions = static_cast<std::size_t>(
      use_static_map_ ? static_lattice_expansions : no_static_lattice_expansions);
  const double static_lattice_deadline_ms =
      declare_parameter<double>("static_global_lattice_deadline_ms", 250.0);
  const double no_static_lattice_deadline_ms =
      declare_parameter<double>("no_static_global_lattice_deadline_ms", 100.0);
  lattice_config_.maximum_search_time_ms =
      use_static_map_ ? static_lattice_deadline_ms : no_static_lattice_deadline_ms;
  const double static_lattice_roi_halo_m =
      declare_parameter<double>("static_global_lattice_roi_halo_m", 90.0);
  const double no_static_lattice_roi_halo_m =
      declare_parameter<double>("no_static_global_lattice_roi_halo_m", 45.0);
  lattice_config_.maximum_search_roi_halo_m =
      use_static_map_ ? static_lattice_roi_halo_m : no_static_lattice_roi_halo_m;
  lattice_config_.maximum_frontier_candidates = static_cast<std::size_t>(
      declare_parameter<std::int64_t>("global_lattice_frontier_candidates", 64));
  lattice_config_.minimum_frontier_endpoint_displacement_m = declare_parameter<double>(
      "global_lattice_frontier_minimum_endpoint_displacement_m", 4.0);
  lattice_config_.minimum_frontier_reachable_depth_m =
      declare_parameter<double>("global_lattice_frontier_reachable_depth_m", 20.0);
  const std::int64_t frontier_validation_maximum_states =
      declare_parameter<std::int64_t>(
          "global_lattice_frontier_validation_maximum_states", 2048);
  if (frontier_validation_maximum_states <= 0) {
    throw std::invalid_argument{
        "global lattice frontier validation maximum states must be positive"};
  }
  lattice_config_.frontier_validation_maximum_states =
      static_cast<std::size_t>(frontier_validation_maximum_states);
  const std::int64_t frontier_validation_expansion_interval =
      declare_parameter<std::int64_t>(
          "global_lattice_frontier_validation_expansion_interval", 256);
  if (frontier_validation_expansion_interval <= 0) {
    throw std::invalid_argument{
        "global lattice frontier validation expansion interval must be positive"};
  }
  lattice_config_.frontier_validation_expansion_interval =
      static_cast<std::size_t>(frontier_validation_expansion_interval);
  lattice_config_.frontier_goal_distance_weight =
      declare_parameter<double>("global_lattice_frontier_goal_distance_weight", 0.25);
  frontier_blacklist_enabled_ =
      declare_parameter<bool>("global_lattice_frontier_blacklist_enabled", false);
  lattice_config_.frontier_blacklist_radius_m =
      declare_parameter<double>("global_lattice_frontier_blacklist_radius_m", 6.0);
  lattice_config_.frontier_blacklist_heading_tolerance_bins =
      static_cast<int>(declare_parameter<std::int64_t>(
          "global_lattice_frontier_blacklist_heading_bins", 1));
  frontier_blacklist_ttl_s_ =
      declare_parameter<double>("global_lattice_frontier_blacklist_ttl_s", 15.0);
  no_static_soft_tabu_penalty_ =
      declare_parameter<double>("global_lattice_soft_tabu_penalty", 40.0);
  no_static_soft_tabu_sample_spacing_m_ =
      declare_parameter<double>("global_lattice_soft_tabu_sample_spacing_m", 4.0);
  no_static_adaptive_reachable_depth_m_ =
      declare_parameter<double>("global_lattice_adaptive_reachable_depth_m", 40.0);
  no_static_adaptive_minimum_guide_length_m_ =
      declare_parameter<double>("global_lattice_adaptive_minimum_guide_length_m", 24.0);
  no_static_adaptive_minimum_endpoint_displacement_m_ = declare_parameter<double>(
      "global_lattice_adaptive_minimum_endpoint_displacement_m", 12.0);
  no_static_adaptive_validation_states_ =
      static_cast<std::size_t>(declare_parameter<std::int64_t>(
          "global_lattice_adaptive_validation_states", 8192));
  no_static_cycle_config_.observation_window_s =
      declare_parameter<double>("global_lattice_cycle_observation_window_s", 20.0);
  no_static_cycle_config_.minimum_generation_changes = static_cast<std::size_t>(
      declare_parameter<std::int64_t>("global_lattice_cycle_minimum_generations", 6));
  no_static_cycle_config_.repeated_endpoint_radius_m =
      declare_parameter<double>("global_lattice_cycle_endpoint_radius_m", 6.0);
  no_static_cycle_config_.maximum_vehicle_displacement_m = declare_parameter<double>(
      "global_lattice_cycle_maximum_vehicle_displacement_m", 12.0);
  no_static_cycle_config_.maximum_mission_progress_m =
      declare_parameter<double>("global_lattice_cycle_maximum_mission_progress_m", 4.0);
  lattice_config_.planning_exposure_tie_break_per_m =
      declare_parameter<double>("global_lattice_planning_tie_break_per_m", 1.0);
  lattice_config_.critical_exposure_tie_break_per_m =
      declare_parameter<double>("global_lattice_critical_tie_break_per_m", 10.0);
  lattice_config_.critical_distance_m = mppi_config_.risk.critical_distance_m;
  lattice_config_.preferred_distance_m = mppi_config_.risk.preferred_distance_m;
  lattice_3d_config_.horizontal_step_m =
      declare_parameter<double>("global_lattice_3d_horizontal_step_m", 2.0);
  lattice_3d_config_.vertical_step_m =
      declare_parameter<double>("global_lattice_3d_vertical_step_m", 1.0);
  lattice_3d_config_.sample_step_m =
      declare_parameter<double>("global_lattice_3d_sample_step_m", 0.5);
  lattice_3d_config_.flight_envelope = flight_envelope_config_;
  lattice_3d_config_.planning_goal_distance_m = static_lattice_distance;
  lattice_3d_config_.critical_distance_m = mppi_config_.risk.critical_distance_m;
  lattice_3d_config_.preferred_distance_m = mppi_config_.risk.preferred_distance_m;
  lattice_3d_config_.nominal_horizontal_speed_mps =
      speed_policy_config_.cruise_speed_mps;
  lattice_3d_config_.nominal_vertical_speed_mps =
      declare_parameter<double>("global_lattice_3d_nominal_vertical_speed_mps", 4.0);
  lattice_3d_config_.vertical_alignment_cost_weight = declare_parameter<double>(
      "global_lattice_3d_vertical_alignment_cost_weight", 0.0);
  lattice_3d_config_.route_shape_turn_cost_per_rad = declare_parameter<double>(
      "global_lattice_3d_route_shape_turn_cost_per_rad", 0.10);
  lattice_3d_config_.passage_topology_transition_cost = declare_parameter<double>(
      "global_lattice_3d_passage_topology_transition_cost", 0.0);
  static_route_geometry_config_.sample_step_m = lattice_3d_config_.sample_step_m;
  static_route_geometry_config_.maximum_shortcut_length_m =
      declare_parameter<double>("static_route_maximum_shortcut_length_m", 30.0);
  static_route_geometry_config_.corner_smoothing_distance_m =
      declare_parameter<double>("static_route_corner_smoothing_distance_m", 2.0);
  static_route_geometry_config_.corner_curve_samples = static_cast<std::size_t>(
      declare_parameter<std::int64_t>("static_route_corner_curve_samples", 4));
  lattice_3d_config_.planning_exposure_cost_per_m =
      declare_parameter<double>("global_lattice_3d_planning_exposure_cost_per_m", 0.05);
  lattice_3d_config_.critical_exposure_cost_per_m =
      declare_parameter<double>("global_lattice_3d_critical_exposure_cost_per_m", 0.50);
  lattice_3d_config_.passage_connection_distance_m =
      declare_parameter<double>("global_lattice_3d_passage_connection_distance_m", 3.0);
  const std::int64_t maximum_entry_portals_per_component =
      declare_parameter<std::int64_t>(
          "free_space_topology_maximum_entry_portals_per_component", 4);
  const std::int64_t maximum_traversals_per_component = declare_parameter<std::int64_t>(
      "free_space_topology_maximum_traversals_per_component", 3);
  if (maximum_entry_portals_per_component <= 0 ||
      maximum_traversals_per_component <= 0) {
    throw std::invalid_argument{"invalid free-space topology router configuration"};
  }
  free_space_topology_router_config_.maximum_entry_portals_per_component =
      static_cast<std::size_t>(maximum_entry_portals_per_component);
  free_space_topology_router_config_.maximum_traversals_per_component =
      static_cast<std::size_t>(maximum_traversals_per_component);
  lattice_3d_config_.frontier_minimum_reachable_depth_m =
      declare_parameter<double>("global_lattice_3d_frontier_reachable_depth_m", 8.0);
  lattice_3d_config_.frontier_validation_maximum_states =
      static_cast<std::size_t>(declare_parameter<std::int64_t>(
          "global_lattice_3d_frontier_validation_maximum_states", 2048));
  lattice_3d_config_.maximum_expansions =
      static_cast<std::size_t>(static_lattice_expansions);
  lattice_3d_config_.maximum_search_time_ms = static_lattice_deadline_ms;
  active_guide_config_.critical_distance_m = mppi_config_.risk.critical_distance_m;
  active_guide_config_.preferred_distance_m = mppi_config_.risk.preferred_distance_m;
  active_guide_config_.validation_sample_step_m =
      declare_parameter<double>("global_guide_validation_sample_step_m", 0.5);
  const double static_guide_replan_remaining_m =
      declare_parameter<double>("static_global_guide_replan_remaining_m", 45.0);
  const double no_static_guide_replan_remaining_m =
      declare_parameter<double>("no_static_global_guide_replan_remaining_m", 15.0);
  active_guide_config_.minimum_remaining_m = use_static_map_
                                                 ? static_guide_replan_remaining_m
                                                 : no_static_guide_replan_remaining_m;
  static_route_extension_config_.minimum_remaining_m = static_guide_replan_remaining_m;
  static_route_extension_config_.latency_margin_s =
      declare_parameter<double>("static_global_guide_extension_latency_margin_s", 0.5);
  static_route_extension_config_.maximum_latency_s =
      declare_parameter<double>("static_global_guide_extension_maximum_latency_s", 8.0);
  static_route_extension_config_.minimum_retry_progress_m =
      declare_parameter<double>("static_global_guide_extension_retry_progress_m", 15.0);
  static_route_extension_config_.minimum_retry_interval_s =
      declare_parameter<double>("static_global_guide_extension_retry_interval_s", 1.0);
  static_route_extension_config_.minimum_endpoint_improvement_m =
      declare_parameter<double>(
          "static_global_guide_extension_minimum_endpoint_improvement_m", 5.0);
  static_route_search_retry_config_.minimum_pose_change_m =
      declare_parameter<double>("static_global_guide_failed_search_pose_change_m", 2.0);
  static_route_search_retry_config_.minimum_objective_change_m =
      declare_parameter<double>("static_global_guide_failed_search_objective_change_m",
                                5.0);
  static_route_search_retry_config_.minimum_retry_interval_s =
      declare_parameter<double>("static_global_guide_failed_search_retry_interval_s",
                                1.0);
  if (!(static_route_search_retry_config_.minimum_pose_change_m > 0.0) ||
      !(static_route_search_retry_config_.minimum_objective_change_m > 0.0) ||
      !(static_route_search_retry_config_.minimum_retry_interval_s > 0.0)) {
    throw std::invalid_argument{"invalid static route search retry configuration"};
  }
  active_guide_config_.maximum_cross_track_m =
      declare_parameter<double>("global_guide_maximum_cross_track_m", 15.0);
  active_guide_config_.velocity_heading_low_speed_mps =
      declare_parameter<double>("global_guide_heading_low_speed_mps", 0.5);
  active_guide_config_.velocity_heading_high_speed_mps =
      declare_parameter<double>("global_guide_heading_high_speed_mps", 1.5);
  guide_progress_config_.observation_window_s =
      declare_parameter<double>("global_guide_stall_observation_window_s", 1.0);
  guide_progress_config_.minimum_progress_m =
      declare_parameter<double>("global_guide_stall_minimum_progress_m", 0.5);
  guide_progress_config_.minimum_predicted_head_progress_m = declare_parameter<double>(
      "global_guide_stall_minimum_predicted_head_progress_m", 0.5);
  mppi_config_.early_exit_on_collision = true;
  physical_footprint_config_.sweep_step_m =
      declare_parameter<double>("physical_footprint_sweep_step_m", 0.25);
  lattice_config_.physical_footprint_radius_m = physical_footprint_config_.radius_m;
  lattice_config_.physical_footprint_lower_extent_m =
      physical_footprint_config_.lower_extent_m;
  lattice_config_.physical_footprint_upper_extent_m =
      physical_footprint_config_.upper_extent_m;
  lattice_config_.physical_footprint_samples =
      physical_footprint_config_.perimeter_samples;
  lattice_config_.physical_footprint_radial_rings =
      physical_footprint_config_.radial_rings;
  lattice_config_.physical_footprint_axial_samples =
      physical_footprint_config_.axial_samples;
  lattice_3d_config_.physical_footprint_radius_m =
      physical_footprint_config_.radius_m +
      (use_static_map_ ? static_route_tracking_margin_m : 0.0);
  lattice_3d_config_.physical_footprint_lower_extent_m =
      physical_footprint_config_.lower_extent_m +
      (use_static_map_ ? static_route_tracking_margin_m : 0.0);
  lattice_3d_config_.physical_footprint_upper_extent_m =
      physical_footprint_config_.upper_extent_m +
      (use_static_map_ ? static_route_tracking_margin_m : 0.0);
  lattice_3d_config_.physical_footprint_samples =
      physical_footprint_config_.perimeter_samples;
  lattice_3d_config_.physical_footprint_radial_rings =
      physical_footprint_config_.radial_rings;
  lattice_3d_config_.physical_footprint_axial_samples =
      physical_footprint_config_.axial_samples;
  lattice_3d_config_.physical_footprint_sweep_step_m =
      physical_footprint_config_.sweep_step_m;
  configureCooperativeTraffic();
  configureNonCooperativeAvoidance();
  liveness_config_.enabled = declare_parameter<bool>("liveness_enabled", true);
  liveness_config_.observation_window_s =
      declare_parameter<double>("liveness_observation_window_s", 1.0);
  liveness_config_.minimum_actual_displacement_m =
      declare_parameter<double>("liveness_minimum_actual_displacement_m", 0.5);
  liveness_config_.minimum_predicted_terminal_progress_m =
      declare_parameter<double>("liveness_minimum_predicted_terminal_progress_m", 5.0);
  rviz_period_ns_ = static_cast<std::int64_t>(1.0e9 / std::max(0.1, rviz_rate_hz_));
  diagnostics_info_period_ns_ =
      static_cast<std::int64_t>(1.0e9 / std::max(0.1, diagnostics_info_rate_hz_));
  diagnostics_file_period_ns_ =
      static_cast<std::int64_t>(1.0e9 / std::max(0.1, diagnostics_file_rate_hz_));

  if (!(tick_rate_hz_ > 0.0) || !(rviz_rate_hz_ > 0.0) ||
      !(diagnostics_info_rate_hz_ > 0.0) || !(deadline_ms_ > 0.0) ||
      !(diagnostics_file_rate_hz_ > 0.0) || !(diagnostics_flush_period_s_ > 0.0) ||
      !(maximum_control_feedback_age_ms_ > 0.0) ||
      !(latest_lidar_obstacle_maximum_age_ms_ > 0.0) ||
      !std::isfinite(constrained_route_speed_limit_mps_) ||
      constrained_route_speed_limit_mps_ < 0.0F ||
      !(route_constraint_diagnostics_distance_m_ >= 0.0) ||
      !(lattice_3d_config_.nominal_horizontal_speed_mps > 0.0) ||
      !(lattice_3d_config_.nominal_vertical_speed_mps > 0.0) ||
      !(lattice_3d_config_.vertical_alignment_cost_weight >= 0.0) ||
      !(lattice_3d_config_.route_shape_turn_cost_per_rad >= 0.0) ||
      !(lattice_3d_config_.passage_topology_transition_cost >= 0.0) ||
      !(static_route_geometry_config_.maximum_shortcut_length_m > 0.0) ||
      !(static_route_geometry_config_.corner_smoothing_distance_m >= 0.0) ||
      static_route_geometry_config_.corner_curve_samples < 2U ||
      !(lattice_3d_config_.planning_exposure_cost_per_m >= 0.0) ||
      !(lattice_3d_config_.critical_exposure_cost_per_m >= 0.0) ||
      !(lattice_3d_config_.passage_connection_distance_m > 0.0) ||
      free_space_topology_router_config_.maximum_entry_portals_per_component == 0U ||
      free_space_topology_router_config_.maximum_traversals_per_component == 0U ||
      !(lattice_3d_config_.frontier_minimum_reachable_depth_m > 0.0) ||
      lattice_3d_config_.frontier_validation_maximum_states == 0U ||
      !(physical_footprint_config_.sweep_step_m > 0.0) ||
      !(physical_footprint_config_.radius_m >= 0.0) ||
      !(physical_footprint_config_.lower_extent_m >= 0.0) ||
      !(physical_footprint_config_.upper_extent_m >= 0.0) ||
      !(mppi_config_.costs.planning_exposure_weight >= 0.0F) ||
      !(mppi_config_.costs.critical_exposure_weight >= 0.0F) ||
      !(mppi_config_.costs.obstacle_approach_weight >= 0.0F) ||
      !(mppi_config_.risk.obstacle_approach_response_time_s >= 0.0F) ||
      !(mppi_config_.risk.obstacle_approach_deceleration_mps2 > 0.0F) ||
      !stoppingCapabilityIsValid(speed_policy_config_.stopping_capability) ||
      speed_policy_config_.stopping_capability
              .maximum_commanded_horizontal_deceleration_mps2 >
          static_cast<double>(
              mppi_config_.dynamics.maximum_horizontal_acceleration_mps2) ||
      finite_horizon_config_.stopping_capability
              .guaranteed_horizontal_deceleration_mps2 >
          static_cast<double>(
              mppi_config_.dynamics.maximum_horizontal_acceleration_mps2) ||
      finite_horizon_config_.stopping_capability.guaranteed_vertical_deceleration_mps2 >
          static_cast<double>(
              mppi_config_.dynamics.maximum_vertical_acceleration_mps2) ||
      physical_footprint_config_.perimeter_samples == 0U ||
      physical_footprint_config_.radial_rings == 0U ||
      physical_footprint_config_.axial_samples < 2U ||
      !(frontier_blacklist_ttl_s_ > 0.0) || !(no_static_soft_tabu_penalty_ >= 0.0) ||
      !(no_static_soft_tabu_sample_spacing_m_ > 0.0) ||
      !(no_static_adaptive_reachable_depth_m_ > 0.0) ||
      !(no_static_adaptive_minimum_guide_length_m_ > 0.0) ||
      !(no_static_adaptive_minimum_endpoint_displacement_m_ > 0.0) ||
      no_static_adaptive_validation_states_ == 0U ||
      !(no_static_esdf_update_rate_hz_ > 0.0) ||
      !(no_static_esdf_half_extent_m_ > 0.0) ||
      !(no_static_esdf_recenter_margin_m_ >= 0.0) ||
      no_static_esdf_recenter_margin_m_ >= no_static_esdf_half_extent_m_ ||
      no_static_cycle_config_.minimum_generation_changes < 2U) {
    throw std::invalid_argument{"invalid production MPPI configuration"};
  }
  const double planning_tick_period_s = 1.0 / tick_rate_hz_;
  if (!std::isfinite(planning_tick_phase_offset_s_) ||
      planning_tick_phase_offset_s_ < 0.0 ||
      planning_tick_phase_offset_s_ >= planning_tick_period_s) {
    throw std::invalid_argument{
        "planning tick phase offset must be in [0, tick period)"};
  }

  liveness_supervisor_ = std::make_unique<MppiLivenessSupervisor>(liveness_config_);
  active_guide_lifecycle_ =
      std::make_unique<ActiveGlobalGuideLifecycle>(active_guide_config_);
  guide_progress_tracker_ =
      std::make_unique<GlobalGuideProgressTracker>(guide_progress_config_);
  mission_goal_capture_latch_ =
      std::make_unique<MissionGoalCaptureLatch>(mission_goal_capture_config_);
  no_static_cycle_detector_ =
      std::make_unique<NoStaticRouteCycleDetector>(no_static_cycle_config_);
  planning_worker_pool_ = std::make_unique<BoundedWorkerPool>(planner_worker_count_);
  engine_ = std::make_unique<mppi::MppiCudaEngine>(mppi_config_);
  if (use_static_map_) {
    const auto package_share = std::filesystem::path{
        ament_index_cpp::get_package_share_directory("drone_city_nav")};
    std::filesystem::path occupancy_path = declare_parameter<std::string>(
        "static_occupancy_3d_path", "worlds/generated_city.occupancy3d");
    if (occupancy_path.is_relative()) {
      occupancy_path = package_share / occupancy_path;
    }
    static_occupancy_3d_ = OccupancyGrid3D::load(occupancy_path);
    std::filesystem::path cache_path = declare_parameter<std::string>(
        "static_esdf_3d_cache_path", "worlds/generated_city.esdf3d");
    if (cache_path.empty()) {
      cache_path = occupancy_path;
      cache_path.replace_extension(".esdf3d");
    } else if (cache_path.is_relative()) {
      cache_path = package_share / cache_path;
    }
    const double required_maximum_distance_m =
        static_cast<double>(mppi_config_.risk.preferred_distance_m) + 20.0;
    try {
      StaticEsdfCache cache = StaticEsdfCache::load(cache_path);
      if (cache.compatibleWith(*static_occupancy_3d_, required_maximum_distance_m)) {
        RCLCPP_INFO(get_logger(),
                    "STATIC_ESDF_CACHE_READY path=%s fingerprint=%" PRIu64
                    " maximum_distance_m=%.2f chunks=%zu bytes=%zu "
                    "shared_resource_reused=%s",
                    cache_path.c_str(), cache.occupancyFingerprint(),
                    cache.maximumDistanceM(), cache.storedChunkCount(),
                    cache.compressedBytes(),
                    cache.sharedResourceReused() ? "true" : "false");
        static_esdf_cache_ = std::move(cache);
      } else {
        RCLCPP_WARN(get_logger(),
                    "STATIC_ESDF_CACHE_FALLBACK path=%s reason=incompatible_world_or_"
                    "distance cache_fingerprint=%" PRIu64
                    " occupancy_fingerprint=%" PRIu64
                    " cache_maximum_distance_m=%.2f requested_maximum_distance_m=%.2f",
                    cache_path.c_str(), cache.occupancyFingerprint(),
                    static_occupancy_3d_->fingerprint(), cache.maximumDistanceM(),
                    required_maximum_distance_m);
      }
    } catch (const std::exception& error) {
      RCLCPP_WARN(get_logger(),
                  "STATIC_ESDF_CACHE_FALLBACK path=%s reason=load_failed error=%s",
                  cache_path.c_str(), error.what());
    }
    std::filesystem::path topology_path =
        declare_parameter<std::string>("static_free_space_topology_3d_path", "");
    if (topology_path.empty()) {
      topology_path = occupancy_path;
      topology_path.replace_extension(".topology3d");
    } else if (topology_path.is_relative()) {
      topology_path = package_share / topology_path;
    }
    try {
      FreeSpaceTopology3D topology = FreeSpaceTopology3D::load(topology_path);
      if (topology.compatibleWith(*static_occupancy_3d_)) {
        static_free_space_topology_3d_ = std::move(topology);
      } else {
        RCLCPP_WARN(get_logger(),
                    "FREE_SPACE_TOPOLOGY_FALLBACK path=%s reason=incompatible_world "
                    "topology_fingerprint=%" PRIu64 " occupancy_fingerprint=%" PRIu64,
                    topology_path.c_str(), topology.occupancyFingerprint(),
                    static_occupancy_3d_->fingerprint());
      }
    } catch (const std::exception& error) {
      RCLCPP_WARN(get_logger(),
                  "FREE_SPACE_TOPOLOGY_FALLBACK path=%s reason=load_failed error=%s",
                  topology_path.c_str(), error.what());
    }
    const std::span<const PassageTraversalEdge> passage_traversals =
        static_free_space_topology_3d_
            ? std::span<const PassageTraversalEdge>{static_free_space_topology_3d_
                                                        ->traversalEdges()}
            : std::span<const PassageTraversalEdge>{};
    static_portal_edges_ = std::make_shared<const std::vector<PassageTraversalEdge>>(
        passage_traversals.begin(), passage_traversals.end());
    if (static_free_space_topology_3d_ &&
        !static_free_space_topology_3d_->segments().empty()) {
      static_free_space_topology_router_ = std::make_unique<FreeSpaceTopologyRouter>(
          *static_free_space_topology_3d_, free_space_topology_router_config_);
    }
    RCLCPP_INFO(get_logger(),
                "STATIC_WORLD_3D path=%s fingerprint=%" PRIu64
                " occupied_voxels=%zu passage_regions=%zu portals=%zu "
                "passage_segments=%zu portal_edges=%zu topology_path=%s "
                "topology_ready=%s topology_mode=%s "
                "dimensions=%dx%dx%d",
                occupancy_path.c_str(), static_occupancy_3d_->fingerprint(),
                static_occupancy_3d_->occupiedVoxelCount(),
                static_free_space_topology_3d_
                    ? static_free_space_topology_3d_->regions().size()
                    : 0U,
                static_free_space_topology_3d_
                    ? static_free_space_topology_3d_->portals().size()
                    : 0U,
                static_free_space_topology_3d_
                    ? static_free_space_topology_3d_->segments().size()
                    : 0U,
                static_portal_edges_->size(), topology_path.c_str(),
                static_free_space_topology_3d_ ? "true" : "false",
                static_free_space_topology_router_ ? "sparse_lazy" : "legacy_eager",
                static_occupancy_3d_->bounds().width_cells,
                static_occupancy_3d_->bounds().height_cells,
                static_occupancy_3d_->bounds().depth_cells);
  }
  std::filesystem::create_directories(diagnostics_output_dir_);
  diagnostics_stream_.open(diagnostics_output_dir_ / "mppi_ticks.jsonl",
                           std::ios::trunc);
  diagnostics_error_stream_.open(diagnostics_output_dir_ / "mppi_error_context.jsonl",
                                 std::ios::trunc);
  last_diagnostics_flush_time_ = std::chrono::steady_clock::now();
  input_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  planning_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions input_subscription_options;
  input_subscription_options.callback_group = input_callback_group_;
  const auto sensor_qos = rclcpp::SensorDataQoS{};
  local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      declare_parameter<std::string>("px4_local_position_topic",
                                     "/fmu/out/vehicle_local_position_v1"),
      sensor_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr message) {
        onLocalPosition(*message);
      },
      input_subscription_options);
  navigation_readiness_sub_ = create_subscription<std_msgs::msg::Bool>(
      declare_parameter<std::string>("navigation_readiness_topic",
                                     "/drone_city_nav/navigation_ready"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        onNavigationReadiness(*message);
      },
      input_subscription_options);
  raw_snapshot_sub_ = create_subscription<msg::RawObstacleSnapshot>(
      declare_parameter<std::string>("raw_obstacle_snapshot_topic",
                                     "/drone_city_nav/raw_obstacle_snapshot"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](msg::RawObstacleSnapshot::ConstSharedPtr message) {
        onRawObstacleSnapshot(std::move(message));
      },
      input_subscription_options);
  raw_delta_sub_ = create_subscription<msg::RawObstacleDelta>(
      declare_parameter<std::string>("raw_obstacle_delta_topic",
                                     "/drone_city_nav/raw_obstacle_delta"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](msg::RawObstacleDelta::ConstSharedPtr message) {
        onRawObstacleDelta(std::move(message));
      },
      input_subscription_options);
  latest_lidar_obstacle_scan_sub_ = create_subscription<msg::LatestLidarObstacleScan>(
      declare_parameter<std::string>("latest_lidar_obstacle_scan_topic",
                                     "/drone_city_nav/latest_lidar_obstacle_scan"),
      rclcpp::SensorDataQoS{},
      [this](const msg::LatestLidarObstacleScan::SharedPtr message) {
        onLatestLidarObstacleScan(*message);
      },
      input_subscription_options);
  memory_status_sub_ = create_subscription<msg::ObstacleMemoryStatus>(
      declare_parameter<std::string>("obstacle_memory_status_topic",
                                     "/drone_city_nav/obstacle_memory_status"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](const msg::ObstacleMemoryStatus::SharedPtr message) {
        onMemoryStatus(*message);
      },
      input_subscription_options);
  applied_control_sub_ = create_subscription<msg::MppiControlFeedback>(
      declare_parameter<std::string>("applied_control_feedback_topic",
                                     "/drone_city_nav/mppi/applied_control"),
      rclcpp::QoS{10}.reliable(),
      [this](const msg::MppiControlFeedback::SharedPtr message) {
        onAppliedControl(*message);
      },
      input_subscription_options);
  navigation_objective_sub_ = create_subscription<msg::NavigationObjective>(
      declare_parameter<std::string>("navigation_objective_topic",
                                     "/drone_city_nav/navigation_objective"),
      rclcpp::QoS{1}.reliable().transient_local(),
      [this](const msg::NavigationObjective::SharedPtr message) {
        onNavigationObjective(*message);
      },
      input_subscription_options);
  createCooperativeTrafficInterfaces(input_subscription_options);
  createNonCooperativeAvoidanceInterface(input_subscription_options);
  radar_track_mode_command_pub_ = create_publisher<msg::RadarTrackModeCommand>(
      declare_parameter<std::string>("radar_track_mode_command_topic",
                                     "/drone_city_nav/radar/track_mode_command"),
      rclcpp::QoS{1}.reliable().transient_local());
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
      declare_parameter<std::string>("path_topic", "/drone_city_nav/mppi/path"),
      rclcpp::QoS{1}.reliable());
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      declare_parameter<std::string>("markers_topic", "/drone_city_nav/mppi/markers"),
      rclcpp::QoS{1}.reliable());
  status_pub_ = create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>("status_topic", "/drone_city_nav/mppi/status"),
      rclcpp::QoS{10}.best_effort());
  world_readiness_pub_ = create_publisher<std_msgs::msg::Bool>(
      declare_parameter<std::string>("world_readiness_topic",
                                     "/drone_city_nav/mppi/world_ready"),
      rclcpp::QoS{1}.reliable().transient_local());
  execution_horizon_pub_ = create_publisher<msg::MppiTrajectoryHorizon>(
      declare_parameter<std::string>("execution_horizon_topic",
                                     "/drone_city_nav/mppi/execution_horizon"),
      rclcpp::QoS{2}.reliable());
  publishWorldReadiness(false);
  diagnostics_worker_ =
      std::jthread([this](const std::stop_token token) { diagnosticsWorker(token); });
  esdf_worker_ =
      std::jthread([this](const std::stop_token token) { esdfWorker(token); });
  guide_worker_ =
      std::jthread([this](const std::stop_token token) { guideWorker(token); });
  if (planning_tick_phase_offset_s_ > 0.0) {
    planning_start_timer_ = create_wall_timer(
        std::chrono::duration<double>{planning_tick_phase_offset_s_},
        [this]() {
          planning_start_timer_->cancel();
          startPlanningTimer();
        },
        planning_callback_group_);
  } else {
    startPlanningTimer();
  }
  RCLCPP_INFO(
      get_logger(),
      "Production MPPI ready: rollouts=%zu open_static_rollouts=%zu "
      "direct_tracking_rollouts=%zu adaptive_clearance_m=%.1f "
      "steps=%zu rate=%.1fHz "
      "deadline=%.1fms known_solids=%zu static_map=%s route3d=%s "
      "horizon=%.1fs guide_window=%.1fm cruise=%.1fmps speed_cap=%.1fmps "
      "acceleration_cap=%.1fmps2 jerk_cap=%.1fmps3 speed_tracking_weight=%.2f "
      "constrained_route_speed_limit=%.1fmps head_progress=%.2fs "
      "far_cost_sampling=(%.2fs,%u) liveness=%s "
      "sticky_guide=true frontier_blacklist=%s guide_replan_remaining=%.1fm "
      "guide_heading_blend=(%.1f,%.1f)mps planner_workers=%zu "
      "planner_tick_phase_ms=%.1f no_static_esdf=(%.1fHz,%.1fm,%.1fm)",
      mppi_config_.rollouts, rollout_budget_config_.open_static_rollouts,
      rollout_budget_config_.direct_tracking_rollouts,
      rollout_budget_config_.minimum_reduced_clearance_m, mppi_config_.steps,
      tick_rate_hz_, deadline_ms_, 0UL, use_static_map_ ? "true" : "false", "false",
      static_cast<double>(mppi_config_.steps) * mppi_config_.dynamics.dt_s,
      lattice_config_.receding_goal_distance_m, speed_policy_config_.cruise_speed_mps,
      mppi_config_.dynamics.maximum_horizontal_speed_mps,
      mppi_config_.dynamics.maximum_horizontal_acceleration_mps2,
      mppi_config_.dynamics.maximum_control_jerk_mps3,
      mppi_config_.costs.speed_tracking_weight,
      use_static_map_ ? constrained_route_speed_limit_mps_ : 0.0F,
      mppi_config_.costs.head_progress_horizon_s,
      mppi_config_.horizon_sampling.full_rate_duration_s,
      mppi_config_.horizon_sampling.far_cost_stride,
      liveness_config_.enabled ? "true" : "false",
      frontier_blacklist_enabled_ ? "true" : "false",
      active_guide_config_.minimum_remaining_m,
      active_guide_config_.velocity_heading_low_speed_mps,
      active_guide_config_.velocity_heading_high_speed_mps, planner_worker_count_,
      planning_tick_phase_offset_s_ * 1000.0, no_static_esdf_update_rate_hz_,
      no_static_esdf_half_extent_m_, no_static_esdf_recenter_margin_m_);
}

void ProductionMppiNode::startPlanningTimer() {
  planning_timer_ = create_wall_timer(
      std::chrono::duration<double>{1.0 / tick_rate_hz_}, [this]() { planningTick(); },
      planning_callback_group_);
}

ProductionMppiNode::~ProductionMppiNode() {
  if (diagnostics_worker_.joinable()) {
    diagnostics_worker_.request_stop();
    diagnostics_mailbox_.notifyAll();
    diagnostics_worker_.join();
  }
  if (esdf_worker_.joinable()) {
    esdf_worker_.request_stop();
    raw_queue_condition_.notify_all();
    esdf_worker_.join();
  }
  if (guide_worker_.joinable()) {
    guide_worker_.request_stop();
    guide_queue_condition_.notify_all();
    guide_worker_.join();
  }
  publishSummary();
}

} // namespace drone_city_nav

RCLCPP_COMPONENTS_REGISTER_NODE(drone_city_nav::ProductionMppiNode)
