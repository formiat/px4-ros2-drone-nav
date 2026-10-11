#include "drone_city_nav/mission_waypoint_sequence.hpp"
#include "drone_city_nav/sensor_braking_contract_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/trajectory_compiler_3d.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "production_mppi_config_ros.hpp"
#include "production_mppi_config_ros_loader.hpp"

namespace drone_city_nav::production_mppi_config_ros_detail {

void ProductionMppiConfigLoader::declarePlanning() {
  ProductionMppiConfig::Planning& planning = config_.planning;
  planning.tick_rate_hz = declare<double>("tick_rate_hz", 50.0);
  planning.deadline_ms = declare<double>("deadline_ms", 20.0);
  planning.planning_tick_phase_offset_s =
      declare<double>("planning_tick_phase_offset_s", 0.0);
  const std::int64_t planner_worker_count =
      declare<std::int64_t>("planner_worker_count", 4);
  if (planner_worker_count < 1 || planner_worker_count > 8) {
    throw std::invalid_argument{"planner worker count must be in [1, 8]"};
  }
  planning.planner_worker_count = static_cast<std::size_t>(planner_worker_count);
  planning.mission_start =
      Point3{declare<double>("start_x_m", 54.0), declare<double>("start_y_m", 54.0),
             declare<double>("start_z_m", 0.0)};
  planning.mission_goal_capture.capture_radius_m =
      declare<double>("mission_goal_capture_radius_m", 2.0);
  planning.mission_waypoint_sequence.goal_radius_m =
      planning.mission_goal_capture.capture_radius_m;
  planning.mission_waypoint_sequence.stop_speed_mps =
      declare<double>("mission_waypoint_stop_speed_mps", 0.8);
  planning.mission_waypoint_sequence.stop_hold_s =
      declare<double>("mission_waypoint_hold_s", 2.0);
  planning.mission_waypoints = missionWaypointsFromFlatParameters(
      declare<std::vector<double>>("mission_goal_sequence_xyz_m", {}));
  planning.configured_mission_objective_enabled =
      declare<bool>("configured_mission_objective_enabled", false);
  planning.dynamic_objective_replan_distance_m =
      declare<double>("dynamic_objective_replan_distance_m", 2.0);
  planning.dynamic_objective_replan_period_s =
      declare<double>("dynamic_objective_replan_period_s", 0.25);
  planning.tracking_objective_ray_sample_spacing_m =
      declare<double>("tracking_objective_ray_sample_spacing_m", 0.25);
  planning.static_tracking_esdf_refresh_margin_m =
      declare<double>("static_tracking_esdf_refresh_margin_m", 15.0);

  planning.optional_constraints = ProductionNavigationOptionalConstraints{
      .clearance_costs_enabled = declare<bool>("clearance_costs_enabled", false),
      .static_route_geometry_optimization_enabled =
          declare<bool>("static_route_geometry_optimization_enabled", false),
      .static_route_shortcut_optimization_enabled =
          declare<bool>("static_route_shortcut_optimization_enabled", false),
      .stochastic_trajectory_selection_enabled =
          declare<bool>("stochastic_trajectory_selection_enabled", false),
      .route_cross_track_constraints_enabled =
          declare<bool>("execution_route_cross_track_constraints_enabled", false),
      .route_tracking_tube_constraints_enabled =
          declare<bool>("execution_route_tracking_tube_constraints_enabled", false),
      .route_progress_replan_enabled =
          declare<bool>("route_progress_replan_enabled", false),
      .no_eligible_route_replan_enabled =
          declare<bool>("no_eligible_route_replan_enabled", false),
      .latest_sensor_freshness_required =
          declare<bool>("execution_latest_sensor_freshness_required", false),
      .nonphysical_execution_revocation_enabled =
          declare<bool>("execution_nonphysical_revocation_enabled", false),
  };
  planning.constrained_route_speed_limit_mps =
      static_cast<float>(declare<double>("constrained_route_speed_limit_mps", 10.0));

  PersistentPlannerConfig3D& planner = planning.persistent_planner;
  planner.minimum_horizontal_step_m =
      declare<double>("persistent_planner_minimum_horizontal_step_m", 2.0);
  planner.minimum_vertical_step_m =
      declare<double>("persistent_planner_minimum_vertical_step_m", 1.0);
  const std::int64_t maximum_adaptive_lattice_level =
      declare<std::int64_t>("persistent_planner_maximum_adaptive_lattice_level", 2);
  if (maximum_adaptive_lattice_level < 0 || maximum_adaptive_lattice_level > 10) {
    throw std::invalid_argument{
        "persistent_planner_maximum_adaptive_lattice_level must be in [0, 10]"};
  }
  planner.maximum_adaptive_lattice_level =
      static_cast<std::size_t>(maximum_adaptive_lattice_level);
  planner.goal_tolerance_m =
      declare<double>("persistent_planner_goal_tolerance_m", 2.0);
  planner.feasibility_goal_connector_reach_m =
      declare<double>("persistent_planner_feasibility_goal_connector_reach_m", 40.0);
  planner.feasibility_clearance_ranking_distance_m = declare<double>(
      "persistent_planner_feasibility_clearance_ranking_distance_m", 2.0);
  const std::int64_t connector_search_radius_cells =
      declare<std::int64_t>("persistent_planner_connector_search_radius_cells", 2);
  if (connector_search_radius_cells < 0 || connector_search_radius_cells > 32) {
    throw std::invalid_argument{
        "persistent_planner_connector_search_radius_cells must be in [0, 32]"};
  }
  planner.connector_search_radius_cells =
      static_cast<std::size_t>(connector_search_radius_cells);
  planner.feasibility_first_enabled =
      declare<bool>("persistent_planner_feasibility_first_enabled", true);
  planner.maximum_feasibility_expansions_per_update = declarePositiveSize(
      "persistent_planner_maximum_feasibility_expansions_per_update", 4'096);
  planner.maximum_feasibility_compute_time_ms =
      declare<double>("persistent_planner_maximum_feasibility_compute_time_ms", 50.0);
  planner.maximum_expansions_per_update =
      declarePositiveSize("persistent_planner_maximum_expansions_per_update", 200'000);
  planner.maximum_incremental_changed_voxels = declarePositiveSize(
      "persistent_planner_maximum_incremental_changed_voxels", 32'768);
  planner.maximum_extracted_path_nodes =
      declarePositiveSize("persistent_planner_maximum_extracted_path_nodes", 8'192);
  planner.maximum_shortcut_checks =
      declarePositiveSize("persistent_planner_maximum_shortcut_checks", 8'192);
  planner.guaranteed_spatial_search_fraction =
      declare<double>("persistent_planner_guaranteed_spatial_search_fraction", 0.34);
  planner.guaranteed_refinement_expansion_fraction = declare<double>(
      "persistent_planner_guaranteed_refinement_expansion_fraction", 0.34);
  planner.continuity_improvement_margin_s =
      declare<double>("persistent_planner_continuity_improvement_margin_s", 1.0);
  planner.edge_refinement_offsets =
      declarePositiveSize("persistent_planner_edge_refinement_offsets", 3);
  planner.maximum_edge_refinement_probes =
      declarePositiveSize("persistent_planner_maximum_edge_refinement_probes", 256);
  planner.departure_refinement_subdivisions =
      declarePositiveSize("persistent_planner_departure_refinement_subdivisions", 4);
  planner.maximum_departure_refinement_probes = declarePositiveSize(
      "persistent_planner_maximum_departure_refinement_probes", 512);
  planner.escape_search_radius_cells =
      declarePositiveSize("persistent_planner_escape_search_radius_cells", 6);
  planner.escape_search_maximum_probes_per_update = declarePositiveSize(
      "persistent_planner_escape_search_maximum_probes_per_update", 512);
  planner.clearance_centering_passes =
      declarePositiveSize("persistent_planner_clearance_centering_passes", 3);
  planner.maximum_clearance_centering_queries = declarePositiveSize(
      "persistent_planner_maximum_clearance_centering_queries", 4'096);
  planner.maximum_compute_time_ms =
      declare<double>("persistent_planner_maximum_compute_time_ms", 150.0);
  planner.maximum_no_route_compute_time_ms =
      declare<double>("persistent_planner_maximum_no_route_compute_time_ms", 60.0);
  planner.clearance_ranking_weight =
      declare<double>("persistent_planner_clearance_ranking_weight", 1.5);
  planner.clearance_ranking_distance_m =
      declare<double>("persistent_planner_clearance_ranking_distance_m", 6.0);

  planning.route_sampling_step_m = declare<double>("route_sampling_step_m", 0.5);
  planning.route_completion_tolerance_m =
      declare<double>("route_completion_tolerance_m", 2.0);
  planning.static_esdf_route_lookahead_m =
      declare<double>("static_esdf_route_lookahead_m", 180.0);
  planning.static_route_geometry.sample_step_m = planning.route_sampling_step_m;
  planner.route_sampling_step_m = planning.route_sampling_step_m;
  planning.static_route_geometry.enabled =
      planning.optional_constraints.static_route_geometry_optimization_enabled;
  planning.static_route_geometry.shortcut_optimization_enabled =
      planning.optional_constraints.static_route_shortcut_optimization_enabled;
  planning.static_route_geometry.maximum_shortcut_length_m =
      declare<double>("static_route_maximum_shortcut_length_m", 30.0);
  planning.static_route_geometry.sparse_deviation_tolerance_m =
      declare<double>("static_route_sparse_deviation_tolerance_m", 0.05);
  planning.static_route_geometry.maximum_shortcut_turn_increase_rad =
      declare<double>("static_route_maximum_shortcut_turn_increase_rad", 0.35);
  planning.static_route_geometry.shortcut_validation_batch_size =
      static_cast<std::size_t>(std::max<std::int64_t>(
          0, declare<std::int64_t>("static_route_shortcut_validation_batch_size", 4)));
  planning.static_route_geometry.corner_smoothing_distance_m =
      declare<double>("static_route_corner_smoothing_distance_m", 6.0);
  planning.static_route_geometry.corner_smoothing_minimum_distance_m =
      declare<double>("static_route_corner_smoothing_minimum_distance_m", 0.5);
  planning.static_route_geometry.corner_curve_samples = static_cast<std::size_t>(
      declare<std::int64_t>("static_route_corner_curve_samples", 4));
  const double static_route_replan_remaining_m =
      declare<double>("static_route_replan_remaining_m", 45.0);
  const double no_static_route_replan_remaining_m =
      declare<double>("no_static_route_replan_remaining_m", 15.0);
  planning.route_tracking_policy.minimum_remaining_m =
      config_.world.use_static_map ? static_route_replan_remaining_m
                                   : no_static_route_replan_remaining_m;
  planning.route_tracking_policy.maximum_cross_track_m =
      declare<double>("route_maximum_cross_track_m", 15.0);

  planning.static_route_extension.minimum_remaining_m =
      planning.route_tracking_policy.minimum_remaining_m;
  planning.static_route_extension.required_certified_overlap_m =
      declare<double>("route_required_certified_overlap_m", 8.0);
  planning.static_route_extension.latency_margin_s =
      declare<double>("route_extension_latency_margin_s", 0.5);
  planning.static_route_extension.maximum_latency_s =
      declare<double>("route_extension_maximum_latency_s", 8.0);
  planning.static_route_extension.minimum_retry_progress_m =
      declare<double>("route_extension_retry_progress_m", 15.0);
  planning.static_route_extension.minimum_retry_interval_s =
      declare<double>("route_extension_retry_interval_s", 1.0);
  planning.static_route_extension.minimum_endpoint_improvement_m =
      declare<double>("route_extension_minimum_endpoint_improvement_m", 5.0);
  planning.route_successor_improvement.minimum_absolute_improvement_s =
      declare<double>("route_successor_minimum_time_improvement_s", 1.0);
  planning.route_successor_improvement.minimum_relative_improvement =
      declare<double>("route_successor_minimum_time_improvement_ratio", 0.05);
  // The refinement search shares the successor admission margins: a gain the
  // lifecycle would not admit is not worth the planner budget.
  planner.execution_time_refinement_minimum_improvement_s =
      planning.route_successor_improvement.minimum_absolute_improvement_s;
  planner.execution_time_refinement_minimum_improvement_ratio =
      planning.route_successor_improvement.minimum_relative_improvement;
  planning.future_route_connector.tangent_departure_length_m =
      declare<double>("route_connector_departure_m", 0.5);
  planning.future_route_connector.successor_join_station_m =
      declare<double>("route_connector_join_m", 2.0);
  planning.future_route_connector.curve_control_distance_m =
      declare<double>("route_connector_control_m", 0.75);
  const std::int64_t connector_curve_samples =
      declare<std::int64_t>("route_connector_curve_samples", 12);
  planning.future_route_connector.curve_samples =
      connector_curve_samples >= 0 ? static_cast<std::size_t>(connector_curve_samples)
                                   : 0U;
  planning.future_route_connector.minimum_continuous_turn_alignment =
      declare<double>("route_connector_minimum_continuous_turn_alignment",
                      TrajectoryCompilerConfig3D{}.minimum_continuous_turn_alignment);
  planning.certified_route_splice.required_overlap_m =
      planning.static_route_extension.required_certified_overlap_m;
  planning.certified_route_splice.sample_step_m =
      declare<double>("route_splice_sample_step_m", planning.route_sampling_step_m);
  planning.certified_route_splice.maximum_position_separation_m =
      declare<double>("route_splice_maximum_position_separation_m", 0.05);
  planning.certified_route_splice.minimum_tangent_alignment =
      declare<double>("route_splice_minimum_tangent_alignment", 0.995);
  planning.certified_route_splice.activation_station_tolerance_m =
      declare<double>("route_splice_activation_station_tolerance_m", 1.0);
  planning.static_route_search_retry.minimum_pose_change_m =
      declare<double>("route_failed_search_pose_change_m", 2.0);
  planning.static_route_search_retry.minimum_objective_change_m =
      declare<double>("route_failed_search_objective_change_m", 5.0);
  planning.static_route_search_retry.minimum_retry_interval_s =
      declare<double>("route_failed_search_retry_interval_s", 1.0);
  planning.route_progress.observation_window_s =
      declare<double>("route_stall_observation_window_s", 1.0);
  planning.route_progress.minimum_progress_m =
      declare<double>("route_stall_minimum_progress_m", 0.5);
  planning.route_progress.minimum_predicted_head_progress_m =
      declare<double>("route_stall_minimum_predicted_head_progress_m", 0.5);
  planning.route_stall_recovery_enabled =
      declare<bool>("route_stall_recovery_enabled", false);

  planning.cooperative_traffic_enabled =
      declare<bool>("cooperative_traffic_enabled", false);
  planning.vehicle_id = declare<std::string>("vehicle_id", "");
  planning.cooperative_passage_route.desired_center_separation_m =
      declare<double>("cooperative_passage_desired_center_separation_m", 5.0);
  planning.cooperative_passage_volume.minimum_wall_clearance_m =
      declare<double>("cooperative_passage_minimum_wall_clearance_m", 1.0);
  planning.cooperative_passage_volume.lateral_probe_step_m =
      declare<double>("cooperative_passage_lateral_probe_step_m", 0.5);
  planning.cooperative_passage_volume.cross_section_spacing_m =
      declare<double>("cooperative_passage_cross_section_spacing_m", 1.0);
  planning.cooperative_passage_volume.secondary_probe_step_m =
      declare<double>("cooperative_passage_secondary_probe_step_m", 0.5);
  planning.cooperative_passage_volume.maximum_cross_section_probe_m =
      declare<double>("cooperative_passage_maximum_probe_m", 30.0);
  planning.cooperative_passage_route.preferred_transition_length_m =
      declare<double>("cooperative_passage_preferred_transition_m", 10.0);
  planning.cooperative_passage_route.minimum_transition_length_m =
      declare<double>("cooperative_passage_minimum_transition_m", 3.0);
  planning.cooperative_passage_route.directional_offset_fraction =
      declare<double>("cooperative_passage_directional_offset_fraction", 0.5);
  planning.cooperative_passage_timing.minimum_prediction_speed_mps =
      declare<double>("cooperative_passage_minimum_prediction_speed_mps", 1.0);
  planning.cooperative_passage_timing.maximum_prediction_horizon_s =
      declare<double>("cooperative_passage_maximum_prediction_horizon_s", 30.0);
  planning.cooperative_passage_yield.stopping_buffer_m =
      declare<double>("cooperative_passage_stopping_buffer_m", 2.0);
  planning.cooperative_passage_yield.reaction_latency_s =
      declare<double>("cooperative_passage_reaction_latency_s", 0.1);
  planning.cooperative_passage_yield.maximum_braking_acceleration_mps2 =
      declare<double>("cooperative_passage_maximum_braking_mps2", 8.0);
  planning.topics.cooperative_maneuver_command = declare<std::string>(
      "cooperative_maneuver_command_topic", "/drone_city_nav/cooperative/command");
  planning.topics.cooperative_passage_state = declare<std::string>(
      "cooperative_passage_state_topic", "/drone_city_nav/cooperative/passage_state");

  planning.liveness.enabled = declare<bool>("liveness_enabled", false);
  planning.liveness.observation_window_s =
      declare<double>("liveness_observation_window_s", 1.0);
  planning.liveness.minimum_actual_displacement_m =
      declare<double>("liveness_minimum_actual_displacement_m", 0.5);
  planning.liveness.minimum_offroute_displacement_m =
      declare<double>("liveness_minimum_offroute_displacement_m", 2.0);
  planning.liveness.stalled_windows_before_reseed =
      static_cast<std::size_t>(std::max<std::int64_t>(
          1, declare<std::int64_t>("liveness_stalled_windows_before_reseed", 2)));
  planning.topics.navigation_objective = declare<std::string>(
      "navigation_objective_topic", "/drone_city_nav/navigation_objective");
}

} // namespace drone_city_nav::production_mppi_config_ros_detail
