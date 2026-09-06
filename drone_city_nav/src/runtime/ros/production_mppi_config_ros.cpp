#include "production_mppi_config_ros.hpp"

#include "drone_city_nav/mission_waypoint_sequence.hpp"
#include "drone_city_nav/sensor_braking_contract_3d.hpp"
#include "drone_city_nav/trajectory_compiler_3d.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::int64_t durationNanoseconds(const double seconds,
                                               const char* const parameter_name,
                                               const bool allow_zero = false) {
  const long double nanoseconds = static_cast<long double>(seconds) * 1'000'000'000.0L;
  const long double first_unrepresentable_rounding_input =
      static_cast<long double>(std::numeric_limits<std::int64_t>::max()) + 0.5L;
  if (!std::isfinite(seconds) || (allow_zero ? seconds < 0.0 : !(seconds > 0.0)) ||
      nanoseconds >= first_unrepresentable_rounding_input) {
    throw std::invalid_argument{std::string{parameter_name} +
                                " must be finite, non-negative, and representable"};
  }
  const std::int64_t duration_ns = static_cast<std::int64_t>(std::llround(nanoseconds));
  if (!allow_zero && duration_ns <= 0) {
    throw std::invalid_argument{std::string{parameter_name} +
                                " rounds to a non-positive duration"};
  }
  return duration_ns;
}

[[nodiscard]] std::int64_t checkedDurationSum(const std::int64_t first_ns,
                                              const std::int64_t second_ns,
                                              const std::int64_t third_ns,
                                              const char* const name) {
  if (first_ns < 0 || second_ns < 0 || third_ns < 0 ||
      first_ns > std::numeric_limits<std::int64_t>::max() - second_ns ||
      first_ns + second_ns > std::numeric_limits<std::int64_t>::max() - third_ns) {
    throw std::invalid_argument{std::string{name} + " is not representable"};
  }
  return first_ns + second_ns + third_ns;
}

class ProductionMppiConfigLoader final {
public:
  explicit ProductionMppiConfigLoader(rclcpp::Node& node)
      : node_{node} {
  }

  [[nodiscard]] ProductionMppiConfig load() {
    declareDiagnostics();
    declareWorld();
    declareExecution();
    declarePlanning();
    declareControl();
    finalize();
    if (!config_.valid()) {
      throw std::invalid_argument{"invalid production MPPI configuration"};
    }
    return std::move(config_);
  }

private:
  template<typename T>
  [[nodiscard]] T declare(const char* const name, T default_value) {
    return node_.get().declare_parameter<T>(name, std::move(default_value));
  }

  [[nodiscard]] std::size_t declarePositiveSize(const char* const name,
                                                const std::int64_t default_value) {
    const std::int64_t value = declare<std::int64_t>(name, default_value);
    if (value <= 0) {
      throw std::invalid_argument{std::string{name} + " must be positive"};
    }
    return static_cast<std::size_t>(value);
  }

  void declareDiagnostics() {
    ProductionMppiConfig::Diagnostics& diagnostics = config_.diagnostics;
    diagnostics.rviz_rate_hz = declare<double>("rviz_rate_hz", 10.0);
    diagnostics.info_rate_hz = declare<double>("diagnostics_info_rate_hz", 5.0);
    diagnostics.file_rate_hz = declare<double>("diagnostics_file_rate_hz", 5.0);
    diagnostics.flush_period_s = declare<double>("diagnostics_flush_period_s", 1.0);
    const std::int64_t error_ring_capacity =
        declare<std::int64_t>("diagnostics_error_ring_capacity", 25);
    if (error_ring_capacity < 1 || error_ring_capacity > 1000) {
      throw std::invalid_argument{
          "diagnostics error ring capacity must be in [1, 1000]"};
    }
    diagnostics.error_ring_capacity = static_cast<std::size_t>(error_ring_capacity);
    diagnostics.route_constraint_distance_m =
        declare<double>("route_constraint_diagnostics_distance_m", 30.0);
    diagnostics.output_dir = declare<std::string>("diagnostics_output_dir", "log/mppi");
    diagnostics.topics.path =
        declare<std::string>("path_topic", "/drone_city_nav/mppi/path");
    diagnostics.topics.markers =
        declare<std::string>("markers_topic", "/drone_city_nav/mppi/markers");
    diagnostics.topics.status =
        declare<std::string>("status_topic", "/drone_city_nav/mppi/status");
    diagnostics.topics.world_readiness = declare<std::string>(
        "world_readiness_topic", "/drone_city_nav/mppi/world_ready");
    diagnostics.topics.planner_health = declare<std::string>(
        "planner_health_topic", "/drone_city_nav/mppi/planner_alive");
    diagnostics.topics.navigation_health = declare<std::string>(
        "navigation_health_topic", "/drone_city_nav/mppi/navigation_health");
  }

  void declareWorld() {
    ProductionMppiConfig::World& world = config_.world;
    world.use_static_map = declare<bool>("use_static_map", true);
    world.maximum_esdf_age_ms = declare<double>("maximum_esdf_age_ms", 1000.0);
    world.no_static_3d_esdf_update_rate_hz =
        declare<double>("no_static_3d_esdf_update_rate_hz", 1.0);
    const std::int64_t world_worker_count =
        declare<std::int64_t>("world_worker_count", 2);
    if (world_worker_count < 1 || world_worker_count > 8) {
      throw std::invalid_argument{"world_worker_count must be within [1, 8]"};
    }
    world.world_worker_count = static_cast<std::size_t>(world_worker_count);
    world.no_static_3d_esdf_window.horizontal_half_extent_m =
        declare<double>("no_static_3d_esdf_horizontal_half_extent_m", 20.0);
    world.no_static_3d_esdf_window.vertical_half_extent_m =
        declare<double>("no_static_3d_esdf_vertical_half_extent_m", 15.0);
    world.no_static_3d_esdf_window.horizontal_recenter_margin_m =
        declare<double>("no_static_3d_esdf_horizontal_recenter_margin_m", 12.0);
    world.no_static_3d_esdf_window.vertical_recenter_margin_m =
        declare<double>("no_static_3d_esdf_vertical_recenter_margin_m", 9.0);
    world.frame_id = declare<std::string>("frame_id", "map");
    world.gazebo_aligned_rviz_axes_swapped =
        declare<bool>(kGazeboAlignedRvizAxesSwappedParameter.data(), true);
    world.px4_map_transform = Px4MapFrameTransform{
        .map_origin = Point3{declare<double>("px4_local_origin_x_m", 54.0),
                             declare<double>("px4_local_origin_y_m", 54.0),
                             declare<double>("px4_local_origin_z_m", 0.0)},
        .m00 = declare<double>("px4_to_map_m00", 1.0),
        .m01 = declare<double>("px4_to_map_m01", 0.0),
        .m10 = declare<double>("px4_to_map_m10", 0.0),
        .m11 = declare<double>("px4_to_map_m11", 1.0),
    };
    world.px4_map_transform.validate();
    world.flight_envelope.minimum_target_z_m =
        declare<double>("minimum_target_z_m", 1.0);
    world.flight_envelope.maximum_target_z_m =
        declare<double>("maximum_target_z_m", 32.0);
    world.physical_footprint.radius_m =
        declare<double>("physical_footprint_radius_m", 0.82);
    world.physical_footprint.body_radius_m =
        declare<double>("physical_footprint_body_radius_m", 0.55);
    world.physical_footprint.lower_extent_m =
        declare<double>("physical_footprint_lower_extent_m", 0.23);
    world.physical_footprint.upper_extent_m =
        declare<double>("physical_footprint_upper_extent_m", 0.35);
    world.physical_footprint.perimeter_samples =
        declarePositiveSize("physical_footprint_samples", 12);
    world.physical_footprint.radial_rings =
        declarePositiveSize("physical_footprint_radial_rings", 2);
    world.physical_footprint.axial_samples =
        declarePositiveSize("physical_footprint_axial_samples", 3);
    world.physical_footprint.sweep_step_m =
        declare<double>("physical_footprint_sweep_step_m", 0.25);
    world.static_occupancy_3d_path = declare<std::string>(
        "static_occupancy_3d_path", "worlds/generated_city.occupancy3d");
    world.static_esdf_3d_cache_path = declare<std::string>(
        "static_esdf_3d_cache_path", "worlds/generated_city.esdf3d");
    world.static_free_space_topology_3d_path =
        declare<std::string>("static_free_space_topology_3d_path", "");
    world.topics.px4_local_position = declare<std::string>(
        "px4_local_position_topic", "/fmu/out/vehicle_local_position_v1");
    world.topics.navigation_readiness = declare<std::string>(
        "navigation_readiness_topic", "/drone_city_nav/navigation_ready");
    world.topics.raw_obstacle_snapshot_3d = declare<std::string>(
        "raw_obstacle_snapshot_3d_topic", "/drone_city_nav/raw_obstacle_snapshot_3d");
    world.topics.raw_obstacle_delta_3d = declare<std::string>(
        "raw_obstacle_delta_3d_topic", "/drone_city_nav/raw_obstacle_delta_3d");
    world.topics.latest_lidar_obstacle_scan =
        declare<std::string>("latest_lidar_obstacle_scan_topic",
                             "/drone_city_nav/latest_lidar_obstacle_scan");
    world.topics.obstacle_memory_status = declare<std::string>(
        "obstacle_memory_status_topic", "/drone_city_nav/obstacle_memory_status");
  }

  void declareExecution() {
    ProductionMppiConfig::Execution& execution = config_.execution;
    execution.maximum_pose_age_ms = declare<double>("maximum_pose_age_ms", 150.0);
    execution.maximum_vehicle_status_age_ms =
        declare<double>("maximum_vehicle_status_age_ms", 1000.0);
    execution.maximum_pose_prediction_age_ms =
        declare<double>("maximum_pose_prediction_age_ms", 1000.0);
    execution.maximum_control_feedback_age_ms =
        declare<double>("maximum_control_feedback_age_ms", 200.0);
    execution.horizon_acknowledgement_grace_ms =
        declare<double>("execution_horizon_acknowledgement_grace_ms", 100.0);
    execution.latest_lidar_obstacle_maximum_age_ms =
        declare<double>("latest_lidar_obstacle_maximum_age_ms", 1000.0);
    execution.stationary_hold_validity_s =
        declare<double>("stationary_hold_validity_s", 1.0);
    execution.navigation_health = NavigationHealthConfig{
        .terminal_failure_enabled =
            declare<bool>("navigation_health_terminal_failure_enabled", false),
        .maximum_unavailable_world_age_ms =
            declare<double>("maximum_unavailable_world_age_ms", 30'000.0),
        .maximum_no_executable_route_age_ms =
            declare<double>("maximum_no_executable_route_age_ms", 30'000.0),
        .maximum_unacknowledged_horizon_age_ms =
            declare<double>("maximum_unacknowledged_horizon_age_ms", 10'000.0),
        .maximum_recovery_attempts =
            static_cast<std::uint32_t>(std::clamp<std::int64_t>(
                declare<std::int64_t>("maximum_navigation_recovery_attempts", 64), 1,
                10'000)),
    };
    execution.mission_waypoint_capture_gate.target_match_tolerance_m =
        declare<double>("mission_waypoint_target_match_tolerance_m", 1.0e-3);
    execution.topics.px4_vehicle_status =
        declare<std::string>("px4_vehicle_status_topic", "/fmu/out/vehicle_status_v1");
    execution.topics.px4_vehicle_land_detected = declare<std::string>(
        "px4_vehicle_land_detected_topic", "/fmu/out/vehicle_land_detected");
    execution.topics.applied_control_feedback = declare<std::string>(
        "applied_control_feedback_topic", "/drone_city_nav/mppi/applied_control");
    execution.topics.execution_horizon = declare<std::string>(
        "execution_horizon_topic", "/drone_city_nav/mppi/execution_horizon");
    execution.topics.mission_waypoint_acknowledgement =
        declare<std::string>("mission_waypoint_acknowledgement_topic",
                             "/drone_city_nav/mission_waypoint_acknowledgement");
  }

  void declarePlanning();
  void declareControl();
  void finalize();

  std::reference_wrapper<rclcpp::Node> node_;
  ProductionMppiConfig config_{};
};

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
      declare<double>("dynamic_objective_replan_distance_m", 5.0);
  planning.dynamic_objective_replan_period_s =
      declare<double>("dynamic_objective_replan_period_s", 0.25);
  planning.tracking_objective_ray_sample_spacing_m =
      declare<double>("tracking_objective_ray_sample_spacing_m", 0.25);
  planning.tracking_capture_radius_m =
      declare<double>("tracking_capture_radius_m", 5.0);
  planning.static_tracking_esdf_refresh_margin_m =
      declare<double>("static_tracking_esdf_refresh_margin_m", 15.0);
  planning.direct_tracking_maneuver = DirectTrackingManeuverConfig{
      .bearing_change_threshold_rad = declare<double>(
          "direct_tracking_reseed_bearing_change_rad", 0.5235987755982988),
      .minimum_closing_speed_mps =
          declare<double>("direct_tracking_minimum_closing_speed_mps", 0.5),
      .closing_recovery_speed_mps =
          declare<double>("direct_tracking_closing_recovery_speed_mps", 1.5),
      .no_closing_duration_s =
          declare<double>("direct_tracking_no_closing_reseed_delay_s", 1.0),
      .minimum_reseed_interval_s =
          declare<double>("direct_tracking_minimum_reseed_interval_s", 0.5),
  };

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
      .latest_lidar_freshness_required =
          declare<bool>("execution_latest_lidar_freshness_required", false),
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
  planner.maximum_compute_time_ms =
      declare<double>("persistent_planner_maximum_compute_time_ms", 150.0);
  planner.clearance_ranking_weight =
      declare<double>("persistent_planner_clearance_ranking_weight", 1.5);
  planner.clearance_ranking_distance_m =
      declare<double>("persistent_planner_clearance_ranking_distance_m", 6.0);
  planner.clearance_ranking_critical_weight =
      declare<double>("persistent_planner_clearance_ranking_critical_weight", 100.0);

  planning.route_sampling_step_m = declare<double>("route_sampling_step_m", 0.5);
  planning.route_completion_tolerance_m =
      declare<double>("route_completion_tolerance_m", 2.0);
  planning.static_esdf_route_lookahead_m =
      declare<double>("static_esdf_route_lookahead_m", 180.0);
  planning.static_route_geometry.sample_step_m = planning.route_sampling_step_m;
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
      declare<double>("static_route_corner_smoothing_distance_m", 2.0);
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

  planning.noncooperative_avoidance_enabled =
      declare<bool>("noncooperative_avoidance_enabled", false);
  planning.topics.noncooperative_tracks = declare<std::string>(
      "noncooperative_tracks_topic", "/drone_city_nav/noncooperative_tracks");
  planning.noncooperative_avoidance.prediction_horizon_s =
      declare<double>("noncooperative_prediction_horizon_s", 4.0);
  planning.noncooperative_avoidance.strong_separation_m =
      declare<double>("noncooperative_strong_separation_m", 10.0);
  planning.noncooperative_avoidance.anticipation_separation_m =
      declare<double>("noncooperative_anticipation_separation_m", 20.0);
  planning.noncooperative_avoidance.release_separation_m =
      declare<double>("noncooperative_release_separation_m", 15.0);
  planning.noncooperative_avoidance.release_hysteresis_s =
      declare<double>("noncooperative_release_hysteresis_s", 1.0);
  planning.noncooperative_avoidance.maximum_track_age_s =
      declare<double>("noncooperative_maximum_track_age_s", 0.75);
  planning.noncooperative_avoidance.tracked_aircraft_radius_m =
      declare<double>("noncooperative_tracked_aircraft_radius_m", 0.82);
  planning.noncooperative_avoidance.minimum_relative_speed_mps =
      declare<double>("noncooperative_minimum_relative_speed_mps", 0.05);
  planning.noncooperative_avoidance.candidate_acceleration_fraction =
      declare<double>("noncooperative_candidate_acceleration_fraction", 0.95);
  planning.noncooperative_avoidance.candidate_duration_s =
      declare<double>("noncooperative_candidate_duration_s", 1.5);
  planning.noncooperative_avoidance.strong_cost_weight =
      declare<double>("noncooperative_strong_cost_weight", 4000.0);
  planning.noncooperative_avoidance.anticipation_cost_weight =
      declare<double>("noncooperative_anticipation_cost_weight", 40.0);
  planning.noncooperative_avoidance.time_to_collision_gain_s =
      declare<double>("noncooperative_time_to_collision_gain_s", 1.0);
  planning.noncooperative_avoidance.maximum_time_to_collision_multiplier =
      declare<double>("noncooperative_maximum_ttc_multiplier", 4.0);

  planning.liveness.enabled = declare<bool>("liveness_enabled", false);
  planning.liveness.observation_window_s =
      declare<double>("liveness_observation_window_s", 1.0);
  planning.liveness.minimum_actual_displacement_m =
      declare<double>("liveness_minimum_actual_displacement_m", 0.5);
  planning.liveness.minimum_predicted_terminal_progress_m =
      declare<double>("liveness_minimum_predicted_terminal_progress_m", 5.0);
  planning.topics.navigation_objective = declare<std::string>(
      "navigation_objective_topic", "/drone_city_nav/navigation_objective");
  planning.topics.radar_track_mode_command = declare<std::string>(
      "radar_track_mode_command_topic", "/drone_city_nav/radar/track_mode_command");
}

void ProductionMppiConfigLoader::declareControl() {
  ProductionMppiConfig::Control& control = config_.control;
  mppi::BenchmarkConfig& mppi = control.mppi;
  mppi.altitude_envelope = mppi::AltitudeEnvelopeConfig{
      .minimum_z_m =
          static_cast<float>(config_.world.flight_envelope.minimum_target_z_m),
      .maximum_z_m =
          static_cast<float>(config_.world.flight_envelope.maximum_target_z_m),
  };
  mppi.rollouts = declarePositiveSize("rollouts", 8192);
  control.rollout_budget.full_rollouts = mppi.rollouts;
  control.rollout_budget.open_static_rollouts =
      declarePositiveSize("open_static_rollouts", 6144);
  control.rollout_budget.direct_tracking_rollouts =
      declarePositiveSize("direct_tracking_rollouts", 4096);
  control.rollout_budget.minimum_reduced_clearance_m =
      static_cast<float>(declare<double>("adaptive_rollout_minimum_clearance_m", 8.0));
  control.rollout_budget.maximum_world_age_ms =
      declare<double>("adaptive_rollout_maximum_world_age_ms", 250.0);
  control.rollout_budget.maximum_tracking_age_ms =
      declare<double>("adaptive_rollout_maximum_tracking_age_ms", 250.0);
  const double dt_s = declare<double>("dt_s", 0.05);
  if (!std::isfinite(dt_s) || !(dt_s > 0.0)) {
    throw std::invalid_argument{"dt_s must be finite and positive"};
  }
  mppi.dynamics.dt_s = static_cast<float>(dt_s);
  const double static_horizon_duration_s =
      declare<double>("static_horizon_duration_s", 6.0);
  const double no_static_horizon_duration_s =
      declare<double>("no_static_horizon_duration_s", 4.0);
  const double active_horizon_duration_s = config_.world.use_static_map
                                               ? static_horizon_duration_s
                                               : no_static_horizon_duration_s;
  if (!std::isfinite(active_horizon_duration_s) || !(active_horizon_duration_s > 0.0)) {
    throw std::invalid_argument{"active horizon duration must be finite and positive"};
  }
  const double static_stale_esdf_execution_window_s = declare<double>(
      "static_stale_esdf_execution_window_s", static_horizon_duration_s);
  const double no_static_stale_esdf_execution_window_s = declare<double>(
      "no_static_stale_esdf_execution_window_s", no_static_horizon_duration_s);
  config_.execution.stale_esdf_execution_window_ms =
      1000.0 * (config_.world.use_static_map ? static_stale_esdf_execution_window_s
                                             : no_static_stale_esdf_execution_window_s);
  const double horizon_steps =
      std::ceil(active_horizon_duration_s / static_cast<double>(mppi.dynamics.dt_s));
  if (!std::isfinite(horizon_steps) || horizon_steps < 1.0 ||
      horizon_steps > static_cast<double>(std::numeric_limits<std::size_t>::max())) {
    throw std::invalid_argument{"MPPI horizon step count is not representable"};
  }
  mppi.steps = static_cast<std::size_t>(horizon_steps);

  control.speed_policy.horizon_duration_s = active_horizon_duration_s;
  control.speed_policy.cruise_speed_mps = declare<double>("cruise_speed_mps", 5.0);
  control.speed_policy.absolute_speed_limit_mps =
      declare<double>("absolute_speed_limit_mps", 10.0);
  const double maximum_horizontal_acceleration_mps2 =
      declare<double>("maximum_horizontal_acceleration_mps2", 4.0);
  const double maximum_vertical_acceleration_mps2 =
      declare<double>("maximum_vertical_acceleration_mps2", 4.0);
  const double maximum_control_jerk_mps3 =
      declare<double>("maximum_control_jerk_mps3", 12.0);
  control.speed_policy.maximum_lateral_acceleration_mps2 =
      maximum_horizontal_acceleration_mps2;
  control.speed_policy.stopping_capability
      .maximum_commanded_horizontal_deceleration_mps2 =
      maximum_horizontal_acceleration_mps2;
  control.speed_policy.stopping_capability.guaranteed_horizontal_deceleration_mps2 =
      maximum_horizontal_acceleration_mps2;
  control.speed_policy.stopping_capability.guaranteed_vertical_deceleration_mps2 =
      declare<double>("guaranteed_vertical_stopping_deceleration_mps2", 2.0);
  control.speed_policy.stopping_capability.reaction_latency_s =
      declare<double>("speed_reaction_latency_s", 0.10);
  control.speed_policy.sensor_braking_contract = SensorBrakingContract3D{
      .guaranteed_detection_range_m =
          declare<double>("guaranteed_lidar_detection_range_m", 30.0),
      .maximum_evidence_age_s =
          config_.execution.latest_lidar_obstacle_maximum_age_ms * 1.0e-3,
      .physical_margin_m = declare<double>("sensor_braking_physical_margin_m", 3.0),
      .maximum_forward_acceleration_mps2 = std::hypot(
          maximum_horizontal_acceleration_mps2, maximum_vertical_acceleration_mps2),
      .maximum_control_jerk_mps3 = maximum_control_jerk_mps3,
  };
  // Braking completes at the goal capture's stationary tolerance: a wider
  // margin leaves the vehicle drifting to rest outside the tolerance with no
  // reference speed left to close the gap.
  // Half the stationary hold tolerance: the limiter must still allow motion
  // at the capture boundary, or the vehicle settles just outside it.
  control.speed_policy.goal_margin_m = declare<double>(
      "goal_braking_margin_m", 0.5 * kStationaryExecutionHoldPositionToleranceM);
  control.speed_policy.curvature_preview_distance_m =
      declare<double>("curvature_preview_distance_m", 60.0);
  control.speed_policy.curvature_measurement_window_m =
      declare<double>("route_curvature_measurement_window_m", 5.0);
  control.speed_policy.minimum_target_lookahead_m =
      declare<double>("minimum_target_lookahead_m", 30.0);
  control.speed_policy.maximum_target_lookahead_m =
      declare<double>("maximum_target_lookahead_m", 100.0);
  mppi.stopping_capability = control.speed_policy.stopping_capability;
  mppi.dynamics.maximum_horizontal_speed_mps =
      static_cast<float>(control.speed_policy.absolute_speed_limit_mps);
  const double sensor_braking_speed_limit_mps =
      sensorBrakingMaximumSpeedMps(control.speed_policy.sensor_braking_contract,
                                   control.speed_policy.stopping_capability,
                                   control.speed_policy.absolute_speed_limit_mps);
  mppi.dynamics.maximum_translational_speed_mps =
      std::nextafter(static_cast<float>(sensor_braking_speed_limit_mps), 0.0F);
  mppi.dynamics.maximum_horizontal_acceleration_mps2 =
      static_cast<float>(maximum_horizontal_acceleration_mps2);
  config_.execution.finite_horizon =
      mppi::makeFiniteHorizonConfig(control.speed_policy.stopping_capability);
  mppi.dynamics.maximum_control_jerk_mps3 =
      static_cast<float>(maximum_control_jerk_mps3);
  mppi.dynamics.maximum_vertical_acceleration_mps2 =
      static_cast<float>(maximum_vertical_acceleration_mps2);
  mppi.altitude_envelope.guaranteed_vertical_deceleration_mps2 = static_cast<float>(
      control.speed_policy.stopping_capability.guaranteed_vertical_deceleration_mps2);
  mppi.altitude_envelope.reaction_latency_s =
      static_cast<float>(control.speed_policy.stopping_capability.reaction_latency_s);
  mppi.dynamics.maximum_vertical_speed_mps =
      static_cast<float>(declare<double>("maximum_vertical_speed_mps", 5.0));

  const double angular_minimum_interval_ms =
      declare<double>("navigation_angular_derivative_minimum_interval_ms", 1.0);
  const double angular_maximum_interval_ms =
      declare<double>("navigation_angular_derivative_maximum_interval_ms", 100.0);
  control.navigation_angular_derivative = NavigationAngularDerivativeConfig{
      .minimum_interval_s = angular_minimum_interval_ms * 1.0e-3,
      .maximum_interval_s = angular_maximum_interval_ms * 1.0e-3,
      .maximum_yaw_rate_radps =
          static_cast<double>(mppi.dynamics.maximum_yaw_rate_radps),
      .maximum_yaw_acceleration_radps2 =
          static_cast<double>(mppi.dynamics.maximum_yaw_acceleration_radps2),
  };
  control.constrained_route.maximum_vertical_acceleration_mps2 =
      mppi.dynamics.maximum_vertical_acceleration_mps2;
  control.constrained_route.maximum_vertical_speed_mps =
      mppi.dynamics.maximum_vertical_speed_mps;
  control.constrained_route.alignment_distance_buffer_m =
      declare<double>("constrained_route_alignment_distance_buffer_m", 5.0);
  control.constrained_route.stationary_hold_distance_m =
      declare<double>("constrained_route_stationary_hold_distance_m", 2.0);
  control.constrained_route.vertical_capture_margin_m =
      declare<double>("constrained_route_vertical_capture_margin_m", 0.5);
  control.constrained_route.vertical_capture_speed_mps =
      declare<double>("constrained_route_vertical_capture_speed_mps", 0.75);
  control.tracking_error_tube.response_time_s =
      declare<double>("tracking_error_tube_response_time_s", 0.15);
  control.tracking_error_tube.minimum_progress_speed_mps =
      declare<double>("tracking_error_tube_minimum_progress_speed_mps", 1.0);
  // The live clearance speed limit enforces the same tube law as the route
  // certification, so it shares the tube configuration.
  control.speed_policy.clearance_response_time_s =
      control.tracking_error_tube.response_time_s;
  control.speed_policy.clearance_minimum_progress_speed_mps =
      control.tracking_error_tube.minimum_progress_speed_mps;

  mppi.footprint = mppi::FootprintConfig{
      .radius_m = static_cast<float>(config_.world.physical_footprint.radius_m),
      .lower_extent_m =
          static_cast<float>(config_.world.physical_footprint.lower_extent_m),
      .upper_extent_m =
          static_cast<float>(config_.world.physical_footprint.upper_extent_m),
      .perimeter_samples = static_cast<std::uint32_t>(
          config_.world.physical_footprint.perimeter_samples),
      .radial_rings =
          static_cast<std::uint32_t>(config_.world.physical_footprint.radial_rings),
      .axial_samples =
          static_cast<std::uint32_t>(config_.world.physical_footprint.axial_samples),
  };
  mppi.footprint.clearance_broad_phase_enabled =
      declare<bool>("mppi_footprint_clearance_broad_phase_enabled", true);
  mppi.costs.temperature = static_cast<float>(declare<double>("mppi_temperature", 8.0));
  mppi.costs.adaptive_temperature_cost_fraction = static_cast<float>(
      declare<double>("mppi_adaptive_temperature_cost_fraction", 0.0));
  mppi.costs.body_collision_gate_enabled =
      declare<bool>("mppi_body_collision_gate_enabled", false);
  mppi.costs.route_directed_candidate_cost_tolerance = static_cast<float>(
      declare<double>("route_directed_candidate_cost_tolerance", 0.5));
  mppi.costs.head_progress_horizon_s =
      static_cast<float>(declare<double>("head_progress_horizon_s", 0.4));
  mppi.costs.head_progress_weight =
      static_cast<float>(declare<double>("head_progress_weight", 8.0));
  mppi.costs.altitude_tracking_weight =
      static_cast<float>(declare<double>("altitude_tracking_weight", 4.0));
  mppi.costs.route_progress_integral_weight =
      static_cast<float>(declare<double>("route_progress_integral_weight", 2.0));
  const float planning_exposure_weight =
      static_cast<float>(declare<double>("planning_exposure_weight", 2.0));
  const float critical_exposure_weight =
      static_cast<float>(declare<double>("critical_exposure_weight", 20.0));
  const float critical_clearance_proximity_weight =
      static_cast<float>(declare<double>("critical_clearance_proximity_weight", 400.0));
  const float obstacle_approach_weight =
      static_cast<float>(declare<double>("obstacle_approach_weight", 40.0));
  if (config_.planning.optional_constraints.clearance_costs_enabled) {
    mppi.costs.planning_exposure_weight = planning_exposure_weight;
    mppi.costs.critical_exposure_weight = critical_exposure_weight;
    mppi.costs.critical_clearance_proximity_weight =
        critical_clearance_proximity_weight;
    mppi.costs.obstacle_approach_weight = obstacle_approach_weight;
  } else {
    mppi.costs.planning_exposure_weight = 0.0F;
    mppi.costs.critical_exposure_weight = 0.0F;
    mppi.costs.critical_clearance_proximity_weight = 0.0F;
    mppi.costs.obstacle_approach_weight = 0.0F;
  }
  mppi.horizon_sampling.full_rate_duration_s =
      static_cast<float>(declare<double>("far_horizon_full_rate_duration_s", 2.0));
  mppi.horizon_sampling.far_cost_stride =
      static_cast<std::uint32_t>(declare<std::int64_t>("far_horizon_cost_stride", 2));
  const double static_speed_tracking_weight =
      declare<double>("static_speed_tracking_weight", 1.0);
  const double no_static_speed_tracking_weight =
      declare<double>("no_static_speed_tracking_weight", 1.0);
  mppi.costs.speed_tracking_weight = static_cast<float>(
      config_.world.use_static_map ? static_speed_tracking_weight
                                   : no_static_speed_tracking_weight);
  mppi.costs.overspeed_weight =
      static_cast<float>(declare<double>("overspeed_weight", 200.0));
  mppi.risk.critical_distance_m =
      static_cast<float>(declare<double>("critical_distance_m", 1.0));
  mppi.risk.preferred_distance_m =
      static_cast<float>(declare<double>("preferred_distance_m", 6.0));
  mppi.risk.obstacle_approach_response_time_s =
      static_cast<float>(declare<double>("obstacle_approach_response_time_s", 0.25));
  mppi.risk.obstacle_approach_deceleration_mps2 =
      static_cast<float>(declare<double>("obstacle_approach_deceleration_mps2", 4.0));
  const std::int64_t seed = declare<std::int64_t>("seed", 42);
  if (seed < 0) {
    throw std::invalid_argument{"seed must be non-negative"};
  }
  mppi.seed = static_cast<std::uint64_t>(seed);
  mppi.early_exit_on_altitude_envelope_violation = true;

  mppi.cooperative.desired_minimum_separation_m = static_cast<float>(
      declare<double>("cooperative_desired_minimum_separation_m", 5.0));
  mppi.cooperative.candidate_acceleration_fraction = static_cast<float>(
      declare<double>("cooperative_candidate_acceleration_fraction", 0.75));
  mppi.cooperative.candidate_duration_s =
      static_cast<float>(declare<double>("cooperative_candidate_duration_s", 1.5));
  mppi.costs.peer_separation_weight =
      static_cast<float>(declare<double>("cooperative_peer_separation_weight", 80.0));
  mppi.costs.cooperative_maneuver_preference_weight = static_cast<float>(
      declare<double>("cooperative_maneuver_preference_weight", 1.5));
}

void ProductionMppiConfigLoader::finalize() {
  ProductionMppiConfig::World& world = config_.world;
  ProductionMppiConfig::Planning& planning = config_.planning;
  ProductionMppiConfig::Execution& execution = config_.execution;
  ProductionMppiConfig::Control& control = config_.control;
  ProductionMppiConfig::Diagnostics& diagnostics = config_.diagnostics;

  if (!std::isfinite(planning.tick_rate_hz) || !(planning.tick_rate_hz > 0.0)) {
    throw std::invalid_argument{"tick_rate_hz must be finite and positive"};
  }
  const double planning_tick_period_s = 1.0 / planning.tick_rate_hz;
  if (!std::isfinite(planning.planning_tick_phase_offset_s) ||
      planning.planning_tick_phase_offset_s < 0.0 ||
      planning.planning_tick_phase_offset_s >= planning_tick_period_s) {
    throw std::invalid_argument{
        "planning tick phase offset must be in [0, one planning tick period)"};
  }
  // The planner's critical ranking band is the execution risk model's critical
  // distance: both sides then agree on which metres are nearly unexecutable.
  planning.persistent_planner.clearance_ranking_critical_distance_m =
      static_cast<double>(control.mppi.risk.critical_distance_m);
  execution.stationary_hold_validity_ns = durationNanoseconds(
      execution.stationary_hold_validity_s, "stationary_hold_validity_s");
  execution.horizon_acknowledgement_grace_ns =
      durationNanoseconds(execution.horizon_acknowledgement_grace_ms * 1.0e-3,
                          "execution_horizon_acknowledgement_grace_ms", true);
  execution.mission_waypoint_capture_gate = MissionWaypointCaptureGateConfig{
      .goal_radius_m = planning.mission_goal_capture.capture_radius_m,
      .target_match_tolerance_m =
          execution.mission_waypoint_capture_gate.target_match_tolerance_m,
      .stop_speed_mps = planning.mission_waypoint_sequence.stop_speed_mps,
      .stop_hold_s = planning.mission_waypoint_sequence.stop_hold_s,
      .maximum_pose_age_s = execution.maximum_pose_age_ms * 1.0e-3,
      .maximum_vehicle_status_age_s = execution.maximum_vehicle_status_age_ms * 1.0e-3,
      .maximum_feedback_age_s = execution.maximum_control_feedback_age_ms * 1.0e-3,
  };
  const std::int64_t goal_capture_hold_ns =
      durationNanoseconds(planning.mission_waypoint_sequence.stop_hold_s,
                          "mission waypoint stop hold", true);
  const std::int64_t goal_capture_feedback_margin_ns =
      durationNanoseconds(execution.maximum_control_feedback_age_ms * 1.0e-3,
                          "mission goal capture feedback margin");
  const std::int64_t goal_capture_tick_margin_ns = durationNanoseconds(
      2.0 / planning.tick_rate_hz, "mission goal capture tick margin");
  execution.mission_goal_capture_hold_validity_ns = checkedDurationSum(
      goal_capture_hold_ns, goal_capture_feedback_margin_ns,
      goal_capture_tick_margin_ns, "mission goal capture hold lease");

  planning.cooperative_passage_volume.flight_envelope = world.flight_envelope;
  planning.cooperative_passage_volume.footprint = SweptFootprintConfig{
      .radius_m = world.physical_footprint.radius_m,
      .lower_extent_m = world.physical_footprint.lower_extent_m,
      .upper_extent_m = world.physical_footprint.upper_extent_m,
      .perimeter_samples = world.physical_footprint.perimeter_samples,
      .radial_rings = world.physical_footprint.radial_rings,
      .axial_samples = world.physical_footprint.axial_samples,
      .sweep_step_m = world.physical_footprint.sweep_step_m,
  };
  planning.cooperative_passage_route.footprint =
      planning.cooperative_passage_volume.footprint;

  planning.static_route_extension.maximum_horizontal_acceleration_mps2 =
      control.mppi.dynamics.maximum_horizontal_acceleration_mps2;
  planning.static_route_extension.maximum_vertical_acceleration_mps2 =
      control.mppi.dynamics.maximum_vertical_acceleration_mps2;
  planning.static_route_extension.maximum_control_jerk_mps3 =
      control.mppi.dynamics.maximum_control_jerk_mps3;
  planning.static_route_extension.stopping_capability =
      control.speed_policy.stopping_capability;
  planning.persistent_planner.time_model = FlightTimeModel3D{
      .maximum_horizontal_speed_mps =
          std::min(control.speed_policy.cruise_speed_mps,
                   control.speed_policy.absolute_speed_limit_mps),
      .maximum_vertical_speed_mps =
          static_cast<double>(control.mppi.dynamics.maximum_vertical_speed_mps),
      .maximum_translational_speed_mps =
          static_cast<double>(control.mppi.dynamics.maximum_translational_speed_mps),
      .maximum_horizontal_acceleration_mps2 = static_cast<double>(
          control.mppi.dynamics.maximum_horizontal_acceleration_mps2),
      .maximum_vertical_acceleration_mps2 =
          static_cast<double>(control.mppi.dynamics.maximum_vertical_acceleration_mps2),
      .maximum_control_jerk_mps3 =
          static_cast<double>(control.mppi.dynamics.maximum_control_jerk_mps3),
      .maximum_yaw_acceleration_radps2 =
          static_cast<double>(control.mppi.dynamics.maximum_yaw_acceleration_radps2),
      .maximum_yaw_rate_radps =
          static_cast<double>(control.mppi.dynamics.maximum_yaw_rate_radps),
  };
  planning.persistent_planner.minimum_continuous_turn_alignment =
      planning.future_route_connector.minimum_continuous_turn_alignment;
  planning.persistent_planner.physical_footprint = world.physical_footprint;
  planning.persistent_planner.flight_envelope = world.flight_envelope;

  diagnostics.rviz_period_ns =
      static_cast<std::int64_t>(1.0e9 / std::max(0.1, diagnostics.rviz_rate_hz));
  diagnostics.info_period_ns =
      static_cast<std::int64_t>(1.0e9 / std::max(0.1, diagnostics.info_rate_hz));
  diagnostics.file_period_ns =
      static_cast<std::int64_t>(1.0e9 / std::max(0.1, diagnostics.file_rate_hz));

  execution.validation_policy = VersionedExecutionValidationPolicy3D::capture(
      world.flight_envelope, control.mppi.dynamics, control.mppi.altitude_envelope,
      world.physical_footprint, execution.latest_lidar_obstacle_maximum_age_ms,
      execution.maximum_pose_prediction_age_ms,
      execution.maximum_control_feedback_age_ms,
      planning.optional_constraints.route_cross_track_constraints_enabled,
      planning.optional_constraints.latest_lidar_freshness_required,
      planning.optional_constraints.route_tracking_tube_constraints_enabled,
      control.speed_policy.minimum_target_lookahead_m);
  if (execution.validation_policy == nullptr) {
    throw std::invalid_argument{"invalid immutable execution validation policy"};
  }
}

} // namespace

ProductionMppiConfig declareProductionMppiConfig(rclcpp::Node& node) {
  return ProductionMppiConfigLoader{node}.load();
}

} // namespace drone_city_nav
