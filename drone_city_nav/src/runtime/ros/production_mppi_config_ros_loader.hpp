#pragma once

#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

#include "production_mppi_config_ros.hpp"

namespace drone_city_nav::production_mppi_config_ros_detail {

// The loader declares the node's parameters group by group and assembles the
// configuration; the planning and control groups are in their own units.

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
    diagnostics.topics.navigation_progress = declare<std::string>(
        "navigation_progress_topic", "/drone_city_nav/mppi/navigation_progress");
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
    // The configured extents measure the hull; the envelope's grow with the
    // tilt the dynamics reach once those are known.
    world.physical_footprint.body_lower_extent_m =
        world.physical_footprint.lower_extent_m;
    world.physical_footprint.body_upper_extent_m =
        world.physical_footprint.upper_extent_m;
    world.physical_footprint.perimeter_samples =
        declarePositiveSize("physical_footprint_samples", 12);
    world.physical_footprint.radial_rings =
        declarePositiveSize("physical_footprint_radial_rings", 2);
    world.physical_footprint.axial_samples =
        declarePositiveSize("physical_footprint_axial_samples", 3);
    world.physical_footprint.sweep_step_m =
        declare<double>("physical_footprint_sweep_step_m", 0.25);
    world.static_occupancy_3d_path =
        declare<std::string>("static_occupancy_3d_path", "");
    world.static_esdf_3d_cache_path =
        declare<std::string>("static_esdf_3d_cache_path", "");
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
    world.topics.light_headroom =
        declare<std::string>("light_headroom_topic", "/stereo_depth/light_headroom");
    world.topics.carried_light_output = declare<std::string>(
        "carried_light_output_topic", "/carried_light/output_share");
    world.topics.silent_range =
        declare<std::string>("silent_range_topic", "/stereo_depth/silent_range");
    world.topics.void_range =
        declare<std::string>("void_range_topic", "/stereo_depth/void_range");
    world.topics.latest_sensor_obstacle_scan =
        declare<std::string>("latest_sensor_obstacle_scan_topic",
                             "/drone_city_nav/latest_sensor_obstacle_scan");
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
    execution.latest_sensor_obstacle_maximum_age_ms =
        declare<double>("latest_sensor_obstacle_maximum_age_ms", 1000.0);
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

} // namespace drone_city_nav::production_mppi_config_ros_detail
