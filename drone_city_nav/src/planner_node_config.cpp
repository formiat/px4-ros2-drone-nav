#include "drone_city_nav/planner_node_config.hpp"

#include "drone_city_nav/grid_config.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::int64_t secondsToNanoseconds(const double seconds) {
  return static_cast<std::int64_t>(seconds * 1.0e9);
}

} // namespace

PlannerNodeConfig loadPlannerNodeConfig(rclcpp::Node& node) {
  PlannerNodeConfig config{};
  config.frame_id = node.declare_parameter<std::string>("frame_id", "map");
  config.start = Point2{node.declare_parameter<double>("start_x_m", 0.0),
                        node.declare_parameter<double>("start_y_m", 0.0)};
  config.goal = Point2{node.declare_parameter<double>("goal_x_m", 85.0),
                       node.declare_parameter<double>("goal_y_m", 0.0)};
  config.cruise_altitude_m = node.declare_parameter<double>("cruise_altitude_m", 12.0);
  config.inflation_radius_m = node.declare_parameter<double>("inflation_radius_m", 2.5);

  config.timing.max_pose_staleness_ns = secondsToNanoseconds(std::clamp<double>(
      node.declare_parameter<double>("max_pose_staleness_s", 1.0), 0.0, 3600.0));
  config.planner_core.stable_path_goal_tolerance_m = std::clamp(
      node.declare_parameter<double>("stable_path_goal_tolerance_m", 3.0), 0.0, 1000.0);
  config.memory_grid.occupied_value = static_cast<int>(std::clamp<std::int64_t>(
      node.declare_parameter<std::int64_t>("memory_occupied_value", 100), 1, 100));
  config.memory_grid.free_value = static_cast<int>(std::clamp<std::int64_t>(
      node.declare_parameter<std::int64_t>("memory_free_value", 0), 0, 100));

  config.static_map.enabled = node.declare_parameter<bool>("use_static_map", true);
  config.planning_grid_builder.use_static_map = config.static_map.enabled;
  config.static_map.configured_path = node.declare_parameter<std::string>(
      "static_map_path", "worlds/generated_city.map2d");
  config.static_map.expected_frame_id = config.frame_id;
  config.static_map.min_blocking_height_m = std::clamp(
      node.declare_parameter<double>("static_map_min_blocking_height_m", 0.0), 0.0,
      100000.0);
  const double planning_grid_origin_x =
      node.declare_parameter<double>("planning_grid_origin_x", -10.0);
  const double planning_grid_origin_y =
      node.declare_parameter<double>("planning_grid_origin_y", -10.0);
  const double planning_grid_resolution_m =
      node.declare_parameter<double>("planning_grid_resolution_m", 0.5);
  const double planning_grid_width_m =
      node.declare_parameter<double>("planning_grid_width_m", 115.0);
  const double planning_grid_height_m =
      node.declare_parameter<double>("planning_grid_height_m", 175.0);
  config.planning_grid_builder.fallback_bounds = boundedGridBounds(
      planning_grid_origin_x, planning_grid_origin_y, planning_grid_resolution_m,
      planning_grid_width_m, planning_grid_height_m);
  const GridBounds& fallback_bounds = config.planning_grid_builder.fallback_bounds;
  if (!std::isfinite(planning_grid_origin_x) ||
      !std::isfinite(planning_grid_origin_y) ||
      !std::isfinite(planning_grid_resolution_m) ||
      !std::isfinite(planning_grid_width_m) || !std::isfinite(planning_grid_height_m) ||
      planning_grid_resolution_m <= 0.0 || planning_grid_width_m <= 0.0 ||
      planning_grid_height_m <= 0.0 ||
      planning_grid_origin_x != fallback_bounds.origin_x ||
      planning_grid_origin_y != fallback_bounds.origin_y ||
      planning_grid_resolution_m != fallback_bounds.resolution_m ||
      boundedPositiveCellCount(planning_grid_width_m, fallback_bounds.resolution_m) !=
          fallback_bounds.width_cells ||
      boundedPositiveCellCount(planning_grid_height_m, fallback_bounds.resolution_m) !=
          fallback_bounds.height_cells) {
    RCLCPP_WARN(node.get_logger(),
                "Sanitized planning grid bounds: requested origin=(%.3f, %.3f) "
                "resolution=%.3f size=(%.3f, %.3f)m final origin=(%.3f, %.3f) "
                "resolution=%.3f cells=%dx%d",
                planning_grid_origin_x, planning_grid_origin_y,
                planning_grid_resolution_m, planning_grid_width_m,
                planning_grid_height_m, fallback_bounds.origin_x,
                fallback_bounds.origin_y, fallback_bounds.resolution_m,
                fallback_bounds.width_cells, fallback_bounds.height_cells);
  }
  config.timing.max_current_lidar_staleness_ns =
      secondsToNanoseconds(std::clamp<double>(
          node.declare_parameter<double>("max_current_lidar_staleness_s", 0.75), 0.0,
          3600.0));
  config.lidar_projection.max_lidar_range_m =
      node.declare_parameter<double>("max_lidar_range_m", 35.0);
  config.lidar_projection.range_hit_epsilon_m =
      node.declare_parameter<double>("range_hit_epsilon_m", 0.05);
  config.lidar_projection.scan_yaw_offset_rad =
      node.declare_parameter<double>("scan_yaw_offset_rad", 0.0);
  config.current_lidar.use_px4_heading_for_scan =
      node.declare_parameter<bool>("use_px4_heading_for_scan", false);
  config.current_lidar.motion_compensate_lidar_pose =
      node.declare_parameter<bool>("motion_compensate_lidar_pose", true);
  config.current_lidar.lidar_pose_latency_s = std::clamp(
      node.declare_parameter<double>("lidar_pose_latency_s", 0.05), 0.0, 1.0);
  config.lidar_projection.compensate_attitude =
      node.declare_parameter<bool>("compensate_lidar_attitude", false);
  config.lidar_projection.lidar_mount_roll_rad =
      node.declare_parameter<double>("lidar_mount_roll_rad", 0.0);
  config.lidar_projection.lidar_mount_pitch_rad =
      node.declare_parameter<double>("lidar_mount_pitch_rad", 0.0);
  config.lidar_projection.lidar_mount_yaw_rad =
      node.declare_parameter<double>("lidar_mount_yaw_rad", 0.0);
  config.lidar_projection.lidar_z_offset_m =
      node.declare_parameter<double>("lidar_z_offset_m", 0.0);
  config.lidar_projection.min_projected_altitude_m =
      node.declare_parameter<double>("min_projected_lidar_altitude_m", 0.0);
  config.lidar_projection.max_projected_altitude_m =
      node.declare_parameter<double>("max_projected_lidar_altitude_m", 100000.0);

  config.planner_core.astar.turn_cost_weight = std::clamp(
      node.declare_parameter<double>("astar_turn_cost_weight", 0.0), 0.0, 1000.0);
  config.planner_core.astar.heuristic_weight = std::clamp(
      node.declare_parameter<double>("astar_heuristic_weight", 1.0), 1.0, 10.0);
  config.planner_core.astar.evasive_maneuvering_enabled =
      node.declare_parameter<bool>("astar_evasive_maneuvering_enabled", false);
  config.planner_core.astar.evasive_maneuvering_straight_cost_weight =
      std::clamp(node.declare_parameter<double>(
                     "astar_evasive_maneuvering_straight_cost_weight", 1.0),
                 0.0, 1000.0);
  config.planner_core.astar.initial_heading_bias_enabled =
      node.declare_parameter<bool>("astar_initial_heading_bias_enabled", false);
  config.planner_core.astar.initial_heading_bias_min_speed_mps = std::clamp(
      node.declare_parameter<double>("astar_initial_heading_bias_min_speed_mps", 0.5),
      0.0, 100.0);
  config.planner_core.astar.initial_heading_bias_weight = std::clamp(
      node.declare_parameter<double>("astar_initial_heading_bias_weight", 50.0), 0.0,
      1000.0);
  config.planner_core.clearance_diagnostic_radius_m = 10.0;

  config.trajectory_planner.speed_profile.cruise_speed_mps =
      std::clamp(node.declare_parameter<double>("cruise_speed_mps", 12.0), 0.0, 100.0);
  config.trajectory_planner.speed_profile.min_turn_speed_mps =
      std::clamp(node.declare_parameter<double>("min_turn_speed_mps", 2.0), 0.0,
                 config.trajectory_planner.speed_profile.cruise_speed_mps);
  config.trajectory_planner.speed_profile.max_accel_mps2 =
      std::clamp(node.declare_parameter<double>("max_accel_mps2", 3.0), 0.0, 100.0);
  config.trajectory_planner.speed_profile.max_decel_mps2 =
      std::clamp(node.declare_parameter<double>("max_decel_mps2", 4.0), 0.0, 100.0);
  config.trajectory_planner.speed_profile.max_lateral_accel_mps2 = std::clamp(
      node.declare_parameter<double>("max_lateral_accel_mps2", 3.0), 0.0, 100.0);
  config.trajectory_planner.speed_profile.speed_profile_decel_mps2 = std::clamp(
      node.declare_parameter<double>("speed_profile_decel_mps2", 2.0), 0.0, 100.0);
  config.trajectory_planner.speed_profile.speed_profile_sample_step_m = std::clamp(
      node.declare_parameter<double>("speed_profile_sample_step_m", 1.0), 0.1, 10.0);
  config.trajectory_planner.speed_profile.speed_profile_lookahead_time_s = std::clamp(
      node.declare_parameter<double>("speed_profile_lookahead_time_s", 1.0), 0.0, 30.0);
  config.trajectory_planner.speed_profile.speed_profile_lookahead_min_m = std::clamp(
      node.declare_parameter<double>("speed_profile_lookahead_min_m", 5.0), 0.0, 500.0);
  const double requested_speed_profile_lookahead_max_m =
      std::clamp(node.declare_parameter<double>("speed_profile_lookahead_max_m", 35.0),
                 0.0, 5000.0);
  config.trajectory_planner.speed_profile.speed_profile_lookahead_max_m =
      std::max(requested_speed_profile_lookahead_max_m,
               config.trajectory_planner.speed_profile.speed_profile_lookahead_min_m);
  config.trajectory_planner.corridor.max_radius_m = std::clamp(
      node.declare_parameter<double>("corridor_max_radius_m", 40.0), 1.0, 5000.0);
  config.trajectory_planner.corridor.sample_step_m = std::clamp(
      node.declare_parameter<double>("corridor_sample_step_m", 1.0), 0.1, 20.0);
  config.trajectory_planner.corridor.ray_step_m =
      std::clamp(node.declare_parameter<double>("corridor_ray_step_m", 0.0), 0.0, 20.0);
  config.trajectory_planner.corridor.safety_margin_m = std::clamp(
      node.declare_parameter<double>("corridor_safety_margin_m", 0.5), 0.0, 100.0);
  config.trajectory_planner.corridor.center_recovery_max_m =
      std::clamp(node.declare_parameter<double>("corridor_center_recovery_max_m", 3.0),
                 0.0, 5000.0);
  config.trajectory_planner.corridor.lateral_limit_window_m = std::clamp(
      node.declare_parameter<double>("corridor_lateral_limit_window_m", 20.0), 0.1,
      5000.0);
  config.trajectory_planner.corridor.lateral_limit_ratio = std::clamp(
      node.declare_parameter<double>("corridor_lateral_limit_ratio", 1.25), 1.0, 100.0);
  config.trajectory_planner.corridor.lateral_limit_margin_m =
      std::clamp(node.declare_parameter<double>("corridor_lateral_limit_margin_m", 1.0),
                 0.0, 5000.0);
  config.trajectory_planner.racing_line.max_iterations =
      static_cast<std::size_t>(std::clamp<std::int64_t>(
          node.declare_parameter<std::int64_t>("racing_line_max_iterations", 80), 1,
          10000));
  config.trajectory_planner.racing_line.initial_offset_step_m = std::clamp(
      node.declare_parameter<double>("racing_line_initial_offset_step_m", 2.0), 0.001,
      500.0);
  config.trajectory_planner.racing_line.min_offset_step_m =
      std::clamp(node.declare_parameter<double>("racing_line_min_offset_step_m", 0.1),
                 0.001, config.trajectory_planner.racing_line.initial_offset_step_m);
  config.trajectory_planner.racing_line.optimizer_sample_step_m = std::clamp(
      node.declare_parameter<double>("racing_line_optimizer_sample_step_m", 5.0), 0.0,
      100.0);
  config.trajectory_planner.racing_line.cooling_ratio = std::clamp(
      node.declare_parameter<double>("racing_line_cooling_ratio", 0.5), 0.05, 0.95);
  config.trajectory_planner.racing_line.weight_length = std::clamp(
      node.declare_parameter<double>("racing_line_weight_length", 0.02), 0.0, 1.0e6);
  config.trajectory_planner.racing_line.weight_curvature =
      std::clamp(node.declare_parameter<double>("racing_line_weight_curvature", 250.0),
                 0.0, 1.0e9);
  config.trajectory_planner.racing_line.weight_curvature_change = std::clamp(
      node.declare_parameter<double>("racing_line_weight_curvature_change", 100.0), 0.0,
      1.0e9);
  config.trajectory_planner.racing_line.weight_offset_change = std::clamp(
      node.declare_parameter<double>("racing_line_weight_offset_change", 0.5), 0.0,
      1.0e9);
  config.trajectory_planner.racing_line.weight_offset_second_change = std::clamp(
      node.declare_parameter<double>("racing_line_weight_offset_second_change", 5.0),
      0.0, 1.0e9);
  config.trajectory_planner.racing_line.weight_time = std::clamp(
      node.declare_parameter<double>("racing_line_weight_time", 50.0), 0.0, 1.0e9);
  config.trajectory_planner.racing_line.weight_edge_margin =
      std::clamp(node.declare_parameter<double>("racing_line_weight_edge_margin", 80.0),
                 0.0, 1.0e9);
  config.trajectory_planner.racing_line.desired_edge_margin_m = std::clamp(
      node.declare_parameter<double>("racing_line_desired_edge_margin_m", 6.0), 0.0,
      1000.0);
  config.trajectory_planner.racing_line.max_length_ratio = std::clamp(
      node.declare_parameter<double>("racing_line_max_length_ratio", 1.6), 1.0, 100.0);
  config.trajectory_planner.racing_line.regularization_iterations =
      static_cast<std::size_t>(
          std::clamp<std::int64_t>(node.declare_parameter<std::int64_t>(
                                       "racing_line_regularization_iterations", 2),
                                   0, 100));
  config.trajectory_planner.racing_line.regularization_max_time_regression_s =
      std::clamp(node.declare_parameter<double>(
                     "racing_line_regularization_max_time_regression_s", 0.5),
                 0.0, 3600.0);
  config.trajectory_planner.racing_line.parallel_workers =
      static_cast<std::size_t>(std::clamp<std::int64_t>(
          node.declare_parameter<std::int64_t>("racing_line_parallel_workers", 0), 0,
          1024));
  config.trajectory_planner.turn_smoothing.trigger_heading_delta_rad = std::clamp(
      node.declare_parameter<double>("turn_smoothing_trigger_heading_delta_deg", 37.0) *
          std::numbers::pi / 180.0,
      0.0, std::numbers::pi);
  config.trajectory_planner.turn_smoothing.trigger_min_radius_m = std::clamp(
      node.declare_parameter<double>("turn_smoothing_trigger_min_radius_m", 12.0), 0.0,
      10000.0);
  config.trajectory_planner.turn_smoothing.entry_distance_m = std::clamp(
      node.declare_parameter<double>("turn_smoothing_entry_distance_m", 45.0), 0.1,
      5000.0);
  config.trajectory_planner.turn_smoothing.exit_distance_m =
      std::clamp(node.declare_parameter<double>("turn_smoothing_exit_distance_m", 45.0),
                 0.1, 5000.0);
  config.trajectory_planner.turn_smoothing.sample_step_m = std::clamp(
      node.declare_parameter<double>("turn_smoothing_sample_step_m", 1.0), 0.1, 20.0);
  config.trajectory_planner.turn_smoothing.outer_bias_ratio = std::clamp(
      node.declare_parameter<double>("turn_smoothing_outer_bias_ratio", 0.45), 0.0,
      1.0);
  config.trajectory_planner.turn_smoothing.min_outer_shift_m = std::clamp(
      node.declare_parameter<double>("turn_smoothing_min_outer_shift_m", 2.0), 0.0,
      5000.0);
  config.trajectory_planner.turn_smoothing.max_outer_shift_m =
      std::max(config.trajectory_planner.turn_smoothing.min_outer_shift_m,
               std::clamp(node.declare_parameter<double>(
                              "turn_smoothing_max_outer_shift_m", 12.0),
                          0.0, 5000.0));
  config.trajectory_planner.turn_smoothing.min_corridor_margin_m = std::clamp(
      node.declare_parameter<double>("turn_smoothing_min_corridor_margin_m", 0.5), 0.0,
      1000.0);
  config.trajectory_planner.turn_smoothing.max_length_ratio = std::clamp(
      node.declare_parameter<double>("turn_smoothing_max_length_ratio", 1.25), 1.0,
      100.0);
  config.trajectory_planner.turn_smoothing.min_heading_improvement_rad =
      std::clamp(node.declare_parameter<double>(
                     "turn_smoothing_min_heading_improvement_deg", 3.0) *
                     std::numbers::pi / 180.0,
                 0.0, std::numbers::pi);
  config.trajectory_planner.turn_smoothing.max_passes =
      static_cast<std::size_t>(std::clamp<std::int64_t>(
          node.declare_parameter<std::int64_t>("turn_smoothing_max_passes", 8), 0,
          100));
  config.trajectory_planner.debug_sample_step_m = std::clamp(
      node.declare_parameter<double>("final_trajectory_debug_sample_step_m", 1.0), 0.1,
      20.0);

  config.initial_pose.use_until_px4 =
      node.declare_parameter<bool>("use_initial_pose_until_px4", true);
  config.initial_pose.heading_rad =
      node.declare_parameter<double>("initial_heading_rad", 0.0);
  config.initial_pose.px4_local_origin =
      Point2{node.declare_parameter<double>("px4_local_origin_x_m", 0.0),
             node.declare_parameter<double>("px4_local_origin_y_m", 0.0)};
  config.initial_pose.position =
      Point2{node.declare_parameter<double>("initial_x_m", 0.0),
             node.declare_parameter<double>("initial_y_m", 0.0)};

  config.topics.obstacle_memory_grid = node.declare_parameter<std::string>(
      "obstacle_memory_grid_topic", "/drone_city_nav/obstacle_memory_grid");
  config.topics.lidar = node.declare_parameter<std::string>("lidar_topic", "/scan");
  config.topics.local_position = node.declare_parameter<std::string>(
      "px4_local_position_topic", "/fmu/out/vehicle_local_position");
  config.topics.attitude = node.declare_parameter<std::string>(
      "px4_vehicle_attitude_topic", "/fmu/out/vehicle_attitude");
  config.topics.prohibited_grid = node.declare_parameter<std::string>(
      "prohibited_grid_topic", "/drone_city_nav/prohibited_grid");
  config.topics.static_map_grid = node.declare_parameter<std::string>(
      "static_map_grid_topic", "/drone_city_nav/static_map_grid");
  config.topics.static_map_points = node.declare_parameter<std::string>(
      "static_map_points_topic", "/drone_city_nav/static_map_points");
  config.timing.static_map_debug_publish_period_s = std::clamp(
      node.declare_parameter<double>("static_map_debug_publish_period_s", 1.0), 0.0,
      60.0);
  config.topics.path =
      node.declare_parameter<std::string>("path_topic", "/drone_city_nav/path");
  config.topics.path_id =
      node.declare_parameter<std::string>("path_id_topic", "/drone_city_nav/path_id");
  config.topics.trajectory_diagnostics = node.declare_parameter<std::string>(
      "trajectory_diagnostics_topic", "/drone_city_nav/trajectory_diagnostics");
  config.topics.current_waypoint = node.declare_parameter<std::string>(
      "current_waypoint_topic", "/drone_city_nav/current_waypoint");
  config.timing.path_prohibited_intersection_check_period_s =
      node.declare_parameter<double>("path_prohibited_intersection_check_period_s",
                                     0.5);
  config.planning_grid_builder.inflation_radius_m = config.inflation_radius_m;

  return config;
}

} // namespace drone_city_nav
