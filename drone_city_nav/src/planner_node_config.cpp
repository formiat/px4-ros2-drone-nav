#include "drone_city_nav/planner_node_config.hpp"

#include "drone_city_nav/grid_config.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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
  config.fallback.stable_path_reuse_enabled =
      node.declare_parameter<bool>("stable_path_reuse_enabled", true);
  config.planner_core.stable_path_reuse_max_deviation_m = std::clamp(
      node.declare_parameter<double>("stable_path_reuse_max_deviation_m", 3.0), 0.0,
      1000.0);
  config.planner_core.stable_path_goal_tolerance_m = std::clamp(
      node.declare_parameter<double>("stable_path_goal_tolerance_m", 3.0), 0.0, 1000.0);
  config.planner_core.nearest_free_radius_cells =
      static_cast<int>(std::clamp<std::int64_t>(
          node.declare_parameter<std::int64_t>("nearest_free_radius_cells", 10), 0,
          100000));
  config.memory_grid.occupied_value = static_cast<int>(std::clamp<std::int64_t>(
      node.declare_parameter<std::int64_t>("memory_occupied_value", 100), 1, 100));
  config.memory_grid.free_value = static_cast<int>(std::clamp<std::int64_t>(
      node.declare_parameter<std::int64_t>("memory_free_value", 0), 0, 100));

  config.static_map.enabled = node.declare_parameter<bool>("use_static_map", true);
  config.planning_grid_builder.use_static_map = config.static_map.enabled;
  config.planning_grid_builder.use_obstacle_memory =
      node.declare_parameter<bool>("use_obstacle_memory", true);
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
  config.planning_grid_builder.use_current_lidar_obstacles =
      node.declare_parameter<bool>("use_current_lidar_obstacles", true);
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
  config.planner_core.astar.evasive_maneuvering_enabled =
      node.declare_parameter<bool>("astar_evasive_maneuvering_enabled", false);
  config.planner_core.astar.evasive_maneuvering_straight_cost_weight =
      std::clamp(node.declare_parameter<double>(
                     "astar_evasive_maneuvering_straight_cost_weight", 1.0),
                 0.0, 1000.0);
  config.planner_core.clearance_diagnostic_radius_m = 10.0;

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
  config.topics.current_waypoint = node.declare_parameter<std::string>(
      "current_waypoint_topic", "/drone_city_nav/current_waypoint");
  config.timing.path_prohibited_intersection_check_period_s =
      node.declare_parameter<double>("path_prohibited_intersection_check_period_s",
                                     0.5);
  config.planning_grid_builder.inflation_radius_m = config.inflation_radius_m;

  return config;
}

} // namespace drone_city_nav
