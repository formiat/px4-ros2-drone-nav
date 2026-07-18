#include "drone_city_nav/lidar_debug_node_config.hpp"

#include <rclcpp/rclcpp.hpp>

#include <limits>

namespace drone_city_nav {

void sanitizeLidarDebugNodeConfig(LidarDebugNodeConfig& config) {
  config.snapshot_period_s = std::max(0.1, config.snapshot_period_s);
  config.image_size_px = static_cast<int>(std::clamp(config.image_size_px, 200, 4000));
  config.view_radius_m = std::max(5.0, config.view_radius_m);
  config.max_lidar_range_m = std::max(1.0, config.max_lidar_range_m);
  config.range_hit_epsilon_m = std::max(0.0, config.range_hit_epsilon_m);
  config.lidar_pose_latency_s = std::clamp(config.lidar_pose_latency_s, 0.0, 1.0);
  config.lidar_scan_duration_override_s =
      std::clamp(config.lidar_scan_duration_override_s, 0.0, 1.0);
  config.hit_memory_resolution_m = std::max(0.05, config.hit_memory_resolution_m);
  config.min_remember_altitude_m = std::max(0.0, config.min_remember_altitude_m);
  config.beam_csv_stride = std::clamp<std::size_t>(config.beam_csv_stride, 1U, 100000U);
  config.max_logged_hit_points =
      std::min<std::size_t>(config.max_logged_hit_points, 100000U);
  config.max_remembered_hit_points =
      std::clamp<std::size_t>(config.max_remembered_hit_points, 1U, 1000000U);
}

[[nodiscard]] LidarDebugNodeConfig loadLidarDebugNodeConfig(rclcpp::Node& node) {
  LidarDebugNodeConfig config{};
  config.output_dir =
      node.declare_parameter<std::string>("output_dir", config.output_dir);
  config.snapshot_period_s =
      node.declare_parameter<double>("snapshot_period_s", config.snapshot_period_s);
  config.image_size_px = static_cast<int>(
      node.declare_parameter<std::int64_t>("image_size_px", config.image_size_px));
  config.view_radius_m =
      node.declare_parameter<double>("view_radius_m", config.view_radius_m);
  config.max_lidar_range_m =
      node.declare_parameter<double>("max_lidar_range_m", config.max_lidar_range_m);
  config.range_hit_epsilon_m =
      node.declare_parameter<double>("range_hit_epsilon_m", config.range_hit_epsilon_m);
  config.initial_heading_rad =
      node.declare_parameter<double>("initial_heading_rad", config.initial_heading_rad);
  config.px4_local_origin =
      Point2{node.declare_parameter<double>("px4_local_origin_x_m", 0.0),
             node.declare_parameter<double>("px4_local_origin_y_m", 0.0)};
  config.scan_yaw_offset_rad =
      node.declare_parameter<double>("scan_yaw_offset_rad", config.scan_yaw_offset_rad);
  config.motion_compensate_lidar_pose = node.declare_parameter<bool>(
      "motion_compensate_lidar_pose", config.motion_compensate_lidar_pose);
  config.lidar_pose_latency_s = node.declare_parameter<double>(
      "lidar_pose_latency_s", config.lidar_pose_latency_s);
  config.lidar_scan_duration_override_s = node.declare_parameter<double>(
      "lidar_scan_duration_override_s", config.lidar_scan_duration_override_s);
  config.compensate_lidar_attitude = node.declare_parameter<bool>(
      "compensate_lidar_attitude", config.compensate_lidar_attitude);
  config.lidar_z_offset_m =
      node.declare_parameter<double>("lidar_z_offset_m", config.lidar_z_offset_m);
  config.min_projected_lidar_altitude_m = node.declare_parameter<double>(
      "min_projected_lidar_altitude_m", config.min_projected_lidar_altitude_m);
  config.max_projected_lidar_altitude_m = node.declare_parameter<double>(
      "max_projected_lidar_altitude_m", config.max_projected_lidar_altitude_m);
  config.use_px4_heading_for_scan = node.declare_parameter<bool>(
      "use_px4_heading_for_scan", config.use_px4_heading_for_scan);
  config.lidar_mount_roll_rad = node.declare_parameter<double>(
      "lidar_mount_roll_rad", config.lidar_mount_roll_rad);
  config.lidar_mount_pitch_rad = node.declare_parameter<double>(
      "lidar_mount_pitch_rad", config.lidar_mount_pitch_rad);
  config.lidar_mount_yaw_rad =
      node.declare_parameter<double>("lidar_mount_yaw_rad", config.lidar_mount_yaw_rad);
  config.beam_csv_stride = static_cast<std::size_t>(std::max<std::int64_t>(
      node.declare_parameter<std::int64_t>(
          "beam_csv_stride", static_cast<std::int64_t>(config.beam_csv_stride)),
      0));
  config.max_logged_hit_points = static_cast<std::size_t>(std::max<std::int64_t>(
      node.declare_parameter<std::int64_t>(
          "max_logged_hit_points",
          static_cast<std::int64_t>(config.max_logged_hit_points)),
      0));
  config.max_snapshots = static_cast<std::uint64_t>(
      std::clamp<std::int64_t>(node.declare_parameter<std::int64_t>("max_snapshots", 0),
                               0, std::numeric_limits<std::int32_t>::max()));

  config.topics.lidar =
      node.declare_parameter<std::string>("lidar_topic", config.topics.lidar);
  config.topics.prohibited_grid = node.declare_parameter<std::string>(
      "prohibited_grid_topic", config.topics.prohibited_grid);
  config.topics.memory_grid = node.declare_parameter<std::string>(
      "memory_grid_topic", config.topics.memory_grid);
  config.topics.path =
      node.declare_parameter<std::string>("path_topic", config.topics.path);
  config.topics.pointcloud =
      node.declare_parameter<std::string>("pointcloud_topic", config.topics.pointcloud);
  config.topics.raw_lidar_3d_pointcloud = node.declare_parameter<std::string>(
      "raw_lidar_3d_pointcloud_topic", config.topics.raw_lidar_3d_pointcloud);
  config.topics.remembered_pointcloud = node.declare_parameter<std::string>(
      "remembered_pointcloud_topic", config.topics.remembered_pointcloud);
  config.topics.prohibited_pointcloud = node.declare_parameter<std::string>(
      "prohibited_pointcloud_topic", config.topics.prohibited_pointcloud);
  config.topics.raw_memory_pointcloud = node.declare_parameter<std::string>(
      "raw_memory_pointcloud_topic", config.topics.raw_memory_pointcloud);
  config.hit_memory_resolution_m = node.declare_parameter<double>(
      "hit_memory_resolution_m", config.hit_memory_resolution_m);
  config.min_remember_altitude_m = node.declare_parameter<double>(
      "min_remember_altitude_m", config.min_remember_altitude_m);
  config.max_remembered_hit_points = static_cast<std::size_t>(std::max<std::int64_t>(
      node.declare_parameter<std::int64_t>(
          "max_remembered_hit_points",
          static_cast<std::int64_t>(config.max_remembered_hit_points)),
      0));
  config.current_pointcloud_z_m = node.declare_parameter<double>(
      "current_lidar_pointcloud_z_m", config.current_pointcloud_z_m);
  config.remembered_pointcloud_z_m = node.declare_parameter<double>(
      "remembered_lidar_pointcloud_z_m", config.remembered_pointcloud_z_m);
  config.prohibited_pointcloud_z_m = node.declare_parameter<double>(
      "prohibited_pointcloud_z_m", config.prohibited_pointcloud_z_m);
  config.raw_memory_pointcloud_z_m = node.declare_parameter<double>(
      "raw_memory_pointcloud_z_m", config.raw_memory_pointcloud_z_m);
  config.topics.px4_local_position = node.declare_parameter<std::string>(
      "px4_local_position_topic", config.topics.px4_local_position);
  config.topics.px4_vehicle_attitude = node.declare_parameter<std::string>(
      "px4_vehicle_attitude_topic", config.topics.px4_vehicle_attitude);

  sanitizeLidarDebugNodeConfig(config);
  return config;
}

} // namespace drone_city_nav
