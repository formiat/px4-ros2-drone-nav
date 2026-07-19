#pragma once

#include "drone_city_nav/types.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace rclcpp {
class Node;
} // namespace rclcpp

namespace drone_city_nav {

struct LidarDebugNodeTopics {
  std::string lidar{"/scan"};
  std::string prohibited_grid{"/drone_city_nav/prohibited_grid"};
  std::string memory_grid{"/drone_city_nav/obstacle_memory_grid"};
  std::string path{"/drone_city_nav/final_trajectory_path"};
  std::string pointcloud{"/drone_city_nav/lidar_debug_points"};
  std::string raw_lidar_3d_pointcloud{"/drone_city_nav/raw_lidar_hit_points_3d"};
  std::string remembered_pointcloud{"/drone_city_nav/remembered_lidar_points"};
  std::string prohibited_pointcloud{"/drone_city_nav/prohibited_obstacle_points"};
  std::string raw_memory_pointcloud{"/drone_city_nav/raw_memory_obstacle_points"};
  std::string px4_local_position{"/fmu/out/vehicle_local_position_v1"};
  std::string px4_vehicle_attitude{"/fmu/out/vehicle_attitude"};
  std::string px4_timesync_status{"/fmu/out/timesync_status"};
};

struct LidarDebugNodeConfig {
  std::string output_dir{"log/lidar_debug"};
  double snapshot_period_s{1.0};
  int image_size_px{900};
  double view_radius_m{45.0};
  double max_lidar_range_m{35.0};
  double range_hit_epsilon_m{0.05};
  double initial_heading_rad{0.0};
  Point2 px4_local_origin{};
  double scan_yaw_offset_rad{0.0};
  bool motion_compensate_lidar_pose{true};
  double lidar_pose_latency_s{0.05};
  double lidar_scan_duration_override_s{0.0};
  bool compensate_lidar_attitude{true};
  double lidar_z_offset_m{0.0};
  double min_projected_lidar_altitude_m{0.0};
  double max_projected_lidar_altitude_m{100000.0};
  bool use_px4_heading_for_scan{true};
  double lidar_mount_roll_rad{0.0};
  double lidar_mount_pitch_rad{0.0};
  double lidar_mount_yaw_rad{0.0};
  bool use_full_lidar_extrinsic{true};
  Point3 lidar_translation_body_frd_m{0.12, 0.0, -0.315};
  std::array<double, 4> lidar_flu_to_body_frd_quaternion{0.0, 1.0, 0.0, 0.0};
  double hit_memory_resolution_m{0.25};
  double min_remember_altitude_m{0.0};
  double current_pointcloud_z_m{0.05};
  double remembered_pointcloud_z_m{0.05};
  double prohibited_pointcloud_z_m{0.05};
  double raw_memory_pointcloud_z_m{0.05};
  std::size_t beam_csv_stride{1U};
  std::size_t max_logged_hit_points{256U};
  std::size_t max_remembered_hit_points{50000U};
  std::uint64_t max_snapshots{0U};
  LidarDebugNodeTopics topics{};
};

void sanitizeLidarDebugNodeConfig(LidarDebugNodeConfig& config);

[[nodiscard]] LidarDebugNodeConfig loadLidarDebugNodeConfig(rclcpp::Node& node);

} // namespace drone_city_nav
