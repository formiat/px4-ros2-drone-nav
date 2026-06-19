#pragma once

#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/planning_grid_builder.hpp"
#include "drone_city_nav/static_map_source.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <string>

namespace drone_city_nav {

struct PlannerTopics {
  std::string obstacle_memory_grid{"/drone_city_nav/obstacle_memory_grid"};
  std::string lidar{"/scan"};
  std::string local_position{"/fmu/out/vehicle_local_position"};
  std::string attitude{"/fmu/out/vehicle_attitude"};
  std::string prohibited_grid{"/drone_city_nav/prohibited_grid"};
  std::string static_map_grid{"/drone_city_nav/static_map_grid"};
  std::string static_map_points{"/drone_city_nav/static_map_points"};
  std::string path{"/drone_city_nav/path"};
  std::string current_waypoint{"/drone_city_nav/current_waypoint"};
};

struct PlannerTimingConfig {
  std::int64_t max_pose_staleness_ns{1'000'000'000};
  std::int64_t max_current_lidar_staleness_ns{750'000'000};
  double static_map_debug_publish_period_s{1.0};
  double replan_period_s{0.5};
};

struct PlannerFallbackConfig {
  bool direct_path_fallback{false};
  bool reuse_last_valid_path_on_failure{false};
  bool stable_path_reuse_enabled{true};
  double max_initial_lateral_deviation_m{8.0};
};

struct PlannerInitialPoseConfig {
  bool use_until_px4{true};
  Point2 position{};
  double heading_rad{0.0};
  Point2 px4_local_origin{};
};

struct PlannerMemoryGridConfig {
  int occupied_value{100};
  int free_value{0};
};

struct PlannerCurrentLidarConfig {
  bool use_px4_heading_for_scan{false};
  double sensor_hit_depth_m{0.0};
};

struct PlannerNodeConfig {
  std::string frame_id{"map"};
  Point2 start{};
  Point2 goal{85.0, 0.0};
  double cruise_altitude_m{12.0};
  double inflation_radius_m{2.5};

  PlannerCoreConfig planner_core{};
  PlanningGridBuilderConfig planning_grid_builder{};
  PathSmoothingConfig path_smoothing{};
  LidarProjectionConfig lidar_projection{};
  StaticMapSourceConfig static_map{};
  PlannerTopics topics{};
  PlannerTimingConfig timing{};
  PlannerFallbackConfig fallback{};
  PlannerInitialPoseConfig initial_pose{};
  PlannerMemoryGridConfig memory_grid{};
  PlannerCurrentLidarConfig current_lidar{};
};

[[nodiscard]] PlannerNodeConfig loadPlannerNodeConfig(rclcpp::Node& node);

} // namespace drone_city_nav
