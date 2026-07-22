#pragma once

#include "drone_city_nav/ambiguous_lidar_hit_tracker.hpp"
#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/lidar_ingestion_decision.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/path_raw_clearance_monitor.hpp"
#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/planning_grid_builder.hpp"
#include "drone_city_nav/static_map_source.hpp"
#include "drone_city_nav/trajectory_planner.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <cstdint>
#include <string>

namespace drone_city_nav {

struct PlannerTopics {
  std::string obstacle_memory_snapshot{"/drone_city_nav/obstacle_memory_snapshot"};
  std::string lidar{"/scan"};
  std::string local_position{"/fmu/out/vehicle_local_position"};
  std::string attitude{"/fmu/out/vehicle_attitude"};
  std::string timesync_status{"/fmu/out/timesync_status"};
  std::string prohibited_grid{"/drone_city_nav/prohibited_grid"};
  std::string static_map_grid{"/drone_city_nav/static_map_grid"};
  std::string static_map_points{"/drone_city_nav/static_map_points"};
  std::string static_building_markers{"/drone_city_nav/static_building_markers"};
  std::string known_passage_markers{"/drone_city_nav/known_passage_markers"};
  std::string path{"/drone_city_nav/path"};
  std::string path_id{"/drone_city_nav/path_id"};
  std::string trajectory_diagnostics{"/drone_city_nav/trajectory_diagnostics"};
  std::string replan_blocker{"/drone_city_nav/replan_blocker"};
  std::string replan_truncation{"/drone_city_nav/replan_truncation"};
  std::string truncation_suffix_ack{"/drone_city_nav/truncation_suffix_ack"};
  std::string executable_trajectory{"/drone_city_nav/executable_trajectory"};
  std::string current_waypoint{"/drone_city_nav/current_waypoint"};
};

struct PlannerTimingConfig {
  std::int64_t max_pose_staleness_ns{1'000'000'000};
  std::int64_t max_current_lidar_staleness_ns{750'000'000};
  double static_map_debug_publish_period_s{1.0};
  double known_passage_debug_publish_period_s{1.0};
  double path_prohibited_intersection_check_period_s{0.5};
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

struct PlannerMemorySnapshotTransportConfig {
  double diagnostic_period_s{5.0};
  double max_age_ms{350.0};
  double max_callback_time_ms{100.0};
  double max_apply_delay_ms{300.0};
  double min_apply_rate_hz{1.0};
};

struct PlannerCurrentLidarConfig {
  bool use_px4_heading_for_scan{true};
  bool motion_compensate_lidar_pose{true};
  double lidar_pose_latency_s{0.05};
  AmbiguousLidarHitTrackerConfig ambiguous_hit_confirmation{};
  LidarIngestionConfidenceConfig ingestion_confidence{};
};

struct PlannerNodeConfig {
  std::string frame_id{"map"};
  Point2 start{};
  Point2 goal{85.0, 0.0};
  double initial_altitude_m{12.0};
  double inflation_radius_m{1.0};
  double planning_clearance_m{3.0};
  double no_static_planning_clearance_m{5.0};
  double local_inflation_relaxation_radius_m{5.0};
  PlannerCoreConfig planner_core{};
  TrajectoryPlannerConfig trajectory_planner{};
  PlanningGridBuilderConfig planning_grid_builder{};
  LidarProjectionConfig lidar_projection{};
  StaticMapSourceConfig static_map{};
  KnownPassageSourceConfig known_passages{};
  KnownPassageValidationConfig known_passage_validation{};
  double known_static_lidar_hit_closer_range_tolerance_m{0.5};
  double known_static_lidar_hit_farther_range_tolerance_m{1.5};
  double known_static_lidar_hit_endpoint_volume_tolerance_m{0.75};
  double known_static_opening_boundary_tolerance_m{0.30};
  GroundLidarRejectionConfig ground_lidar_rejection{};
  bool safe_trajectory_truncation_enabled{true};
  PathRawClearanceMonitorConfig path_raw_clearance_monitor{};
  PlannerTopics topics{};
  PlannerTimingConfig timing{};
  PlannerInitialPoseConfig initial_pose{};
  PlannerMemoryGridConfig memory_grid{};
  PlannerMemorySnapshotTransportConfig memory_snapshot_transport{};
  PlannerCurrentLidarConfig current_lidar{};
};

[[nodiscard]] PlannerNodeConfig loadPlannerNodeConfig(rclcpp::Node& node);

} // namespace drone_city_nav
