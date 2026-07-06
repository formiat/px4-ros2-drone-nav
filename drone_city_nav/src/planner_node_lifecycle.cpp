#include "planner_node.hpp"

namespace drone_city_nav {

PlannerNode::PlannerNode()
    : Node{"planner_node"} {
  const PlannerNodeConfig config = loadPlannerNodeConfig(*this);
  applyConfig(config);
  if (config.initial_pose.use_until_px4) {
    current_pose_ =
        Pose2{config.initial_pose.position, config.initial_pose.heading_rad};
    pose_valid_ = true;
    last_pose_update_ns_ = get_clock()->now().nanoseconds();
  }

  const auto sensor_qos = rclcpp::SensorDataQoS{};
  memory_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      config.topics.obstacle_memory_grid, rclcpp::QoS{1}.transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        onMemoryGrid(*msg);
      });
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      config.topics.lidar, sensor_qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { onScan(*msg); });
  local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      config.topics.local_position, sensor_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        onLocalPosition(*msg);
      });
  attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      config.topics.attitude, sensor_qos,
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        onAttitude(*msg);
      });

  prohibited_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      config.topics.prohibited_grid, rclcpp::QoS{1}.transient_local());
  static_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      config.topics.static_map_grid, rclcpp::QoS{1}.transient_local());
  static_map_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      config.topics.static_map_points, rclcpp::QoS{1}.transient_local());
  path_pub_ = create_publisher<nav_msgs::msg::Path>(config.topics.path,
                                                    rclcpp::QoS{1}.reliable());
  path_id_pub_ = create_publisher<std_msgs::msg::UInt64>(config.topics.path_id,
                                                         rclcpp::QoS{1}.reliable());
  trajectory_diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
      config.topics.trajectory_diagnostics, rclcpp::QoS{1}.reliable());
  waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      config.topics.current_waypoint, rclcpp::QoS{1}.reliable());

  loadConfiguredStaticMap();
  if (static_map_debug_publish_period_s_ > 0.0) {
    static_map_debug_timer_ = create_wall_timer(
        std::chrono::duration<double>{static_map_debug_publish_period_s_},
        [this]() { republishStaticMapDebug(); });
  }
  timer_ = create_wall_timer(
      std::chrono::duration<double>{
          std::max(0.05, config.timing.path_prohibited_intersection_check_period_s)},
      [this]() { checkCurrentPathAndPublish(); });

  RCLCPP_INFO(get_logger(),
              "Planner ready: start=(%.1f, %.1f) goal=(%.1f, %.1f) "
              "runtime_inflation=%.2fm planning_clearance=%.2fm "
              "planning_effective_inflation=%.2fm",
              start_.x, start_.y, goal_.x, goal_.y, inflation_radius_m_,
              planning_clearance_m_, inflation_radius_m_ + planning_clearance_m_);
  RCLCPP_INFO(get_logger(),
              "Planner subscriptions: obstacle_memory_grid='%s' local_position='%s' "
              "attitude='%s'",
              config.topics.obstacle_memory_grid.c_str(),
              config.topics.local_position.c_str(), config.topics.attitude.c_str());
  RCLCPP_INFO(get_logger(), "Planner publications: path='%s' path_id='%s'",
              config.topics.path.c_str(), config.topics.path_id.c_str());
  RCLCPP_INFO(get_logger(),
              "Planning grid contract: raw_sources=[static,memory,current_lidar] "
              "runtime_inflation=%.2fm planning_clearance=%.2fm "
              "planning_effective_inflation=%.2fm prohibited_output='%s'",
              inflation_radius_m_, planning_clearance_m_,
              inflation_radius_m_ + planning_clearance_m_,
              config.topics.prohibited_grid.c_str());
  RCLCPP_INFO(
      get_logger(),
      "Planner trajectory pipeline: output_path=final_optimized_trajectory "
      "rough_astar_scope=internal_seed "
      "speed[cruise=%.2fmps min_turn=%.2fmps profile_accel=%.2fmps2 "
      "profile_decel=%.2fmps2 turn_lateral=%.2fmps2 "
      "sample_step=%.2fm] "
      "corridor[max_radius=%.2fm sample_step=%.2fm center_recovery_max=%.2fm "
      "lateral_window=%.2fm lateral_ratio=%.2f lateral_margin=%.2fm "
      "parallel_workers=%zu] "
      "trajectory_optimizer[iterations=%zu optimizer_sample_step=%.2fm "
      "offset_step=%.2fm "
      "min_step=%.2fm weights(curvature=%.2f curvature_change=%.2f "
      "preferred_radius=%.2fm "
      "radius_shortfall=%.2f offset_change=%.2f "
      "offset_second=%.2f offset_slope=%.2f max_offset_slope=%.2f/m "
      "parallel=always parallel_workers=%zu "
      "window(pre=%.2fm post=%.2fm heading=%.1fdeg width=%.2fm) "
      "dp_offset_step=%.2fm async_workers=%zu)] "
      "turn_smoothing[trigger_heading=%.1fdeg trigger_radius=%.2fm "
      "trigger_speed=%.2fmps entry=%.2fm exit=%.2fm sample_step=%.2fm outer_bias=%.2f "
      "outer_shift=[%.2f, %.2f] max_passes=%zu]",
      trajectory_planner_config_.speed_profile.cruise_speed_mps,
      trajectory_planner_config_.speed_profile.min_turn_speed_mps,
      trajectory_planner_config_.speed_profile.speed_profile_accel_mps2,
      trajectory_planner_config_.speed_profile.speed_profile_decel_mps2,
      trajectory_planner_config_.speed_profile.turn_speed_lateral_accel_mps2,
      trajectory_planner_config_.speed_profile.speed_profile_sample_step_m,
      trajectory_planner_config_.corridor.max_radius_m,
      trajectory_planner_config_.corridor.sample_step_m,
      trajectory_planner_config_.corridor.center_recovery_max_m,
      trajectory_planner_config_.corridor.lateral_limit_window_m,
      trajectory_planner_config_.corridor.lateral_limit_ratio,
      trajectory_planner_config_.corridor.lateral_limit_margin_m,
      trajectory_planner_config_.corridor.parallel_workers,
      trajectory_planner_config_.trajectory_optimizer.max_iterations,
      trajectory_planner_config_.trajectory_optimizer.optimizer_sample_step_m,
      trajectory_planner_config_.trajectory_optimizer.initial_offset_step_m,
      trajectory_planner_config_.trajectory_optimizer.min_offset_step_m,
      trajectory_planner_config_.trajectory_optimizer.weight_curvature,
      trajectory_planner_config_.trajectory_optimizer.weight_curvature_change,
      trajectory_planner_config_.trajectory_optimizer.preferred_min_radius_m,
      trajectory_planner_config_.trajectory_optimizer.weight_radius_shortfall,
      trajectory_planner_config_.trajectory_optimizer.weight_offset_change,
      trajectory_planner_config_.trajectory_optimizer.weight_offset_second_change,
      trajectory_planner_config_.trajectory_optimizer.weight_offset_slope,
      trajectory_planner_config_.trajectory_optimizer.max_offset_slope_per_m,
      trajectory_planner_config_.trajectory_optimizer.parallel_workers,
      trajectory_planner_config_.trajectory_optimizer.window_pre_margin_m,
      trajectory_planner_config_.trajectory_optimizer.window_post_margin_m,
      radiansToDegrees(
          trajectory_planner_config_.trajectory_optimizer.window_heading_threshold_rad),
      trajectory_planner_config_.trajectory_optimizer.window_width_change_threshold_m,
      trajectory_planner_config_.trajectory_optimizer.dp_offset_step_m,
      trajectory_planner_config_.trajectory_optimizer.async_refinement_workers,
      radiansToDegrees(
          trajectory_planner_config_.turn_smoothing.trigger_heading_delta_rad),
      trajectory_planner_config_.turn_smoothing.trigger_min_radius_m,
      trajectory_planner_config_.turn_smoothing.trigger_speed_limit_mps,
      trajectory_planner_config_.turn_smoothing.entry_distance_m,
      trajectory_planner_config_.turn_smoothing.exit_distance_m,
      trajectory_planner_config_.turn_smoothing.sample_step_m,
      trajectory_planner_config_.turn_smoothing.outer_bias_ratio,
      trajectory_planner_config_.turn_smoothing.min_outer_shift_m,
      trajectory_planner_config_.turn_smoothing.max_outer_shift_m,
      trajectory_planner_config_.turn_smoothing.max_passes);
  RCLCPP_INFO(get_logger(),
              "Planner obstacle sources: static=%s memory=always current_lidar=always "
              "static_path='%s' fallback_grid=%dx%d@%.2fm origin=(%.2f, %.2f)",
              use_static_map_ ? "true" : "false",
              static_map_resolved_path_.string().c_str(),
              fallback_grid_bounds_.width_cells, fallback_grid_bounds_.height_cells,
              fallback_grid_bounds_.resolution_m, fallback_grid_bounds_.origin_x,
              fallback_grid_bounds_.origin_y);
  RCLCPP_INFO(get_logger(),
              "Planner lidar overlay: enabled=always topic='%s' max_range=%.2f "
              "max_staleness=%.2fs yaw_source=%s compensate_attitude=%s "
              "motion_compensation=%s pose_latency=%.3fs "
              "lidar_z_offset=%.2f "
              "projected_altitude_range=[%.2f, %.2f] "
              "lidar_mount_rpy=(%.3f, %.3f, %.3f)",
              config.topics.lidar.c_str(), max_lidar_range_m_,
              static_cast<double>(max_current_lidar_staleness_ns_) / 1.0e9,
              use_px4_heading_for_scan_ ? "px4_heading" : "initial_map_aligned",
              compensate_lidar_attitude_ ? "true" : "false",
              motion_compensate_lidar_pose_ ? "true" : "false", lidar_pose_latency_s_,
              lidar_z_offset_m_, min_projected_lidar_altitude_m_,
              max_projected_lidar_altitude_m_, lidar_mount_roll_rad_,
              lidar_mount_pitch_rad_, lidar_mount_yaw_rad_);
  RCLCPP_INFO(get_logger(),
              "Planner path policy: stable_path_reuse=always "
              "stable_goal_tolerance=%.2fm default_trajectory_altitude=%.2fm",
              stable_path_goal_tolerance_m_, cruise_altitude_m_);
  RCLCPP_INFO(get_logger(),
              "Planner path preference: astar_heuristic_weight=%.2f "
              "astar_turn_weight=%.2f "
              "evasive_maneuvering=%s evasive_straight_weight=%.2f "
              "initial_heading_bias=%s initial_heading_min_speed=%.2fm/s "
              "initial_heading_weight=%.2f",
              astar_config_.heuristic_weight, astar_config_.turn_cost_weight,
              astar_config_.evasive_maneuvering_enabled ? "true" : "false",
              astar_config_.evasive_maneuvering_straight_cost_weight,
              astar_config_.initial_heading_bias_enabled ? "true" : "false",
              astar_config_.initial_heading_bias_min_speed_mps,
              astar_config_.initial_heading_bias_weight);
}

void PlannerNode::applyConfig(const PlannerNodeConfig& config) {
  frame_id_ = config.frame_id;
  start_ = config.start;
  goal_ = config.goal;
  cruise_altitude_m_ = config.cruise_altitude_m;
  inflation_radius_m_ = config.inflation_radius_m;
  planning_clearance_m_ = config.planning_clearance_m;
  max_pose_staleness_ns_ = config.timing.max_pose_staleness_ns;
  stable_path_goal_tolerance_m_ = config.planner_core.stable_path_goal_tolerance_m;
  memory_occupied_value_ = config.memory_grid.occupied_value;
  memory_free_value_ = config.memory_grid.free_value;
  use_static_map_ = config.static_map.enabled;
  static_map_path_param_ = config.static_map.configured_path.string();
  static_map_min_blocking_height_m_ = config.static_map.min_blocking_height_m;
  fallback_grid_bounds_ = config.planning_grid_builder.fallback_bounds;
  max_current_lidar_staleness_ns_ = config.timing.max_current_lidar_staleness_ns;
  max_lidar_range_m_ = config.lidar_projection.max_lidar_range_m;
  range_hit_epsilon_m_ = config.lidar_projection.range_hit_epsilon_m;
  scan_yaw_offset_rad_ = config.lidar_projection.scan_yaw_offset_rad;
  use_px4_heading_for_scan_ = config.current_lidar.use_px4_heading_for_scan;
  motion_compensate_lidar_pose_ = config.current_lidar.motion_compensate_lidar_pose;
  lidar_pose_latency_s_ = config.current_lidar.lidar_pose_latency_s;
  compensate_lidar_attitude_ = config.lidar_projection.compensate_attitude;
  lidar_mount_roll_rad_ = config.lidar_projection.lidar_mount_roll_rad;
  lidar_mount_pitch_rad_ = config.lidar_projection.lidar_mount_pitch_rad;
  lidar_mount_yaw_rad_ = config.lidar_projection.lidar_mount_yaw_rad;
  lidar_z_offset_m_ = config.lidar_projection.lidar_z_offset_m;
  min_projected_lidar_altitude_m_ = config.lidar_projection.min_projected_altitude_m;
  max_projected_lidar_altitude_m_ = config.lidar_projection.max_projected_altitude_m;
  astar_config_ = config.planner_core.astar;
  trajectory_planner_config_ = config.trajectory_planner;
  refinement_scheduler_.configure(
      trajectory_planner_config_.trajectory_optimizer.async_refinement_workers);
  initial_heading_rad_ = config.initial_pose.heading_rad;
  px4_local_origin_ = config.initial_pose.px4_local_origin;
  static_map_debug_publish_period_s_ = config.timing.static_map_debug_publish_period_s;
  planner_core_.setConfig(config.planner_core);
}

} // namespace drone_city_nav
