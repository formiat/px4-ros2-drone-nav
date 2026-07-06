#include <memory>

#include "planner_node.hpp"

namespace drone_city_nav {

void PlannerNode::onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg) {
  if (!msg.xy_valid || !std::isfinite(msg.x) || !std::isfinite(msg.y)) {
    invalidateCurrentPose();
    current_velocity_ = Point2{};
    current_speed_mps_ = std::numeric_limits<double>::quiet_NaN();
    current_velocity_valid_ = false;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planner invalidated cached pose after invalid PX4 local position: "
        "xy_valid=%s x=%.2f y=%.2f",
        msg.xy_valid ? "true" : "false", static_cast<double>(msg.x),
        static_cast<double>(msg.y));
    return;
  }

  current_pose_.position = Point2{static_cast<double>(msg.x) + px4_local_origin_.x,
                                  static_cast<double>(msg.y) + px4_local_origin_.y};
  if (msg.heading_good_for_control && std::isfinite(msg.heading)) {
    current_pose_.yaw_rad = static_cast<double>(msg.heading);
  }
  if (msg.z_valid && std::isfinite(msg.z)) {
    current_altitude_m_ = -static_cast<double>(msg.z);
    altitude_valid_ = true;
  } else {
    altitude_valid_ = false;
  }
  if (msg.v_xy_valid && std::isfinite(msg.vx) && std::isfinite(msg.vy)) {
    current_velocity_ =
        Point2{static_cast<double>(msg.vx), static_cast<double>(msg.vy)};
    current_speed_mps_ = std::hypot(current_velocity_.x, current_velocity_.y);
    current_velocity_valid_ = true;
  } else {
    current_velocity_ = Point2{};
    current_speed_mps_ = std::numeric_limits<double>::quiet_NaN();
    current_velocity_valid_ = false;
  }
  pose_valid_ = true;
  last_pose_update_ns_ = get_clock()->now().nanoseconds();

  if (!local_position_seen_) {
    local_position_seen_ = true;
    const double altitude_m = (msg.z_valid && std::isfinite(msg.z))
                                  ? -static_cast<double>(msg.z)
                                  : std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(get_logger(),
                "First valid PX4 local position: x=%.2f y=%.2f z=%.2f "
                "altitude=%.2f yaw=%.2f distance_to_start=%.2f "
                "distance_to_goal=%.2f",
                current_pose_.position.x, current_pose_.position.y,
                static_cast<double>(msg.z), altitude_m, current_pose_.yaw_rad,
                distance(current_pose_.position, start_),
                distance(current_pose_.position, goal_));
  }
}

void PlannerNode::onMemoryGrid(const nav_msgs::msg::OccupancyGrid& msg) {
  RawOccupancyGridFromRosResult converted = rawOccupancyGridFromRos(
      msg, RawOccupancyGridFromRosConfig{memory_occupied_value_, memory_free_value_});
  if (!converted.grid.has_value()) {
    if (converted.error == OccupancyGridFromRosError::kMismatchedDataSize) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring obstacle memory grid with mismatched data size: expected=%zu "
          "got=%zu",
          converted.expected_data_size, converted.actual_data_size);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring invalid obstacle memory grid metadata");
    }
    return;
  }

  memory_grid_ = std::move(*converted.grid);
  if (converted.intermediate_value_cells > 0U) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignored intermediate values while reading raw obstacle memory grid: "
        "intermediate_cells=%zu. Raw memory topics must use only occupied=%d, "
        "free=%d, or unknown=-1 values.",
        converted.intermediate_value_cells, memory_occupied_value_, memory_free_value_);
  }
  if (!memory_grid_seen_) {
    memory_grid_seen_ = true;
    RCLCPP_INFO(get_logger(),
                "First obstacle memory grid: size=%dx%d resolution=%.2f origin=(%.2f, "
                "%.2f)",
                memory_grid_->width(), memory_grid_->height(),
                memory_grid_->resolution(), memory_grid_->originX(),
                memory_grid_->originY());
  }
}

void PlannerNode::onScan(const sensor_msgs::msg::LaserScan& msg) {
  last_scan_ = msg;
  scan_seen_ = true;
  last_scan_update_ns_ = get_clock()->now().nanoseconds();
  last_scan_projection_pose_valid_ = pose_valid_;
  if (last_scan_projection_pose_valid_) {
    last_scan_projection_pose_ = currentLidarProjectionPose();
    const LidarPoseMotionCompensationResult motion_compensation =
        compensateLidarPoseForLatency(last_scan_projection_pose_.position,
                                      current_velocity_, motion_compensate_lidar_pose_,
                                      current_velocity_valid_,
                                      currentLidarPoseReceiveLagSeconds(
                                          last_scan_update_ns_, last_pose_update_ns_),
                                      lidar_pose_latency_s_);
    last_scan_projection_pose_.position = motion_compensation.position;
    last_scan_pose_lag_s_ = motion_compensation.pose_lag_s;
    last_scan_pose_latency_s_ = motion_compensation.latency_s;
    last_scan_motion_shift_ = motion_compensation.applied_shift;
    last_scan_motion_shift_m_ = motion_compensation.applied_shift_m;
  } else {
    last_scan_pose_lag_s_ = 0.0;
    last_scan_pose_latency_s_ = 0.0;
    last_scan_motion_shift_ = Point2{};
    last_scan_motion_shift_m_ = 0.0;
  }
  if (!scan_seen_logged_) {
    scan_seen_logged_ = true;
    RCLCPP_INFO(get_logger(),
                "First planner lidar scan: beams=%zu range=[%.2f, %.2f] "
                "angle=[%.2f, %.2f] projection_pose=%s pose_lag=%.3fs "
                "pose_latency=%.3fs motion_shift=(%.2f, %.2f) "
                "motion_shift_m=%.2f",
                last_scan_.ranges.size(), static_cast<double>(last_scan_.range_min),
                static_cast<double>(last_scan_.range_max),
                static_cast<double>(last_scan_.angle_min),
                static_cast<double>(last_scan_.angle_max),
                last_scan_projection_pose_valid_ ? "true" : "false",
                last_scan_pose_lag_s_, last_scan_pose_latency_s_,
                last_scan_motion_shift_.x, last_scan_motion_shift_.y,
                last_scan_motion_shift_m_);
  }
}

void PlannerNode::onAttitude(const px4_msgs::msg::VehicleAttitude& msg) {
  const auto euler = quaternionToEuler(msg.q);
  if (!euler.has_value()) {
    attitude_valid_ = false;
    return;
  }

  current_attitude_ = *euler;
  attitude_valid_ = true;
}

[[nodiscard]] std::filesystem::path
PlannerNode::staticMapPackageShareDirectory() const {
  try {
    return std::filesystem::path{
        ament_index_cpp::get_package_share_directory("drone_city_nav")};
  } catch (const std::exception&) {
    return {};
  }
}

void PlannerNode::loadConfiguredStaticMap() {
  StaticMapSourceResult result = loadStaticMapSource(StaticMapSourceConfig{
      use_static_map_, static_map_path_param_, staticMapPackageShareDirectory(),
      frame_id_, static_map_min_blocking_height_m_});
  static_map_resolved_path_ = result.resolved_path;

  if (result.status == StaticMapSourceStatus::kDisabled) {
    RCLCPP_INFO(get_logger(), "Static city map source is disabled");
    return;
  }

  if (result.status == StaticMapSourceStatus::kLoadFailed || !result.grid.has_value()) {
    static_grid_.reset();
    static_map_rectangles_ = 0U;
    static_map_occupied_cells_ = 0U;
    RCLCPP_ERROR(get_logger(), "Failed to load static city map: path='%s' error='%s'",
                 static_map_resolved_path_.string().c_str(),
                 result.error_message.c_str());
    return;
  }

  if (!result.frame_matches) {
    RCLCPP_WARN(get_logger(),
                "Static city map frame differs from planner frame: map='%s' "
                "planner='%s'",
                result.map_frame_id.c_str(), frame_id_.c_str());
  }
  static_grid_ = std::move(result.grid);
  static_map_rectangles_ = result.rectangles;
  static_map_occupied_cells_ = result.occupied_cells;
  RCLCPP_INFO(get_logger(),
              "Static city map loaded: path='%s' frame='%s' rectangles=%zu "
              "occupied_cells=%zu grid=%dx%d@%.2fm origin=(%.2f, %.2f) "
              "min_blocking_height=%.2f",
              static_map_resolved_path_.string().c_str(), result.map_frame_id.c_str(),
              static_map_rectangles_, static_map_occupied_cells_, static_grid_->width(),
              static_grid_->height(), static_grid_->resolution(),
              static_grid_->originX(), static_grid_->originY(),
              static_map_min_blocking_height_m_);
  publishStaticMapDebug(*static_grid_, true);
}

void PlannerNode::loadConfiguredKnownPassages() {
  const KnownPassageSourceResult result = loadKnownPassageMapSource(
      KnownPassageSourceConfig{use_known_passages_, known_passages_path_param_,
                               staticMapPackageShareDirectory(), frame_id_});
  known_passages_resolved_path_ = result.resolved_path;

  if (result.status == KnownPassageSourceStatus::kDisabled) {
    known_passages_.reset();
    known_passage_structures_ = 0U;
    known_passage_openings_ = 0U;
    RCLCPP_INFO(get_logger(), "Known passage map source is disabled");
    publishKnownPassageDebug(true);
    return;
  }

  if (result.status == KnownPassageSourceStatus::kLoadFailed ||
      !result.map.has_value()) {
    known_passages_.reset();
    known_passage_structures_ = 0U;
    known_passage_openings_ = 0U;
    RCLCPP_ERROR(get_logger(),
                 "Failed to load known passage map: path='%s' status=%s error='%s'",
                 known_passages_resolved_path_.string().c_str(),
                 knownPassageSourceStatusName(result.status),
                 result.error_message.c_str());
    publishKnownPassageDebug(true);
    return;
  }

  if (!result.frame_matches) {
    RCLCPP_WARN(get_logger(),
                "Known passage map frame differs from planner frame: map='%s' "
                "planner='%s'",
                result.map->frame_id.c_str(), frame_id_.c_str());
  }

  known_passages_ = result.map;
  known_passage_structures_ = result.structures;
  known_passage_openings_ = result.openings;
  RCLCPP_INFO(get_logger(),
              "Known passage map loaded: path='%s' status=%s frame='%s' "
              "structures=%zu openings=%zu markers_topic='%s'",
              known_passages_resolved_path_.string().c_str(),
              knownPassageSourceStatusName(result.status),
              known_passages_->frame_id.c_str(), known_passage_structures_,
              known_passage_openings_,
              known_passage_markers_pub_ ? known_passage_markers_pub_->get_topic_name()
                                         : "<unavailable>");
  publishKnownPassageDebug(true);
}

[[nodiscard]] PlanningGridBuilderConfig PlannerNode::planningGridBuilderConfig() const {
  PlanningGridBuilderConfig config{};
  config.use_static_map = use_static_map_;
  config.fallback_bounds = fallback_grid_bounds_;
  config.inflation_radius_m = inflation_radius_m_;
  config.planning_clearance_m = planning_clearance_m_;
  return config;
}

[[nodiscard]] std::optional<PlanningGridBuildResult>
PlannerNode::buildPlanningGrid(const std::int64_t now_ns) {
  const PlanningGridBuilderConfig config = planningGridBuilderConfig();
  PlanningGridSources sources{};
  sources.static_grid = static_grid_ ? &*static_grid_ : nullptr;
  sources.static_rectangles = static_map_rectangles_;
  sources.static_occupied_cells = static_map_occupied_cells_;
  sources.static_map_path = static_map_resolved_path_.string();
  sources.memory_grid = memory_grid_ ? &*memory_grid_ : nullptr;

  std::optional<OccupancyGrid2D> current_lidar_grid;
  if (const std::optional<GridBounds> bounds =
          selectPlanningGridBounds(config, sources);
      bounds.has_value()) {
    current_lidar_grid.emplace(*bounds);
    sources.current_lidar = overlayCurrentLidarHits(*current_lidar_grid, now_ns);
    sources.current_lidar_grid = &*current_lidar_grid;
  }

  PlanningGridBuildResult result = planning_grid_builder_.build(config, sources);
  if (current_lidar_grid.has_value()) {
    result.current_lidar_grid = std::move(current_lidar_grid);
  }
  if (result.memory.enabled && !result.memory.seen) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Obstacle memory source is enabled but no grid has been "
                         "received yet");
  } else if (result.memory.seen && memory_grid_.has_value() &&
             !result.memory.geometry_matches) {
    const OccupancyGrid2D& memory_grid = *memory_grid_;
    std::optional<OccupancyGrid2D> diagnostic_grid;
    const OccupancyGrid2D* planning_grid = result.grid ? &*result.grid : nullptr;
    if (planning_grid == nullptr) {
      if (const std::optional<GridBounds> bounds =
              selectPlanningGridBounds(config, sources);
          bounds.has_value()) {
        diagnostic_grid.emplace(*bounds);
        planning_grid = &*diagnostic_grid;
      }
    }
    if (planning_grid != nullptr) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Skipping obstacle memory overlay due to grid geometry mismatch: "
          "planning=%dx%d@%.2f origin=(%.2f, %.2f) memory=%dx%d@%.2f "
          "origin=(%.2f, %.2f)",
          planning_grid->width(), planning_grid->height(), planning_grid->resolution(),
          planning_grid->originX(), planning_grid->originY(), memory_grid.width(),
          memory_grid.height(), memory_grid.resolution(), memory_grid.originX(),
          memory_grid.originY());
    }
  }

  const PlannerGridReadinessDecision grid_readiness =
      evaluatePlannerGridReadiness(result);
  if (grid_readiness.reason == PlannerGridReadinessReason::kStaticMapMissing) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Planner static map source is enabled but not loaded; "
                         "skipping path check");
    return std::nullopt;
  }
  if (!grid_readiness.ready) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Planner has no ready obstacle source data; skipping path "
                         "check status=%s",
                         planningGridStatusName(result.status));
    return std::nullopt;
  }

  return result;
}

void PlannerNode::checkCurrentPathAndPublish() {
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  const bool pose_fresh =
      timestampIsFresh(last_pose_update_ns_, now_ns, max_pose_staleness_ns_);
  const double pose_age_s = poseAgeSeconds(now_ns);
  const PlannerRuntimeReadinessDecision runtime_readiness =
      evaluatePlannerRuntimeReadiness(PlannerRuntimeReadinessInput{
          pose_valid_, finite2D(current_pose_.position), pose_fresh});
  if (runtime_readiness.reason == PlannerRuntimeReadinessReason::kStalePose) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planner skipped path check because PX4 local position is stale; keeping the "
        "last published path: pose_age_s=%.2f",
        pose_age_s);
    return;
  }
  if (!runtime_readiness.ready) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Planner is waiting for a valid PX4 local position; "
                         "keeping the last published path");
    return;
  }
  const auto planning_grid_started_at = std::chrono::steady_clock::now();
  auto planning_result = buildPlanningGrid(now_ns);
  const double planning_grid_duration_ms =
      elapsedMilliseconds(planning_grid_started_at);
  if (!planning_result.has_value()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planner skipped path publication because the planning grid is "
        "not ready; keeping the last published path");
    return;
  }
  if (!planning_result->grid.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Planner grid builder returned no grid despite a ready "
                         "result; keeping the last published path");
    return;
  }
  if (!planning_result->planning_grid.has_value()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planner grid builder returned no planning-clearance grid despite a ready "
        "result; keeping the last published path");
    return;
  }
  OccupancyGrid2D prohibited_grid = std::move(*planning_result->grid);
  OccupancyGrid2D planning_grid = std::move(*planning_result->planning_grid);
  publishProhibitedGrid(prohibited_grid);
  if (pollPendingTrajectoryRefinement(prohibited_grid)) {
    return;
  }
  if (keepCurrentPathIfStillClear(prohibited_grid, *planning_result)) {
    return;
  }

  const AStarConfig planning_astar_config = astarConfigForCurrentVelocity();
  auto path_result =
      computePathOnGrid(planning_grid, "planning_clearance", planning_astar_config);
  if (!path_result.has_value()) {
    publishPlanningFailureHold();
    return;
  }
  const GridStats prohibited_grid_stats = collectGridStats(prohibited_grid);
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Planning summary: pose=(%.2f, %.2f) distance_to_start=%.2f "
      "distance_to_goal=%.2f raw_static=%zu raw_memory=%zu "
      "raw_current_lidar=%zu "
      "runtime_prohibited[prohibited=%zu occupied=%zu inflated=%zu free=%zu "
      "unknown=%zu inflation_radius_m=%.2f] "
      "planning_grid[prohibited=%zu occupied=%zu inflated=%zu free=%zu "
      "unknown=%zu planning_clearance_m=%.2f effective_inflation_m=%.2f] "
      "inflation_owner=planner "
      "static_grid_cache[eligible=%s hit=%s rebuilt=%s "
      "static_distance=%.1fms static_masks=%.1fms dynamic_distance=%.1fms "
      "dynamic_masks=%.1fms static_sources=%zu dynamic_sources=%zu] "
      "static[enabled=%s loaded=%s used=%s rectangles=%zu occupied_cells=%zu "
      "path='%s'] "
      "memory[enabled=%s seen=%s used=%s geometry_matches=%s occupied=%zu free=%zu "
      "unknown=%zu overlay_occupied=%zu overlay_free=%zu] "
      "current_lidar[enabled=%s used=%s fresh=%s processed=%zu hits=%zu "
      "altitude_rejected=%zu occupied_cells=%zu overlay_applied=%zu "
      "overlay_preserved=%zu outside=%zu] "
      "source=planning_clearance astar_status=%s heuristic_weight=%.2f expanded=%zu "
      "cost=%.2f raw_path=%zu smoothed_path=%zu "
      "initial_heading_bias[enabled=%s active=%s speed=%.2f min_speed=%.2f "
      "weight=%.2f velocity=(%.2f, %.2f)] "
      "path_metrics[raw_segments=%zu raw_straight_segments=%zu raw_turns=%zu "
      "raw_length=%.2f smoothed_segments=%zu smoothed_straight_segments=%zu "
      "smoothed_turns=%zu smoothed_length=%.2f] "
      "planning_path_clearance[raw=%.2f smoothed=%.2f] "
      "timing[grid=%.1f path_total=%.1f astar=%.1f smoothing=%.1f "
      "core_breakdown[grid_stats=%.1f raw_metrics=%.1f smoothed_metrics=%.1f "
      "clearance_field=%.1f clearance_cache_hit=%s raw_clearance=%.1f "
      "smoothed_clearance=%.1f]]",
      current_pose_.position.x, current_pose_.position.y,
      distance(current_pose_.position, start_), distance(current_pose_.position, goal_),
      planning_result->static_source.occupied_cells,
      planning_result->memory.source_counts.occupied_cells,
      planning_result->current_lidar.occupied_cells,
      prohibited_grid_stats.occupied_cells + prohibited_grid_stats.inflated_cells,
      prohibited_grid_stats.occupied_cells, prohibited_grid_stats.inflated_cells,
      prohibited_grid_stats.free_cells, prohibited_grid_stats.unknown_cells,
      inflation_radius_m_,
      path_result->grid_stats.occupied_cells + path_result->grid_stats.inflated_cells,
      path_result->grid_stats.occupied_cells, path_result->grid_stats.inflated_cells,
      path_result->grid_stats.free_cells, path_result->grid_stats.unknown_cells,
      planning_clearance_m_, inflation_radius_m_ + planning_clearance_m_,
      planning_result->cache.static_cache_eligible ? "true" : "false",
      planning_result->cache.static_cache_hit ? "true" : "false",
      planning_result->cache.static_cache_rebuilt ? "true" : "false",
      planning_result->cache.static_distance_field_duration_ms,
      planning_result->cache.static_inflation_mask_duration_ms,
      planning_result->cache.dynamic_distance_field_duration_ms,
      planning_result->cache.dynamic_inflation_mask_duration_ms,
      planning_result->cache.static_distance_source_cells,
      planning_result->cache.dynamic_distance_source_cells,
      planning_result->static_source.enabled ? "true" : "false",
      planning_result->static_source.loaded ? "true" : "false",
      planning_result->static_source.used ? "true" : "false",
      planning_result->static_source.rectangles,
      planning_result->static_source.occupied_cells,
      planning_result->static_source.path.c_str(),
      planning_result->memory.enabled ? "true" : "false",
      planning_result->memory.seen ? "true" : "false",
      planning_result->memory.used ? "true" : "false",
      planning_result->memory.geometry_matches ? "true" : "false",
      planning_result->memory.source_counts.occupied_cells,
      planning_result->memory.source_counts.free_cells,
      planning_result->memory.source_counts.unknown_cells,
      planning_result->memory.overlay.occupied_cells_applied,
      planning_result->memory.overlay.free_cells_applied,
      planning_result->current_lidar.enabled ? "true" : "false",
      planning_result->current_lidar.used ? "true" : "false",
      planning_result->current_lidar.fresh ? "true" : "false",
      planning_result->current_lidar.processed_beams,
      planning_result->current_lidar.hit_beams,
      planning_result->current_lidar.altitude_rejected_beams,
      planning_result->current_lidar.occupied_cells,
      planning_result->current_lidar.overlay_occupied_cells_applied,
      planning_result->current_lidar.overlay_occupied_cells_preserved,
      planning_result->current_lidar.outside_hits,
      astarStatusName(path_result->astar.status),
      planning_astar_config.heuristic_weight, path_result->astar.expanded_cells,
      path_result->astar.total_cost, path_result->raw_path_metrics.points,
      path_result->smoothed_path_metrics.points,
      planning_astar_config.initial_heading_bias_enabled ? "true" : "false",
      initialHeadingBiasActive(planning_astar_config) ? "true" : "false",
      current_speed_mps_, planning_astar_config.initial_heading_bias_min_speed_mps,
      planning_astar_config.initial_heading_bias_weight,
      planning_astar_config.initial_heading_bias_velocity_x_mps,
      planning_astar_config.initial_heading_bias_velocity_y_mps,
      path_result->raw_path_metrics.segments,
      path_result->raw_path_metrics.straight_segments,
      path_result->raw_path_metrics.turns, path_result->raw_path_metrics.length_m,
      path_result->smoothed_path_metrics.segments,
      path_result->smoothed_path_metrics.straight_segments,
      path_result->smoothed_path_metrics.turns,
      path_result->smoothed_path_metrics.length_m, path_result->raw_path_clearance_m,
      path_result->smoothed_path_clearance_m, planning_grid_duration_ms,
      path_result->total_duration_ms, path_result->astar_duration_ms,
      path_result->smoothing_duration_ms, path_result->grid_stats_duration_ms,
      path_result->raw_path_metrics_duration_ms,
      path_result->smoothed_path_metrics_duration_ms,
      path_result->prohibited_clearance_field_duration_ms,
      path_result->prohibited_clearance_field_cache_hit ? "true" : "false",
      path_result->raw_path_clearance_duration_ms,
      path_result->smoothed_path_clearance_duration_ms);
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Path smoothing diagnostics: input_points=%zu output_points=%zu "
      "checks=%zu accepted=%zu shortcuts=%zu forced_adjacent=%zu rejected=%zu "
      "rejected_prohibited=%zu rejected_outside_grid=%zu "
      "rejected_prohibited_cells=%zu "
      "raw_segment_lengths[min=%.2f mean=%.2f max=%.2f lt2=%zu lt5=%zu "
      "lt10=%zu] "
      "smoothed_segment_lengths[min=%.2f mean=%.2f max=%.2f lt2=%zu lt5=%zu "
      "lt10=%zu]",
      path_result->smoothing_stats.input_points,
      path_result->smoothing_stats.output_points,
      path_result->smoothing_stats.line_of_sight_checks,
      path_result->smoothing_stats.accepted_segments,
      path_result->smoothing_stats.shortcut_segments,
      path_result->smoothing_stats.forced_adjacent_segments,
      path_result->smoothing_stats.rejected_segments,
      path_result->smoothing_stats.rejected_prohibited,
      path_result->smoothing_stats.rejected_outside_grid,
      path_result->smoothing_stats.rejected_prohibited_cells,
      path_result->raw_path_metrics.min_segment_length_m,
      path_result->raw_path_metrics.mean_segment_length_m,
      path_result->raw_path_metrics.max_segment_length_m,
      path_result->raw_path_metrics.segments_shorter_than_2m,
      path_result->raw_path_metrics.segments_shorter_than_5m,
      path_result->raw_path_metrics.segments_shorter_than_10m,
      path_result->smoothed_path_metrics.min_segment_length_m,
      path_result->smoothed_path_metrics.mean_segment_length_m,
      path_result->smoothed_path_metrics.max_segment_length_m,
      path_result->smoothed_path_metrics.segments_shorter_than_2m,
      path_result->smoothed_path_metrics.segments_shorter_than_5m,
      path_result->smoothed_path_metrics.segments_shorter_than_10m);
  if (path_result->smoothing_returned_empty_path) {
    RCLCPP_WARN(get_logger(),
                "Path smoothing returned an empty path; falling back to raw A* path: "
                "raw_points=%zu",
                path_result->astar.path.size());
  }
  publishPathFromPathCells(planning_grid, prohibited_grid, path_result->astar.path,
                           path_result->smoothed_cells, "planning_clearance",
                           path_result->prohibited_clearance_field,
                           path_result->prohibited_clearance_field_cache_hit);
}

[[nodiscard]] AStarConfig PlannerNode::astarConfigForCurrentVelocity() const {
  AStarConfig config = astar_config_;
  if (current_velocity_valid_ && std::isfinite(current_speed_mps_) &&
      current_speed_mps_ >= config.initial_heading_bias_min_speed_mps) {
    config.initial_heading_bias_velocity_x_mps = current_velocity_.x;
    config.initial_heading_bias_velocity_y_mps = current_velocity_.y;
  }
  return config;
}

[[nodiscard]] bool
PlannerNode::initialHeadingBiasActive(const AStarConfig& config) noexcept {
  const double speed_mps = std::hypot(config.initial_heading_bias_velocity_x_mps,
                                      config.initial_heading_bias_velocity_y_mps);
  return config.initial_heading_bias_enabled &&
         config.initial_heading_bias_weight > 0.0 && std::isfinite(speed_mps) &&
         speed_mps >= config.initial_heading_bias_min_speed_mps;
}

[[nodiscard]] std::optional<PathComputationResult>
PlannerNode::computePathOnGrid(const OccupancyGrid2D& grid, const char* source_label,
                               const AStarConfig& astar_config) {
  const auto start_cell = grid.worldToCell(current_pose_.position);
  const auto goal_cell = grid.worldToCell(goal_);
  if (!start_cell.has_value() || !goal_cell.has_value()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Start or goal is outside the %s planning grid: "
                          "start=(%.2f, %.2f) goal=(%.2f, %.2f)",
                          source_label, current_pose_.position.x,
                          current_pose_.position.y, goal_.x, goal_.y);
    return std::nullopt;
  }
  const bool start_prohibited = grid.isProhibited(*start_cell);
  const bool goal_prohibited = grid.isProhibited(*goal_cell);
  if (goal_prohibited || grid.isOccupied(*start_cell)) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Start or goal is not plannable on %s grid; refusing to move planning "
        "endpoints: start_cell=(%d,%d) occupied=%s inflated=%s prohibited=%s "
        "goal_cell=(%d,%d) occupied=%s inflated=%s prohibited=%s",
        source_label, start_cell->x, start_cell->y,
        grid.isOccupied(*start_cell) ? "true" : "false",
        grid.isInflated(*start_cell) ? "true" : "false",
        start_prohibited ? "true" : "false", goal_cell->x, goal_cell->y,
        grid.isOccupied(*goal_cell) ? "true" : "false",
        grid.isInflated(*goal_cell) ? "true" : "false",
        goal_prohibited ? "true" : "false");
    return std::nullopt;
  }
  if (start_prohibited) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Start is inside inflated prohibited space on %s grid; attempting "
        "escape-start planning: start_cell=(%d,%d)",
        source_label, start_cell->x, start_cell->y);
  }

  const auto path_compute_started_at = std::chrono::steady_clock::now();
  ClearanceField2D prebuilt_clearance_field = ClearanceField2D::build(
      grid, planner_core_.config().clearance_diagnostic_radius_m,
      ClearanceSource::kProhibited);
  auto result = planner_core_.computePath(PathComputationInput{
      .grid = &grid,
      .current_position = current_pose_.position,
      .goal = goal_,
      .astar = astar_config,
      .prohibited_clearance_field = &prebuilt_clearance_field,
      .prohibited_clearance_field_cache_hit = false,
  });
  const double path_compute_duration_ms = elapsedMilliseconds(path_compute_started_at);
  ++astar_runs_;
  if (!result.has_value()) {
    ++astar_failures_;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "A* did not find a path on %s grid: start=(%d,%d) goal=(%d,%d) "
        "duration_ms=%.1f",
        source_label, start_cell->x, start_cell->y, goal_cell->x, goal_cell->y,
        path_compute_duration_ms);
    return std::nullopt;
  }

  result->owned_prohibited_clearance_field =
      std::make_shared<ClearanceField2D>(std::move(prebuilt_clearance_field));
  result->prohibited_clearance_field = result->owned_prohibited_clearance_field.get();
  ++astar_successes_;
  if (result->start_escape_used && result->requested_start_cell.has_value() &&
      result->start_cell.has_value()) {
    RCLCPP_WARN(get_logger(),
                "A* recovered from inflated start on %s grid: requested_start=(%d,%d) "
                "escape_start=(%d,%d) escape_distance=%.2fm",
                source_label, result->requested_start_cell->x,
                result->requested_start_cell->y, result->start_cell->x,
                result->start_cell->y, result->start_escape_distance_m);
  }
  return result;
}

} // namespace drone_city_nav
