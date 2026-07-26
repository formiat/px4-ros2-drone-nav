#include "drone_city_nav/grid_config.hpp"

#include <array>
#include <memory>

#include "planner_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] double terminalBrakingDistanceM(const double speed_mps,
                                              const double decel_mps2,
                                              const double margin_m) noexcept {
  const double speed = std::max(0.0, std::isfinite(speed_mps) ? speed_mps : 0.0);
  const double decel =
      std::max(1.0e-6, std::isfinite(decel_mps2) ? decel_mps2 : 1.0e-6);
  return speed * speed / (2.0 * decel) +
         std::max(0.0, std::isfinite(margin_m) ? margin_m : 0.0);
}

} // namespace

void PlannerNode::onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg) {
  const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
  if (!msg.xy_valid || !std::isfinite(msg.x) || !std::isfinite(msg.y)) {
    {
      const std::scoped_lock lock{navigation_state_mutex_};
      live_navigation_state_.pose = Pose2{};
      live_navigation_state_.velocity = Point2{};
      live_navigation_state_.altitude_m = std::numeric_limits<double>::quiet_NaN();
      live_navigation_state_.speed_mps = std::numeric_limits<double>::quiet_NaN();
      live_navigation_state_.stamp_ns = 0;
      live_navigation_state_.pose_valid = false;
      live_navigation_state_.altitude_valid = false;
      live_navigation_state_.velocity_valid = false;
    }
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planner invalidated cached pose after invalid PX4 local position: "
        "xy_valid=%s x=%.2f y=%.2f",
        msg.xy_valid ? "true" : "false", static_cast<double>(msg.x),
        static_cast<double>(msg.y));
    return;
  }

  NavigationStateSnapshot navigation = navigationStateSnapshot();
  navigation.pose.position = Point2{static_cast<double>(msg.x) + px4_local_origin_.x,
                                    static_cast<double>(msg.y) + px4_local_origin_.y};
  if (msg.heading_good_for_control && std::isfinite(msg.heading)) {
    navigation.pose.yaw_rad = static_cast<double>(msg.heading);
  }
  if (msg.z_valid && std::isfinite(msg.z)) {
    navigation.altitude_m = -static_cast<double>(msg.z);
    navigation.altitude_valid = true;
  } else {
    navigation.altitude_valid = false;
  }
  if (msg.v_xy_valid && std::isfinite(msg.vx) && std::isfinite(msg.vy)) {
    navigation.velocity =
        Point2{static_cast<double>(msg.vx), static_cast<double>(msg.vy)};
    navigation.speed_mps = std::hypot(navigation.velocity.x, navigation.velocity.y);
    navigation.velocity_valid = true;
  } else {
    navigation.velocity = Point2{};
    navigation.speed_mps = std::numeric_limits<double>::quiet_NaN();
    navigation.velocity_valid = false;
  }
  navigation.pose_valid = true;
  navigation.stamp_ns = receive_stamp_ns;
  {
    const std::scoped_lock lock{navigation_state_mutex_};
    live_navigation_state_ = navigation;
  }
  {
    const std::scoped_lock lock{lidar_pose_history_mutex_};
    lidar_pose_history_.addPosition(
        receive_stamp_ns,
        Point3{navigation.pose.position.x, navigation.pose.position.y,
               navigation.altitude_m},
        use_px4_heading_for_scan_ ? navigation.pose.yaw_rad : initial_heading_rad_,
        navigation.altitude_valid &&
            (!use_px4_heading_for_scan_ || msg.heading_good_for_control),
        px4_ros_time_mapper_.recoverPx4LocalTimeNs(msg.timestamp_sample).value_or(0),
        lidarPoseSourceTimestampNanoseconds(msg.timestamp_sample));
  }

  if (!local_position_seen_) {
    local_position_seen_ = true;
    const double altitude_m = (msg.z_valid && std::isfinite(msg.z))
                                  ? -static_cast<double>(msg.z)
                                  : std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(get_logger(),
                "First valid PX4 local position: x=%.2f y=%.2f z=%.2f "
                "altitude=%.2f yaw=%.2f distance_to_start=%.2f "
                "distance_to_goal=%.2f",
                navigation.pose.position.x, navigation.pose.position.y,
                static_cast<double>(msg.z), altitude_m, navigation.pose.yaw_rad,
                distance(navigation.pose.position, start_),
                distance(navigation.pose.position, goal_));
  }
}

void PlannerNode::onScan(const sensor_msgs::msg::LaserScan& msg) {
  const NavigationStateSnapshot navigation = navigationStateSnapshot();
  LidarInputSnapshot lidar;
  lidar.scan = msg;
  lidar.seen = true;
  lidar.update_ns = get_clock()->now().nanoseconds();
  lidar.projection_pose_valid = navigation.pose_valid;
  LidarPoseAlignmentStatus alignment_status =
      LidarPoseAlignmentStatus::kPositionHistoryEmpty;
  if (lidar.projection_pose_valid) {
    lidar.projection_pose = LidarProjectionPose{
        navigation.pose.position,
        navigation.altitude_m,
        use_px4_heading_for_scan_ ? navigation.pose.yaw_rad : initial_heading_rad_,
        navigation.attitude.roll_rad,
        navigation.attitude.pitch_rad,
        navigation.altitude_valid,
        navigation.attitude_valid};
    const LidarPoseMotionCompensationResult motion_compensation =
        compensateLidarPoseForLatency(
            lidar.projection_pose.position, navigation.velocity,
            motion_compensate_lidar_pose_, navigation.velocity_valid,
            currentLidarPoseReceiveLagSeconds(lidar.update_ns, navigation.stamp_ns),
            lidar_pose_latency_s_);
    lidar.projection_pose.position = motion_compensation.position;
    lidar.pose_lag_s = motion_compensation.pose_lag_s;
    lidar.pose_latency_s = motion_compensation.latency_s;
    lidar.motion_shift = motion_compensation.applied_shift;
    lidar.motion_shift_m = motion_compensation.applied_shift_m;
    const std::uint64_t scan_stamp = stampNanoseconds(msg.header.stamp);
    const LaserScanTiming scan_timing{
        .first_beam_stamp_ns =
            scan_stamp <=
                    static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())
                ? static_cast<std::int64_t>(scan_stamp)
                : 0,
        .first_beam_stamp_valid =
            scan_stamp > 0U &&
            scan_stamp <=
                static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()),
        .time_increment_s = static_cast<double>(msg.time_increment),
        .receive_stamp_ns = lidar.update_ns,
        .receive_stamp_valid = lidar.update_ns > 0,
    };
    LidarBeamPoseAlignmentResult alignment{};
    {
      const std::scoped_lock lock{lidar_pose_history_mutex_};
      alignment = timestampAlignedLidarBeamPosesWithDiagnostics(
          lidar_pose_history_, scan_timing, msg.ranges.size(),
          use_px4_heading_for_scan_ ? std::nullopt
                                    : std::optional<double>{initial_heading_rad_},
          &px4_ros_time_mapper_);
    }
    alignment_status = alignment.status;
    lidar.beam_projection_poses =
        alignment.aligned() ? alignment.poses : std::vector<LidarProjectionPose>{};
    if (alignment.sourceAligned()) {
      lidar.projection_pose_source = LidarProjectionPoseSource::kSourceTimestampAligned;
    } else if (alignment.aligned()) {
      lidar.projection_pose_source =
          LidarProjectionPoseSource::kReceiveTimestampAligned;
    } else if (motion_compensation.applied) {
      lidar.projection_pose_source =
          LidarProjectionPoseSource::kMotionExtrapolatedFallback;
    } else {
      lidar.projection_pose_source = LidarProjectionPoseSource::kCallbackPoseFallback;
    }
    const std::string alignment_diagnostic = formatLidarPoseAlignmentDiagnostic(
        alignment.aligned() ? "Planner lidar 6DoF pose alignment"
                            : "Planner lidar 6DoF pose alignment fallback",
        alignment, scan_timing, lidar.update_ns);
    if (alignment.aligned()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "%s",
                           alignment_diagnostic.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s",
                           alignment_diagnostic.c_str());
    }
  } else {
    lidar.projection_pose_source = LidarProjectionPoseSource::kCallbackPoseFallback;
  }
  {
    const std::scoped_lock lock{lidar_input_mutex_};
    live_lidar_input_ = lidar;
  }
  if (!scan_seen_logged_) {
    scan_seen_logged_ = true;
    RCLCPP_INFO(get_logger(),
                "First planner lidar scan: beams=%zu range=[%.2f, %.2f] "
                "angle=[%.2f, %.2f] projection_pose=%s alignment=%s pose_lag=%.3fs "
                "pose_latency=%.3fs motion_shift=(%.2f, %.2f) "
                "motion_shift_m=%.2f",
                lidar.scan.ranges.size(), static_cast<double>(lidar.scan.range_min),
                static_cast<double>(lidar.scan.range_max),
                static_cast<double>(lidar.scan.angle_min),
                static_cast<double>(lidar.scan.angle_max),
                lidar.projection_pose_valid ? "true" : "false",
                lidarPoseAlignmentStatusName(alignment_status), lidar.pose_lag_s,
                lidar.pose_latency_s, lidar.motion_shift.x, lidar.motion_shift.y,
                lidar.motion_shift_m);
  }
}

void PlannerNode::onAttitude(const px4_msgs::msg::VehicleAttitude& msg) {
  const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
  {
    const std::scoped_lock lock{lidar_pose_history_mutex_};
    lidar_pose_history_.addAttitude(
        receive_stamp_ns, msg.q,
        px4_ros_time_mapper_.recoverPx4LocalTimeNs(msg.timestamp_sample).value_or(0),
        lidarPoseSourceTimestampNanoseconds(msg.timestamp_sample));
  }
  const auto euler = quaternionToEuler(msg.q);
  {
    const std::scoped_lock lock{navigation_state_mutex_};
    live_navigation_state_.attitude_valid = euler.has_value();
    if (euler.has_value()) {
      live_navigation_state_.attitude = *euler;
    }
  }
}

void PlannerNode::onTimesyncStatus(const px4_msgs::msg::TimesyncStatus& msg) {
  Px4RosTimeMappingDiagnostics diagnostics{};
  {
    const std::scoped_lock lock{lidar_pose_history_mutex_};
    px4_ros_time_mapper_.observeTimesync(msg.timestamp, msg.estimated_offset,
                                         msg.round_trip_time,
                                         get_clock()->now().nanoseconds());
    diagnostics = px4_ros_time_mapper_.diagnostics();
  }
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Planner PX4/ROS clock mapping: ready=%s samples=%zu scale=%.9f "
      "offset_ms=%.3f max_residual_ms=%.3f estimated_offset_ms=%.3f",
      diagnostics.ready ? "true" : "false", diagnostics.sample_count, diagnostics.scale,
      1.0e-6 * diagnostics.offset_ns, 1.0e-6 * diagnostics.max_fit_residual_ns,
      1.0e-6 * static_cast<double>(diagnostics.latest_estimated_offset_ns));
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
    static_grid_.reset();
    static_map_debug_.reset();
    static_map_rectangles_ = 0U;
    static_map_occupied_cells_ = 0U;
    RCLCPP_INFO(get_logger(), "Static city map source is disabled");
    return;
  }

  if (result.status == StaticMapSourceStatus::kLoadFailed || !result.grid.has_value()) {
    static_grid_.reset();
    static_map_debug_.reset();
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
  static_map_debug_ = std::move(result.map);
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
  known_static_lidar_classifier_.reset();
  const KnownPassageSourceResult result = loadKnownPassageMapSource(
      KnownPassageSourceConfig{use_known_passages_, known_passages_path_param_,
                               staticMapPackageShareDirectory(), frame_id_});
  known_passages_resolved_path_ = result.resolved_path;
  const auto log_classifier = [this]() {
    const char* status = "disabled";
    if (known_static_lidar_hit_classifier_enabled_) {
      status = known_static_lidar_classifier_.has_value() ? "ready" : "fail_open";
    }
    RCLCPP_INFO(get_logger(),
                "Known static lidar classifier: node=planner status=%s path='%s' "
                "volumes=%zu closer_tolerance=%.3fm farther_tolerance=%.3fm "
                "endpoint_volume_tolerance=%.3fm opening_boundary_tolerance=%.3fm",
                status, known_passages_resolved_path_.string().c_str(),
                known_static_lidar_classifier_.has_value()
                    ? known_static_lidar_classifier_->volumeCount()
                    : 0U,
                known_static_lidar_hit_closer_range_tolerance_m_,
                known_static_lidar_hit_farther_range_tolerance_m_,
                known_static_lidar_hit_endpoint_volume_tolerance_m_,
                known_static_opening_boundary_tolerance_m_);
  };

  if (result.status == KnownPassageSourceStatus::kDisabled) {
    known_passages_.reset();
    known_passage_structures_ = 0U;
    known_passage_openings_ = 0U;
    RCLCPP_INFO(get_logger(), "Known passage map source is disabled");
    log_classifier();
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
    log_classifier();
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
  if (known_static_lidar_hit_classifier_enabled_ && result.frame_matches) {
    std::vector<KnownPassageSolidVolume> volumes =
        knownPassageSolidVolumes(*known_passages_);
    if (!volumes.empty()) {
      known_static_lidar_classifier_.emplace(
          std::move(volumes),
          KnownStaticLidarHitClassifierConfig{
              .closer_range_tolerance_m =
                  known_static_lidar_hit_closer_range_tolerance_m_,
              .farther_range_tolerance_m =
                  known_static_lidar_hit_farther_range_tolerance_m_,
              .endpoint_volume_tolerance_m =
                  known_static_lidar_hit_endpoint_volume_tolerance_m_,
              .opening_boundary_tolerance_m =
                  known_static_opening_boundary_tolerance_m_});
    }
  }
  RCLCPP_INFO(get_logger(),
              "Known passage map loaded: path='%s' status=%s frame='%s' "
              "structures=%zu openings=%zu markers_topic='%s'",
              known_passages_resolved_path_.string().c_str(),
              knownPassageSourceStatusName(result.status),
              known_passages_->frame_id.c_str(), known_passage_structures_,
              known_passage_openings_,
              known_passage_markers_pub_ ? known_passage_markers_pub_->get_topic_name()
                                         : "<unavailable>");
  log_classifier();
  publishKnownPassageDebug(true);
}

[[nodiscard]] ObstacleFieldBuilderConfig
PlannerNode::planningGridBuilderConfig() const {
  ObstacleFieldBuilderConfig config{};
  config.use_static_map = use_static_map_;
  config.fallback_bounds = fallback_grid_bounds_;
  config.inflation_radius_m = inflation_radius_m_;
  config.planning_clearance_m = planning_clearance_m_;
  if (!use_static_map_ && no_static_rollout_enabled_ &&
      no_static_rollout_local_window_enabled_ && finite2D(current_pose_.position)) {
    const double horizon_m = rollout_planner_.config().horizon_m;
    const double safety_halo_m = inflation_radius_m_ + planning_clearance_m_ +
                                 planner_core_.config().clearance_diagnostic_radius_m +
                                 no_static_rollout_local_window_extra_margin_m_;
    const double half_extent_m = horizon_m + safety_halo_m;
    Point2 direction{1.0, 0.0};
    const Point2 goal_delta{goal_.x - current_pose_.position.x,
                            goal_.y - current_pose_.position.y};
    const double goal_distance_m = std::hypot(goal_delta.x, goal_delta.y);
    if (goal_distance_m > 1.0e-6) {
      direction =
          Point2{goal_delta.x / goal_distance_m, goal_delta.y / goal_distance_m};
    }
    const Point2 center{current_pose_.position.x + horizon_m * direction.x,
                        current_pose_.position.y + horizon_m * direction.y};
    config.local_planning_bounds = boundedGridBounds(
        center.x - half_extent_m, center.y - half_extent_m,
        fallback_grid_bounds_.resolution_m, 2.0 * half_extent_m, 2.0 * half_extent_m);
  }
  return config;
}

[[nodiscard]] std::optional<ObstacleFieldBuildResult>
PlannerNode::buildObstacleField(const std::int64_t now_ns) {
  const ObstacleFieldBuilderConfig config = planningGridBuilderConfig();
  PlanningGridSources sources{};
  sources.static_grid = static_grid_ ? &*static_grid_ : nullptr;
  sources.static_rectangles = static_map_rectangles_;
  sources.static_occupied_cells = static_map_occupied_cells_;
  sources.static_map_path = static_map_resolved_path_.string();
  sources.memory_grid = memory_grid_ ? &*memory_grid_ : nullptr;
  sources.memory_producer_instance_id =
      last_memory_snapshot_applied_producer_instance_id_;
  sources.memory_sequence = last_memory_snapshot_applied_sequence_;
  sources.lidar_update_ns = last_scan_update_ns_;

  std::optional<OccupancyGrid2D> current_lidar_grid;
  if (const std::optional<GridBounds> bounds =
          selectPlanningGridBounds(config, sources);
      bounds.has_value()) {
    current_lidar_grid.emplace(*bounds);
    sources.current_lidar = overlayCurrentLidarHits(*current_lidar_grid, now_ns);
    sources.current_lidar_grid = &*current_lidar_grid;
  }
  ObstacleFieldBuildResult result = planning_grid_builder_.build(config, sources);
  if (current_lidar_grid.has_value()) {
    result.current_lidar_grid = std::move(current_lidar_grid);
  }
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Planner obstacle source state: "
      "static[enabled=%s loaded=%s used=%s occupied=%zu] "
      "memory[enabled=%s seen=%s used=%s occupied=%zu] "
      "current_lidar[enabled=%s used=%s fresh=%s occupied=%zu]",
      result.static_source.enabled ? "true" : "false",
      result.static_source.loaded ? "true" : "false",
      result.static_source.used ? "true" : "false", result.static_source.occupied_cells,
      result.memory.enabled ? "true" : "false", result.memory.seen ? "true" : "false",
      result.memory.used ? "true" : "false", result.memory.source_counts.occupied_cells,
      result.current_lidar.enabled ? "true" : "false",
      result.current_lidar.used ? "true" : "false",
      result.current_lidar.fresh ? "true" : "false",
      result.current_lidar.occupied_cells);
  const LidarIngestionDecisionStats& lidar_decisions =
      result.current_lidar.ingestion_decisions;
  const std::string lidar_decision_summary =
      formatLidarIngestionDecisionStatsSummary(lidar_decisions);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                       "Planner current lidar decisions: %s",
                       lidar_decision_summary.c_str());
  if (result.memory.enabled && !result.memory.seen) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Obstacle memory source is enabled but no grid has been "
                         "received yet");
  } else if (result.memory.seen && memory_grid_.has_value() &&
             !result.memory.geometry_matches) {
    const OccupancyGrid2D& memory_grid = *memory_grid_;
    std::optional<OccupancyGrid2D> diagnostic_grid;
    const OccupancyGrid2D* planning_grid =
        result.raw_occupancy ? &*result.raw_occupancy : nullptr;
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
  schedulePlanningCycle(PlanningWakeReason::kPeriodicTimer);
}

// The planning transaction intentionally keeps all stage decisions in one worker
// callback so a generation uses one immutable input snapshot.
// NOLINTNEXTLINE(readability-function-size)
void PlannerNode::runPlanningCycle(const PlanningJobIdentity& identity) {
  const auto cycle_started_at = std::chrono::steady_clock::now();
  const std::uint64_t invalidation_generation = identity.invalidation_generation;
  RCLCPP_INFO(
      get_logger(),
      "PLANNING_CYCLE_START cycle_sequence=%" PRIu64 " invalidation_generation=%" PRIu64
      " wake_reason=%s coalesced_requests=%" PRIu64,
      identity.cycle_sequence, invalidation_generation,
      planningWakeReasonName(identity.wake_reason), identity.coalesced_requests);
  const NavigationStateSnapshot navigation = navigationStateSnapshot();
  applyNavigationStateSnapshot(navigation);
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
  std::optional<TruncationReplanState> truncation_replan = truncationReplanState();
  if (truncation_replan.has_value() && !truncation_replan->confirmed) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "REPLAN_TRUNCATION planning_wait=true blocked_path_id=%" PRIu64
                         " generation=%" PRIu64 " grid_build_skipped=true",
                         truncation_replan->blocked_path_id,
                         truncation_replan->generation);
    return;
  }
  if (truncation_replan.has_value() && truncation_replan->awaiting_ack) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "REPLAN_TRUNCATION planning_wait=true reason=awaiting_suffix_ack "
        "blocked_path_id=%" PRIu64 " generation=%" PRIu64 " path_id=%" PRIu64
        " attempt=%zu grid_build_skipped=true",
        truncation_replan->blocked_path_id, truncation_replan->generation,
        truncation_replan->published_suffix_path_id,
        truncation_replan->publication_attempts);
    return;
  }
  const bool no_static_rollout_mode =
      plannerModePrimaryAction(use_static_map_, no_static_rollout_enabled_) ==
      PlannerModePrimaryAction::kRollout;
  const RolloutSuccessorSnapshot* successor_snapshot =
      truncation_replan.has_value() &&
              truncation_replan->rollout_successor_snapshot.has_value()
          ? &*truncation_replan->rollout_successor_snapshot
          : nullptr;
  const bool reuse_successor_snapshot =
      no_static_rollout_mode && !no_static_astar_recovery_enabled_ &&
      truncation_replan.has_value() && truncation_replan->confirmed &&
      !truncation_replan->awaiting_ack && successor_snapshot != nullptr &&
      successor_snapshot->prepared_grid != nullptr &&
      successor_snapshot->blocked_path_id == truncation_replan->blocked_path_id &&
      successor_snapshot->truncation_generation == truncation_replan->generation &&
      successor_snapshot->temporary_prefix_fingerprint ==
          truncation_replan->temporary_prefix_fingerprint &&
      obstacleRiskVersionsEqual(successor_snapshot->grid_version,
                                successor_snapshot->prepared_grid->version);

  std::optional<ObstacleFieldBuildResult> planning_result;
  std::shared_ptr<const PreparedObstacleRiskSnapshot> prepared;
  double planning_grid_duration_ms{0.0};
  if (reuse_successor_snapshot) {
    prepared = successor_snapshot->prepared_grid;
    const double snapshot_age_ms =
        successor_snapshot->blocker_detected_stamp_ns > 0 &&
                now_ns >= successor_snapshot->blocker_detected_stamp_ns
            ? 1.0e-6 * static_cast<double>(
                           now_ns - successor_snapshot->blocker_detected_stamp_ns)
            : std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(get_logger(),
                "ROLLOUT_SUCCESSOR_SNAPSHOT generation=%" PRIu64
                " blocked_path_id=%" PRIu64 " grid_revision=%" PRIu64
                " snapshot_reused=true snapshot_age_ms=%.2f grid_build_ms=0.00 "
                "clearance_build_ms=0.00",
                truncation_replan->generation, truncation_replan->blocked_path_id,
                prepared->version.build_revision, snapshot_age_ms);
    if (std::isfinite(snapshot_age_ms) && snapshot_age_ms > 2000.0) {
      RCLCPP_WARN(get_logger(),
                  "ROLLOUT_SUCCESSOR_SNAPSHOT old_snapshot=true generation=%" PRIu64
                  " snapshot_age_ms=%.2f action=continue_with_runtime_monitor",
                  truncation_replan->generation, snapshot_age_ms);
    }
  } else {
    applyLatestLidarInputSnapshot();
    applyPendingMemorySnapshot(now_ns);
    const auto planning_grid_started_at = std::chrono::steady_clock::now();
    planning_result = buildObstacleField(now_ns);
    planning_grid_duration_ms = elapsedMilliseconds(planning_grid_started_at);
    if (!planning_result.has_value()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Planner skipped path publication because the planning grid is "
          "not ready; keeping the last published path");
      return;
    }
    std::optional<PreparedObstacleRiskSnapshot> prepared_value =
        preparePlanningGridSnapshot(*planning_result, navigation.pose.position);
    if (!prepared_value.has_value()) {
      RCLCPP_ERROR(get_logger(),
                   "Planner could not prepare an immutable grid snapshot from a "
                   "completed grid build; keeping the last published path");
      return;
    }
    prepared = std::make_shared<const PreparedObstacleRiskSnapshot>(
        std::move(*prepared_value));
  }
  RCLCPP_INFO(
      get_logger(),
      "PLANNING_GRID_TIMING cycle_sequence=%" PRIu64 " invalidation_generation=%" PRIu64
      " grid_revision=%" PRIu64
      " grid_build_ms=%.2f snapshot_reused=%s local_window=%s "
      "raw_grid=%dx%d evaluation_grid=%dx%d source_cells=%zu "
      "evaluation_cells=%zu reduction=%.2fx",
      identity.cycle_sequence, invalidation_generation,
      prepared->version.build_revision, planning_grid_duration_ms,
      reuse_successor_snapshot ? "true" : "false",
      prepared->raw_occupancy.bounds().width_cells !=
                  prepared->evaluation_bounds.width_cells ||
              prepared->raw_occupancy.bounds().height_cells !=
                  prepared->evaluation_bounds.height_cells
          ? "true"
          : "false",
      prepared->raw_occupancy.width(), prepared->raw_occupancy.height(),
      prepared->evaluation_bounds.width_cells, prepared->evaluation_bounds.height_cells,
      prepared->raw_occupancy.cellCount(),
      gridBoundsCellCount(prepared->evaluation_bounds),
      gridBoundsCellCount(prepared->evaluation_bounds) > 0U
          ? static_cast<double>(prepared->raw_occupancy.cellCount()) /
                static_cast<double>(gridBoundsCellCount(prepared->evaluation_bounds))
          : std::numeric_limits<double>::infinity());
  const OccupancyGrid2D& prohibited_grid = prepared->raw_occupancy;
  const OccupancyGrid2D& planning_grid = prepared->raw_occupancy;
  publishRawObstacleSnapshot(*prepared);
  if (!reuse_successor_snapshot) {
    truncation_replan = truncationReplanState();
  }
  const bool active_rollout_artifact_available =
      !use_static_map_ && no_static_rollout_enabled_ &&
      trajectorySamplesAreUsable(executable_trajectory_artifact_.samples);
  const double rollout_exhaustion_epsilon_m =
      std::max(0.5, prohibited_grid.resolution());
  ExecutableSuffixDecision rollout_runtime{};
  if (active_rollout_artifact_available) {
    const ExecutableTrajectoryProgress progress = updateExecutableTrajectoryProgress(
        executable_trajectory_artifact_, navigation.pose.position,
        stable_path_goal_tolerance_m_);
    rollout_runtime =
        evaluateExecutableSuffix(prohibited_grid, executable_trajectory_artifact_,
                                 progress, rollout_exhaustion_epsilon_m);
    const BlockedSpan* blocked_span = rollout_runtime.blocked_span.has_value()
                                          ? &*rollout_runtime.blocked_span
                                          : nullptr;
    const char* exhaustion_reason = "not_exhausted";
    if (progress.diverged ||
        (rollout_runtime.exhausted &&
         progress.terminal_distance_m > stable_path_goal_tolerance_m_)) {
      exhaustion_reason = "projection_diverged";
    } else if (rollout_runtime.exhausted) {
      exhaustion_reason = "normally_exhausted";
    }
    RCLCPP_INFO(
        get_logger(),
        "ROLLOUT_RUNTIME_PATH_CHECK path_id=%" PRIu64
        " projection_valid=%s previous_s=%.2f current_s=%.2f remaining=%.2f "
        "cross_track=%.2f terminal_distance=%.2f checked_from_s=%.2f "
        "blocked=%s trigger=%s first_blocked_s=%.2f distance_to_blocker=%.2f "
        "cell=(%d,%d) active_prefix_available=%s exhaustion_reason=%s "
        "decision=%s planner_last_published_path_id=%" PRIu64 " activation_pending=%s",
        executable_trajectory_artifact_.path_id, progress.valid ? "true" : "false",
        progress.previous_s_m, progress.projected_s_m, progress.remaining_m,
        progress.cross_track_m, progress.terminal_distance_m, progress.projected_s_m,
        rollout_runtime.blocked ? "true" : "false",
        blocked_span != nullptr ? blockedSpanTriggerName(blocked_span->trigger)
                                : "none",
        blocked_span != nullptr ? blocked_span->first_blocked_s_m
                                : std::numeric_limits<double>::quiet_NaN(),
        blocked_span != nullptr
            ? std::max(0.0, blocked_span->first_blocked_s_m - progress.projected_s_m)
            : std::numeric_limits<double>::quiet_NaN(),
        blocked_span != nullptr && blocked_span->first_cell_available
            ? blocked_span->first_cell.x
            : -1,
        blocked_span != nullptr && blocked_span->first_cell_available
            ? blocked_span->first_cell.y
            : -1,
        progress.valid && !rollout_runtime.exhausted ? "true" : "false",
        exhaustion_reason,
        !progress.valid           ? "projection_unavailable"
        : blocked_span == nullptr ? "clear"
                                  : "raw_occupied_confirmed",
        last_published_path_id_, localHorizonAckPending() ? "true" : "false");
  }
  const bool active_prefix_available = active_rollout_artifact_available &&
                                       rollout_runtime.progress.valid &&
                                       !rollout_runtime.exhausted;
  const double terminal_braking_distance_m = terminalBrakingDistanceM(
      navigation.speed_mps, no_static_terminal_braking_decel_mps2_,
      no_static_terminal_braking_margin_m_);
  const double current_speed_mps =
      std::max(0.0, std::isfinite(navigation.speed_mps) ? navigation.speed_mps : 0.0);
  const bool active_rollout_exhausting =
      active_prefix_available && !rollout_runtime.blocked &&
      rollout_runtime.progress.remaining_m <=
          std::max(0.4 * rollout_planner_.config().horizon_m,
                   terminal_braking_distance_m +
                       no_static_prefix_duration_s_ * current_speed_mps);
  bool current_path_kept = false;
  if (!truncation_replan.has_value()) {
    if (!planning_result.has_value()) {
      RCLCPP_ERROR(get_logger(),
                   "Planner runtime path check has no matching grid build result");
      return;
    }
    current_path_kept = keepCurrentPathIfStillClear(
        prohibited_grid, *planning_result, prepared,
        active_rollout_artifact_available ? &rollout_runtime : nullptr);
    if (current_path_kept && !active_rollout_artifact_available) {
      return;
    }
    truncation_replan = truncationReplanState();
  }
  if (truncation_replan.has_value() && !truncation_replan->confirmed) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "REPLAN_TRUNCATION planning_wait=true blocked_path_id=%" PRIu64
                         " generation=%" PRIu64,
                         truncation_replan->blocked_path_id,
                         truncation_replan->generation);
    return;
  }
  if (truncation_replan.has_value() && truncation_replan->awaiting_ack) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "REPLAN_TRUNCATION planning_wait=true reason=awaiting_suffix_ack "
        "blocked_path_id=%" PRIu64 " generation=%" PRIu64 " path_id=%" PRIu64
        " attempt=%zu",
        truncation_replan->blocked_path_id, truncation_replan->generation,
        truncation_replan->published_suffix_path_id,
        truncation_replan->publication_attempts);
    return;
  }

  TrajectoryDeliveryDiagnostics replan_delivery =
      pending_replan_delivery_.value_or(TrajectoryDeliveryDiagnostics{});
  pending_replan_delivery_.reset();
  const bool astar_planning_allowed =
      astarPlanningAllowed(use_static_map_, no_static_astar_recovery_enabled_);
  if (truncation_replan.has_value() && astar_planning_allowed &&
      runConfirmedRepairRace(*prepared, *truncation_replan, replan_delivery)) {
    return;
  }
  if (truncation_replan.has_value() && !astar_planning_allowed) {
    RCLCPP_INFO(get_logger(),
                "REPLAN_TRUNCATION repair_race_skipped=true "
                "reason=no_static_astar_disabled generation=%" PRIu64
                " action=rollout_successor",
                truncation_replan->generation);
  }

  const Point2 planning_start =
      truncation_replan.has_value()
          ? Point2{truncation_replan->position.x, truncation_replan->position.y}
          : navigation.pose.position;
  const AStarConfig planning_astar_config = astarConfigForCurrentVelocity(
      truncation_replan.has_value() ? std::optional<Point2>{truncation_replan->tangent}
                                    : std::nullopt);
  RCLCPP_INFO(
      get_logger(),
      "Planning start snapshot: cycle_sequence=%" PRIu64
      " invalidation_generation=%" PRIu64 " start=(%.2f, %.2f) pose_stamp_ns=%" PRId64
      " speed_mps=%.2f velocity_valid=%s source=%s truncation_generation=%" PRIu64,
      identity.cycle_sequence, invalidation_generation, planning_start.x,
      planning_start.y, navigation.stamp_ns, navigation.speed_mps,
      navigation.velocity_valid ? "true" : "false",
      truncation_replan.has_value() ? "confirmed_truncation" : "current_pose",
      truncation_replan.has_value() ? truncation_replan->generation : 0U);
  std::vector<TrajectoryRiskContext> risk_contexts{
      TrajectoryRiskContext{"raw_risk", &planning_grid, &prepared->risk_field,
                            &prepared->rawClearance(), true},
  };
  if (plannerModePrimaryAction(use_static_map_, no_static_rollout_enabled_) ==
      PlannerModePrimaryAction::kRollout) {
    if (localHorizonAckPending()) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "LOCAL_HORIZON publication_coalesced=true reason=awaiting_offboard_ack");
      return;
    }
    const bool truncation_rollout = truncation_replan.has_value();
    const bool rollout_prefix_available =
        active_prefix_available && !truncation_rollout;
    const double current_goal_distance_m = distance(planning_start, goal_);
    if (!std::isfinite(no_static_best_goal_distance_m_) ||
        current_goal_distance_m + 0.5 < no_static_best_goal_distance_m_) {
      no_static_best_goal_distance_m_ = current_goal_distance_m;
      no_static_last_progress_at_ = cycle_started_at;
    }
    const double seconds_since_progress =
        no_static_last_progress_at_ == std::chrono::steady_clock::time_point{}
            ? 0.0
            : std::chrono::duration<double>(cycle_started_at -
                                            no_static_last_progress_at_)
                  .count();
    if (no_static_orchestrator_.hasRecoveryGuide() &&
        distance(planning_start,
                 no_static_orchestrator_.recoveryPreferredTarget(
                     planning_start, no_static_recovery_lookahead_m_, goal_)) <= 2.0) {
      no_static_orchestrator_.clearRecoveryGuide();
    }
    Point2 rollout_start = planning_start;
    Point2 rollout_velocity =
        navigation.velocity_valid ? navigation.velocity : Point2{};
    if (truncation_rollout) {
      const double tangent_norm =
          std::hypot(truncation_replan->tangent.x, truncation_replan->tangent.y);
      const double join_speed_mps = std::max(0.0, navigation.speed_mps);
      rollout_velocity =
          tangent_norm > 1.0e-6
              ? Point2{truncation_replan->tangent.x * join_speed_mps / tangent_norm,
                       truncation_replan->tangent.y * join_speed_mps / tangent_norm}
              : Point2{};
    }
    constexpr double kTruncationHoldPositionToleranceM{1.0};
    constexpr double kTruncationHoldSpeedToleranceMps{0.5};
    const bool truncation_hold_captured =
        truncation_rollout &&
        truncationHoldCaptured(
            navigation.pose.position, navigation.speed_mps,
            Point2{truncation_replan->position.x, truncation_replan->position.y},
            kTruncationHoldPositionToleranceM, kTruncationHoldSpeedToleranceMps);
    bool stationary_restart =
        truncation_rollout
            ? truncation_replan->immediate_hold || truncation_hold_captured
            : active_rollout_artifact_available && rollout_runtime.exhausted &&
                  !rollout_runtime.blocked;
    if (truncation_hold_captured) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "REPLAN_TRUNCATION stationary_restart=true generation=%" PRIu64
          " distance=%.2f speed=%.2f position_tolerance=%.2f speed_tolerance=%.2f",
          truncation_replan->generation,
          distance(navigation.pose.position, Point2{truncation_replan->position.x,
                                                    truncation_replan->position.y}),
          navigation.speed_mps, kTruncationHoldPositionToleranceM,
          kTruncationHoldSpeedToleranceMps);
    }
    if (stationary_restart) {
      rollout_velocity = Point2{};
    }
    double stable_prefix_distance_m = 0.0;
    if (rollout_prefix_available) {
      stable_prefix_distance_m =
          std::max(0.0, navigation.speed_mps * no_static_prefix_duration_s_);
      const double join_s_m = std::min(
          executable_trajectory_artifact_.samples.back().s_m,
          executable_trajectory_artifact_.current_s_m + stable_prefix_distance_m);
      const TrajectoryPointSample join =
          trajectorySampleAtS(executable_trajectory_artifact_.samples, join_s_m);
      rollout_start = join.point;
      const double join_speed_mps = std::max(0.0, navigation.speed_mps);
      rollout_velocity =
          Point2{join.tangent.x * join_speed_mps, join.tangent.y * join_speed_mps};
    }
    const Point2 mission_or_recovery_target =
        no_static_orchestrator_.recoveryPreferredTarget(
            planning_start, no_static_recovery_lookahead_m_, goal_);
    const Point2 preferred_target = mission_or_recovery_target;
    if (stationary_restart) {
      rollout_velocity = Point2{};
    }
    const bool mission_goal_within_minimum_length =
        distance(rollout_start, goal_) <= no_static_rollout_min_length_m_ + 1.0e-6;
    const double required_rollout_length_m =
        mission_goal_within_minimum_length
            ? 0.0
            : std::max(no_static_rollout_min_length_m_,
                       stationary_restart ? 0.0 : terminal_braking_distance_m);
    const double terminal_response_clearance_m =
        stationary_restart ? 0.0
                           : current_speed_mps * no_static_terminal_response_delay_s_ +
                                 std::sqrt(0.5) * planning_grid.resolution();
    const double vehicle_clearance_envelope_m =
        no_static_vehicle_clearance_m_ + no_static_tracking_error_margin_m_ +
        std::sqrt(0.5) * planning_grid.resolution();
    const bool blocked_replacement_context =
        truncation_rollout || rollout_runtime.blocked;
    const auto rollout_started_at = std::chrono::steady_clock::now();
    ++rollout_cycles_;
    bool validated_candidate_seen = false;
    bool rollout_recovery_requested = false;
    RolloutRejectReason last_rollout_reject = RolloutRejectReason::kNoCandidate;
    std::size_t total_rollout_candidates = 0U;
    for (std::size_t risk_attempt_index = 0U; risk_attempt_index < risk_contexts.size();
         ++risk_attempt_index) {
      const TrajectoryRiskContext& risk_attempt = risk_contexts[risk_attempt_index];
      const std::optional<PathRiskScore> active_suffix_risk =
          active_prefix_available
              ? evaluateExecutableSuffixRisk(
                    *risk_attempt.raw_occupancy, *risk_attempt.risk_field,
                    executable_trajectory_artifact_, rollout_runtime.progress)
              : std::nullopt;
      RCLCPP_INFO(
          get_logger(),
          "ROLLOUT_INPUT generation=%" PRIu64 " grid_revision=%" PRIu64
          " grid=%s start=(%.2f,%.2f) velocity=(%.2f,%.2f) speed=%.2f "
          "preferred_target=(%.2f,%.2f) target_distance=%.2f "
          "vehicle_clearance_envelope=%.2f tracking_error_margin=%.2f "
          "terminal_response_clearance=%.2f "
          "terminal_braking_decel=%.2f terminal_braking_margin=%.2f "
          "target_source=mission_or_recovery "
          "active_path=%s active_path_id=%" PRIu64 " active_s=%.2f "
          "active_remaining=%.2f stable_prefix_m=%.2f "
          "grid_size=%dx%d resolution=%.2f",
          invalidation_generation, prepared->version.build_revision,
          std::string{risk_attempt.name}.c_str(), rollout_start.x, rollout_start.y,
          rollout_velocity.x, rollout_velocity.y,
          std::hypot(rollout_velocity.x, rollout_velocity.y), preferred_target.x,
          preferred_target.y, distance(rollout_start, preferred_target),
          vehicle_clearance_envelope_m, no_static_tracking_error_margin_m_,
          terminal_response_clearance_m, no_static_terminal_braking_decel_mps2_,
          no_static_terminal_braking_margin_m_,
          rollout_prefix_available ? "true" : "false",
          active_rollout_artifact_available ? executable_trajectory_artifact_.path_id
                                            : 0U,
          active_rollout_artifact_available
              ? executable_trajectory_artifact_.current_s_m
              : 0.0,
          active_rollout_artifact_available ? rollout_runtime.progress.remaining_m
                                            : 0.0,
          stable_prefix_distance_m, risk_attempt.raw_occupancy->width(),
          risk_attempt.raw_occupancy->height(),
          risk_attempt.raw_occupancy->resolution());
      const auto rollout_generation_started_at = std::chrono::steady_clock::now();
      const RolloutResult rollout = rollout_planner_.plan(RolloutInput{
          .position = rollout_start,
          .velocity = rollout_velocity,
          .preferred_target = preferred_target,
          .grid = risk_attempt.raw_occupancy,
          .risk_field = &prepared->risk_field,
          .minimum_length_m = required_rollout_length_m,
          .minimum_path_clearance_m = vehicle_clearance_envelope_m,
          .minimum_terminal_clearance_m = terminal_response_clearance_m,
          .terminal_braking_deceleration_mps2 = no_static_terminal_braking_decel_mps2_,
          .terminal_braking_margin_m = no_static_terminal_braking_margin_m_,
          .stationary_restart = stationary_restart,
          .generation = invalidation_generation,
          .grid_revision = prepared->version.build_revision,
      });
      const double rollout_generation_ms =
          elapsedMilliseconds(rollout_generation_started_at);
      const RolloutGridRejectionDiagnostic* first_grid_rejection =
          rollout.diagnostics.first_grid_rejection.has_value()
              ? &*rollout.diagnostics.first_grid_rejection
              : nullptr;
      RCLCPP_INFO(
          get_logger(),
          "ROLLOUT_GENERATION generation=%" PRIu64 " grid_revision=%" PRIu64
          " generated=%zu accepted=%zu grid_rejected=%zu outside_rejected=%zu "
          "dynamic_rejected=%zu dynamic_breakdown[acceleration=%zu curvature=%zu "
          "lateral_acceleration=%zu] first_grid_rejection[reason=%s "
          "candidate_index=%zu segment=%zu position=(%.2f,%.2f) "
          "cell=(%d,%d) has_cell=%s] duration_ms=%.2f",
          invalidation_generation, prepared->version.build_revision,
          rollout.diagnostics.generated, rollout.ranked_candidates.size(),
          rollout.diagnostics.grid_rejections,
          rollout.diagnostics.outside_grid_rejections,
          rollout.diagnostics.dynamic_limit_rejections,
          rollout.diagnostics.acceleration_rejections,
          rollout.diagnostics.curvature_rejections,
          rollout.diagnostics.lateral_acceleration_rejections,
          first_grid_rejection != nullptr
              ? rolloutGridRejectReasonName(first_grid_rejection->reason)
              : "none",
          first_grid_rejection != nullptr ? first_grid_rejection->deterministic_index
                                          : 0U,
          first_grid_rejection != nullptr ? first_grid_rejection->segment_index : 0U,
          first_grid_rejection != nullptr ? first_grid_rejection->position.x : 0.0,
          first_grid_rejection != nullptr ? first_grid_rejection->position.y : 0.0,
          first_grid_rejection != nullptr && first_grid_rejection->cell.has_value()
              ? first_grid_rejection->cell->x
              : -1,
          first_grid_rejection != nullptr && first_grid_rejection->cell.has_value()
              ? first_grid_rejection->cell->y
              : -1,
          first_grid_rejection != nullptr && first_grid_rejection->cell.has_value()
              ? "true"
              : "false",
          rollout_generation_ms);
      last_rollout_reject = rollout.reject_reason;
      total_rollout_candidates += rollout.ranked_candidates.size();
      rollout_candidates_ += rollout.ranked_candidates.size();
      std::size_t finalist_index = 0U;
      for (const RolloutCandidate& finalist :
           rollout.rankedShortlist(rollout_planner_.config().max_finalists)) {
        ++finalist_index;
        std::vector<TrajectoryPointSample> geometry_samples = finalist.samples;
        if (rollout_prefix_available) {
          const StablePrefixStitchResult stitch =
              stitchStableExecutablePrefix(executable_trajectory_artifact_.samples,
                                           executable_trajectory_artifact_.current_s_m,
                                           stable_prefix_distance_m, finalist.samples);
          if (!stitch.valid) {
            const double expected_join_s_m = std::min(
                executable_trajectory_artifact_.samples.back().s_m,
                executable_trajectory_artifact_.current_s_m + stable_prefix_distance_m);
            const TrajectoryPointSample expected_join = trajectorySampleAtS(
                executable_trajectory_artifact_.samples, expected_join_s_m);
            RCLCPP_WARN(
                get_logger(),
                "ROLLOUT_STITCH_REJECT generation=%" PRIu64
                " finalist=%zu candidate_index=%zu current_s=%.2f "
                "prefix_distance=%.2f expected_join=(%.2f,%.2f) "
                "successor_start=(%.2f,%.2f) endpoint_error=%.2f",
                invalidation_generation, finalist_index, finalist.deterministic_index,
                executable_trajectory_artifact_.current_s_m, stable_prefix_distance_m,
                expected_join.point.x, expected_join.point.y,
                finalist.samples.empty() ? 0.0 : finalist.samples.front().point.x,
                finalist.samples.empty() ? 0.0 : finalist.samples.front().point.y,
                finalist.samples.empty()
                    ? std::numeric_limits<double>::infinity()
                    : distance(expected_join.point, finalist.samples.front().point));
            continue;
          }
          geometry_samples = stitch.samples;
        }
        double rollout_start_altitude_m =
            navigation.altitude_valid ? navigation.altitude_m : initial_altitude_m_;
        if (truncation_rollout) {
          rollout_start_altitude_m = truncation_replan->altitude_m;
        }
        assignTrajectorySampleAltitude(geometry_samples, rollout_start_altitude_m);
        const std::array<TrajectoryRiskContext, 1U> finalization_grids{risk_attempt};
        const auto candidate_finalization_started_at = std::chrono::steady_clock::now();
        const TrajectoryPlannerResult finalized = finalizeStitchedTrajectory(
            StitchedTrajectoryFinalizationInput{
                .geometry_samples = geometry_samples,
                .known_passage_map =
                    known_passages_.has_value() ? &*known_passages_ : nullptr,
                .risk_contexts = finalization_grids,
                .start_mode = stationary_restart
                                  ? PassageInsertionStartMode::kTerminalHoldRestart
                                  : PassageInsertionStartMode::kMovingJoin,
            },
            trajectoryPlannerConfigForCurrentAltitude(
                truncation_rollout
                    ? std::optional<double>{truncation_replan->altitude_m}
                    : std::nullopt));
        const double candidate_finalization_ms =
            elapsedMilliseconds(candidate_finalization_started_at);
        RCLCPP_INFO(
            get_logger(),
            "NO_STATIC_ROLLOUT generation=%" PRIu64 " grid_revision=%" PRIu64
            " risk_attempt=%zu/%zu grid=%s finalist=%zu/%zu score=%.3f "
            "score_parts[progress=%.3f lateral=%.3f heading=%.3f "
            "curvature=%.3f] progress=%.2fm candidate_index=%zu samples=%zu "
            "endpoint=(%.2f,%.2f) heading_offset_rad=%.3f target_speed=%.2f "
            "terminal_stopping_m=%.2f curvature_1pm=%.4f valid=%s "
            "status=%.*s quality=%.*s "
            "finalization[vertical_valid=%s passage_valid=%s passage_reason=%s "
            "solid_valid=%s solid_reason=%s insertion_required=%s "
            "insertion_satisfied=%s insertion_reason=%s] generated=%zu "
            "grid_rejected=%zu outside_rejected=%zu dynamic_rejected=%zu "
            "mode=%s prefix_m=%.2f suffix_m=%.2f guide_revision=%" PRIu64
            " rollout_generation_ms=%.2f candidate_finalization_ms=%.2f",
            invalidation_generation, prepared->version.build_revision,
            risk_attempt_index + 1U, risk_contexts.size(),
            std::string{risk_attempt.name}.c_str(), finalist_index,
            rollout.rankedShortlist(rollout_planner_.config().max_finalists).size(),
            finalist.score, finalist.progress_cost, finalist.lateral_deviation_cost,
            finalist.heading_change_cost, finalist.curvature_cost, finalist.progress_m,
            finalist.deterministic_index, finalist.samples.size(),
            finalist.samples.empty() ? 0.0 : finalist.samples.back().point.x,
            finalist.samples.empty() ? 0.0 : finalist.samples.back().point.y,
            finalist.heading_offset_rad, finalist.target_speed_mps,
            finalist.terminal_stopping_distance_m, finalist.curvature_1pm,
            finalized.valid ? "true" : "false",
            static_cast<int>(
                trajectoryPlannerStatusName(finalized.stats.status).size()),
            trajectoryPlannerStatusName(finalized.stats.status).data(),
            static_cast<int>(trajectoryQualityName(finalized.stats.quality).size()),
            trajectoryQualityName(finalized.stats.quality).data(),
            finalized.stats.vertical_profile.valid ? "true" : "false",
            finalized.stats.known_passage_validation.valid ? "true" : "false",
            knownPassageValidationReasonName(
                finalized.stats.known_passage_validation.worst_reason),
            finalized.stats.known_passage_solid_validation.valid ? "true" : "false",
            knownPassageSolidValidationReasonName(
                finalized.stats.known_passage_solid_validation.reason),
            finalized.stats.passage_insertion.repair_required ? "true" : "false",
            finalized.stats.passage_insertion.repair_satisfied ? "true" : "false",
            passageInsertionRejectReasonName(
                finalized.stats.passage_insertion.final_reason),
            rollout.diagnostics.generated, rollout.diagnostics.grid_rejections,
            rollout.diagnostics.outside_grid_rejections,
            rollout.diagnostics.dynamic_limit_rejections,
            noStaticPlannerModeName(no_static_orchestrator_.mode()),
            stable_prefix_distance_m,
            finalist.samples.empty() ? 0.0 : finalist.samples.back().s_m,
            no_static_orchestrator_.recoveryGuideRevision(), rollout_generation_ms,
            candidate_finalization_ms);
        if (!finalized.valid) {
          continue;
        }
        const double candidate_remaining_m =
            finalized.samples.empty() ? 0.0 : finalized.samples.back().s_m;
        constexpr double kRolloutLengthToleranceM{0.5};
        const bool terminal_length_sufficient =
            candidate_remaining_m + kRolloutLengthToleranceM >=
            required_rollout_length_m;
        if (!terminal_length_sufficient) {
          RCLCPP_WARN(
              get_logger(),
              "NO_STATIC_ROLLOUT terminal_length_rejected=true generation=%" PRIu64
              " finalist=%zu remaining=%.2f required=%.2f minimum=%.2f "
              "braking=%.2f tolerance=%.2f stationary_restart=%s "
              "mission_goal_exception=%s speed=%.2f active_blocked=%s action=%s",
              invalidation_generation, finalist_index, candidate_remaining_m,
              required_rollout_length_m, no_static_rollout_min_length_m_,
              terminal_braking_distance_m, kRolloutLengthToleranceM,
              stationary_restart ? "true" : "false",
              mission_goal_within_minimum_length ? "true" : "false",
              navigation.speed_mps, blocked_replacement_context ? "true" : "false",
              blocked_replacement_context ? "safe_truncation_hold"
                                          : "keep_clear_prefix_and_retry");
          continue;
        }
        validated_candidate_seen = true;
        const std::uint64_t latest_generation = latestPlanningInvalidationGeneration();
        const std::uint64_t latest_grid_revision =
            reuse_successor_snapshot
                ? prepared->version.build_revision
                : planning_grid_snapshot_builder_.nextRevision() - 1U;
        const NoStaticPlannerDecision orchestration =
            no_static_orchestrator_.decide(NoStaticPlannerDecisionInput{
                .generation = invalidation_generation,
                .latest_generation = latest_generation,
                .grid_revision = prepared->version.build_revision,
                .latest_grid_revision = latest_grid_revision,
                .candidate_valid = true,
                .active_prefix_available = rollout_prefix_available,
                .active_suffix_blocked = truncation_rollout || rollout_runtime.blocked,
                .active_suffix_exhausting = active_rollout_exhausting,
                .temporary_hold_active =
                    !rollout_prefix_available && last_valid_path_points_.empty(),
                .candidate_score = finalist.score,
                .active_score = active_rollout_score_,
                .candidate_risk = finalized.stats.final_risk,
                .active_risk = active_suffix_risk,
                .seconds_since_progress = seconds_since_progress,
                .candidate_heading_offset_rad = finalist.heading_offset_rad,
            });
        RCLCPP_INFO(
            get_logger(),
            "ROLLOUT_DECISION generation=%" PRIu64 " finalist=%zu "
            "candidate_index=%zu action=%s mode=%s candidate_score=%.3f "
            "active_score=%.3f active_score_valid=%s seconds_since_progress=%.2f "
            "candidate_risk=%s active_risk=%s active_risk_valid=%s "
            "active_blocked=%s active_exhausting=%s temporary_hold=%s "
            "candidate_generation=%" PRIu64 " latest_generation=%" PRIu64
            " candidate_grid_revision=%" PRIu64 " latest_grid_revision=%" PRIu64,
            invalidation_generation, finalist_index, finalist.deterministic_index,
            noStaticPlannerActionName(orchestration.action),
            noStaticPlannerModeName(orchestration.mode), finalist.score,
            active_rollout_score_.value_or(0.0),
            active_rollout_score_.has_value() ? "true" : "false",
            seconds_since_progress,
            obstacleRiskTierName(finalized.stats.final_risk.worst_tier),
            active_suffix_risk.has_value()
                ? obstacleRiskTierName(active_suffix_risk->worst_tier)
                : "none",
            active_suffix_risk.has_value() ? "true" : "false",
            rollout_runtime.blocked ? "true" : "false",
            active_rollout_exhausting ? "true" : "false",
            !rollout_prefix_available && last_valid_path_points_.empty() ? "true"
                                                                         : "false",
            invalidation_generation, latest_generation,
            prepared->version.build_revision, latest_grid_revision);
        if (orchestration.action == NoStaticPlannerAction::kRejectStale) {
          RCLCPP_WARN(
              get_logger(),
              "ROLLOUT_REJECT_STALE candidate_generation=%" PRIu64
              " latest_generation=%" PRIu64 " invalidation_reason=%s "
              "cycle_sequence=%" PRIu64,
              invalidation_generation, latest_generation,
              planningInvalidationReasonName(latestPlanningInvalidationReason()),
              identity.cycle_sequence);
          schedulePlanningCycle(PlanningWakeReason::kStaleRetry);
          return;
        }
        if (orchestration.action == NoStaticPlannerAction::kKeep) {
          return;
        }
        if (orchestration.action == NoStaticPlannerAction::kRequestRecovery) {
          if (no_static_astar_recovery_enabled_) {
            ++rollout_recovery_requests_;
            rollout_recovery_requested = true;
            break;
          }
          RCLCPP_WARN(get_logger(),
                      "NO_STATIC_ROLLOUT astar_recovery_disabled=true "
                      "requested_after_valid_candidate=true action=publish_rollout "
                      "generation=%" PRIu64,
                      invalidation_generation);
        }
        if (orchestration.action == NoStaticPlannerAction::kHold) {
          publishPath({}, PathPublicationReason::kHoldAfterPlanningFailure, nullptr,
                      TrajectoryEndpointSemantics::kTemporaryReplanHold);
          return;
        }
        const bool reaches_mission_goal =
            distance(finalized.samples.back().point, goal_) <=
            stable_path_goal_tolerance_m_;
        TrajectoryDeliveryDiagnostics rollout_delivery = replan_delivery;
        rollout_delivery.generation = invalidation_generation;
        rollout_delivery.activate_after_terminal_hold = stationary_restart;
        rollout_delivery.planning_algorithm = PlanningAlgorithm::kRollout;
        if (truncation_rollout) {
          rollout_delivery.blocked_path_id = truncation_replan->blocked_path_id;
          rollout_delivery.truncation_generation = truncation_replan->generation;
          rollout_delivery.temporary_prefix_fingerprint =
              truncation_replan->temporary_prefix_fingerprint;
          rollout_delivery.truncation_suffix = true;
          rollout_delivery.truncation_immediate_hold =
              truncation_replan->immediate_hold;
          rollout_delivery.truncation_suffix_activation_mode =
              static_cast<std::uint8_t>(
                  stationary_restart ? TruncationSuffixActivationMode::kAfterHold
                                     : TruncationSuffixActivationMode::kMovingJoin);
          rollout_delivery.planning_start_position = planning_start;
        }
        std::uint64_t published_path_id = 0U;
        const std::vector<Point2> route_points =
            trajectorySamplePoints(finalized.samples);
        const std::string grid_name{risk_attempt.name};
        TrajectoryPublicationStageTimings stage_timings{
            .grid_build_ms = planning_grid_duration_ms,
            .rollout_generation_ms = rollout_generation_ms,
            .candidate_finalization_ms = candidate_finalization_ms,
        };
        const bool published = publishTrajectoryResult(
            finalized, route_points, "no_static_rollout",
            elapsedMilliseconds(cycle_started_at), rollout_delivery, grid_name,
            grid_name, &published_path_id, &prepared->version,
            reaches_mission_goal ? TrajectoryEndpointSemantics::kMissionGoal
                                 : TrajectoryEndpointSemantics::kLocalHorizon,
            &stage_timings, risk_attempt.raw_occupancy, risk_attempt.risk_field,
            risk_attempt.raw_clearance, finalist.score);
        stage_timings.publication_total_ms = elapsedMilliseconds(cycle_started_at);
        const double blocker_to_successor_publish_ms =
            truncation_rollout && successor_snapshot != nullptr &&
                    successor_snapshot->blocker_detected_stamp_ns > 0 &&
                    get_clock()->now().nanoseconds() >=
                        successor_snapshot->blocker_detected_stamp_ns
                ? 1.0e-6 *
                      static_cast<double>(get_clock()->now().nanoseconds() -
                                          successor_snapshot->blocker_detected_stamp_ns)
                : std::numeric_limits<double>::quiet_NaN();
        constexpr double kRolloutPublicationDeadlineMs{500.0};
        const double deadline_duration_ms =
            std::isfinite(blocker_to_successor_publish_ms)
                ? blocker_to_successor_publish_ms
                : stage_timings.publication_total_ms;
        const bool rollout_deadline_missed =
            deadline_duration_ms > kRolloutPublicationDeadlineMs;
        if (rollout_deadline_missed) {
          ++rollout_deadline_missed_;
        }
        RCLCPP_INFO(
            get_logger(),
            "NO_STATIC_ROLLOUT_TIMING generation=%" PRIu64 " grid_revision=%" PRIu64
            " finalist=%zu published=%s "
            "rollout_generation_ms=%.2f candidate_finalization_ms=%.2f "
            "grid_build_ms=%.2f fresh_grid_build_ms=not_run "
            "fresh_grid_prepare_ms=not_run final_validation_ms=%.2f "
            "publication_total_ms=%.2f rollout_deadline_ms=%.2f "
            "rollout_deadline_missed=%s",
            invalidation_generation, prepared->version.build_revision, finalist_index,
            published ? "true" : "false", stage_timings.rollout_generation_ms,
            stage_timings.candidate_finalization_ms, stage_timings.grid_build_ms,
            stage_timings.final_validation_ms, stage_timings.publication_total_ms,
            kRolloutPublicationDeadlineMs, rollout_deadline_missed ? "true" : "false");
        if (truncation_rollout) {
          RCLCPP_INFO(
              get_logger(),
              "ROLLOUT_SUCCESSOR_TIMING generation=%" PRIu64 " grid_revision=%" PRIu64
              " snapshot_reused=%s snapshot_age_ms=%.2f grid_build_ms=%.2f "
              "clearance_build_ms=0.00 rollout_generation_ms=%.2f "
              "candidate_finalization_ms=%.2f final_validation_ms=%.2f "
              "blocker_to_successor_publish_ms=%.2f rollout_deadline_missed=%s",
              truncation_replan->generation, prepared->version.build_revision,
              reuse_successor_snapshot ? "true" : "false",
              blocker_to_successor_publish_ms, stage_timings.grid_build_ms,
              stage_timings.rollout_generation_ms,
              stage_timings.candidate_finalization_ms,
              stage_timings.final_validation_ms, blocker_to_successor_publish_ms,
              rollout_deadline_missed ? "true" : "false");
        }
        if (published) {
          ++rollout_publications_;
          return;
        }
      }
      if (validated_candidate_seen || rollout_recovery_requested) {
        break;
      }
      RCLCPP_INFO(get_logger(),
                  "NO_STATIC_ROLLOUT risk_attempt_exhausted=true "
                  "generation=%" PRIu64 " grid_revision=%" PRIu64
                  " risk_attempt=%zu/%zu grid=%s reject=%s candidates=%zu "
                  "fallback=%s",
                  invalidation_generation, prepared->version.build_revision,
                  risk_attempt_index + 1U, risk_contexts.size(),
                  std::string{risk_attempt.name}.c_str(),
                  rolloutRejectReasonName(rollout.reject_reason),
                  rollout.ranked_candidates.size(),
                  risk_attempt_index + 1U < risk_contexts.size()
                      ? std::string{risk_contexts[risk_attempt_index + 1U].name}.c_str()
                      : "none");
    }
    rollout_durations_ms_.push_back(elapsedMilliseconds(rollout_started_at));
    constexpr std::size_t kMaximumRolloutDurationSamples{256U};
    if (rollout_durations_ms_.size() > kMaximumRolloutDurationSamples) {
      rollout_durations_ms_.erase(rollout_durations_ms_.begin());
    }
    if (validated_candidate_seen && !rollout_recovery_requested) {
      return;
    }
    if (truncation_rollout &&
        noteTruncationSuccessorPlanningReject(truncation_replan->generation, 3U)) {
      RCLCPP_WARN(get_logger(),
                  "ROLLOUT_SUCCESSOR_SNAPSHOT generation=%" PRIu64
                  " invalidated=true reason=repeated_planning_reject rejection_limit=3 "
                  "action=rebuild_current_snapshot",
                  truncation_replan->generation);
      schedulePlanningCycle(PlanningWakeReason::kRetry);
      return;
    }
    if (!rollout_recovery_requested) {
      RCLCPP_WARN(get_logger(),
                  "NO_STATIC_ROLLOUT no_validated_candidate=true generation=%" PRIu64
                  " grid_revision=%" PRIu64 " reject=%s candidates=%zu; "
                  "requesting A* recovery",
                  invalidation_generation, prepared->version.build_revision,
                  rolloutRejectReasonName(last_rollout_reject),
                  total_rollout_candidates);
      const NoStaticPlannerDecision failure_decision =
          no_static_orchestrator_.decide(NoStaticPlannerDecisionInput{
              .generation = invalidation_generation,
              .latest_generation = latestPlanningInvalidationGeneration(),
              .grid_revision = prepared->version.build_revision,
              .latest_grid_revision =
                  reuse_successor_snapshot
                      ? prepared->version.build_revision
                      : planning_grid_snapshot_builder_.nextRevision() - 1U,
              .candidate_valid = false,
              .active_prefix_available = rollout_prefix_available,
              .temporary_hold_active =
                  !rollout_prefix_available && last_valid_path_points_.empty(),
              .active_score = active_rollout_score_,
              .seconds_since_progress = seconds_since_progress,
          });
      if (failure_decision.action == NoStaticPlannerAction::kRejectStale) {
        const std::uint64_t latest_generation = latestPlanningInvalidationGeneration();
        RCLCPP_WARN(get_logger(),
                    "ROLLOUT_REJECT_STALE candidate_generation=%" PRIu64
                    " latest_generation=%" PRIu64 " invalidation_reason=%s "
                    "cycle_sequence=%" PRIu64,
                    invalidation_generation, latest_generation,
                    planningInvalidationReasonName(latestPlanningInvalidationReason()),
                    identity.cycle_sequence);
        schedulePlanningCycle(PlanningWakeReason::kStaleRetry);
        return;
      }
      if (failure_decision.action != NoStaticPlannerAction::kRequestRecovery) {
        ++rollout_failures_;
        return;
      }
      if (!no_static_astar_recovery_enabled_) {
        ++rollout_failures_;
        no_static_orchestrator_.clearRecoveryGuide();
        RCLCPP_WARN(get_logger(),
                    "NO_STATIC_ROLLOUT astar_recovery_disabled=true "
                    "no_validated_candidate=true action=%s generation=%" PRIu64,
                    rollout_prefix_available ? "keep_active_prefix_and_retry"
                    : truncation_rollout     ? "retry_from_truncation_point"
                                             : "restart_from_current_pose",
                    invalidation_generation);
        return;
      }
      ++rollout_recovery_requests_;
    }
  }
  if (!astar_planning_allowed) {
    RCLCPP_WARN(get_logger(),
                "A_STAR_SUPPRESSED map_mode=no_static recovery_enabled=false "
                "truncation_generation=%" PRIu64 " action=keep_or_retry_rollout",
                truncation_replan.has_value() ? truncation_replan->generation : 0U);
    return;
  }
  std::optional<PathComputationResult> path_result;
  std::size_t astar_grid_index = 0U;
  Point2 astar_goal = goal_;
  const bool bounded_no_static_recovery =
      plannerModePrimaryAction(use_static_map_, no_static_rollout_enabled_) ==
          PlannerModePrimaryAction::kRollout &&
      !truncation_replan.has_value();
  bool recovery_goal_available = true;
  if (bounded_no_static_recovery) {
    const std::optional<Point2> recovery_goal = boundedNoStaticRecoveryGoal(
        planning_grid, prepared->risk_field, planning_start, goal_);
    recovery_goal_available = recovery_goal.has_value();
    if (recovery_goal.has_value()) {
      astar_goal = *recovery_goal;
      RCLCPP_INFO(get_logger(),
                  "NO_STATIC_ROLLOUT recovery_endpoint=bounded_reachable "
                  "start=(%.2f,%.2f) "
                  "endpoint=(%.2f,%.2f) mission_goal=(%.2f,%.2f) "
                  "evaluation_origin=(%.2f,%.2f) evaluation_size=%dx%d",
                  planning_start.x, planning_start.y, astar_goal.x, astar_goal.y,
                  goal_.x, goal_.y, prepared->evaluation_bounds.origin_x,
                  prepared->evaluation_bounds.origin_y,
                  prepared->evaluation_bounds.width_cells,
                  prepared->evaluation_bounds.height_cells);
    } else {
      RCLCPP_WARN(get_logger(),
                  "NO_STATIC_ROLLOUT recovery_endpoint_unavailable=true "
                  "start=(%.2f,%.2f) mission_goal=(%.2f,%.2f)",
                  planning_start.x, planning_start.y, goal_.x, goal_.y);
    }
  }
  if (recovery_goal_available) {
    for (; astar_grid_index < risk_contexts.size(); ++astar_grid_index) {
      const TrajectoryRiskContext& candidate = risk_contexts[astar_grid_index];
      const std::string candidate_name{candidate.name};
      path_result = computePathOnGrid(*candidate.raw_occupancy, *candidate.risk_field,
                                      candidate_name.c_str(), planning_astar_config,
                                      planning_start, astar_goal);
      if (path_result.has_value()) {
        break;
      }
    }
  }
  if (!path_result.has_value()) {
    if (plannerModePrimaryAction(use_static_map_, no_static_rollout_enabled_) ==
            PlannerModePrimaryAction::kRollout &&
        !truncation_replan.has_value()) {
      const NoStaticPlannerDecision failure =
          no_static_orchestrator_.decideRecoveryFailure(active_prefix_available);
      RCLCPP_WARN(get_logger(),
                  "NO_STATIC_ROLLOUT recovery_failed=true action=%s "
                  "active_prefix=%s",
                  noStaticPlannerActionName(failure.action),
                  active_prefix_available ? "true" : "false");
      if (failure.action == NoStaticPlannerAction::kHold) {
        publishPath({}, PathPublicationReason::kHoldAfterPlanningFailure, nullptr,
                    TrajectoryEndpointSemantics::kTemporaryReplanHold);
      }
      return;
    }
    publishPlanningFailureHold();
    return;
  }
  if (plannerModePrimaryAction(use_static_map_, no_static_rollout_enabled_) ==
          PlannerModePrimaryAction::kRollout &&
      !truncation_replan.has_value()) {
    const std::vector<GridIndex>& guide_cells = path_result->smoothed_cells.empty()
                                                    ? path_result->astar.path
                                                    : path_result->smoothed_cells;
    if (guide_cells.empty()) {
      return;
    }
    constexpr std::size_t kMaximumRecoveryGuidePoints{512U};
    std::vector<Point2> guide;
    guide.reserve(std::min(guide_cells.size(), kMaximumRecoveryGuidePoints));
    for (const GridIndex cell : std::span<const GridIndex>{guide_cells}.first(
             std::min(guide_cells.size(), kMaximumRecoveryGuidePoints))) {
      guide.push_back(risk_contexts[astar_grid_index].raw_occupancy->cellCenter(cell));
    }
    no_static_orchestrator_.setRecoveryGuide(guide, prepared->version.build_revision);
    const Point2 guide_target = no_static_orchestrator_.recoveryPreferredTarget(
        planning_start, no_static_recovery_lookahead_m_, goal_);
    RCLCPP_WARN(get_logger(),
                "NO_STATIC_ROLLOUT recovery_guide_ready=true generation=%" PRIu64
                " guide_revision=%" PRIu64 " lookahead=(%.2f,%.2f) "
                "guide_cells=%zu; A* path is not published",
                invalidation_generation,
                no_static_orchestrator_.recoveryGuideRevision(), guide_target.x,
                guide_target.y, guide_cells.size());
    schedulePlanningCycle(PlanningWakeReason::kRecoveryGuideReady);
    return;
  }
  const std::string astar_grid_name{risk_contexts[astar_grid_index].name};
  RCLCPP_INFO(get_logger(),
              "GRID_STAGE_SELECTED stage=astar grid=%s attempt=%zu candidates=%zu",
              astar_grid_name.c_str(), astar_grid_index + 1U, risk_contexts.size());
  const GridStats raw_grid_stats = collectGridStats(prepared->raw_occupancy);
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Planning summary: pose=(%.2f, %.2f) distance_to_start=%.2f "
      "distance_to_goal=%.2f raw[occupied=%zu free=%zu unknown=%zu] "
      "sources[static=%zu memory=%zu current_lidar=%zu] "
      "risk_policy[critical=%.2f preferred=%.2f] "
      "snapshot[producer=%" PRIu64 " revision=%" PRIu64 " policy=%" PRIu64 "] "
      "source=%s astar_status=%s heuristic_weight=%.2f expanded=%zu "
      "cost=%.2f raw_path=%zu smoothed_path=%zu "
      "path_risk[tier=%s critical_exposure=%.2f planning_exposure=%.2f "
      "min_raw_clearance=%.2f] "
      "timing[risk_snapshot=%.1f path_total=%.1f astar=%.1f smoothing=%.1f]",
      current_pose_.position.x, current_pose_.position.y,
      distance(current_pose_.position, start_), distance(current_pose_.position, goal_),
      raw_grid_stats.occupied_cells, raw_grid_stats.free_cells,
      raw_grid_stats.unknown_cells, planning_result->static_source.occupied_cells,
      planning_result->memory.source_counts.occupied_cells,
      planning_result->current_lidar.occupied_cells,
      prepared->risk_field.policy().critical_distance_m,
      prepared->risk_field.policy().preferred_distance_m,
      raw_obstacle_producer_instance_id_, prepared->version.build_revision,
      prepared->version.risk_policy_fingerprint, astar_grid_name.c_str(),
      astarStatusName(path_result->astar.status),
      planning_astar_config.heuristic_weight, path_result->astar.expanded_cells,
      path_result->astar.total_cost, path_result->raw_path_metrics.points,
      path_result->smoothed_path_metrics.points,
      obstacleRiskTierName(path_result->astar.risk.worst_tier),
      path_result->astar.risk.critical_exposure_m,
      path_result->astar.risk.planning_exposure_m,
      path_result->astar.risk.minimum_raw_clearance_m, planning_grid_duration_ms,
      path_result->total_duration_ms, path_result->astar_duration_ms,
      path_result->smoothing_duration_ms);
  const LidarIngestionDecisionStats& lidar_decisions =
      planning_result->current_lidar.ingestion_decisions;
  const std::string lidar_decision_samples =
      formatLidarIngestionRepresentativeDiagnostics(lidar_decisions);
  if (lidar_decisions.invariant_fallbacks > 0U) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planner current lidar replaced %zu malformed accepted decisions with "
        "conservative no-expected-surface metadata: %s",
        lidar_decisions.invariant_fallbacks, lidar_decision_samples.c_str());
  }
  if (!lidar_decision_samples.empty()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Planner current lidar decision samples: %s",
                         lidar_decision_samples.c_str());
  }
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
  PathPublicationOutcome publication_outcome = publishPathFromPathCells(
      *planning_result, risk_contexts, astar_grid_index, path_result->astar.path,
      path_result->smoothed_cells, astar_grid_name.c_str(), planning_start,
      replan_delivery, PassageInsertionStartMode::kMovingJoin,
      truncation_replan.has_value() ? &*truncation_replan : nullptr);
  if (publication_outcome == PathPublicationOutcome::kRetryAfterTerminalHold &&
      truncation_replan.has_value()) {
    publication_outcome = publishTerminalHoldRestartSuffix(
        *planning_result, risk_contexts, planning_start, *truncation_replan,
        replan_delivery);
    if (publication_outcome != PathPublicationOutcome::kPublished) {
      publishPlanningFailureHold();
    }
  }
  const double cycle_duration_s = elapsedMilliseconds(cycle_started_at) * 1.0e-3;
  RCLCPP_INFO(get_logger(),
              "Planning worker cycle complete: cycle_sequence=%" PRIu64
              " invalidation_generation=%" PRIu64 " published=%s duration_ms=%.1f",
              identity.cycle_sequence, invalidation_generation,
              publication_outcome == PathPublicationOutcome::kPublished ? "true"
                                                                        : "false",
              cycle_duration_s * 1000.0);
}

[[nodiscard]] AStarConfig PlannerNode::astarConfigForCurrentVelocity(
    const std::optional<Point2> initial_tangent) const {
  AStarConfig config = astar_config_;
  config.risk_policy = {
      .critical_distance_m = inflation_radius_m_,
      .preferred_distance_m = inflation_radius_m_ + planning_clearance_m_,
  };
  if (initial_tangent.has_value()) {
    const double tangent_norm = std::hypot(initial_tangent->x, initial_tangent->y);
    if (std::isfinite(tangent_norm) && tangent_norm > 1.0e-6) {
      const double bias_speed_mps =
          std::max(config.initial_heading_bias_min_speed_mps, 1.0);
      config.initial_heading_bias_velocity_x_mps =
          initial_tangent->x * bias_speed_mps / tangent_norm;
      config.initial_heading_bias_velocity_y_mps =
          initial_tangent->y * bias_speed_mps / tangent_norm;
      return config;
    }
  }
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

[[nodiscard]] std::optional<PathComputationResult> PlannerNode::computePathOnGrid(
    const OccupancyGrid2D& grid, const ObstacleRiskField& risk_field,
    const char* source_label, const AStarConfig& astar_config,
    const Point2 planning_start, const Point2 planning_goal) {
  const auto start_cell = grid.worldToCell(planning_start);
  const auto goal_cell = grid.worldToCell(planning_goal);
  if (!start_cell.has_value() || !goal_cell.has_value()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Start or goal is outside the %s planning grid: "
                          "start=(%.2f, %.2f) goal=(%.2f, %.2f)",
                          source_label, planning_start.x, planning_start.y,
                          planning_goal.x, planning_goal.y);
    return std::nullopt;
  }
  const bool start_occupied = grid.isOccupied(*start_cell);
  const bool goal_occupied = grid.isOccupied(*goal_cell);
  if (goal_occupied || start_occupied) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Start or goal is raw occupied on %s grid: start_cell=(%d,%d) "
        "raw_occupied=%s goal_cell=(%d,%d) raw_occupied=%s",
        source_label, start_cell->x, start_cell->y, start_occupied ? "true" : "false",
        goal_cell->x, goal_cell->y, goal_occupied ? "true" : "false");
    return std::nullopt;
  }

  const auto path_compute_started_at = std::chrono::steady_clock::now();
  auto result = planner_core_.computePath(PathComputationInput{
      .grid = &grid,
      .risk_field = &risk_field,
      .current_position = planning_start,
      .goal = planning_goal,
      .astar = astar_config,
      .prohibited_clearance_field = &risk_field.occupiedClearance(),
      .prohibited_clearance_field_cache_hit = true,
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

  ++astar_successes_;
  return result;
}

} // namespace drone_city_nav
