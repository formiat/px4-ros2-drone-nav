#include "lidar_debug_node.hpp"

namespace drone_city_nav {

void LidarDebugNode::writeSnapshot() {
  if (max_snapshots_ > 0U && snapshot_index_ >= max_snapshots_) {
    return;
  }
  if (!scan_seen_ || !pose_seen_ || !last_scan_projection_seen_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Lidar debug waiting for scan projection: scan=%s pose=%s "
                         "projection=%s",
                         scan_seen_ ? "true" : "false", pose_seen_ ? "true" : "false",
                         last_scan_projection_seen_ ? "true" : "false");
    return;
  }

  ++snapshot_index_;
  const std::string prefix = lidarSnapshotPrefix(snapshot_index_);
  const std::filesystem::path image_path =
      std::filesystem::path{output_dir_} / (prefix + ".ppm");
  const std::filesystem::path csv_path =
      std::filesystem::path{output_dir_} / (prefix + "_scan.csv");

  LidarSnapshotStats stats = last_scan_stats_;
  const bool csv_ok = writeLidarScanCsv(csv_path, last_scan_rows_);
  if (!csv_ok) {
    RCLCPP_WARN(get_logger(), "Failed to write lidar debug CSV: '%s'",
                csv_path.string().c_str());
  }
  countGrid(stats);

  const std::vector<Point2> path_points = pathPoints();
  const LidarDebugRenderConfig render_config{image_size_px_, view_radius_m_};
  const std::optional<GridImageView> grid_view = gridImageView();
  const LidarDebugFrame frame{last_projected_pose_.position,
                              headingDirection(last_projected_projection_yaw_rad_),
                              grid_view,
                              path_points,
                              last_scan_hit_points_,
                              remembered_hit_points_};
  const DebugImage image = renderLidarDebugImage(render_config, frame);
  const bool image_ok = writePpm(image_path, image);
  publishPointCloud(last_scan_hit_points_, current_pointcloud_z_m_, pointcloud_pub_);
  publishPointCloud(remembered_hit_points_, remembered_pointcloud_z_m_,
                    remembered_pointcloud_pub_);
  publishProhibitedPointCloud();

  writeSummary(prefix, image_path, csv_path, stats, image_ok);
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  const double projection_attitude_yaw_delta_rad =
      yawDeltaRad(last_projected_projection_yaw_rad_, last_projected_attitude_.yaw_rad);
  RCLCPP_INFO(
      get_logger(),
      "LIDAR_DEBUG snapshot=%s pose=(%.2f, %.2f) altitude=%.2f "
      "speed=%.2f speed_valid=%s yaw_source=%s projection_yaw=%.3f "
      "px4_heading_seen=%s attitude_valid=%s attitude_yaw=%.3f "
      "yaw_delta=%.3f roll=%.3f pitch=%.3f tilt=%.3f "
      "scan_age=%.3f pose_age=%.3f heading_age=%.3f attitude_age=%.3f "
      "pose_lag=%.3f pose_latency=%.3f motion_time_offset=%.3f "
      "motion_shift=(%.2f, %.2f) motion_shift_m=%.2f "
      "scan_duration=%.3f scan_time_increment=%.6f "
      "beams=%zu hits=%zu altitude_rejected=%zu "
      "projection_rejected=%zu remembered_hits=%zu grid=%s "
      "path_waypoints=%zu image='%s' csv='%s'",
      prefix.c_str(), last_projected_pose_.position.x, last_projected_pose_.position.y,
      last_projected_altitude_m_, last_projected_horizontal_speed_mps_,
      last_projected_horizontal_speed_valid_ ? "true" : "false", yawSourceName(),
      last_projected_projection_yaw_rad_,
      last_projected_px4_heading_seen_ ? "true" : "false",
      last_projected_attitude_valid_ ? "true" : "false",
      last_projected_attitude_.yaw_rad, projection_attitude_yaw_delta_rad,
      last_projected_attitude_.roll_rad, last_projected_attitude_.pitch_rad,
      last_projected_attitude_tilt_rad_, ageSecondsOrNan(last_scan_receive_ns_, now_ns),
      ageSecondsOrNan(last_projected_pose_receive_ns_, now_ns),
      ageSecondsOrNan(last_projected_heading_receive_ns_, now_ns),
      ageSecondsOrNan(last_projected_attitude_receive_ns_, now_ns),
      last_projected_pose_lag_s_, last_projected_pose_latency_s_,
      last_projected_motion_time_offset_s_, last_projected_motion_shift_.x,
      last_projected_motion_shift_.y, last_projected_motion_shift_m_,
      last_projected_scan_duration_s_, last_projected_scan_time_increment_s_,
      stats.processed_beams, stats.hit_beams, stats.altitude_rejected_beams,
      stats.projection_rejected_beams, remembered_hit_points_.size(),
      grid_seen_ ? "true" : "false", path_seen_ ? last_path_.poses.size() : 0U,
      image_path.string().c_str(), csv_path.string().c_str());
}

[[nodiscard]] double LidarDebugNode::scanRangeMax() const {
  return std::min(static_cast<double>(last_scan_.range_max), max_lidar_range_m_);
}

[[nodiscard]] const char* LidarDebugNode::yawSourceName() const noexcept {
  return use_px4_heading_for_scan_ ? "px4_heading" : "initial_heading";
}

[[nodiscard]] double LidarDebugNode::projectionYawRad() const noexcept {
  if (use_px4_heading_for_scan_ && px4_heading_seen_) {
    return current_pose_.yaw_rad;
  }
  return initial_heading_rad_;
}

[[nodiscard]] double LidarDebugNode::scanTimeIncrementSeconds(
    const sensor_msgs::msg::LaserScan& scan) const noexcept {
  return lidarScanTimeIncrementSeconds(static_cast<double>(scan.scan_time),
                                       static_cast<double>(scan.time_increment),
                                       scan.ranges.size());
}

[[nodiscard]] double LidarDebugNode::scanDurationSeconds(
    const sensor_msgs::msg::LaserScan& scan) const noexcept {
  return lidarScanDurationSeconds(static_cast<double>(scan.scan_time),
                                  static_cast<double>(scan.time_increment),
                                  scan.ranges.size(), lidar_scan_duration_override_s_);
}

[[nodiscard]] double LidarDebugNode::poseReceiveLagSeconds() const noexcept {
  if (last_scan_receive_ns_ <= 0 || last_pose_receive_ns_ <= 0 ||
      last_scan_receive_ns_ <= last_pose_receive_ns_) {
    return 0.0;
  }
  return static_cast<double>(last_scan_receive_ns_ - last_pose_receive_ns_) /
         static_cast<double>(kNanosecondsPerSecond);
}

[[nodiscard]] LidarProjectionPose LidarDebugNode::lidarProjectionPose() const {
  return LidarProjectionPose{
      last_projected_pose_.position,      last_projected_altitude_m_,
      last_projected_projection_yaw_rad_, last_projected_attitude_.roll_rad,
      last_projected_attitude_.pitch_rad, last_projected_altitude_valid_,
      last_projected_attitude_valid_};
}

[[nodiscard]] LidarProjectionConfig LidarDebugNode::lidarProjectionConfig() const {
  return LidarProjectionConfig{max_lidar_range_m_,
                               range_hit_epsilon_m_,
                               scan_yaw_offset_rad_,
                               lidar_z_offset_m_,
                               min_projected_lidar_altitude_m_,
                               max_projected_lidar_altitude_m_,
                               compensate_lidar_attitude_,
                               lidar_mount_roll_rad_,
                               lidar_mount_pitch_rad_,
                               lidar_mount_yaw_rad_};
}

[[nodiscard]] LidarBeamProjection
LidarDebugNode::projectScanBeam(const std::size_t beam_index,
                                const float raw_range) const {
  return projectLidarBeam(lidarProjectionPose(), lidarProjectionConfig(),
                          static_cast<double>(last_scan_.range_min), scanRangeMax(),
                          static_cast<double>(last_scan_.angle_min),
                          static_cast<double>(last_scan_.angle_increment), beam_index,
                          raw_range);
}

[[nodiscard]] Point2
LidarDebugNode::headingDirection(const double projection_yaw_rad) const {
  const double yaw = projection_yaw_rad + scan_yaw_offset_rad_;
  return Point2{std::cos(yaw), std::sin(yaw)};
}

[[nodiscard]] std::vector<LidarSnapshotCsvRow>
LidarDebugNode::collectScanRows(LidarSnapshotStats& stats) const {
  const double scan_range_max = scanRangeMax();
  const LidarDebugSnapshotOutput output = buildLidarDebugSnapshotOutput(
      LidarDebugSnapshotInput{
          scan_seen_, pose_seen_, last_scan_projection_seen_,
          std::span<const float>{last_scan_.ranges.data(), last_scan_.ranges.size()},
          static_cast<double>(last_scan_.range_min), scan_range_max,
          static_cast<double>(last_scan_.angle_min),
          static_cast<double>(last_scan_.angle_increment), beam_csv_stride_,
          remembered_hit_points_.size()},
      [](const std::size_t beam_index, const float raw_range,
         const void* context) -> LidarBeamProjection {
        return static_cast<const LidarDebugNode*>(context)->projectScanBeam(beam_index,
                                                                            raw_range);
      },
      this);
  stats = output.stats;
  return output.rows;
}

[[nodiscard]] std::optional<GridImageView> LidarDebugNode::gridImageView() const {
  if (grid_seen_ && last_grid_.info.resolution > 0.0F && last_grid_.info.width > 0U &&
      last_grid_.info.height > 0U) {
    return GridImageView{
        static_cast<int>(last_grid_.info.width),
        static_cast<int>(last_grid_.info.height),
        static_cast<double>(last_grid_.info.resolution),
        last_grid_.info.origin.position.x,
        last_grid_.info.origin.position.y,
        std::span<const std::int8_t>{last_grid_.data.data(), last_grid_.data.size()}};
  }
  return std::nullopt;
}

} // namespace drone_city_nav
