#include "lidar_debug_node.hpp"

namespace drone_city_nav {

void LidarDebugNode::countGrid(LidarSnapshotStats& stats) const {
  if (!grid_seen_) {
    return;
  }

  const double resolution = static_cast<double>(last_grid_.info.resolution);
  if (!(resolution > 0.0)) {
    return;
  }

  const int width = static_cast<int>(last_grid_.info.width);
  const int height = static_cast<int>(last_grid_.info.height);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                         static_cast<std::size_t>(x);
      if (index >= last_grid_.data.size()) {
        continue;
      }

      const std::int8_t value = last_grid_.data[index];
      if (value < 0) {
        ++stats.grid_unknown;
        continue;
      }
      if (value >= 100) {
        ++stats.grid_occupied;
      } else if (value >= 80) {
        ++stats.grid_inflated;
      } else {
        ++stats.grid_free;
      }
    }
  }
}

[[nodiscard]] std::vector<Point2>
LidarDebugNode::collectGridPoints(const nav_msgs::msg::OccupancyGrid& grid,
                                  const std::uint8_t min_value,
                                  const std::uint8_t max_value) const {
  return collectOccupancyGridPoints(grid, min_value, max_value);
}

[[nodiscard]] std::vector<Point2> LidarDebugNode::collectProhibitedGridPoints() const {
  if (!grid_seen_) {
    return {};
  }
  return collectGridPoints(last_grid_, 80, 99);
}

[[nodiscard]] std::vector<Point2> LidarDebugNode::collectOccupiedGridPoints(
    const nav_msgs::msg::OccupancyGrid& grid) const {
  return collectGridPoints(grid, 100, 100);
}

void LidarDebugNode::publishProhibitedPointCloud() {
  publishPointCloud(collectProhibitedGridPoints(), prohibited_pointcloud_z_m_,
                    prohibited_pointcloud_pub_);
}

[[nodiscard]] std::pair<int, int>
LidarDebugNode::hitMemoryKey(const Point2 point) const {
  return {static_cast<int>(std::floor(point.x / hit_memory_resolution_m_)),
          static_cast<int>(std::floor(point.y / hit_memory_resolution_m_))};
}

[[nodiscard]] bool LidarDebugNode::rememberedHitsAllowed() const {
  if (!(min_remember_altitude_m_ > 0.0)) {
    return true;
  }
  return altitude_valid_ && std::isfinite(current_altitude_m_) &&
         current_altitude_m_ >= min_remember_altitude_m_;
}

void LidarDebugNode::rememberHitPoints(const std::vector<Point2>& hit_points) {
  if (!rememberedHitsAllowed()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Skipping remembered lidar hit updates below memory altitude: "
                         "altitude=%.2f valid=%s required=%.2f",
                         current_altitude_m_, altitude_valid_ ? "true" : "false",
                         min_remember_altitude_m_);
    return;
  }

  for (const Point2 point : hit_points) {
    if (!finite2D(point)) {
      continue;
    }
    const auto key = hitMemoryKey(point);
    if (remembered_hit_cells_.contains(key)) {
      continue;
    }
    if (remembered_hit_points_.size() >= max_remembered_hit_points_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Remembered lidar hit memory is full: points=%zu max=%zu "
                           "resolution=%.2fm",
                           remembered_hit_points_.size(), max_remembered_hit_points_,
                           hit_memory_resolution_m_);
      return;
    }
    remembered_hit_cells_.insert(key);
    remembered_hit_points_.push_back(point);
  }
}

[[nodiscard]] std::vector<Point2> LidarDebugNode::pathPoints() const {
  std::vector<Point2> path_points;
  if (!path_seen_ || last_path_.poses.empty()) {
    return path_points;
  }
  path_points.reserve(last_path_.poses.size());
  for (const auto& pose : last_path_.poses) {
    path_points.push_back(Point2{pose.pose.position.x, pose.pose.position.y});
  }
  return path_points;
}

void LidarDebugNode::writeSummary(const std::string& prefix,
                                  const std::filesystem::path& image_path,
                                  const std::filesystem::path& csv_path,
                                  const LidarSnapshotStats& stats,
                                  const bool image_ok) {
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  const double projection_attitude_yaw_delta_rad =
      yawDeltaRad(last_projected_projection_yaw_rad_, last_projected_attitude_.yaw_rad);
  LidarSnapshotRecord record;
  record.snapshot = prefix;
  record.time_s = now().seconds();
  record.position = last_projected_pose_.position;
  record.yaw_rad = last_projected_pose_.yaw_rad;
  record.altitude_m = last_projected_altitude_m_;
  record.horizontal_speed_mps = last_projected_horizontal_speed_mps_;
  record.horizontal_speed_valid = last_projected_horizontal_speed_valid_;
  record.attitude_valid = last_projected_attitude_valid_;
  record.roll_rad = last_projected_attitude_.roll_rad;
  record.pitch_rad = last_projected_attitude_.pitch_rad;
  record.attitude_yaw_rad = last_projected_attitude_.yaw_rad;
  record.tilt_rad = last_projected_attitude_tilt_rad_;
  record.yaw_source = yawSourceName();
  record.projection_yaw_rad = last_projected_projection_yaw_rad_;
  record.px4_heading_valid = last_projected_px4_heading_seen_;
  record.yaw_delta_to_attitude_rad = projection_attitude_yaw_delta_rad;
  record.scan_receive_age_s = ageSecondsOrNan(last_scan_receive_ns_, now_ns);
  record.scan_stamp_age_s = ageSecondsOrNan(last_scan_stamp_ns_, now_ns);
  record.pose_receive_age_s = ageSecondsOrNan(last_projected_pose_receive_ns_, now_ns);
  record.heading_receive_age_s =
      ageSecondsOrNan(last_projected_heading_receive_ns_, now_ns);
  record.attitude_receive_age_s =
      ageSecondsOrNan(last_projected_attitude_receive_ns_, now_ns);
  record.motion_compensation_enabled = motion_compensate_lidar_pose_;
  record.pose_lag_s = last_projected_pose_lag_s_;
  record.pose_latency_s = last_projected_pose_latency_s_;
  record.motion_time_offset_s = last_projected_motion_time_offset_s_;
  record.motion_shift = last_projected_motion_shift_;
  record.motion_shift_m = last_projected_motion_shift_m_;
  record.scan_duration_s = last_projected_scan_duration_s_;
  record.scan_time_increment_s = last_projected_scan_time_increment_s_;
  record.scan_beams = last_scan_.ranges.size();
  record.scan_range_min_m = last_scan_.range_min;
  record.scan_range_max_m = last_scan_.range_max;
  record.scan_angle_min_rad = last_scan_.angle_min;
  record.scan_angle_max_rad = last_scan_.angle_max;
  record.compensate_attitude = compensate_lidar_attitude_;
  record.use_px4_heading_for_scan = use_px4_heading_for_scan_;
  record.initial_heading_rad = initial_heading_rad_;
  record.scan_yaw_offset_rad = scan_yaw_offset_rad_;
  record.lidar_mount_roll_rad = lidar_mount_roll_rad_;
  record.lidar_mount_pitch_rad = lidar_mount_pitch_rad_;
  record.lidar_mount_yaw_rad = lidar_mount_yaw_rad_;
  record.min_projected_altitude_m = min_projected_lidar_altitude_m_;
  record.max_projected_altitude_m = max_projected_lidar_altitude_m_;
  record.grid_seen = grid_seen_;
  record.path_seen = path_seen_;
  record.path_waypoints = path_seen_ ? last_path_.poses.size() : 0U;
  record.remembered_hits = remembered_hit_points_.size();
  record.image_ok = image_ok;
  record.image_path = image_path;
  record.scan_csv_path = csv_path;
  record.max_logged_hit_points = max_logged_hit_points_;
  record.stats = stats;
  writeLidarSnapshotSummary(summary_stream_, record);
}

void LidarDebugNode::publishPointCloud(
    const std::vector<Point2>& points, const double z_m,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) {
  if (!publisher) {
    return;
  }
  sensor_msgs::msg::PointCloud2 cloud =
      buildLidarDebugPointCloud(points, z_m, now(), "map");
  publisher->publish(cloud);
}

void LidarDebugNode::publishRawLidarPointCloud(const std::vector<Point3>& points) {
  if (!raw_lidar_3d_pointcloud_pub_) {
    return;
  }
  raw_lidar_3d_pointcloud_pub_->publish(
      buildLidarDebugPointCloud(points, last_scan_.header.stamp, "map"));
}

} // namespace drone_city_nav
