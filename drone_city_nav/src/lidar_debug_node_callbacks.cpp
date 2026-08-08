#include "lidar_debug_node.hpp"

namespace drone_city_nav {

void LidarDebugNode::onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg) {
  last_pose_receive_ns_ = get_clock()->now().nanoseconds();
  if (!msg.xy_valid || !std::isfinite(msg.x) || !std::isfinite(msg.y)) {
    return;
  }

  current_pose_.position = Point2{static_cast<double>(msg.x) + px4_local_origin_.x,
                                  static_cast<double>(msg.y) + px4_local_origin_.y};
  const bool heading_valid = px4HeadingReadyForMapping(
      msg.heading_good_for_control, static_cast<double>(msg.heading),
      static_cast<double>(msg.heading_var), maximum_heading_variance_rad2_);
  const MappingYawSelection mapping_yaw =
      mapping_yaw_tracker_.update(heading_valid, static_cast<double>(msg.heading));
  const bool starts_new_px4_generation =
      use_px4_heading_for_scan_ &&
      mapping_yaw.source == MappingYawSource::kPx4Heading && !px4_heading_seen_;
  if (starts_new_px4_generation) {
    lidar_pose_history_.startNewGeneration();
    pending_lidar_scans_.clear();
  }
  px4_heading_seen_ = mapping_yaw.source == MappingYawSource::kPx4Heading;
  if (mapping_yaw.valid) {
    current_pose_.yaw_rad = mapping_yaw.yaw_rad;
    last_heading_receive_ns_ = last_pose_receive_ns_;
  }
  if (msg.z_valid && std::isfinite(msg.z)) {
    current_altitude_m_ = -static_cast<double>(msg.z);
    altitude_valid_ = true;
  }
  const LidarPoseSourceStampResult source_stamp = resolveLidarPoseSourceStamp(
      px4_ros_time_mapper_, msg.timestamp_sample, last_pose_receive_ns_,
      lidar_pose_source_stamp_config_);
  if (source_stamp.resolved()) {
    lidar_pose_history_.addPosition(
        last_pose_receive_ns_,
        Point3{current_pose_.position.x, current_pose_.position.y, current_altitude_m_},
        use_px4_heading_for_scan_ ? current_pose_.yaw_rad : initial_heading_rad_,
        altitude_valid_ && mapping_yaw.valid, source_stamp.acquisition_stamp_ns,
        lidarPoseSourceTimestampNanoseconds(msg.timestamp_sample));
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "LIDAR_DEBUG_POSE_HISTORY position_rejected=true reason=%s",
                         lidarPoseSourceStampStatusName(source_stamp.status));
  }
  if (msg.v_xy_valid && std::isfinite(msg.vx) && std::isfinite(msg.vy)) {
    current_velocity_ =
        Point2{static_cast<double>(msg.vx), static_cast<double>(msg.vy)};
    horizontal_speed_mps_ =
        std::hypot(static_cast<double>(msg.vx), static_cast<double>(msg.vy));
    horizontal_speed_valid_ = true;
  } else {
    current_velocity_ = Point2{};
    horizontal_speed_valid_ = false;
  }
  pose_seen_ = true;
  processPendingLidarScans();
}

void LidarDebugNode::onAttitude(const px4_msgs::msg::VehicleAttitude& msg) {
  last_attitude_receive_ns_ = get_clock()->now().nanoseconds();
  const LidarPoseSourceStampResult source_stamp = resolveLidarPoseSourceStamp(
      px4_ros_time_mapper_, msg.timestamp_sample, last_attitude_receive_ns_,
      lidar_pose_source_stamp_config_);
  if (source_stamp.resolved()) {
    lidar_pose_history_.addAttitude(
        last_attitude_receive_ns_, msg.q, source_stamp.acquisition_stamp_ns,
        lidarPoseSourceTimestampNanoseconds(msg.timestamp_sample));
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "LIDAR_DEBUG_POSE_HISTORY attitude_rejected=true reason=%s",
                         lidarPoseSourceStampStatusName(source_stamp.status));
  }
  const auto euler = quaternionToEuler(msg.q);
  if (!euler.has_value()) {
    attitude_valid_ = false;
    return;
  }

  attitude_ = *euler;
  attitude_tilt_rad_ = std::hypot(attitude_.roll_rad, attitude_.pitch_rad);
  attitude_valid_ = true;
  processPendingLidarScans();
}

void LidarDebugNode::onTimesyncStatus(const px4_msgs::msg::TimesyncStatus& msg) {
  px4_ros_time_mapper_.observeTimesync(msg.timestamp, msg.estimated_offset,
                                       msg.round_trip_time,
                                       get_clock()->now().nanoseconds());
  processPendingLidarScans();
}

void LidarDebugNode::onScan(const sensor_msgs::msg::LaserScan& msg) {
  if (!diagnosticsSelected()) {
    return;
  }

  if (pending_lidar_scans_.size() >= lidar_scan_alignment_queue_capacity_) {
    pending_lidar_scans_.pop_front();
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "LIDAR_DEBUG_SCAN_ALIGNMENT dropped=true reason=queue_capacity capacity=%zu",
        lidar_scan_alignment_queue_capacity_);
  }
  pending_lidar_scans_.push_back(
      PendingLidarScan{msg, get_clock()->now().nanoseconds()});
  processPendingLidarScans();
}

void LidarDebugNode::processPendingLidarScans() {
  while (!pending_lidar_scans_.empty()) {
    const PendingLidarScanDisposition disposition =
        processPendingLidarScan(pending_lidar_scans_.front());
    if (disposition == PendingLidarScanDisposition::kWaitForPoseBracket) {
      return;
    }
    pending_lidar_scans_.pop_front();
  }
}

LidarDebugNode::PendingLidarScanDisposition
LidarDebugNode::processPendingLidarScan(const PendingLidarScan& pending) {
  const sensor_msgs::msg::LaserScan& msg = pending.scan;
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  const bool wait_expired =
      pending.receive_stamp_ns <= 0 ||
      now_ns - pending.receive_stamp_ns >= lidar_scan_alignment_maximum_wait_ns_;

  last_scan_ = msg;
  scan_seen_ = true;
  last_scan_receive_ns_ = pending.receive_stamp_ns;
  last_scan_stamp_ns_ = toNanoseconds(msg.header.stamp);

  if (!pose_seen_) {
    return wait_expired ? PendingLidarScanDisposition::kConsumed
                        : PendingLidarScanDisposition::kWaitForPoseBracket;
  }

  if (!lidarDebugProjectionHeadingReady(use_px4_heading_for_scan_, px4_heading_seen_)) {
    if (!wait_expired) {
      return PendingLidarScanDisposition::kWaitForPoseBracket;
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "LIDAR_DEBUG_SCAN_ALIGNMENT dropped=true reason=px4_heading_not_stable "
        "scan_beams=%zu maximum_heading_variance_rad2=%.6f",
        msg.ranges.size(), maximum_heading_variance_rad2_);
    return PendingLidarScanDisposition::kConsumed;
  }

  last_projected_scan_duration_s_ = scanDurationSeconds(msg);
  last_projected_scan_time_increment_s_ = scanTimeIncrementSeconds(msg);
  const LaserScanTiming timing{
      .first_beam_stamp_ns = last_scan_stamp_ns_,
      .first_beam_stamp_valid = last_scan_stamp_ns_ > 0,
      .time_increment_s = last_projected_scan_time_increment_s_,
      .receive_stamp_ns = last_scan_receive_ns_,
      .receive_stamp_valid = last_scan_receive_ns_ > 0,
  };
  const LidarAcquisitionPoseResult acquisition_pose = resolveLidarAcquisitionBeamPoses(
      lidar_pose_history_, timing, msg.ranges.size(), lidar_acquisition_pose_config_,
      use_px4_heading_for_scan_ ? std::nullopt
                                : std::optional<double>{initial_heading_rad_},
      &px4_ros_time_mapper_);
  const std::string alignment_diagnostic = formatLidarAcquisitionPoseDiagnostic(
      "Lidar debug acquisition pose", acquisition_pose, timing, now_ns);
  if (!acquisition_pose.resolved()) {
    const bool permanent_failure =
        acquisition_pose.status ==
            LidarAcquisitionPoseStatus::kInvalidSensorTimeOffset ||
        acquisition_pose.status == LidarAcquisitionPoseStatus::kInvalidScanTimestamp;
    if (!permanent_failure && !wait_expired) {
      return PendingLidarScanDisposition::kWaitForPoseBracket;
    }
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "LIDAR_DEBUG_SCAN_ALIGNMENT dropped=true queue_wait_ms=%.3f %s",
        1.0e-6 * static_cast<double>(now_ns - pending.receive_stamp_ns),
        alignment_diagnostic.c_str());
    return PendingLidarScanDisposition::kConsumed;
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                       "LIDAR_DEBUG_SCAN_ALIGNMENT queue_wait_ms=%.3f %s",
                       1.0e-6 * static_cast<double>(now_ns - pending.receive_stamp_ns),
                       alignment_diagnostic.c_str());

  last_pose_alignment_ = acquisition_pose.alignment;
  last_projected_beam_poses_ = acquisition_pose.alignment.poses;
  const LidarProjectionPose& first_beam_pose = last_projected_beam_poses_.front();
  last_projected_pose_ = Pose2{first_beam_pose.position, first_beam_pose.yaw_rad};
  last_projected_altitude_m_ = first_beam_pose.altitude_m;
  last_projected_altitude_valid_ = first_beam_pose.altitude_valid;
  last_projected_velocity_ = current_velocity_;
  last_projected_horizontal_speed_mps_ = horizontal_speed_mps_;
  last_projected_attitude_ = AttitudeEuler{
      first_beam_pose.roll_rad, first_beam_pose.pitch_rad, first_beam_pose.yaw_rad};
  last_projected_attitude_tilt_rad_ =
      std::hypot(first_beam_pose.roll_rad, first_beam_pose.pitch_rad);
  last_projected_projection_yaw_rad_ = first_beam_pose.yaw_rad;
  last_projected_horizontal_speed_valid_ = horizontal_speed_valid_;
  last_projected_attitude_valid_ = first_beam_pose.attitude_valid;
  last_projected_px4_heading_seen_ = px4_heading_seen_;
  last_projected_pose_receive_ns_ = last_pose_receive_ns_;
  last_projected_heading_receive_ns_ = last_heading_receive_ns_;
  last_projected_attitude_receive_ns_ = last_attitude_receive_ns_;
  last_projected_pose_lag_s_ = poseReceiveLagSeconds();
  last_projected_pose_latency_s_ = lidar_acquisition_pose_config_.sensor_time_offset_s;
  last_projected_motion_time_offset_s_ =
      static_cast<double>(acquisition_pose.sensor_time_offset_ns) * 1.0e-9;
  last_projected_motion_shift_ =
      Point2{first_beam_pose.position.x - current_pose_.position.x,
             first_beam_pose.position.y - current_pose_.position.y};
  last_projected_motion_shift_m_ =
      std::hypot(last_projected_motion_shift_.x, last_projected_motion_shift_.y);

  LidarSnapshotStats stats{};
  last_scan_rows_ = collectScanRows(stats);
  last_scan_stats_ = stats;
  last_scan_hit_points_ = stats.hit_points;
  last_scan_projection_seen_ = true;
  publishRawLidarPointCloud(collectRawLidarHitPoints3D());

  rememberHitPoints(last_scan_hit_points_);
  publishPointCloud(last_scan_hit_points_, current_pointcloud_z_m_, pointcloud_pub_);
  publishPointCloud(remembered_hit_points_, remembered_pointcloud_z_m_,
                    remembered_pointcloud_pub_);
  return PendingLidarScanDisposition::kConsumed;
}

} // namespace drone_city_nav
