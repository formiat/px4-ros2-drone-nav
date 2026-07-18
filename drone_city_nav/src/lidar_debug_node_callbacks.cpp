#include "lidar_debug_node.hpp"

namespace drone_city_nav {

void LidarDebugNode::onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg) {
  last_pose_receive_ns_ = get_clock()->now().nanoseconds();
  if (!msg.xy_valid || !std::isfinite(msg.x) || !std::isfinite(msg.y)) {
    return;
  }

  current_pose_.position = Point2{static_cast<double>(msg.x) + px4_local_origin_.x,
                                  static_cast<double>(msg.y) + px4_local_origin_.y};
  const bool heading_valid = msg.heading_good_for_control && std::isfinite(msg.heading);
  if (heading_valid) {
    current_pose_.yaw_rad = static_cast<double>(msg.heading);
    px4_heading_seen_ = true;
    last_heading_receive_ns_ = last_pose_receive_ns_;
  }
  if (msg.z_valid && std::isfinite(msg.z)) {
    current_altitude_m_ = -static_cast<double>(msg.z);
    altitude_valid_ = true;
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
}

void LidarDebugNode::onAttitude(const px4_msgs::msg::VehicleAttitude& msg) {
  last_attitude_receive_ns_ = get_clock()->now().nanoseconds();
  const auto euler = quaternionToEuler(msg.q);
  if (!euler.has_value()) {
    attitude_valid_ = false;
    return;
  }

  attitude_ = *euler;
  attitude_tilt_rad_ = std::hypot(attitude_.roll_rad, attitude_.pitch_rad);
  attitude_valid_ = true;
}

void LidarDebugNode::onScan(const sensor_msgs::msg::LaserScan& msg) {
  last_scan_ = msg;
  scan_seen_ = true;
  last_scan_receive_ns_ = get_clock()->now().nanoseconds();
  last_scan_stamp_ns_ = toNanoseconds(msg.header.stamp);

  if (!pose_seen_) {
    publishRawLidarPointCloud({});
    return;
  }

  last_projected_pose_ = current_pose_;
  last_projected_altitude_m_ = current_altitude_m_;
  last_projected_altitude_valid_ = altitude_valid_;
  last_projected_velocity_ = current_velocity_;
  last_projected_horizontal_speed_mps_ = horizontal_speed_mps_;
  last_projected_attitude_ = attitude_;
  last_projected_attitude_tilt_rad_ = attitude_tilt_rad_;
  last_projected_projection_yaw_rad_ = projectionYawRad();
  last_projected_horizontal_speed_valid_ = horizontal_speed_valid_;
  last_projected_attitude_valid_ = attitude_valid_;
  last_projected_px4_heading_seen_ = px4_heading_seen_;
  last_projected_pose_receive_ns_ = last_pose_receive_ns_;
  last_projected_heading_receive_ns_ = last_heading_receive_ns_;
  last_projected_attitude_receive_ns_ = last_attitude_receive_ns_;
  last_projected_scan_duration_s_ = scanDurationSeconds(msg);
  last_projected_scan_time_increment_s_ = scanTimeIncrementSeconds(msg);
  const LidarPoseMotionCompensationResult motion_compensation =
      compensateLidarPoseForLatency(
          current_pose_.position, current_velocity_, motion_compensate_lidar_pose_,
          horizontal_speed_valid_, poseReceiveLagSeconds(), lidar_pose_latency_s_);
  last_projected_pose_.position = motion_compensation.position;
  last_projected_pose_lag_s_ = motion_compensation.pose_lag_s;
  last_projected_pose_latency_s_ = motion_compensation.latency_s;
  last_projected_motion_time_offset_s_ = motion_compensation.signed_time_offset_s;
  last_projected_motion_shift_ = motion_compensation.applied_shift;
  last_projected_motion_shift_m_ = motion_compensation.applied_shift_m;

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
}

} // namespace drone_city_nav
