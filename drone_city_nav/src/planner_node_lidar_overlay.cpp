#include <algorithm>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include "planner_node.hpp"

namespace drone_city_nav {
[[nodiscard]] double PlannerNode::currentLidarRangeMax() const {
  return std::min(static_cast<double>(last_scan_.range_max), max_lidar_range_m_);
}

[[nodiscard]] double PlannerNode::currentLidarPoseReceiveLagSeconds(
    const std::int64_t scan_receive_ns, const std::int64_t pose_receive_ns) const {
  if (scan_receive_ns > 0 && pose_receive_ns > 0 && scan_receive_ns > pose_receive_ns) {
    return static_cast<double>(scan_receive_ns - pose_receive_ns) / 1.0e9;
  }
  return 0.0;
}

[[nodiscard]] LidarProjectionPose PlannerNode::currentLidarProjectionPose() const {
  return LidarProjectionPose{current_pose_.position,
                             current_altitude_m_,
                             use_px4_heading_for_scan_ ? current_pose_.yaw_rad
                                                       : initial_heading_rad_,
                             current_attitude_.roll_rad,
                             current_attitude_.pitch_rad,
                             altitude_valid_,
                             attitude_valid_};
}

[[nodiscard]] LidarProjectionConfig PlannerNode::currentLidarProjectionConfig() const {
  return LidarProjectionConfig{
      .max_lidar_range_m = max_lidar_range_m_,
      .range_hit_epsilon_m = range_hit_epsilon_m_,
      .scan_yaw_offset_rad = scan_yaw_offset_rad_,
      .lidar_z_offset_m = lidar_z_offset_m_,
      .min_projected_altitude_m = min_projected_lidar_altitude_m_,
      .max_projected_altitude_m = max_projected_lidar_altitude_m_,
      .compensate_attitude = compensate_lidar_attitude_,
      .lidar_mount_roll_rad = lidar_mount_roll_rad_,
      .lidar_mount_pitch_rad = lidar_mount_pitch_rad_,
      .lidar_mount_yaw_rad = lidar_mount_yaw_rad_,
      .use_full_lidar_extrinsic = use_full_lidar_extrinsic_,
      .lidar_translation_body_frd_m = lidar_translation_body_frd_m_,
      .lidar_flu_to_body_frd_quaternion = lidar_flu_to_body_frd_quaternion_,
  };
}

CurrentLidarOverlayStats
PlannerNode::overlayCurrentLidarHits(OccupancyGrid2D& grid,
                                     const std::int64_t now_ns) const {
  CurrentLidarOverlayStats stats{};
  stats.enabled = true;

  stats.fresh =
      timestampIsFresh(last_scan_update_ns_, now_ns, max_current_lidar_staleness_ns_);
  if (!scan_seen_ || !stats.fresh) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planner current lidar overlay is waiting for a fresh scan: seen=%s "
        "fresh=%s age_s=%.2f",
        scan_seen_ ? "true" : "false", stats.fresh ? "true" : "false",
        scanAgeSeconds(now_ns));
    return stats;
  }
  if (!last_scan_projection_pose_valid_) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planner current lidar overlay is waiting for a valid scan projection pose");
    return stats;
  }

  const double scan_range_max = currentLidarRangeMax();
  if (!(scan_range_max > 0.0) || last_scan_.angle_increment == 0.0F) {
    return stats;
  }
  const std::uint64_t scan_stamp_ns = stampNanoseconds(last_scan_.header.stamp);
  const bool scan_stamp_valid =
      scan_stamp_ns > 0U &&
      scan_stamp_ns <=
          static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());

  const LidarScanView scan_view{
      .ranges =
          std::span<const float>{last_scan_.ranges.data(), last_scan_.ranges.size()},
      .range_min_m = static_cast<double>(last_scan_.range_min),
      .range_max_m = scan_range_max,
      .angle_min_rad = static_cast<double>(last_scan_.angle_min),
      .angle_increment_rad = static_cast<double>(last_scan_.angle_increment),
      .timing =
          LaserScanTiming{
              .first_beam_stamp_ns =
                  scan_stamp_valid ? static_cast<std::int64_t>(scan_stamp_ns) : 0,
              .first_beam_stamp_valid = scan_stamp_valid,
              .time_increment_s = static_cast<double>(last_scan_.time_increment),
              .receive_stamp_ns = last_scan_update_ns_,
              .receive_stamp_valid = last_scan_update_ns_ > 0,
          },
      .beam_projection_poses = last_scan_projection_poses_,
      .projection_pose_source = last_scan_projection_pose_source_,
  };
  const CurrentLidarOverlayStats overlay_stats =
      drone_city_nav::overlayCurrentLidarHits(
          grid, scan_view, last_scan_projection_pose_, currentLidarProjectionConfig(),
          known_static_lidar_classifier_.has_value() ? &*known_static_lidar_classifier_
                                                     : nullptr,
          &ground_lidar_rejection_config_, &current_lidar_ambiguous_hit_tracker_);
  stats.used = overlay_stats.used;
  stats.processed_beams = overlay_stats.processed_beams;
  stats.hit_beams = overlay_stats.hit_beams;
  stats.altitude_rejected_beams = overlay_stats.altitude_rejected_beams;
  stats.occupied_cells = overlay_stats.occupied_cells;
  stats.outside_hits = overlay_stats.outside_hits;
  stats.timestamp_aligned_beams = overlay_stats.timestamp_aligned_beams;
  stats.ambiguous_hits_pending_confirmation =
      overlay_stats.ambiguous_hits_pending_confirmation;
  stats.ambiguous_hits_confirmed = overlay_stats.ambiguous_hits_confirmed;
  stats.overlay_occupied_cells_applied = overlay_stats.overlay_occupied_cells_applied;
  stats.overlay_occupied_cells_preserved =
      overlay_stats.overlay_occupied_cells_preserved;
  stats.known_static_lidar = overlay_stats.known_static_lidar;
  stats.ingestion_decisions = overlay_stats.ingestion_decisions;
  stats.accepted_hits = overlay_stats.accepted_hits;
  stats.retained_known_static_hits = overlay_stats.retained_known_static_hits;
  return stats;
}

} // namespace drone_city_nav
