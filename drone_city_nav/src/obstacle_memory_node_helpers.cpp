#include "obstacle_memory_node_helpers.hpp"

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <exception>
#include <limits>
#include <numbers>
#include <utility>

namespace drone_city_nav {

std::optional<std::int64_t>
validRosStampNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  constexpr std::int64_t kNanosecondsPerSecond{1'000'000'000};
  if (stamp.sec < 0 ||
      stamp.nanosec >= static_cast<std::uint32_t>(kNanosecondsPerSecond) ||
      (stamp.sec == 0 && stamp.nanosec == 0U)) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<std::int64_t>(stamp.nanosec);
}

double navigationPoseAgeSeconds(const std::int64_t last_pose_update_ns,
                                const std::int64_t now_ns) noexcept {
  if (last_pose_update_ns <= 0 || now_ns <= last_pose_update_ns) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(now_ns - last_pose_update_ns) / 1.0e9;
}

double navigationPoseReceiveLagSeconds(const std::int64_t last_pose_update_ns,
                                       const std::int64_t scan_receive_ns) noexcept {
  if (scan_receive_ns > 0 && last_pose_update_ns > 0 &&
      scan_receive_ns > last_pose_update_ns) {
    return static_cast<double>(scan_receive_ns - last_pose_update_ns) / 1.0e9;
  }
  return 0.0;
}

LidarPoseSampleResult samplePoseAtRosAcquisition(const LidarPoseHistory& pose_history,
                                                 const Px4RosTimeMapper& time_mapper,
                                                 const std::int64_t ros_stamp_ns,
                                                 const bool stamp_valid) noexcept {
  if (!stamp_valid) {
    return {};
  }
  const auto px4_stamp_ns = time_mapper.rosToPx4LocalTimeNs(ros_stamp_ns);
  if (px4_stamp_ns.has_value()) {
    LidarPoseSampleResult source_result = pose_history.sampleWithDiagnostics(
        *px4_stamp_ns, LidarPoseTimeBasis::kPx4AcquisitionTime);
    if (source_result.aligned_pose.has_value()) {
      const auto map_timing = [&time_mapper](LidarPoseTemporalAlignment& timing) {
        timing.from_acquisition_ros_stamp_ns =
            time_mapper.px4LocalToRosTimeNs(timing.from_acquisition_stamp_ns)
                .value_or(0);
        timing.to_acquisition_ros_stamp_ns =
            time_mapper.px4LocalToRosTimeNs(timing.to_acquisition_stamp_ns).value_or(0);
      };
      map_timing(source_result.position_timing);
      map_timing(source_result.attitude_timing);
      return source_result;
    }
  }
  return pose_history.sampleWithDiagnostics(ros_stamp_ns,
                                            LidarPoseTimeBasis::kReceiveTime);
}

void logFirstNavigationPose(rclcpp::Node& node, bool& pose_seen,
                            const NavigationPose2D& pose, const char* source_name) {
  if (pose_seen) {
    return;
  }
  pose_seen = true;
  RCLCPP_INFO(node.get_logger(),
              "First valid navigation pose: source=%s x=%.2f y=%.2f altitude=%.2f "
              "altitude_valid=%s yaw=%.2f",
              source_name, pose.pose.position.x, pose.pose.position.y, pose.altitude_m,
              pose.altitude_valid ? "true" : "false", pose.pose.yaw_rad);
}

void invalidateObstacleNavigationPose(NavigationPose2D& pose,
                                      std::int64_t& last_pose_update_ns,
                                      Point2& velocity, bool& velocity_valid) noexcept {
  invalidateNavigationPose(pose);
  last_pose_update_ns = 0;
  velocity = Point2{};
  velocity_valid = false;
}

std::int8_t rawOccupancyValue(const OccupancyGrid2D& grid, const GridIndex cell) {
  if (grid.isOccupied(cell)) {
    return static_cast<std::int8_t>(100);
  }
  if (grid.state(cell) == CellState::kFree) {
    return static_cast<std::int8_t>(0);
  }
  return static_cast<std::int8_t>(-1);
}

nav_msgs::msg::OccupancyGrid
makeObstacleMemoryOccupancyGridMessage(const OccupancyGrid2D& grid,
                                       const rclcpp::Time& stamp,
                                       const std::string& frame_id) {
  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.info.map_load_time = stamp;
  message.info.resolution = static_cast<float>(grid.resolution());
  message.info.width = static_cast<std::uint32_t>(grid.width());
  message.info.height = static_cast<std::uint32_t>(grid.height());
  message.info.origin.position.x = grid.originX();
  message.info.origin.position.y = grid.originY();
  message.info.origin.orientation.w = 1.0;
  message.data.assign(grid.cellCount(), static_cast<std::int8_t>(-1));
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const GridIndex cell{x, y};
      message.data[grid.linearIndex(cell)] = rawOccupancyValue(grid, cell);
    }
  }
  return message;
}

AmbiguousLidarHitTrackerConfig
declareAmbiguousLidarHitTrackerConfig(rclcpp::Node& node) {
  return AmbiguousLidarHitTrackerConfig{
      .required_independent_scans = static_cast<std::size_t>(std::clamp<std::int64_t>(
          node.declare_parameter<std::int64_t>(
              "ambiguous_lidar_hit_required_independent_scans", 3),
          1, 20)),
      .max_scan_gap_ns = static_cast<std::int64_t>(
          1'000'000.0 * std::clamp(node.declare_parameter<double>(
                                       "ambiguous_lidar_hit_max_scan_gap_ms", 500.0),
                                   1.0, 10'000.0)),
      .retention_ns = static_cast<std::int64_t>(
          1'000'000.0 * std::clamp(node.declare_parameter<double>(
                                       "ambiguous_lidar_hit_retention_ms", 2000.0),
                                   1.0, 60'000.0)),
      .endpoint_voxel_size_m = std::clamp(
          node.declare_parameter<double>("ambiguous_lidar_hit_voxel_size_m", 0.5), 0.1,
          5.0),
      .min_viewpoint_translation_m =
          std::clamp(node.declare_parameter<double>(
                         "ambiguous_lidar_hit_min_viewpoint_shift_m", 0.5),
                     0.0, 10.0),
      .min_viewpoint_direction_change_rad =
          std::clamp(node.declare_parameter<double>(
                         "ambiguous_lidar_hit_min_viewpoint_angle_deg", 4.0),
                     0.0, 180.0) *
          std::numbers::pi / 180.0,
  };
}

LidarMappingYawConfig declareLidarMappingYawConfig(rclcpp::Node& node) {
  LidarMappingYawConfig config;
  config.use_px4_heading =
      node.declare_parameter<bool>("use_px4_heading_for_scan", true);
  config.initial_heading_rad =
      node.declare_parameter<double>("initial_heading_rad", 0.0);
  const double maximum_heading_variance_rad2 =
      node.declare_parameter<double>("maximum_heading_variance_rad2", 0.05);
  if (std::isfinite(maximum_heading_variance_rad2) &&
      maximum_heading_variance_rad2 >= 0.0) {
    config.maximum_heading_variance_rad2 = maximum_heading_variance_rad2;
  }
  config.startup_stable_sample_count = static_cast<std::size_t>(
      std::clamp<std::int64_t>(node.declare_parameter<std::int64_t>(
                                   "startup_heading_stable_sample_count", 5),
                               1, 1000));
  const double startup_maximum_sample_delta_rad =
      node.declare_parameter<double>("startup_heading_maximum_sample_delta_rad", 0.05);
  if (std::isfinite(startup_maximum_sample_delta_rad) &&
      startup_maximum_sample_delta_rad >= 0.0) {
    config.startup_maximum_sample_delta_rad =
        std::min(startup_maximum_sample_delta_rad, std::numbers::pi);
  }
  return config;
}

GroundLidarRejectionConfig
declareGroundLidarRejectionConfig(rclcpp::Node& node, const double max_lidar_range_m) {
  GroundLidarRejectionConfig config;
  config.enabled = node.declare_parameter<bool>("ground_lidar_rejection_enabled", true);
  config.ground_altitude_m =
      node.declare_parameter<double>("ground_lidar_altitude_m", 0.05);
  config.closer_range_tolerance_m =
      node.declare_parameter<double>("ground_lidar_closer_range_tolerance_m", 0.5);
  config.farther_range_tolerance_m =
      node.declare_parameter<double>("ground_lidar_farther_range_tolerance_m", 1.5);
  config.candidate_endpoint_altitude_tolerance_m = node.declare_parameter<double>(
      "ground_lidar_candidate_endpoint_altitude_tolerance_m", 1.5);
  config.attached_endpoint_altitude_tolerance_m = node.declare_parameter<double>(
      "ground_lidar_attached_endpoint_altitude_tolerance_m", 0.3);
  const bool valid = std::isfinite(config.ground_altitude_m) &&
                     std::isfinite(config.closer_range_tolerance_m) &&
                     config.closer_range_tolerance_m >= 0.0 &&
                     std::isfinite(config.farther_range_tolerance_m) &&
                     config.farther_range_tolerance_m >= 0.0 &&
                     std::isfinite(config.candidate_endpoint_altitude_tolerance_m) &&
                     config.candidate_endpoint_altitude_tolerance_m >= 0.0 &&
                     std::isfinite(config.attached_endpoint_altitude_tolerance_m) &&
                     config.attached_endpoint_altitude_tolerance_m >= 0.0 &&
                     config.attached_endpoint_altitude_tolerance_m <=
                         config.candidate_endpoint_altitude_tolerance_m &&
                     std::isfinite(max_lidar_range_m) && max_lidar_range_m > 0.0;
  const char* status = "disabled";
  if (config.enabled) {
    status = valid ? "ready" : "unavailable";
  }
  if (config.enabled && !valid) {
    RCLCPP_WARN(node.get_logger(),
                "Ground lidar classifier: node=obstacle_memory status=%s "
                "ground_altitude=%.3fm closer_tolerance=%.3fm "
                "farther_tolerance=%.3fm candidate_altitude_tolerance=%.3fm "
                "attached_altitude_tolerance=%.3fm",
                status, config.ground_altitude_m, config.closer_range_tolerance_m,
                config.farther_range_tolerance_m,
                config.candidate_endpoint_altitude_tolerance_m,
                config.attached_endpoint_altitude_tolerance_m);
  } else {
    RCLCPP_INFO(node.get_logger(),
                "Ground lidar classifier: node=obstacle_memory status=%s "
                "ground_altitude=%.3fm closer_tolerance=%.3fm "
                "farther_tolerance=%.3fm candidate_altitude_tolerance=%.3fm "
                "attached_altitude_tolerance=%.3fm",
                status, config.ground_altitude_m, config.closer_range_tolerance_m,
                config.farther_range_tolerance_m,
                config.candidate_endpoint_altitude_tolerance_m,
                config.attached_endpoint_altitude_tolerance_m);
  }
  return config;
}

LidarIngestionConfidenceConfig
declareLidarIngestionConfidenceConfig(rclcpp::Node& node) {
  return LidarIngestionConfidenceConfig{
      .enabled = node.declare_parameter<bool>(
          "lidar_uncertain_hit_confirmation_enabled", true),
      .require_source_timestamp_alignment_for_unknown = node.declare_parameter<bool>(
          "lidar_uncertain_unknown_require_source_timestamp_alignment", true),
      .reliable_range_margin_m =
          std::clamp(node.declare_parameter<double>(
                         "lidar_uncertain_unknown_reliable_range_margin_m", 0.5),
                     0.0, 10.0),
  };
}

} // namespace drone_city_nav
