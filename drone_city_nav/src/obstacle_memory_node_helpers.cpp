#include "obstacle_memory_node_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

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

LidarAcquisitionPoseConfig declareLidarAcquisitionPoseConfig(rclcpp::Node& node) {
  const auto offset = [&node](const char* const name, const double fallback) {
    return std::clamp(node.declare_parameter<double>(name, fallback), -1.0, 1.0);
  };
  return LidarAcquisitionPoseConfig{
      .apply_source_time_offsets =
          node.declare_parameter<bool>("motion_compensate_lidar_pose", true),
      .position_source_time_offset_s =
          offset("lidar_position_source_time_offset_s", 0.05),
      .attitude_source_time_offset_s =
          offset("lidar_attitude_source_time_offset_s", 0.0),
      .require_source_timestamp_alignment = true,
      .require_bracketed_pose = true,
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

DynamicAgentLidarStateConfig declareDynamicAgentLidarStateConfig(rclcpp::Node& node) {
  const auto maximum_peers = static_cast<std::size_t>(std::clamp<std::int64_t>(
      node.declare_parameter<std::int64_t>("cooperative_maximum_peers", 32), 1, 1024));
  return DynamicAgentLidarStateConfig{
      .cooperative_enabled =
          node.declare_parameter<bool>("cooperative_traffic_enabled", false),
      .own_vehicle_id = node.declare_parameter<std::string>("vehicle_id", ""),
      .cooperative_peer_horizontal_margin_m = node.declare_parameter<double>(
          "cooperative_peer_filter_horizontal_margin_m", 0.0),
      .cooperative_peer_vertical_margin_m = node.declare_parameter<double>(
          "cooperative_peer_filter_vertical_margin_m", 0.0),
      .cooperative_alignment_extrapolation_s = node.declare_parameter<double>(
          "cooperative_peer_filter_alignment_extrapolation_s", 0.5),
      .peer_store =
          CooperativePeerStoreConfig{
              .maximum_publication_age_s = node.declare_parameter<double>(
                  "cooperative_peer_maximum_publication_age_s", 0.5),
              .maximum_peers = maximum_peers,
          },
  };
}

} // namespace drone_city_nav
