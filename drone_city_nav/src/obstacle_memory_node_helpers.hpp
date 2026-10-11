#pragma once

#include "drone_city_nav/dynamic_agent_lidar_state.hpp"
#include "drone_city_nav/lidar_acquisition_pose.hpp"

#include <rclcpp/node.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace drone_city_nav {

[[nodiscard]] std::optional<std::int64_t>
validRosStampNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept;

struct LidarMappingYawConfig {
  bool use_px4_heading{true};
  double initial_heading_rad{0.0};
  double maximum_heading_variance_rad2{0.05};
  std::size_t startup_stable_sample_count{5U};
  double startup_maximum_sample_delta_rad{0.05};
};

[[nodiscard]] LidarMappingYawConfig declareLidarMappingYawConfig(rclcpp::Node& node);

// The per-source time offsets of the lidar acquisition pose, clamped to a
// second either way; production mapping accepts only source-aligned,
// bracketed poses.
[[nodiscard]] LidarAcquisitionPoseConfig
declareLidarAcquisitionPoseConfig(rclcpp::Node& node);

[[nodiscard]] DynamicAgentLidarStateConfig
declareDynamicAgentLidarStateConfig(rclcpp::Node& node);

} // namespace drone_city_nav
