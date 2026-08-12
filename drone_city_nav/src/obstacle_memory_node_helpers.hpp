#pragma once

#include "drone_city_nav/ambiguous_lidar_hit_tracker.hpp"
#include "drone_city_nav/dynamic_agent_lidar_state.hpp"
#include "drone_city_nav/lidar_ingestion_decision.hpp"
#include "drone_city_nav/lidar_pose_history.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/px4_ros_time_mapper.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

[[nodiscard]] std::optional<std::int64_t>
validRosStampNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept;

[[nodiscard]] double navigationPoseAgeSeconds(std::int64_t last_pose_update_ns,
                                              std::int64_t now_ns) noexcept;

[[nodiscard]] double
navigationPoseReceiveLagSeconds(std::int64_t last_pose_update_ns,
                                std::int64_t scan_receive_ns) noexcept;

[[nodiscard]] LidarPoseSampleResult
samplePoseAtRosAcquisition(const LidarPoseHistory& pose_history,
                           const Px4RosTimeMapper& time_mapper,
                           std::int64_t ros_stamp_ns, bool stamp_valid) noexcept;

void logFirstNavigationPose(rclcpp::Node& node, bool& pose_seen,
                            const NavigationPose2D& pose, const char* source_name);

void invalidateObstacleNavigationPose(NavigationPose2D& pose,
                                      std::int64_t& last_pose_update_ns,
                                      Point2& velocity, bool& velocity_valid) noexcept;

[[nodiscard]] std::int8_t rawOccupancyValue(const OccupancyGrid2D& grid,
                                            GridIndex cell);

[[nodiscard]] nav_msgs::msg::OccupancyGrid
makeObstacleMemoryOccupancyGridMessage(const OccupancyGrid2D& grid,
                                       const rclcpp::Time& stamp,
                                       const std::string& frame_id);

[[nodiscard]] AmbiguousLidarHitTrackerConfig
declareAmbiguousLidarHitTrackerConfig(rclcpp::Node& node);

struct LidarMappingYawConfig {
  bool use_px4_heading{true};
  double initial_heading_rad{0.0};
  double maximum_heading_variance_rad2{0.05};
  std::size_t startup_stable_sample_count{5U};
  double startup_maximum_sample_delta_rad{0.05};
};

[[nodiscard]] LidarMappingYawConfig declareLidarMappingYawConfig(rclcpp::Node& node);

[[nodiscard]] GroundLidarRejectionConfig
declareGroundLidarRejectionConfig(rclcpp::Node& node, double max_lidar_range_m);

[[nodiscard]] LidarIngestionConfidenceConfig
declareLidarIngestionConfidenceConfig(rclcpp::Node& node);

[[nodiscard]] DynamicAgentLidarStateConfig
declareDynamicAgentLidarStateConfig(rclcpp::Node& node);

struct DynamicAgentLidarScanView {
  std::span<const float> ranges{};
  std::span<const LidarProjectionPose> beam_projection_poses{};
  LidarProjectionConfig projection_config{};
  double range_min_m{0.0};
  double range_max_m{0.0};
  double angle_min_rad{0.0};
  double angle_increment_rad{0.0};
};

[[nodiscard]] DynamicAgentLidarScanView makeDynamicAgentLidarScanView(
    const sensor_msgs::msg::LaserScan& scan,
    std::span<const LidarProjectionPose> beam_projection_poses,
    const LidarProjectionConfig& projection_config) noexcept;

struct DynamicAgentLidarScanFilterResult {
  std::vector<float> tracked_agent_ranges;
  std::vector<float> cooperative_memory_ranges;
  std::size_t tracked_agent_filtered_beams{0U};
  std::size_t tracked_agent_matches{0U};
  std::size_t cooperative_filtered_beams{0U};
  std::size_t cooperative_peer_matches{0U};
  bool tracked_agent_filter_applied{false};
  bool cooperative_filter_applied{false};
  bool tracked_agent_excluded_from_latest_safety{false};

  [[nodiscard]] std::span<const float>
  persistentRanges(std::span<const float> raw_ranges) const noexcept;

  [[nodiscard]] std::span<const float>
  latestSafetyRanges(std::span<const float> raw_ranges) const noexcept;
};

[[nodiscard]] DynamicAgentLidarScanFilterResult
filterDynamicAgentsFromLidarScan(const DynamicAgentLidarScanView& scan,
                                 const DynamicAgentLidarFilterPlan& filter_plan);

} // namespace drone_city_nav
