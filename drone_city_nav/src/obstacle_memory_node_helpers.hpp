#pragma once

#include "drone_city_nav/ambiguous_lidar_hit_tracker.hpp"
#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/known_static_lidar_hit_classifier.hpp"
#include "drone_city_nav/lidar_ingestion_decision.hpp"
#include "drone_city_nav/lidar_pose_history.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/px4_ros_time_mapper.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

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

[[nodiscard]] const PassageStructure*
passageStructureNearPoint(const std::optional<KnownPassageMap>& map, Point2 point,
                          double margin_m) noexcept;

[[nodiscard]] AmbiguousLidarHitTrackerConfig
declareAmbiguousLidarHitTrackerConfig(rclcpp::Node& node);

struct LidarMappingYawConfig {
  bool use_px4_heading{true};
  double initial_heading_rad{0.0};
  double maximum_heading_variance_rad2{0.05};
  double startup_alignment_tolerance_rad{0.15};
};

[[nodiscard]] LidarMappingYawConfig declareLidarMappingYawConfig(rclcpp::Node& node);

struct KnownStaticLidarSetup {
  std::optional<KnownPassageMap> passage_map;
  std::optional<KnownStaticLidarHitClassifier> classifier;
  std::filesystem::path resolved_path;
  bool passages_enabled{false};
  bool classifier_enabled{false};
  double closer_range_tolerance_m{0.5};
  double farther_range_tolerance_m{1.5};
  double endpoint_volume_tolerance_m{0.75};
  double opening_boundary_tolerance_m{0.50};
};

[[nodiscard]] KnownStaticLidarSetup
declareKnownStaticLidarSetup(rclcpp::Node& node, const std::string& frame_id,
                             bool use_static_map);

[[nodiscard]] GroundLidarRejectionConfig
declareGroundLidarRejectionConfig(rclcpp::Node& node, double max_lidar_range_m);

[[nodiscard]] LidarIngestionConfidenceConfig
declareLidarIngestionConfidenceConfig(rclcpp::Node& node);

} // namespace drone_city_nav
