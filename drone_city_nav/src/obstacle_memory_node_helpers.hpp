#pragma once

#include "drone_city_nav/ambiguous_lidar_hit_tracker.hpp"
#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/known_static_lidar_hit_classifier.hpp"
#include "drone_city_nav/lidar_ingestion_decision.hpp"
#include "drone_city_nav/occupancy_grid.hpp"

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

struct KnownStaticLidarSetup {
  std::optional<KnownPassageMap> passage_map;
  std::optional<KnownStaticLidarHitClassifier> classifier;
  std::filesystem::path resolved_path;
  double closer_range_tolerance_m{0.5};
  double farther_range_tolerance_m{1.5};
  double endpoint_volume_tolerance_m{0.75};
  double opening_boundary_tolerance_m{0.50};
};

[[nodiscard]] KnownStaticLidarSetup
declareKnownStaticLidarSetup(rclcpp::Node& node, const std::string& frame_id);

[[nodiscard]] GroundLidarRejectionConfig
declareGroundLidarRejectionConfig(rclcpp::Node& node, double max_lidar_range_m);

[[nodiscard]] LidarIngestionConfidenceConfig
declareLidarIngestionConfidenceConfig(rclcpp::Node& node);

} // namespace drone_city_nav
