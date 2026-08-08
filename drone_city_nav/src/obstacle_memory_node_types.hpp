#pragma once

#include "drone_city_nav/lidar_memory_hit_diagnostics.hpp"
#include "drone_city_nav/lidar_pose_history.hpp"
#include "drone_city_nav/obstacle_memory.hpp"
#include "drone_city_nav/px4_ros_time_mapper.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

#include <cstdint>
#include <vector>

namespace drone_city_nav {

struct LidarMemoryHitDiagnosticBatch {
  std::vector<ObstacleMemoryOccupiedTransition> transitions;
  LidarMemoryHitDiagnosticContext common_context;
  LidarPoseHistory pose_history;
  Px4RosTimeMapper time_mapper;
};

struct PendingLidarScan {
  sensor_msgs::msg::LaserScan scan;
  std::int64_t receive_stamp_ns{0};
};

enum class PendingLidarScanDisposition : std::uint8_t {
  kWaitForPoseBracket,
  kConsumed,
};

} // namespace drone_city_nav
