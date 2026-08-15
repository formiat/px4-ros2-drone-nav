#pragma once

#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace drone_city_nav {

struct LatestLidarObstacleScanBuildInput {
  std::span<const float> ranges{};
  std::span<const LidarProjectionPose> beam_projection_poses{};
  LidarProjectionConfig projection_config{};
  double range_min_m{0.0};
  double range_max_m{0.0};
  double angle_min_rad{0.0};
  double angle_increment_rad{0.0};
};

struct LatestLidarObstacleScanBuildResult {
  LidarProjectionBodyFrame acquisition_body_frame{};
  std::vector<Point3> hit_points_body_frd;
  std::size_t source_beam_count{0U};
  std::size_t invalid_beam_count{0U};
  bool valid{false};
};

struct LatestLidarObstacleSnapshot {
  std::vector<Point3> hit_points_map_m;
  std::int64_t acquisition_stamp_ns{0};
  std::int64_t receive_stamp_ns{0};
  std::uint64_t sequence{0U};
  std::uint64_t pose_generation{0U};
  std::size_t source_beam_count{0U};
  std::size_t invalid_beam_count{0U};
};

[[nodiscard]] LatestLidarObstacleScanBuildResult
buildLatestLidarObstacleScan(const LatestLidarObstacleScanBuildInput& input);

} // namespace drone_city_nav
