#pragma once

#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

struct DynamicAgentLidarVolume {
  Point3 position{};
  double radius_m{0.0};
  double lower_extent_m{0.0};
  double upper_extent_m{0.0};
};

struct TrackedAgentLidarFilterInput {
  std::span<const LidarProjectionPose> beam_projection_poses{};
  LidarProjectionConfig projection_config{};
  double range_min_m{0.0};
  double range_max_m{0.0};
  double angle_min_rad{0.0};
  double angle_increment_rad{0.0};
  std::span<const DynamicAgentLidarVolume> agents{};
};

struct TrackedAgentLidarFilterResult {
  std::vector<float> ranges;
  std::size_t filtered_beams{0U};
  std::size_t matched_agents{0U};
};

[[nodiscard]] TrackedAgentLidarFilterResult
filterTrackedAgentLidarHits(std::span<const float> ranges,
                            const TrackedAgentLidarFilterInput& input);

} // namespace drone_city_nav
