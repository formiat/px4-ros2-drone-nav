#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

struct TrackedAgentLidarFilterInput {
  Pose2 scan_pose{};
  double scan_altitude_m{0.0};
  double angle_min_rad{0.0};
  double angle_increment_rad{0.0};
  double scan_yaw_offset_rad{0.0};
  Point3 agent_position{};
  double agent_radius_m{1.0};
  double vertical_tolerance_m{1.0};
};

struct TrackedAgentLidarFilterResult {
  std::vector<float> ranges;
  std::size_t filtered_beams{0U};
};

[[nodiscard]] TrackedAgentLidarFilterResult
filterTrackedAgentLidarHits(std::span<const float> ranges,
                            const TrackedAgentLidarFilterInput& input);

} // namespace drone_city_nav
