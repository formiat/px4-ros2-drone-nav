#include "drone_city_nav/tracked_agent_lidar_filter.hpp"

#include <cmath>
#include <limits>

namespace drone_city_nav {

TrackedAgentLidarFilterResult
filterTrackedAgentLidarHits(const std::span<const float> ranges,
                            const TrackedAgentLidarFilterInput& input) {
  TrackedAgentLidarFilterResult result;
  result.ranges.assign(ranges.begin(), ranges.end());
  if (!(input.agent_radius_m > 0.0) || input.vertical_tolerance_m < 0.0 ||
      std::abs(input.agent_position.z - input.scan_altitude_m) >
          input.vertical_tolerance_m) {
    return result;
  }
  for (std::size_t index = 0U; index < result.ranges.size(); ++index) {
    const double range = static_cast<double>(result.ranges[index]);
    if (!std::isfinite(range) || range <= 0.0) {
      continue;
    }
    const double angle = input.scan_pose.yaw_rad + input.scan_yaw_offset_rad +
                         input.angle_min_rad +
                         static_cast<double>(index) * input.angle_increment_rad;
    const Point2 endpoint{input.scan_pose.position.x + range * std::cos(angle),
                          input.scan_pose.position.y + range * std::sin(angle)};
    if (std::hypot(endpoint.x - input.agent_position.x,
                   endpoint.y - input.agent_position.y) <= input.agent_radius_m) {
      result.ranges[index] = std::numeric_limits<float>::quiet_NaN();
      ++result.filtered_beams;
    }
  }
  return result;
}

} // namespace drone_city_nav
