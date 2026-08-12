#include "drone_city_nav/tracked_agent_lidar_filter.hpp"

#include <cmath>
#include <limits>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool validVolume(const DynamicAgentLidarVolume& agent) noexcept {
  return std::isfinite(agent.position.x) && std::isfinite(agent.position.y) &&
         std::isfinite(agent.position.z) && agent.radius_m > 0.0 &&
         agent.lower_extent_m >= 0.0 && agent.upper_extent_m >= 0.0;
}

[[nodiscard]] bool contains(const DynamicAgentLidarVolume& agent,
                            const Point3& endpoint) noexcept {
  return endpoint.z >= agent.position.z - agent.lower_extent_m &&
         endpoint.z <= agent.position.z + agent.upper_extent_m &&
         std::hypot(endpoint.x - agent.position.x, endpoint.y - agent.position.y) <=
             agent.radius_m;
}

} // namespace

TrackedAgentLidarFilterResult
filterTrackedAgentLidarHits(const std::span<const float> ranges,
                            const TrackedAgentLidarFilterInput& input) {
  TrackedAgentLidarFilterResult result;
  result.ranges.assign(ranges.begin(), ranges.end());
  if (ranges.empty() || input.beam_projection_poses.size() != ranges.size() ||
      input.agents.empty()) {
    return result;
  }

  std::vector<bool> matched_agents(input.agents.size(), false);
  for (std::size_t beam_index = 0U; beam_index < result.ranges.size(); ++beam_index) {
    const LidarBeamProjection projection = projectLidarBeam(
        input.beam_projection_poses[beam_index], input.projection_config,
        input.range_min_m, input.range_max_m, input.angle_min_rad,
        input.angle_increment_rad, beam_index, result.ranges[beam_index]);
    if (!projection.hit || !projection.endpoint_xyz_valid) {
      continue;
    }
    for (std::size_t agent_index = 0U; agent_index < input.agents.size();
         ++agent_index) {
      const DynamicAgentLidarVolume& agent = input.agents[agent_index];
      if (!validVolume(agent) || !contains(agent, projection.endpoint_map_m)) {
        continue;
      }
      result.ranges[beam_index] = std::numeric_limits<float>::quiet_NaN();
      matched_agents[agent_index] = true;
      ++result.filtered_beams;
      break;
    }
  }
  for (const bool matched : matched_agents) {
    result.matched_agents += matched ? 1U : 0U;
  }
  return result;
}

} // namespace drone_city_nav
