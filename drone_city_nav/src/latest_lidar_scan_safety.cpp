#include "drone_city_nav/latest_lidar_scan_safety.hpp"

#include <cmath>

namespace drone_city_nav {

LatestLidarSafetyScanBuildResult
buildLatestLidarSafetyScan(const LatestLidarSafetyScanBuildInput& input) {
  LatestLidarSafetyScanBuildResult result{};
  result.source_beam_count = input.ranges.size();
  if (input.ranges.empty() ||
      input.beam_projection_poses.size() != input.ranges.size()) {
    return result;
  }
  result.acquisition_body_frame = lidarProjectionBodyFrame(
      input.beam_projection_poses.front(), input.projection_config);
  if (!result.acquisition_body_frame.valid) {
    return result;
  }
  result.hit_points_body_frd.reserve(input.ranges.size());
  for (std::size_t beam_index = 0U; beam_index < input.ranges.size(); ++beam_index) {
    const LidarBeamProjection projection = projectLidarBeam(
        input.beam_projection_poses[beam_index], input.projection_config,
        input.range_min_m, input.range_max_m, input.angle_min_rad,
        input.angle_increment_rad, beam_index, input.ranges[beam_index]);
    if (projection.status == LidarBeamProjectionStatus::kInvalidScan ||
        projection.status == LidarBeamProjectionStatus::kInvalidRange) {
      ++result.invalid_beam_count;
      continue;
    }
    if (!projection.hit || !projection.endpoint_xyz_valid) {
      continue;
    }
    const Point3 body_point =
        lidarMapPointToBody(result.acquisition_body_frame, projection.endpoint_map_m);
    if (std::isfinite(body_point.x) && std::isfinite(body_point.y) &&
        std::isfinite(body_point.z)) {
      result.hit_points_body_frd.push_back(body_point);
    } else {
      ++result.invalid_beam_count;
    }
  }
  result.valid = true;
  return result;
}

} // namespace drone_city_nav
