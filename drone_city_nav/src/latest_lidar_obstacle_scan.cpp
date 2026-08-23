#include "drone_city_nav/latest_lidar_obstacle_scan.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {

LatestLidarObstacleScanBuildResult
buildLatestLidarObstacleScan(const LatestLidarObstacleScanBuildInput& input) {
  LatestLidarObstacleScanBuildResult result{};
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

LatestLidarObstacleFreshness
assessLatestLidarObstacleFreshness(const LatestLidarObstacleSnapshot& snapshot,
                                   const std::int64_t now_ns,
                                   const double maximum_age_ms) noexcept {
  LatestLidarObstacleFreshness result;
  if (snapshot.acquisition_stamp_ns <= 0 || snapshot.receive_stamp_ns <= 0 ||
      now_ns <= 0 || !std::isfinite(maximum_age_ms) || maximum_age_ms <= 0.0) {
    return result;
  }

  const auto maximum_age_ns =
      static_cast<std::int64_t>(std::llround(maximum_age_ms * 1.0e6));
  if (maximum_age_ns <= 0) {
    return result;
  }
  const std::int64_t acquisition_age_ns = now_ns - snapshot.acquisition_stamp_ns;
  const std::int64_t receive_age_ns = now_ns - snapshot.receive_stamp_ns;
  result.receive_time_fallback = acquisition_age_ns < 0;
  result.age_ms = static_cast<double>(
                      std::max({std::int64_t{0}, acquisition_age_ns, receive_age_ns})) *
                  1.0e-6;
  result.fresh = acquisition_age_ns >= -maximum_age_ns &&
                 acquisition_age_ns <= maximum_age_ns && receive_age_ns >= 0 &&
                 receive_age_ns <= maximum_age_ns;
  return result;
}

} // namespace drone_city_nav
