#include "drone_city_nav/latest_sensor_obstacle_scan.hpp"

#include "drone_city_nav/producer_instance_id.hpp"

#include <cmath>
#include <unordered_map>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kLatestSensorProducerDomain{0x4c49444152505244ULL};

} // namespace

LatestSensorObstacleScanBuildResult
buildLatestSensorObstacleScan(const LatestSensorObstacleScanBuildInput& input) {
  LatestSensorObstacleScanBuildResult result{};
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
  // An all-invalid scan carries no authenticated free or occupied ray evidence.
  // Reject it at the producer boundary instead of advancing the evidence stream
  // with a snapshot every consumer must discard.
  result.valid = result.invalid_beam_count < result.source_beam_count;
  return result;
}

std::vector<Point3>
thinnedNearestReturns(const std::span<const Point3> hit_points_body_frd,
                      const double cell_m) {
  std::vector<Point3> kept;
  if (!(cell_m > 0.0)) {
    kept.assign(hit_points_body_frd.begin(), hit_points_body_frd.end());
    return kept;
  }
  const auto squaredRange = [](const Point3& point) {
    return point.x * point.x + point.y * point.y + point.z * point.z;
  };
  // 21 bits an axis: 52 km of 0.05 m cells either way, past any sensor.
  const auto cellCoordinate = [cell_m](const double value) {
    return static_cast<std::uint64_t>(
               static_cast<std::int64_t>(std::floor(value / cell_m)) + (1LL << 20U)) &
           ((1ULL << 21U) - 1ULL);
  };
  std::unordered_map<std::uint64_t, std::size_t> kept_index_by_cell;
  kept_index_by_cell.reserve(hit_points_body_frd.size());
  for (const Point3& point : hit_points_body_frd) {
    const std::uint64_t cell = (cellCoordinate(point.x) << 42U) |
                               (cellCoordinate(point.y) << 21U) |
                               cellCoordinate(point.z);
    const auto [entry, inserted] = kept_index_by_cell.try_emplace(cell, kept.size());
    if (inserted) {
      kept.push_back(point);
    } else if (squaredRange(point) < squaredRange(kept[entry->second])) {
      kept[entry->second] = point;
    }
  }
  return kept;
}

std::uint64_t createLatestSensorObstacleProducerInstanceId() noexcept {
  return createProducerInstanceId(kLatestSensorProducerDomain);
}

} // namespace drone_city_nav
