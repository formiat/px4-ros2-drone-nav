#pragma once

#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace drone_city_nav {

struct LatestSensorObstacleScanBuildInput {
  std::span<const float> ranges{};
  std::span<const LidarProjectionPose> beam_projection_poses{};
  LidarProjectionConfig projection_config{};
  double range_min_m{0.0};
  double range_max_m{0.0};
  double angle_min_rad{0.0};
  double angle_increment_rad{0.0};
};

struct LatestSensorObstacleScanBuildResult {
  LidarProjectionBodyFrame acquisition_body_frame{};
  std::vector<Point3> hit_points_body_frd;
  std::size_t source_beam_count{0U};
  std::size_t invalid_beam_count{0U};
  bool valid{false};
};

[[nodiscard]] LatestSensorObstacleScanBuildResult
buildLatestSensorObstacleScan(const LatestSensorObstacleScanBuildInput& input);

// The scan's returns thinned to one per cubic cell of the body frame, the one
// nearest the vehicle, in the order the cells were first met. A sensor's
// returns lie a centimetre apart on a surface a metre away, 66 000 of them on
// a wall the envelope touches, and every consumer of the scan pays for each:
// one validation of an 80-segment path beside such a wall cost 487 ms, and
// 23 ms against the scan thinned to 0.05 m cells, so a planning tick lasted
// 2.5 s while the vehicle flew its resident horizon unattended (r545; r536
// for 6 s). The nearest return of a cell is kept, so the evidence closest to
// the vehicle survives in every cell, and a dropped return lies within the
// cell's diagonal, 0.087 m, of a kept one: under the 0.125 m contact
// tolerance the scan is judged with and the 0.25 m voxel of the memory.
inline constexpr double kLatestSensorScanCellM{0.05};

[[nodiscard]] std::vector<Point3>
thinnedNearestReturns(std::span<const Point3> hit_points_body_frd, double cell_m);

[[nodiscard]] std::uint64_t createLatestSensorObstacleProducerInstanceId() noexcept;

} // namespace drone_city_nav
