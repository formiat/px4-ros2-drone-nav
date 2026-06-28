#include "drone_city_nav/lidar_debug_snapshot_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numbers>
#include <sstream>

namespace drone_city_nav {

[[nodiscard]] std::string lidarSnapshotPrefix(const std::uint64_t snapshot_index) {
  std::ostringstream stream;
  stream << "snapshot_" << std::setw(6) << std::setfill('0') << snapshot_index;
  return stream.str();
}

[[nodiscard]] double ageSecondsOrNan(const std::int64_t stamp_ns,
                                     const std::int64_t now_ns) noexcept {
  if (stamp_ns <= 0 || now_ns <= 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (stamp_ns >= now_ns) {
    return 0.0;
  }
  constexpr double kNanosecondsPerSecond = 1.0e9;
  return static_cast<double>(now_ns - stamp_ns) / kNanosecondsPerSecond;
}

[[nodiscard]] double yawDeltaRad(const double lhs_rad, const double rhs_rad) noexcept {
  if (!std::isfinite(lhs_rad) || !std::isfinite(rhs_rad)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::remainder(lhs_rad - rhs_rad, 2.0 * std::numbers::pi);
}

[[nodiscard]] double lidarScanDurationSeconds(const double scan_time_s,
                                              const double time_increment_s,
                                              const std::size_t beam_count,
                                              const double override_s) noexcept {
  if (override_s > 0.0) {
    return override_s;
  }
  if (std::isfinite(scan_time_s) && scan_time_s > 0.0) {
    return scan_time_s;
  }
  if (std::isfinite(time_increment_s) && time_increment_s > 0.0 && beam_count > 1U) {
    return time_increment_s * static_cast<double>(beam_count - 1U);
  }
  return 0.0;
}

[[nodiscard]] double
lidarScanTimeIncrementSeconds(const double scan_time_s, const double time_increment_s,
                              const std::size_t beam_count) noexcept {
  if (std::isfinite(time_increment_s) && time_increment_s > 0.0) {
    return time_increment_s;
  }
  if (scan_time_s > 0.0 && beam_count > 1U) {
    return scan_time_s / static_cast<double>(beam_count - 1U);
  }
  return 0.0;
}

[[nodiscard]] LidarDebugSnapshotOutput
buildLidarDebugSnapshotOutput(const LidarDebugSnapshotInput& input,
                              const LidarBeamProjectionCallback project_beam,
                              const void* context) {
  LidarDebugSnapshotOutput output{};
  output.remembered_hits_count = input.remembered_hits_count;
  if (!input.scan_seen) {
    output.readiness = LidarDebugSnapshotReadiness::kNoScan;
    return output;
  }
  if (!input.pose_seen) {
    output.readiness = LidarDebugSnapshotReadiness::kNoPose;
    return output;
  }
  if (!input.projection_seen || project_beam == nullptr) {
    output.readiness = LidarDebugSnapshotReadiness::kNoProjection;
    return output;
  }

  output.readiness = LidarDebugSnapshotReadiness::kReady;
  output.ready = true;
  const std::size_t stride = std::max<std::size_t>(input.beam_csv_stride, 1U);
  if (!(input.range_max_m > 0.0) || input.angle_increment_rad == 0.0) {
    return output;
  }
  output.rows.reserve((input.ranges.size() + stride - 1U) / stride);

  for (std::size_t i = 0U; i < input.ranges.size(); i += stride) {
    const float raw_range = input.ranges[i];
    ++output.stats.processed_beams;

    const LidarBeamProjection projection = project_beam(i, raw_range, context);
    switch (projection.status) {
      case LidarBeamProjectionStatus::kAccepted:
        ++output.stats.accepted_beams;
        if (projection.hit) {
          ++output.stats.hit_beams;
          output.stats.hit_points.push_back(projection.endpoint);
          output.hit_points.push_back(projection.endpoint);
        }
        if (std::isfinite(projection.endpoint_altitude_m)) {
          output.stats.endpoint_altitude_min_m = std::min(
              output.stats.endpoint_altitude_min_m, projection.endpoint_altitude_m);
          output.stats.endpoint_altitude_max_m = std::max(
              output.stats.endpoint_altitude_max_m, projection.endpoint_altitude_m);
        }
        break;
      case LidarBeamProjectionStatus::kAltitudeRejected:
        ++output.stats.altitude_rejected_beams;
        break;
      case LidarBeamProjectionStatus::kInvalidRange:
        ++output.stats.invalid_range_beams;
        ++output.stats.projection_rejected_beams;
        break;
      case LidarBeamProjectionStatus::kInvalidScan:
        ++output.stats.invalid_scan_beams;
        ++output.stats.projection_rejected_beams;
        break;
    }

    const bool endpoint_available =
        projection.status == LidarBeamProjectionStatus::kAccepted ||
        projection.status == LidarBeamProjectionStatus::kAltitudeRejected;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double angle_rad =
        input.angle_min_rad + static_cast<double>(i) * input.angle_increment_rad;
    output.rows.push_back(LidarSnapshotCsvRow{
        i, angle_rad, std::isfinite(raw_range) ? static_cast<double>(raw_range) : nan,
        projection.used_range_m, projection.hit,
        endpoint_available ? projection.endpoint.x : nan,
        endpoint_available ? projection.endpoint.y : nan,
        projection.endpoint_altitude_m, projection.status, projection.lidar_direction,
        projection.body_frd_direction, projection.ned_direction});
  }
  return output;
}

} // namespace drone_city_nav
