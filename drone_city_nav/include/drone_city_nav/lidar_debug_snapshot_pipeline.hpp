#pragma once

#include "drone_city_nav/lidar_snapshot_writer.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

enum class LidarDebugSnapshotReadiness {
  kReady,
  kNoScan,
  kNoPose,
  kNoProjection,
};

struct LidarDebugSnapshotInput {
  bool scan_seen{false};
  bool pose_seen{false};
  bool projection_seen{false};
  std::span<const float> ranges{};
  double range_min_m{0.0};
  double range_max_m{0.0};
  double angle_min_rad{0.0};
  double angle_increment_rad{0.0};
  std::size_t beam_csv_stride{1U};
  std::size_t remembered_hits_count{0U};
};

struct LidarDebugSnapshotOutput {
  LidarDebugSnapshotReadiness readiness{LidarDebugSnapshotReadiness::kNoScan};
  bool ready{false};
  LidarSnapshotStats stats{};
  std::vector<LidarSnapshotCsvRow> rows;
  std::vector<Point2> hit_points;
  std::size_t remembered_hits_count{0U};
};

using LidarBeamProjectionCallback = LidarBeamProjection (*)(std::size_t beam_index,
                                                            float raw_range,
                                                            const void* context);

[[nodiscard]] std::string lidarSnapshotPrefix(std::uint64_t snapshot_index);

[[nodiscard]] double ageSecondsOrNan(std::int64_t stamp_ns,
                                     std::int64_t now_ns) noexcept;

[[nodiscard]] double yawDeltaRad(double lhs_rad, double rhs_rad) noexcept;

[[nodiscard]] double lidarScanDurationSeconds(double scan_time_s,
                                              double time_increment_s,
                                              std::size_t beam_count,
                                              double override_s) noexcept;

[[nodiscard]] double lidarScanTimeIncrementSeconds(double scan_time_s,
                                                   double time_increment_s,
                                                   std::size_t beam_count) noexcept;

[[nodiscard]] LidarDebugSnapshotOutput
buildLidarDebugSnapshotOutput(const LidarDebugSnapshotInput& input,
                              LidarBeamProjectionCallback project_beam,
                              const void* context);

} // namespace drone_city_nav
