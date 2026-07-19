#pragma once

#include "drone_city_nav/lidar_ingestion_decision.hpp"
#include "drone_city_nav/lidar_motion_compensation.hpp"
#include "drone_city_nav/lidar_pose_history.hpp"
#include "drone_city_nav/obstacle_memory.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iosfwd>
#include <optional>
#include <string>
#include <string_view>

namespace drone_city_nav {

// Captures the exact inputs used for one memory hit without participating in
// mapping. It exists to make a retained lidar return reproducible from a run.
struct LidarMemoryHitDiagnosticContext {
  std::int64_t callback_stamp_ns{0};
  std::int64_t pose_sample_stamp_ns{0};
  bool pose_sample_stamp_valid{false};
  std::int64_t pose_receive_stamp_ns{0};
  bool pose_receive_stamp_valid{false};
  std::int64_t attitude_sample_stamp_ns{0};
  bool attitude_sample_stamp_valid{false};
  std::int64_t attitude_receive_stamp_ns{0};
  bool attitude_receive_stamp_valid{false};
  LidarProjectionPose vehicle_pose{};
  Point2 horizontal_velocity{};
  bool horizontal_velocity_valid{false};
  LidarPoseMotionCompensationResult motion_compensation{};
  LidarPoseSampleResult acquisition_pose_alignment{};
  double scan_range_min_m{0.0};
  double scan_range_max_m{0.0};
  double scan_angle_min_rad{0.0};
  double scan_angle_increment_rad{0.0};
  double scan_time_increment_s{0.0};
  double scan_duration_s{0.0};
  LidarProjectionConfig projection_config{};
  GroundLidarRejectionConfig ground_config{};
  double known_static_closer_range_tolerance_m{0.0};
  double known_static_farther_range_tolerance_m{0.0};
  double known_static_endpoint_volume_tolerance_m{0.0};
  double known_static_opening_boundary_tolerance_m{0.0};
};

struct LidarMemoryHitDiagnosticRecord {
  std::uint64_t record_index{0U};
  ObstacleMemoryOccupiedTransition transition{};
  LidarMemoryHitDiagnosticContext context{};
};

struct LidarMemoryHitDumpConfig {
  bool enabled{true};
  std::filesystem::path path{};
  std::uint64_t max_records{10000U};
};

enum class LidarMemoryHitDumpOpenStatus {
  kReady,
  kDisabled,
  kCreateDirectoryFailed,
  kOpenFailed,
};

enum class LidarMemoryHitDumpWriteStatus {
  kWritten,
  kDisabled,
  kLimitReached,
  kWriteFailed,
};

struct LidarMemoryHitDumpWriteResult {
  LidarMemoryHitDumpWriteStatus status{LidarMemoryHitDumpWriteStatus::kDisabled};
  std::uint64_t record_index{0U};
  bool first_limit_reached{false};
};

class LidarMemoryHitDumpWriter {
public:
  [[nodiscard]] LidarMemoryHitDumpOpenStatus open(LidarMemoryHitDumpConfig config);

  [[nodiscard]] LidarMemoryHitDumpWriteResult
  write(const LidarMemoryHitDiagnosticRecord& record);

  [[nodiscard]] const std::filesystem::path& path() const noexcept;

private:
  LidarMemoryHitDumpConfig config_{};
  std::ofstream stream_;
  std::uint64_t records_written_{0U};
  bool limit_reported_{false};
};

void writeLidarMemoryHitDiagnosticJson(std::ostream& stream,
                                       const LidarMemoryHitDiagnosticRecord& record);

[[nodiscard]] std::optional<std::int64_t>
px4TimestampNanoseconds(std::uint64_t timestamp_us) noexcept;

[[nodiscard]] double lidarScanDurationSeconds(double scan_time_s,
                                              double time_increment_s,
                                              std::size_t beam_count) noexcept;

[[nodiscard]] std::string formatPassageMemoryHitDiagnostic(
    std::uint64_t dump_record_index, std::string_view structure_id,
    const ObstacleMemoryOccupiedTransition& transition,
    const Point3& vehicle_position_map_m, const Point2& scan_pose_map_m,
    const LidarMemoryHitDiagnosticContext& context);

[[nodiscard]] bool
isRetainedExpectedSurfaceHit(const LidarIngestionDecision& decision) noexcept;

} // namespace drone_city_nav
