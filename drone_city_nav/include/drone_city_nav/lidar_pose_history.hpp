#pragma once

#include "drone_city_nav/lidar_beam_observation.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace drone_city_nav {

struct LidarPoseHistoryConfig {
  std::int64_t retention_ns{3'000'000'000};
  std::int64_t max_extrapolation_ns{100'000'000};
};

enum class LidarPoseTemporalMode : std::uint8_t {
  kUnavailable,
  kExact,
  kInterpolated,
  kExtrapolatedBefore,
  kExtrapolatedAfter,
};

struct LidarPoseTemporalAlignment {
  LidarPoseTemporalMode mode{LidarPoseTemporalMode::kUnavailable};
  std::int64_t requested_stamp_ns{0};
  std::int64_t from_receive_stamp_ns{0};
  std::int64_t to_receive_stamp_ns{0};
  std::int64_t from_source_stamp_ns{0};
  std::int64_t to_source_stamp_ns{0};
  std::int64_t signed_extrapolation_ns{0};
  double interpolation_ratio{0.0};
};

struct TimestampAlignedLidarPose {
  LidarProjectionPose pose{};
  std::int64_t requested_stamp_ns{0};
  std::int64_t position_stamp_error_ns{0};
  std::int64_t attitude_stamp_error_ns{0};
  bool position_interpolated{false};
  bool attitude_interpolated{false};
  LidarPoseTemporalAlignment position_timing{};
  LidarPoseTemporalAlignment attitude_timing{};
};

enum class LidarPoseAlignmentStatus {
  kAligned,
  kEmptyScan,
  kInvalidScanStamp,
  kInvalidBeamStamp,
  kPositionHistoryEmpty,
  kAttitudeHistoryEmpty,
  kExtrapolationExceeded,
  kAttitudeInvalid,
};

struct LidarPoseSampleResult {
  std::optional<TimestampAlignedLidarPose> aligned_pose;
  LidarPoseAlignmentStatus status{LidarPoseAlignmentStatus::kInvalidScanStamp};
  std::int64_t position_stamp_error_ns{0};
  std::int64_t attitude_stamp_error_ns{0};
  LidarPoseTemporalAlignment position_timing{};
  LidarPoseTemporalAlignment attitude_timing{};
};

struct LidarBeamPoseAlignmentResult {
  std::vector<LidarProjectionPose> poses;
  LidarPoseAlignmentStatus status{LidarPoseAlignmentStatus::kInvalidScanStamp};
  std::size_t failed_beam_index{0U};
  std::int64_t requested_stamp_ns{0};
  std::int64_t position_stamp_error_ns{0};
  std::int64_t attitude_stamp_error_ns{0};
  std::size_t position_sample_count{0U};
  std::size_t attitude_sample_count{0U};
  LidarPoseTemporalAlignment position_timing{};
  LidarPoseTemporalAlignment attitude_timing{};

  [[nodiscard]] bool aligned() const noexcept {
    return status == LidarPoseAlignmentStatus::kAligned;
  }
};

class LidarPoseHistory {
public:
  explicit LidarPoseHistory(LidarPoseHistoryConfig config = {});

  void addPosition(std::int64_t stamp_ns, const Point3& position_map_m, double yaw_rad,
                   bool yaw_valid, std::int64_t source_stamp_ns = 0);
  void addAttitude(std::int64_t stamp_ns, const std::array<float, 4>& quaternion,
                   std::int64_t source_stamp_ns = 0);

  [[nodiscard]] std::optional<TimestampAlignedLidarPose>
  sample(std::int64_t stamp_ns) const noexcept;
  [[nodiscard]] LidarPoseSampleResult
  sampleWithDiagnostics(std::int64_t stamp_ns) const noexcept;

  void clear() noexcept;
  [[nodiscard]] std::size_t positionSampleCount() const noexcept;
  [[nodiscard]] std::size_t attitudeSampleCount() const noexcept;

private:
  struct PositionSample {
    std::int64_t stamp_ns{0};
    std::int64_t source_stamp_ns{0};
    Point3 position_map_m{};
    double yaw_rad{0.0};
  };

  struct AttitudeSample {
    std::int64_t stamp_ns{0};
    std::int64_t source_stamp_ns{0};
    std::array<double, 4> quaternion{1.0, 0.0, 0.0, 0.0};
  };

  void prune(std::int64_t newest_stamp_ns);

  LidarPoseHistoryConfig config_{};
  std::deque<PositionSample> positions_;
  std::deque<AttitudeSample> attitudes_;
};

[[nodiscard]] std::optional<std::vector<LidarProjectionPose>>
timestampAlignedLidarBeamPoses(const LidarPoseHistory& history,
                               const LaserScanTiming& timing, std::size_t beam_count,
                               std::optional<double> fixed_yaw_rad = std::nullopt);

[[nodiscard]] LidarBeamPoseAlignmentResult
timestampAlignedLidarBeamPosesWithDiagnostics(
    const LidarPoseHistory& history, const LaserScanTiming& timing,
    std::size_t beam_count, std::optional<double> fixed_yaw_rad = std::nullopt);

[[nodiscard]] const char*
lidarPoseAlignmentStatusName(LidarPoseAlignmentStatus status) noexcept;

[[nodiscard]] const char*
lidarPoseTemporalModeName(LidarPoseTemporalMode mode) noexcept;

[[nodiscard]] std::int64_t
lidarPoseSourceTimestampNanoseconds(std::uint64_t timestamp_us) noexcept;

[[nodiscard]] std::string formatLidarPoseAlignmentDiagnostic(
    const char* prefix, const LidarBeamPoseAlignmentResult& result,
    const LaserScanTiming& timing, std::int64_t receive_stamp_ns);

} // namespace drone_city_nav
