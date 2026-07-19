#pragma once

#include "drone_city_nav/lidar_beam_observation.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace drone_city_nav {

struct LidarPoseHistoryConfig {
  std::int64_t retention_ns{3'000'000'000};
  std::int64_t max_extrapolation_ns{100'000'000};
};

struct TimestampAlignedLidarPose {
  LidarProjectionPose pose{};
  std::int64_t requested_stamp_ns{0};
  std::int64_t position_stamp_error_ns{0};
  std::int64_t attitude_stamp_error_ns{0};
  bool position_interpolated{false};
  bool attitude_interpolated{false};
};

class LidarPoseHistory {
public:
  explicit LidarPoseHistory(LidarPoseHistoryConfig config = {});

  void addPosition(std::int64_t stamp_ns, const Point3& position_map_m, double yaw_rad,
                   bool yaw_valid);
  void addAttitude(std::int64_t stamp_ns, const std::array<float, 4>& quaternion);

  [[nodiscard]] std::optional<TimestampAlignedLidarPose>
  sample(std::int64_t stamp_ns) const noexcept;

  void clear() noexcept;
  [[nodiscard]] std::size_t positionSampleCount() const noexcept;
  [[nodiscard]] std::size_t attitudeSampleCount() const noexcept;

private:
  struct PositionSample {
    std::int64_t stamp_ns{0};
    Point3 position_map_m{};
    double yaw_rad{0.0};
  };

  struct AttitudeSample {
    std::int64_t stamp_ns{0};
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

} // namespace drone_city_nav
