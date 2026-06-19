#pragma once

#include "drone_city_nav/types.hpp"

#include <array>
#include <cstddef>
#include <limits>
#include <optional>

namespace drone_city_nav {

struct AttitudeEuler {
  double roll_rad{0.0};
  double pitch_rad{0.0};
  double yaw_rad{0.0};
};

struct LidarProjectionPose {
  Point2 position{};
  double altitude_m{0.0};
  double yaw_rad{0.0};
  double roll_rad{0.0};
  double pitch_rad{0.0};
  bool altitude_valid{false};
  bool attitude_valid{false};
};

struct LidarProjectionConfig {
  double max_lidar_range_m{35.0};
  double range_hit_epsilon_m{0.05};
  double scan_yaw_offset_rad{0.0};
  double lidar_z_offset_m{0.0};
  double min_projected_altitude_m{0.0};
  double max_projected_altitude_m{100000.0};
  bool swap_lidar_xy_to_local_frame{false};
  bool compensate_attitude{false};
  double lidar_mount_roll_rad{0.0};
  double lidar_mount_pitch_rad{0.0};
  double lidar_mount_yaw_rad{0.0};
};

enum class LidarBeamProjectionStatus {
  kAccepted,
  kInvalidScan,
  kInvalidRange,
  kAltitudeRejected,
};

struct LidarBeamProjection {
  LidarBeamProjectionStatus status{LidarBeamProjectionStatus::kInvalidScan};
  bool hit{false};
  double used_range_m{0.0};
  double endpoint_altitude_m{std::numeric_limits<double>::quiet_NaN()};
  double depth_endpoint_altitude_m{std::numeric_limits<double>::quiet_NaN()};
  Point2 endpoint{};
  Point2 depth_endpoint{};
  Point3 lidar_direction{};
  Point3 body_frd_direction{};
  Point3 ned_direction{};
};

[[nodiscard]] std::optional<AttitudeEuler>
quaternionToEuler(const std::array<float, 4>& quaternion) noexcept;

[[nodiscard]] bool lidarRawRangeUsable(float raw_range, double range_min_m) noexcept;

[[nodiscard]] bool lidarRangeIsHit(float raw_range, double scan_range_max_m,
                                   double range_min_m,
                                   double range_hit_epsilon_m) noexcept;

[[nodiscard]] LidarBeamProjection
projectLidarBeam(const LidarProjectionPose& pose, const LidarProjectionConfig& config,
                 double scan_range_min_m, double scan_range_max_m, double angle_min_rad,
                 double angle_increment_rad, std::size_t beam_index, float raw_range,
                 double sensor_hit_depth_m) noexcept;

} // namespace drone_city_nav
