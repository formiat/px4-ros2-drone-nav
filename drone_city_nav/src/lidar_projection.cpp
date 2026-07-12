#include "drone_city_nav/lidar_projection.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] bool rangeIsPositiveInfinity(const float value) noexcept {
  return std::isinf(value) && value > 0.0F;
}

[[nodiscard]] bool altitudeInRange(const double altitude_m,
                                   const LidarProjectionConfig& config) noexcept {
  return std::isfinite(altitude_m) && altitude_m >= config.min_projected_altitude_m &&
         altitude_m <= config.max_projected_altitude_m;
}

[[nodiscard]] double squaredNorm(const Point3& vector) noexcept {
  return vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
}

[[nodiscard]] Point3 normalizeOrZero(const Point3& vector) noexcept {
  const double norm_sq = squaredNorm(vector);
  if (!std::isfinite(norm_sq) || norm_sq <= 0.0) {
    return Point3{};
  }
  const double inv_norm = 1.0 / std::sqrt(norm_sq);
  return Point3{vector.x * inv_norm, vector.y * inv_norm, vector.z * inv_norm};
}

[[nodiscard]] Point3 scanDirectionInLidarFluFrame(const double angle_rad) noexcept {
  return Point3{std::cos(angle_rad), std::sin(angle_rad), 0.0};
}

[[nodiscard]] Point3 rotateRzyx(const Point3& vector, const double roll_rad,
                                const double pitch_rad, const double yaw_rad) noexcept {
  const double cr = std::cos(roll_rad);
  const double sr = std::sin(roll_rad);
  const double cp = std::cos(pitch_rad);
  const double sp = std::sin(pitch_rad);
  const double cy = std::cos(yaw_rad);
  const double sy = std::sin(yaw_rad);

  return Point3{cp * cy * vector.x + (sr * sp * cy - cr * sy) * vector.y +
                    (cr * sp * cy + sr * sy) * vector.z,
                cp * sy * vector.x + (sr * sp * sy + cr * cy) * vector.y +
                    (cr * sp * sy - sr * cy) * vector.z,
                -sp * vector.x + sr * cp * vector.y + cr * cp * vector.z};
}

[[nodiscard]] Point3 lidarFluToBodyFrd(const Point3& vector) noexcept {
  // ROS optical/lidar scans are handled in FLU, while PX4 attitude is body FRD.
  // The NED projection below expects body FRD before applying roll/pitch/yaw.
  return Point3{vector.x, -vector.y, -vector.z};
}

[[nodiscard]] Point3
mountedLidarDirection(const Point3& lidar_direction,
                      const LidarProjectionConfig& config) noexcept {
  return rotateRzyx(lidar_direction, config.lidar_mount_roll_rad,
                    config.lidar_mount_pitch_rad, config.lidar_mount_yaw_rad);
}

[[nodiscard]] Point3
projectDirectionToNed(const Point3& lidar_direction, const LidarProjectionPose& pose,
                      const LidarProjectionConfig& config) noexcept {
  const Point3 body_frd_direction =
      lidarFluToBodyFrd(mountedLidarDirection(lidar_direction, config));
  const double roll_rad =
      config.compensate_attitude && pose.attitude_valid ? pose.roll_rad : 0.0;
  const double pitch_rad =
      config.compensate_attitude && pose.attitude_valid ? pose.pitch_rad : 0.0;
  return normalizeOrZero(rotateRzyx(body_frd_direction, roll_rad, pitch_rad,
                                    pose.yaw_rad + config.scan_yaw_offset_rad));
}

[[nodiscard]] bool validProjectionInputs(const LidarProjectionPose& pose,
                                         const LidarProjectionConfig& config,
                                         const double scan_range_min_m,
                                         const double scan_range_max_m,
                                         const double angle_min_rad,
                                         const double angle_increment_rad) noexcept {
  return finite2D(pose.position) && std::isfinite(pose.yaw_rad) &&
         config.max_lidar_range_m > 0.0 && std::isfinite(config.max_lidar_range_m) &&
         std::isfinite(config.range_hit_epsilon_m) &&
         std::isfinite(config.scan_yaw_offset_rad) &&
         std::isfinite(config.lidar_z_offset_m) &&
         std::isfinite(config.min_projected_altitude_m) &&
         !std::isnan(config.max_projected_altitude_m) &&
         std::isfinite(config.lidar_mount_roll_rad) &&
         std::isfinite(config.lidar_mount_pitch_rad) &&
         std::isfinite(config.lidar_mount_yaw_rad) &&
         config.min_projected_altitude_m <= config.max_projected_altitude_m &&
         std::isfinite(scan_range_min_m) && std::isfinite(scan_range_max_m) &&
         scan_range_max_m > scan_range_min_m && std::isfinite(angle_min_rad) &&
         std::isfinite(angle_increment_rad) && angle_increment_rad != 0.0;
}

} // namespace

std::optional<AttitudeEuler>
quaternionToEuler(const std::array<float, 4>& quaternion) noexcept {
  const double w = static_cast<double>(quaternion[0]);
  const double x = static_cast<double>(quaternion[1]);
  const double y = static_cast<double>(quaternion[2]);
  const double z = static_cast<double>(quaternion[3]);
  const double norm_sq = w * w + x * x + y * y + z * z;
  if (!std::isfinite(norm_sq) || norm_sq <= 0.0) {
    return std::nullopt;
  }

  const double inv_norm = 1.0 / std::sqrt(norm_sq);
  const double nw = w * inv_norm;
  const double nx = x * inv_norm;
  const double ny = y * inv_norm;
  const double nz = z * inv_norm;

  const double sin_roll = 2.0 * (nw * nx + ny * nz);
  const double cos_roll = 1.0 - 2.0 * (nx * nx + ny * ny);
  const double roll = std::atan2(sin_roll, cos_roll);

  const double sin_pitch = std::clamp(2.0 * (nw * ny - nz * nx), -1.0, 1.0);
  const double pitch = std::asin(sin_pitch);

  const double sin_yaw = 2.0 * (nw * nz + nx * ny);
  const double cos_yaw = 1.0 - 2.0 * (ny * ny + nz * nz);
  const double yaw = std::atan2(sin_yaw, cos_yaw);

  if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw)) {
    return std::nullopt;
  }
  return AttitudeEuler{roll, pitch, yaw};
}

bool lidarRawRangeUsable(const float raw_range, const double range_min_m) noexcept {
  const bool finite_range = std::isfinite(raw_range);
  return range_min_m >= 0.0 &&
         ((finite_range && static_cast<double>(raw_range) >= range_min_m) ||
          rangeIsPositiveInfinity(raw_range));
}

bool lidarRangeIsHit(const float raw_range, const double scan_range_max_m,
                     const double range_min_m,
                     const double range_hit_epsilon_m) noexcept {
  return std::isfinite(raw_range) && static_cast<double>(raw_range) >= range_min_m &&
         static_cast<double>(raw_range) < scan_range_max_m - range_hit_epsilon_m;
}

LidarBeamProjection
projectLidarBeam(const LidarProjectionPose& pose, const LidarProjectionConfig& config,
                 const double scan_range_min_m, const double scan_range_max_m,
                 const double angle_min_rad, const double angle_increment_rad,
                 const std::size_t beam_index, const float raw_range) noexcept {
  LidarBeamProjection projection{};
  if (!validProjectionInputs(pose, config, scan_range_min_m, scan_range_max_m,
                             angle_min_rad, angle_increment_rad)) {
    projection.status = LidarBeamProjectionStatus::kInvalidScan;
    return projection;
  }
  const double scan_range_max = std::min(scan_range_max_m, config.max_lidar_range_m);
  if (!(scan_range_max > scan_range_min_m)) {
    projection.status = LidarBeamProjectionStatus::kInvalidScan;
    return projection;
  }

  const double beam_angle_rad =
      angle_min_rad + static_cast<double>(beam_index) * angle_increment_rad;
  projection.lidar_direction = scanDirectionInLidarFluFrame(beam_angle_rad);
  projection.body_frd_direction = normalizeOrZero(
      lidarFluToBodyFrd(mountedLidarDirection(projection.lidar_direction, config)));
  projection.ned_direction =
      projectDirectionToNed(projection.lidar_direction, pose, config);

  if (!lidarRawRangeUsable(raw_range, scan_range_min_m)) {
    projection.status = LidarBeamProjectionStatus::kInvalidRange;
    return projection;
  }

  projection.hit = lidarRangeIsHit(raw_range, scan_range_max, scan_range_min_m,
                                   config.range_hit_epsilon_m);
  projection.used_range_m =
      projection.hit ? static_cast<double>(raw_range) : scan_range_max;

  const Point3 world_direction = projection.ned_direction;

  projection.endpoint =
      Point2{pose.position.x + projection.used_range_m * world_direction.x,
             pose.position.y + projection.used_range_m * world_direction.y};

  if (pose.altitude_valid && std::isfinite(pose.altitude_m + config.lidar_z_offset_m)) {
    const double origin_altitude_m = pose.altitude_m + config.lidar_z_offset_m;
    projection.ray_origin_map_m =
        Point3{pose.position.x, pose.position.y, origin_altitude_m};
    projection.ray_direction_map =
        Point3{world_direction.x, world_direction.y, -world_direction.z};
    projection.endpoint_altitude_m =
        origin_altitude_m - projection.used_range_m * world_direction.z;
    projection.endpoint_map_m = Point3{projection.endpoint.x, projection.endpoint.y,
                                       projection.endpoint_altitude_m};
    if (!altitudeInRange(projection.endpoint_altitude_m, config)) {
      projection.status = LidarBeamProjectionStatus::kAltitudeRejected;
      return projection;
    }
  }

  projection.status = LidarBeamProjectionStatus::kAccepted;
  return projection;
}

} // namespace drone_city_nav
