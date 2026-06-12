#include "drone_city_nav/navigation_pose.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace drone_city_nav {
namespace {

constexpr double kEarthRadiusM = 6'378'137.0;

[[nodiscard]] double degreesToRadians(const double degrees) noexcept {
  return degrees * std::numbers::pi / 180.0;
}

[[nodiscard]] bool finiteFix(const GpsFixSample& fix) noexcept {
  return std::isfinite(fix.latitude_deg) && std::isfinite(fix.longitude_deg) &&
         std::isfinite(fix.altitude_m);
}

[[nodiscard]] bool freshStamp(const std::int64_t stamp_ns, const std::int64_t now_ns,
                              const std::int64_t max_staleness_ns) noexcept {
  if (max_staleness_ns <= 0) {
    return true;
  }
  if (stamp_ns <= 0 || now_ns <= 0) {
    return false;
  }
  if (stamp_ns > now_ns) {
    return true;
  }
  return now_ns - stamp_ns <= max_staleness_ns;
}

} // namespace

double normalizeYaw(const double yaw_rad) noexcept {
  if (!std::isfinite(yaw_rad)) {
    return yaw_rad;
  }

  double normalized = std::remainder(yaw_rad, 2.0 * std::numbers::pi);
  if (normalized <= -std::numbers::pi) {
    normalized += 2.0 * std::numbers::pi;
  }
  if (normalized > std::numbers::pi) {
    normalized -= 2.0 * std::numbers::pi;
  }
  return normalized;
}

bool validGpsFix(const GpsFixSample& fix, const GpsCompassConfig& config,
                 const std::int64_t now_ns) noexcept {
  if (fix.status < config.min_fix_status || !finiteFix(fix) ||
      !freshStamp(fix.stamp_ns, now_ns, config.max_gps_staleness_ns)) {
    return false;
  }

  if (!fix.horizontal_variance_known) {
    return !config.require_known_gps_covariance;
  }

  return std::isfinite(fix.horizontal_variance_m2) &&
         fix.horizontal_variance_m2 <= config.max_gps_horizontal_variance_m2;
}

Point2 projectWgs84ToLocal(const GpsFixSample& fix,
                           const GeoReference& origin) noexcept {
  const double delta_lat_rad = degreesToRadians(fix.latitude_deg - origin.latitude_deg);
  const double delta_lon_rad =
      degreesToRadians(fix.longitude_deg - origin.longitude_deg);
  const double origin_lat_rad = degreesToRadians(origin.latitude_deg);
  return Point2{delta_lat_rad * kEarthRadiusM,
                delta_lon_rad * kEarthRadiusM * std::cos(origin_lat_rad)};
}

std::optional<double> yawFromQuaternion(const QuaternionSample& quaternion) noexcept {
  const double norm_sq = quaternion.w * quaternion.w + quaternion.x * quaternion.x +
                         quaternion.y * quaternion.y + quaternion.z * quaternion.z;
  if (!std::isfinite(norm_sq) || norm_sq <= 0.0) {
    return std::nullopt;
  }

  const double inv_norm = 1.0 / std::sqrt(norm_sq);
  const double w = quaternion.w * inv_norm;
  const double x = quaternion.x * inv_norm;
  const double y = quaternion.y * inv_norm;
  const double z = quaternion.z * inv_norm;
  const double sin_yaw = 2.0 * (w * z + x * y);
  const double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
  const double yaw = std::atan2(sin_yaw, cos_yaw);
  if (!std::isfinite(yaw)) {
    return std::nullopt;
  }
  return normalizeYaw(yaw);
}

std::optional<NavigationPose2D> makeNavigationPoseFromGpsCompass(
    const GpsFixSample& fix, const double compass_yaw_rad, const std::int64_t now_ns,
    const GpsCompassConfig& config, GeoReference& origin) noexcept {
  if (!validGpsFix(fix, config, now_ns) || !std::isfinite(compass_yaw_rad)) {
    return std::nullopt;
  }

  if (!origin.initialized) {
    if (!config.auto_initialize_origin) {
      origin = GeoReference{config.origin_latitude_deg, config.origin_longitude_deg,
                            config.origin_altitude_m, true};
    } else {
      origin = GeoReference{fix.latitude_deg, fix.longitude_deg, fix.altitude_m, true};
    }
  }

  const Point2 local_position = projectWgs84ToLocal(fix, origin);
  const double yaw_rad =
      normalizeYaw(compass_yaw_rad + config.magnetic_declination_rad +
                   config.compass_to_body_yaw_offset_rad + config.yaw_offset_rad);
  if (!std::isfinite(local_position.x) || !std::isfinite(local_position.y) ||
      !std::isfinite(yaw_rad)) {
    return std::nullopt;
  }

  return NavigationPose2D{Pose2{local_position, yaw_rad},
                          fix.altitude_m - origin.altitude_m,
                          fix.stamp_ns,
                          true,
                          true,
                          true};
}

} // namespace drone_city_nav
