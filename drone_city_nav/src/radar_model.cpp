#include "drone_city_nav/radar_model.hpp"

#include <cmath>
#include <numbers>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finite(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finite(const Vec3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] bool validRadarState(const TimedVehicleState& state) noexcept {
  return state.position_valid && state.velocity_valid && state.heading_valid &&
         state.stamp_ns > 0 && finite(state.position) && finite(state.velocity) &&
         std::isfinite(state.heading_rad);
}

[[nodiscard]] double normalizedAngle(const double angle_rad) noexcept {
  return std::remainder(angle_rad, 2.0 * std::numbers::pi);
}

[[nodiscard]] double dot(const Vec3& first, const Vec3& second) noexcept {
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

} // namespace

std::optional<RadarDetectionSample>
simulateIdealRadarDetection(const TimedVehicleState& radar,
                            const TimedVehicleState& target,
                            const std::uint64_t detection_id) noexcept {
  if (!validRadarState(radar) || !target.position_valid || !target.velocity_valid ||
      target.stamp_ns <= 0 || !finite(target.position) || !finite(target.velocity)) {
    return std::nullopt;
  }
  const Vec3 offset{target.position.x - radar.position.x,
                    target.position.y - radar.position.y,
                    target.position.z - radar.position.z};
  const double horizontal_range = std::hypot(offset.x, offset.y);
  const double range = std::hypot(horizontal_range, offset.z);
  if (!(range > 1.0e-6) || !std::isfinite(range)) {
    return std::nullopt;
  }
  const Vec3 line_of_sight{offset.x / range, offset.y / range, offset.z / range};
  const Vec3 relative_velocity{target.velocity.x - radar.velocity.x,
                               target.velocity.y - radar.velocity.y,
                               target.velocity.z - radar.velocity.z};
  return RadarDetectionSample{
      .detection_id = detection_id,
      .range_m = range,
      .azimuth_rad =
          normalizedAngle(std::atan2(offset.y, offset.x) - radar.heading_rad),
      .elevation_rad = std::atan2(offset.z, horizontal_range),
      .radial_velocity_mps = dot(relative_velocity, line_of_sight),
  };
}

std::optional<Vec3>
radarDetectionLineOfSightMap(const TimedVehicleState& radar,
                             const RadarDetectionSample& detection) noexcept {
  if (!validRadarState(radar) || !(detection.range_m > 0.0) ||
      !std::isfinite(detection.range_m) || !std::isfinite(detection.azimuth_rad) ||
      !std::isfinite(detection.elevation_rad) ||
      !std::isfinite(detection.radial_velocity_mps)) {
    return std::nullopt;
  }
  const double horizontal = std::cos(detection.elevation_rad);
  const double map_azimuth = radar.heading_rad + detection.azimuth_rad;
  const Vec3 direction{horizontal * std::cos(map_azimuth),
                       horizontal * std::sin(map_azimuth),
                       std::sin(detection.elevation_rad)};
  if (!finite(direction)) {
    return std::nullopt;
  }
  return direction;
}

std::optional<Point3>
radarDetectionPositionMap(const TimedVehicleState& radar,
                          const RadarDetectionSample& detection) noexcept {
  const std::optional<Vec3> direction = radarDetectionLineOfSightMap(radar, detection);
  if (!direction.has_value()) {
    return std::nullopt;
  }
  return Point3{radar.position.x + direction->x * detection.range_m,
                radar.position.y + direction->y * detection.range_m,
                radar.position.z + direction->z * detection.range_m};
}

} // namespace drone_city_nav
