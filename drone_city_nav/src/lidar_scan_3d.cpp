#include "drone_city_nav/lidar_scan_3d.hpp"

#include <cmath>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] double pointNorm(const Point3& point) noexcept {
  return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

[[nodiscard]] double interpolatedAngle(const double minimum, const double maximum,
                                       const std::size_t index,
                                       const std::size_t sample_count) noexcept {
  if (sample_count <= 1U) {
    return 0.5 * (minimum + maximum);
  }
  return std::lerp(minimum, maximum,
                   static_cast<double>(index) / static_cast<double>(sample_count - 1U));
}

} // namespace

bool organizedLidarScan3DConfigIsValid(
    const OrganizedLidarScan3DConfig& config) noexcept {
  return config.horizontal_samples > 0U && config.vertical_samples > 0U &&
         std::isfinite(config.horizontal_min_angle_rad) &&
         std::isfinite(config.horizontal_max_angle_rad) &&
         config.horizontal_min_angle_rad < config.horizontal_max_angle_rad &&
         std::isfinite(config.vertical_min_angle_rad) &&
         std::isfinite(config.vertical_max_angle_rad) &&
         config.vertical_min_angle_rad < config.vertical_max_angle_rad &&
         std::isfinite(config.minimum_range_m) && config.minimum_range_m >= 0.0 &&
         std::isfinite(config.maximum_range_m) &&
         config.maximum_range_m > config.minimum_range_m &&
         std::isfinite(config.hit_epsilon_m) && config.hit_epsilon_m >= 0.0 &&
         config.hit_epsilon_m < config.maximum_range_m - config.minimum_range_m;
}

OrganizedLidarScan3DResult
decodeOrganizedLidarScan3D(const std::span<const Point3> returns_lidar_flu,
                           const OrganizedLidarScan3DConfig& config) {
  OrganizedLidarScan3DResult result;
  if (!organizedLidarScan3DConfigIsValid(config)) {
    return result;
  }
  const std::size_t expected_samples =
      config.horizontal_samples * config.vertical_samples;
  result.organized_dimensions_match = returns_lidar_flu.size() == expected_samples;
  if (!result.organized_dimensions_match) {
    return result;
  }
  result.beams.reserve(expected_samples);
  for (std::size_t row = 0U; row < config.vertical_samples; ++row) {
    const double vertical_angle =
        interpolatedAngle(config.vertical_min_angle_rad, config.vertical_max_angle_rad,
                          row, config.vertical_samples);
    const double vertical_cosine = std::cos(vertical_angle);
    for (std::size_t column = 0U; column < config.horizontal_samples; ++column) {
      const double horizontal_angle = interpolatedAngle(
          config.horizontal_min_angle_rad, config.horizontal_max_angle_rad, column,
          config.horizontal_samples);
      const Vec3 direction{vertical_cosine * std::cos(horizontal_angle),
                           vertical_cosine * std::sin(horizontal_angle),
                           std::sin(vertical_angle)};
      const Point3& point = returns_lidar_flu[row * config.horizontal_samples + column];
      const double range_m =
          finitePoint(point) ? pointNorm(point) : config.maximum_range_m;
      if (!std::isfinite(range_m) || range_m < config.minimum_range_m) {
        result.beams.push_back(LidarBeamSample3D{.direction_lidar_flu = direction,
                                                 .range_m = range_m,
                                                 .hit = false,
                                                 .valid = false});
        ++result.invalid_beams;
        continue;
      }
      const bool hit = range_m < config.maximum_range_m - config.hit_epsilon_m;
      result.beams.push_back(
          LidarBeamSample3D{.direction_lidar_flu = direction,
                            .range_m = hit ? range_m : config.maximum_range_m,
                            .hit = hit,
                            .valid = true});
      result.hit_beams += hit ? 1U : 0U;
      result.miss_beams += hit ? 0U : 1U;
    }
  }
  return result;
}

} // namespace drone_city_nav
