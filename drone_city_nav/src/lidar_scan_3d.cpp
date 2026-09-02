#include "drone_city_nav/lidar_scan_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

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

bool lidarSurfaceInterpolation3DConfigIsValid(
    const LidarSurfaceInterpolation3DConfig& config) noexcept {
  return std::isfinite(config.maximum_incidence_rad) &&
         config.maximum_incidence_rad > 0.0 &&
         config.maximum_incidence_rad < 0.5 * std::numbers::pi &&
         std::isfinite(config.maximum_row_incidence_rad) &&
         config.maximum_row_incidence_rad > 0.0 &&
         config.maximum_row_incidence_rad < 0.5 * std::numbers::pi &&
         std::isfinite(config.minimum_vertical_fraction) &&
         config.minimum_vertical_fraction >= 0.0 &&
         config.minimum_vertical_fraction <= 1.0 &&
         std::isfinite(config.sample_spacing_m) && config.sample_spacing_m > 0.0;
}

namespace {

[[nodiscard]] Point3 beamEndpoint(const LidarBeamSample3D& beam) noexcept {
  return Point3{beam.direction_lidar_flu.x * beam.range_m,
                beam.direction_lidar_flu.y * beam.range_m,
                beam.direction_lidar_flu.z * beam.range_m};
}

// Joins two adjacent returns when their range step fits one surface viewed
// within the incidence limit, appending the samples strictly between them.
void interpolateBetweenBeams(
    const LidarBeamSample3D& first, const LidarBeamSample3D& second,
    const double beam_spacing_rad, const OrganizedLidarScan3DConfig& scan_config,
    const LidarSurfaceInterpolation3DConfig& config, const double incidence_tangent,
    const double minimum_vertical_fraction, std::vector<LidarBeamSample3D>& samples) {
  if (!first.valid || !second.valid || !first.hit || !second.hit ||
      !(beam_spacing_rad > 0.0)) {
    return;
  }
  const double farther_range_m = std::max(first.range_m, second.range_m);
  const double surface_range_step_m =
      incidence_tangent * farther_range_m * beam_spacing_rad;
  if (std::abs(first.range_m - second.range_m) > surface_range_step_m) {
    return;
  }
  const Point3 start = beamEndpoint(first);
  const Point3 end = beamEndpoint(second);
  const Point3 delta{end.x - start.x, end.y - start.y, end.z - start.z};
  const double gap_m = pointNorm(delta);
  if (!(gap_m > config.sample_spacing_m) ||
      std::abs(delta.z) < minimum_vertical_fraction * gap_m) {
    return;
  }
  const std::size_t segments =
      static_cast<std::size_t>(std::ceil(gap_m / config.sample_spacing_m));
  for (std::size_t index = 1U; index < segments; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(segments);
    const Point3 point{start.x + ratio * delta.x, start.y + ratio * delta.y,
                       start.z + ratio * delta.z};
    const double range_m = pointNorm(point);
    if (!(range_m >= scan_config.minimum_range_m) ||
        range_m >= scan_config.maximum_range_m) {
      continue;
    }
    samples.push_back(LidarBeamSample3D{
        .direction_lidar_flu = {point.x / range_m, point.y / range_m,
                                point.z / range_m},
        .range_m = range_m,
        .hit = true,
        .valid = true,
        .interpolated = true,
    });
  }
}

} // namespace

std::vector<LidarBeamSample3D>
interpolateOrganizedLidarSurfaces3D(const std::span<const LidarBeamSample3D> beams,
                                    const OrganizedLidarScan3DConfig& scan_config,
                                    const LidarSurfaceInterpolation3DConfig& config) {
  std::vector<LidarBeamSample3D> samples;
  if (!config.enabled || !organizedLidarScan3DConfigIsValid(scan_config) ||
      !lidarSurfaceInterpolation3DConfigIsValid(config) ||
      beams.size() != scan_config.horizontal_samples * scan_config.vertical_samples) {
    return samples;
  }
  const std::size_t columns = scan_config.horizontal_samples;
  const std::size_t rows = scan_config.vertical_samples;
  const double vertical_spacing_rad =
      rows > 1U
          ? (scan_config.vertical_max_angle_rad - scan_config.vertical_min_angle_rad) /
                static_cast<double>(rows - 1U)
          : 0.0;
  const double horizontal_spacing_rad = columns > 1U
                                            ? (scan_config.horizontal_max_angle_rad -
                                               scan_config.horizontal_min_angle_rad) /
                                                  static_cast<double>(columns - 1U)
                                            : 0.0;
  const double incidence_tangent = std::tan(config.maximum_incidence_rad);
  const double row_incidence_tangent = std::tan(config.maximum_row_incidence_rad);
  for (std::size_t row = 0U; row < rows; ++row) {
    for (std::size_t column = 0U; column < columns; ++column) {
      const LidarBeamSample3D& beam = beams[row * columns + column];
      if (row + 1U < rows) {
        interpolateBetweenBeams(beam, beams[(row + 1U) * columns + column],
                                vertical_spacing_rad, scan_config, config,
                                incidence_tangent, config.minimum_vertical_fraction,
                                samples);
      }
      if (column + 1U < columns) {
        interpolateBetweenBeams(beam, beams[row * columns + column + 1U],
                                horizontal_spacing_rad, scan_config, config,
                                row_incidence_tangent, 0.0, samples);
      }
    }
  }
  return samples;
}

} // namespace drone_city_nav
