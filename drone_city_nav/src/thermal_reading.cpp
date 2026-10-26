#include "drone_city_nav/thermal_reading.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace drone_city_nav {
namespace {

[[nodiscard]] Point3 unit(const double x, const double y, const double z) noexcept {
  const double norm = std::sqrt(x * x + y * y + z * z);
  return norm > 0.0 ? Point3{x / norm, y / norm, z / norm} : Point3{1.0, 0.0, 0.0};
}

[[nodiscard]] double dot(const Point3& a, const Point3& b) noexcept {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

[[nodiscard]] double norm(const Point3& point) noexcept {
  return std::sqrt(dot(point, point));
}

[[nodiscard]] double angleBetween(const Point3& a, const Point3& b) noexcept {
  return std::acos(
      std::clamp(dot(a, b) / std::max(norm(a) * norm(b), 1e-12), -1.0, 1.0));
}

} // namespace

ThermalReading readThermalFrame(const std::span<const std::uint16_t> kelvin_hundredths,
                                const std::size_t width, const std::size_t height,
                                const ThermalReadingConfig& config) {
  ThermalReading reading;
  const std::size_t pixels = width * height;
  if (pixels == 0U || kelvin_hundredths.size() < pixels) {
    return reading;
  }
  std::vector<std::uint16_t> sorted(kelvin_hundredths.begin(),
                                    kelvin_hundredths.begin() +
                                        static_cast<std::ptrdiff_t>(pixels));
  std::nth_element(sorted.begin(),
                   sorted.begin() + static_cast<std::ptrdiff_t>(pixels / 2U),
                   sorted.end());
  const double threshold =
      static_cast<double>(sorted[pixels / 2U]) + 100.0 * config.above_median_k;
  const double focal_px =
      (static_cast<double>(width) / 2.0) / std::tan(config.horizontal_fov_rad / 2.0);
  const double centre_u = (static_cast<double>(width) - 1.0) / 2.0;
  const double centre_v = (static_cast<double>(height) - 1.0) / 2.0;
  std::size_t count{0U};
  double sum_u{0.0};
  double sum_v{0.0};
  for (std::size_t index = 0U; index < pixels; ++index) {
    if (static_cast<double>(kelvin_hundredths[index]) >= threshold) {
      ++count;
      sum_u += static_cast<double>(index % width);
      sum_v += static_cast<double>(index / width);
    }
  }
  reading.hot_share = static_cast<double>(count) / static_cast<double>(pixels);
  if (count < std::max<std::size_t>(config.least_hot_pixels, 1U)) {
    return reading;
  }
  reading.hot_pixels = count;
  const double mean_u = sum_u / static_cast<double>(count);
  const double mean_v = sum_v / static_cast<double>(count);
  // Forward-left-up: a column to the right is a negative y, a row down a negative z.
  reading.direction =
      unit(1.0, -(mean_u - centre_u) / focal_px, -(mean_v - centre_v) / focal_px);
  double widest{0.0};
  for (std::size_t index = 0U; index < pixels; ++index) {
    if (static_cast<double>(kelvin_hundredths[index]) >= threshold) {
      const Point3 ray =
          unit(1.0, -(static_cast<double>(index % width) - centre_u) / focal_px,
               -(static_cast<double>(index / width) - centre_v) / focal_px);
      widest = std::max(widest, angleBetween(ray, reading.direction));
    }
  }
  reading.half_angle_rad = widest + std::atan(0.5 / focal_px);
  return reading;
}

double heatSourceRangeM(const ThermalReading& reading,
                        const std::span<const StereoDepthReturn> returns,
                        const double cone_margin_rad,
                        const double conservative_range_m) {
  std::vector<double> ranges;
  for (const StereoDepthReturn& ray : returns) {
    if (ray.hit && !ray.unobservable &&
        angleBetween(ray.point, reading.direction) <=
            reading.half_angle_rad + cone_margin_rad) {
      ranges.push_back(norm(ray.point));
    }
  }
  if (ranges.empty()) {
    return conservative_range_m;
  }
  std::nth_element(ranges.begin(),
                   ranges.begin() + static_cast<std::ptrdiff_t>(ranges.size() / 2U),
                   ranges.end());
  return ranges[ranges.size() / 2U];
}

std::vector<Point3> heatStandoffCap(const ThermalReading& reading, const double range_m,
                                    const double standoff_m, const double spacing_m,
                                    const double nearest_m) {
  std::vector<Point3> cap;
  if (!(standoff_m > 0.0) || !(spacing_m > 0.0) || !(range_m >= 0.0)) {
    return cap;
  }
  const Point3 centre{range_m * reading.direction.x, range_m * reading.direction.y,
                      range_m * reading.direction.z};
  // A Fibonacci lattice over the sphere, one point a `spacing_m` square.
  const auto count = static_cast<std::size_t>(std::ceil(
      4.0 * std::numbers::pi * standoff_m * standoff_m / (spacing_m * spacing_m)));
  const double golden = std::numbers::pi * (3.0 - std::sqrt(5.0));
  for (std::size_t index = 0U; index < count; ++index) {
    const double z =
        1.0 - 2.0 * (static_cast<double>(index) + 0.5) / static_cast<double>(count);
    const double radius = std::sqrt(std::max(0.0, 1.0 - z * z));
    const double angle = golden * static_cast<double>(index);
    const Point3 normal{radius * std::cos(angle), radius * std::sin(angle), z};
    if (dot(normal, reading.direction) >= 0.0) {
      continue;
    }
    const Point3 point{centre.x + standoff_m * normal.x,
                       centre.y + standoff_m * normal.y,
                       centre.z + standoff_m * normal.z};
    if (norm(point) < nearest_m) {
      continue;
    }
    cap.push_back(point);
  }
  return cap;
}

bool withinHeatStandoff(const Point3& point, const ThermalReading& reading,
                        const double range_m, const double standoff_m) noexcept {
  const Point3 offset{point.x - range_m * reading.direction.x,
                      point.y - range_m * reading.direction.y,
                      point.z - range_m * reading.direction.z};
  return norm(offset) < standoff_m;
}

} // namespace drone_city_nav
