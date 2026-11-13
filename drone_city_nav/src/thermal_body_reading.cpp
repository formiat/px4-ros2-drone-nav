#include "drone_city_nav/thermal_reading.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] Point3 unit(const double x, const double y, const double z) noexcept {
  const double norm = std::sqrt(x * x + y * y + z * z);
  return norm > 0.0 ? Point3{x / norm, y / norm, z / norm} : Point3{1.0, 0.0, 0.0};
}

} // namespace

std::vector<ThermalBody>
readThermalBodies(const std::span<const std::uint16_t> kelvin_hundredths,
                  const std::size_t width, const std::size_t height,
                  const ThermalReadingConfig& source, const ThermalBodyConfig& config) {
  std::vector<ThermalBody> bodies;
  const std::size_t pixels = width * height;
  if (pixels == 0U || kelvin_hundredths.size() < pixels) {
    return bodies;
  }
  std::vector<std::uint16_t> sorted(kelvin_hundredths.begin(),
                                    kelvin_hundredths.begin() +
                                        static_cast<std::ptrdiff_t>(pixels));
  std::nth_element(sorted.begin(),
                   sorted.begin() + static_cast<std::ptrdiff_t>(pixels / 2U),
                   sorted.end());
  const double median = static_cast<double>(sorted[pixels / 2U]);
  const double warm = median + 100.0 * config.above_median_k;
  const double hot = median + 100.0 * source.above_median_k;
  const double same = 100.0 * config.same_surface_k;
  const auto warmAt = [&](const std::size_t index) {
    const double value = static_cast<double>(kelvin_hundredths[index]);
    return value >= warm && value < hot;
  };
  const double focal_px =
      (static_cast<double>(width) / 2.0) / std::tan(source.horizontal_fov_rad / 2.0);
  const double centre_u = (static_cast<double>(width) - 1.0) / 2.0;
  const double centre_v = (static_cast<double>(height) - 1.0) / 2.0;
  std::vector<char> taken(pixels, 0);
  std::vector<std::size_t> region;
  for (std::size_t seed = 0U; seed < pixels; ++seed) {
    if (taken[seed] != 0 || !warmAt(seed)) {
      continue;
    }
    // One surface: warm pixels joined side to side that read alike.
    region.assign(1U, seed);
    taken[seed] = 1;
    for (std::size_t next = 0U; next < region.size(); ++next) {
      const std::size_t index = region[next];
      const std::size_t u = index % width;
      const std::size_t v = index / width;
      const double value = static_cast<double>(kelvin_hundredths[index]);
      const auto join = [&](const std::size_t neighbour) {
        if (taken[neighbour] == 0 && warmAt(neighbour) &&
            std::abs(static_cast<double>(kelvin_hundredths[neighbour]) - value) <=
                same) {
          taken[neighbour] = 1;
          region.push_back(neighbour);
        }
      };
      if (u > 0U) {
        join(index - 1U);
      }
      if (u + 1U < width) {
        join(index + 1U);
      }
      if (v > 0U) {
        join(index - width);
      }
      if (v + 1U < height) {
        join(index + width);
      }
    }
    if (region.size() < std::max<std::size_t>(config.least_warm_pixels, 1U)) {
      continue;
    }
    std::size_t least_u{width};
    std::size_t most_u{0U};
    std::size_t least_v{height};
    std::size_t most_v{0U};
    double sum_u{0.0};
    double sum_v{0.0};
    for (const std::size_t index : region) {
      const std::size_t u = index % width;
      const std::size_t v = index / width;
      least_u = std::min(least_u, u);
      most_u = std::max(most_u, u);
      least_v = std::min(least_v, v);
      most_v = std::max(most_v, v);
      sum_u += static_cast<double>(u);
      sum_v += static_cast<double>(v);
    }
    const double count = static_cast<double>(region.size());
    ThermalBody body;
    body.warm_pixels = region.size();
    // Forward-left-up: a column to the right is a negative y, a row down a negative z.
    body.direction = unit(1.0, -(sum_u / count - centre_u) / focal_px,
                          -(sum_v / count - centre_v) / focal_px);
    const double columns = static_cast<double>(most_u - least_u + 1U);
    const double rows = static_cast<double>(most_v - least_v + 1U);
    body.width_rad = 2.0 * std::atan(columns / (2.0 * focal_px));
    body.height_rad = 2.0 * std::atan(rows / (2.0 * focal_px));
    body.half_angle_rad = std::atan(std::hypot(columns, rows) / (2.0 * focal_px)) +
                          std::atan(0.5 / focal_px);
    const bool at_edge =
        least_u == 0U || least_v == 0U || most_u + 1U == width || most_v + 1U == height;
    body.range_by_size_m =
        at_edge ? 0.0 : std::sqrt(config.body_pixel_metres_squared / count);
    bodies.push_back(body);
  }
  std::ranges::sort(bodies, [](const ThermalBody& a, const ThermalBody& b) {
    return a.warm_pixels > b.warm_pixels;
  });
  if (bodies.size() > config.most_bodies) {
    bodies.resize(config.most_bodies);
  }
  return bodies;
}

bool inRegion(const Point3& point, const Point3& direction, const double width_rad,
              const double height_rad, const double margin_rad) noexcept {
  const auto azimuth = [](const Point3& p) { return std::atan2(p.y, p.x); };
  const auto elevation = [](const Point3& p) {
    return std::atan2(p.z, std::hypot(p.x, p.y));
  };
  if (!(point.x > 0.0)) {
    return false;
  }
  return std::abs(azimuth(point) - azimuth(direction)) <=
             width_rad / 2.0 + margin_rad &&
         std::abs(elevation(point) - elevation(direction)) <=
             height_rad / 2.0 + margin_rad;
}

std::vector<ThermalBodySighting>
sightBodies(const std::span<const ThermalBody> regions,
            const std::span<const StereoDepthReturn> returns, const double margin_rad,
            const ThermalBodyConfig& config) {
  std::vector<ThermalBodySighting> bodies;
  std::vector<double> ranges;
  for (const ThermalBody& region : regions) {
    ranges.clear();
    for (const StereoDepthReturn& ray : returns) {
      if (ray.hit && !ray.unobservable &&
          inRegion(ray.point, region.direction, region.width_rad, region.height_rad,
                   margin_rad)) {
        ranges.push_back(std::sqrt(ray.point.x * ray.point.x +
                                   ray.point.y * ray.point.y +
                                   ray.point.z * ray.point.z));
      }
    }
    double range_m = region.range_by_size_m;
    // The hits that put the body where its size does are its own: the rest of
    // the silhouette's frame is what stands behind it. Ranged by the median
    // of them all, a body 10 to 20 m off was put on the wall 3.5 to 10 m
    // behind it in one sighting of four, and that wall's returns were marked
    // a warm body's (r3479).
    const auto apart = [&](const double range) {
      return std::abs(range - region.range_by_size_m) >
             config.size_range_error_share * region.range_by_size_m;
    };
    if (region.range_by_size_m > 0.0 && !std::ranges::all_of(ranges, apart)) {
      std::erase_if(ranges, apart);
    }
    if (!ranges.empty()) {
      std::nth_element(ranges.begin(),
                       ranges.begin() + static_cast<std::ptrdiff_t>(ranges.size() / 2U),
                       ranges.end());
      range_m = ranges[ranges.size() / 2U];
    }
    if (bodySized(region, range_m, config)) {
      bodies.push_back(ThermalBodySighting{.direction = region.direction,
                                           .width_rad = region.width_rad,
                                           .height_rad = region.height_rad,
                                           .range_m = range_m});
    }
  }
  return bodies;
}

bool onBody(const Point3& point, const ThermalBodySighting& body,
            const double margin_rad, const double depth_m) noexcept {
  const double range =
      std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
  return std::abs(range - body.range_m) <= depth_m &&
         inRegion(point, body.direction, body.width_rad, body.height_rad, margin_rad);
}

bool bodySized(const ThermalBody& region, const double range_m,
               const ThermalBodyConfig& config) noexcept {
  if (!(range_m > 0.0) || !std::isfinite(range_m) || region.warm_pixels == 0U) {
    return false;
  }
  return 2.0 * range_m * std::tan(region.width_rad / 2.0) <= config.widest_m &&
         2.0 * range_m * std::tan(region.height_rad / 2.0) <= config.tallest_m;
}

} // namespace drone_city_nav
