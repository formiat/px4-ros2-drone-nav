#include "drone_city_nav/thermal_reading.hpp"

#include <algorithm>
#include <array>
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

[[nodiscard]] Point3 minus(const Point3& a, const Point3& b) noexcept {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

[[nodiscard]] Point3 scaled(const Point3& a, const double factor) noexcept {
  return {a.x * factor, a.y * factor, a.z * factor};
}

[[nodiscard]] double angleBetween(const Point3& a, const Point3& b) noexcept {
  return std::acos(
      std::clamp(dot(a, b) / std::max(norm(a) * norm(b), 1e-12), -1.0, 1.0));
}

// A symmetric 3 x 3 system by the adjugate: small, and nothing of Eigen in
// the core library.
using Matrix3 = std::array<std::array<double, 3>, 3>;

[[nodiscard]] std::optional<Point3> solve(const Matrix3& a, const Point3& b) noexcept {
  const double determinant = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
                             a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                             a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
  if (!(std::abs(determinant) > 1e-9)) {
    return std::nullopt;
  }
  const Matrix3 adjugate{
      {{a[1][1] * a[2][2] - a[1][2] * a[2][1], a[0][2] * a[2][1] - a[0][1] * a[2][2],
        a[0][1] * a[1][2] - a[0][2] * a[1][1]},
       {a[1][2] * a[2][0] - a[1][0] * a[2][2], a[0][0] * a[2][2] - a[0][2] * a[2][0],
        a[0][2] * a[1][0] - a[0][0] * a[1][2]},
       {a[1][0] * a[2][1] - a[1][1] * a[2][0], a[0][1] * a[2][0] - a[0][0] * a[2][1],
        a[0][0] * a[1][1] - a[0][1] * a[1][0]}}};
  const std::array<double, 3> rhs{b.x, b.y, b.z};
  std::array<double, 3> result{};
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t column = 0U; column < 3U; ++column) {
      result[row] += adjugate[row][column] * rhs[column];
    }
    result[row] /= determinant;
  }
  return Point3{result[0], result[1], result[2]};
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
                        const double cone_margin_rad, const double ceiling_range_m) {
  std::vector<double> ranges;
  for (const StereoDepthReturn& ray : returns) {
    if (ray.hit && !ray.unobservable &&
        angleBetween(ray.point, reading.direction) <=
            reading.half_angle_rad + cone_margin_rad) {
      ranges.push_back(norm(ray.point));
    }
  }
  if (ranges.empty()) {
    return ceiling_range_m;
  }
  std::nth_element(ranges.begin(),
                   ranges.begin() + static_cast<std::ptrdiff_t>(ranges.size() / 2U),
                   ranges.end());
  return std::min(ranges[ranges.size() / 2U], ceiling_range_m);
}

void HeatSourceFix::observe(const Point3& origin, const Point3& direction,
                            const double stamp_s) {
  rays_.push_back({origin, direction, stamp_s});
  while (!rays_.empty() && (stamp_s - rays_.front().stamp_s > config_.window_s ||
                            rays_.front().stamp_s > stamp_s)) {
    rays_.pop_front();
  }
}

double HeatSourceFix::baselineM() const noexcept {
  if (rays_.size() < 2U) {
    return 0.0;
  }
  Point3 mean_direction{0.0, 0.0, 0.0};
  Point3 mean_origin{0.0, 0.0, 0.0};
  for (const Ray& ray : rays_) {
    mean_direction = {mean_direction.x + ray.direction.x,
                      mean_direction.y + ray.direction.y,
                      mean_direction.z + ray.direction.z};
    mean_origin = {mean_origin.x + ray.origin.x, mean_origin.y + ray.origin.y,
                   mean_origin.z + ray.origin.z};
  }
  mean_origin = scaled(mean_origin, 1.0 / static_cast<double>(rays_.size()));
  if (!(norm(mean_direction) > 0.0)) {
    return 0.0;
  }
  mean_direction = scaled(mean_direction, 1.0 / norm(mean_direction));
  // The origins' spread across the mean bearing: along it a bearing fixes nothing.
  double widest{0.0};
  for (const Ray& ray : rays_) {
    const Point3 offset = minus(ray.origin, mean_origin);
    widest = std::max(widest, norm(minus(offset, scaled(mean_direction,
                                                        dot(offset, mean_direction)))));
  }
  return 2.0 * widest;
}

std::optional<Point3> HeatSourceFix::point() const {
  if (rays_.size() < 2U || baselineM() < config_.least_baseline_m) {
    return std::nullopt;
  }
  // The point nearest the rays: the sum over them of (I - d d^T)(p - o) = 0.
  Matrix3 normal{};
  Point3 right{0.0, 0.0, 0.0};
  for (const Ray& ray : rays_) {
    const std::array<double, 3> d{ray.direction.x, ray.direction.y, ray.direction.z};
    const std::array<double, 3> o{ray.origin.x, ray.origin.y, ray.origin.z};
    const double along = d[0] * o[0] + d[1] * o[1] + d[2] * o[2];
    std::array<double, 3> projected{};
    for (std::size_t row = 0U; row < 3U; ++row) {
      for (std::size_t column = 0U; column < 3U; ++column) {
        normal[row][column] += (row == column ? 1.0 : 0.0) - d[row] * d[column];
      }
      projected[row] = o[row] - d[row] * along;
    }
    right = {right.x + projected[0], right.y + projected[1], right.z + projected[2]};
  }
  const std::optional<Point3> p = solve(normal, right);
  if (!p.has_value() || !std::isfinite(p->x) || !std::isfinite(p->y) ||
      !std::isfinite(p->z)) {
    return std::nullopt;
  }
  double residual_square{0.0};
  for (const Ray& ray : rays_) {
    const Point3 offset = minus(*p, ray.origin);
    const double along = dot(offset, ray.direction);
    // Behind a bearing's origin the point is no fix of it.
    if (along <= 0.0) {
      return std::nullopt;
    }
    const Point3 miss = minus(offset, scaled(ray.direction, along));
    residual_square += dot(miss, miss);
  }
  if (std::sqrt(residual_square / static_cast<double>(rays_.size())) >
      config_.most_residual_m) {
    return std::nullopt;
  }
  return p;
}

std::vector<Point3> heatStandoffCap(const ThermalReading& reading, const double range_m,
                                    const double standoff_m, const double spacing_m) {
  std::vector<Point3> cap;
  if (!(standoff_m > 0.0) || !(spacing_m > 0.0) || !(range_m >= 0.0)) {
    return cap;
  }
  const Point3 centre = scaled(reading.direction, range_m);
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
    if (range_m >= standoff_m && dot(normal, reading.direction) >= 0.0) {
      continue;
    }
    cap.push_back(Point3{centre.x + standoff_m * normal.x,
                         centre.y + standoff_m * normal.y,
                         centre.z + standoff_m * normal.z});
  }
  return cap;
}

bool crossesHeatStandoff(const Point3& point, const ThermalReading& reading,
                         const double range_m, const double standoff_m) noexcept {
  const double length = norm(point);
  if (!(length > 0.0)) {
    return false;
  }
  const Point3 centre = scaled(reading.direction, range_m);
  // The ray's closest approach to the centre, and where it enters the ball.
  const double along = dot(centre, point) / length;
  const Point3 miss = minus(centre, scaled(point, along / length));
  const double miss_square = dot(miss, miss);
  if (miss_square >= standoff_m * standoff_m) {
    return false;
  }
  const double half_chord = std::sqrt(standoff_m * standoff_m - miss_square);
  // A ball wholly behind the sensor is not crossed; from inside it every ray is.
  if (along + half_chord <= 0.0) {
    return false;
  }
  return length >= std::max(0.0, along - half_chord);
}

} // namespace drone_city_nav
