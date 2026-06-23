#include "drone_city_nav/corridor.hpp"

#include "drone_city_nav/clearance_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] Point2 operator+(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 operator*(const Point2 point, const double scale) noexcept {
  return Point2{point.x * scale, point.y * scale};
}

[[nodiscard]] Point2 leftNormal(const Point2 tangent) noexcept {
  return Point2{-tangent.y, tangent.x};
}

[[nodiscard]] double norm(const Point2 point) noexcept {
  return std::hypot(point.x, point.y);
}

[[nodiscard]] Point2 normalized(const Point2 point) noexcept {
  const double length = norm(point);
  if (!(length > kTinyDistanceM)) {
    return Point2{};
  }
  return Point2{point.x / length, point.y / length};
}

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] double raycastBound(const OccupancyGrid2D& grid, const Point2 origin,
                                  const Point2 direction, const CorridorConfig& config,
                                  CorridorStats& stats) {
  const double max_radius = sanitizedPositive(config.max_radius_m, 40.0, 0.1, 5000.0);
  const double safety_margin =
      sanitizedPositive(config.safety_margin_m, 0.5, 0.0, max_radius);
  const double ray_step = config.ray_step_m > 0.0 && std::isfinite(config.ray_step_m)
                              ? std::clamp(config.ray_step_m, 0.02, max_radius)
                              : std::max(0.02, grid.resolution() * 0.5);

  double last_clear_distance = 0.0;
  const int ray_steps =
      static_cast<int>(std::ceil((max_radius + kTinyDistanceM) / ray_step));
  for (int step_index = 0; step_index <= ray_steps; ++step_index) {
    const double distance_m =
        std::min(max_radius, static_cast<double>(step_index) * ray_step);
    const Point2 point = origin + direction * distance_m;
    const std::optional<GridIndex> cell = grid.worldToCell(point);
    if (!cell.has_value()) {
      ++stats.outside_grid_samples;
      return std::max(0.0, last_clear_distance - safety_margin);
    }
    if (grid.isProhibited(*cell)) {
      if (distance_m <= kTinyDistanceM) {
        ++stats.route_prohibited_samples;
      }
      return std::max(0.0, last_clear_distance - safety_margin);
    }
    last_clear_distance = distance_m;
  }
  return std::max(0.0, max_radius - safety_margin);
}

void updateRange(double& min_value, double& max_value, const double value,
                 const bool first_sample) {
  if (first_sample) {
    min_value = value;
    max_value = value;
    return;
  }
  min_value = std::min(min_value, value);
  max_value = std::max(max_value, value);
}

} // namespace

CorridorResult buildCorridor(const std::span<const Point2> route_points,
                             const OccupancyGrid2D& prohibited_grid,
                             const CorridorConfig& config) {
  CorridorResult result{};
  result.stats.input_points = route_points.size();
  if (route_points.size() < 2U) {
    return result;
  }

  const std::vector<TrajectorySegment> route = lineTrajectoryFromPoints(route_points);
  if (!trajectoryIsUsable(route)) {
    return result;
  }

  const double max_radius = sanitizedPositive(config.max_radius_m, 40.0, 0.1, 5000.0);
  const double sample_step =
      sanitizedPositive(config.sample_step_m, 1.0, 0.05, std::max(0.05, max_radius));
  const ClearanceField2D clearance_field = ClearanceField2D::build(
      prohibited_grid, max_radius, ClearanceSource::kProhibited);

  const double length = trajectoryLengthM(route);
  const std::size_t sample_count =
      static_cast<std::size_t>(std::ceil(length / sample_step)) + 1U;
  result.samples.reserve(sample_count + 1U);

  double width_sum = 0.0;
  double clearance_sum = 0.0;
  for (std::size_t i = 0U; i <= sample_count; ++i) {
    const double s_m = std::min(length, static_cast<double>(i) * sample_step);
    const Point2 center = trajectoryPointAtS(route, s_m);
    const Point2 tangent = normalized(trajectoryTangentAtS(route, s_m));
    if (!finite2D(center) || !(norm(tangent) > kTinyDistanceM)) {
      continue;
    }

    const Point2 normal = leftNormal(tangent);
    CorridorSample sample{};
    sample.s_m = s_m;
    sample.center = center;
    sample.tangent = tangent;
    sample.normal = normal;
    sample.left_bound_m =
        raycastBound(prohibited_grid, center, normal, config, result.stats);
    sample.right_bound_m =
        raycastBound(prohibited_grid, center, normal * -1.0, config, result.stats);

    const std::optional<GridIndex> cell = prohibited_grid.worldToCell(center);
    if (cell.has_value()) {
      sample.clearance_m = clearance_field.distanceAt(*cell);
    }
    result.samples.push_back(sample);
    const double width = sample.left_bound_m + sample.right_bound_m;
    width_sum += width;
    clearance_sum += sample.clearance_m;
    const bool first = result.samples.size() == 1U;
    updateRange(result.stats.min_width_m, result.stats.max_width_m, width, first);
    updateRange(result.stats.min_clearance_m, result.stats.max_clearance_m,
                sample.clearance_m, first);

    if (s_m >= length) {
      break;
    }
  }

  result.stats.samples = result.samples.size();
  if (!result.samples.empty()) {
    result.stats.mean_width_m = width_sum / static_cast<double>(result.samples.size());
    result.stats.mean_clearance_m =
        clearance_sum / static_cast<double>(result.samples.size());
  }
  result.valid =
      result.samples.size() >= 2U && result.stats.route_prohibited_samples == 0U;
  return result;
}

} // namespace drone_city_nav
