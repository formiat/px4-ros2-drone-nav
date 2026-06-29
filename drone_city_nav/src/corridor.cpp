#include "drone_city_nav/corridor.hpp"

#include "drone_city_nav/clearance_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

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

[[nodiscard]] double corridorRayStep(const OccupancyGrid2D& grid,
                                     const CorridorConfig& config,
                                     const double max_radius) noexcept {
  return config.ray_step_m > 0.0 && std::isfinite(config.ray_step_m)
             ? std::clamp(config.ray_step_m, 0.02, max_radius)
             : std::max(0.02, grid.resolution() * 0.5);
}

[[nodiscard]] bool pointIsProhibited(const OccupancyGrid2D& grid, const Point2 point) {
  const std::optional<GridIndex> cell = grid.worldToCell(point);
  return !cell.has_value() || grid.isProhibited(*cell);
}

[[nodiscard]] std::optional<Point2>
recoverCorridorCenter(const OccupancyGrid2D& grid, const Point2 center,
                      const Point2 normal, const double max_recovery_m,
                      const double search_step_m, double& recovery_m) {
  if (!(max_recovery_m > kTinyDistanceM) || !(search_step_m > kTinyDistanceM)) {
    return std::nullopt;
  }

  const int search_steps =
      static_cast<int>(std::ceil((max_recovery_m + kTinyDistanceM) / search_step_m));
  for (int step_index = 1; step_index <= search_steps; ++step_index) {
    const double distance_m =
        std::min(max_recovery_m, static_cast<double>(step_index) * search_step_m);
    const Point2 left_candidate = center + normal * distance_m;
    if (!pointIsProhibited(grid, left_candidate)) {
      recovery_m = distance_m;
      return left_candidate;
    }

    const Point2 right_candidate = center + normal * -distance_m;
    if (!pointIsProhibited(grid, right_candidate)) {
      recovery_m = distance_m;
      return right_candidate;
    }
  }

  return std::nullopt;
}

[[nodiscard]] double raycastBound(const OccupancyGrid2D& grid, const Point2 origin,
                                  const Point2 direction, const CorridorConfig& config,
                                  CorridorStats& stats) {
  const double max_radius = sanitizedPositive(config.max_radius_m, 40.0, 0.1, 5000.0);
  const double safety_margin =
      sanitizedPositive(config.safety_margin_m, 0.0, 0.0, max_radius);
  const double ray_step = corridorRayStep(grid, config, max_radius);

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

[[nodiscard]] double median(std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
  if (values.size() % 2U == 1U) {
    return *middle;
  }
  return (*(middle - 1) + *middle) * 0.5;
}

[[nodiscard]] double localMedianBound(const std::span<const CorridorSample> samples,
                                      const std::size_t index, const double window_m,
                                      const bool left_bound) {
  std::vector<double> values;
  values.reserve(samples.size());
  const double center_s = samples[index].s_m;
  for (const CorridorSample& sample : samples) {
    if (std::abs(sample.s_m - center_s) > window_m) {
      continue;
    }
    const double bound = left_bound ? sample.left_bound_m : sample.right_bound_m;
    if (std::isfinite(bound) && bound >= 0.0) {
      values.push_back(bound);
    }
  }
  return median(values);
}

void applyLocalLateralLimit(std::vector<CorridorSample>& samples,
                            const CorridorConfig& config, CorridorStats& stats) {
  if (samples.size() < 3U) {
    return;
  }
  const double window_m =
      sanitizedPositive(config.lateral_limit_window_m, 20.0, 0.05, 5000.0);
  const double ratio = sanitizedPositive(config.lateral_limit_ratio, 1.25, 1.0, 100.0);
  const double margin_m =
      sanitizedPositive(config.lateral_limit_margin_m, 1.0, 0.0, 5000.0);
  const std::vector<CorridorSample> raw_samples = samples;

  for (std::size_t i = 0U; i < samples.size(); ++i) {
    const double left_limit =
        localMedianBound(raw_samples, i, window_m, true) * ratio + margin_m;
    const double right_limit =
        localMedianBound(raw_samples, i, window_m, false) * ratio + margin_m;
    const double old_left = samples[i].left_bound_m;
    const double old_right = samples[i].right_bound_m;
    samples[i].left_bound_m =
        std::min(samples[i].left_bound_m, std::max(0.0, left_limit));
    samples[i].right_bound_m =
        std::min(samples[i].right_bound_m, std::max(0.0, right_limit));

    const double left_reduction = std::max(0.0, old_left - samples[i].left_bound_m);
    const double right_reduction = std::max(0.0, old_right - samples[i].right_bound_m);
    const double max_reduction = std::max(left_reduction, right_reduction);
    if (max_reduction > kTinyDistanceM) {
      ++stats.lateral_limited_samples;
      stats.max_lateral_bound_reduction_m =
          std::max(stats.max_lateral_bound_reduction_m, max_reduction);
    }
  }
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
  const double ray_step = corridorRayStep(prohibited_grid, config, max_radius);
  const double center_recovery_max_m =
      sanitizedPositive(config.center_recovery_max_m, 3.0, 0.0, max_radius);
  const ClearanceField2D clearance_field = ClearanceField2D::build(
      prohibited_grid, max_radius, ClearanceSource::kProhibited);

  const double length = trajectoryLengthM(route);
  const std::size_t sample_count =
      static_cast<std::size_t>(std::ceil(length / sample_step)) + 1U;
  result.samples.reserve(sample_count + 1U);

  for (std::size_t i = 0U; i <= sample_count; ++i) {
    const double s_m = std::min(length, static_cast<double>(i) * sample_step);
    const Point2 route_center = trajectoryPointAtS(route, s_m);
    const Point2 tangent = normalized(trajectoryTangentAtS(route, s_m));
    if (!finite2D(route_center) || !(norm(tangent) > kTinyDistanceM)) {
      continue;
    }

    const Point2 normal = leftNormal(tangent);
    Point2 center = route_center;
    double center_recovery_m = 0.0;
    if (pointIsProhibited(prohibited_grid, route_center)) {
      if (const std::optional<Point2> recovered_center =
              recoverCorridorCenter(prohibited_grid, route_center, normal,
                                    center_recovery_max_m, ray_step, center_recovery_m);
          recovered_center.has_value()) {
        center = *recovered_center;
        ++result.stats.center_recovered_samples;
        result.stats.max_center_recovery_m =
            std::max(result.stats.max_center_recovery_m, center_recovery_m);
      } else {
        ++result.stats.route_prohibited_samples;
        ++result.stats.center_unrecoverable_samples;
      }
    }

    CorridorSample sample{};
    sample.s_m = s_m;
    sample.route_center = route_center;
    sample.center = center;
    sample.tangent = tangent;
    sample.normal = normal;
    sample.center_recovery_m = center_recovery_m;
    sample.left_bound_m =
        raycastBound(prohibited_grid, center, normal, config, result.stats);
    sample.right_bound_m =
        raycastBound(prohibited_grid, center, normal * -1.0, config, result.stats);

    const std::optional<GridIndex> cell = prohibited_grid.worldToCell(center);
    if (cell.has_value()) {
      sample.clearance_m = clearance_field.distanceAt(*cell);
    }
    result.samples.push_back(sample);

    if (s_m >= length) {
      break;
    }
  }

  applyLocalLateralLimit(result.samples, config, result.stats);

  result.stats.samples = result.samples.size();
  double width_sum = 0.0;
  double clearance_sum = 0.0;
  bool first_sample = true;
  for (const CorridorSample& sample : result.samples) {
    const double width = sample.left_bound_m + sample.right_bound_m;
    width_sum += width;
    clearance_sum += sample.clearance_m;
    updateRange(result.stats.min_width_m, result.stats.max_width_m, width,
                first_sample);
    updateRange(result.stats.min_clearance_m, result.stats.max_clearance_m,
                sample.clearance_m, first_sample);
    first_sample = false;
  }
  if (!result.samples.empty()) {
    result.stats.mean_width_m = width_sum / static_cast<double>(result.samples.size());
    result.stats.mean_clearance_m =
        clearance_sum / static_cast<double>(result.samples.size());
  }
  result.valid = result.samples.size() >= 2U &&
                 result.stats.route_prohibited_samples == 0U &&
                 result.stats.center_unrecoverable_samples == 0U;
  return result;
}

} // namespace drone_city_nav
