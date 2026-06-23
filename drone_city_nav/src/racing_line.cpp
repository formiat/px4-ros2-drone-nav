#include "drone_city_nav/racing_line.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] Point2 operator+(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] Point2 operator*(const Point2 point, const double scale) noexcept {
  return Point2{point.x * scale, point.y * scale};
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

[[nodiscard]] double cross(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] std::vector<Point2>
pointsFromOffsets(const std::span<const CorridorSample> corridor_samples,
                  const std::span<const double> offsets) {
  std::vector<Point2> points;
  points.reserve(corridor_samples.size());
  for (std::size_t i = 0U; i < corridor_samples.size(); ++i) {
    points.push_back(corridor_samples[i].center +
                     corridor_samples[i].normal * offsets[i]);
  }
  return points;
}

[[nodiscard]] double pathLength(const std::span<const Point2> points) {
  double length = 0.0;
  for (std::size_t i = 1U; i < points.size(); ++i) {
    length += distance(points[i - 1U], points[i]);
  }
  return length;
}

[[nodiscard]] double discreteCurvature(const Point2 previous, const Point2 current,
                                       const Point2 next) {
  const double a = distance(previous, current);
  const double b = distance(current, next);
  const double c = distance(previous, next);
  if (!(a > kTinyDistanceM) || !(b > kTinyDistanceM) || !(c > kTinyDistanceM)) {
    return 0.0;
  }
  const double signed_double_area = cross(current - previous, next - previous);
  return 2.0 * signed_double_area / (a * b * c);
}

[[nodiscard]] bool segmentTraversable(const OccupancyGrid2D& grid, const Point2 start,
                                      const Point2 end) {
  const std::optional<GridIndex> start_cell = grid.worldToCell(start);
  const std::optional<GridIndex> end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    return false;
  }
  const std::vector<GridIndex> cells = grid.cellsOnLine(*start_cell, *end_cell);
  return std::ranges::all_of(
      cells, [&grid](const GridIndex cell) { return !grid.isProhibited(cell); });
}

[[nodiscard]] bool pathTraversable(const OccupancyGrid2D& grid,
                                   const std::span<const Point2> points) {
  if (points.size() < 2U) {
    return false;
  }
  for (std::size_t i = 1U; i < points.size(); ++i) {
    if (!segmentTraversable(grid, points[i - 1U], points[i])) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] double costForPoints(const std::span<const Point2> points,
                                   const std::span<const double> offsets,
                                   const RacingLineConfig& config) {
  if (points.size() < 2U) {
    return std::numeric_limits<double>::infinity();
  }

  const double weight_length = sanitizedPositive(config.weight_length, 1.0, 0.0, 1.0e6);
  const double weight_curvature =
      sanitizedPositive(config.weight_curvature, 25.0, 0.0, 1.0e9);
  const double weight_curvature_change =
      sanitizedPositive(config.weight_curvature_change, 10.0, 0.0, 1.0e9);
  const double weight_center_bias =
      sanitizedPositive(config.weight_center_bias, 0.02, 0.0, 1.0e6);

  double curvature_cost = 0.0;
  double curvature_change_cost = 0.0;
  double center_bias_cost = 0.0;
  double previous_curvature = 0.0;
  bool previous_curvature_valid = false;
  for (const double offset : offsets) {
    center_bias_cost += offset * offset;
  }
  for (std::size_t i = 1U; i + 1U < points.size(); ++i) {
    const double curvature =
        discreteCurvature(points[i - 1U], points[i], points[i + 1U]);
    curvature_cost += curvature * curvature;
    if (previous_curvature_valid) {
      const double change = curvature - previous_curvature;
      curvature_change_cost += change * change;
    }
    previous_curvature = curvature;
    previous_curvature_valid = true;
  }

  return weight_length * pathLength(points) + weight_curvature * curvature_cost +
         weight_curvature_change * curvature_change_cost +
         weight_center_bias * center_bias_cost;
}

void populateSampleGeometry(std::vector<TrajectoryPointSample>& samples) {
  double s_m = 0.0;
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    if (i > 0U) {
      s_m += distance(samples[i - 1U].point, samples[i].point);
    }
    samples[i].s_m = s_m;
    if (samples.size() == 1U) {
      samples[i].tangent = Point2{1.0, 0.0};
    } else if (i == 0U) {
      samples[i].tangent = normalized(samples[i + 1U].point - samples[i].point);
    } else if (i + 1U == samples.size()) {
      samples[i].tangent = normalized(samples[i].point - samples[i - 1U].point);
    } else {
      samples[i].tangent = normalized(samples[i + 1U].point - samples[i - 1U].point);
      samples[i].curvature_1pm = discreteCurvature(
          samples[i - 1U].point, samples[i].point, samples[i + 1U].point);
    }
  }
}

void updateCurvatureStats(const std::span<const TrajectoryPointSample> samples,
                          RacingLineStats& stats) {
  double curvature_sum = 0.0;
  std::size_t curvature_count = 0U;
  for (const TrajectoryPointSample& sample : samples) {
    const double abs_curvature = std::abs(sample.curvature_1pm);
    stats.max_abs_curvature_1pm = std::max(stats.max_abs_curvature_1pm, abs_curvature);
    curvature_sum += abs_curvature;
    ++curvature_count;
  }
  if (curvature_count > 0U) {
    stats.mean_abs_curvature_1pm = curvature_sum / static_cast<double>(curvature_count);
  }
}

} // namespace

RacingLineResult
optimizeRacingLine(const std::span<const CorridorSample> corridor_samples,
                   const OccupancyGrid2D& prohibited_grid,
                   const RacingLineConfig& config) {
  RacingLineResult result{};
  result.stats.input_samples = corridor_samples.size();
  if (!config.enabled || corridor_samples.size() < 2U) {
    return result;
  }

  const std::size_t sample_count = corridor_samples.size();
  std::vector<double> offsets(sample_count, 0.0);
  std::vector<Point2> centerline = pointsFromOffsets(corridor_samples, offsets);
  if (!pathTraversable(prohibited_grid, centerline)) {
    ++result.stats.collision_rejections;
    return result;
  }

  result.stats.centerline_length_m = pathLength(centerline);
  double best_cost = costForPoints(centerline, offsets, config);
  result.stats.initial_cost = best_cost;

  const double min_step =
      sanitizedPositive(config.min_offset_step_m, 0.1, 0.001, 100.0);
  const double cooling = sanitizedPositive(config.cooling_ratio, 0.5, 0.05, 0.95);
  double step = std::max(
      min_step, sanitizedPositive(config.initial_offset_step_m, 2.0, 0.001, 500.0));
  const std::size_t max_iterations = std::clamp<std::size_t>(
      config.max_iterations, 1U, static_cast<std::size_t>(10000U));
  const double max_length_ratio =
      sanitizedPositive(config.max_length_ratio, 1.25, 1.0, 100.0);

  for (std::size_t iteration = 0U; iteration < max_iterations && step >= min_step;
       ++iteration) {
    bool changed = false;
    for (std::size_t i = 1U; i + 1U < sample_count; ++i) {
      double best_offset = offsets[i];
      for (const double delta : {-step, 0.0, step}) {
        std::vector<double> candidate_offsets = offsets;
        candidate_offsets[i] =
            std::clamp(offsets[i] + delta, -corridor_samples[i].right_bound_m,
                       corridor_samples[i].left_bound_m);
        std::vector<Point2> candidate_points =
            pointsFromOffsets(corridor_samples, candidate_offsets);
        ++result.stats.candidate_evaluations;
        if (pathLength(candidate_points) >
            result.stats.centerline_length_m * max_length_ratio) {
          continue;
        }
        if (!pathTraversable(prohibited_grid, candidate_points)) {
          ++result.stats.collision_rejections;
          continue;
        }
        const double candidate_cost =
            costForPoints(candidate_points, candidate_offsets, config);
        if (candidate_cost + 1.0e-9 < best_cost) {
          best_cost = candidate_cost;
          best_offset = candidate_offsets[i];
          changed = true;
        }
      }
      offsets[i] = best_offset;
    }
    ++result.stats.iterations;
    if (!changed) {
      step *= cooling;
    }
  }

  std::vector<Point2> final_points = pointsFromOffsets(corridor_samples, offsets);
  if (!pathTraversable(prohibited_grid, final_points)) {
    ++result.stats.collision_rejections;
    return result;
  }

  result.samples.reserve(sample_count);
  for (std::size_t i = 0U; i < sample_count; ++i) {
    TrajectoryPointSample sample{};
    sample.point = final_points[i];
    sample.left_bound_m = corridor_samples[i].left_bound_m;
    sample.right_bound_m = corridor_samples[i].right_bound_m;
    result.stats.max_abs_offset_m =
        std::max(result.stats.max_abs_offset_m, std::abs(offsets[i]));
    result.samples.push_back(sample);
  }
  populateSampleGeometry(result.samples);
  result.stats.output_samples = result.samples.size();
  result.stats.final_length_m = pathLength(final_points);
  result.stats.final_cost = best_cost;
  updateCurvatureStats(result.samples, result.stats);
  result.valid = trajectorySamplesAreUsable(result.samples);
  return result;
}

} // namespace drone_city_nav
