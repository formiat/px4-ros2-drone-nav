#include "drone_city_nav/trajectory_straightening.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] Point2 operator+(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] Point2 operator*(const Point2 point, const double scale) noexcept {
  return Point2{point.x * scale, point.y * scale};
}

[[nodiscard]] double dot(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.x + lhs.y * rhs.y;
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

[[nodiscard]] double headingErrorRad(const Point2 lhs, const Point2 rhs) noexcept {
  const Point2 lhs_normalized = normalized(lhs);
  const Point2 rhs_normalized = normalized(rhs);
  if (!(norm(lhs_normalized) > kTinyDistanceM) ||
      !(norm(rhs_normalized) > kTinyDistanceM)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::acos(std::clamp(dot(lhs_normalized, rhs_normalized), -1.0, 1.0));
}

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] double pathLength(const std::span<const TrajectoryPointSample> samples,
                                const std::size_t start_index,
                                const std::size_t end_index) {
  if (samples.empty() || start_index >= samples.size() || end_index >= samples.size() ||
      start_index >= end_index) {
    return 0.0;
  }

  double length = 0.0;
  for (std::size_t i = start_index + 1U; i <= end_index; ++i) {
    length += distance(samples[i - 1U].point, samples[i].point);
  }
  return length;
}

[[nodiscard]] bool segmentIsTraversable(const OccupancyGrid2D& grid, const Point2 start,
                                        const Point2 end) {
  const std::optional<GridIndex> start_cell = grid.worldToCell(start);
  const std::optional<GridIndex> end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    return false;
  }
  const std::vector<GridIndex> cells = grid.cellsOnLine(*start_cell, *end_cell);
  return std::ranges::none_of(
      cells, [&grid](const GridIndex cell) { return grid.isProhibited(cell); });
}

[[nodiscard]] std::optional<CorridorSample>
nearestCorridorSample(const std::span<const CorridorSample> corridor_samples,
                      const Point2 point) {
  if (corridor_samples.empty()) {
    return std::nullopt;
  }

  const CorridorSample* best = nullptr;
  double best_distance_sq = std::numeric_limits<double>::infinity();
  for (const CorridorSample& sample : corridor_samples) {
    const double distance_sq = squaredDistance(sample.center, point);
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best = &sample;
    }
  }
  if (best == nullptr) {
    return std::nullopt;
  }
  return *best;
}

[[nodiscard]] bool pointInsideCorridor(const CorridorSample& corridor,
                                       const Point2 point,
                                       const double margin_m) noexcept {
  const double offset_m = dot(point - corridor.center, corridor.normal);
  return offset_m <= corridor.left_bound_m - margin_m + kTinyDistanceM &&
         -offset_m <= corridor.right_bound_m - margin_m + kTinyDistanceM;
}

[[nodiscard]] bool lineInsideCorridor(const Point2 start, const Point2 end,
                                      const std::span<const CorridorSample> corridor,
                                      const double step_m, const double margin_m) {
  const double length = distance(start, end);
  if (!(length > kTinyDistanceM) || !finite2D(start) || !finite2D(end)) {
    return false;
  }

  const std::size_t steps =
      std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(length / step_m)));
  for (std::size_t i = 0U; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const Point2 point = start + (end - start) * t;
    const std::optional<CorridorSample> sample = nearestCorridorSample(corridor, point);
    if (!sample.has_value() || !pointInsideCorridor(*sample, point, margin_m)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool
rangeShapeIsStraightEnough(const std::span<const TrajectoryPointSample> samples,
                           const std::size_t start_index, const std::size_t end_index,
                           const TrajectoryStraighteningConfig& config,
                           TrajectoryStraighteningStats& stats) {
  const double subpath_length = pathLength(samples, start_index, end_index);
  const double chord_length =
      distance(samples[start_index].point, samples[end_index].point);
  if (!(chord_length > kTinyDistanceM) || !(subpath_length > kTinyDistanceM)) {
    ++stats.rejected_shape;
    return false;
  }

  const double min_segment_length =
      sanitizedPositive(config.min_segment_length_m, 20.0, 0.0, 100000.0);
  if (subpath_length < min_segment_length) {
    ++stats.rejected_too_short;
    return false;
  }

  const double max_length_ratio =
      sanitizedPositive(config.max_path_length_ratio, 1.035, 1.0, 100.0);
  if (subpath_length > chord_length * max_length_ratio) {
    ++stats.rejected_shape;
    return false;
  }

  const Point2 chord_direction =
      normalized(samples[end_index].point - samples[start_index].point);
  const double max_heading_error =
      sanitizedPositive(config.max_heading_error_rad, 0.35, 0.0, std::numbers::pi);
  for (std::size_t i = start_index + 1U; i <= end_index; ++i) {
    const Point2 local_direction = normalized(samples[i].point - samples[i - 1U].point);
    if (headingErrorRad(chord_direction, local_direction) > max_heading_error) {
      ++stats.rejected_shape;
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool canReplaceRangeWithLine(
    const std::span<const TrajectoryPointSample> samples, const std::size_t start_index,
    const std::size_t end_index, const std::span<const CorridorSample> corridor_samples,
    const OccupancyGrid2D& prohibited_grid, const TrajectoryStraighteningConfig& config,
    TrajectoryStraighteningStats& stats) {
  if (end_index <= start_index + 1U) {
    return true;
  }
  if (!rangeShapeIsStraightEnough(samples, start_index, end_index, config, stats)) {
    return false;
  }
  const Point2 start = samples[start_index].point;
  const Point2 end = samples[end_index].point;
  if (!segmentIsTraversable(prohibited_grid, start, end)) {
    ++stats.rejected_prohibited;
    return false;
  }
  const double validation_step =
      sanitizedPositive(config.validation_step_m, 2.0, 0.1, 1000.0);
  const double corridor_margin =
      sanitizedPositive(config.min_corridor_margin_m, 0.5, 0.0, 1000.0);
  if (!lineInsideCorridor(start, end, corridor_samples, validation_step,
                          corridor_margin)) {
    ++stats.rejected_corridor;
    return false;
  }
  return true;
}

void assignCorridorMetadata(std::vector<TrajectoryPointSample>& samples,
                            const std::span<const CorridorSample> corridor_samples) {
  for (TrajectoryPointSample& sample : samples) {
    const std::optional<CorridorSample> corridor =
        nearestCorridorSample(corridor_samples, sample.point);
    if (!corridor.has_value()) {
      continue;
    }
    sample.left_bound_m = corridor->left_bound_m;
    sample.right_bound_m = corridor->right_bound_m;
    sample.racing_offset_m = dot(sample.point - corridor->center, corridor->normal);
  }
}

} // namespace

TrajectoryStraighteningResult
straightenTrajectory(const std::span<const TrajectoryPointSample> samples,
                     const std::span<const CorridorSample> corridor_samples,
                     const OccupancyGrid2D& prohibited_grid,
                     const TrajectoryStraighteningConfig& config) {
  TrajectoryStraighteningResult result{};
  result.stats.input_samples = samples.size();
  result.samples.assign(samples.begin(), samples.end());
  if (samples.size() < 3U || corridor_samples.empty()) {
    result.stats.output_samples = result.samples.size();
    result.valid = trajectorySamplesAreUsable(result.samples);
    return result;
  }

  const TrajectoryShapeDiagnostics before_shape =
      computeTrajectoryShapeDiagnostics(samples);
  result.stats.max_heading_delta_before_rad = before_shape.max_heading_delta_rad;
  result.stats.max_curvature_jump_before_1pm = before_shape.max_curvature_jump_1pm;

  std::vector<Point2> output_points;
  output_points.reserve(samples.size());
  output_points.push_back(samples.front().point);
  std::size_t current_index = 0U;
  while (current_index + 1U < samples.size()) {
    std::size_t best_index = current_index + 1U;
    for (std::size_t candidate = samples.size() - 1U; candidate > current_index + 1U;
         --candidate) {
      if (canReplaceRangeWithLine(samples, current_index, candidate, corridor_samples,
                                  prohibited_grid, config, result.stats)) {
        best_index = candidate;
        break;
      }
    }
    if (best_index > current_index + 1U) {
      ++result.stats.collapsed_segments;
    }
    output_points.push_back(samples[best_index].point);
    current_index = best_index;
  }

  result.samples = trajectoryPointSamplesFromPoints(
      std::span<const Point2>{output_points.data(), output_points.size()});
  assignCorridorMetadata(result.samples, corridor_samples);
  const TrajectoryShapeDiagnostics after_shape =
      computeTrajectoryShapeDiagnostics(result.samples);
  result.stats.output_samples = result.samples.size();
  result.stats.max_heading_delta_after_rad = after_shape.max_heading_delta_rad;
  result.stats.max_curvature_jump_after_1pm = after_shape.max_curvature_jump_1pm;
  result.changed = result.samples.size() != samples.size();
  result.valid = trajectorySamplesAreUsable(result.samples);
  return result;
}

} // namespace drone_city_nav
