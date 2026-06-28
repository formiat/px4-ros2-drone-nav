#include "drone_city_nav/planner_core.hpp"

#include "drone_city_nav/clearance_field.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceSqM = 1.0e-12;
constexpr double kTurnAngleThresholdRad = 1.0e-3;
constexpr double kShortSegment2M = 2.0;
constexpr double kShortSegment5M = 5.0;
constexpr double kShortSegment10M = 10.0;

struct UnitDirection2D {
  double x{0.0};
  double y{0.0};
};

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] double
normalizedClearanceDiagnosticRadiusM(const double configured_radius_m) noexcept {
  if (std::isfinite(configured_radius_m) && configured_radius_m > 0.0) {
    return configured_radius_m;
  }
  return 10.0;
}

[[nodiscard]] double
normalizedEscapeSearchRadiusM(const double configured_radius_m) noexcept {
  if (std::isfinite(configured_radius_m) && configured_radius_m > 0.0) {
    return configured_radius_m;
  }
  return 10.0;
}

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point start) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - start)
                                 .count()) /
         1000.0;
}

[[nodiscard]] bool directionChanged(const UnitDirection2D previous,
                                    const UnitDirection2D current) noexcept {
  const double dot =
      std::clamp(previous.x * current.x + previous.y * current.y, -1.0, 1.0);
  return std::acos(dot) > kTurnAngleThresholdRad;
}

void addPathSegmentMetrics(PathMetrics& metrics,
                           std::optional<UnitDirection2D>& previous, const double dx_m,
                           const double dy_m) {
  const double segment_length_m = std::hypot(dx_m, dy_m);
  if (segment_length_m <= 0.0) {
    return;
  }

  ++metrics.segments;
  metrics.length_m += segment_length_m;
  if (metrics.segments == 1U) {
    metrics.min_segment_length_m = segment_length_m;
    metrics.max_segment_length_m = segment_length_m;
  } else {
    metrics.min_segment_length_m =
        std::min(metrics.min_segment_length_m, segment_length_m);
    metrics.max_segment_length_m =
        std::max(metrics.max_segment_length_m, segment_length_m);
  }
  metrics.mean_segment_length_m =
      metrics.length_m / static_cast<double>(metrics.segments);
  if (segment_length_m < kShortSegment2M) {
    ++metrics.segments_shorter_than_2m;
  }
  if (segment_length_m < kShortSegment5M) {
    ++metrics.segments_shorter_than_5m;
  }
  if (segment_length_m < kShortSegment10M) {
    ++metrics.segments_shorter_than_10m;
  }

  const UnitDirection2D current{dx_m / segment_length_m, dy_m / segment_length_m};
  if (!previous.has_value()) {
    ++metrics.straight_segments;
    previous = current;
    return;
  }

  if (directionChanged(*previous, current)) {
    ++metrics.turns;
    ++metrics.straight_segments;
  }
  previous = current;
}

} // namespace

const char*
stablePathDecisionReasonName(const StablePathDecisionReason reason) noexcept {
  switch (reason) {
    case StablePathDecisionReason::kDisabled:
      return "disabled";
    case StablePathDecisionReason::kNoPreviousPath:
      return "no_previous_path";
    case StablePathDecisionReason::kGoalMismatch:
      return "goal_mismatch";
    case StablePathDecisionReason::kProjectionUnavailable:
      return "projection_unavailable";
    case StablePathDecisionReason::kClear:
      return "clear";
    case StablePathDecisionReason::kProhibitedConfirmed:
      return "prohibited_confirmed";
  }
  return "unknown";
}

GridStats collectGridStats(const OccupancyGrid2D& grid) {
  GridStats stats{};
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const GridIndex cell{x, y};
      if (grid.isInflated(cell)) {
        ++stats.inflated_cells;
      }
      switch (grid.state(cell)) {
        case CellState::kUnknown:
          ++stats.unknown_cells;
          break;
        case CellState::kFree:
          ++stats.free_cells;
          break;
        case CellState::kOccupied:
          ++stats.occupied_cells;
          break;
      }
    }
  }
  return stats;
}

PathMetrics gridPathMetrics(const OccupancyGrid2D& grid,
                            const std::span<const GridIndex> path) {
  PathMetrics metrics{};
  metrics.points = path.size();
  if (!(grid.resolution() > 0.0) || path.size() < 2U) {
    return metrics;
  }

  std::optional<UnitDirection2D> previous_direction;
  for (std::size_t i = 1U; i < path.size(); ++i) {
    const double dx_m =
        static_cast<double>(path[i].x - path[i - 1U].x) * grid.resolution();
    const double dy_m =
        static_cast<double>(path[i].y - path[i - 1U].y) * grid.resolution();
    addPathSegmentMetrics(metrics, previous_direction, dx_m, dy_m);
  }
  return metrics;
}

PathMetrics pointPathMetrics(const std::span<const Point2> path_points) {
  PathMetrics metrics{};
  metrics.points = path_points.size();
  if (path_points.size() < 2U) {
    return metrics;
  }

  std::optional<UnitDirection2D> previous_direction;
  for (std::size_t i = 1U; i < path_points.size(); ++i) {
    addPathSegmentMetrics(metrics, previous_direction,
                          path_points[i].x - path_points[i - 1U].x,
                          path_points[i].y - path_points[i - 1U].y);
  }
  return metrics;
}

[[nodiscard]] double pathMinimumClearanceM(const ClearanceField2D& clearance_field,
                                           std::span<const GridIndex> path);

double pathMinimumProhibitedClearanceM(const OccupancyGrid2D& grid,
                                       const std::span<const GridIndex> path,
                                       const double max_distance_m) {
  const ClearanceField2D clearance_field = ClearanceField2D::build(
      grid, normalizedClearanceDiagnosticRadiusM(max_distance_m),
      ClearanceSource::kProhibited);
  return pathMinimumClearanceM(clearance_field, path);
}

double pathMinimumClearanceM(const ClearanceField2D& clearance_field,
                             const std::span<const GridIndex> path) {
  double minimum_clearance_m = std::numeric_limits<double>::infinity();
  for (const GridIndex cell : path) {
    if (!clearance_field.contains(cell)) {
      continue;
    }
    minimum_clearance_m =
        std::min(minimum_clearance_m, clearance_field.distanceAt(cell));
  }
  return minimum_clearance_m;
}

bool pathSegmentIsAllowed(const OccupancyGrid2D& grid, const Point2 start,
                          const Point2 end) {
  const auto start_cell = grid.worldToCell(start);
  const auto end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    return false;
  }

  return hasLineOfSight(grid, *start_cell, *end_cell);
}

bool pathSegmentIsTraversable(const OccupancyGrid2D& grid, const Point2 start,
                              const Point2 end) {
  const auto start_cell = grid.worldToCell(start);
  const auto end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    return false;
  }

  const std::vector<GridIndex> line_cells = grid.cellsOnLine(*start_cell, *end_cell);
  if (line_cells.empty()) {
    return false;
  }

  const bool starts_prohibited = grid.isProhibited(line_cells.front());
  if (!starts_prohibited) {
    return std::ranges::none_of(
        line_cells, [&grid](const GridIndex cell) { return grid.isProhibited(cell); });
  }

  if (grid.isProhibited(line_cells.back())) {
    return false;
  }

  bool escaped_prohibited_prefix = false;
  for (const GridIndex cell : line_cells) {
    const bool prohibited = grid.isProhibited(cell);
    if (!prohibited) {
      escaped_prohibited_prefix = true;
      continue;
    }
    if (escaped_prohibited_prefix) {
      return false;
    }
  }

  return escaped_prohibited_prefix;
}

std::optional<GridIndex> nearestAllowedEscapeStartCell(
    const OccupancyGrid2D& grid, const GridIndex prohibited_start_cell,
    const Point2 current_position, const double max_search_radius_m) {
  if (!grid.contains(prohibited_start_cell) || !finite2D(current_position) ||
      !grid.isProhibited(prohibited_start_cell) ||
      grid.isOccupied(prohibited_start_cell)) {
    return std::nullopt;
  }

  const double resolution_m = grid.resolution();
  if (!(resolution_m > 0.0)) {
    return std::nullopt;
  }

  const double search_radius_m = normalizedEscapeSearchRadiusM(max_search_radius_m);
  const int radius_cells =
      std::max(1, static_cast<int>(std::ceil(search_radius_m / resolution_m)));
  const double radius_sq_m = search_radius_m * search_radius_m;

  std::optional<GridIndex> best_cell;
  double best_distance_sq_m = std::numeric_limits<double>::infinity();
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const GridIndex candidate{prohibited_start_cell.x + dx,
                                prohibited_start_cell.y + dy};
      if (!grid.contains(candidate) || grid.isProhibited(candidate)) {
        continue;
      }

      const Point2 candidate_center = grid.cellCenter(candidate);
      const double candidate_distance_sq_m =
          squaredDistance(current_position, candidate_center);
      if (candidate_distance_sq_m > radius_sq_m ||
          candidate_distance_sq_m >= best_distance_sq_m) {
        continue;
      }
      if (!pathSegmentIsTraversable(grid, current_position, candidate_center)) {
        continue;
      }

      best_cell = candidate;
      best_distance_sq_m = candidate_distance_sq_m;
    }
  }

  return best_cell;
}

std::optional<PathProjection2D>
closestPathProjection(const std::span<const Point2> path_points,
                      const Point2 current_position) {
  if (path_points.empty() || !finite2D(current_position)) {
    return std::nullopt;
  }
  if (path_points.size() == 1U) {
    return PathProjection2D{0U, 0.0,
                            squaredDistance(current_position, path_points.front()),
                            path_points.front()};
  }

  PathProjection2D best{};
  for (std::size_t i = 0U; i + 1U < path_points.size(); ++i) {
    const Point2 segment_start = path_points[i];
    const Point2 segment_end = path_points[i + 1U];
    const Point2 segment{segment_end.x - segment_start.x,
                         segment_end.y - segment_start.y};
    const double segment_length_sq = squaredDistance(segment_start, segment_end);
    const double segment_t =
        segment_length_sq > kTinyDistanceSqM
            ? std::clamp(((current_position.x - segment_start.x) * segment.x +
                          (current_position.y - segment_start.y) * segment.y) /
                             segment_length_sq,
                         0.0, 1.0)
            : 0.0;
    const Point2 projected{segment_start.x + segment.x * segment_t,
                           segment_start.y + segment.y * segment_t};
    const double distance_sq = squaredDistance(current_position, projected);
    if (distance_sq < best.distance_sq) {
      best = PathProjection2D{i, segment_t, distance_sq, projected};
    }
  }

  return best;
}

std::optional<std::vector<Point2>> remainingPathFromCurrentPose(
    const std::span<const Point2> path_points, const Point2 current_position,
    const Point2 goal, const double stable_path_goal_tolerance_m, double& deviation_m) {
  deviation_m = std::numeric_limits<double>::quiet_NaN();
  if (path_points.size() < 2U || !finite2D(current_position)) {
    return std::nullopt;
  }
  if (distance(path_points.back(), goal) > stable_path_goal_tolerance_m) {
    return std::nullopt;
  }

  const auto projection = closestPathProjection(path_points, current_position);
  if (!projection.has_value()) {
    return std::nullopt;
  }

  deviation_m = std::sqrt(projection->distance_sq);
  std::vector<Point2> remaining_path;
  remaining_path.reserve(path_points.size() - projection->segment_start_index + 1U);
  remaining_path.push_back(current_position);

  const std::size_t next_index =
      std::min(projection->segment_start_index + 1U, path_points.size() - 1U);
  for (std::size_t i = next_index; i < path_points.size(); ++i) {
    if (squaredDistance(remaining_path.back(), path_points[i]) <= kTinyDistanceSqM) {
      continue;
    }
    remaining_path.push_back(path_points[i]);
  }

  if (remaining_path.size() < 2U &&
      squaredDistance(current_position, goal) > kTinyDistanceSqM) {
    remaining_path.push_back(goal);
  }
  if (remaining_path.size() < 2U) {
    return std::nullopt;
  }

  return remaining_path;
}

bool pathIsTraversable(const OccupancyGrid2D& grid,
                       const std::span<const Point2> path_points,
                       std::size_t* const non_traversable_segment_index) {
  if (path_points.size() < 2U) {
    return true;
  }

  for (std::size_t index = 1U; index < path_points.size(); ++index) {
    if (pathSegmentIsTraversable(grid, path_points[index - 1U], path_points[index])) {
      continue;
    }
    if (non_traversable_segment_index != nullptr) {
      *non_traversable_segment_index = index - 1U;
    }
    return false;
  }
  return true;
}

PlannerCore::PlannerCore(const PlannerCoreConfig& config)
    : config_{config} {
}

void PlannerCore::setConfig(const PlannerCoreConfig& config) {
  config_ = config;
}

const PlannerCoreConfig& PlannerCore::config() const noexcept {
  return config_;
}

std::optional<PathComputationResult>
PlannerCore::computePath(const OccupancyGrid2D& grid, const Point2 current_position,
                         const Point2 goal) const {
  return computePath(grid, current_position, goal, config_.astar);
}

std::optional<PathComputationResult>
PlannerCore::computePath(const OccupancyGrid2D& grid, const Point2 current_position,
                         const Point2 goal, const AStarConfig& astar_config) const {
  const auto total_started_at = std::chrono::steady_clock::now();
  PathComputationResult result{};
  result.requested_start_cell = grid.worldToCell(current_position);
  result.start_cell = result.requested_start_cell;
  result.goal_cell = grid.worldToCell(goal);
  if (!result.start_cell.has_value() || !result.goal_cell.has_value()) {
    return std::nullopt;
  }
  if (grid.isProhibited(*result.goal_cell)) {
    return std::nullopt;
  }

  if (grid.isProhibited(*result.start_cell)) {
    const std::optional<GridIndex> escape_start_cell =
        nearestAllowedEscapeStartCell(grid, *result.start_cell, current_position,
                                      config_.start_prohibited_escape_search_radius_m);
    if (!escape_start_cell.has_value()) {
      return std::nullopt;
    }

    result.start_escape_used = true;
    result.start_escape_distance_m =
        distance(current_position, grid.cellCenter(*escape_start_cell));
    result.start_cell = *escape_start_cell;
  }

  const auto astar_started_at = std::chrono::steady_clock::now();
  result.astar =
      planner_.plan(grid, *result.start_cell, *result.goal_cell, astar_config);
  result.astar_duration_ms = elapsedMilliseconds(astar_started_at);
  if (!result.astar.success) {
    return std::nullopt;
  }

  const auto smoothing_started_at = std::chrono::steady_clock::now();
  const PathSmoothingResult smoothing = smoothPathWithStats(grid, result.astar.path);
  result.smoothing_duration_ms = elapsedMilliseconds(smoothing_started_at);
  result.smoothed_cells = smoothing.path;
  result.smoothing_stats = smoothing.stats;
  if (result.smoothed_cells.empty() && !result.astar.path.empty()) {
    result.smoothing_returned_empty_path = true;
  }
  const auto grid_stats_started_at = std::chrono::steady_clock::now();
  result.grid_stats = collectGridStats(grid);
  result.grid_stats_duration_ms = elapsedMilliseconds(grid_stats_started_at);

  const auto raw_metrics_started_at = std::chrono::steady_clock::now();
  result.raw_path_metrics = gridPathMetrics(grid, result.astar.path);
  result.raw_path_metrics_duration_ms = elapsedMilliseconds(raw_metrics_started_at);

  const auto smoothed_metrics_started_at = std::chrono::steady_clock::now();
  result.smoothed_path_metrics = gridPathMetrics(grid, result.smoothed_cells);
  result.smoothed_path_metrics_duration_ms =
      elapsedMilliseconds(smoothed_metrics_started_at);

  const double clearance_radius_m =
      normalizedClearanceDiagnosticRadiusM(config_.clearance_diagnostic_radius_m);
  const auto clearance_field_started_at = std::chrono::steady_clock::now();
  const ClearanceFieldCacheLookup clearance_field =
      prohibited_clearance_cache_.getOrBuild(grid, clearance_radius_m,
                                             ClearanceSource::kProhibited);
  result.prohibited_clearance_field_duration_ms =
      elapsedMilliseconds(clearance_field_started_at);
  result.prohibited_clearance_field_cache_hit = clearance_field.cache_hit;
  if (clearance_field.field == nullptr) {
    return std::nullopt;
  }

  const auto raw_clearance_started_at = std::chrono::steady_clock::now();
  result.raw_path_clearance_m =
      pathMinimumClearanceM(*clearance_field.field, result.astar.path);
  result.raw_path_clearance_duration_ms = elapsedMilliseconds(raw_clearance_started_at);

  const auto smoothed_clearance_started_at = std::chrono::steady_clock::now();
  result.smoothed_path_clearance_m =
      pathMinimumClearanceM(*clearance_field.field, result.smoothed_cells);
  result.smoothed_path_clearance_duration_ms =
      elapsedMilliseconds(smoothed_clearance_started_at);
  result.total_duration_ms = elapsedMilliseconds(total_started_at);
  return result;
}

StablePathDecision PlannerCore::evaluateStablePath(
    const OccupancyGrid2D& grid, const std::span<const Point2> previous_path,
    const Point2 current_position, const Point2 goal) const {
  StablePathDecision decision{};
  if (previous_path.size() < 2U) {
    decision.reason = StablePathDecisionReason::kNoPreviousPath;
    return decision;
  }
  decision.endpoint_goal_distance_m = distance(previous_path.back(), goal);
  if (decision.endpoint_goal_distance_m > config_.stable_path_goal_tolerance_m) {
    decision.reason = StablePathDecisionReason::kGoalMismatch;
    return decision;
  }

  auto remaining_path = remainingPathFromCurrentPose(
      previous_path, current_position, goal, config_.stable_path_goal_tolerance_m,
      decision.deviation_m);
  if (!remaining_path.has_value()) {
    decision.reason = StablePathDecisionReason::kProjectionUnavailable;
    return decision;
  }

  decision.remaining_path = std::move(*remaining_path);
  if (pathIsTraversable(grid, decision.remaining_path,
                        &decision.prohibited_segment_index)) {
    decision.keep_path = true;
    decision.reason = StablePathDecisionReason::kClear;
    return decision;
  }

  decision.reason = StablePathDecisionReason::kProhibitedConfirmed;
  return decision;
}

} // namespace drone_city_nav
