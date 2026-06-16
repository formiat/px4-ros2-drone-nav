#include "drone_city_nav/planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceSqM = 1.0e-12;
constexpr double kTurnAngleThresholdRad = 1.0e-3;
constexpr double kDetourLengthToleranceM = 1.0e-6;

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

[[nodiscard]] bool directionChanged(const UnitDirection2D previous,
                                    const UnitDirection2D current) noexcept {
  const double dot =
      std::clamp(previous.x * current.x + previous.y * current.y, -1.0, 1.0);
  return std::acos(dot) > kTurnAngleThresholdRad;
}

[[nodiscard]] bool hasComfortCost(const AStarConfig& config) noexcept {
  return config.obstacle_clearance_cost_weight > 0.0 || config.turn_cost_weight > 0.0 ||
         (config.evasive_maneuvering_enabled &&
          config.evasive_maneuvering_straight_cost_weight > 0.0);
}

[[nodiscard]] AStarConfig shortestPathConfig(AStarConfig config) noexcept {
  config.obstacle_clearance_cost_radius_m = 0.0;
  config.obstacle_clearance_cost_weight = 0.0;
  config.turn_cost_weight = 0.0;
  config.evasive_maneuvering_enabled = false;
  config.evasive_maneuvering_straight_cost_weight = 0.0;
  return config;
}

[[nodiscard]] double detourLengthLimitM(const double shortest_length_m,
                                        const double max_detour_ratio) noexcept {
  if (!std::isfinite(shortest_length_m) || shortest_length_m < 0.0 ||
      !(max_detour_ratio > 0.0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return shortest_length_m * (1.0 + max_detour_ratio);
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
    case StablePathDecisionReason::kDeviationTooLarge:
      return "deviation_too_large";
    case StablePathDecisionReason::kClear:
      return "clear";
    case StablePathDecisionReason::kBlockedUnconfirmed:
      return "blocked_unconfirmed";
    case StablePathDecisionReason::kBlockedConfirmed:
      return "blocked_confirmed";
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

double nearestBlockedDistanceM(const OccupancyGrid2D& grid, const GridIndex cell,
                               const double max_distance_m) {
  if (!(grid.resolution() > 0.0)) {
    return std::numeric_limits<double>::infinity();
  }

  const int radius_cells = static_cast<int>(std::ceil(
      normalizedClearanceDiagnosticRadiusM(max_distance_m) / grid.resolution()));
  double nearest_distance_m = std::numeric_limits<double>::infinity();
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const GridIndex candidate{cell.x + dx, cell.y + dy};
      if (!grid.contains(candidate) || !grid.isBlocked(candidate)) {
        continue;
      }
      nearest_distance_m =
          std::min(nearest_distance_m,
                   distance(grid.cellCenter(cell), grid.cellCenter(candidate)));
    }
  }

  return nearest_distance_m;
}

double pathMinimumBlockedClearanceM(const OccupancyGrid2D& grid,
                                    const std::span<const GridIndex> path,
                                    const double max_distance_m) {
  double minimum_clearance_m = std::numeric_limits<double>::infinity();
  for (const GridIndex cell : path) {
    if (!grid.contains(cell)) {
      continue;
    }
    minimum_clearance_m = std::min(minimum_clearance_m,
                                   nearestBlockedDistanceM(grid, cell, max_distance_m));
  }
  return minimum_clearance_m;
}

bool pathSegmentIsUnblocked(const OccupancyGrid2D& grid, const Point2 start,
                            const Point2 end) {
  const auto start_cell = grid.worldToCell(start);
  const auto end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    return false;
  }

  return hasLineOfSight(grid, *start_cell, *end_cell);
}

double pathSegmentOccupiedLengthM(const OccupancyGrid2D& grid, const Point2 start,
                                  const Point2 end) {
  const auto start_cell = grid.worldToCell(start);
  const auto end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    return std::numeric_limits<double>::infinity();
  }

  const std::vector<GridIndex> segment_cells = grid.cellsOnLine(*start_cell, *end_cell);
  const auto occupied_count = static_cast<double>(std::ranges::count_if(
      segment_cells, [&grid](const GridIndex cell) { return grid.isOccupied(cell); }));
  return occupied_count * grid.resolution();
}

double pathSegmentBlockedLengthM(const OccupancyGrid2D& grid, const Point2 start,
                                 const Point2 end) {
  const auto start_cell = grid.worldToCell(start);
  const auto end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    return std::numeric_limits<double>::infinity();
  }

  const std::vector<GridIndex> segment_cells = grid.cellsOnLine(*start_cell, *end_cell);
  const auto blocked_count = static_cast<double>(std::ranges::count_if(
      segment_cells, [&grid](const GridIndex cell) { return grid.isBlocked(cell); }));
  return blocked_count * grid.resolution();
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
    const Point2 goal, const double stable_path_goal_tolerance_m,
    const double stable_path_reuse_max_deviation_m, double& deviation_m) {
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
  if (deviation_m > stable_path_reuse_max_deviation_m) {
    return std::nullopt;
  }

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

bool pathHasOccupiedCells(const OccupancyGrid2D& grid,
                          const std::span<const Point2> path_points,
                          const double stable_path_blocking_occupied_length_m,
                          std::size_t* const blocking_segment_index,
                          double* const blocking_occupied_length_m) {
  if (path_points.size() < 2U) {
    return false;
  }

  for (std::size_t index = 1U; index < path_points.size(); ++index) {
    const double blocked_length_m =
        pathSegmentBlockedLengthM(grid, path_points[index - 1U], path_points[index]);
    if (blocked_length_m < stable_path_blocking_occupied_length_m) {
      continue;
    }
    if (blocking_segment_index != nullptr) {
      *blocking_segment_index = index - 1U;
    }
    if (blocking_occupied_length_m != nullptr) {
      *blocking_occupied_length_m = blocked_length_m;
    }
    return true;
  }
  return false;
}

bool pathIsUnblocked(const OccupancyGrid2D& grid,
                     const std::span<const Point2> path_points,
                     std::size_t* const blocked_segment_index) {
  if (path_points.size() < 2U) {
    return true;
  }

  for (std::size_t index = 1U; index < path_points.size(); ++index) {
    if (pathSegmentIsUnblocked(grid, path_points[index - 1U], path_points[index])) {
      continue;
    }
    if (blocked_segment_index != nullptr) {
      *blocked_segment_index = index - 1U;
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
  PathComputationResult result{};
  result.start_cell = grid.worldToCell(current_position);
  result.goal_cell = grid.worldToCell(goal);
  if (!result.start_cell.has_value() || !result.goal_cell.has_value()) {
    return std::nullopt;
  }

  result.unblocked_start_cell =
      grid.nearestUnblocked(*result.start_cell, config_.nearest_free_radius_cells);
  result.unblocked_goal_cell =
      grid.nearestUnblocked(*result.goal_cell, config_.nearest_free_radius_cells);
  if (!result.unblocked_start_cell.has_value() ||
      !result.unblocked_goal_cell.has_value()) {
    return std::nullopt;
  }

  const bool detour_limited =
      config_.comfort_path_max_detour_ratio > 0.0 && hasComfortCost(config_.astar);
  if (detour_limited) {
    const AStarResult shortest_path =
        planner_.plan(grid, *result.unblocked_start_cell, *result.unblocked_goal_cell,
                      shortestPathConfig(config_.astar));
    if (!shortest_path.success) {
      return std::nullopt;
    }

    result.comfort_path_detour_limited = true;
    result.shortest_path_length_m = gridPathMetrics(grid, shortest_path.path).length_m;
    result.comfort_path_length_limit_m = detourLengthLimitM(
        result.shortest_path_length_m, config_.comfort_path_max_detour_ratio);

    const AStarResult comfort_path = planner_.plan(
        grid, *result.unblocked_start_cell, *result.unblocked_goal_cell, config_.astar);
    result.comfort_path_length_m =
        comfort_path.success ? gridPathMetrics(grid, comfort_path.path).length_m
                             : std::numeric_limits<double>::infinity();

    if (comfort_path.success &&
        result.comfort_path_length_m <=
            result.comfort_path_length_limit_m + kDetourLengthToleranceM) {
      result.astar = comfort_path;
      result.comfort_path_selected = true;
    } else {
      result.astar = shortest_path;
    }
  } else {
    result.astar = planner_.plan(grid, *result.unblocked_start_cell,
                                 *result.unblocked_goal_cell, config_.astar);
    if (!result.astar.success) {
      return std::nullopt;
    }
  }

  result.smoothed_cells = smoothPath(grid, result.astar.path, config_.smoothing);
  result.grid_stats = collectGridStats(grid);
  result.raw_path_metrics = gridPathMetrics(grid, result.astar.path);
  result.smoothed_path_metrics = gridPathMetrics(grid, result.smoothed_cells);
  result.raw_path_clearance_m = pathMinimumBlockedClearanceM(
      grid, result.astar.path,
      normalizedClearanceDiagnosticRadiusM(config_.clearance_diagnostic_radius_m));
  result.smoothed_path_clearance_m = pathMinimumBlockedClearanceM(
      grid, result.smoothed_cells,
      normalizedClearanceDiagnosticRadiusM(config_.clearance_diagnostic_radius_m));
  return result;
}

StablePathDecision
PlannerCore::evaluateStablePath(const OccupancyGrid2D& grid,
                                const std::span<const Point2> previous_path,
                                const Point2 current_position, const Point2 goal,
                                const int current_blocked_confirmations) const {
  StablePathDecision decision{};
  if (previous_path.size() < 2U) {
    decision.reason = StablePathDecisionReason::kNoPreviousPath;
    return decision;
  }
  if (distance(previous_path.back(), goal) > config_.stable_path_goal_tolerance_m) {
    decision.reason = StablePathDecisionReason::kGoalMismatch;
    return decision;
  }

  auto remaining_path = remainingPathFromCurrentPose(
      previous_path, current_position, goal, config_.stable_path_goal_tolerance_m,
      config_.stable_path_reuse_max_deviation_m, decision.deviation_m);
  if (!remaining_path.has_value()) {
    decision.reason =
        std::isfinite(decision.deviation_m) &&
                decision.deviation_m > config_.stable_path_reuse_max_deviation_m
            ? StablePathDecisionReason::kDeviationTooLarge
            : StablePathDecisionReason::kProjectionUnavailable;
    return decision;
  }

  decision.remaining_path = std::move(*remaining_path);
  const bool occupied = pathHasOccupiedCells(
      grid, decision.remaining_path, config_.stable_path_blocking_occupied_length_m,
      &decision.blocking_segment_index, &decision.blocking_occupied_length_m);
  if (!occupied) {
    decision.keep_path = true;
    decision.reason = StablePathDecisionReason::kClear;
    decision.blocked_confirmations = 0;
    return decision;
  }

  decision.blocked_confirmations = std::max(0, current_blocked_confirmations) + 1;
  if (decision.blocked_confirmations <
      config_.stable_path_blocked_confirmations_required) {
    decision.keep_path = true;
    decision.reason = StablePathDecisionReason::kBlockedUnconfirmed;
    return decision;
  }

  decision.reason = StablePathDecisionReason::kBlockedConfirmed;
  return decision;
}

} // namespace drone_city_nav
