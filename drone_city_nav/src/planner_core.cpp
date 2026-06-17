#include "drone_city_nav/planner_core.hpp"

#include "drone_city_nav/clearance_field.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceSqM = 1.0e-12;
constexpr double kTurnAngleThresholdRad = 1.0e-3;

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
    case StablePathDecisionReason::kProhibitedUnconfirmed:
      return "prohibited_unconfirmed";
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

double nearestProhibitedDistanceM(const OccupancyGrid2D& grid, const GridIndex cell,
                                  const double max_distance_m) {
  const ClearanceField2D clearance_field = ClearanceField2D::build(
      grid, normalizedClearanceDiagnosticRadiusM(max_distance_m),
      ClearanceSource::kProhibited);
  if (!clearance_field.contains(cell)) {
    return std::numeric_limits<double>::infinity();
  }
  return clearance_field.distanceAt(cell);
}

double pathMinimumProhibitedClearanceM(const OccupancyGrid2D& grid,
                                       const std::span<const GridIndex> path,
                                       const double max_distance_m) {
  const ClearanceField2D clearance_field = ClearanceField2D::build(
      grid, normalizedClearanceDiagnosticRadiusM(max_distance_m),
      ClearanceSource::kProhibited);
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

double pathSegmentProhibitedLengthM(const OccupancyGrid2D& grid, const Point2 start,
                                    const Point2 end) {
  const auto start_cell = grid.worldToCell(start);
  const auto end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    return std::numeric_limits<double>::infinity();
  }

  const std::vector<GridIndex> segment_cells = grid.cellsOnLine(*start_cell, *end_cell);
  const auto prohibited_count = static_cast<double>(
      std::ranges::count_if(segment_cells, [&grid](const GridIndex cell) {
        return grid.isProhibited(cell);
      }));
  return prohibited_count * grid.resolution();
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

bool pathHasProhibitedCells(const OccupancyGrid2D& grid,
                            const std::span<const Point2> path_points,
                            const double stable_path_prohibited_length_m,
                            std::size_t* const prohibited_segment_index,
                            double* const prohibited_length_m) {
  if (path_points.size() < 2U) {
    return false;
  }

  for (std::size_t index = 1U; index < path_points.size(); ++index) {
    const double segment_prohibited_length_m =
        pathSegmentProhibitedLengthM(grid, path_points[index - 1U], path_points[index]);
    if (segment_prohibited_length_m < stable_path_prohibited_length_m) {
      continue;
    }
    if (prohibited_segment_index != nullptr) {
      *prohibited_segment_index = index - 1U;
    }
    if (prohibited_length_m != nullptr) {
      *prohibited_length_m = segment_prohibited_length_m;
    }
    return true;
  }
  return false;
}

bool pathIsAllowed(const OccupancyGrid2D& grid,
                   const std::span<const Point2> path_points,
                   std::size_t* const prohibited_segment_index) {
  if (path_points.size() < 2U) {
    return true;
  }

  for (std::size_t index = 1U; index < path_points.size(); ++index) {
    if (pathSegmentIsAllowed(grid, path_points[index - 1U], path_points[index])) {
      continue;
    }
    if (prohibited_segment_index != nullptr) {
      *prohibited_segment_index = index - 1U;
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

  result.allowed_start_cell =
      grid.nearestAllowed(*result.start_cell, config_.nearest_free_radius_cells);
  result.allowed_goal_cell =
      grid.nearestAllowed(*result.goal_cell, config_.nearest_free_radius_cells);
  if (!result.allowed_start_cell.has_value() || !result.allowed_goal_cell.has_value()) {
    return std::nullopt;
  }

  result.astar = planner_.plan(grid, *result.allowed_start_cell,
                               *result.allowed_goal_cell, config_.astar);
  if (!result.astar.success) {
    return std::nullopt;
  }

  result.smoothed_cells = smoothPath(grid, result.astar.path, config_.smoothing);
  result.grid_stats = collectGridStats(grid);
  result.raw_path_metrics = gridPathMetrics(grid, result.astar.path);
  result.smoothed_path_metrics = gridPathMetrics(grid, result.smoothed_cells);
  result.raw_path_clearance_m = pathMinimumProhibitedClearanceM(
      grid, result.astar.path,
      normalizedClearanceDiagnosticRadiusM(config_.clearance_diagnostic_radius_m));
  result.smoothed_path_clearance_m = pathMinimumProhibitedClearanceM(
      grid, result.smoothed_cells,
      normalizedClearanceDiagnosticRadiusM(config_.clearance_diagnostic_radius_m));
  return result;
}

StablePathDecision
PlannerCore::evaluateStablePath(const OccupancyGrid2D& grid,
                                const std::span<const Point2> previous_path,
                                const Point2 current_position, const Point2 goal,
                                const int current_prohibited_confirmations) const {
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
  const bool has_prohibited_cells = pathHasProhibitedCells(
      grid, decision.remaining_path, config_.stable_path_prohibited_length_m,
      &decision.prohibited_segment_index, &decision.prohibited_length_m);
  if (!has_prohibited_cells) {
    decision.keep_path = true;
    decision.reason = StablePathDecisionReason::kClear;
    decision.prohibited_confirmations = 0;
    return decision;
  }

  decision.prohibited_confirmations = std::max(0, current_prohibited_confirmations) + 1;
  if (decision.prohibited_confirmations <
      config_.stable_path_prohibited_confirmations_required) {
    decision.keep_path = true;
    decision.reason = StablePathDecisionReason::kProhibitedUnconfirmed;
    return decision;
  }

  decision.reason = StablePathDecisionReason::kProhibitedConfirmed;
  return decision;
}

} // namespace drone_city_nav
