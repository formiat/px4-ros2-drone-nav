#include "drone_city_nav/astar_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <queue>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr int kDirectionCount = 8;
constexpr int kDirectionStateCount = kDirectionCount + 1;
constexpr int kStartDirectionState = kDirectionCount;
constexpr std::size_t kNoParent = std::numeric_limits<std::size_t>::max();
// The planner keeps heading as part of the search state so turn penalties can
// prefer flyable paths without losing the physical distance cost.
constexpr std::array<GridIndex, kDirectionCount> kNeighborOffsets{{
    {-1, -1},
    {0, -1},
    {1, -1},
    {-1, 0},
    {1, 0},
    {-1, 1},
    {0, 1},
    {1, 1},
}};

struct OpenNode {
  GridIndex cell{};
  int direction_state{kStartDirectionState};
  RankedPathCost cost{};
  double f_geometry{0.0};
};

struct CompareOpenNode {
  [[nodiscard]] bool operator()(const OpenNode& lhs,
                                const OpenNode& rhs) const noexcept {
    if (!pathRiskEqual(lhs.cost.risk, rhs.cost.risk)) {
      return pathRiskLess(rhs.cost.risk, lhs.cost.risk);
    }
    if (lhs.f_geometry < rhs.f_geometry) {
      return false;
    }
    if (rhs.f_geometry < lhs.f_geometry) {
      return true;
    }
    if (lhs.cost.algorithm_cost != rhs.cost.algorithm_cost) {
      return lhs.cost.algorithm_cost < rhs.cost.algorithm_cost;
    }
    return lhs.cost.deterministic_tiebreak > rhs.cost.deterministic_tiebreak;
  }
};

[[nodiscard]] double heuristic(const GridIndex from, const GridIndex to,
                               const double resolution_m) noexcept {
  const double dx = static_cast<double>(from.x - to.x);
  const double dy = static_cast<double>(from.y - to.y);
  return std::hypot(dx, dy) * resolution_m;
}

[[nodiscard]] double weightedHeuristic(const GridIndex from, const GridIndex to,
                                       const double resolution_m,
                                       const AStarConfig& config) noexcept {
  const double weight = std::isfinite(config.heuristic_weight)
                            ? std::max(1.0, config.heuristic_weight)
                            : 1.0;
  return weight * heuristic(from, to, resolution_m);
}

[[nodiscard]] double stepDistanceM(const GridIndex offset,
                                   const double resolution_m) noexcept {
  const double cell_distance =
      (offset.x != 0 && offset.y != 0) ? std::numbers::sqrt2 : 1.0;
  return cell_distance * resolution_m;
}

[[nodiscard]] std::size_t stateIndex(const OccupancyGrid2D& grid, const GridIndex cell,
                                     const int direction_state) noexcept {
  return grid.linearIndex(cell) * static_cast<std::size_t>(kDirectionStateCount) +
         static_cast<std::size_t>(direction_state);
}

[[nodiscard]] GridIndex cellFromStateIndex(const OccupancyGrid2D& grid,
                                           const std::size_t state_index) noexcept {
  const std::size_t cell_index =
      state_index / static_cast<std::size_t>(kDirectionStateCount);
  const auto width = static_cast<std::size_t>(grid.width());
  return GridIndex{static_cast<int>(cell_index % width),
                   static_cast<int>(cell_index / width)};
}

[[nodiscard]] bool cellIsHardBlocked(const OccupancyGrid2D& grid,
                                     const ObstacleRiskField& risk_field,
                                     const GridIndex cell) {
  return !grid.contains(cell) || grid.isOccupied(cell) ||
         !risk_field.containsEvaluationPoint(grid.cellCenter(cell));
}

[[nodiscard]] bool diagonalMoveCutsRawCorner(const OccupancyGrid2D& grid,
                                             const ObstacleRiskField& risk_field,
                                             const GridIndex from, const GridIndex to) {
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  if (std::abs(dx) != 1 || std::abs(dy) != 1) {
    return false;
  }

  const GridIndex adjacent_x{from.x + dx, from.y};
  const GridIndex adjacent_y{from.x, from.y + dy};
  return cellIsHardBlocked(grid, risk_field, adjacent_x) ||
         cellIsHardBlocked(grid, risk_field, adjacent_y);
}

[[nodiscard]] PathRiskScore startRisk(const ObstacleRiskField& field,
                                      const GridIndex cell) {
  PathRiskScore risk{};
  risk.worst_tier = field.tierAt(cell);
  risk.minimum_raw_clearance_m = field.occupiedClearance().distanceAt(cell);
  return risk;
}

[[nodiscard]] PathRiskScore extendRisk(const PathRiskScore& current,
                                       const ObstacleRiskField& field,
                                       const GridIndex cell,
                                       const double step_distance_m) {
  PathRiskScore next = current;
  const ObstacleRiskTier tier = field.tierAt(cell);
  if (static_cast<std::uint8_t>(tier) > static_cast<std::uint8_t>(next.worst_tier)) {
    next.worst_tier = tier;
  }
  if (tier == ObstacleRiskTier::kCriticalBand) {
    next.critical_exposure_m += step_distance_m;
  } else if (tier == ObstacleRiskTier::kPlanningBand) {
    next.planning_exposure_m += step_distance_m;
  }
  next.minimum_raw_clearance_m = std::min(next.minimum_raw_clearance_m,
                                          field.occupiedClearance().distanceAt(cell));
  return next;
}

[[nodiscard]] double directionPreferenceCost(const AStarConfig& config,
                                             const int current_direction_state,
                                             const int next_direction_state) noexcept {
  if (current_direction_state == kStartDirectionState) {
    return 0.0;
  }

  if (config.evasive_maneuvering_enabled) {
    if (!(config.evasive_maneuvering_straight_cost_weight > 0.0) ||
        current_direction_state != next_direction_state) {
      return 0.0;
    }
    return config.evasive_maneuvering_straight_cost_weight;
  }

  if (!(config.turn_cost_weight > 0.0) ||
      current_direction_state == next_direction_state) {
    return 0.0;
  }

  return config.turn_cost_weight;
}

[[nodiscard]] double initialHeadingBiasCost(const AStarConfig& config,
                                            const int current_direction_state,
                                            const GridIndex offset) noexcept {
  if (!config.initial_heading_bias_enabled ||
      current_direction_state != kStartDirectionState ||
      !(config.initial_heading_bias_weight > 0.0)) {
    return 0.0;
  }

  const double speed_mps = std::hypot(config.initial_heading_bias_velocity_x_mps,
                                      config.initial_heading_bias_velocity_y_mps);
  if (!std::isfinite(speed_mps) ||
      speed_mps < std::max(0.0, config.initial_heading_bias_min_speed_mps)) {
    return 0.0;
  }

  const double move_norm =
      std::hypot(static_cast<double>(offset.x), static_cast<double>(offset.y));
  if (!(move_norm > 0.0)) {
    return 0.0;
  }

  const double velocity_x = config.initial_heading_bias_velocity_x_mps / speed_mps;
  const double velocity_y = config.initial_heading_bias_velocity_y_mps / speed_mps;
  const double move_x = static_cast<double>(offset.x) / move_norm;
  const double move_y = static_cast<double>(offset.y) / move_norm;
  const double alignment =
      std::clamp(move_x * velocity_x + move_y * velocity_y, -1.0, 1.0);
  return config.initial_heading_bias_weight * (1.0 - alignment);
}

[[nodiscard]] std::vector<GridIndex>
reconstructPath(const OccupancyGrid2D& grid, const std::vector<std::size_t>& parents,
                const std::size_t start_state_index,
                const std::size_t goal_state_index) {
  std::vector<GridIndex> path;
  std::size_t current_state_index = goal_state_index;

  while (current_state_index != kNoParent) {
    path.push_back(cellFromStateIndex(grid, current_state_index));
    if (current_state_index == start_state_index) {
      break;
    }
    current_state_index = parents.at(current_state_index);
    if (current_state_index == kNoParent) {
      return {};
    }
  }

  std::reverse(path.begin(), path.end());
  return path;
}

} // namespace

AStarConfig astarConfigForRiskField(AStarConfig base,
                                    const ObstacleRiskField& risk_field) {
  base.risk_policy = risk_field.policy();
  return base;
}

const char* astarStatusName(const AStarStatus status) noexcept {
  switch (status) {
    case AStarStatus::kSuccess:
      return "success";
    case AStarStatus::kInvalidStartOrGoal:
      return "invalid_start_or_goal";
    case AStarStatus::kRawOccupiedStartOrGoal:
      return "raw_occupied_start_or_goal";
    case AStarStatus::kUnreachable:
      return "unreachable";
    case AStarStatus::kStateSpaceTooLarge:
      return "state_space_too_large";
    case AStarStatus::kCanceled:
      return "canceled";
  }
  return "unknown";
}

AStarResult AStarPlanner::plan(const OccupancyGrid2D& grid, const GridIndex start,
                               const GridIndex goal, const AStarConfig& config,
                               const std::stop_token stop_token) const {
  const ObstacleRiskField risk_field =
      ObstacleRiskField::build(grid, config.risk_policy);
  return plan(grid, start, goal, risk_field, config, stop_token);
}

AStarResult AStarPlanner::plan(const OccupancyGrid2D& grid, const GridIndex start,
                               const GridIndex goal,
                               const ObstacleRiskField& risk_field,
                               const AStarConfig& config,
                               const std::stop_token stop_token) const {
  AStarResult result{};
  if (stop_token.stop_requested()) {
    result.status = AStarStatus::kCanceled;
    return result;
  }
  if (!grid.contains(start) || !grid.contains(goal)) {
    result.status = AStarStatus::kInvalidStartOrGoal;
    return result;
  }
  if (!risk_field.containsEvaluationPoint(grid.cellCenter(start)) ||
      !risk_field.containsEvaluationPoint(grid.cellCenter(goal))) {
    result.status = AStarStatus::kInvalidStartOrGoal;
    return result;
  }
  if (grid.isOccupied(start) || grid.isOccupied(goal)) {
    result.status = AStarStatus::kRawOccupiedStartOrGoal;
    return result;
  }

  const std::size_t direction_states = static_cast<std::size_t>(kDirectionStateCount);
  if (grid.cellCount() > std::numeric_limits<std::size_t>::max() / direction_states) {
    result.status = AStarStatus::kStateSpaceTooLarge;
    return result;
  }

  const std::size_t state_count = grid.cellCount() * direction_states;
  std::vector<RankedPathCost> g_scores(
      state_count,
      RankedPathCost{
          .risk =
              PathRiskScore{
                  .outside_bounds = true,
                  .intersects_raw_occupied = true,
                  .worst_tier = ObstacleRiskTier::kCriticalBand,
                  .critical_exposure_m = std::numeric_limits<double>::infinity(),
                  .planning_exposure_m = std::numeric_limits<double>::infinity(),
              },
          .algorithm_cost = std::numeric_limits<double>::infinity(),
          .deterministic_tiebreak = std::numeric_limits<std::uint64_t>::max(),
      });
  std::vector<std::size_t> parents(state_count, kNoParent);
  std::vector<std::uint8_t> closed(state_count, 0U);
  std::priority_queue<OpenNode, std::vector<OpenNode>, CompareOpenNode> open;

  const std::size_t start_index = stateIndex(grid, start, kStartDirectionState);
  g_scores[start_index] = RankedPathCost{
      .risk = startRisk(risk_field, start),
      .algorithm_cost = 0.0,
      .deterministic_tiebreak = 0U,
  };
  open.push(OpenNode{
      .cell = start,
      .direction_state = kStartDirectionState,
      .cost = g_scores[start_index],
      .f_geometry = weightedHeuristic(start, goal, grid.resolution(), config),
  });

  while (!open.empty()) {
    if (stop_token.stop_requested()) {
      result.status = AStarStatus::kCanceled;
      return result;
    }
    const OpenNode current = open.top();
    open.pop();

    const std::size_t current_index =
        stateIndex(grid, current.cell, current.direction_state);
    if (closed[current_index] != 0U) {
      continue;
    }
    closed[current_index] = 1U;
    ++result.expanded_cells;

    if (current.cell == goal) {
      result.path = reconstructPath(grid, parents, start_index, current_index);
      result.success = !result.path.empty();
      result.status =
          result.success ? AStarStatus::kSuccess : AStarStatus::kUnreachable;
      result.total_cost = current.cost.algorithm_cost;
      std::vector<Point2> path_points;
      path_points.reserve(result.path.size());
      for (const GridIndex cell : result.path) {
        path_points.push_back(grid.cellCenter(cell));
      }
      result.risk = risk_field.evaluate(grid, path_points);
      return result;
    }

    for (std::size_t direction_index = 0U; direction_index < kNeighborOffsets.size();
         ++direction_index) {
      const GridIndex offset = kNeighborOffsets.at(direction_index);
      const GridIndex next{current.cell.x + offset.x, current.cell.y + offset.y};
      if (cellIsHardBlocked(grid, risk_field, next) ||
          diagonalMoveCutsRawCorner(grid, risk_field, current.cell, next)) {
        continue;
      }

      const int next_direction_state = static_cast<int>(direction_index);
      const std::size_t next_index = stateIndex(grid, next, next_direction_state);
      if (closed[next_index] != 0U) {
        continue;
      }

      const double step_distance_m = stepDistanceM(offset, grid.resolution());
      RankedPathCost tentative = g_scores[current_index];
      tentative.risk = extendRisk(tentative.risk, risk_field, next, step_distance_m);
      tentative.algorithm_cost +=
          step_distance_m +
          directionPreferenceCost(config, current.direction_state,
                                  next_direction_state) +
          initialHeadingBiasCost(config, current.direction_state, offset);
      tentative.deterministic_tiebreak = static_cast<std::uint64_t>(next_index);
      if (!rankedPathCostLess(tentative, g_scores[next_index])) {
        continue;
      }

      parents[next_index] = current_index;
      g_scores[next_index] = tentative;
      open.push(OpenNode{
          .cell = next,
          .direction_state = next_direction_state,
          .cost = tentative,
          .f_geometry = tentative.algorithm_cost +
                        weightedHeuristic(next, goal, grid.resolution(), config),
      });
    }
  }

  result.status = AStarStatus::kUnreachable;
  return result;
}

} // namespace drone_city_nav
