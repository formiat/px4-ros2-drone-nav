#include "drone_city_nav/astar_planner.hpp"

#include "drone_city_nav/clearance_field.hpp"

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
  double f_score{0.0};
  double g_score{0.0};
};

struct CompareOpenNode {
  [[nodiscard]] bool operator()(const OpenNode& lhs,
                                const OpenNode& rhs) const noexcept {
    if (lhs.f_score == rhs.f_score) {
      return lhs.g_score < rhs.g_score;
    }
    return lhs.f_score > rhs.f_score;
  }
};

[[nodiscard]] double heuristic(const GridIndex from, const GridIndex to,
                               const double resolution_m) noexcept {
  const double dx = static_cast<double>(from.x - to.x);
  const double dy = static_cast<double>(from.y - to.y);
  return std::hypot(dx, dy) * resolution_m;
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

[[nodiscard]] bool diagonalMoveCutsProhibitedCorner(const OccupancyGrid2D& grid,
                                                    const GridIndex from,
                                                    const GridIndex to) {
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  if (std::abs(dx) != 1 || std::abs(dy) != 1) {
    return false;
  }

  const GridIndex adjacent_x{from.x + dx, from.y};
  const GridIndex adjacent_y{from.x, from.y + dy};
  return grid.isProhibited(adjacent_x) || grid.isProhibited(adjacent_y);
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

[[nodiscard]] double clearanceCost(const ClearanceField2D& field,
                                   const AStarConfig& config, const GridIndex cell,
                                   const double step_distance_m) {
  if (field.maxDistanceM() <= 0.0 || !(config.obstacle_clearance_cost_weight > 0.0)) {
    return 0.0;
  }

  const double distance_m = field.distanceAt(cell);
  if (!std::isfinite(distance_m)) {
    return 0.0;
  }

  const double comfort_radius_m = config.obstacle_clearance_cost_radius_m;
  if (!(comfort_radius_m > 0.0) || distance_m >= comfort_radius_m) {
    return 0.0;
  }

  const double normalized_proximity =
      std::clamp((comfort_radius_m - distance_m) / comfort_radius_m, 0.0, 1.0);
  return config.obstacle_clearance_cost_weight * normalized_proximity * step_distance_m;
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

const char* astarStatusName(const AStarStatus status) noexcept {
  switch (status) {
    case AStarStatus::kSuccess:
      return "success";
    case AStarStatus::kInvalidStartOrGoal:
      return "invalid_start_or_goal";
    case AStarStatus::kProhibitedStartOrGoal:
      return "prohibited_start_or_goal";
    case AStarStatus::kUnreachable:
      return "unreachable";
    case AStarStatus::kExpansionBudgetExceeded:
      return "expansion_budget_exceeded";
    case AStarStatus::kStateSpaceTooLarge:
      return "state_space_too_large";
  }
  return "unknown";
}

AStarResult AStarPlanner::plan(const OccupancyGrid2D& grid, const GridIndex start,
                               const GridIndex goal, const AStarConfig& config) const {
  AStarResult result{};
  if (!grid.contains(start) || !grid.contains(goal)) {
    result.status = AStarStatus::kInvalidStartOrGoal;
    return result;
  }
  if (grid.isProhibited(start) || grid.isProhibited(goal)) {
    result.status = AStarStatus::kProhibitedStartOrGoal;
    return result;
  }

  const std::size_t direction_states = static_cast<std::size_t>(kDirectionStateCount);
  if (grid.cellCount() > std::numeric_limits<std::size_t>::max() / direction_states) {
    result.status = AStarStatus::kStateSpaceTooLarge;
    return result;
  }

  const ClearanceField2D clearance_field = ClearanceField2D::build(
      grid, config.obstacle_clearance_cost_radius_m, ClearanceSource::kOccupied);
  const std::size_t state_count = grid.cellCount() * direction_states;
  std::vector<double> g_scores(state_count, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parents(state_count, kNoParent);
  std::vector<std::uint8_t> closed(state_count, 0U);
  std::priority_queue<OpenNode, std::vector<OpenNode>, CompareOpenNode> open;

  const std::size_t start_index = stateIndex(grid, start, kStartDirectionState);
  g_scores[start_index] = 0.0;
  open.push(OpenNode{start, kStartDirectionState,
                     heuristic(start, goal, grid.resolution()), 0.0});

  while (!open.empty() && result.expanded_cells < config.max_expansions) {
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
      result.total_cost = current.g_score;
      return result;
    }

    for (std::size_t direction_index = 0U; direction_index < kNeighborOffsets.size();
         ++direction_index) {
      const GridIndex offset = kNeighborOffsets.at(direction_index);
      const GridIndex next{current.cell.x + offset.x, current.cell.y + offset.y};
      if (!grid.contains(next) || grid.isProhibited(next) ||
          diagonalMoveCutsProhibitedCorner(grid, current.cell, next)) {
        continue;
      }

      const int next_direction_state = static_cast<int>(direction_index);
      const std::size_t next_index = stateIndex(grid, next, next_direction_state);
      if (closed[next_index] != 0U) {
        continue;
      }

      const double step_distance_m = stepDistanceM(offset, grid.resolution());
      const double tentative_g =
          g_scores[current_index] + step_distance_m +
          clearanceCost(clearance_field, config, next, step_distance_m) +
          directionPreferenceCost(config, current.direction_state,
                                  next_direction_state);
      if (tentative_g >= g_scores[next_index]) {
        continue;
      }

      parents[next_index] = current_index;
      g_scores[next_index] = tentative_g;
      const double f_score = tentative_g + heuristic(next, goal, grid.resolution());
      open.push(OpenNode{next, next_direction_state, f_score, tentative_g});
    }
  }

  if (!open.empty() && result.expanded_cells >= config.max_expansions) {
    result.status = AStarStatus::kExpansionBudgetExceeded;
  } else {
    result.status = AStarStatus::kUnreachable;
  }
  return result;
}

} // namespace drone_city_nav
