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

constexpr int kNoBlockedCellInRange = std::numeric_limits<int>::max();
constexpr int kDirectionCount = 8;
constexpr int kDirectionStateCount = kDirectionCount + 1;
constexpr int kStartDirectionState = kDirectionCount;
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

struct ClearanceField {
  int radius_cells{0};
  std::vector<int> distance_cells;
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

[[nodiscard]] double heuristic(const GridIndex from, const GridIndex to) noexcept {
  const double dx = static_cast<double>(from.x - to.x);
  const double dy = static_cast<double>(from.y - to.y);
  return std::hypot(dx, dy);
}

[[nodiscard]] std::size_t stateIndex(const OccupancyGrid2D& grid, const GridIndex cell,
                                     const int direction_state) noexcept {
  return grid.linearIndex(cell) * static_cast<std::size_t>(kDirectionStateCount) +
         static_cast<std::size_t>(direction_state);
}

[[nodiscard]] GridIndex cellFromStateIndex(const OccupancyGrid2D& grid,
                                           const int state_index) noexcept {
  const int cell_index = state_index / kDirectionStateCount;
  return GridIndex{cell_index % grid.width(), cell_index / grid.width()};
}

[[nodiscard]] bool diagonalMoveCutsBlockedCorner(const OccupancyGrid2D& grid,
                                                 const GridIndex from,
                                                 const GridIndex to) {
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  if (std::abs(dx) != 1 || std::abs(dy) != 1) {
    return false;
  }

  const GridIndex adjacent_x{from.x + dx, from.y};
  const GridIndex adjacent_y{from.x, from.y + dy};
  return grid.isBlocked(adjacent_x) || grid.isBlocked(adjacent_y);
}

[[nodiscard]] int clearanceRadiusCells(const OccupancyGrid2D& grid,
                                       const AStarConfig& config) noexcept {
  if (!(config.obstacle_clearance_cost_radius_m > 0.0) ||
      !(config.obstacle_clearance_cost_weight > 0.0) || !(grid.resolution() > 0.0)) {
    return 0;
  }
  return std::max(0, static_cast<int>(std::ceil(
                         config.obstacle_clearance_cost_radius_m / grid.resolution())));
}

[[nodiscard]] ClearanceField buildClearanceField(const OccupancyGrid2D& grid,
                                                 const int radius_cells) {
  ClearanceField field{};
  field.radius_cells = std::max(0, radius_cells);
  field.distance_cells.assign(grid.cellCount(), kNoBlockedCellInRange);
  if (field.radius_cells == 0) {
    return field;
  }

  std::queue<GridIndex> queue;
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const GridIndex cell{x, y};
      if (!grid.isBlocked(cell)) {
        continue;
      }
      field.distance_cells[grid.linearIndex(cell)] = 0;
      queue.push(cell);
    }
  }

  while (!queue.empty()) {
    const GridIndex current = queue.front();
    queue.pop();
    const int current_distance = field.distance_cells.at(grid.linearIndex(current));
    if (current_distance >= field.radius_cells) {
      continue;
    }

    for (const GridIndex offset : kNeighborOffsets) {
      const GridIndex next{current.x + offset.x, current.y + offset.y};
      if (!grid.contains(next)) {
        continue;
      }
      const std::size_t next_index = grid.linearIndex(next);
      if (field.distance_cells[next_index] <= current_distance + 1) {
        continue;
      }
      field.distance_cells[next_index] = current_distance + 1;
      queue.push(next);
    }
  }

  return field;
}

[[nodiscard]] double turnCost(const AStarConfig& config,
                              const int current_direction_state,
                              const int next_direction_state) noexcept {
  if (!(config.turn_cost_weight > 0.0) ||
      current_direction_state == kStartDirectionState ||
      current_direction_state == next_direction_state) {
    return 0.0;
  }

  return config.turn_cost_weight;
}

[[nodiscard]] double clearanceCost(const OccupancyGrid2D& grid,
                                   const ClearanceField& field,
                                   const AStarConfig& config, const GridIndex cell) {
  if (field.radius_cells <= 0 || !(config.obstacle_clearance_cost_weight > 0.0)) {
    return 0.0;
  }

  const int distance_cells = field.distance_cells.at(grid.linearIndex(cell));
  if (distance_cells == kNoBlockedCellInRange || distance_cells > field.radius_cells) {
    return 0.0;
  }

  const double normalized_proximity =
      static_cast<double>(field.radius_cells + 1 - distance_cells) /
      static_cast<double>(field.radius_cells + 1);
  return config.obstacle_clearance_cost_weight * normalized_proximity;
}

[[nodiscard]] std::vector<GridIndex> reconstructPath(const OccupancyGrid2D& grid,
                                                     const std::vector<int>& parents,
                                                     const int start_state_index,
                                                     const int goal_state_index) {
  std::vector<GridIndex> path;
  int current_state_index = goal_state_index;

  while (current_state_index >= 0) {
    path.push_back(cellFromStateIndex(grid, current_state_index));
    if (current_state_index == start_state_index) {
      break;
    }
    current_state_index = parents.at(static_cast<std::size_t>(current_state_index));
    if (current_state_index < 0) {
      return {};
    }
  }

  std::reverse(path.begin(), path.end());
  return path;
}

} // namespace

AStarResult AStarPlanner::plan(const OccupancyGrid2D& grid, const GridIndex start,
                               const GridIndex goal, const AStarConfig& config) const {
  AStarResult result{};
  if (!grid.contains(start) || !grid.contains(goal) || grid.isBlocked(start) ||
      grid.isBlocked(goal)) {
    return result;
  }

  const ClearanceField clearance_field =
      buildClearanceField(grid, clearanceRadiusCells(grid, config));
  const std::size_t state_count =
      grid.cellCount() * static_cast<std::size_t>(kDirectionStateCount);
  std::vector<double> g_scores(state_count, std::numeric_limits<double>::infinity());
  std::vector<int> parents(state_count, -1);
  std::vector<std::uint8_t> closed(state_count, 0U);
  std::priority_queue<OpenNode, std::vector<OpenNode>, CompareOpenNode> open;

  const std::size_t start_index = stateIndex(grid, start, kStartDirectionState);
  g_scores[start_index] = 0.0;
  open.push(OpenNode{start, kStartDirectionState, heuristic(start, goal), 0.0});

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
      result.path = reconstructPath(grid, parents, static_cast<int>(start_index),
                                    static_cast<int>(current_index));
      result.success = !result.path.empty();
      return result;
    }

    for (std::size_t direction_index = 0U; direction_index < kNeighborOffsets.size();
         ++direction_index) {
      const GridIndex offset = kNeighborOffsets.at(direction_index);
      const GridIndex next{current.cell.x + offset.x, current.cell.y + offset.y};
      if (!grid.contains(next) || grid.isBlocked(next) ||
          diagonalMoveCutsBlockedCorner(grid, current.cell, next)) {
        continue;
      }

      const int next_direction_state = static_cast<int>(direction_index);
      const std::size_t next_index = stateIndex(grid, next, next_direction_state);
      if (closed[next_index] != 0U) {
        continue;
      }

      const double step_cost =
          (offset.x != 0 && offset.y != 0) ? std::numbers::sqrt2 : 1.0;
      const double tentative_g =
          g_scores[current_index] + step_cost +
          clearanceCost(grid, clearance_field, config, next) +
          turnCost(config, current.direction_state, next_direction_state);
      if (tentative_g >= g_scores[next_index]) {
        continue;
      }

      parents[next_index] = static_cast<int>(current_index);
      g_scores[next_index] = tentative_g;
      const double f_score = tentative_g + heuristic(next, goal);
      open.push(OpenNode{next, next_direction_state, f_score, tentative_g});
    }
  }

  return result;
}

} // namespace drone_city_nav
