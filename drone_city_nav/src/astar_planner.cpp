#include "drone_city_nav/astar_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>

namespace drone_city_nav {
namespace {

struct OpenNode {
  GridIndex cell{};
  double f_score{0.0};
  double g_score{0.0};
};

struct CompareOpenNode {
  [[nodiscard]] bool operator()(const OpenNode &lhs,
                                const OpenNode &rhs) const noexcept {
    if (lhs.f_score == rhs.f_score) {
      return lhs.g_score < rhs.g_score;
    }
    return lhs.f_score > rhs.f_score;
  }
};

[[nodiscard]] double heuristic(const GridIndex from,
                               const GridIndex to) noexcept {
  const double dx = static_cast<double>(from.x - to.x);
  const double dy = static_cast<double>(from.y - to.y);
  return std::hypot(dx, dy);
}

[[nodiscard]] bool diagonalMoveCutsBlockedCorner(const OccupancyGrid2D &grid,
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

[[nodiscard]] std::vector<GridIndex>
reconstructPath(const OccupancyGrid2D &grid, const std::vector<int> &parents,
                const GridIndex start, const GridIndex goal) {
  std::vector<GridIndex> path;
  GridIndex current = goal;
  path.push_back(current);

  while (current != start) {
    const int parent_index = parents.at(grid.linearIndex(current));
    if (parent_index < 0) {
      return {};
    }
    const int width = grid.width();
    current = GridIndex{parent_index % width, parent_index / width};
    path.push_back(current);
  }

  std::reverse(path.begin(), path.end());
  return path;
}

} // namespace

AStarResult AStarPlanner::plan(const OccupancyGrid2D &grid,
                               const GridIndex start, const GridIndex goal,
                               const AStarConfig &config) const {
  AStarResult result{};
  if (!grid.contains(start) || !grid.contains(goal) || grid.isBlocked(start) ||
      grid.isBlocked(goal)) {
    return result;
  }

  constexpr std::array<GridIndex, 8> kNeighborOffsets{{
      {-1, -1},
      {0, -1},
      {1, -1},
      {-1, 0},
      {1, 0},
      {-1, 1},
      {0, 1},
      {1, 1},
  }};

  const std::size_t cell_count = grid.cellCount();
  std::vector<double> g_scores(cell_count,
                               std::numeric_limits<double>::infinity());
  std::vector<int> parents(cell_count, -1);
  std::vector<std::uint8_t> closed(cell_count, 0U);
  std::priority_queue<OpenNode, std::vector<OpenNode>, CompareOpenNode> open;

  const std::size_t start_index = grid.linearIndex(start);
  g_scores[start_index] = 0.0;
  open.push(OpenNode{start, heuristic(start, goal), 0.0});

  while (!open.empty() && result.expanded_cells < config.max_expansions) {
    const OpenNode current = open.top();
    open.pop();

    const std::size_t current_index = grid.linearIndex(current.cell);
    if (closed[current_index] != 0U) {
      continue;
    }
    closed[current_index] = 1U;
    ++result.expanded_cells;

    if (current.cell == goal) {
      result.path = reconstructPath(grid, parents, start, goal);
      result.success = !result.path.empty();
      return result;
    }

    for (const GridIndex offset : kNeighborOffsets) {
      const GridIndex next{current.cell.x + offset.x,
                           current.cell.y + offset.y};
      if (!grid.contains(next) || grid.isBlocked(next) ||
          diagonalMoveCutsBlockedCorner(grid, current.cell, next)) {
        continue;
      }

      const std::size_t next_index = grid.linearIndex(next);
      if (closed[next_index] != 0U) {
        continue;
      }

      const double step_cost =
          (offset.x != 0 && offset.y != 0) ? std::sqrt(2.0) : 1.0;
      const double tentative_g = g_scores[current_index] + step_cost;
      if (tentative_g >= g_scores[next_index]) {
        continue;
      }

      parents[next_index] = static_cast<int>(current_index);
      g_scores[next_index] = tentative_g;
      const double f_score = tentative_g + heuristic(next, goal);
      open.push(OpenNode{next, f_score, tentative_g});
    }
  }

  return result;
}

} // namespace drone_city_nav
