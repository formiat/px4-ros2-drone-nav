#include "drone_city_nav/directed_inflation_escape.hpp"

#include "drone_city_nav/distance_field.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <queue>
#include <ranges>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kCostEpsilon = 1.0e-9;

struct SearchNode {
  double cost{kInfinity};
  double path_length_m{kInfinity};
  GridIndex cell{};
};

struct SearchNodeGreater {
  [[nodiscard]] bool operator()(const SearchNode& lhs,
                                const SearchNode& rhs) const noexcept {
    if (std::abs(lhs.cost - rhs.cost) > kCostEpsilon) {
      return lhs.cost > rhs.cost;
    }
    if (std::abs(lhs.path_length_m - rhs.path_length_m) > kCostEpsilon) {
      return lhs.path_length_m > rhs.path_length_m;
    }
    if (lhs.cell.y != rhs.cell.y) {
      return lhs.cell.y > rhs.cell.y;
    }
    return lhs.cell.x > rhs.cell.x;
  }
};

struct SearchResult {
  InflationEscapeNeed need{InflationEscapeNeed::kNoReachableExit};
  Point2 target{};
  double path_length_m{0.0};
  std::size_t cells_considered{0U};
  std::vector<Point2> centerline;
};

constexpr std::array<GridIndex, 8U> kNeighborOffsets{
    GridIndex{-1, -1}, GridIndex{0, -1}, GridIndex{1, -1}, GridIndex{-1, 0},
    GridIndex{1, 0},   GridIndex{-1, 1}, GridIndex{0, 1},  GridIndex{1, 1},
};

[[nodiscard]] bool finitePositive(const double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] bool validConfig(const DirectedInflationEscapeConfig& config) noexcept {
  return finitePositive(config.tunnel_width_m) && finitePositive(config.max_length_m) &&
         config.exit_depth_m >= 0.0 && std::isfinite(config.exit_depth_m) &&
         std::isfinite(config.inflation_exposure_cost_weight) &&
         config.inflation_exposure_cost_weight >= 0.0 &&
         std::isfinite(config.occupied_clearance_cost_weight) &&
         config.occupied_clearance_cost_weight >= 0.0 &&
         finitePositive(config.mission_egress_distance_m) &&
         config.stable_exit_cycles > 0U;
}

[[nodiscard]] double stepLength(const OccupancyGrid2D& grid,
                                const GridIndex offset) noexcept {
  return grid.resolution() *
         (offset.x != 0 && offset.y != 0 ? std::numbers::sqrt2 : 1.0);
}

[[nodiscard]] bool stableExitCell(const OccupancyGrid2D& grid,
                                  const DistanceField2D& prohibited_distance,
                                  const GridIndex cell, const double exit_depth_m) {
  return !grid.isProhibited(cell) &&
         prohibited_distance.distanceAt(cell) + kCostEpsilon >= exit_depth_m;
}

[[nodiscard]] bool missionEgressAvailable(const OccupancyGrid2D& grid,
                                          const Point2 start, const Point2 mission_goal,
                                          const double requested_distance_m) {
  const double goal_distance_m = distance(start, mission_goal);
  if (goal_distance_m <= kCostEpsilon) {
    return true;
  }
  const double checked_distance_m = std::min(goal_distance_m, requested_distance_m);
  const double step_m = std::max(0.25 * grid.resolution(), 0.05);
  const std::size_t steps = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(checked_distance_m / step_m)));
  for (std::size_t index = 0U; index <= steps; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(steps);
    const Point2 point{
        start.x +
            ratio * checked_distance_m * (mission_goal.x - start.x) / goal_distance_m,
        start.y +
            ratio * checked_distance_m * (mission_goal.y - start.y) / goal_distance_m,
    };
    const std::optional<GridIndex> cell = grid.worldToCell(point);
    if (!cell.has_value() || grid.isProhibited(*cell)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] std::vector<Point2>
reconstructCenterline(const OccupancyGrid2D& grid, const GridIndex start,
                      const GridIndex target, const std::vector<GridIndex>& parent) {
  std::vector<Point2> reversed;
  GridIndex cell = target;
  while (true) {
    reversed.push_back(grid.cellCenter(cell));
    if (cell == start) {
      break;
    }
    const GridIndex next = parent[grid.linearIndex(cell)];
    if (next == cell || !grid.contains(next)) {
      return {};
    }
    cell = next;
  }
  std::ranges::reverse(reversed);
  return reversed;
}

[[nodiscard]] SearchResult findEscape(const OccupancyGrid2D& grid,
                                      const Point2 current_position,
                                      const Point2 mission_goal,
                                      const DirectedInflationEscapeConfig& config) {
  SearchResult result{};
  const std::optional<GridIndex> start = grid.worldToCell(current_position);
  if (!start.has_value()) {
    result.need = InflationEscapeNeed::kOutsideGrid;
    return result;
  }
  if (grid.isOccupied(*start)) {
    result.need = InflationEscapeNeed::kStartOccupied;
    return result;
  }
  if (!grid.isInflated(*start)) {
    result.need = InflationEscapeNeed::kNotNeeded;
    return result;
  }

  result.need = InflationEscapeNeed::kNeeded;
  const double field_distance_m =
      config.max_length_m + config.exit_depth_m + config.tunnel_width_m;
  const DistanceField2D occupied_distance =
      DistanceField2D::build(grid, field_distance_m, DistanceFieldSource::kOccupied);
  const DistanceField2D prohibited_distance =
      DistanceField2D::build(grid, field_distance_m, DistanceFieldSource::kProhibited);

  std::vector<double> best_cost(grid.cellCount(), kInfinity);
  std::vector<GridIndex> parent(grid.cellCount(), GridIndex{-1, -1});
  std::priority_queue<SearchNode, std::vector<SearchNode>, SearchNodeGreater> open;
  best_cost[grid.linearIndex(*start)] = 0.0;
  parent[grid.linearIndex(*start)] = *start;
  open.push(SearchNode{.cost = 0.0, .path_length_m = 0.0, .cell = *start});

  while (!open.empty()) {
    const SearchNode current = open.top();
    open.pop();
    const std::size_t current_index = grid.linearIndex(current.cell);
    if (current.cost > best_cost[current_index] + kCostEpsilon) {
      continue;
    }
    ++result.cells_considered;
    const Point2 current_point = grid.cellCenter(current.cell);
    if (current.cell != *start &&
        stableExitCell(grid, prohibited_distance, current.cell, config.exit_depth_m) &&
        missionEgressAvailable(grid, current_point, mission_goal,
                               config.mission_egress_distance_m)) {
      result.target = current_point;
      result.path_length_m = current.path_length_m;
      result.centerline = reconstructCenterline(grid, *start, current.cell, parent);
      if (result.centerline.empty()) {
        result.need = InflationEscapeNeed::kNoReachableExit;
      }
      return result;
    }

    for (const GridIndex offset : kNeighborOffsets) {
      const GridIndex neighbor{current.cell.x + offset.x, current.cell.y + offset.y};
      if (!grid.contains(neighbor) || grid.isOccupied(neighbor)) {
        continue;
      }
      const double step_m = stepLength(grid, offset);
      const double next_length_m = current.path_length_m + step_m;
      if (next_length_m > config.max_length_m + kCostEpsilon) {
        continue;
      }
      const double occupied_clearance_m = occupied_distance.distanceAt(neighbor);
      const double clearance_penalty =
          std::isfinite(occupied_clearance_m)
              ? config.occupied_clearance_cost_weight /
                    std::max(grid.resolution(), occupied_clearance_m)
              : 0.0;
      const double inflation_penalty =
          grid.isInflated(neighbor) ? config.inflation_exposure_cost_weight : 0.0;
      const double goal_delta_m = distance(grid.cellCenter(neighbor), mission_goal) -
                                  distance(grid.cellCenter(current.cell), mission_goal);
      const double goal_tie_break = 1.0e-6 * std::max(-1.0, goal_delta_m);
      const double next_cost = current.cost +
                               step_m * (1.0 + inflation_penalty + clearance_penalty) +
                               goal_tie_break;
      const std::size_t neighbor_index = grid.linearIndex(neighbor);
      if (next_cost + kCostEpsilon < best_cost[neighbor_index]) {
        best_cost[neighbor_index] = next_cost;
        parent[neighbor_index] = current.cell;
        open.push(SearchNode{
            .cost = next_cost, .path_length_m = next_length_m, .cell = neighbor});
      }
    }
  }

  result.need = InflationEscapeNeed::kNoReachableExit;
  return result;
}

[[nodiscard]] bool centerlineBlocked(const OccupancyGrid2D& grid,
                                     const std::vector<Point2>& centerline) {
  return std::ranges::any_of(centerline, [&grid](const Point2 point) {
    const std::optional<GridIndex> cell = grid.worldToCell(point);
    return !cell.has_value() || grid.isOccupied(*cell);
  });
}

[[nodiscard]] double pointToSegmentDistance(const Point2 point, const Point2 start,
                                            const Point2 end) noexcept {
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length_squared = (dx * dx) + (dy * dy);
  if (length_squared <= kCostEpsilon) {
    return distance(point, start);
  }
  const double projection = std::clamp(
      ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared, 0.0, 1.0);
  return distance(point, Point2{start.x + projection * dx, start.y + projection * dy});
}

[[nodiscard]] double
distanceToCenterline(const Point2 point,
                     const std::vector<Point2>& centerline) noexcept {
  if (centerline.empty()) {
    return kInfinity;
  }
  if (centerline.size() == 1U) {
    return distance(point, centerline.front());
  }
  double minimum_distance = kInfinity;
  for (std::size_t index = 1U; index < centerline.size(); ++index) {
    minimum_distance =
        std::min(minimum_distance, pointToSegmentDistance(point, centerline[index - 1U],
                                                          centerline[index]));
  }
  return minimum_distance;
}

} // namespace

DirectedInflationEscapeResult DirectedInflationEscapePlanner::update(
    const OccupancyGrid2D& original_grid, const Point2 current_position,
    const Point2 mission_goal, const DirectedInflationEscapeConfig& config) {
  DirectedInflationEscapeResult result{};
  result.start = current_position;
  if (!config.enabled || !validConfig(config)) {
    reset();
    return result;
  }

  const std::optional<GridIndex> current_cell =
      original_grid.worldToCell(current_position);
  if (!current_cell.has_value()) {
    reset();
    result.need = InflationEscapeNeed::kOutsideGrid;
    result.state = DirectedInflationEscapeState::kFailed;
    return result;
  }
  if (original_grid.isOccupied(*current_cell)) {
    reset();
    result.need = InflationEscapeNeed::kStartOccupied;
    result.state = DirectedInflationEscapeState::kFailed;
    return result;
  }

  if (!episode_.centerline.empty()) {
    const bool current_is_allowed = !original_grid.isInflated(*current_cell);
    const bool mission_egress_available =
        current_is_allowed &&
        missionEgressAvailable(original_grid, current_position, mission_goal,
                               config.mission_egress_distance_m);
    result.mission_egress_available = mission_egress_available;
    if (mission_egress_available) {
      ++episode_.stable_exit_cycles;
    } else {
      episode_.stable_exit_cycles = 0U;
    }
    if (episode_.stable_exit_cycles >= config.stable_exit_cycles) {
      episode_.awaiting_mission_continuation = true;
    }
    if (episode_.awaiting_mission_continuation &&
        episode_.mission_continuation_confirmed) {
      result.need = InflationEscapeNeed::kNotNeeded;
      result.state = DirectedInflationEscapeState::kCompleted;
      result.episode_generation = episode_.generation;
      result.target = episode_.target;
      result.stable_exit_cycles = episode_.stable_exit_cycles;
      result.mission_egress_available = true;
      episode_ = Episode{};
      return result;
    }

    const bool blocked = centerlineBlocked(original_grid, episode_.centerline);
    const bool off_centerline =
        !current_is_allowed &&
        distanceToCenterline(current_position, episode_.centerline) >
            0.5 * config.tunnel_width_m + kCostEpsilon;
    const bool target_too_far =
        !current_is_allowed && distance(current_position, episode_.target) >
                                   config.max_length_m + kCostEpsilon;
    if (blocked || off_centerline || target_too_far) {
      episode_ = Episode{};
      result.centerline_blocked = blocked;
      result.episode_off_centerline = off_centerline;
      result.episode_target_too_far = target_too_far;
    } else {
      result.need = InflationEscapeNeed::kNeeded;
      result.state = DirectedInflationEscapeState::kActive;
      result.applied = true;
      result.episode_generation = episode_.generation;
      result.target = episode_.target;
      result.centerline_length_m = episode_.centerline_length_m;
      result.stable_exit_cycles = episode_.stable_exit_cycles;
      result.awaiting_mission_continuation = episode_.awaiting_mission_continuation;
      result.centerline = episode_.centerline;
      return result;
    }
  }

  if (!original_grid.isInflated(*current_cell)) {
    result.need = InflationEscapeNeed::kNotNeeded;
    if (result.centerline_blocked || result.episode_off_centerline ||
        result.episode_target_too_far) {
      result.state = DirectedInflationEscapeState::kFailed;
    }
    return result;
  }

  const SearchResult search =
      findEscape(original_grid, current_position, mission_goal, config);
  result.need = search.need;
  result.cells_considered = search.cells_considered;
  if (search.need != InflationEscapeNeed::kNeeded || search.centerline.empty()) {
    result.state = DirectedInflationEscapeState::kFailed;
    return result;
  }

  episode_ = Episode{
      .generation = next_generation_++,
      .start = current_position,
      .target = search.target,
      .centerline_length_m = search.path_length_m,
      .stable_exit_cycles = 0U,
      .centerline = search.centerline,
  };
  result.state = DirectedInflationEscapeState::kStarted;
  result.applied = true;
  result.episode_generation = episode_.generation;
  result.target = episode_.target;
  result.centerline_length_m = episode_.centerline_length_m;
  result.centerline = episode_.centerline;
  return result;
}

bool DirectedInflationEscapePlanner::confirmMissionContinuation(
    const std::uint64_t episode_generation) {
  if (episode_generation == 0U || episode_.generation != episode_generation ||
      !episode_.awaiting_mission_continuation) {
    return false;
  }
  episode_.mission_continuation_confirmed = true;
  return true;
}

void DirectedInflationEscapePlanner::reset() noexcept {
  episode_ = Episode{};
}

LocalInflationRelaxationStats
applyDirectedInflationEscape(OccupancyGrid2D& planning_grid,
                             const DirectedInflationEscapeResult& escape,
                             const double tunnel_width_m) {
  LocalInflationRelaxationStats combined{};
  if (!escape.applied || escape.centerline.empty() || !finitePositive(tunnel_width_m)) {
    return combined;
  }

  const double tunnel_radius_m = 0.5 * tunnel_width_m;
  for (const Point2 point : escape.centerline) {
    const LocalInflationRelaxationStats step =
        planning_grid.clearInflationWithinRadius(point, tunnel_radius_m);
    combined.cells_considered += step.cells_considered;
    combined.inflated_cells_cleared += step.inflated_cells_cleared;
    combined.occupied_cells_preserved += step.occupied_cells_preserved;
    combined.cells_outside_bounds += step.cells_outside_bounds;
    combined.center_inside_bounds =
        combined.center_inside_bounds || step.center_inside_bounds;
  }
  return combined;
}

std::string_view inflationEscapeNeedName(const InflationEscapeNeed need) noexcept {
  switch (need) {
    case InflationEscapeNeed::kNotNeeded:
      return "not_needed";
    case InflationEscapeNeed::kNeeded:
      return "needed";
    case InflationEscapeNeed::kStartOccupied:
      return "start_occupied";
    case InflationEscapeNeed::kOutsideGrid:
      return "outside_grid";
    case InflationEscapeNeed::kNoReachableExit:
      return "no_reachable_exit";
  }
  return "unknown";
}

std::string_view
directedInflationEscapeStateName(const DirectedInflationEscapeState state) noexcept {
  switch (state) {
    case DirectedInflationEscapeState::kInactive:
      return "inactive";
    case DirectedInflationEscapeState::kStarted:
      return "started";
    case DirectedInflationEscapeState::kActive:
      return "active";
    case DirectedInflationEscapeState::kCompleted:
      return "completed";
    case DirectedInflationEscapeState::kFailed:
      return "failed";
  }
  return "unknown";
}

} // namespace drone_city_nav
