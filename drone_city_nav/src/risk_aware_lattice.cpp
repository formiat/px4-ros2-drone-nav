#include "drone_city_nav/risk_aware_lattice.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <queue>
#include <unordered_map>

namespace drone_city_nav {
namespace {

struct LatticeKey {
  int x{0};
  int y{0};
  int heading{0};

  bool operator==(const LatticeKey&) const = default;
};

struct LatticeKeyHash {
  std::size_t operator()(const LatticeKey& key) const noexcept {
    const std::uint64_t x = static_cast<std::uint32_t>(key.x);
    const std::uint64_t y = static_cast<std::uint32_t>(key.y);
    const std::uint64_t heading = static_cast<std::uint32_t>(key.heading);
    return static_cast<std::size_t>((x << 32U) ^ (y << 8U) ^ heading);
  }
};

struct Record {
  double cost{std::numeric_limits<double>::infinity()};
  std::optional<LatticeKey> parent;
};

struct QueueEntry {
  double priority{0.0};
  LatticeKey key{};

  bool operator>(const QueueEntry& other) const noexcept {
    return priority > other.priority;
  }
};

[[nodiscard]] Point2 cellCenter(const mppi::EsdfGrid& grid, const LatticeKey& key) {
  return Point2{
      grid.origin_x_m + (static_cast<double>(key.x) + 0.5) * grid.resolution_m,
      grid.origin_y_m + (static_cast<double>(key.y) + 0.5) * grid.resolution_m};
}

[[nodiscard]] std::optional<float> clearanceAt(const mppi::EsdfGrid& grid,
                                               const std::span<const float> esdf_m,
                                               const Point2 point) {
  const int x =
      static_cast<int>(std::floor((point.x - grid.origin_x_m) / grid.resolution_m));
  const int y =
      static_cast<int>(std::floor((point.y - grid.origin_y_m) / grid.resolution_m));
  if (x < 0 || y < 0 || x >= grid.width || y >= grid.height) {
    return std::nullopt;
  }
  return esdf_m[static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) +
                static_cast<std::size_t>(x)];
}

[[nodiscard]] double headingForBin(const int bin, const int bins) {
  return 2.0 * std::numbers::pi * static_cast<double>(bin) / static_cast<double>(bins);
}

[[nodiscard]] int wrapHeading(const int heading, const int bins) {
  const int wrapped = heading % bins;
  return wrapped < 0 ? wrapped + bins : wrapped;
}

[[nodiscard]] int nearestHeadingBin(const double heading_rad, const int bins) {
  return wrapHeading(
      static_cast<int>(std::lround(heading_rad * static_cast<double>(bins) /
                                   (2.0 * std::numbers::pi))),
      bins);
}

[[nodiscard]] int headingBinDistance(const int first, const int second,
                                     const int bins) noexcept {
  const int direct = std::abs(first - second);
  return std::min(direct, bins - direct);
}

[[nodiscard]] bool
pointInsidePortalFootprint(const Point2 point,
                           const SemanticPortalPrimitive& portal) noexcept {
  const Point2 offset{point.x - portal.center.x, point.y - portal.center.y};
  const Point2 lateral_axis{-portal.normal_xy.y, portal.normal_xy.x};
  const double longitudinal =
      offset.x * portal.normal_xy.x + offset.y * portal.normal_xy.y;
  const double lateral = offset.x * lateral_axis.x + offset.y * lateral_axis.y;
  return std::abs(longitudinal) <= 0.5 * portal.depth_m &&
         std::abs(lateral) <= 0.5 * portal.width_m;
}

[[nodiscard]] bool pointInsideAnyPortalFootprint(
    const Point2 point,
    const std::span<const SemanticPortalPrimitive> portals) noexcept {
  return std::ranges::any_of(portals, [&](const SemanticPortalPrimitive& portal) {
    return pointInsidePortalFootprint(point, portal);
  });
}

struct SegmentEvaluation {
  bool valid{false};
  double risk_cost{0.0};
};

[[nodiscard]] SegmentEvaluation
evaluateSegment(const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
                const Point2 start, const Point2 endpoint,
                const RiskAwareLatticeConfig& config,
                const std::span<const SemanticPortalPrimitive> portals,
                const bool allow_portal_footprint) {
  SegmentEvaluation result{.valid = true};
  const double length_m = distance(start, endpoint);
  const int sample_count =
      static_cast<int>(std::ceil(length_m / config.primitive_sample_step_m));
  for (int sample_index = 1; sample_index <= sample_count; ++sample_index) {
    const double sample_distance = std::min(
        length_m, static_cast<double>(sample_index) * config.primitive_sample_step_m);
    const double ratio = length_m > 0.0 ? sample_distance / length_m : 1.0;
    const Point2 sample{std::lerp(start.x, endpoint.x, ratio),
                        std::lerp(start.y, endpoint.y, ratio)};
    const std::optional<float> clearance = clearanceAt(grid, esdf_m, sample);
    if (!clearance.has_value() || *clearance <= config.collision_radius_m ||
        (!allow_portal_footprint && pointInsideAnyPortalFootprint(sample, portals))) {
      result.valid = false;
      return result;
    }
    if (*clearance < config.critical_distance_m) {
      result.risk_cost += config.critical_cost_per_m * config.primitive_sample_step_m;
    } else if (*clearance < config.preferred_distance_m) {
      result.risk_cost += config.planning_cost_per_m * config.primitive_sample_step_m;
    }
  }
  return result;
}

struct PortalSuccessor {
  Point2 endpoint{};
  int heading_bin{0};
  double length_m{0.0};
};

[[nodiscard]] std::optional<PortalSuccessor>
portalSuccessor(const Point2 current, const int current_heading,
                const SemanticPortalPrimitive& portal, const int direction,
                const RiskAwareLatticeConfig& config) {
  const Point2 lateral_axis{-portal.normal_xy.y, portal.normal_xy.x};
  const Point2 offset{current.x - portal.center.x, current.y - portal.center.y};
  const double longitudinal =
      offset.x * portal.normal_xy.x + offset.y * portal.normal_xy.y;
  const double lateral = offset.x * lateral_axis.x + offset.y * lateral_axis.y;
  const double directed_longitudinal = static_cast<double>(direction) * longitudinal;
  const double half_depth_m = 0.5 * portal.depth_m;
  const double usable_half_width_m =
      0.5 * portal.width_m - config.portal_lateral_margin_m;
  if (!(usable_half_width_m > 0.0) || std::abs(lateral) > usable_half_width_m ||
      directed_longitudinal < -half_depth_m - config.portal_entry_capture_distance_m ||
      directed_longitudinal >= half_depth_m) {
    return std::nullopt;
  }
  const double heading =
      std::atan2(static_cast<double>(direction) * portal.normal_xy.y,
                 static_cast<double>(direction) * portal.normal_xy.x);
  const int heading_bin = nearestHeadingBin(heading, config.heading_bins);
  if (headingBinDistance(current_heading, heading_bin, config.heading_bins) >
      config.portal_maximum_heading_delta_bins) {
    return std::nullopt;
  }
  const double exit_longitudinal =
      static_cast<double>(direction) * (half_depth_m + config.portal_exit_extension_m);
  const Point2 endpoint{
      portal.center.x + exit_longitudinal * portal.normal_xy.x +
          lateral * lateral_axis.x,
      portal.center.y + exit_longitudinal * portal.normal_xy.y +
          lateral * lateral_axis.y,
  };
  return PortalSuccessor{
      .endpoint = endpoint,
      .heading_bin = heading_bin,
      .length_m = distance(current, endpoint),
  };
}

[[nodiscard]] Point2 recedingGoal(const Point2 start, const Point2 mission_goal,
                                  const RiskAwareLatticeConfig& config,
                                  bool& reached_mission_goal) {
  const double distance =
      std::hypot(mission_goal.x - start.x, mission_goal.y - start.y);
  reached_mission_goal = distance <= config.receding_goal_distance_m;
  if (reached_mission_goal || distance <= 1.0e-6) {
    return mission_goal;
  }
  const double ratio = config.receding_goal_distance_m / distance;
  return Point2{start.x + (mission_goal.x - start.x) * ratio,
                start.y + (mission_goal.y - start.y) * ratio};
}

[[nodiscard]] bool
primitiveIsCollisionFree(const mppi::EsdfGrid& grid,
                         const std::span<const float> esdf_m, const Point2 start,
                         const int heading_bin, const RiskAwareLatticeConfig& config,
                         const std::span<const SemanticPortalPrimitive> portals) {
  const double heading = headingForBin(heading_bin, config.heading_bins);
  const Point2 endpoint{start.x + std::cos(heading) * config.primitive_length_m,
                        start.y + std::sin(heading) * config.primitive_length_m};
  return evaluateSegment(grid, esdf_m, start, endpoint, config, portals, false).valid;
}

[[nodiscard]] double guideLength(const std::span<const Point2> guide) noexcept {
  double length_m = 0.0;
  for (std::size_t index = 1U; index < guide.size(); ++index) {
    length_m += std::hypot(guide[index].x - guide[index - 1U].x,
                           guide[index].y - guide[index - 1U].y);
  }
  return length_m;
}

} // namespace

RiskAwareLatticeResult planRiskAwareMotionPrimitiveGuide(
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m, const Point2 start,
    const double start_heading_rad, const Point2 mission_goal,
    const RiskAwareLatticeConfig& config,
    const std::span<const SemanticPortalPrimitive> portals) {
  RiskAwareLatticeResult result;
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution_m <= 0.0F ||
      esdf_m.size() != static_cast<std::size_t>(grid.width) *
                           static_cast<std::size_t>(grid.height) ||
      config.heading_bins < 4 || !(config.portal_lateral_margin_m >= 0.0) ||
      !(config.portal_entry_capture_distance_m > 0.0) ||
      !(config.portal_exit_extension_m > 0.0) ||
      config.portal_maximum_heading_delta_bins < 0 ||
      config.portal_maximum_heading_delta_bins > config.heading_bins / 2) {
    return result;
  }
  result.planning_goal =
      recedingGoal(start, mission_goal, config, result.reached_mission_goal);
  const auto makeKey = [&grid, &config](const Point2 point, const int heading) {
    return LatticeKey{
        static_cast<int>(std::floor((point.x - grid.origin_x_m) / grid.resolution_m)),
        static_cast<int>(std::floor((point.y - grid.origin_y_m) / grid.resolution_m)),
        wrapHeading(heading, config.heading_bins)};
  };
  const LatticeKey start_key =
      makeKey(start, nearestHeadingBin(start_heading_rad, config.heading_bins));
  if (!clearanceAt(grid, esdf_m, start).has_value()) {
    return result;
  }
  result.status = LatticePlanStatus::kDeadEnd;
  result.termination = LatticeSearchTermination::kOpenSetExhausted;
  std::unordered_map<LatticeKey, Record, LatticeKeyHash> records;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> open;
  records[start_key].cost = 0.0;
  open.push(QueueEntry{0.0, start_key});
  std::optional<LatticeKey> best_goal;
  double best_goal_distance = std::numeric_limits<double>::infinity();
  bool exact_terminal_connector = false;
  double exact_terminal_connector_cost = 0.0;

  while (!open.empty() && result.expansions < config.maximum_expansions) {
    const QueueEntry current_entry = open.top();
    open.pop();
    const auto current_record = records.find(current_entry.key);
    if (current_record == records.end()) {
      continue;
    }
    const Point2 current = cellCenter(grid, current_entry.key);
    const double goal_distance = std::hypot(result.planning_goal.x - current.x,
                                            result.planning_goal.y - current.y);
    if (goal_distance < best_goal_distance) {
      best_goal_distance = goal_distance;
      best_goal = current_entry.key;
    }
    if (goal_distance <= config.goal_tolerance_m) {
      const SegmentEvaluation connector = evaluateSegment(
          grid, esdf_m, current, result.planning_goal, config, portals, false);
      if (connector.valid) {
        best_goal = current_entry.key;
        result.planning_goal_reached = true;
        exact_terminal_connector = true;
        exact_terminal_connector_cost =
            distance(current, result.planning_goal) + connector.risk_cost;
        break;
      }
    }
    ++result.expansions;
    for (const int heading_delta : {-1, 0, 1}) {
      const int next_heading =
          wrapHeading(current_entry.key.heading + heading_delta, config.heading_bins);
      const double heading = headingForBin(next_heading, config.heading_bins);
      const Point2 endpoint{current.x + std::cos(heading) * config.primitive_length_m,
                            current.y + std::sin(heading) * config.primitive_length_m};
      const SegmentEvaluation segment =
          evaluateSegment(grid, esdf_m, current, endpoint, config, portals, false);
      if (!segment.valid) {
        continue;
      }
      const LatticeKey next = makeKey(endpoint, next_heading);
      const double next_cost = current_record->second.cost + config.primitive_length_m +
                               segment.risk_cost +
                               config.turn_cost * std::abs(heading_delta);
      Record& record = records[next];
      if (next_cost >= record.cost) {
        continue;
      }
      record.cost = next_cost;
      record.parent = current_entry.key;
      const double heuristic = std::hypot(result.planning_goal.x - endpoint.x,
                                          result.planning_goal.y - endpoint.y);
      open.push(QueueEntry{next_cost + config.heuristic_weight * heuristic, next});
    }
    for (const SemanticPortalPrimitive& portal : portals) {
      for (const int direction : {-1, 1}) {
        const std::optional<PortalSuccessor> successor = portalSuccessor(
            current, current_entry.key.heading, portal, direction, config);
        if (!successor.has_value()) {
          continue;
        }
        const SegmentEvaluation segment = evaluateSegment(
            grid, esdf_m, current, successor->endpoint, config, portals, true);
        if (!segment.valid) {
          continue;
        }
        const LatticeKey next = makeKey(successor->endpoint, successor->heading_bin);
        const int heading_delta = headingBinDistance(
            current_entry.key.heading, successor->heading_bin, config.heading_bins);
        const double next_cost = current_record->second.cost + successor->length_m +
                                 segment.risk_cost +
                                 config.turn_cost * static_cast<double>(heading_delta);
        Record& record = records[next];
        if (next_cost >= record.cost) {
          continue;
        }
        record.cost = next_cost;
        record.parent = current_entry.key;
        const double heuristic = distance(result.planning_goal, successor->endpoint);
        open.push(QueueEntry{next_cost + config.heuristic_weight * heuristic, next});
      }
    }
  }
  if (result.planning_goal_reached) {
    result.termination = LatticeSearchTermination::kPlanningGoalReached;
  } else if (!open.empty() && result.expansions >= config.maximum_expansions) {
    result.termination = LatticeSearchTermination::kExpansionBudgetExhausted;
  } else {
    result.termination = LatticeSearchTermination::kOpenSetExhausted;
  }
  if (!best_goal.has_value()) {
    return result;
  }
  result.cost = records[*best_goal].cost + exact_terminal_connector_cost;
  for (std::optional<LatticeKey> key = best_goal; key.has_value();) {
    result.guide.push_back(cellCenter(grid, *key));
    key = records[*key].parent;
  }
  std::ranges::reverse(result.guide);
  if (exact_terminal_connector &&
      (result.guide.empty() ||
       distance(result.guide.back(), result.planning_goal) > 1.0e-6)) {
    result.guide.push_back(result.planning_goal);
  }
  result.exact_terminal_connector = exact_terminal_connector;
  result.guide_length_m = guideLength(result.guide);
  result.remaining_goal_distance_m =
      exact_terminal_connector ? 0.0 : best_goal_distance;
  result.achieved_progress_m =
      std::hypot(result.planning_goal.x - start.x, result.planning_goal.y - start.y) -
      result.remaining_goal_distance_m;
  const Point2 terminal = cellCenter(grid, *best_goal);
  for (const int heading_delta : {-1, 0, 1}) {
    const int next_heading =
        wrapHeading(best_goal->heading + heading_delta, config.heading_bins);
    if (primitiveIsCollisionFree(grid, esdf_m, terminal, next_heading, config,
                                 portals)) {
      ++result.terminal_successor_count;
    }
  }
  if (result.planning_goal_reached) {
    result.status = LatticePlanStatus::kReachedPlanningGoal;
  } else if (result.guide.size() >= config.minimum_frontier_guide_points &&
             result.guide_length_m >= config.minimum_frontier_guide_length_m &&
             result.achieved_progress_m >= config.minimum_frontier_progress_m &&
             result.terminal_successor_count > 0U) {
    result.status = LatticePlanStatus::kViableFrontier;
  } else {
    result.status = LatticePlanStatus::kDeadEnd;
  }
  result.valid = result.status == LatticePlanStatus::kReachedPlanningGoal ||
                 result.status == LatticePlanStatus::kViableFrontier;
  return result;
}

const char* latticePlanStatusName(const LatticePlanStatus status) noexcept {
  switch (status) {
    case LatticePlanStatus::kInvalidInput:
      return "invalid_input";
    case LatticePlanStatus::kReachedPlanningGoal:
      return "reached_planning_goal";
    case LatticePlanStatus::kViableFrontier:
      return "viable_frontier";
    case LatticePlanStatus::kDeadEnd:
      return "dead_end";
  }
  return "unknown";
}

const char*
latticeSearchTerminationName(const LatticeSearchTermination termination) noexcept {
  switch (termination) {
    case LatticeSearchTermination::kInvalidInput:
      return "invalid_input";
    case LatticeSearchTermination::kPlanningGoalReached:
      return "planning_goal_reached";
    case LatticeSearchTermination::kOpenSetExhausted:
      return "open_set_exhausted";
    case LatticeSearchTermination::kExpansionBudgetExhausted:
      return "expansion_budget_exhausted";
  }
  return "unknown";
}

} // namespace drone_city_nav
