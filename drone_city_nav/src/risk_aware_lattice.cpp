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

} // namespace

RiskAwareLatticeResult planRiskAwareMotionPrimitiveGuide(
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m, const Point2 start,
    const double start_heading_rad, const Point2 mission_goal,
    const RiskAwareLatticeConfig& config) {
  RiskAwareLatticeResult result;
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution_m <= 0.0F ||
      esdf_m.size() != static_cast<std::size_t>(grid.width) *
                           static_cast<std::size_t>(grid.height) ||
      config.heading_bins < 4) {
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
  std::unordered_map<LatticeKey, Record, LatticeKeyHash> records;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> open;
  records[start_key].cost = 0.0;
  open.push(QueueEntry{0.0, start_key});
  std::optional<LatticeKey> best_goal;
  double best_goal_distance = std::numeric_limits<double>::infinity();

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
      best_goal = current_entry.key;
      break;
    }
    ++result.expansions;
    for (const int heading_delta : {-1, 0, 1}) {
      const int next_heading =
          wrapHeading(current_entry.key.heading + heading_delta, config.heading_bins);
      const double heading = headingForBin(next_heading, config.heading_bins);
      bool collision = false;
      double risk_cost = 0.0;
      Point2 endpoint = current;
      const int sample_count = static_cast<int>(
          std::ceil(config.primitive_length_m / config.primitive_sample_step_m));
      for (int sample_index = 1; sample_index <= sample_count; ++sample_index) {
        const double distance =
            std::min(config.primitive_length_m, static_cast<double>(sample_index) *
                                                    config.primitive_sample_step_m);
        endpoint = Point2{current.x + std::cos(heading) * distance,
                          current.y + std::sin(heading) * distance};
        const std::optional<float> clearance = clearanceAt(grid, esdf_m, endpoint);
        if (!clearance.has_value() || *clearance <= config.collision_radius_m) {
          collision = true;
          break;
        }
        if (*clearance < config.critical_distance_m) {
          risk_cost += config.critical_cost_per_m * config.primitive_sample_step_m;
        } else if (*clearance < config.preferred_distance_m) {
          risk_cost += config.planning_cost_per_m * config.primitive_sample_step_m;
        }
      }
      if (collision) {
        continue;
      }
      const LatticeKey next = makeKey(endpoint, next_heading);
      const double next_cost = current_record->second.cost + config.primitive_length_m +
                               risk_cost + config.turn_cost * std::abs(heading_delta);
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
  }
  if (!best_goal.has_value()) {
    return result;
  }
  result.cost = records[*best_goal].cost;
  for (std::optional<LatticeKey> key = best_goal; key.has_value();) {
    result.guide.push_back(cellCenter(grid, *key));
    key = records[*key].parent;
  }
  std::ranges::reverse(result.guide);
  result.valid = result.guide.size() >= 2U;
  return result;
}

} // namespace drone_city_nav
