#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include "drone_city_nav/esdf_query.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>

namespace drone_city_nav {
namespace {

struct Key {
  int x{0};
  int y{0};
  int z{0};

  [[nodiscard]] bool operator==(const Key&) const noexcept = default;
};

struct KeyHash {
  [[nodiscard]] std::size_t operator()(const Key key) const noexcept {
    std::size_t seed = static_cast<std::size_t>(key.x) * 73856093U;
    seed ^= static_cast<std::size_t>(key.y) * 19349663U;
    seed ^= static_cast<std::size_t>(key.z) * 83492791U;
    return seed;
  }
};

struct Record {
  double g{std::numeric_limits<double>::infinity()};
  Key parent{};
  bool has_parent{false};
  double minimum_clearance_m{std::numeric_limits<double>::infinity()};
};

struct QueueEntry {
  double f{0.0};
  double g_at_insert{0.0};
  std::uint64_t sequence{0U};
  Key key{};
};

struct Greater {
  [[nodiscard]] bool operator()(const QueueEntry& lhs,
                                const QueueEntry& rhs) const noexcept {
    return lhs.f == rhs.f ? lhs.sequence > rhs.sequence : lhs.f > rhs.f;
  }
};

[[nodiscard]] Point3 pointFor(const Key key, const Point3& origin,
                              const RiskAwareLattice3DConfig& config) noexcept {
  return Point3{origin.x + static_cast<double>(key.x) * config.horizontal_step_m,
                origin.y + static_cast<double>(key.y) * config.horizontal_step_m,
                origin.z + static_cast<double>(key.z) * config.vertical_step_m};
}

[[nodiscard]] bool stageAllows(const Lattice3DRiskStage stage, const double clearance_m,
                               const RiskAwareLattice3DConfig& config) noexcept {
  switch (stage) {
    case Lattice3DRiskStage::kPreferredOnly:
      return clearance_m >= config.preferred_distance_m;
    case Lattice3DRiskStage::kPlanningAllowed:
      return clearance_m >= config.critical_distance_m;
    case Lattice3DRiskStage::kCriticalAllowed:
      return true;
  }
  return false;
}

struct EdgeEvaluation {
  bool valid{false};
  double minimum_clearance_m{std::numeric_limits<double>::infinity()};
  double exposure_cost{0.0};
};

[[nodiscard]] EdgeEvaluation evaluateEdge(const mppi::EsdfGrid& grid,
                                          const std::span<const float> esdf_m,
                                          const Point3& first, const Point3& second,
                                          const Lattice3DRiskStage stage,
                                          const RiskAwareLattice3DConfig& config) {
  const double length = distance3D(first, second);
  const std::size_t samples = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / config.sample_step_m)));
  EdgeEvaluation result{.valid = true};
  for (std::size_t sample = 1U; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
    const Point3 point{std::lerp(first.x, second.x, ratio),
                       std::lerp(first.y, second.y, ratio),
                       std::lerp(first.z, second.z, ratio)};
    const EsdfQueryResult query = queryConservativeEsdf3D(
        grid, esdf_m, static_cast<float>(point.x), static_cast<float>(point.y),
        static_cast<float>(point.z));
    if (query.status != EsdfQueryStatus::kValid || query.raw_occupied ||
        !stageAllows(stage, query.clearance_m, config)) {
      return {};
    }
    result.minimum_clearance_m =
        std::min(result.minimum_clearance_m, static_cast<double>(query.clearance_m));
    if (query.clearance_m < config.preferred_distance_m) {
      result.exposure_cost +=
          config.risk_exposure_tie_break_per_m * length / static_cast<double>(samples);
    }
  }
  return result;
}

[[nodiscard]] std::vector<Point3>
reconstruct(const Key terminal, const Point3& origin,
            const RiskAwareLattice3DConfig& config,
            const std::unordered_map<Key, Record, KeyHash>& records) {
  std::vector<Point3> points;
  Key current = terminal;
  while (true) {
    points.push_back(pointFor(current, origin, config));
    const auto found = records.find(current);
    if (found == records.end() || !found->second.has_parent) {
      break;
    }
    current = found->second.parent;
  }
  std::ranges::reverse(points);
  return points;
}

[[nodiscard]] RiskAwareLattice3DResult
searchStage(const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
            const Point3& start, const Vec3& preferred_direction,
            const Point3& planning_goal, const Point3& mission_goal,
            const Lattice3DRiskStage stage, const RiskAwareLattice3DConfig& config) {
  using Clock = std::chrono::steady_clock;
  const auto deadline = Clock::now() + std::chrono::duration<double, std::milli>(
                                           config.maximum_search_time_ms / 3.0);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, Greater> open;
  std::unordered_map<Key, Record, KeyHash> records;
  const Key root{};
  records[root].g = 0.0;
  std::uint64_t sequence = 0U;
  open.push(QueueEntry{.f = distance3D(start, planning_goal),
                       .g_at_insert = 0.0,
                       .sequence = sequence++,
                       .key = root});
  Key best = root;
  double best_remaining = distance3D(start, planning_goal);
  std::size_t expansions = 0U;
  std::size_t stale = 0U;
  std::size_t open_peak = 1U;
  bool reached = false;
  bool timed_out = false;
  constexpr std::array<int, 3> kOffsets{-1, 0, 1};
  while (!open.empty()) {
    if (expansions >= config.maximum_expansions || Clock::now() >= deadline) {
      timed_out = true;
      break;
    }
    const QueueEntry entry = open.top();
    open.pop();
    const auto found = records.find(entry.key);
    if (found == records.end() || entry.g_at_insert > found->second.g + 1.0e-9) {
      ++stale;
      continue;
    }
    ++expansions;
    const Point3 current = pointFor(entry.key, start, config);
    const double remaining = distance3D(current, planning_goal);
    if (remaining < best_remaining) {
      best_remaining = remaining;
      best = entry.key;
    }
    if (remaining <= config.goal_tolerance_m) {
      best = entry.key;
      reached = true;
      break;
    }
    for (const int dx : kOffsets) {
      for (const int dy : kOffsets) {
        for (const int dz : kOffsets) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const Key next{entry.key.x + dx, entry.key.y + dy, entry.key.z + dz};
          const Point3 successor = pointFor(next, start, config);
          const EdgeEvaluation edge =
              evaluateEdge(grid, esdf_m, current, successor, stage, config);
          if (!edge.valid) {
            continue;
          }
          const double edge_length = distance3D(current, successor);
          const double horizontal =
              std::hypot(static_cast<double>(dx), static_cast<double>(dy));
          double heading_cost = 0.0;
          const double preferred_norm =
              std::hypot(preferred_direction.x, preferred_direction.y);
          if (horizontal > 0.0 && preferred_norm > 1.0e-6) {
            const double cosine =
                std::clamp((static_cast<double>(dx) * preferred_direction.x +
                            static_cast<double>(dy) * preferred_direction.y) /
                               (horizontal * preferred_norm),
                           -1.0, 1.0);
            heading_cost = config.heading_bias_cost_per_rad * std::acos(cosine);
          }
          const double candidate_g =
              found->second.g + edge_length + edge.exposure_cost +
              config.vertical_cost_per_m * std::abs(static_cast<double>(dz)) +
              heading_cost;
          Record& candidate = records[next];
          if (!(candidate_g + 1.0e-9 < candidate.g)) {
            continue;
          }
          candidate.g = candidate_g;
          candidate.parent = entry.key;
          candidate.has_parent = true;
          candidate.minimum_clearance_m =
              std::min(found->second.minimum_clearance_m, edge.minimum_clearance_m);
          open.push(
              QueueEntry{.f = candidate_g + 1.5 * distance3D(successor, planning_goal),
                         .g_at_insert = candidate_g,
                         .sequence = sequence++,
                         .key = next});
          open_peak = std::max(open_peak, open.size());
        }
      }
    }
  }
  RiskAwareLattice3DResult result;
  result.risk_stage = stage;
  result.expansions = expansions;
  result.stale_queue_pops = stale;
  result.open_peak = open_peak;
  result.points = reconstruct(best, start, config, records);
  result.achieved_progress_m = distance3D(start, planning_goal) - best_remaining;
  result.minimum_clearance_m = records.at(best).minimum_clearance_m;
  if (reached) {
    if (evaluateEdge(grid, esdf_m, result.points.back(), planning_goal, stage, config)
            .valid) {
      result.points.push_back(planning_goal);
    }
    result.reached_mission_goal = distance3D(planning_goal, mission_goal) <= 1.0e-6;
    result.status = Lattice3DStatus::kReachedPlanningGoal;
  } else if (result.points.size() >= 3U && result.achieved_progress_m >= 4.0) {
    result.status = Lattice3DStatus::kViableFrontier;
  } else {
    result.status = timed_out ? Lattice3DStatus::kSearchIncomplete
                              : Lattice3DStatus::kMotionGraphExhausted;
  }
  result.route = sampleRoute3D(result.points, config.sample_step_m, 0.0);
  return result;
}

} // namespace

RiskAwareLattice3DResult
planRiskAwareLattice3D(const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
                       const Point3& start, const Vec3& preferred_direction,
                       const Point3& mission_goal,
                       const RiskAwareLattice3DConfig& config) {
  if (grid.depth <= 1 ||
      esdf_m.size() != static_cast<std::size_t>(grid.width) *
                           static_cast<std::size_t>(grid.height) *
                           static_cast<std::size_t>(grid.depth) ||
      !(config.horizontal_step_m > 0.0) || !(config.vertical_step_m > 0.0)) {
    return {};
  }
  const double full_distance = distance3D(start, mission_goal);
  const double ratio = full_distance > config.planning_goal_distance_m
                           ? config.planning_goal_distance_m / full_distance
                           : 1.0;
  const Point3 planning_goal{std::lerp(start.x, mission_goal.x, ratio),
                             std::lerp(start.y, mission_goal.y, ratio), mission_goal.z};
  std::optional<RiskAwareLattice3DResult> best_frontier;
  RiskAwareLattice3DResult last_result;
  for (const Lattice3DRiskStage stage :
       {Lattice3DRiskStage::kPreferredOnly, Lattice3DRiskStage::kPlanningAllowed,
        Lattice3DRiskStage::kCriticalAllowed}) {
    RiskAwareLattice3DResult result =
        searchStage(grid, esdf_m, start, preferred_direction, planning_goal,
                    mission_goal, stage, config);
    if (result.status == Lattice3DStatus::kReachedPlanningGoal) {
      return result;
    }
    if (result.status == Lattice3DStatus::kViableFrontier &&
        (!best_frontier.has_value() ||
         result.achieved_progress_m > best_frontier->achieved_progress_m + 1.0e-6)) {
      best_frontier = result;
    }
    last_result = std::move(result);
  }
  if (best_frontier.has_value()) {
    return std::move(*best_frontier);
  }
  return last_result;
}

const char* lattice3DStatusName(const Lattice3DStatus status) noexcept {
  switch (status) {
    case Lattice3DStatus::kInvalidInput:
      return "invalid_input";
    case Lattice3DStatus::kReachedPlanningGoal:
      return "reached_planning_goal";
    case Lattice3DStatus::kViableFrontier:
      return "viable_frontier";
    case Lattice3DStatus::kSearchIncomplete:
      return "search_incomplete";
    case Lattice3DStatus::kMotionGraphExhausted:
      return "motion_graph_exhausted";
  }
  return "unknown";
}

} // namespace drone_city_nav
