#include "drone_city_nav/planner_runtime_state.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr std::array<GridIndex, 8U> kRecoveryNeighborOffsets{{
    {-1, -1},
    {0, -1},
    {1, -1},
    {-1, 0},
    {1, 0},
    {-1, 1},
    {0, 1},
    {1, 1},
}};

[[nodiscard]] bool recoveryCellBlocked(const OccupancyGrid2D& grid,
                                       const ObstacleRiskField& risk_field,
                                       const GridIndex cell) {
  return !grid.contains(cell) || grid.isOccupied(cell) ||
         !risk_field.containsEvaluationPoint(grid.cellCenter(cell));
}

[[nodiscard]] bool recoveryMoveCutsRawCorner(const OccupancyGrid2D& grid,
                                             const ObstacleRiskField& risk_field,
                                             const GridIndex from, const GridIndex to) {
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  if (std::abs(dx) != 1 || std::abs(dy) != 1) {
    return false;
  }
  return recoveryCellBlocked(grid, risk_field, GridIndex{from.x + dx, from.y}) ||
         recoveryCellBlocked(grid, risk_field, GridIndex{from.x, from.y + dy});
}

[[nodiscard]] std::vector<std::uint8_t>
recoveryReachableCells(const OccupancyGrid2D& grid, const ObstacleRiskField& risk_field,
                       const GridIndex start) {
  std::vector<std::uint8_t> reachable(grid.cellCount(), 0U);
  if (recoveryCellBlocked(grid, risk_field, start)) {
    return reachable;
  }

  std::queue<GridIndex> pending;
  reachable.at(grid.linearIndex(start)) = 1U;
  pending.push(start);
  while (!pending.empty()) {
    const GridIndex current = pending.front();
    pending.pop();
    for (const GridIndex offset : kRecoveryNeighborOffsets) {
      const GridIndex next{current.x + offset.x, current.y + offset.y};
      if (recoveryCellBlocked(grid, risk_field, next) ||
          recoveryMoveCutsRawCorner(grid, risk_field, current, next)) {
        continue;
      }
      const std::size_t next_index = grid.linearIndex(next);
      if (reachable.at(next_index) != 0U) {
        continue;
      }
      reachable.at(next_index) = 1U;
      pending.push(next);
    }
  }
  return reachable;
}

} // namespace

void PlanningRequestState::schedule(const PlanningWakeReason reason) noexcept {
  if (pending_) {
    ++coalesced_requests_;
    return;
  }
  pending_ = true;
  pending_wake_reason_ = reason;
}

void PlanningRequestState::invalidate(
    const PlanningInvalidationReason reason) noexcept {
  ++latest_invalidation_generation_;
  latest_invalidation_reason_ = reason;
  if (pending_) {
    ++coalesced_requests_;
  }
  pending_ = true;
  pending_wake_reason_ = PlanningWakeReason::kInvalidation;
}

bool PlanningRequestState::pending() const noexcept {
  return pending_;
}

bool PlanningRequestState::running() const noexcept {
  return running_;
}

PlanningJobIdentity PlanningRequestState::beginCycle() noexcept {
  PlanningJobIdentity identity{
      .cycle_sequence = ++next_cycle_sequence_,
      .invalidation_generation = latest_invalidation_generation_,
      .wake_reason = pending_wake_reason_,
      .coalesced_requests = coalesced_requests_,
  };
  pending_ = false;
  running_ = true;
  coalesced_requests_ = 0U;
  return identity;
}

void PlanningRequestState::finishCycle() noexcept {
  running_ = false;
}

std::uint64_t PlanningRequestState::latestInvalidationGeneration() const noexcept {
  return latest_invalidation_generation_;
}

PlanningInvalidationReason
PlanningRequestState::latestInvalidationReason() const noexcept {
  return latest_invalidation_reason_;
}

const char* planningWakeReasonName(const PlanningWakeReason reason) noexcept {
  switch (reason) {
    case PlanningWakeReason::kPeriodicTimer:
      return "periodic_timer";
    case PlanningWakeReason::kRetry:
      return "retry";
    case PlanningWakeReason::kStaleRetry:
      return "stale_retry";
    case PlanningWakeReason::kRecoveryGuideReady:
      return "recovery_guide_ready";
    case PlanningWakeReason::kInvalidation:
      return "invalidation";
  }
  return "unknown";
}

const char*
planningInvalidationReasonName(const PlanningInvalidationReason reason) noexcept {
  switch (reason) {
    case PlanningInvalidationReason::kNone:
      return "none";
    case PlanningInvalidationReason::kTruncationChanged:
      return "truncation_changed";
    case PlanningInvalidationReason::kActivePrefixBlocked:
      return "active_prefix_blocked";
  }
  return "unknown";
}

[[nodiscard]] double ageSecondsFromStamp(const std::int64_t stamp_ns,
                                         const std::int64_t now_ns) noexcept {
  if (stamp_ns <= 0 || now_ns <= stamp_ns) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(now_ns - stamp_ns) / 1.0e9;
}

[[nodiscard]] PlannerRuntimeReadinessDecision
evaluatePlannerRuntimeReadiness(const PlannerRuntimeReadinessInput& input) noexcept {
  if (!input.pose_valid || !input.pose_finite) {
    return PlannerRuntimeReadinessDecision{PlannerRuntimeReadinessReason::kNoPose,
                                           false};
  }
  if (!input.pose_fresh) {
    return PlannerRuntimeReadinessDecision{PlannerRuntimeReadinessReason::kStalePose,
                                           false};
  }
  return PlannerRuntimeReadinessDecision{PlannerRuntimeReadinessReason::kReady, true};
}

[[nodiscard]] PlannerGridReadinessDecision
evaluatePlannerGridReadiness(const ObstacleFieldBuildResult& result) noexcept {
  PlannerGridReadinessDecision decision{};
  decision.memory_geometry_mismatch =
      result.memory.seen && !result.memory.geometry_matches;
  switch (result.status) {
    case PlanningGridStatus::kReady:
      decision.reason = result.raw_occupancy.has_value()
                            ? PlannerGridReadinessReason::kReady
                            : PlannerGridReadinessReason::kMissingGrid;
      decision.ready = result.raw_occupancy.has_value();
      return decision;
    case PlanningGridStatus::kStaticMapEnabledButMissing:
      decision.reason = PlannerGridReadinessReason::kStaticMapMissing;
      return decision;
    case PlanningGridStatus::kNoReadySourceData:
      decision.reason = PlannerGridReadinessReason::kNoReadySourceData;
      return decision;
  }
  return decision;
}

[[nodiscard]] StablePathRuntimeAction
stablePathRuntimeAction(const StablePathDecisionReason reason) noexcept {
  switch (reason) {
    case StablePathDecisionReason::kClear:
      return StablePathRuntimeAction::kReuse;
    case StablePathDecisionReason::kDisabled:
    case StablePathDecisionReason::kNoPreviousPath:
    case StablePathDecisionReason::kGoalMismatch:
    case StablePathDecisionReason::kProjectionUnavailable:
    case StablePathDecisionReason::kProhibitedConfirmed:
      return StablePathRuntimeAction::kRunAStar;
  }
  return StablePathRuntimeAction::kRunAStar;
}

PlannerModePrimaryAction plannerModePrimaryAction(const bool use_static_map,
                                                  const bool rollout_enabled) noexcept {
  return !use_static_map && rollout_enabled ? PlannerModePrimaryAction::kRollout
                                            : PlannerModePrimaryAction::kAStar;
}

bool astarPlanningAllowed(const bool use_static_map,
                          const bool no_static_astar_recovery_enabled) noexcept {
  return use_static_map || no_static_astar_recovery_enabled;
}

std::optional<Point2> boundedNoStaticRecoveryGoal(const OccupancyGrid2D& grid,
                                                  const ObstacleRiskField& risk_field,
                                                  const Point2 start,
                                                  const Point2 mission_goal) {
  const std::optional<GridIndex> start_cell = grid.worldToCell(start);
  if (!start_cell.has_value() || grid.isOccupied(*start_cell) ||
      !risk_field.containsEvaluationPoint(start)) {
    return std::nullopt;
  }
  const std::vector<std::uint8_t> reachable =
      recoveryReachableCells(grid, risk_field, *start_cell);
  if (const std::optional<GridIndex> mission_cell = grid.worldToCell(mission_goal);
      mission_cell.has_value() && !grid.isOccupied(*mission_cell) &&
      risk_field.containsEvaluationPoint(mission_goal) &&
      reachable.at(grid.linearIndex(*mission_cell)) != 0U) {
    return mission_goal;
  }

  const Point2 goal_delta{mission_goal.x - start.x, mission_goal.y - start.y};
  const double goal_distance_m = std::hypot(goal_delta.x, goal_delta.y);
  if (!(goal_distance_m > grid.resolution()) || !std::isfinite(goal_distance_m)) {
    return std::nullopt;
  }
  const Point2 direction{goal_delta.x / goal_distance_m,
                         goal_delta.y / goal_distance_m};
  const GridBounds& evaluation = risk_field.evaluationBounds();
  const double half_cell_m = 0.5 * evaluation.resolution_m;
  const double min_x = evaluation.origin_x + half_cell_m;
  const double min_y = evaluation.origin_y + half_cell_m;
  const double max_x =
      evaluation.origin_x +
      evaluation.resolution_m * static_cast<double>(evaluation.width_cells) -
      half_cell_m;
  const double max_y =
      evaluation.origin_y +
      evaluation.resolution_m * static_cast<double>(evaluation.height_cells) -
      half_cell_m;

  double ray_limit_m = goal_distance_m;
  const auto limit_axis = [&ray_limit_m](const double origin, const double component,
                                         const double minimum, const double maximum) {
    if (component > 1.0e-9) {
      ray_limit_m = std::min(ray_limit_m, (maximum - origin) / component);
    } else if (component < -1.0e-9) {
      ray_limit_m = std::min(ray_limit_m, (minimum - origin) / component);
    }
  };
  limit_axis(start.x, direction.x, min_x, max_x);
  limit_axis(start.y, direction.y, min_y, max_y);
  if (!(ray_limit_m > grid.resolution()) || !std::isfinite(ray_limit_m)) {
    return std::nullopt;
  }
  const Point2 ideal{start.x + direction.x * ray_limit_m,
                     start.y + direction.y * ray_limit_m};

  std::optional<Point2> best;
  double best_distance_sq = std::numeric_limits<double>::infinity();
  double best_forward_m = -std::numeric_limits<double>::infinity();
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      const GridIndex cell{x, y};
      if (reachable.at(grid.linearIndex(cell)) == 0U) {
        continue;
      }
      const Point2 candidate = grid.cellCenter(cell);
      if (!risk_field.containsEvaluationPoint(candidate)) {
        continue;
      }
      const Point2 offset{candidate.x - start.x, candidate.y - start.y};
      const double forward_m = offset.x * direction.x + offset.y * direction.y;
      if (forward_m < grid.resolution()) {
        continue;
      }
      const double candidate_distance_sq = squaredDistance(candidate, ideal);
      if (candidate_distance_sq + 1.0e-9 < best_distance_sq ||
          (std::abs(candidate_distance_sq - best_distance_sq) <= 1.0e-9 &&
           forward_m > best_forward_m)) {
        best = candidate;
        best_distance_sq = candidate_distance_sq;
        best_forward_m = forward_m;
      }
    }
  }
  return best;
}

bool publicationGenerationIsCurrent(const std::uint64_t candidate_generation,
                                    const std::uint64_t latest_generation) noexcept {
  return candidate_generation != 0U && candidate_generation == latest_generation;
}

} // namespace drone_city_nav
