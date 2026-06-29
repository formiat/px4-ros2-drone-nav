#include "drone_city_nav/planner_runtime_state.hpp"

namespace drone_city_nav {

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
evaluatePlannerGridReadiness(const PlanningGridBuildResult& result) noexcept {
  PlannerGridReadinessDecision decision{};
  decision.memory_geometry_mismatch =
      result.memory.seen && !result.memory.geometry_matches;
  switch (result.status) {
    case PlanningGridStatus::kReady:
      decision.reason = result.grid.has_value() && result.planning_grid.has_value()
                            ? PlannerGridReadinessReason::kReady
                            : PlannerGridReadinessReason::kMissingGrid;
      decision.ready = result.grid.has_value() && result.planning_grid.has_value();
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

} // namespace drone_city_nav
