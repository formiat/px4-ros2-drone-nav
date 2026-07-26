#include "drone_city_nav/planner_runtime_state.hpp"

namespace drone_city_nav {

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

bool publicationGenerationIsCurrent(const std::uint64_t candidate_generation,
                                    const std::uint64_t latest_generation) noexcept {
  return candidate_generation != 0U && candidate_generation == latest_generation;
}

} // namespace drone_city_nav
