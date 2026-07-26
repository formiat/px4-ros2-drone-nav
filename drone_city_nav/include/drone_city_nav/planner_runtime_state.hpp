#pragma once

#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/planning_grid_builder.hpp"
#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <limits>
#include <optional>

namespace drone_city_nav {

enum class PlanningWakeReason {
  kPeriodicTimer,
  kRetry,
  kStaleRetry,
  kRecoveryGuideReady,
  kInvalidation,
};

enum class PlanningInvalidationReason {
  kNone,
  kTruncationChanged,
  kActivePrefixBlocked,
};

struct PlanningJobIdentity {
  std::uint64_t cycle_sequence{0U};
  std::uint64_t invalidation_generation{0U};
  PlanningWakeReason wake_reason{PlanningWakeReason::kPeriodicTimer};
  std::uint64_t coalesced_requests{0U};
};

class PlanningRequestState {
public:
  void schedule(PlanningWakeReason reason) noexcept;
  void invalidate(PlanningInvalidationReason reason) noexcept;

  [[nodiscard]] bool pending() const noexcept;
  [[nodiscard]] bool running() const noexcept;
  [[nodiscard]] PlanningJobIdentity beginCycle() noexcept;
  void finishCycle() noexcept;

  [[nodiscard]] std::uint64_t latestInvalidationGeneration() const noexcept;
  [[nodiscard]] PlanningInvalidationReason latestInvalidationReason() const noexcept;

private:
  std::uint64_t next_cycle_sequence_{0U};
  std::uint64_t latest_invalidation_generation_{1U};
  std::uint64_t coalesced_requests_{0U};
  PlanningWakeReason pending_wake_reason_{PlanningWakeReason::kPeriodicTimer};
  PlanningInvalidationReason latest_invalidation_reason_{
      PlanningInvalidationReason::kNone};
  bool pending_{false};
  bool running_{false};
};

[[nodiscard]] const char* planningWakeReasonName(PlanningWakeReason reason) noexcept;

[[nodiscard]] const char*
planningInvalidationReasonName(PlanningInvalidationReason reason) noexcept;

enum class PlannerRuntimeReadinessReason {
  kReady,
  kNoPose,
  kStalePose,
};

struct PlannerRuntimeReadinessInput {
  bool pose_valid{false};
  bool pose_finite{false};
  bool pose_fresh{false};
};

struct PlannerRuntimeReadinessDecision {
  PlannerRuntimeReadinessReason reason{PlannerRuntimeReadinessReason::kNoPose};
  bool ready{false};
};

enum class PlannerGridReadinessReason {
  kReady,
  kStaticMapMissing,
  kNoReadySourceData,
  kMissingGrid,
};

struct PlannerGridReadinessDecision {
  PlannerGridReadinessReason reason{PlannerGridReadinessReason::kNoReadySourceData};
  bool ready{false};
  bool memory_geometry_mismatch{false};
};

enum class StablePathRuntimeAction {
  kReuse,
  kRunAStar,
};

enum class PlannerModePrimaryAction {
  kAStar,
  kRollout,
};

[[nodiscard]] double ageSecondsFromStamp(std::int64_t stamp_ns,
                                         std::int64_t now_ns) noexcept;

[[nodiscard]] PlannerRuntimeReadinessDecision
evaluatePlannerRuntimeReadiness(const PlannerRuntimeReadinessInput& input) noexcept;

[[nodiscard]] PlannerGridReadinessDecision
evaluatePlannerGridReadiness(const ObstacleFieldBuildResult& result) noexcept;

[[nodiscard]] StablePathRuntimeAction
stablePathRuntimeAction(StablePathDecisionReason reason) noexcept;

[[nodiscard]] PlannerModePrimaryAction
plannerModePrimaryAction(bool use_static_map, bool rollout_enabled) noexcept;

[[nodiscard]] bool astarPlanningAllowed(bool use_static_map,
                                        bool no_static_astar_recovery_enabled) noexcept;

[[nodiscard]] std::optional<Point2>
boundedNoStaticRecoveryGoal(const OccupancyGrid2D& grid,
                            const ObstacleRiskField& risk_field, Point2 start,
                            Point2 mission_goal);

[[nodiscard]] bool
publicationGenerationIsCurrent(std::uint64_t candidate_generation,
                               std::uint64_t latest_generation) noexcept;

} // namespace drone_city_nav
