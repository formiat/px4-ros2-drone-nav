#pragma once

#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/planning_grid_builder.hpp"
#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <limits>

namespace drone_city_nav {

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
evaluatePlannerGridReadiness(const PlanningGridBuildResult& result) noexcept;

[[nodiscard]] StablePathRuntimeAction
stablePathRuntimeAction(StablePathDecisionReason reason) noexcept;

[[nodiscard]] PlannerModePrimaryAction
plannerModePrimaryAction(bool use_static_map, bool rollout_enabled) noexcept;

[[nodiscard]] bool
publicationGenerationIsCurrent(std::uint64_t candidate_generation,
                               std::uint64_t latest_generation) noexcept;

} // namespace drone_city_nav
