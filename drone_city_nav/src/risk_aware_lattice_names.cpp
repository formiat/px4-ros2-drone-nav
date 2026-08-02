#include "drone_city_nav/risk_aware_lattice.hpp"

namespace drone_city_nav {

const char* latticePlanStatusName(const LatticePlanStatus status) noexcept {
  switch (status) {
    case LatticePlanStatus::kInvalidInput:
      return "invalid_input";
    case LatticePlanStatus::kReachedPlanningGoal:
      return "reached_planning_goal";
    case LatticePlanStatus::kViableFrontier:
      return "viable_frontier";
    case LatticePlanStatus::kRawSafeDetourPrefix:
      return "raw_safe_detour_prefix";
    case LatticePlanStatus::kSearchIncomplete:
      return "search_incomplete";
    case LatticePlanStatus::kMotionGraphExhausted:
      return "motion_graph_exhausted";
  }
  return "unknown";
}

const char* latticeRiskStageName(const LatticeRiskStage stage) noexcept {
  switch (stage) {
    case LatticeRiskStage::kPreferredOnly:
      return "preferred";
    case LatticeRiskStage::kPlanningAllowed:
      return "planning";
    case LatticeRiskStage::kCriticalAllowed:
      return "critical";
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
    case LatticeSearchTermination::kDeadlineReached:
      return "deadline_reached";
    case LatticeSearchTermination::kRoiBoundaryReached:
      return "roi_boundary_reached";
  }
  return "unknown";
}

} // namespace drone_city_nav
