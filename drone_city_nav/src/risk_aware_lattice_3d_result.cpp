#include "risk_aware_lattice_3d_result.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ranges>
#include <string>
#include <tuple>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::string
topologyName(const std::span<const SelectedChannelTraversal> traversals) {
  if (traversals.empty()) {
    return "lattice";
  }
  std::string result{"channel:"};
  for (std::size_t index = 0U; index < traversals.size(); ++index) {
    if (index > 0U) {
      result += '+';
    }
    result += traversals[index].channel_id;
  }
  return result;
}

[[nodiscard]] bool betterReached(const RiskAwareLattice3DResult& candidate,
                                 const RiskAwareLattice3DResult& current) noexcept {
  if (candidate.objective_cost + 1.0e-9 < current.objective_cost) {
    return true;
  }
  if (std::abs(candidate.objective_cost - current.objective_cost) <= 1.0e-9) {
    return static_cast<unsigned>(candidate.risk_stage) <
           static_cast<unsigned>(current.risk_stage);
  }
  return false;
}

[[nodiscard]] bool betterFrontier(const RiskAwareLattice3DResult& candidate,
                                  const RiskAwareLattice3DResult& current) noexcept {
  if (candidate.achieved_progress_m > current.achieved_progress_m + 1.0e-6) {
    return true;
  }
  return std::abs(candidate.achieved_progress_m - current.achieved_progress_m) <=
             1.0e-6 &&
         betterReached(candidate, current);
}

} // namespace

namespace detail {

Lattice3DStageSelection selectLattice3DStageResult(
    const std::span<const RiskAwareLattice3DResult> stage_results,
    const std::size_t channel_count) {
  std::optional<std::size_t> selected;
  for (std::size_t index = 0U; index < stage_results.size(); ++index) {
    if (stage_results[index].status != Lattice3DStatus::kReachedPlanningGoal) {
      continue;
    }
    if (!selected.has_value() ||
        betterReached(stage_results[index], stage_results[*selected])) {
      selected = index;
    }
  }
  if (!selected.has_value()) {
    for (std::size_t index = 0U; index < stage_results.size(); ++index) {
      if (stage_results[index].status != Lattice3DStatus::kViableFrontier) {
        continue;
      }
      if (!selected.has_value() ||
          betterFrontier(stage_results[index], stage_results[*selected])) {
        selected = index;
      }
    }
  }
  if (!selected.has_value()) {
    const auto incomplete =
        std::ranges::find_if(stage_results, [](const RiskAwareLattice3DResult& result) {
          return result.status == Lattice3DStatus::kSearchIncomplete;
        });
    selected =
        incomplete != stage_results.end()
            ? static_cast<std::size_t>(std::distance(stage_results.begin(), incomplete))
            : stage_results.size() - 1U;
  }

  std::vector<Lattice3DTopologyCandidate> diagnostics;
  diagnostics.reserve(stage_results.size() * (channel_count + 1U));
  for (std::size_t index = 0U; index < stage_results.size(); ++index) {
    const RiskAwareLattice3DResult& result = stage_results[index];
    const bool is_selected = index == *selected;
    diagnostics.insert(diagnostics.end(), result.topology_candidates.begin(),
                       result.topology_candidates.end());
    diagnostics.push_back(Lattice3DTopologyCandidate{
        .topology = topologyName(result.selected_channels),
        .risk_stage = result.risk_stage,
        .status = result.status,
        .termination = result.termination,
        .objective_cost = result.objective_cost,
        .route_length_m = result.route_length_m,
        .estimated_travel_time_s = result.estimated_travel_time_s,
        .vertical_alignment_time_s = result.vertical_alignment_time_s,
        .planning_exposure_m = result.planning_exposure_m,
        .critical_exposure_m = result.critical_exposure_m,
        .turn_cost = result.turn_cost,
        .achieved_progress_m = result.achieved_progress_m,
        .minimum_clearance_m = result.minimum_clearance_m,
        .expansions = result.expansions,
        .stale_queue_pops = result.stale_queue_pops,
        .open_peak = result.open_peak,
        .records_peak = result.records_peak,
        .terminal_successor_count = result.terminal_successor_count,
        .continuation_reachable_states = result.continuation_reachable_states,
        .continuation_reachable_depth_m = result.continuation_reachable_depth_m,
        .decision_reason =
            is_selected ? "minimum_executable_objective" : "not_selected",
        .selected = is_selected,
    });
  }
  std::ranges::stable_sort(diagnostics, [](const Lattice3DTopologyCandidate& lhs,
                                           const Lattice3DTopologyCandidate& rhs) {
    const auto lhs_key =
        std::tuple{lhs.status != Lattice3DStatus::kReachedPlanningGoal,
                   lhs.status != Lattice3DStatus::kViableFrontier, lhs.objective_cost,
                   lhs.topology, static_cast<unsigned>(lhs.risk_stage)};
    const auto rhs_key =
        std::tuple{rhs.status != Lattice3DStatus::kReachedPlanningGoal,
                   rhs.status != Lattice3DStatus::kViableFrontier, rhs.objective_cost,
                   rhs.topology, static_cast<unsigned>(rhs.risk_stage)};
    return lhs_key < rhs_key;
  });
  for (std::size_t rank = 0U; rank < diagnostics.size(); ++rank) {
    diagnostics[rank].candidate_rank = rank;
  }
  return Lattice3DStageSelection{.selected_index = *selected,
                                 .diagnostics = std::move(diagnostics)};
}

} // namespace detail

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

const char* lattice3DRiskStageName(const Lattice3DRiskStage stage) noexcept {
  switch (stage) {
    case Lattice3DRiskStage::kPreferredOnly:
      return "preferred";
    case Lattice3DRiskStage::kPlanningAllowed:
      return "planning";
    case Lattice3DRiskStage::kCriticalAllowed:
      return "critical";
  }
  return "unknown";
}

const char*
lattice3DSearchTerminationName(const Lattice3DSearchTermination termination) noexcept {
  switch (termination) {
    case Lattice3DSearchTermination::kInvalidInput:
      return "invalid_input";
    case Lattice3DSearchTermination::kPlanningGoalReached:
      return "planning_goal_reached";
    case Lattice3DSearchTermination::kOpenSetExhausted:
      return "open_set_exhausted";
    case Lattice3DSearchTermination::kExpansionBudgetExhausted:
      return "expansion_budget_exhausted";
    case Lattice3DSearchTermination::kDeadlineReached:
      return "deadline_reached";
  }
  return "unknown";
}

} // namespace drone_city_nav
