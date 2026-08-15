#include "drone_city_nav/bounded_worker_pool.hpp"
#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <span>
#include <utility>
#include <vector>

#include "risk_aware_lattice_3d_geometry.hpp"
#include "risk_aware_lattice_3d_result.hpp"
#include "risk_aware_lattice_3d_search.hpp"

namespace drone_city_nav {
namespace {

struct TopologySearchBatch {
  std::vector<PassageTraversalEdge> passages;
  detail::Lattice3DTopologyRequirement requirement{
      detail::Lattice3DTopologyRequirement::kUnconstrained};
  std::vector<RiskAwareLattice3DResult> stage_results;
  double worker_ms{0.0};
};

[[nodiscard]] bool validSearchInput(const mppi::EsdfGrid& grid,
                                    const std::span<const float> esdf_m,
                                    const Point3& start, const Point3& mission_goal,
                                    const RiskAwareLattice3DConfig& config) noexcept {
  return grid.depth > 1 &&
         esdf_m.size() == static_cast<std::size_t>(grid.width) *
                              static_cast<std::size_t>(grid.height) *
                              static_cast<std::size_t>(grid.depth) &&
         config.horizontal_step_m > 0.0 && config.vertical_step_m > 0.0 &&
         config.sample_step_m > 0.0 && config.physical_footprint_sweep_step_m > 0.0 &&
         config.nominal_horizontal_speed_mps > 0.0 &&
         config.nominal_vertical_speed_mps > 0.0 &&
         config.passage_connection_distance_m > 0.0 &&
         evaluateFlightEnvelopeAltitude(start.z, config.flight_envelope) ==
             FlightEnvelopeStatus::kValid &&
         evaluateFlightEnvelopeAltitude(mission_goal.z, config.flight_envelope) ==
             FlightEnvelopeStatus::kValid;
}

[[nodiscard]] Point3 planningGoal(const Point3& start, const Point3& mission_goal,
                                  const double maximum_distance_m) noexcept {
  const double full_distance = distance3D(start, mission_goal);
  const double ratio =
      full_distance > maximum_distance_m ? maximum_distance_m / full_distance : 1.0;
  return Point3{std::lerp(start.x, mission_goal.x, ratio),
                std::lerp(start.y, mission_goal.y, ratio), mission_goal.z};
}

[[nodiscard]] std::vector<TopologySearchBatch>
makeTopologySearches(const std::span<const PassageTraversalEdge> passage_traversals,
                     const std::size_t maximum_group_count) {
  std::vector<TopologySearchBatch> searches;
  searches.push_back(TopologySearchBatch{
      .passages = std::vector<PassageTraversalEdge>{passage_traversals.begin(),
                                                    passage_traversals.end()},
      .requirement = detail::Lattice3DTopologyRequirement::kUnconstrained,
      .stage_results = {},
      .worker_ms = 0.0,
  });
  const std::size_t group_count =
      std::min(passage_traversals.size(), maximum_group_count);
  searches.resize(1U + group_count);
  for (std::size_t group_index = 0U; group_index < group_count; ++group_index) {
    searches[1U + group_index].requirement =
        detail::Lattice3DTopologyRequirement::kRequirePassageTraversal;
  }
  for (std::size_t passage_index = 0U; passage_index < passage_traversals.size();
       ++passage_index) {
    if (group_count == 0U) {
      break;
    }
    searches[1U + passage_index % group_count].passages.push_back(
        passage_traversals[passage_index]);
  }
  return searches;
}

void accumulateSearchProfiling(
    const std::span<const RiskAwareLattice3DResult> stage_results,
    Lattice3DSuccessorProfiling& successor_profiling,
    double& continuation_validation_ms) noexcept {
  for (const RiskAwareLattice3DResult& stage_result : stage_results) {
    detail::accumulateLattice3DSuccessorProfile(
        successor_profiling.search, stage_result.successor_profiling.search);
    detail::accumulateLattice3DSuccessorProfile(
        successor_profiling.continuation,
        stage_result.successor_profiling.continuation);
    continuation_validation_ms += stage_result.continuation_validation_ms;
  }
}

} // namespace

RiskAwareLattice3DResult planRiskAwareLattice3D(
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
    const Point3& start, const Vec3& preferred_direction, const Point3& mission_goal,
    const std::span<const PassageTraversalEdge> passage_traversals,
    const RiskAwareLattice3DConfig& config, BoundedWorkerPool* const worker_pool) {
  const auto search_started = std::chrono::steady_clock::now();
  if (!validSearchInput(grid, esdf_m, start, mission_goal, config)) {
    return {};
  }
  const Point3 planning_goal =
      planningGoal(start, mission_goal, config.planning_goal_distance_m);
  std::vector<TopologySearchBatch> topology_searches =
      makeTopologySearches(passage_traversals, config.maximum_topology_search_groups);

  const auto run_topology_search = [&](const std::size_t search_index) {
    const auto topology_started = std::chrono::steady_clock::now();
    TopologySearchBatch& topology = topology_searches[search_index];
    topology.stage_results.reserve(3U);
    for (const Lattice3DRiskStage stage :
         {Lattice3DRiskStage::kPreferredOnly, Lattice3DRiskStage::kPlanningAllowed,
          Lattice3DRiskStage::kCriticalAllowed}) {
      topology.stage_results.push_back(detail::searchRiskAwareLattice3DStage(
          grid, esdf_m, start, preferred_direction, planning_goal, mission_goal,
          topology.passages, stage, topology.requirement, config, worker_pool));
    }
    topology.worker_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - topology_started)
                             .count();
  };
  const bool topology_parallel = worker_pool != nullptr &&
                                 worker_pool->canParallelizeFromCurrentThread() &&
                                 topology_searches.size() > 1U;
  if (topology_parallel) {
    worker_pool->parallelFor(topology_searches.size(), run_topology_search);
  } else {
    for (std::size_t search_index = 0U; search_index < topology_searches.size();
         ++search_index) {
      run_topology_search(search_index);
    }
  }

  std::vector<RiskAwareLattice3DResult> stage_results;
  stage_results.reserve(topology_searches.size() * 3U);
  double topology_search_worker_ms = 0.0;
  for (TopologySearchBatch& topology : topology_searches) {
    topology_search_worker_ms += topology.worker_ms;
    for (RiskAwareLattice3DResult& stage_result : topology.stage_results) {
      stage_results.push_back(std::move(stage_result));
    }
  }

  detail::Lattice3DStageSelection selection =
      detail::selectLattice3DStageResult(stage_results, passage_traversals.size());
  Lattice3DSuccessorProfiling aggregate_successor_profiling;
  double aggregate_continuation_validation_ms = 0.0;
  accumulateSearchProfiling(stage_results, aggregate_successor_profiling,
                            aggregate_continuation_validation_ms);
  RiskAwareLattice3DResult result = std::move(stage_results[selection.selected_index]);
  result.successor_profiling = aggregate_successor_profiling;
  result.topology_searches = topology_searches.size();
  result.parallel_topology_searches = topology_parallel ? topology_searches.size() : 0U;
  result.topology_search_worker_ms = topology_search_worker_ms;
  result.continuation_validation_ms = aggregate_continuation_validation_ms;
  result.planning_goal = planning_goal;
  result.topology_candidates = std::move(selection.diagnostics);
  result.route_fingerprint =
      routeFingerprint(result.route, result.selected_passage_traversals);
  result.search_ms = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - search_started)
                         .count();
  return result;
}

} // namespace drone_city_nav
