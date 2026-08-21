#include "drone_city_nav/bounded_worker_pool.hpp"
#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <ranges>
#include <span>
#include <tuple>
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
         config.frontier_minimum_endpoint_displacement_m > 0.0 &&
         config.observation_frontier_maximum_evaluations > 0U &&
         config.observation_frontier_evaluation_stride > 0U &&
         config.observation_frontier_maximum_searches > 0U &&
         config.observation_frontier_search_time_ms > 0.0 &&
         config.observation_frontier_information_gain_weight >= 0.0 &&
         config.observation_frontier_goal_progress_weight >= 0.0 &&
         config.observation_frontier_path_cost_weight >= 0.0 &&
         config.observation_frontier_clearance_weight >= 0.0 &&
         config.observation_frontier_replacement_minimum_score_improvement >= 0.0 &&
         config.observation_frontier_replacement_minimum_endpoint_improvement_m > 0.0 &&
         evaluateFlightEnvelopeAltitude(start.z, config.flight_envelope) ==
             FlightEnvelopeStatus::kValid &&
         evaluateFlightEnvelopeAltitude(mission_goal.z, config.flight_envelope) ==
             FlightEnvelopeStatus::kValid;
}

[[nodiscard]] bool
validStrategicDirective(const Lattice3DStrategicDirective& directive,
                        const RiskAwareLattice3DConfig& config) noexcept {
  return std::isfinite(directive.planning_goal.x) &&
         std::isfinite(directive.planning_goal.y) &&
         std::isfinite(directive.planning_goal.z) &&
         std::isfinite(directive.preferred_direction.x) &&
         std::isfinite(directive.preferred_direction.y) &&
         std::isfinite(directive.preferred_direction.z) &&
         std::isfinite(directive.selection_score) &&
         evaluateFlightEnvelopeAltitude(directive.planning_goal.z,
                                        config.flight_envelope) ==
             FlightEnvelopeStatus::kValid;
}

struct RankedObservationFrontier {
  ObservationFrontier frontier{};
  double approximate_score{-std::numeric_limits<double>::infinity()};
};

[[nodiscard]] double
observationFrontierScore(const ObservationFrontier& frontier, const Point3& start,
                         const Point3& planning_goal, const double path_cost,
                         const double minimum_clearance_m,
                         const RiskAwareLattice3DConfig& config) noexcept {
  const double goal_progress_m = distance3D(start, planning_goal) -
                                 distance3D(frontier.observation_pose, planning_goal);
  const double bounded_clearance_m =
      std::isfinite(minimum_clearance_m)
          ? std::min(minimum_clearance_m,
                     std::max(config.preferred_distance_m, 1.0) * 2.0)
          : 0.0;
  return config.observation_frontier_information_gain_weight *
             std::log1p(static_cast<double>(frontier.information_gain_voxels)) +
         config.observation_frontier_goal_progress_weight * goal_progress_m +
         config.observation_frontier_clearance_weight * bounded_clearance_m -
         config.observation_frontier_path_cost_weight * path_cost;
}

[[nodiscard]] double
approximateTravelCost(const Point3& start, const Point3& target,
                      const RiskAwareLattice3DConfig& config) noexcept {
  return std::hypot(target.x - start.x, target.y - start.y) /
             config.nominal_horizontal_speed_mps +
         std::abs(target.z - start.z) / config.nominal_vertical_speed_mps;
}

[[nodiscard]] bool
betterObservationRoute(const RiskAwareLattice3DResult& candidate,
                       const RiskAwareLattice3DResult& current) noexcept {
  if (candidate.frontier_selection_score > current.frontier_selection_score + 1.0e-9) {
    return true;
  }
  if (std::abs(candidate.frontier_selection_score - current.frontier_selection_score) >
      1.0e-9) {
    return false;
  }
  return candidate.observation_frontier.has_value() &&
         current.observation_frontier.has_value() &&
         candidate.observation_frontier->id < current.observation_frontier->id;
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
    const RiskAwareLattice3DConfig& config, BoundedWorkerPool* const worker_pool,
    const Lattice3DExplorationContext* const exploration_context) {
  const auto search_started = std::chrono::steady_clock::now();
  if (!validSearchInput(grid, esdf_m, start, mission_goal, config)) {
    return {};
  }
  const Lattice3DStrategicDirective* strategic_directive{nullptr};
  if (exploration_context != nullptr &&
      exploration_context->strategic_directive.has_value()) {
    strategic_directive =
        std::addressof(exploration_context->strategic_directive.value());
  }
  if (strategic_directive != nullptr &&
      !validStrategicDirective(*strategic_directive, config)) {
    return {};
  }
  const Point3 planning_goal =
      strategic_directive != nullptr
          ? strategic_directive->planning_goal
          : planningGoal(start, mission_goal, config.planning_goal_distance_m);
  const Vec3 search_direction = strategic_directive != nullptr
                                    ? strategic_directive->preferred_direction
                                    : preferred_direction;
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
          grid, esdf_m, start, search_direction, planning_goal, mission_goal,
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

  if (result.status != Lattice3DStatus::kReachedPlanningGoal &&
      strategic_directive == nullptr && exploration_context != nullptr &&
      exploration_context->observed_occupancy != nullptr) {
    ObservationFrontierDiscovery discovery = discoverObservationFrontiers(
        *exploration_context->observed_occupancy, exploration_context->map_revision,
        config.sensor_observability, config.observation_frontier_evaluation_stride,
        config.observation_frontier_maximum_evaluations);
    std::vector<RankedObservationFrontier> frontiers;
    frontiers.reserve(discovery.frontiers.size());
    for (const ObservationFrontier& frontier : discovery.frontiers) {
      if (distance3D(start, frontier.observation_pose) + 1.0e-9 <
          config.frontier_minimum_endpoint_displacement_m) {
        continue;
      }
      const double approximate_cost =
          approximateTravelCost(start, frontier.observation_pose, config);
      const double approximate_score = observationFrontierScore(
          frontier, start, planning_goal, approximate_cost, 0.0, config);
      frontiers.push_back(RankedObservationFrontier{
          .frontier = frontier,
          .approximate_score = approximate_score,
      });
    }
    std::ranges::stable_sort(frontiers, [](const RankedObservationFrontier& lhs,
                                           const RankedObservationFrontier& rhs) {
      return std::tuple{-lhs.approximate_score, lhs.frontier.id.value} <
             std::tuple{-rhs.approximate_score, rhs.frontier.id.value};
    });

    const std::size_t search_count =
        std::min(frontiers.size(), config.observation_frontier_maximum_searches);
    const auto exploration_started = std::chrono::steady_clock::now();
    RiskAwareLattice3DResult best_observation_route{};
    bool has_observation_route{false};
    std::size_t performed_searches = 0U;
    for (std::size_t index = 0U; index < search_count; ++index) {
      const double elapsed_ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                    exploration_started)
              .count();
      const double remaining_ms =
          config.observation_frontier_search_time_ms - elapsed_ms;
      if (!(remaining_ms > 1.0)) {
        break;
      }
      const double candidate_budget_ms =
          index + 1U == search_count ? remaining_ms : remaining_ms * 0.70;
      const auto candidate_deadline =
          std::chrono::steady_clock::now() +
          std::chrono::duration<double, std::milli>(candidate_budget_ms);
      const ObservationFrontier& frontier = frontiers[index].frontier;
      const Vec3 frontier_direction{
          frontier.observation_pose.x - start.x,
          frontier.observation_pose.y - start.y,
          frontier.observation_pose.z - start.z,
      };
      std::vector<RiskAwareLattice3DResult> frontier_stage_results;
      frontier_stage_results.reserve(3U);
      for (const Lattice3DRiskStage stage :
           {Lattice3DRiskStage::kPreferredOnly, Lattice3DRiskStage::kPlanningAllowed,
            Lattice3DRiskStage::kCriticalAllowed}) {
        const double stage_remaining_ms =
            std::chrono::duration<double, std::milli>(candidate_deadline -
                                                      std::chrono::steady_clock::now())
                .count();
        if (!(stage_remaining_ms > 1.0)) {
          break;
        }
        RiskAwareLattice3DConfig frontier_config = config;
        // A search stage owns one third of this value because the ordinary
        // mission search runs three progressively relaxed risk stages.
        frontier_config.maximum_search_time_ms = stage_remaining_ms * 3.0;
        frontier_stage_results.push_back(detail::searchRiskAwareLattice3DStage(
            grid, esdf_m, start, frontier_direction, frontier.observation_pose,
            mission_goal, passage_traversals, stage,
            detail::Lattice3DTopologyRequirement::kUnconstrained, frontier_config,
            worker_pool));
        if (frontier_stage_results.back().status ==
            Lattice3DStatus::kReachedPlanningGoal) {
          break;
        }
      }
      ++performed_searches;
      if (frontier_stage_results.empty()) {
        continue;
      }
      accumulateSearchProfiling(frontier_stage_results, aggregate_successor_profiling,
                                aggregate_continuation_validation_ms);
      detail::Lattice3DStageSelection frontier_selection =
          detail::selectLattice3DStageResult(frontier_stage_results,
                                             passage_traversals.size());
      RiskAwareLattice3DResult frontier_result =
          std::move(frontier_stage_results[frontier_selection.selected_index]);
      if (frontier_result.status != Lattice3DStatus::kReachedPlanningGoal) {
        continue;
      }
      frontier_result.status = Lattice3DStatus::kViableFrontier;
      frontier_result.route_purpose = Lattice3DRoutePurpose::kObservationFrontier;
      frontier_result.observation_frontier = frontier;
      frontier_result.reached_mission_goal = false;
      frontier_result.planning_goal = frontier.observation_pose;
      frontier_result.achieved_progress_m =
          distance3D(start, planning_goal) -
          distance3D(frontier.observation_pose, planning_goal);
      frontier_result.frontier_endpoint_displacement_m =
          distance3D(start, frontier.observation_pose);
      frontier_result.frontier_selection_score = observationFrontierScore(
          frontier, start, planning_goal, frontier_result.objective_cost,
          frontier_result.minimum_clearance_m, config);
      frontier_result.topology_candidates = std::move(frontier_selection.diagnostics);
      if (!has_observation_route) {
        best_observation_route = std::move(frontier_result);
        has_observation_route = true;
        continue;
      }
      if (betterObservationRoute(frontier_result, best_observation_route)) {
        best_observation_route = std::move(frontier_result);
      }
    }
    if (has_observation_route) {
      result = std::move(best_observation_route);
    }
    result.frontier_candidates_considered = frontiers.size();
    result.frontier_sampled_free_voxels = discovery.sampled_free_voxels;
    result.frontier_boundary_candidates = discovery.boundary_candidates;
    result.frontier_evaluated_candidates = discovery.evaluated_candidates;
    result.frontier_searches = performed_searches;
    result.frontier_evaluation_budget_exhausted = discovery.evaluation_budget_exhausted;
  }
  result.successor_profiling = aggregate_successor_profiling;
  result.topology_searches = topology_searches.size();
  result.parallel_topology_searches = topology_parallel ? topology_searches.size() : 0U;
  result.topology_search_worker_ms = topology_search_worker_ms;
  result.continuation_validation_ms = aggregate_continuation_validation_ms;
  if (strategic_directive != nullptr) {
    result.planning_goal = planning_goal;
    result.route_purpose = strategic_directive->route_purpose;
    result.observation_frontier = strategic_directive->observation_frontier;
    result.frontier_selection_score = strategic_directive->selection_score;
    result.reached_mission_goal =
        result.status == Lattice3DStatus::kReachedPlanningGoal &&
        strategic_directive->reaches_mission_goal &&
        distance3D(planning_goal, mission_goal) <= 1.0e-6;
    result.topology_candidates = std::move(selection.diagnostics);
  } else if (result.route_purpose == Lattice3DRoutePurpose::kMissionTransit) {
    result.planning_goal = planning_goal;
    result.topology_candidates = std::move(selection.diagnostics);
  }
  result.route_fingerprint =
      routeFingerprint(result.route, result.selected_passage_traversals);
  result.search_ms = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - search_started)
                         .count();
  return result;
}

} // namespace drone_city_nav
