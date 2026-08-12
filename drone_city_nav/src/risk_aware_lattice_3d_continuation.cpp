#include "risk_aware_lattice_3d_continuation.hpp"

#include "drone_city_nav/bounded_worker_pool.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "risk_aware_lattice_3d_geometry.hpp"

namespace drone_city_nav::detail {
namespace {

struct OffsetKey {
  int x{0};
  int y{0};
  int z{0};

  [[nodiscard]] bool operator==(const OffsetKey&) const noexcept = default;
};

struct OffsetKeyHash {
  [[nodiscard]] std::size_t operator()(const OffsetKey& key) const noexcept {
    std::size_t seed = std::hash<int>{}(key.x);
    const auto combine = [&seed](const std::size_t value) {
      seed ^= value + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    };
    combine(std::hash<int>{}(key.y));
    combine(std::hash<int>{}(key.z));
    return seed;
  }
};

struct Candidate {
  OffsetKey key{};
  Point3 point{};
  double goal_distance_m{0.0};
  double goal_altitude_error_m{0.0};
  std::size_t hops{0U};
  std::uint64_t sequence{0U};
};

struct CandidateGreater {
  [[nodiscard]] bool operator()(const Candidate& left,
                                const Candidate& right) const noexcept {
    return std::tuple{left.goal_altitude_error_m, left.goal_distance_m, left.hops,
                      left.sequence} > std::tuple{right.goal_altitude_error_m,
                                                  right.goal_distance_m, right.hops,
                                                  right.sequence};
  }
};

} // namespace

Lattice3DContinuationMetrics evaluateLattice3DContinuation(
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
    const Point3& terminal, const Vec3& incoming_direction, const Point3& planning_goal,
    const Lattice3DRiskStage stage, const RiskAwareLattice3DConfig& config,
    BoundedWorkerPool* const worker_pool) {
  constexpr std::array<int, 3> kHorizontalOffsets{-1, 0, 1};
  constexpr std::array<int, 3> kVerticalOffsets{0, 1, -1};
  const std::size_t maximum_states =
      std::max<std::size_t>(1U, config.frontier_validation_maximum_states);
  std::priority_queue<Candidate, std::vector<Candidate>, CandidateGreater> pending;
  std::unordered_set<OffsetKey, OffsetKeyHash> visited;
  std::unordered_map<OffsetKey, OffsetKey, OffsetKeyHash> parents;
  const OffsetKey origin{};
  std::uint64_t sequence = 0U;
  pending.push(
      Candidate{.key = origin,
                .point = terminal,
                .goal_distance_m = distance3D(terminal, planning_goal),
                .goal_altitude_error_m = std::abs(terminal.z - planning_goal.z),
                .hops = 0U,
                .sequence = sequence++});
  visited.insert(origin);
  Lattice3DContinuationMetrics result;
  OffsetKey best_key = origin;
  Point3 best_point = terminal;
  double best_depth_m = 0.0;
  double best_goal_distance_m = distance3D(terminal, planning_goal);
  std::uint64_t best_sequence = std::numeric_limits<std::uint64_t>::max();
  while (!pending.empty() && result.reachable_states < maximum_states) {
    const Candidate current = pending.top();
    pending.pop();
    const double current_depth_m = distance3D(terminal, current.point);
    if (!(current.key == origin) &&
        current_depth_m + 1.0e-9 >= config.frontier_minimum_reachable_depth_m) {
      best_key = current.key;
      best_point = current.point;
      break;
    }
    const auto collection_started = std::chrono::steady_clock::now();

    struct NeighborEvaluation {
      Candidate candidate{};
      Lattice3DEdgeEvaluation edge{};
    };

    std::vector<NeighborEvaluation> evaluations;
    evaluations.reserve(26U);
    for (const int dx : kHorizontalOffsets) {
      for (const int dy : kHorizontalOffsets) {
        for (const int dz : kVerticalOffsets) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const OffsetKey next{current.key.x + dx, current.key.y + dy,
                               current.key.z + dz};
          if (visited.contains(next)) {
            continue;
          }
          const Point3 successor{
              terminal.x + static_cast<double>(next.x) * config.horizontal_step_m,
              terminal.y + static_cast<double>(next.y) * config.horizontal_step_m,
              terminal.z + static_cast<double>(next.z) * config.vertical_step_m};
          if (current.key == origin) {
            const double departure_length = distance3D(terminal, successor);
            const Vec3 departure{(successor.x - terminal.x) / departure_length,
                                 (successor.y - terminal.y) / departure_length,
                                 (successor.z - terminal.z) / departure_length};
            const double alignment = departure.x * incoming_direction.x +
                                     departure.y * incoming_direction.y +
                                     departure.z * incoming_direction.z;
            if (alignment < -1.0e-6) {
              continue;
            }
          }
          evaluations.push_back(NeighborEvaluation{
              .candidate = Candidate{
                  .key = next,
                  .point = successor,
                  .goal_distance_m = distance3D(successor, planning_goal),
                  .goal_altitude_error_m = std::abs(successor.z - planning_goal.z),
                  .hops = current.hops + 1U,
                  .sequence = sequence++,
              }});
        }
      }
    }
    const auto evaluate_neighbor = [&](const std::size_t candidate_index) {
      NeighborEvaluation& evaluation = evaluations[candidate_index];
      evaluation.edge = evaluateLattice3DEdge(
          grid, esdf_m, current.point, evaluation.candidate.point, stage, config);
    };
    const bool parallel = worker_pool != nullptr &&
                          worker_pool->canParallelizeFromCurrentThread() &&
                          evaluations.size() > 1U;
    if (parallel) {
      worker_pool->parallelFor(evaluations.size(), evaluate_neighbor);
    } else {
      for (std::size_t index = 0U; index < evaluations.size(); ++index) {
        evaluate_neighbor(index);
      }
    }
    for (const NeighborEvaluation& evaluation : evaluations) {
      if (evaluation.edge.status != Lattice3DEdgeEvaluationStatus::kValid) {
        continue;
      }
      if (!visited.insert(evaluation.candidate.key).second) {
        continue;
      }
      if (current.key == origin) {
        ++result.immediate_successors;
      }
      ++result.reachable_states;
      const double candidate_depth_m = distance3D(terminal, evaluation.candidate.point);
      const double candidate_goal_distance_m = evaluation.candidate.goal_distance_m;
      result.reachable_depth_m = std::max(result.reachable_depth_m, candidate_depth_m);
      parents[evaluation.candidate.key] = current.key;
      if (std::tuple{-candidate_depth_m, evaluation.candidate.goal_altitude_error_m,
                     candidate_goal_distance_m, evaluation.candidate.sequence} <
          std::tuple{-best_depth_m, std::abs(best_point.z - planning_goal.z),
                     best_goal_distance_m, best_sequence}) {
        best_key = evaluation.candidate.key;
        best_point = evaluation.candidate.point;
        best_depth_m = candidate_depth_m;
        best_goal_distance_m = candidate_goal_distance_m;
        best_sequence = evaluation.candidate.sequence;
      }
      pending.push(evaluation.candidate);
    }
    ++result.successor_profile.collection_calls;
    result.successor_profile.candidates += evaluations.size();
    if (parallel) {
      ++result.successor_profile.parallel_collection_calls;
      result.successor_profile.parallel_candidates += evaluations.size();
    }
    result.successor_profile.maximum_candidates =
        std::max(result.successor_profile.maximum_candidates, evaluations.size());
    result.successor_profile.worker_ms +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  collection_started)
            .count();
  }
  if (!(best_key == origin)) {
    std::vector<Point3> reverse_path;
    OffsetKey current = best_key;
    reverse_path.push_back(best_point);
    while (!(current == origin)) {
      current = parents.at(current);
      reverse_path.push_back(
          Point3{terminal.x + static_cast<double>(current.x) * config.horizontal_step_m,
                 terminal.y + static_cast<double>(current.y) * config.horizontal_step_m,
                 terminal.z + static_cast<double>(current.z) * config.vertical_step_m});
    }
    result.path.assign(reverse_path.rbegin(), reverse_path.rend());
  }
  return result;
}

} // namespace drone_city_nav::detail
