#include "risk_aware_lattice_3d_continuation.hpp"

#include "drone_city_nav/bounded_worker_pool.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <queue>
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
};

} // namespace

Lattice3DContinuationMetrics evaluateLattice3DContinuation(
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
    const Point3& terminal, const Vec3& incoming_direction,
    const Lattice3DRiskStage stage, const RiskAwareLattice3DConfig& config,
    BoundedWorkerPool* const worker_pool) {
  constexpr std::array<int, 3> kOffsets{-1, 0, 1};
  const std::size_t maximum_states =
      std::max<std::size_t>(1U, config.frontier_validation_maximum_states);
  std::queue<Candidate> pending;
  std::unordered_set<OffsetKey, OffsetKeyHash> visited;
  const OffsetKey origin{};
  pending.push(Candidate{.key = origin, .point = terminal});
  visited.insert(origin);
  Lattice3DContinuationMetrics result;
  while (!pending.empty() && result.reachable_states < maximum_states &&
         result.reachable_depth_m + 1.0e-9 <
             config.frontier_minimum_reachable_depth_m) {
    const Candidate current = pending.front();
    pending.pop();
    const auto collection_started = std::chrono::steady_clock::now();

    struct NeighborEvaluation {
      Candidate candidate{};
      Lattice3DEdgeEvaluation edge{};
    };

    std::vector<NeighborEvaluation> evaluations;
    evaluations.reserve(26U);
    for (const int dx : kOffsets) {
      for (const int dy : kOffsets) {
        for (const int dz : kOffsets) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const OffsetKey next{current.key.x + dx, current.key.y + dy,
                               current.key.z + dz};
          if (visited.contains(next)) {
            continue;
          }
          visited.insert(next);
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
              .candidate = Candidate{.key = next, .point = successor}});
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
      if (current.key == origin) {
        ++result.immediate_successors;
      }
      ++result.reachable_states;
      result.reachable_depth_m = std::max(
          result.reachable_depth_m, distance3D(terminal, evaluation.candidate.point));
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
  return result;
}

} // namespace drone_city_nav::detail
