#include "risk_aware_lattice_successors.hpp"

#include "drone_city_nav/bounded_worker_pool.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <unordered_set>

#include "risk_aware_lattice_geometry.hpp"

namespace drone_city_nav::detail {

std::size_t LatticeKeyHash::operator()(const LatticeKey& key) const noexcept {
  const std::uint64_t x = static_cast<std::uint32_t>(key.x);
  const std::uint64_t y = static_cast<std::uint32_t>(key.y);
  const std::uint64_t heading = static_cast<std::uint32_t>(key.heading);
  return static_cast<std::size_t>((x << 32U) ^ (y << 8U) ^ heading);
}

Point2 latticeCellCenter(const mppi::EsdfGrid& grid, const LatticeKey& key) noexcept {
  return Point2{
      grid.origin_x_m + (static_cast<double>(key.x) + 0.5) * grid.resolution_m,
      grid.origin_y_m + (static_cast<double>(key.y) + 0.5) * grid.resolution_m};
}

LatticeSuccessorCollection collectLatticeSuccessors(
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
    const Point2 current, const LatticeKey& current_key, const LatticeRiskStage stage,
    const RiskAwareLatticeConfig& config, const LatticeSearchRoi& search_roi,
    const std::span<const LatticeFrontierBlacklistEntry> frontier_blacklist,
    BoundedWorkerPool* const worker_pool) {
  struct CandidateSpec {
    int next_heading{0};
    double length_m{0.0};
  };

  struct CandidateEvaluation {
    Point2 endpoint{};
    LatticeKey key{};
    SegmentEvaluation segment{};
    FailureMemoryEvaluation failure_memory{};
    bool inside_roi{false};
  };

  const auto started = std::chrono::steady_clock::now();
  const auto make_key = [&grid, &config](const Point2 point, const int heading) {
    return LatticeKey{
        static_cast<int>(std::floor((point.x - grid.origin_x_m) / grid.resolution_m)),
        static_cast<int>(std::floor((point.y - grid.origin_y_m) / grid.resolution_m)),
        wrapLatticeHeading(heading, config.heading_bins)};
  };
  const auto inside_roi = [&search_roi](const Point2 point) {
    return point.x >= search_roi.minimum_x && point.x <= search_roi.maximum_x &&
           point.y >= search_roi.minimum_y && point.y <= search_roi.maximum_y;
  };

  constexpr std::array normal_heading_offsets{-4, -2, -1, 0, 1, 2, 4, 8};
  std::vector<CandidateSpec> specs;
  specs.reserve(static_cast<std::size_t>(config.heading_bins) +
                normal_heading_offsets.size());
  for (int heading = 0; heading < config.heading_bins; ++heading) {
    specs.push_back(CandidateSpec{heading, config.short_primitive_length_m});
  }
  for (const int offset : normal_heading_offsets) {
    specs.push_back(CandidateSpec{
        wrapLatticeHeading(current_key.heading + offset, config.heading_bins),
        config.primitive_length_m});
  }

  std::vector<CandidateEvaluation> evaluations(specs.size());
  const auto evaluate = [&](const std::size_t index) {
    const CandidateSpec& spec = specs[index];
    const double heading = latticeHeadingForBin(spec.next_heading, config.heading_bins);
    CandidateEvaluation candidate;
    candidate.endpoint = Point2{current.x + std::cos(heading) * spec.length_m,
                                current.y + std::sin(heading) * spec.length_m};
    candidate.inside_roi = inside_roi(candidate.endpoint);
    if (candidate.inside_roi) {
      candidate.segment = evaluateLatticeSegment(grid, esdf_m, current,
                                                 candidate.endpoint, config, stage);
      if (candidate.segment.valid) {
        const std::array segment{current, candidate.endpoint};
        candidate.failure_memory =
            evaluateLatticeFailureMemory(segment, frontier_blacklist, config);
        candidate.key = make_key(candidate.endpoint, spec.next_heading);
      }
    }
    evaluations[index] = candidate;
  };
  const bool parallel = worker_pool != nullptr &&
                        worker_pool->canParallelizeFromCurrentThread() &&
                        specs.size() > 1U;
  if (parallel) {
    worker_pool->parallelFor(specs.size(), evaluate);
  } else {
    for (std::size_t index = 0U; index < specs.size(); ++index) {
      evaluate(index);
    }
  }

  LatticeSuccessorCollection result;
  result.successors.reserve(specs.size());
  std::unordered_set<LatticeKey, LatticeKeyHash> emitted;
  for (std::size_t index = 0U; index < specs.size(); ++index) {
    const CandidateSpec& spec = specs[index];
    const CandidateEvaluation& candidate = evaluations[index];
    ++result.diagnostics.generated;
    if (!candidate.inside_roi) {
      result.roi_boundary_seen = true;
      ++result.diagnostics.rejected_outside_roi;
      continue;
    }
    if (!candidate.segment.valid) {
      recordLatticeSegmentRejection(candidate.segment, result.diagnostics);
      continue;
    }
    if (candidate.failure_memory.hard_rejected) {
      ++result.diagnostics.rejected_blacklisted_failure;
      continue;
    }
    if (!emitted.insert(candidate.key).second) {
      continue;
    }
    const int heading_change = latticeHeadingBinDistance(
        current_key.heading, spec.next_heading, config.heading_bins);
    result.successors.push_back(LatticeSuccessor{
        .key = candidate.key,
        .endpoint = candidate.endpoint,
        .length_m = spec.length_m,
        .edge_cost = spec.length_m + candidate.segment.risk_cost +
                     candidate.failure_memory.soft_penalty_cost +
                     config.turn_cost * static_cast<double>(heading_change)});
    if (candidate.failure_memory.soft_penalty_cost > 0.0) {
      ++result.diagnostics.soft_tabu_penalties_applied;
    }
    ++result.diagnostics.accepted;
  }
  result.profile.collection_calls = 1U;
  result.profile.parallel_collection_calls = parallel ? 1U : 0U;
  result.profile.candidates = specs.size();
  result.profile.parallel_candidates = parallel ? specs.size() : 0U;
  result.profile.maximum_candidates = specs.size();
  result.profile.worker_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - started)
                                 .count();
  return result;
}

} // namespace drone_city_nav::detail
