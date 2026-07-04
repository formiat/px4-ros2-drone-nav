#include "trajectory_optimizer_internal.hpp"

namespace drone_city_nav::trajectory_optimizer_detail {

[[nodiscard]] bool buildDpSeedForWindow(
    const std::span<const CorridorSample> corridor_samples, const ActiveWindow& window,
    const OccupancyGrid2D& prohibited_grid, const TrajectoryOptimizerConfig& config,
    const double requested_step_m, const std::span<const double> base_offsets,
    const std::span<const double> guide_offsets, const double guide_radius_m,
    std::vector<double>& output_offsets, TrajectoryOptimizerStats& stats) {
  if (window.end_index <= window.begin_index + 1U ||
      base_offsets.size() != corridor_samples.size()) {
    return false;
  }

  const auto started_at = std::chrono::steady_clock::now();
  const double offset_step_m = sanitizedPositive(requested_step_m, 1.0, 0.05, 100.0);
  std::vector<std::size_t> indices;
  indices.reserve(window.end_index - window.begin_index - 1U);
  for (std::size_t i = window.begin_index + 1U; i < window.end_index; ++i) {
    indices.push_back(i);
  }
  if (indices.empty()) {
    stats.dp_duration_ms += elapsedMilliseconds(started_at);
    return false;
  }

  std::vector<std::vector<double>> offset_candidates;
  offset_candidates.reserve(indices.size());
  for (const std::size_t sample_index : indices) {
    const std::optional<double> guide =
        guide_offsets.size() == corridor_samples.size()
            ? std::optional<double>{guide_offsets[sample_index]}
            : std::nullopt;
    offset_candidates.push_back(offsetCandidatesForSample(
        corridor_samples[sample_index], offset_step_m, guide, guide_radius_m));
    stats.dp_states += offset_candidates.back().size();
  }

  constexpr double kDpInfinity = std::numeric_limits<double>::infinity();
  std::vector<std::vector<double>> cost(offset_candidates.size());
  std::vector<std::vector<std::size_t>> parent(offset_candidates.size());
  for (std::size_t row = 0U; row < offset_candidates.size(); ++row) {
    cost[row].assign(offset_candidates[row].size(), kDpInfinity);
    parent[row].assign(offset_candidates[row].size(), 0U);
  }

  const auto point_for = [&](const std::size_t sample_index, const double offset) {
    return corridor_samples[sample_index].center +
           corridor_samples[sample_index].normal * offset;
  };
  SegmentTraversabilityCache segment_cache{};
  const Point2 window_start =
      point_for(window.begin_index, base_offsets[window.begin_index]);
  const Point2 window_end = point_for(window.end_index, base_offsets[window.end_index]);
  const double weight_offset_change =
      sanitizedPositive(config.weight_offset_change, 0.5, 0.0, 1.0e9);

  for (std::size_t candidate_index = 0U;
       candidate_index < offset_candidates.front().size(); ++candidate_index) {
    const double offset = offset_candidates.front()[candidate_index];
    const Point2 point = point_for(indices.front(), offset);
    if (!cachedSegmentTraversable(prohibited_grid, window_start, point, segment_cache,
                                  stats.dp_segment_cache_hits,
                                  stats.dp_segment_cache_misses)) {
      continue;
    }
    const double offset_delta = offset - base_offsets[window.begin_index];
    cost.front()[candidate_index] = weight_offset_change * offset_delta * offset_delta;
  }

  for (std::size_t row = 1U; row < offset_candidates.size(); ++row) {
    const std::size_t previous_sample_index = indices[row - 1U];
    const std::size_t sample_index = indices[row];
    for (std::size_t candidate_index = 0U;
         candidate_index < offset_candidates[row].size(); ++candidate_index) {
      const double offset = offset_candidates[row][candidate_index];
      const Point2 point = point_for(sample_index, offset);
      for (std::size_t previous_candidate_index = 0U;
           previous_candidate_index < offset_candidates[row - 1U].size();
           ++previous_candidate_index) {
        ++stats.dp_transitions;
        if (!std::isfinite(cost[row - 1U][previous_candidate_index])) {
          continue;
        }
        const double previous_offset =
            offset_candidates[row - 1U][previous_candidate_index];
        const Point2 previous_point = point_for(previous_sample_index, previous_offset);
        if (!cachedSegmentTraversable(prohibited_grid, previous_point, point,
                                      segment_cache, stats.dp_segment_cache_hits,
                                      stats.dp_segment_cache_misses)) {
          continue;
        }
        const double offset_delta = offset - previous_offset;
        const double candidate_cost =
            cost[row - 1U][previous_candidate_index] +
            weight_offset_change * offset_delta * offset_delta;
        if (candidate_cost + 1.0e-9 < cost[row][candidate_index]) {
          cost[row][candidate_index] = candidate_cost;
          parent[row][candidate_index] = previous_candidate_index;
        }
      }
    }
  }

  std::size_t best_index = 0U;
  double best_cost = kDpInfinity;
  const std::size_t last_row = offset_candidates.size() - 1U;
  const std::size_t last_sample_index = indices.back();
  for (std::size_t candidate_index = 0U;
       candidate_index < offset_candidates[last_row].size(); ++candidate_index) {
    if (!std::isfinite(cost[last_row][candidate_index])) {
      continue;
    }
    const double offset = offset_candidates[last_row][candidate_index];
    const Point2 point = point_for(last_sample_index, offset);
    if (!cachedSegmentTraversable(prohibited_grid, point, window_end, segment_cache,
                                  stats.dp_segment_cache_hits,
                                  stats.dp_segment_cache_misses)) {
      continue;
    }
    const double offset_delta = base_offsets[window.end_index] - offset;
    const double candidate_cost = cost[last_row][candidate_index] +
                                  weight_offset_change * offset_delta * offset_delta;
    if (candidate_cost + 1.0e-9 < best_cost) {
      best_cost = candidate_cost;
      best_index = candidate_index;
    }
  }

  if (!std::isfinite(best_cost)) {
    stats.dp_duration_ms += elapsedMilliseconds(started_at);
    return false;
  }

  output_offsets.assign(base_offsets.begin(), base_offsets.end());
  std::size_t current_index = best_index;
  for (std::size_t reverse_row = offset_candidates.size(); reverse_row > 0U;
       --reverse_row) {
    const std::size_t row = reverse_row - 1U;
    output_offsets[indices[row]] = offset_candidates[row][current_index];
    if (row > 0U) {
      current_index = parent[row][current_index];
    }
  }
  stats.dp_duration_ms += elapsedMilliseconds(started_at);
  return true;
}

void smoothedOffsets(const std::span<const double> offsets,
                     const std::span<const CorridorSample> corridor_samples,
                     std::vector<double>& smoothed) {
  smoothed.assign(offsets.begin(), offsets.end());
  if (offsets.size() <= 2U || offsets.size() != corridor_samples.size()) {
    return;
  }
  for (std::size_t i = 1U; i + 1U < offsets.size(); ++i) {
    const double value =
        0.25 * offsets[i - 1U] + 0.5 * offsets[i] + 0.25 * offsets[i + 1U];
    smoothed[i] = std::clamp(value, -corridor_samples[i].right_bound_m,
                             corridor_samples[i].left_bound_m);
  }
}

} // namespace drone_city_nav::trajectory_optimizer_detail
