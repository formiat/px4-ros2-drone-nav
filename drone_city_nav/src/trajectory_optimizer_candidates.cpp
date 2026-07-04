#include "trajectory_optimizer_internal.hpp"

namespace drone_city_nav::trajectory_optimizer_detail {

[[nodiscard]] bool updateBestCandidate(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const double> candidate_offsets,
    const std::span<const Point2> candidate_points,
    const OccupancyGrid2D& prohibited_grid, const TrajectoryOptimizerConfig& config,
    double& best_cost, std::vector<double>& offsets, std::vector<Point2>& best_points,
    CandidateScore& best_score, double& best_length_m,
    std::vector<TrajectoryPointSample>& scratch_samples,
    SegmentProhibitedCountCache& segment_cache, TrajectoryOptimizerStats& stats,
    TrajectoryOptimizerCandidateDiagnostic* diagnostic) {
  ++stats.candidate_evaluations;
  ++stats.scratch_reused_candidates;
  const double incumbent_score = best_cost;
  const auto evaluation_started_at = std::chrono::steady_clock::now();
  std::size_t cache_hits = 0U;
  std::size_t cache_misses = 0U;
  const PathEvaluation evaluation = evaluatePathCached(
      prohibited_grid, candidate_points, segment_cache, cache_hits, cache_misses);
  const double path_evaluation_duration_ms = elapsedMilliseconds(evaluation_started_at);
  stats.full_path_segment_cache_hits += cache_hits;
  stats.full_path_segment_cache_misses += cache_misses;
  stats.candidate_path_evaluation_duration_ms += path_evaluation_duration_ms;
  if (!evaluation.traversable()) {
    ++stats.collision_rejections;
  }
  const auto score_started_at = std::chrono::steady_clock::now();
  const CandidateScore candidate_score =
      scoreForCandidate(corridor_samples, candidate_points, candidate_offsets,
                        evaluation, config, scratch_samples, stats, nullptr);
  const double score_duration_ms = elapsedMilliseconds(score_started_at);
  stats.candidate_score_duration_ms += score_duration_ms;
  stats.full_candidate_score_duration_ms += score_duration_ms;
  const bool accepted = candidate_score.score + 1.0e-9 < best_cost;
  if (diagnostic != nullptr) {
    populateCandidateDiagnosticFromScore(
        *diagnostic, candidate_score, evaluation, incumbent_score, accepted, 0.0,
        path_evaluation_duration_ms, score_duration_ms, score_duration_ms);
  }
  if (accepted) {
    best_cost = candidate_score.score;
    offsets.assign(candidate_offsets.begin(), candidate_offsets.end());
    best_points.assign(candidate_points.begin(), candidate_points.end());
    best_score = candidate_score;
    best_length_m = evaluation.length_m;
    stats.best_candidate_score = candidate_score.score;
    return true;
  }
  return false;
}

[[nodiscard]] EvaluatedCandidate evaluateCandidateSnapshot(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const double> base_offsets,
    const std::span<const Point2> base_points, const CandidateScore& base_score,
    const double base_length_m, const std::size_t center_index, const double delta_m,
    const OccupancyGrid2D& prohibited_grid, const TrajectoryOptimizerConfig& config,
    const std::span<const std::uint8_t> mutable_indices, const double incumbent_score,
    CandidateWorkBuffer& buffer) {
  EvaluatedCandidate result{};
  result.scratch_reused = true;
  result.snapshot_allocation_avoided =
      buffer.offsets.capacity() >= base_offsets.size() &&
      buffer.points.capacity() >= corridor_samples.size() &&
      buffer.samples.capacity() >= corridor_samples.size();
  buffer.offsets.assign(base_offsets.begin(), base_offsets.end());
  applyOffsetDelta(buffer.offsets, corridor_samples, center_index, delta_m,
                   mutable_indices);
  const OffsetChangeDiagnostics offset_diagnostics =
      offsetChangeDiagnostics(base_offsets, buffer.offsets);
  result.offset_changed_samples = offset_diagnostics.changed_samples;
  result.offset_changed_span_samples = offset_diagnostics.changed_span_samples;
  result.local_speed_window_samples =
      estimatedLocalSpeedWindowSamples(offset_diagnostics, base_offsets.size());
  if (offset_diagnostics.changed_samples == 0U ||
      offsetsNearlyEqual(buffer.offsets, base_offsets)) {
    result.noop = true;
    return result;
  }

  result.local_evaluated = true;
  LocalCandidateScore local_score = evaluateLocalOffsetPath(
      corridor_samples, base_points, base_offsets, buffer.offsets, prohibited_grid,
      base_score, base_length_m, center_index, buffer);
  result.point_build_duration_ms = local_score.point_build_duration_ms;
  result.path_evaluation_duration_ms = local_score.path_evaluation_duration_ms;
  result.score_duration_ms = local_score.score_duration_ms;
  result.local_point_build_duration_ms = local_score.point_build_duration_ms;
  result.local_path_evaluation_duration_ms = local_score.path_evaluation_duration_ms;
  result.local_score_duration_ms =
      local_score.path_evaluation_duration_ms + local_score.score_duration_ms;
  result.local_segment_cache_hits = local_score.segment_cache_hits;
  result.local_segment_cache_misses = local_score.segment_cache_misses;
  result.local_full_score_reason = local_score.full_score_reason;
  result.shadow_boundary_clamped_window_samples =
      local_score.shadow_boundary_clamped_window_samples;
  if (local_score.valid && !local_score.requires_full_score) {
    result.path = local_score.path;
    result.offsets.assign(buffer.offsets.begin(), buffer.offsets.end());
    if (!result.path.traversable()) {
      return result;
    }
    TrajectoryOptimizerStats local_stats{};
    const auto points_started_at = std::chrono::steady_clock::now();
    pointsFromOffsets(corridor_samples, buffer.offsets, buffer.points);
    result.point_build_duration_ms += elapsedMilliseconds(points_started_at);
    const auto score_started_at = std::chrono::steady_clock::now();
    const auto geometry_started_at = std::chrono::steady_clock::now();
    const std::optional<CostBreakdown> incremental_geometry =
        incrementalGeometryBreakdownForChangedSpan(
            base_score, base_points, base_offsets, buffer.points, buffer.offsets,
            offset_diagnostics, config);
    local_stats.candidate_cost_breakdown_duration_ms +=
        elapsedMilliseconds(geometry_started_at);
    result.score =
        scoreForCandidate(corridor_samples, buffer.points, buffer.offsets, result.path,
                          config, buffer.samples, local_stats,
                          incremental_geometry ? &*incremental_geometry : nullptr);
    populateShadowSegmentScoreDiagnostics(
        result, base_score, base_points, base_offsets, buffer.points, buffer.offsets,
        offset_diagnostics, config, incumbent_score, incremental_geometry);
    const double full_score_duration_ms = elapsedMilliseconds(score_started_at);
    result.score_duration_ms += full_score_duration_ms;
    result.full_score_duration_ms += full_score_duration_ms;
    result.sample_build_duration_ms = local_stats.candidate_sample_build_duration_ms;
    result.cost_breakdown_duration_ms =
        local_stats.candidate_cost_breakdown_duration_ms;
    result.shape_diagnostics_duration_ms =
        local_stats.candidate_shape_diagnostics_duration_ms;
    result.full_score_used = true;
    return result;
  }

  result.requires_full_score = true;
  result.offsets.assign(buffer.offsets.begin(), buffer.offsets.end());

  TrajectoryOptimizerStats local_stats{};
  const auto points_started_at = std::chrono::steady_clock::now();
  pointsFromOffsets(corridor_samples, buffer.offsets, buffer.points);
  result.point_build_duration_ms += elapsedMilliseconds(points_started_at);
  const auto full_evaluation_started_at = std::chrono::steady_clock::now();
  result.path = evaluatePathCached(
      prohibited_grid, buffer.points, buffer.full_path_segment_cache,
      result.full_path_segment_cache_hits, result.full_path_segment_cache_misses);
  result.path_evaluation_duration_ms += elapsedMilliseconds(full_evaluation_started_at);
  const auto score_started_at = std::chrono::steady_clock::now();
  const auto geometry_started_at = std::chrono::steady_clock::now();
  const std::optional<CostBreakdown> incremental_geometry =
      incrementalGeometryBreakdownForChangedSpan(base_score, base_points, base_offsets,
                                                 buffer.points, buffer.offsets,
                                                 offset_diagnostics, config);
  local_stats.candidate_cost_breakdown_duration_ms +=
      elapsedMilliseconds(geometry_started_at);
  result.score =
      scoreForCandidate(corridor_samples, buffer.points, buffer.offsets, result.path,
                        config, buffer.samples, local_stats,
                        incremental_geometry ? &*incremental_geometry : nullptr);
  populateShadowSegmentScoreDiagnostics(
      result, base_score, base_points, base_offsets, buffer.points, buffer.offsets,
      offset_diagnostics, config, incumbent_score, incremental_geometry);
  const double full_score_duration_ms = elapsedMilliseconds(score_started_at);
  result.score_duration_ms += full_score_duration_ms;
  result.full_score_duration_ms += full_score_duration_ms;
  result.sample_build_duration_ms = local_stats.candidate_sample_build_duration_ms;
  result.cost_breakdown_duration_ms = local_stats.candidate_cost_breakdown_duration_ms;
  result.shape_diagnostics_duration_ms =
      local_stats.candidate_shape_diagnostics_duration_ms;
  result.full_score_used = true;
  return result;
}

[[nodiscard]] std::vector<double>
offsetCandidatesForSample(const CorridorSample& sample, const double offset_step_m,
                          const std::optional<double> center_offset,
                          const double radius_m) {
  const double step = sanitizedPositive(offset_step_m, 1.0, 0.05, 100.0);
  std::vector<double> candidates;
  const double lower_bound =
      center_offset.has_value() && std::isfinite(radius_m)
          ? std::max(-sample.right_bound_m, *center_offset - radius_m)
          : -sample.right_bound_m;
  const double upper_bound =
      center_offset.has_value() && std::isfinite(radius_m)
          ? std::min(sample.left_bound_m, *center_offset + radius_m)
          : sample.left_bound_m;
  const auto push_if_allowed = [&](const double offset) {
    if (offset + 1.0e-9 >= lower_bound && offset <= upper_bound + 1.0e-9) {
      candidates.push_back(std::clamp(offset, lower_bound, upper_bound));
    }
  };
  push_if_allowed(0.0);
  if (center_offset.has_value()) {
    push_if_allowed(*center_offset);
  }
  if (std::isfinite(sample.left_bound_m) && sample.left_bound_m > 0.0) {
    const auto left_steps =
        static_cast<std::size_t>(std::floor(sample.left_bound_m / step));
    for (std::size_t step_index = 1U; step_index <= left_steps; ++step_index) {
      const double offset = static_cast<double>(step_index) * step;
      push_if_allowed(std::min(offset, sample.left_bound_m));
    }
    push_if_allowed(sample.left_bound_m);
  }
  if (std::isfinite(sample.right_bound_m) && sample.right_bound_m > 0.0) {
    const auto right_steps =
        static_cast<std::size_t>(std::floor(sample.right_bound_m / step));
    for (std::size_t step_index = 1U; step_index <= right_steps; ++step_index) {
      const double offset = -static_cast<double>(step_index) * step;
      push_if_allowed(std::max(offset, -sample.right_bound_m));
    }
    push_if_allowed(-sample.right_bound_m);
  }
  if (candidates.empty() && lower_bound <= upper_bound) {
    candidates.push_back(
        std::clamp(center_offset.value_or(0.0), lower_bound, upper_bound));
  }
  std::sort(candidates.begin(), candidates.end());
  candidates.erase(std::unique(candidates.begin(), candidates.end(),
                               [](const double lhs, const double rhs) {
                                 return std::abs(lhs - rhs) <= 1.0e-9;
                               }),
                   candidates.end());
  return candidates;
}

[[nodiscard]] std::vector<CandidateTask>
candidateTasksForStep(const std::span<const std::size_t> control_indices,
                      const double step) {
  std::vector<CandidateTask> tasks;
  tasks.reserve(control_indices.size() * 2U);
  for (const std::size_t index : control_indices) {
    tasks.push_back(CandidateTask{
        .order = tasks.size(),
        .center_index = index,
        .delta_m = -step,
    });
    tasks.push_back(CandidateTask{
        .order = tasks.size(),
        .center_index = index,
        .delta_m = step,
    });
  }
  return tasks;
}

void incrementLocalFullScoreReason(const LocalFullScoreReason reason,
                                   TrajectoryOptimizerStats& stats) {
  switch (reason) {
    case LocalFullScoreReason::kNone:
      return;
    case LocalFullScoreReason::kInvalidInput:
      ++stats.local_candidate_full_score_required_invalid_input;
      return;
    case LocalFullScoreReason::kBoundaryWindow:
      ++stats.local_candidate_full_score_required_boundary;
      return;
    case LocalFullScoreReason::kUnsafeBase:
      ++stats.local_candidate_full_score_required_unsafe_base;
      return;
    case LocalFullScoreReason::kWindowInvalid:
      ++stats.local_candidate_full_score_required_window_invalid;
      return;
  }
}

void mergeCandidateStats(const EvaluatedCandidate& candidate,
                         TrajectoryOptimizerStats& stats) {
  if (candidate.scratch_reused) {
    ++stats.worker_scratch_reuses;
  }
  if (candidate.snapshot_allocation_avoided) {
    ++stats.candidate_snapshot_allocations_avoided;
  }
  stats.candidate_offset_changed_samples_total += candidate.offset_changed_samples;
  stats.candidate_offset_changed_samples_max = std::max(
      stats.candidate_offset_changed_samples_max, candidate.offset_changed_samples);
  stats.candidate_offset_changed_span_samples_total +=
      candidate.offset_changed_span_samples;
  stats.candidate_offset_changed_span_samples_max =
      std::max(stats.candidate_offset_changed_span_samples_max,
               candidate.offset_changed_span_samples);
  stats.candidate_local_speed_window_samples_total +=
      candidate.local_speed_window_samples;
  stats.candidate_local_speed_window_samples_max =
      std::max(stats.candidate_local_speed_window_samples_max,
               candidate.local_speed_window_samples);
  if (candidate.noop) {
    ++stats.skipped_noop_candidates;
    return;
  }
  if (candidate.local_evaluated) {
    ++stats.local_candidate_evaluations;
    stats.local_candidate_point_build_duration_ms +=
        candidate.local_point_build_duration_ms;
    stats.local_candidate_path_evaluation_duration_ms +=
        candidate.local_path_evaluation_duration_ms;
    stats.local_candidate_score_duration_ms += candidate.local_score_duration_ms;
    stats.candidate_segment_cache_hits += candidate.local_segment_cache_hits;
    stats.candidate_segment_cache_misses += candidate.local_segment_cache_misses;
    if (candidate.requires_full_score) {
      ++stats.local_candidate_full_score_required;
      incrementLocalFullScoreReason(candidate.local_full_score_reason, stats);
      if (candidate.local_full_score_reason == LocalFullScoreReason::kBoundaryWindow) {
        ++stats.shadow_boundary_clamped_local_candidates;
        stats.shadow_boundary_clamped_window_samples_total +=
            candidate.shadow_boundary_clamped_window_samples;
        stats.shadow_boundary_clamped_window_samples_max =
            std::max(stats.shadow_boundary_clamped_window_samples_max,
                     candidate.shadow_boundary_clamped_window_samples);
      }
    }
  }
  stats.full_path_segment_cache_hits += candidate.full_path_segment_cache_hits;
  stats.full_path_segment_cache_misses += candidate.full_path_segment_cache_misses;
  if (candidate.full_score_used) {
    ++stats.local_candidate_full_score_fallbacks;
    stats.full_candidate_score_duration_ms += candidate.full_score_duration_ms;
  }
  if (candidate.shadow_segment_score_valid) {
    ++stats.shadow_segment_score_evaluations;
    stats.shadow_segment_score_window_samples_total +=
        candidate.shadow_segment_score_window_samples;
    stats.shadow_segment_score_window_samples_max =
        std::max(stats.shadow_segment_score_window_samples_max,
                 candidate.shadow_segment_score_window_samples);
    const double score_delta =
        candidate.shadow_segment_score_estimated_score - candidate.score.score;
    if (std::isfinite(score_delta)) {
      stats.shadow_segment_score_abs_error_sum += std::abs(score_delta);
      stats.shadow_segment_score_max_overestimate =
          std::max(stats.shadow_segment_score_max_overestimate, score_delta);
      stats.shadow_segment_score_max_underestimate =
          std::max(stats.shadow_segment_score_max_underestimate, -score_delta);
    }
    if (candidate.shadow_segment_score_would_prune) {
      ++stats.shadow_segment_score_prunable;
      if (std::isfinite(candidate.score.score) &&
          std::isfinite(candidate.shadow_segment_score_incumbent_score) &&
          candidate.score.score + 1.0e-9 <
              candidate.shadow_segment_score_incumbent_score) {
        ++stats.shadow_segment_score_false_prunes;
        stats.shadow_segment_score_max_false_prune_improvement_score = std::max(
            stats.shadow_segment_score_max_false_prune_improvement_score,
            candidate.shadow_segment_score_incumbent_score - candidate.score.score);
      }
    }
  } else if (candidate.local_evaluated) {
    ++stats.shadow_segment_score_unavailable;
  }
  ++stats.candidate_evaluations;
  stats.candidate_point_build_duration_ms += candidate.point_build_duration_ms;
  stats.candidate_path_evaluation_duration_ms += candidate.path_evaluation_duration_ms;
  stats.candidate_score_duration_ms += candidate.score_duration_ms;
  stats.candidate_sample_build_duration_ms += candidate.sample_build_duration_ms;
  stats.candidate_cost_breakdown_duration_ms += candidate.cost_breakdown_duration_ms;
  stats.candidate_shape_diagnostics_duration_ms +=
      candidate.shape_diagnostics_duration_ms;
  if (!candidate.path.traversable()) {
    ++stats.collision_rejections;
  }
}

[[nodiscard]] TrajectoryOptimizerCandidateDiagnostic candidateDiagnosticFromBatchResult(
    const CandidateBatchResult& batch_result,
    const std::span<const CorridorSample> corridor_samples, const std::size_t iteration,
    const double step_m, const double incumbent_score, const bool selected) {
  const EvaluatedCandidate& candidate = batch_result.candidate;
  TrajectoryOptimizerCandidateDiagnostic diagnostic{};
  diagnostic.phase = "iteration";
  diagnostic.iteration = iteration;
  diagnostic.order = batch_result.order;
  diagnostic.center_index = batch_result.center_index;
  diagnostic.step_m = step_m;
  diagnostic.delta_m = batch_result.delta_m;
  if (batch_result.center_index < corridor_samples.size()) {
    diagnostic.center_s_m = corridor_samples[batch_result.center_index].s_m;
  }
  diagnostic.score = candidate.score.score;
  diagnostic.incumbent_score = incumbent_score;
  diagnostic.length_m = candidate.path.length_m;
  diagnostic.noop = candidate.noop;
  diagnostic.traversable = candidate.path.traversable();
  diagnostic.local_evaluated = candidate.local_evaluated;
  diagnostic.requires_full_score = candidate.requires_full_score;
  diagnostic.full_score_used = candidate.full_score_used;
  diagnostic.local_full_score_reason =
      localFullScoreReasonName(candidate.local_full_score_reason);
  diagnostic.prohibited_cells = candidate.path.prohibited_cells;
  diagnostic.outside_grid_segments = candidate.path.outside_grid_segments;
  diagnostic.changed_samples = candidate.offset_changed_samples;
  diagnostic.changed_span_samples = candidate.offset_changed_span_samples;
  diagnostic.point_build_duration_ms = candidate.point_build_duration_ms;
  diagnostic.path_evaluation_duration_ms = candidate.path_evaluation_duration_ms;
  diagnostic.score_duration_ms = candidate.score_duration_ms;
  diagnostic.full_score_duration_ms = candidate.full_score_duration_ms;
  copyCostBreakdownToCandidateDiagnostic(candidate.score.breakdown, diagnostic);

  if (selected) {
    diagnostic.decision = "selected";
  } else if (candidate.noop) {
    diagnostic.decision = "noop";
  } else if (candidate.path.outside_grid_segments > 0U) {
    diagnostic.decision = "outside_grid";
  } else if (candidate.path.prohibited_cells > 0U) {
    diagnostic.decision = "prohibited";
  } else if (!candidate.full_score_used) {
    diagnostic.decision = "not_scored";
  } else if (std::isfinite(candidate.score.score) && std::isfinite(incumbent_score) &&
             candidate.score.score + 1.0e-9 < incumbent_score) {
    diagnostic.decision = "valid_not_iteration_best";
  } else {
    diagnostic.decision = "not_better_than_incumbent";
  }
  return diagnostic;
}

} // namespace drone_city_nav::trajectory_optimizer_detail
