#include "trajectory_optimizer_internal.hpp"

namespace drone_city_nav {

using namespace trajectory_optimizer_detail;

TrajectoryOptimizerResult
optimizeTrajectory(const std::span<const CorridorSample> corridor_samples,
                   const OccupancyGrid2D& prohibited_grid,
                   const TrajectoryOptimizerConfig& config,
                   const VelocityFollowerConfig& speed_config) {
  TrajectoryOptimizerResult result{};
  result.stats.input_samples = corridor_samples.size();
  if (corridor_samples.size() < 2U) {
    return result;
  }

  const std::vector<CorridorSample> optimizer_samples =
      optimizerCorridorSamples(corridor_samples, config);
  const std::size_t sample_count = optimizer_samples.size();
  result.stats.optimizer_samples = sample_count;
  const std::vector<double> zero_offsets(sample_count, 0.0);
  const std::vector<Point2> centerline =
      pointsFromOffsets(optimizer_samples, zero_offsets);
  result.stats.centerline_length_m = pathLength(centerline);
  const std::vector<ActiveWindow> active_windows = detectActiveWindows(
      optimizer_samples, centerline, prohibited_grid, config, result.stats);
  result.active_windows = windowMetadata(active_windows, optimizer_samples);
  std::vector<std::uint8_t> mutable_indices;
  const std::vector<std::size_t> control_indices =
      activeControlIndices(active_windows, sample_count, mutable_indices);
  result.stats.parallel_workers_used = 1U;

  const double min_step =
      sanitizedPositive(config.min_offset_step_m, 0.1, 0.001, 100.0);
  const double cooling = sanitizedPositive(config.cooling_ratio, 0.5, 0.05, 0.95);
  double step = std::max(
      min_step, sanitizedPositive(config.initial_offset_step_m, 2.0, 0.001, 500.0));
  const std::size_t max_iterations = std::clamp<std::size_t>(
      config.max_iterations, 1U, static_cast<std::size_t>(10000U));
  result.stats.candidate_diagnostics.reserve(
      control_indices.size() * 2U * max_iterations + active_windows.size() + 8U);

  TrajectoryOptimizerScratch scratch{};
  scratch.candidate_offsets.reserve(sample_count);
  scratch.accepted_offsets.reserve(sample_count);
  scratch.iteration_best_offsets.reserve(sample_count);
  scratch.smoothed_offsets.reserve(sample_count);
  scratch.candidate_points.reserve(sample_count);
  scratch.accepted_points.reserve(sample_count);
  scratch.candidate_samples.reserve(sample_count);
  std::size_t candidate_worker_count = 1U;
  CandidateBatchWorkspace candidate_workspace{};

  std::vector<double> offsets;
  offsets.reserve(sample_count);
  std::vector<Point2> best_points;
  best_points.reserve(sample_count);
  double best_cost = std::numeric_limits<double>::infinity();
  CandidateScore best_score{};
  double best_length_m = 0.0;
  constexpr std::array kInitialSeeds{
      InitialOffsetSeed::kCenterline, InitialOffsetSeed::kCorridorMidline,
      InitialOffsetSeed::kLeftBiased, InitialOffsetSeed::kRightBiased};
  std::size_t seed_order = 0U;
  for (const InitialOffsetSeed seed : kInitialSeeds) {
    offsetsFromSeed(optimizer_samples, seed, scratch.candidate_offsets);
    const auto points_started_at = std::chrono::steady_clock::now();
    pointsFromOffsets(optimizer_samples, scratch.candidate_offsets,
                      scratch.candidate_points);
    result.stats.candidate_point_build_duration_ms +=
        elapsedMilliseconds(points_started_at);
    TrajectoryOptimizerCandidateDiagnostic diagnostic{};
    diagnostic.phase = initialOffsetSeedName(seed);
    diagnostic.order = seed_order++;
    diagnostic.local_full_score_reason = "none";
    (void)updateBestCandidate(
        optimizer_samples, scratch.candidate_offsets, scratch.candidate_points,
        prohibited_grid, config, best_cost, offsets, best_points, best_score,
        best_length_m, scratch.candidate_samples, scratch.full_path_segment_cache,
        result.stats, &diagnostic);
    result.stats.candidate_diagnostics.push_back(std::move(diagnostic));
  }
  if (offsets.empty()) {
    return result;
  }
  result.stats.initial_cost = best_cost;

  const auto window_eval_started_at = std::chrono::steady_clock::now();
  for (const ActiveWindow& window : active_windows) {
    const std::size_t states_before = result.stats.dp_states;
    const std::size_t transitions_before = result.stats.dp_transitions;
    const double coarse_step =
        sanitizedPositive(config.dp_coarse_offset_step_m, 2.0, 0.05, 100.0);
    const double fine_step =
        sanitizedPositive(config.dp_fine_offset_step_m, 0.75, 0.05, 100.0);
    const double fine_radius =
        sanitizedPositive(config.dp_fine_radius_m, 1.5, 0.05, 5000.0);
    const bool coarse_ok = buildDpSeedForWindow(
        optimizer_samples, window, prohibited_grid, config, coarse_step, offsets, {},
        std::numeric_limits<double>::infinity(), scratch.accepted_offsets,
        result.stats);
    result.stats.dp_coarse_states += result.stats.dp_states - states_before;
    result.stats.dp_coarse_transitions +=
        result.stats.dp_transitions - transitions_before;
    bool dp_ok = false;
    if (coarse_ok) {
      const std::size_t fine_states_before = result.stats.dp_states;
      const std::size_t fine_transitions_before = result.stats.dp_transitions;
      dp_ok =
          buildDpSeedForWindow(optimizer_samples, window, prohibited_grid, config,
                               fine_step, offsets, scratch.accepted_offsets,
                               fine_radius, scratch.candidate_offsets, result.stats);
      result.stats.dp_fine_states += result.stats.dp_states - fine_states_before;
      result.stats.dp_fine_transitions +=
          result.stats.dp_transitions - fine_transitions_before;
      result.stats.dp_coarse_to_fine_used =
          result.stats.dp_coarse_to_fine_used || dp_ok;
    }
    if (!dp_ok && !buildDpSeedForWindow(optimizer_samples, window, prohibited_grid,
                                        config, config.dp_offset_step_m, offsets, {},
                                        std::numeric_limits<double>::infinity(),
                                        scratch.candidate_offsets, result.stats)) {
      continue;
    }
    const bool used_fallback_dp = !dp_ok;
    const auto points_started_at = std::chrono::steady_clock::now();
    pointsFromOffsets(optimizer_samples, scratch.candidate_offsets,
                      scratch.candidate_points);
    result.stats.candidate_point_build_duration_ms +=
        elapsedMilliseconds(points_started_at);
    TrajectoryOptimizerCandidateDiagnostic diagnostic{};
    diagnostic.phase = used_fallback_dp ? "dp_seed_fallback" : "dp_seed_coarse_to_fine";
    diagnostic.order = result.stats.candidate_diagnostics.size();
    diagnostic.center_index =
        window.begin_index + (window.end_index - window.begin_index) / 2U;
    if (diagnostic.center_index < optimizer_samples.size()) {
      diagnostic.center_s_m = optimizer_samples[diagnostic.center_index].s_m;
    }
    diagnostic.local_full_score_reason = "none";
    const bool accepted = updateBestCandidate(
        optimizer_samples, scratch.candidate_offsets, scratch.candidate_points,
        prohibited_grid, config, best_cost, offsets, best_points, best_score,
        best_length_m, scratch.candidate_samples, scratch.full_path_segment_cache,
        result.stats, &diagnostic);
    result.stats.candidate_diagnostics.push_back(std::move(diagnostic));
    if (accepted) {
      result.stats.initial_cost = best_cost;
    }
  }
  result.stats.window_eval_duration_ms += elapsedMilliseconds(window_eval_started_at);

  const std::size_t max_candidate_worker_count =
      desiredWorkerCount(config.parallel_workers, control_indices.size() * 2U);
  std::optional<TrajectoryOptimizerCandidateWorkerPool> candidate_worker_pool;
  if (max_candidate_worker_count > 1U) {
    candidate_worker_pool.emplace(max_candidate_worker_count, result.stats);
  }
  std::vector<double> shadow_segment_score_abs_errors;
  shadow_segment_score_abs_errors.reserve(control_indices.size() * max_iterations);

  for (std::size_t iteration = 0U; iteration < max_iterations && step >= min_step;
       ++iteration) {
    if (control_indices.empty()) {
      break;
    }
    const double incumbent_before_iteration = best_cost;
    const std::vector<CandidateTask> tasks =
        candidateTasksForStep(control_indices, step);
    candidate_worker_count = desiredWorkerCount(config.parallel_workers, tasks.size());
    result.stats.parallel_candidate_evaluation_used =
        result.stats.parallel_candidate_evaluation_used || candidate_worker_count > 1U;
    result.stats.parallel_workers_used =
        std::max(result.stats.parallel_workers_used, candidate_worker_count);
    ++result.stats.candidate_chunks;
    const std::span<const CandidateBatchResult> candidates = evaluateCandidateBatch(
        tasks, optimizer_samples, offsets, best_points, best_score, best_length_m,
        prohibited_grid, config, mutable_indices, best_cost, candidate_workspace,
        candidate_worker_pool.has_value() ? &candidate_worker_pool.value() : nullptr,
        result.stats, candidate_worker_count);

    bool changed = false;
    const EvaluatedCandidate* iteration_winner = nullptr;
    std::optional<std::size_t> iteration_winner_order;
    std::optional<std::size_t> shadow_segment_score_winner_order;
    double best_estimated_score = best_cost;
    double best_shadow_segment_score = best_cost;
    for (const CandidateBatchResult& batch_result : candidates) {
      const EvaluatedCandidate& candidate = batch_result.candidate;
      mergeCandidateStats(candidate, result.stats);
      if (candidate.noop || candidate.offsets.empty()) {
        continue;
      }
      if (candidate.shadow_segment_score_valid) {
        const double score_delta =
            candidate.shadow_segment_score_estimated_score - candidate.score.score;
        if (std::isfinite(score_delta)) {
          shadow_segment_score_abs_errors.push_back(std::abs(score_delta));
        }
      }
      if (candidate.shadow_segment_score_valid &&
          candidate.shadow_segment_score_estimated_score + 1.0e-9 <
              best_shadow_segment_score) {
        best_shadow_segment_score = candidate.shadow_segment_score_estimated_score;
        shadow_segment_score_winner_order = batch_result.order;
      }
      if (!candidate.full_score_used) {
        continue;
      }
      if (candidate.score.score + 1.0e-9 < best_cost) {
        if (candidate.score.score + 1.0e-9 < best_estimated_score) {
          best_estimated_score = candidate.score.score;
          iteration_winner = &candidate;
          iteration_winner_order = batch_result.order;
        }
      }
    }
    if (shadow_segment_score_winner_order.has_value() &&
        shadow_segment_score_winner_order != iteration_winner_order) {
      ++result.stats.shadow_segment_score_winner_mismatches;
    }
    for (const CandidateBatchResult& batch_result : candidates) {
      result.stats.candidate_diagnostics.push_back(candidateDiagnosticFromBatchResult(
          batch_result, optimizer_samples, iteration, step, incumbent_before_iteration,
          iteration_winner_order.has_value() &&
              batch_result.order == *iteration_winner_order));
    }
    if (iteration_winner != nullptr) {
      best_cost = iteration_winner->score.score;
      offsets = iteration_winner->offsets;
      const auto accepted_points_started_at = std::chrono::steady_clock::now();
      pointsFromOffsets(optimizer_samples, offsets, scratch.accepted_points);
      result.stats.candidate_point_build_duration_ms +=
          elapsedMilliseconds(accepted_points_started_at);
      best_points = scratch.accepted_points;
      best_score = iteration_winner->score;
      best_length_m = iteration_winner->path.length_m;
      ++result.stats.local_candidate_acceptance_full_scores;
      result.stats.best_candidate_score = iteration_winner->score.score;
      changed = true;
    }
    ++result.stats.iterations;
    if (!changed) {
      step *= cooling;
    }
  }
  result.stats.shadow_segment_score_abs_error_p95 =
      percentileValue(shadow_segment_score_abs_errors, 0.95);
  std::vector<Point2> final_points = std::move(best_points);
  if (final_points.empty()) {
    final_points = pointsFromOffsets(optimizer_samples, offsets);
  }
  std::vector<TrajectoryPointSample> pre_regularization_samples =
      samplesFromPointsAndOffsets(optimizer_samples, final_points, offsets);
  populateSampleGeometry(pre_regularization_samples);
  const TrajectoryShapeDiagnostics pre_diagnostics =
      computeTrajectoryShapeDiagnostics(pre_regularization_samples);
  result.stats.pre_regularization_max_curvature_jump_1pm =
      pre_diagnostics.max_curvature_jump_1pm;

  std::vector<double> final_offsets = offsets;
  const std::size_t regularization_iterations = std::clamp<std::size_t>(
      config.regularization_iterations, 0U, static_cast<std::size_t>(100U));
  const auto regularization_started_at = std::chrono::steady_clock::now();
  for (std::size_t iteration = 0U; iteration < regularization_iterations; ++iteration) {
    smoothedOffsets(final_offsets, optimizer_samples, scratch.smoothed_offsets);
    const auto points_started_at = std::chrono::steady_clock::now();
    pointsFromOffsets(optimizer_samples, scratch.smoothed_offsets,
                      scratch.candidate_points);
    result.stats.candidate_point_build_duration_ms +=
        elapsedMilliseconds(points_started_at);
    std::size_t cache_hits = 0U;
    std::size_t cache_misses = 0U;
    const PathEvaluation candidate_evaluation =
        evaluatePathCached(prohibited_grid, scratch.candidate_points,
                           scratch.full_path_segment_cache, cache_hits, cache_misses);
    result.stats.full_path_segment_cache_hits += cache_hits;
    result.stats.full_path_segment_cache_misses += cache_misses;
    if (!candidate_evaluation.traversable()) {
      ++result.stats.collision_rejections;
      break;
    }
    const auto sample_started_at = std::chrono::steady_clock::now();
    samplesFromPointsAndOffsets(optimizer_samples, scratch.candidate_points,
                                scratch.smoothed_offsets, scratch.candidate_samples);
    populateSampleGeometry(scratch.candidate_samples);
    result.stats.candidate_sample_build_duration_ms +=
        elapsedMilliseconds(sample_started_at);
    const TrajectoryShapeDiagnostics candidate_diagnostics =
        computeTrajectoryShapeDiagnostics(scratch.candidate_samples);
    if (candidate_diagnostics.max_curvature_jump_1pm + 1.0e-9 >=
        pre_diagnostics.max_curvature_jump_1pm) {
      break;
    }
    final_offsets = scratch.smoothed_offsets;
    final_points = scratch.candidate_points;
    result.stats.regularization_applied = true;
    ++result.stats.regularization_iterations;
  }
  result.stats.regularization_duration_ms =
      elapsedMilliseconds(regularization_started_at);
  std::size_t final_cache_hits = 0U;
  std::size_t final_cache_misses = 0U;
  const PathEvaluation final_evaluation =
      evaluatePathCached(prohibited_grid, final_points, scratch.full_path_segment_cache,
                         final_cache_hits, final_cache_misses);
  result.stats.full_path_segment_cache_hits += final_cache_hits;
  result.stats.full_path_segment_cache_misses += final_cache_misses;
  if (!final_evaluation.traversable()) {
    ++result.stats.collision_rejections;
    return result;
  }
  const auto final_score_started_at = std::chrono::steady_clock::now();
  const CandidateScore final_score = scoreForCandidate(
      optimizer_samples, final_points, final_offsets, final_evaluation, config,
      scratch.candidate_samples, result.stats, nullptr);
  result.stats.full_final_score_duration_ms =
      elapsedMilliseconds(final_score_started_at);

  result.samples.reserve(sample_count);
  for (std::size_t i = 0U; i < sample_count; ++i) {
    TrajectoryPointSample sample{};
    sample.point = final_points[i];
    sample.left_bound_m = optimizer_samples[i].left_bound_m;
    sample.right_bound_m = optimizer_samples[i].right_bound_m;
    sample.lateral_offset_m = final_offsets[i];
    result.stats.max_abs_offset_m =
        std::max(result.stats.max_abs_offset_m, std::abs(final_offsets[i]));
    result.samples.push_back(sample);
  }
  populateSampleGeometry(result.samples);
  result.stats.output_samples = result.samples.size();
  result.stats.final_length_m = pathLength(final_points);
  if (result.stats.centerline_length_m > kTinyDistanceM) {
    result.stats.final_length_ratio =
        result.stats.final_length_m / result.stats.centerline_length_m;
  }
  result.stats.final_cost = final_score.score;
  copyCostBreakdownToStats(final_score.breakdown, result.stats);
  const TrajectoryShapeDiagnostics post_diagnostics =
      computeTrajectoryShapeDiagnostics(result.samples);
  result.stats.post_regularization_max_curvature_jump_1pm =
      post_diagnostics.max_curvature_jump_1pm;
  const TraversalTimeEstimate final_time =
      estimateTraversalTime(result.samples, speed_config, true);
  copyTraversalEstimateToFinalStats(final_time, result.stats);
  updateCurvatureStats(result.samples, result.stats);
  updateEdgeMarginStats(result.samples, result.stats);
  result.valid = trajectorySamplesAreUsable(result.samples);
  return result;
}

} // namespace drone_city_nav
