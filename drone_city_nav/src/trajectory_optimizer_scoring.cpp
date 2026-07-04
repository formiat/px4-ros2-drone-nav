#include "trajectory_optimizer_internal.hpp"

namespace drone_city_nav::trajectory_optimizer_detail {

[[nodiscard]] LocalCandidateScore
evaluateLocalOffsetPath(const std::span<const CorridorSample> corridor_samples,
                        const std::span<const Point2> base_points,
                        const std::span<const double> base_offsets,
                        const std::span<const double> candidate_offsets,
                        const OccupancyGrid2D& prohibited_grid,
                        const CandidateScore& base_score, const double base_length_m,
                        const std::size_t center_index, CandidateWorkBuffer& buffer) {
  LocalCandidateScore result{};
  result.valid = false;
  constexpr std::size_t kLocalScoreRadiusSamples = 6U;
  if (corridor_samples.size() != base_points.size() ||
      base_points.size() != base_offsets.size() ||
      base_offsets.size() != candidate_offsets.size() || base_points.size() < 3U ||
      center_index == 0U || center_index + 1U >= base_points.size()) {
    result.requires_full_score = true;
    result.full_score_reason = LocalFullScoreReason::kInvalidInput;
    return result;
  }

  const auto [begin_index, end_index] = localScoreWindowForCenter(
      center_index, base_points.size(), kLocalScoreRadiusSamples);
  if (begin_index == 0U || end_index + 1U >= base_points.size()) {
    result.shadow_boundary_clamped_window_samples =
        end_index >= begin_index ? end_index - begin_index + 1U : 0U;
    result.requires_full_score = true;
    result.full_score_reason = LocalFullScoreReason::kBoundaryWindow;
    return result;
  }
  if (base_score.breakdown.collision_cost > 0.0 ||
      base_score.breakdown.outside_grid_cost > 0.0) {
    result.requires_full_score = true;
    result.full_score_reason = LocalFullScoreReason::kUnsafeBase;
    return result;
  }

  const auto points_started_at = std::chrono::steady_clock::now();
  copyRange(base_points, begin_index, end_index, buffer.local_base_points);
  copyRange(corridor_samples, begin_index, end_index, buffer.local_corridor_samples);
  copyRange(base_offsets, begin_index, end_index, buffer.local_base_offsets);
  copyRange(candidate_offsets, begin_index, end_index, buffer.local_candidate_offsets);
  pointsFromOffsetsRange(corridor_samples, candidate_offsets, begin_index, end_index,
                         buffer.local_candidate_points);
  result.point_build_duration_ms = elapsedMilliseconds(points_started_at);
  if (buffer.local_base_points.size() != buffer.local_candidate_points.size() ||
      buffer.local_base_points.size() < 2U) {
    result.requires_full_score = true;
    result.full_score_reason = LocalFullScoreReason::kWindowInvalid;
    return result;
  }

  const double base_local_length_m = pathLength(buffer.local_base_points);
  const auto evaluation_started_at = std::chrono::steady_clock::now();
  result.path = evaluateLocalPathWindowCached(
      prohibited_grid, buffer.local_candidate_points, buffer.candidate_segment_cache,
      result.segment_cache_hits, result.segment_cache_misses);
  result.path_evaluation_duration_ms = elapsedMilliseconds(evaluation_started_at);

  const double candidate_length_m =
      base_length_m - base_local_length_m + result.path.length_m;
  result.path.length_m = candidate_length_m;
  result.valid = true;
  return result;
}

[[nodiscard]] CostBreakdown
costBreakdownForPoints(const std::span<const Point2> points,
                       const std::span<const double> offsets,
                       const TrajectoryOptimizerConfig& config) {
  CostBreakdown breakdown{};
  if (points.size() < 2U) {
    breakdown.outside_grid_cost = kOutsideGridPenalty;
    return breakdown;
  }

  const double weight_curvature =
      sanitizedPositive(config.weight_curvature, 300.0, 0.0, 1.0e9);
  const double weight_curvature_change =
      sanitizedPositive(config.weight_curvature_change, 180.0, 0.0, 1.0e9);
  const double preferred_min_radius =
      sanitizedPositive(config.preferred_min_radius_m, 24.0, 0.0, 100000.0);
  const double weight_radius_shortfall =
      sanitizedPositive(config.weight_radius_shortfall, 40.0, 0.0, 1.0e9);
  const double weight_offset_change =
      sanitizedPositive(config.weight_offset_change, 0.5, 0.0, 1.0e9);
  const double weight_offset_second_change =
      sanitizedPositive(config.weight_offset_second_change, 6.5, 0.0, 1.0e9);
  const double weight_offset_slope =
      sanitizedPositive(config.weight_offset_slope, 100.0, 0.0, 1.0e9);
  const double max_offset_slope =
      sanitizedPositive(config.max_offset_slope_per_m, 0.32, 0.0, 100.0);

  double curvature_cost = 0.0;
  double curvature_change_cost = 0.0;
  double radius_shortfall_cost = 0.0;
  double offset_change_cost = 0.0;
  double offset_second_change_cost = 0.0;
  double offset_slope_cost = 0.0;
  double previous_curvature = 0.0;
  bool previous_curvature_valid = false;
  for (std::size_t i = 1U; i < offsets.size(); ++i) {
    const double change = offsets[i] - offsets[i - 1U];
    offset_change_cost += change * change;
    const double ds = i < points.size() ? distance(points[i - 1U], points[i]) : 0.0;
    if (ds > kTinyDistanceM) {
      const double slope_violation =
          std::max(0.0, std::abs(change) / ds - max_offset_slope);
      offset_slope_cost += slope_violation * slope_violation * ds;
    }
  }
  for (std::size_t i = 1U; i + 1U < offsets.size(); ++i) {
    const double second_change = offsets[i + 1U] - 2.0 * offsets[i] + offsets[i - 1U];
    offset_second_change_cost += second_change * second_change;
  }
  for (std::size_t i = 1U; i + 1U < points.size(); ++i) {
    const double curvature =
        discreteCurvature(points[i - 1U], points[i], points[i + 1U]);
    curvature_cost += curvature * curvature;
    if (preferred_min_radius > kTinyDistanceM) {
      const double target_curvature = 1.0 / preferred_min_radius;
      const double shortfall_ratio =
          std::max(0.0, std::abs(curvature) - target_curvature) / target_curvature;
      radius_shortfall_cost += shortfall_ratio * shortfall_ratio;
    }
    if (previous_curvature_valid) {
      const double change = curvature - previous_curvature;
      curvature_change_cost += change * change;
    }
    previous_curvature = curvature;
    previous_curvature_valid = true;
  }

  breakdown.curvature_cost = weight_curvature * curvature_cost;
  breakdown.curvature_change_cost = weight_curvature_change * curvature_change_cost;
  breakdown.radius_shortfall_cost = weight_radius_shortfall * radius_shortfall_cost;
  breakdown.offset_change_cost = weight_offset_change * offset_change_cost;
  breakdown.offset_second_change_cost =
      weight_offset_second_change * offset_second_change_cost;
  breakdown.offset_slope_cost = weight_offset_slope * offset_slope_cost;
  return breakdown;
}

[[nodiscard]] double geometrySubtotal(const CostBreakdown& breakdown) noexcept {
  return breakdown.curvature_cost + breakdown.curvature_change_cost +
         breakdown.radius_shortfall_cost + breakdown.offset_change_cost +
         breakdown.offset_second_change_cost + breakdown.offset_slope_cost;
}

[[nodiscard]] bool geometryCostsAreFinite(const CostBreakdown& breakdown) noexcept {
  return std::isfinite(breakdown.curvature_cost) &&
         std::isfinite(breakdown.curvature_change_cost) &&
         std::isfinite(breakdown.radius_shortfall_cost) &&
         std::isfinite(breakdown.offset_change_cost) &&
         std::isfinite(breakdown.offset_second_change_cost) &&
         std::isfinite(breakdown.offset_slope_cost);
}

[[nodiscard]] std::optional<IndexRange>
boundedIndexRange(std::size_t begin, std::size_t end, const std::size_t max_index) {
  if (end > max_index) {
    end = max_index;
  }
  if (begin > end) {
    return std::nullopt;
  }
  return IndexRange{.begin = begin, .end = end};
}

[[nodiscard]] double curvatureAtIndex(const std::span<const Point2> points,
                                      const std::size_t index) {
  if (index == 0U || index + 1U >= points.size()) {
    return 0.0;
  }
  return discreteCurvature(points[index - 1U], points[index], points[index + 1U]);
}

[[nodiscard]] CostBreakdown localGeometryCostForChangedSpan(
    const std::span<const Point2> points, const std::span<const double> offsets,
    const OffsetChangeDiagnostics& changed, const TrajectoryOptimizerConfig& config) {
  CostBreakdown breakdown{};
  if (points.size() < 2U || points.size() != offsets.size() ||
      changed.changed_samples == 0U || changed.last_changed_index >= points.size()) {
    breakdown.outside_grid_cost = kOutsideGridPenalty;
    return breakdown;
  }

  const double weight_curvature =
      sanitizedPositive(config.weight_curvature, 300.0, 0.0, 1.0e9);
  const double weight_curvature_change =
      sanitizedPositive(config.weight_curvature_change, 180.0, 0.0, 1.0e9);
  const double preferred_min_radius =
      sanitizedPositive(config.preferred_min_radius_m, 24.0, 0.0, 100000.0);
  const double weight_radius_shortfall =
      sanitizedPositive(config.weight_radius_shortfall, 40.0, 0.0, 1.0e9);
  const double weight_offset_change =
      sanitizedPositive(config.weight_offset_change, 0.5, 0.0, 1.0e9);
  const double weight_offset_second_change =
      sanitizedPositive(config.weight_offset_second_change, 6.5, 0.0, 1.0e9);
  const double weight_offset_slope =
      sanitizedPositive(config.weight_offset_slope, 100.0, 0.0, 1.0e9);
  const double max_offset_slope =
      sanitizedPositive(config.max_offset_slope_per_m, 0.32, 0.0, 100.0);

  double curvature_cost = 0.0;
  double curvature_change_cost = 0.0;
  double radius_shortfall_cost = 0.0;
  double offset_change_cost = 0.0;
  double offset_second_change_cost = 0.0;
  double offset_slope_cost = 0.0;

  const std::size_t first = changed.first_changed_index;
  const std::size_t last = changed.last_changed_index;
  const std::optional<IndexRange> segment_range = boundedIndexRange(
      std::max<std::size_t>(1U, first), last + 1U, points.size() - 1U);
  if (segment_range.has_value()) {
    for (std::size_t i = segment_range->begin; i <= segment_range->end; ++i) {
      const double ds = distance(points[i - 1U], points[i]);
      const double change = offsets[i] - offsets[i - 1U];
      offset_change_cost += change * change;
      if (ds > kTinyDistanceM) {
        const double slope_violation =
            std::max(0.0, std::abs(change) / ds - max_offset_slope);
        offset_slope_cost += slope_violation * slope_violation * ds;
      }
    }
  }

  const std::optional<IndexRange> triple_range =
      points.size() >= 3U
          ? boundedIndexRange(std::max<std::size_t>(1U, first > 0U ? first - 1U : 1U),
                              last + 1U, points.size() - 2U)
          : std::nullopt;
  if (triple_range.has_value()) {
    for (std::size_t i = triple_range->begin; i <= triple_range->end; ++i) {
      const double second_change = offsets[i + 1U] - 2.0 * offsets[i] + offsets[i - 1U];
      offset_second_change_cost += second_change * second_change;
      const double curvature = curvatureAtIndex(points, i);
      curvature_cost += curvature * curvature;
      if (preferred_min_radius > kTinyDistanceM) {
        const double target_curvature = 1.0 / preferred_min_radius;
        const double shortfall_ratio =
            std::max(0.0, std::abs(curvature) - target_curvature) / target_curvature;
        radius_shortfall_cost += shortfall_ratio * shortfall_ratio;
      }
    }

    const std::optional<IndexRange> curvature_change_range =
        points.size() >= 4U
            ? boundedIndexRange(std::max<std::size_t>(2U, triple_range->begin),
                                triple_range->end + 1U, points.size() - 2U)
            : std::nullopt;
    if (curvature_change_range.has_value()) {
      for (std::size_t i = curvature_change_range->begin;
           i <= curvature_change_range->end; ++i) {
        const double change =
            curvatureAtIndex(points, i) - curvatureAtIndex(points, i - 1U);
        curvature_change_cost += change * change;
      }
    }
  }

  breakdown.curvature_cost = weight_curvature * curvature_cost;
  breakdown.curvature_change_cost = weight_curvature_change * curvature_change_cost;
  breakdown.radius_shortfall_cost = weight_radius_shortfall * radius_shortfall_cost;
  breakdown.offset_change_cost = weight_offset_change * offset_change_cost;
  breakdown.offset_second_change_cost =
      weight_offset_second_change * offset_second_change_cost;
  breakdown.offset_slope_cost = weight_offset_slope * offset_slope_cost;
  return breakdown;
}

[[nodiscard]] std::size_t
shadowSegmentWindowSamples(const OffsetChangeDiagnostics& changed,
                           const std::size_t sample_count) {
  if (sample_count == 0U || changed.changed_samples == 0U ||
      changed.last_changed_index >= sample_count) {
    return 0U;
  }
  const std::size_t begin =
      changed.first_changed_index > 2U ? changed.first_changed_index - 2U : 0U;
  const std::size_t end = std::min(sample_count - 1U, changed.last_changed_index + 2U);
  return end >= begin ? end - begin + 1U : 0U;
}

[[nodiscard]] std::optional<CostBreakdown> incrementalGeometryBreakdownForChangedSpan(
    const CandidateScore& base_score, const std::span<const Point2> base_points,
    const std::span<const double> base_offsets,
    const std::span<const Point2> candidate_points,
    const std::span<const double> candidate_offsets,
    const OffsetChangeDiagnostics& changed, const TrajectoryOptimizerConfig& config) {
  if (changed.changed_samples == 0U || base_points.size() != candidate_points.size() ||
      base_offsets.size() != candidate_offsets.size() ||
      base_points.size() != base_offsets.size() || !std::isfinite(base_score.score)) {
    return std::nullopt;
  }

  const CostBreakdown base_local =
      localGeometryCostForChangedSpan(base_points, base_offsets, changed, config);
  const CostBreakdown candidate_local = localGeometryCostForChangedSpan(
      candidate_points, candidate_offsets, changed, config);
  if (base_local.outside_grid_cost > 0.0 || candidate_local.outside_grid_cost > 0.0 ||
      !geometryCostsAreFinite(base_score.breakdown) ||
      !geometryCostsAreFinite(base_local) || !geometryCostsAreFinite(candidate_local)) {
    return std::nullopt;
  }

  CostBreakdown breakdown{};
  breakdown.curvature_cost = base_score.breakdown.curvature_cost -
                             base_local.curvature_cost + candidate_local.curvature_cost;
  breakdown.curvature_change_cost = base_score.breakdown.curvature_change_cost -
                                    base_local.curvature_change_cost +
                                    candidate_local.curvature_change_cost;
  breakdown.radius_shortfall_cost = base_score.breakdown.radius_shortfall_cost -
                                    base_local.radius_shortfall_cost +
                                    candidate_local.radius_shortfall_cost;
  breakdown.offset_change_cost = base_score.breakdown.offset_change_cost -
                                 base_local.offset_change_cost +
                                 candidate_local.offset_change_cost;
  breakdown.offset_second_change_cost = base_score.breakdown.offset_second_change_cost -
                                        base_local.offset_second_change_cost +
                                        candidate_local.offset_second_change_cost;
  breakdown.offset_slope_cost = base_score.breakdown.offset_slope_cost -
                                base_local.offset_slope_cost +
                                candidate_local.offset_slope_cost;
  if (!geometryCostsAreFinite(breakdown)) {
    return std::nullopt;
  }
  return breakdown;
}

void populateShadowSegmentScoreDiagnostics(
    EvaluatedCandidate& result, const CandidateScore& base_score,
    const std::span<const Point2> base_points,
    const std::span<const double> base_offsets,
    const std::span<const Point2> candidate_points,
    const std::span<const double> candidate_offsets,
    const OffsetChangeDiagnostics& changed, const TrajectoryOptimizerConfig& config,
    const double incumbent_score,
    const std::optional<CostBreakdown>& incremental_geometry_breakdown) {
  if (changed.changed_samples == 0U || base_points.size() != candidate_points.size() ||
      base_offsets.size() != candidate_offsets.size() ||
      base_points.size() != base_offsets.size() || !std::isfinite(base_score.score) ||
      !std::isfinite(result.score.score)) {
    return;
  }
  const std::optional<CostBreakdown> fallback_geometry =
      incremental_geometry_breakdown.has_value()
          ? std::nullopt
          : incrementalGeometryBreakdownForChangedSpan(
                base_score, base_points, base_offsets, candidate_points,
                candidate_offsets, changed, config);
  const CostBreakdown* estimated_geometry = nullptr;
  if (incremental_geometry_breakdown.has_value()) {
    estimated_geometry = &*incremental_geometry_breakdown;
  } else if (fallback_geometry.has_value()) {
    estimated_geometry = &*fallback_geometry;
  }
  if (estimated_geometry == nullptr) {
    return;
  }

  const double full_candidate_geometry_score = geometrySubtotal(result.score.breakdown);
  const double estimated_score = result.score.score - full_candidate_geometry_score +
                                 geometrySubtotal(*estimated_geometry);
  if (!std::isfinite(estimated_score)) {
    return;
  }

  result.shadow_segment_score_valid = true;
  result.shadow_segment_score_estimated_score = estimated_score;
  result.shadow_segment_score_incumbent_score = incumbent_score;
  result.shadow_segment_score_window_samples =
      shadowSegmentWindowSamples(changed, base_points.size());
  result.shadow_segment_score_would_prune =
      std::isfinite(incumbent_score) && estimated_score + 1.0e-9 >= incumbent_score;
}

[[nodiscard]] CandidateScore scoreForCandidate(
    const std::span<const CorridorSample> corridor_samples,
    const std::span<const Point2> points, const std::span<const double> offsets,
    const PathEvaluation& evaluation, const TrajectoryOptimizerConfig& config,
    std::vector<TrajectoryPointSample>& scratch_samples,
    TrajectoryOptimizerStats& stats, const CostBreakdown* geometry_breakdown_override) {
  CandidateScore result{};
  const auto cost_started_at = std::chrono::steady_clock::now();
  if (geometry_breakdown_override != nullptr) {
    result.breakdown = *geometry_breakdown_override;
  } else {
    result.breakdown = costBreakdownForPoints(points, offsets, config);
  }
  result.breakdown.collision_cost =
      static_cast<double>(evaluation.prohibited_cells) * kCollisionPenalty;
  result.breakdown.outside_grid_cost =
      static_cast<double>(evaluation.outside_grid_segments) * kOutsideGridPenalty;
  stats.candidate_cost_breakdown_duration_ms += elapsedMilliseconds(cost_started_at);
  if (evaluation.traversable()) {
    const auto sample_started_at = std::chrono::steady_clock::now();
    samplesFromPointsAndOffsets(corridor_samples, points, offsets, scratch_samples);
    populateSampleGeometry(scratch_samples);
    stats.candidate_sample_build_duration_ms += elapsedMilliseconds(sample_started_at);
    const auto shape_started_at = std::chrono::steady_clock::now();
    const TrajectoryShapeDiagnostics shape =
        computeTrajectoryShapeDiagnostics(scratch_samples);
    stats.candidate_shape_diagnostics_duration_ms +=
        elapsedMilliseconds(shape_started_at);
    const double heading_jump_overrun =
        std::max(0.0, shape.max_heading_delta_rad - kHeadingJumpSoftLimitRad);
    result.breakdown.heading_jump_cost =
        heading_jump_overrun * heading_jump_overrun * kHeadingJumpPenalty;
    if (shape.max_heading_delta_rad > kHeadingJumpHardLimitRad) {
      result.breakdown.heading_jump_cost += kHeadingJumpHardPenalty;
    }
  }
  result.score = result.breakdown.total();
  return result;
}

} // namespace drone_city_nav::trajectory_optimizer_detail
