#include "turn_smoothing_internal.hpp"

namespace drone_city_nav::turn_smoothing_detail {

namespace {

constexpr double kCurvatureJumpRegressionTolerance = 0.035;
constexpr double kCurvatureJumpRegressionFactor = 1.4;

[[nodiscard]] double
maxAllowedCurvatureJumpAfter(const double before_curvature_jump_1pm) noexcept {
  return std::max(before_curvature_jump_1pm + kCurvatureJumpRegressionTolerance,
                  before_curvature_jump_1pm * kCurvatureJumpRegressionFactor);
}

} // namespace

[[nodiscard]] bool segmentIsTraversable(const OccupancyGrid2D& grid, const Point2 start,
                                        const Point2 end) {
  const std::optional<GridIndex> start_cell = grid.worldToCell(start);
  const std::optional<GridIndex> end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    return false;
  }
  const std::vector<GridIndex> cells = grid.cellsOnLine(*start_cell, *end_cell);
  return std::ranges::none_of(
      cells, [&grid](const GridIndex cell) { return grid.isProhibited(cell); });
}

[[nodiscard]] SegmentCellKey orderedSegmentKey(GridIndex start,
                                               GridIndex end) noexcept {
  if (end.x < start.x || (end.x == start.x && end.y < start.y)) {
    std::swap(start, end);
  }
  return SegmentCellKey{.start = start, .end = end};
}

[[nodiscard]] bool cachedSegmentIsTraversable(const OccupancyGrid2D& grid,
                                              const Point2 start, const Point2 end,
                                              SegmentTraversabilityCache& cache,
                                              TurnSmoothingStats& stats) {
  const std::optional<GridIndex> start_cell = grid.worldToCell(start);
  const std::optional<GridIndex> end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    ++stats.traversability_cache_misses;
    return false;
  }
  const SegmentCellKey key = orderedSegmentKey(*start_cell, *end_cell);
  if (const auto iter = cache.values.find(key); iter != cache.values.end()) {
    ++stats.traversability_cache_hits;
    return iter->second;
  }
  ++stats.traversability_cache_misses;
  const bool traversable = segmentIsTraversable(grid, start, end);
  cache.values.emplace(key, traversable);
  return traversable;
}

[[nodiscard]] bool
pathIsTraversable(const OccupancyGrid2D& grid,
                  const std::span<const TrajectoryPointSample> samples) {
  if (samples.size() < 2U) {
    return false;
  }
  for (std::size_t i = 1U; i < samples.size(); ++i) {
    if (!segmentIsTraversable(grid, samples[i - 1U].point, samples[i].point)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool
pathIsTraversableCached(const OccupancyGrid2D& grid,
                        const std::span<const TrajectoryPointSample> samples,
                        SegmentTraversabilityCache& cache, TurnSmoothingStats& stats) {
  if (samples.size() < 2U) {
    return false;
  }
  for (std::size_t i = 1U; i < samples.size(); ++i) {
    if (!cachedSegmentIsTraversable(grid, samples[i - 1U].point, samples[i].point,
                                    cache, stats)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] double innerMarginM(const TrajectoryPointSample& sample,
                                  const double turn_sign) noexcept {
  if (turn_sign > 0.0) {
    return sample.left_bound_m - sample.lateral_offset_m;
  }
  return sample.right_bound_m + sample.lateral_offset_m;
}

void updateMinInnerMargin(TurnSmoothingStats& stats,
                          const TrajectoryPointSample& sample,
                          const double turn_sign) noexcept {
  const double margin = innerMarginM(sample, turn_sign);
  if (!std::isfinite(margin)) {
    return;
  }
  if (!std::isfinite(stats.min_inner_margin_m)) {
    stats.min_inner_margin_m = margin;
    return;
  }
  stats.min_inner_margin_m = std::min(stats.min_inner_margin_m, margin);
}

void sampleRangeInto(const std::span<const TrajectoryPointSample> samples,
                     const std::size_t start_index, const std::size_t end_index,
                     std::vector<TrajectoryPointSample>& result) {
  result.clear();
  if (samples.empty() || start_index >= samples.size() || end_index >= samples.size() ||
      start_index >= end_index) {
    return;
  }
  result.reserve(end_index - start_index + 1U);
  result.insert(result.end(),
                samples.begin() + static_cast<std::ptrdiff_t>(start_index),
                samples.begin() + static_cast<std::ptrdiff_t>(end_index + 1U));
  populateSampleGeometry(result);
}

void replaceRangeInto(const std::span<const TrajectoryPointSample> samples,
                      const std::size_t entry_index, const std::size_t exit_index,
                      const std::span<const TrajectoryPointSample> replacement,
                      std::vector<TrajectoryPointSample>& result) {
  result.clear();
  result.reserve(samples.size() + replacement.size());
  result.insert(result.end(), samples.begin(),
                samples.begin() + static_cast<std::ptrdiff_t>(entry_index));
  result.insert(result.end(), replacement.begin(), replacement.end());
  result.insert(result.end(),
                samples.begin() + static_cast<std::ptrdiff_t>(exit_index + 1U),
                samples.end());
  populateSampleGeometry(result);
}

[[nodiscard]] LocalTrajectoryMetrics
localTrajectoryMetrics(const std::span<const TrajectoryPointSample> samples,
                       const VelocityFollowerConfig& speed_config,
                       TurnSmoothingStats* stats, const bool include_speed_profile) {
  const auto metrics_started_at = std::chrono::steady_clock::now();
  LocalTrajectoryMetrics metrics{};
  if (samples.size() < 2U) {
    if (stats != nullptr) {
      stats->metrics_duration_ms += elapsedMilliseconds(metrics_started_at);
    }
    return metrics;
  }

  metrics.valid = trajectorySamplesAreUsable(samples);
  metrics.length_m = pathLength(samples);
  const auto shape_started_at = std::chrono::steady_clock::now();
  metrics.shape = computeTrajectoryShapeDiagnostics(samples);
  if (stats != nullptr) {
    stats->shape_diagnostics_duration_ms += elapsedMilliseconds(shape_started_at);
  }
  for (const TrajectoryPointSample& sample : samples) {
    const double abs_curvature = std::abs(sample.curvature_1pm);
    metrics.max_abs_curvature_1pm =
        std::max(metrics.max_abs_curvature_1pm, abs_curvature);
    if (abs_curvature > kTinyDistanceM) {
      metrics.min_radius_m = std::min(metrics.min_radius_m, 1.0 / abs_curvature);
    }
  }
  if (!std::isfinite(metrics.min_radius_m)) {
    metrics.min_radius_m = std::numeric_limits<double>::infinity();
  }

  if (include_speed_profile) {
    const auto speed_started_at = std::chrono::steady_clock::now();
    const TraversalTimeEstimate time =
        estimateTraversalTime(samples, speed_config, false);
    if (stats != nullptr) {
      stats->speed_profile_duration_ms += elapsedMilliseconds(speed_started_at);
    }
    metrics.min_speed_limit_mps = time.min_speed_limit_mps;
    metrics.estimated_time_s = time.estimated_time_s;
  }
  if (stats != nullptr) {
    stats->metrics_duration_ms += elapsedMilliseconds(metrics_started_at);
  }
  return metrics;
}

[[nodiscard]] LocalTrajectoryMetrics
cachedBeforeMetrics(const std::span<const TrajectoryPointSample> samples,
                    const std::size_t entry_index, const std::size_t exit_index,
                    const VelocityFollowerConfig& speed_config,
                    TurnSmoothingWorkBuffer& buffer, TurnSmoothingStats& stats) {
  const IndexRangeKey key{.begin_index = entry_index, .end_index = exit_index};
  if (const auto iter = buffer.before_metrics_cache.find(key);
      iter != buffer.before_metrics_cache.end()) {
    ++stats.before_metrics_cache_hits;
    return iter->second;
  }
  ++stats.before_metrics_cache_misses;
  sampleRangeInto(samples, entry_index, exit_index, buffer.before_local);
  const LocalTrajectoryMetrics metrics =
      localTrajectoryMetrics(buffer.before_local, speed_config, &stats);
  buffer.before_metrics_cache.emplace(key, metrics);
  return metrics;
}

[[nodiscard]] double smoothingAttemptScore(const LocalTrajectoryMetrics& metrics) {
  if (!metrics.valid) {
    return std::numeric_limits<double>::infinity();
  }
  constexpr double kCurvatureWeight = 5.0;
  constexpr double kCurvatureJumpWeight = 10.0;
  return kCurvatureWeight * metrics.max_abs_curvature_1pm +
         kCurvatureJumpWeight * metrics.shape.max_curvature_jump_1pm;
}

[[nodiscard]] std::optional<CornerCandidate>
worstCorner(const std::span<const TrajectoryPointSample> samples,
            const TurnSmoothingConfig& config,
            const VelocityFollowerConfig& speed_config, TurnSmoothingStats& stats) {
  const std::vector<CornerCandidate> candidates =
      cornerCandidatesBySeverity(samples, config, speed_config, stats);
  if (candidates.empty()) {
    return std::nullopt;
  }
  return candidates.front();
}

[[nodiscard]] std::vector<CornerCandidate>
cornerCandidatesBySeverity(const std::span<const TrajectoryPointSample> samples,
                           const TurnSmoothingConfig& config,
                           const VelocityFollowerConfig& speed_config,
                           TurnSmoothingStats& stats) {
  std::vector<CornerCandidate> candidates;
  if (samples.size() < 3U) {
    return candidates;
  }
  candidates.reserve(samples.size());
  for (std::size_t i = 1U; i + 1U < samples.size(); ++i) {
    const CornerCandidate candidate =
        cornerCandidateAt(samples, i, config, speed_config);
    if (!candidate.valid) {
      continue;
    }
    ++stats.detected_corners;
    updateMinInnerMargin(stats, samples[i], candidate.turn_sign);
    candidates.push_back(candidate);
  }
  std::ranges::sort(candidates,
                    [](const CornerCandidate& lhs, const CornerCandidate& rhs) {
                      if (lhs.abs_heading_delta_rad == rhs.abs_heading_delta_rad) {
                        return lhs.index < rhs.index;
                      }
                      return lhs.abs_heading_delta_rad > rhs.abs_heading_delta_rad;
                    });
  return candidates;
}

[[nodiscard]] const char*
shapeImprovementRejectDetail(const TrajectoryShapeDiagnostics& before,
                             const TrajectoryShapeDiagnostics& after,
                             const TurnSmoothingConfig& config) {
  const double min_improvement = sanitizedPositive(config.min_heading_improvement_rad,
                                                   0.05, 0.0, std::numbers::pi);
  constexpr double kMaxAcceptedHeadingDeltaRad = std::numbers::pi / 3.0;
  const double max_allowed_curvature_jump =
      maxAllowedCurvatureJumpAfter(before.max_curvature_jump_1pm);
  if (after.max_heading_delta_rad > kMaxAcceptedHeadingDeltaRad) {
    return "heading_delta_too_high";
  }
  if (after.max_curvature_jump_1pm > max_allowed_curvature_jump) {
    return "curvature_jump_too_high";
  }
  if (after.max_heading_delta_rad + min_improvement < before.max_heading_delta_rad) {
    return "none";
  }
  if (after.max_heading_delta_rad <= before.max_heading_delta_rad + 1.0e-9 &&
      after.max_curvature_jump_1pm + 1.0e-9 < before.max_curvature_jump_1pm) {
    return "none";
  }
  return "shape_not_improved";
}

[[nodiscard]] const char*
globalShapeRegressionRejectDetail(const TrajectoryShapeDiagnostics& before,
                                  const TrajectoryShapeDiagnostics& after) {
  constexpr double kHeadingRegressionToleranceRad = std::numbers::pi / 180.0;
  if (after.max_heading_delta_rad >
      before.max_heading_delta_rad + kHeadingRegressionToleranceRad) {
    return "global_heading_delta_regression";
  }
  const double max_allowed_curvature_jump =
      maxAllowedCurvatureJumpAfter(before.max_curvature_jump_1pm);
  if (after.max_curvature_jump_1pm > max_allowed_curvature_jump) {
    return "global_curvature_jump_regression";
  }
  return "none";
}

[[nodiscard]] SmoothingRejectReason
candidateRegressionReason(const LocalTrajectoryMetrics& before,
                          const LocalTrajectoryMetrics& after) noexcept {
  constexpr double kRadiusToleranceM = 0.25;
  if (!before.valid || !after.valid) {
    return SmoothingRejectReason::kNotImproved;
  }

  const double max_allowed_curvature_jump =
      maxAllowedCurvatureJumpAfter(before.shape.max_curvature_jump_1pm);
  if (after.shape.max_curvature_jump_1pm > max_allowed_curvature_jump) {
    return SmoothingRejectReason::kCurvatureRegression;
  }
  if (std::isfinite(before.min_radius_m) && std::isfinite(after.min_radius_m) &&
      after.min_radius_m + kRadiusToleranceM < before.min_radius_m) {
    return SmoothingRejectReason::kRadiusRegression;
  }
  return SmoothingRejectReason::kNone;
}

[[nodiscard]] const char*
regressionRejectDetail(const SmoothingRejectReason reason) noexcept {
  switch (reason) {
    case SmoothingRejectReason::kCurvatureRegression:
      return "curvature_jump_regression";
    case SmoothingRejectReason::kRadiusRegression:
      return "radius_regression";
    case SmoothingRejectReason::kNotImproved:
      return "local_metrics_invalid";
    case SmoothingRejectReason::kNone:
    case SmoothingRejectReason::kProhibited:
      return "none";
  }
  return "unknown";
}

} // namespace drone_city_nav::turn_smoothing_detail
