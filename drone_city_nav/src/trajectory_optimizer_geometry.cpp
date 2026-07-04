#include "trajectory_optimizer_internal.hpp"

namespace drone_city_nav::trajectory_optimizer_detail {

[[nodiscard]] Point2 operator+(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] Point2 operator*(const Point2 point, const double scale) noexcept {
  return Point2{point.x * scale, point.y * scale};
}

[[nodiscard]] double norm(const Point2 point) noexcept {
  return std::hypot(point.x, point.y);
}

[[nodiscard]] Point2 normalized(const Point2 point) noexcept {
  const double length = norm(point);
  if (!(length > kTinyDistanceM)) {
    return Point2{};
  }
  return Point2{point.x / length, point.y / length};
}

[[nodiscard]] double cross(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

[[nodiscard]] double dot(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

void startBlockedSpanDiagnostic(PathEvaluation& evaluation,
                                const std::size_t segment_index,
                                const double segment_start_s_m,
                                const Point2 segment_start) {
  if (evaluation.blocked_span_diagnostic_count >=
      kMaxCenterlineBlockedSpanDiagnostics) {
    return;
  }
  TrajectoryOptimizerBlockedSpanDiagnostic& span =
      evaluation.blocked_span_diagnostics.at(evaluation.blocked_span_diagnostic_count);
  span.begin_segment_index = segment_index;
  span.end_segment_index = segment_index;
  span.begin_s_m = segment_start_s_m;
  span.end_s_m = segment_start_s_m;
  span.length_m = 0.0;
  span.begin_x_m = segment_start.x;
  span.begin_y_m = segment_start.y;
  span.end_x_m = segment_start.x;
  span.end_y_m = segment_start.y;
  ++evaluation.blocked_span_diagnostic_count;
}

void updateBlockedSpanDiagnostic(PathEvaluation& evaluation,
                                 const std::size_t diagnostic_index,
                                 const std::size_t segment_index,
                                 const double segment_end_s_m, const Point2 segment_end,
                                 const std::size_t prohibited_cells,
                                 const std::size_t outside_grid_segments) {
  if (diagnostic_index >= evaluation.blocked_span_diagnostic_count) {
    return;
  }
  TrajectoryOptimizerBlockedSpanDiagnostic& span =
      evaluation.blocked_span_diagnostics.at(diagnostic_index);
  span.end_segment_index = segment_index;
  span.end_s_m = segment_end_s_m;
  span.length_m = span.end_s_m - span.begin_s_m;
  span.end_x_m = segment_end.x;
  span.end_y_m = segment_end.y;
  span.prohibited_cells += prohibited_cells;
  span.outside_grid_segments += outside_grid_segments;
}

void recordBlockedPoint(PathEvaluation& evaluation, const std::size_t segment_index,
                        const double segment_start_s_m, const Point2 segment_start,
                        const Point2 segment_end, const Point2 blocked_point,
                        const bool outside_grid) {
  const Point2 segment = segment_end - segment_start;
  const double segment_length_sq = squaredDistance(segment_start, segment_end);
  const double t =
      segment_length_sq > kTinyDistanceM * kTinyDistanceM
          ? std::clamp(dot(blocked_point - segment_start, segment) / segment_length_sq,
                       0.0, 1.0)
          : 0.0;
  const double blocked_s_m =
      segment_start_s_m + t * distance(segment_start, segment_end);
  if (!evaluation.has_first_blocked_point ||
      blocked_s_m < evaluation.first_blocked_s_m) {
    evaluation.first_blocked_segment_index = segment_index;
    evaluation.first_blocked_s_m = blocked_s_m;
    evaluation.first_blocked_point = blocked_point;
    evaluation.has_first_blocked_point = true;
    evaluation.first_blocked_outside_grid = outside_grid;
  }
  if (!evaluation.has_last_blocked_point ||
      blocked_s_m >= evaluation.last_blocked_s_m) {
    evaluation.last_blocked_segment_index = segment_index;
    evaluation.last_blocked_s_m = blocked_s_m;
    evaluation.last_blocked_point = blocked_point;
    evaluation.has_last_blocked_point = true;
    evaluation.last_blocked_outside_grid = outside_grid;
  }
}

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] double percentileValue(std::vector<double>& values,
                                     const double percentile) {
  if (values.empty()) {
    return 0.0;
  }
  std::ranges::sort(values);
  const double bounded_percentile =
      std::isfinite(percentile) ? std::clamp(percentile, 0.0, 1.0) : 1.0;
  const std::size_t index =
      bounded_percentile <= 0.0
          ? 0U
          : std::min<std::size_t>(
                values.size() - 1U,
                static_cast<std::size_t>(std::ceil(
                    bounded_percentile * static_cast<double>(values.size()))) -
                    1U);
  return values[index];
}

void pointsFromOffsets(const std::span<const CorridorSample> corridor_samples,
                       const std::span<const double> offsets,
                       std::vector<Point2>& points) {
  points.clear();
  points.reserve(corridor_samples.size());
  if (corridor_samples.size() != offsets.size()) {
    return;
  }
  for (std::size_t i = 0U; i < corridor_samples.size(); ++i) {
    points.push_back(corridor_samples[i].center +
                     corridor_samples[i].normal * offsets[i]);
  }
}

[[nodiscard]] std::vector<Point2>
pointsFromOffsets(const std::span<const CorridorSample> corridor_samples,
                  const std::span<const double> offsets) {
  std::vector<Point2> points;
  pointsFromOffsets(corridor_samples, offsets, points);
  return points;
}

[[nodiscard]] bool offsetsNearlyEqual(const std::span<const double> lhs,
                                      const std::span<const double> rhs) noexcept {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0U; i < lhs.size(); ++i) {
    if (std::abs(lhs[i] - rhs[i]) > 1.0e-9) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] OffsetChangeDiagnostics
offsetChangeDiagnostics(const std::span<const double> base_offsets,
                        const std::span<const double> candidate_offsets) noexcept {
  OffsetChangeDiagnostics diagnostics{};
  if (base_offsets.size() != candidate_offsets.size() || base_offsets.empty()) {
    return diagnostics;
  }
  for (std::size_t i = 0U; i < base_offsets.size(); ++i) {
    if (std::abs(base_offsets[i] - candidate_offsets[i]) <= 1.0e-9) {
      continue;
    }
    if (diagnostics.changed_samples == 0U) {
      diagnostics.first_changed_index = i;
    }
    diagnostics.last_changed_index = i;
    ++diagnostics.changed_samples;
  }
  if (diagnostics.changed_samples > 0U) {
    diagnostics.changed_span_samples =
        diagnostics.last_changed_index - diagnostics.first_changed_index + 1U;
  }
  return diagnostics;
}

[[nodiscard]] std::size_t
estimatedLocalSpeedWindowSamples(const OffsetChangeDiagnostics& diagnostics,
                                 const std::size_t sample_count) noexcept {
  if (diagnostics.changed_samples == 0U || sample_count == 0U) {
    return 0U;
  }
  constexpr std::size_t kGeometryNeighborSamples = 2U;
  constexpr std::size_t kProfileContextSamples = 12U;
  const std::size_t context = kGeometryNeighborSamples + kProfileContextSamples;
  const std::size_t begin = diagnostics.first_changed_index > context
                                ? diagnostics.first_changed_index - context
                                : 0U;
  const std::size_t end =
      std::min(sample_count - 1U, diagnostics.last_changed_index + context);
  return end >= begin ? end - begin + 1U : 0U;
}

void samplesFromPointsAndOffsets(const std::span<const CorridorSample> corridor_samples,
                                 const std::span<const Point2> points,
                                 const std::span<const double> offsets,
                                 std::vector<TrajectoryPointSample>& samples) {
  samples.clear();
  if (corridor_samples.size() != points.size() || points.size() != offsets.size()) {
    return;
  }
  samples.reserve(points.size());
  for (std::size_t i = 0U; i < points.size(); ++i) {
    TrajectoryPointSample sample{};
    sample.point = points[i];
    sample.left_bound_m = corridor_samples[i].left_bound_m;
    sample.right_bound_m = corridor_samples[i].right_bound_m;
    sample.lateral_offset_m = offsets[i];
    samples.push_back(sample);
  }
}

[[nodiscard]] std::vector<TrajectoryPointSample>
samplesFromPointsAndOffsets(const std::span<const CorridorSample> corridor_samples,
                            const std::span<const Point2> points,
                            const std::span<const double> offsets) {
  std::vector<TrajectoryPointSample> samples;
  samplesFromPointsAndOffsets(corridor_samples, points, offsets, samples);
  return samples;
}

void applyOffsetDelta(std::vector<double>& offsets,
                      const std::span<const CorridorSample> corridor_samples,
                      const std::size_t center_index, const double delta_m,
                      const std::span<const std::uint8_t> mutable_indices) {
  constexpr std::array<std::pair<int, double>, 7U> kSmoothingKernel{
      {{-3, 0.125}, {-2, 0.25}, {-1, 0.5}, {0, 1.0}, {1, 0.5}, {2, 0.25}, {3, 0.125}}};
  if (offsets.size() != corridor_samples.size() || offsets.size() <= 2U) {
    return;
  }

  for (const auto& [relative_index, weight] : kSmoothingKernel) {
    if (relative_index < 0 &&
        center_index < static_cast<std::size_t>(-relative_index)) {
      continue;
    }
    const std::size_t index =
        relative_index < 0 ? center_index - static_cast<std::size_t>(-relative_index)
                           : center_index + static_cast<std::size_t>(relative_index);
    if (index == 0U || index + 1U >= offsets.size()) {
      continue;
    }
    if (!mutable_indices.empty() && mutable_indices[index] == 0U) {
      continue;
    }
    offsets[index] = std::clamp(offsets[index] + delta_m * weight,
                                -corridor_samples[index].right_bound_m,
                                corridor_samples[index].left_bound_m);
  }
}

[[nodiscard]] const char* initialOffsetSeedName(const InitialOffsetSeed seed) noexcept {
  switch (seed) {
    case InitialOffsetSeed::kCenterline:
      return "seed_centerline";
    case InitialOffsetSeed::kCorridorMidline:
      return "seed_corridor_midline";
    case InitialOffsetSeed::kLeftBiased:
      return "seed_left_biased";
    case InitialOffsetSeed::kRightBiased:
      return "seed_right_biased";
  }
  return "seed_unknown";
}

[[nodiscard]] double offsetForSeed(const CorridorSample& sample,
                                   const InitialOffsetSeed seed) noexcept {
  switch (seed) {
    case InitialOffsetSeed::kCenterline:
      return 0.0;
    case InitialOffsetSeed::kCorridorMidline:
      return std::clamp(0.5 * (sample.left_bound_m - sample.right_bound_m),
                        -sample.right_bound_m, sample.left_bound_m);
    case InitialOffsetSeed::kLeftBiased:
      return 0.75 * sample.left_bound_m;
    case InitialOffsetSeed::kRightBiased:
      return -0.75 * sample.right_bound_m;
  }
  return 0.0;
}

void offsetsFromSeed(const std::span<const CorridorSample> corridor_samples,
                     const InitialOffsetSeed seed, std::vector<double>& offsets) {
  offsets.assign(corridor_samples.size(), 0.0);
  if (corridor_samples.size() <= 2U) {
    return;
  }
  for (std::size_t i = 1U; i + 1U < corridor_samples.size(); ++i) {
    offsets[i] = offsetForSeed(corridor_samples[i], seed);
  }
}

[[nodiscard]] std::vector<CorridorSample>
optimizerCorridorSamples(const std::span<const CorridorSample> corridor_samples,
                         const TrajectoryOptimizerConfig& config) {
  const double sample_step_m =
      sanitizedPositive(config.optimizer_sample_step_m, 0.0, 0.0, 5000.0);
  if (!(sample_step_m > kTinyDistanceM) || corridor_samples.size() <= 2U) {
    return std::vector<CorridorSample>{corridor_samples.begin(),
                                       corridor_samples.end()};
  }

  std::vector<CorridorSample> samples;
  samples.reserve(corridor_samples.size());
  samples.push_back(corridor_samples.front());
  double last_s_m = corridor_samples.front().s_m;
  for (std::size_t i = 1U; i + 1U < corridor_samples.size(); ++i) {
    if (corridor_samples[i].s_m - last_s_m + kTinyDistanceM < sample_step_m) {
      continue;
    }
    samples.push_back(corridor_samples[i]);
    last_s_m = corridor_samples[i].s_m;
  }
  if (distance(samples.back().center, corridor_samples.back().center) >
      kTinyDistanceM) {
    samples.push_back(corridor_samples.back());
  }
  return samples;
}

[[nodiscard]] double pathLength(const std::span<const Point2> points) {
  double length = 0.0;
  for (std::size_t i = 1U; i < points.size(); ++i) {
    length += distance(points[i - 1U], points[i]);
  }
  return length;
}

[[nodiscard]] double headingDeltaRad(const Point2 lhs, const Point2 rhs) noexcept {
  const double lhs_norm = norm(lhs);
  const double rhs_norm = norm(rhs);
  if (!(lhs_norm > kTinyDistanceM) || !(rhs_norm > kTinyDistanceM)) {
    return 0.0;
  }
  const double cosine =
      std::clamp((lhs.x * rhs.x + lhs.y * rhs.y) / (lhs_norm * rhs_norm), -1.0, 1.0);
  return std::acos(cosine);
}

[[nodiscard]] double discreteCurvature(const Point2 previous, const Point2 current,
                                       const Point2 next) {
  const double a = distance(previous, current);
  const double b = distance(current, next);
  const double c = distance(previous, next);
  if (!(a > kTinyDistanceM) || !(b > kTinyDistanceM) || !(c > kTinyDistanceM)) {
    return 0.0;
  }
  const double signed_double_area = cross(current - previous, next - previous);
  return 2.0 * signed_double_area / (a * b * c);
}

[[nodiscard]] double edgeMarginM(const CorridorSample& sample,
                                 const double offset_m) noexcept {
  return std::min(sample.left_bound_m - offset_m, sample.right_bound_m + offset_m);
}

[[nodiscard]] PathEvaluation evaluatePath(const OccupancyGrid2D& grid,
                                          const std::span<const Point2> points) {
  PathEvaluation evaluation{};
  if (points.size() < 2U) {
    ++evaluation.outside_grid_segments;
    return evaluation;
  }

  bool previous_segment_blocked = false;
  std::size_t current_span_diagnostic_index = kMaxCenterlineBlockedSpanDiagnostics;
  for (std::size_t i = 1U; i < points.size(); ++i) {
    const Point2 start = points[i - 1U];
    const Point2 end = points[i];
    const double segment_start_s_m = evaluation.length_m;
    evaluation.length_m += distance(start, end);
    const std::optional<GridIndex> start_cell = grid.worldToCell(start);
    const std::optional<GridIndex> end_cell = grid.worldToCell(end);
    bool segment_blocked = false;
    std::size_t segment_prohibited_cells = 0U;
    std::size_t segment_outside_grid_segments = 0U;
    if (!start_cell.has_value() || !end_cell.has_value()) {
      ++evaluation.outside_grid_segments;
      segment_outside_grid_segments = 1U;
      segment_blocked = true;
      recordBlockedPoint(evaluation, i - 1U, segment_start_s_m, start, end, start,
                         true);
      recordBlockedPoint(evaluation, i - 1U, segment_start_s_m, start, end, end, true);
    } else {
      const std::vector<GridIndex> cells = grid.cellsOnLine(*start_cell, *end_cell);
      for (const GridIndex cell : cells) {
        if (grid.isProhibited(cell)) {
          ++evaluation.prohibited_cells;
          ++segment_prohibited_cells;
          segment_blocked = true;
          recordBlockedPoint(evaluation, i - 1U, segment_start_s_m, start, end,
                             grid.cellCenter(cell), false);
        }
      }
    }
    if (segment_blocked) {
      ++evaluation.blocked_segment_count;
      if (!previous_segment_blocked) {
        ++evaluation.blocked_span_count;
        const std::size_t previous_diagnostic_count =
            evaluation.blocked_span_diagnostic_count;
        startBlockedSpanDiagnostic(evaluation, i - 1U, segment_start_s_m, start);
        current_span_diagnostic_index =
            evaluation.blocked_span_diagnostic_count > previous_diagnostic_count
                ? evaluation.blocked_span_diagnostic_count - 1U
                : kMaxCenterlineBlockedSpanDiagnostics;
      }
      updateBlockedSpanDiagnostic(evaluation, current_span_diagnostic_index, i - 1U,
                                  evaluation.length_m, end, segment_prohibited_cells,
                                  segment_outside_grid_segments);
    } else {
      current_span_diagnostic_index = kMaxCenterlineBlockedSpanDiagnostics;
    }
    previous_segment_blocked = segment_blocked;
  }
  return evaluation;
}

[[nodiscard]] SegmentCellKey orderedSegmentKey(GridIndex start,
                                               GridIndex end) noexcept {
  if (end.x < start.x || (end.x == start.x && end.y < start.y)) {
    std::swap(start, end);
  }
  return SegmentCellKey{.start = start, .end = end};
}

[[nodiscard]] bool cachedSegmentTraversable(const OccupancyGrid2D& grid,
                                            const Point2 start, const Point2 end,
                                            SegmentTraversabilityCache& cache,
                                            std::size_t& hits, std::size_t& misses) {
  const std::optional<GridIndex> start_cell = grid.worldToCell(start);
  const std::optional<GridIndex> end_cell = grid.worldToCell(end);
  if (!start_cell.has_value() || !end_cell.has_value()) {
    ++misses;
    return false;
  }
  const SegmentCellKey key = orderedSegmentKey(*start_cell, *end_cell);
  if (const auto iter = cache.values.find(key); iter != cache.values.end()) {
    ++hits;
    return iter->second;
  }
  ++misses;
  const bool traversable = std::ranges::all_of(
      grid.cellsOnLine(*start_cell, *end_cell),
      [&grid](const GridIndex cell) { return !grid.isProhibited(cell); });
  cache.values.emplace(key, traversable);
  return traversable;
}

[[nodiscard]] std::pair<std::size_t, std::size_t>
localScoreWindowForCenter(const std::size_t center_index,
                          const std::size_t sample_count,
                          const std::size_t radius_samples) noexcept {
  if (sample_count == 0U) {
    return {0U, 0U};
  }
  const std::size_t begin =
      center_index > radius_samples ? center_index - radius_samples : 0U;
  const std::size_t end = std::min(sample_count - 1U, center_index + radius_samples);
  return {begin, end};
}

void pointsFromOffsetsRange(const std::span<const CorridorSample> corridor_samples,
                            const std::span<const double> offsets,
                            const std::size_t begin_index, const std::size_t end_index,
                            std::vector<Point2>& points) {
  points.clear();
  if (corridor_samples.size() != offsets.size() || begin_index > end_index ||
      end_index >= corridor_samples.size()) {
    return;
  }
  points.reserve(end_index - begin_index + 1U);
  for (std::size_t i = begin_index; i <= end_index; ++i) {
    points.push_back(corridor_samples[i].center +
                     corridor_samples[i].normal * offsets[i]);
  }
}

void copyRange(const std::span<const Point2> source, const std::size_t begin_index,
               const std::size_t end_index, std::vector<Point2>& destination) {
  destination.clear();
  if (begin_index > end_index || end_index >= source.size()) {
    return;
  }
  destination.reserve(end_index - begin_index + 1U);
  for (std::size_t i = begin_index; i <= end_index; ++i) {
    destination.push_back(source[i]);
  }
}

void copyRange(const std::span<const double> source, const std::size_t begin_index,
               const std::size_t end_index, std::vector<double>& destination) {
  destination.clear();
  if (begin_index > end_index || end_index >= source.size()) {
    return;
  }
  destination.reserve(end_index - begin_index + 1U);
  for (std::size_t i = begin_index; i <= end_index; ++i) {
    destination.push_back(source[i]);
  }
}

void copyRange(const std::span<const CorridorSample> source,
               const std::size_t begin_index, const std::size_t end_index,
               std::vector<CorridorSample>& destination) {
  destination.clear();
  if (begin_index > end_index || end_index >= source.size()) {
    return;
  }
  destination.reserve(end_index - begin_index + 1U);
  for (std::size_t i = begin_index; i <= end_index; ++i) {
    destination.push_back(source[i]);
  }
}

[[nodiscard]] PathEvaluation
evaluateLocalPathWindowCached(const OccupancyGrid2D& grid,
                              const std::span<const Point2> local_points,
                              SegmentTraversabilityCache& cache,
                              std::size_t& cache_hits, std::size_t& cache_misses) {
  PathEvaluation evaluation{};
  if (local_points.size() < 2U) {
    ++evaluation.outside_grid_segments;
    return evaluation;
  }
  for (std::size_t i = 1U; i < local_points.size(); ++i) {
    const Point2 start = local_points[i - 1U];
    const Point2 end = local_points[i];
    evaluation.length_m += distance(start, end);
    if (!cachedSegmentTraversable(grid, start, end, cache, cache_hits, cache_misses)) {
      ++evaluation.prohibited_cells;
    }
  }
  return evaluation;
}

[[nodiscard]] PathEvaluation evaluatePathCached(const OccupancyGrid2D& grid,
                                                const std::span<const Point2> points,
                                                SegmentProhibitedCountCache& cache,
                                                std::size_t& cache_hits,
                                                std::size_t& cache_misses) {
  PathEvaluation evaluation{};
  if (points.size() < 2U) {
    ++evaluation.outside_grid_segments;
    return evaluation;
  }
  for (std::size_t i = 1U; i < points.size(); ++i) {
    const Point2 start = points[i - 1U];
    const Point2 end = points[i];
    evaluation.length_m += distance(start, end);
    const std::optional<GridIndex> start_cell = grid.worldToCell(start);
    const std::optional<GridIndex> end_cell = grid.worldToCell(end);
    if (!start_cell.has_value() || !end_cell.has_value()) {
      ++cache_misses;
      ++evaluation.outside_grid_segments;
      continue;
    }
    const SegmentCellKey key = orderedSegmentKey(*start_cell, *end_cell);
    const auto cached = cache.values.find(key);
    const std::size_t prohibited_cells =
        cached != cache.values.end() ? (++cache_hits, cached->second) : [&] {
          ++cache_misses;
          std::size_t computed = 0U;
          for (const GridIndex cell : grid.cellsOnLine(*start_cell, *end_cell)) {
            if (grid.isProhibited(cell)) {
              ++computed;
            }
          }
          cache.values.emplace(key, computed);
          return computed;
        }();
    evaluation.prohibited_cells += prohibited_cells;
  }
  return evaluation;
}

void populateSampleGeometry(std::vector<TrajectoryPointSample>& samples) {
  double s_m = 0.0;
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    if (i > 0U) {
      s_m += distance(samples[i - 1U].point, samples[i].point);
    }
    samples[i].s_m = s_m;
    if (samples.size() == 1U) {
      samples[i].tangent = Point2{1.0, 0.0};
    } else if (i == 0U) {
      samples[i].tangent = normalized(samples[i + 1U].point - samples[i].point);
    } else if (i + 1U == samples.size()) {
      samples[i].tangent = normalized(samples[i].point - samples[i - 1U].point);
    } else {
      samples[i].tangent = normalized(samples[i + 1U].point - samples[i - 1U].point);
      samples[i].curvature_1pm = discreteCurvature(
          samples[i - 1U].point, samples[i].point, samples[i + 1U].point);
    }
  }
}

void updateCurvatureStats(const std::span<const TrajectoryPointSample> samples,
                          TrajectoryOptimizerStats& stats) {
  double curvature_sum = 0.0;
  std::size_t curvature_count = 0U;
  for (const TrajectoryPointSample& sample : samples) {
    const double abs_curvature = std::abs(sample.curvature_1pm);
    stats.max_abs_curvature_1pm = std::max(stats.max_abs_curvature_1pm, abs_curvature);
    curvature_sum += abs_curvature;
    ++curvature_count;
  }
  if (curvature_count > 0U) {
    stats.mean_abs_curvature_1pm = curvature_sum / static_cast<double>(curvature_count);
  }
}

void copyTraversalEstimateToFinalStats(const TraversalTimeEstimate& estimate,
                                       TrajectoryOptimizerStats& stats) {
  stats.estimated_time_s = estimate.estimated_time_s;
  stats.min_speed_limit_mps = estimate.min_speed_limit_mps;
  stats.max_speed_limit_mps = estimate.max_speed_limit_mps;
  stats.curvature_limited_samples = estimate.curvature_limited_samples;
}

void copyCostBreakdownToStats(const CostBreakdown& breakdown,
                              TrajectoryOptimizerStats& stats) {
  stats.cost_curvature = breakdown.curvature_cost;
  stats.cost_curvature_change = breakdown.curvature_change_cost;
  stats.cost_radius_shortfall = breakdown.radius_shortfall_cost;
  stats.cost_heading_jump = breakdown.heading_jump_cost;
  stats.cost_offset_change = breakdown.offset_change_cost;
  stats.cost_offset_second_change = breakdown.offset_second_change_cost;
  stats.cost_offset_slope = breakdown.offset_slope_cost;
  stats.cost_collision = breakdown.collision_cost;
  stats.cost_outside_grid = breakdown.outside_grid_cost;
}

void copyCostBreakdownToCandidateDiagnostic(
    const CostBreakdown& breakdown,
    TrajectoryOptimizerCandidateDiagnostic& diagnostic) {
  diagnostic.cost_curvature = breakdown.curvature_cost;
  diagnostic.cost_curvature_change = breakdown.curvature_change_cost;
  diagnostic.cost_radius_shortfall = breakdown.radius_shortfall_cost;
  diagnostic.cost_heading_jump = breakdown.heading_jump_cost;
  diagnostic.cost_offset_change = breakdown.offset_change_cost;
  diagnostic.cost_offset_second_change = breakdown.offset_second_change_cost;
  diagnostic.cost_offset_slope = breakdown.offset_slope_cost;
  diagnostic.cost_collision = breakdown.collision_cost;
  diagnostic.cost_outside_grid = breakdown.outside_grid_cost;
}

void populateCandidateDiagnosticFromScore(
    TrajectoryOptimizerCandidateDiagnostic& diagnostic, const CandidateScore& score,
    const PathEvaluation& evaluation, const double incumbent_score, const bool accepted,
    const double point_build_duration_ms, const double path_evaluation_duration_ms,
    const double score_duration_ms, const double full_score_duration_ms) {
  diagnostic.score = score.score;
  diagnostic.incumbent_score = incumbent_score;
  diagnostic.length_m = evaluation.length_m;
  diagnostic.traversable = evaluation.traversable();
  diagnostic.full_score_used = true;
  diagnostic.prohibited_cells = evaluation.prohibited_cells;
  diagnostic.outside_grid_segments = evaluation.outside_grid_segments;
  diagnostic.point_build_duration_ms = point_build_duration_ms;
  diagnostic.path_evaluation_duration_ms = path_evaluation_duration_ms;
  diagnostic.score_duration_ms = score_duration_ms;
  diagnostic.full_score_duration_ms = full_score_duration_ms;
  copyCostBreakdownToCandidateDiagnostic(score.breakdown, diagnostic);
  if (accepted) {
    diagnostic.decision = "selected";
  } else if (evaluation.outside_grid_segments > 0U) {
    diagnostic.decision = "outside_grid";
  } else if (evaluation.prohibited_cells > 0U) {
    diagnostic.decision = "prohibited";
  } else {
    diagnostic.decision = "not_better_than_incumbent";
  }
}

void updateEdgeMarginStats(const std::span<const TrajectoryPointSample> samples,
                           TrajectoryOptimizerStats& stats) {
  double margin_sum = 0.0;
  std::size_t margin_count = 0U;
  for (const TrajectoryPointSample& sample : samples) {
    CorridorSample bounds{};
    bounds.left_bound_m = sample.left_bound_m;
    bounds.right_bound_m = sample.right_bound_m;
    const double margin = edgeMarginM(bounds, sample.lateral_offset_m);
    if (!std::isfinite(margin)) {
      continue;
    }
    if (!std::isfinite(stats.min_edge_margin_m)) {
      stats.min_edge_margin_m = margin;
    } else {
      stats.min_edge_margin_m = std::min(stats.min_edge_margin_m, margin);
    }
    margin_sum += margin;
    ++margin_count;
  }
  if (margin_count > 0U) {
    stats.mean_edge_margin_m = margin_sum / static_cast<double>(margin_count);
  }
}

} // namespace drone_city_nav::trajectory_optimizer_detail
