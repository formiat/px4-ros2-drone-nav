#include "drone_city_nav/turn_smoothing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] Point2 operator+(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] Point2 operator*(const Point2 point, const double scale) noexcept {
  return Point2{point.x * scale, point.y * scale};
}

[[nodiscard]] double dot(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

[[nodiscard]] double cross(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.y - lhs.y * rhs.x;
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

[[nodiscard]] Point2 leftNormal(const Point2 tangent) noexcept {
  return Point2{-tangent.y, tangent.x};
}

[[nodiscard]] double normalizeAngle(const double angle_rad) noexcept {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

[[nodiscard]] double headingOf(const Point2 vector) noexcept {
  return std::atan2(vector.y, vector.x);
}

[[nodiscard]] double signedHeadingDelta(const Point2 previous,
                                        const Point2 next) noexcept {
  return normalizeAngle(headingOf(next) - headingOf(previous));
}

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
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
      samples[i].tangent = normalized(samples[1U].point - samples[0U].point);
    } else if (i + 1U == samples.size()) {
      samples[i].tangent = normalized(samples[i].point - samples[i - 1U].point);
    } else {
      samples[i].tangent = normalized(samples[i + 1U].point - samples[i - 1U].point);
      samples[i].curvature_1pm = discreteCurvature(
          samples[i - 1U].point, samples[i].point, samples[i + 1U].point);
    }
  }
}

[[nodiscard]] double pathLength(const std::span<const TrajectoryPointSample> samples,
                                const std::size_t start_index,
                                const std::size_t end_index) {
  if (samples.empty() || start_index >= samples.size() || end_index >= samples.size() ||
      start_index >= end_index) {
    return 0.0;
  }
  double length = 0.0;
  for (std::size_t i = start_index + 1U; i <= end_index; ++i) {
    length += distance(samples[i - 1U].point, samples[i].point);
  }
  return length;
}

[[nodiscard]] double pathLength(const std::span<const TrajectoryPointSample> samples) {
  if (samples.size() < 2U) {
    return 0.0;
  }
  return pathLength(samples, 0U, samples.size() - 1U);
}

[[nodiscard]] std::optional<CorridorSample>
nearestCorridorSample(const std::span<const CorridorSample> corridor_samples,
                      const Point2 point) {
  if (corridor_samples.empty()) {
    return std::nullopt;
  }
  const CorridorSample* best = nullptr;
  double best_distance_sq = std::numeric_limits<double>::infinity();
  for (const CorridorSample& sample : corridor_samples) {
    const double distance_sq = squaredDistance(sample.center, point);
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best = &sample;
    }
  }
  if (best == nullptr) {
    return std::nullopt;
  }
  return *best;
}

[[nodiscard]] std::optional<CorridorSample>
corridorSampleAtS(const std::span<const CorridorSample> corridor_samples,
                  const double s_m) {
  if (corridor_samples.empty()) {
    return std::nullopt;
  }
  if (s_m <= corridor_samples.front().s_m) {
    return corridor_samples.front();
  }
  for (std::size_t i = 1U; i < corridor_samples.size(); ++i) {
    if (s_m > corridor_samples[i].s_m && i + 1U < corridor_samples.size()) {
      continue;
    }
    const CorridorSample& previous = corridor_samples[i - 1U];
    const CorridorSample& next = corridor_samples[i];
    const double span = next.s_m - previous.s_m;
    const double t =
        span > kTinyDistanceM ? std::clamp((s_m - previous.s_m) / span, 0.0, 1.0) : 0.0;
    CorridorSample sample{};
    sample.s_m = previous.s_m + (next.s_m - previous.s_m) * t;
    sample.route_center =
        previous.route_center + (next.route_center - previous.route_center) * t;
    sample.center = previous.center + (next.center - previous.center) * t;
    sample.tangent =
        normalized(previous.tangent + (next.tangent - previous.tangent) * t);
    if (!(norm(sample.tangent) > kTinyDistanceM)) {
      sample.tangent = previous.tangent;
    }
    sample.normal = leftNormal(sample.tangent);
    sample.left_bound_m =
        previous.left_bound_m + (next.left_bound_m - previous.left_bound_m) * t;
    sample.right_bound_m =
        previous.right_bound_m + (next.right_bound_m - previous.right_bound_m) * t;
    sample.clearance_m =
        previous.clearance_m + (next.clearance_m - previous.clearance_m) * t;
    sample.center_recovery_m =
        previous.center_recovery_m +
        (next.center_recovery_m - previous.center_recovery_m) * t;
    return sample;
  }
  return corridor_samples.back();
}

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

[[nodiscard]] bool pointInsideCorridor(const CorridorSample& corridor,
                                       const Point2 point,
                                       const double margin_m) noexcept {
  const double offset_m = dot(point - corridor.center, corridor.normal);
  return offset_m <= corridor.left_bound_m - margin_m + kTinyDistanceM &&
         -offset_m <= corridor.right_bound_m - margin_m + kTinyDistanceM;
}

[[nodiscard]] bool
pathInsideCorridor(const std::span<const TrajectoryPointSample> samples,
                   const std::span<const CorridorSample> corridor_samples,
                   const double margin_m) {
  return std::ranges::all_of(samples, [&](const TrajectoryPointSample& sample) {
    const std::optional<CorridorSample> corridor =
        nearestCorridorSample(corridor_samples, sample.point);
    return corridor.has_value() &&
           pointInsideCorridor(*corridor, sample.point, margin_m);
  });
}

[[nodiscard]] double innerMarginM(const TrajectoryPointSample& sample,
                                  const double turn_sign) noexcept {
  if (turn_sign > 0.0) {
    return sample.left_bound_m - sample.racing_offset_m;
  }
  return sample.right_bound_m + sample.racing_offset_m;
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

struct CornerCandidate {
  bool valid{false};
  std::size_t index{0U};
  double signed_heading_delta_rad{0.0};
  double abs_heading_delta_rad{0.0};
  double radius_m{std::numeric_limits<double>::infinity()};
  double turn_sign{1.0};
};

[[nodiscard]] CornerCandidate
cornerCandidateAt(const std::span<const TrajectoryPointSample> samples,
                  const std::size_t index, const TurnSmoothingConfig& config) {
  CornerCandidate candidate{};
  if (index == 0U || index + 1U >= samples.size()) {
    return candidate;
  }

  const Point2 previous_direction =
      normalized(samples[index].point - samples[index - 1U].point);
  const Point2 next_direction =
      normalized(samples[index + 1U].point - samples[index].point);
  if (!(norm(previous_direction) > kTinyDistanceM) ||
      !(norm(next_direction) > kTinyDistanceM)) {
    return candidate;
  }

  const double signed_delta = signedHeadingDelta(previous_direction, next_direction);
  const double abs_delta = std::abs(signed_delta);
  const double curvature = std::abs(discreteCurvature(
      samples[index - 1U].point, samples[index].point, samples[index + 1U].point));
  const double radius = curvature > kTinyDistanceM
                            ? 1.0 / curvature
                            : std::numeric_limits<double>::infinity();
  const double trigger_heading_delta =
      sanitizedPositive(config.trigger_heading_delta_rad, 0.65, 0.0, std::numbers::pi);
  const double trigger_min_radius =
      sanitizedPositive(config.trigger_min_radius_m, 12.0, 0.0, 10000.0);
  const bool triggered_by_heading = abs_delta >= trigger_heading_delta;
  const bool triggered_by_radius =
      radius < trigger_min_radius && abs_delta > trigger_heading_delta * 0.35;
  if (!triggered_by_heading && !triggered_by_radius) {
    return candidate;
  }

  candidate.valid = true;
  candidate.index = index;
  candidate.signed_heading_delta_rad = signed_delta;
  candidate.abs_heading_delta_rad = abs_delta;
  candidate.radius_m = radius;
  candidate.turn_sign = signed_delta >= 0.0 ? 1.0 : -1.0;
  return candidate;
}

[[nodiscard]] std::size_t
findEntryIndex(const std::span<const TrajectoryPointSample> samples,
               const std::size_t corner, const double entry_distance_m) {
  double accumulated = 0.0;
  std::size_t index = corner;
  while (index > 0U && accumulated < entry_distance_m) {
    accumulated += distance(samples[index].point, samples[index - 1U].point);
    --index;
  }
  return index;
}

[[nodiscard]] std::size_t
findExitIndex(const std::span<const TrajectoryPointSample> samples,
              const std::size_t corner, const double exit_distance_m) {
  double accumulated = 0.0;
  std::size_t index = corner;
  while (index + 1U < samples.size() && accumulated < exit_distance_m) {
    accumulated += distance(samples[index].point, samples[index + 1U].point);
    ++index;
  }
  return index;
}

[[nodiscard]] double outwardShiftFor(const TrajectoryPointSample& sample,
                                     const Point2 outward,
                                     const TurnSmoothingConfig& config) {
  const Point2 normal = leftNormal(sample.tangent);
  const double side = dot(outward, normal);
  const double bound = side >= 0.0 ? sample.left_bound_m : sample.right_bound_m;
  const double margin =
      sanitizedPositive(config.min_corridor_margin_m, 0.5, 0.0, 1000.0);
  if (!std::isfinite(bound) || bound <= margin) {
    return 0.0;
  }
  const double ratio = sanitizedPositive(config.outer_bias_ratio, 0.45, 0.0, 1.0);
  const double max_shift =
      sanitizedPositive(config.max_outer_shift_m, 12.0, 0.0, 1000.0);
  const double min_shift =
      sanitizedPositive(config.min_outer_shift_m, 2.0, 0.0, max_shift);
  const double available_shift = std::max(0.0, bound - margin);
  return std::min(max_shift, std::min(available_shift,
                                      std::max(min_shift, available_shift * ratio)));
}

[[nodiscard]] Point2 cubicBezier(const Point2 p0, const Point2 p1, const Point2 p2,
                                 const Point2 p3, const double t) noexcept {
  const double u = 1.0 - t;
  return p0 * (u * u * u) + p1 * (3.0 * u * u * t) + p2 * (3.0 * u * t * t) +
         p3 * (t * t * t);
}

[[nodiscard]] TrajectoryPointSample
sampleForPoint(const Point2 point, const double station_hint_m,
               const std::span<const CorridorSample> corridor_samples) {
  TrajectoryPointSample sample{};
  sample.point = point;
  const std::optional<CorridorSample> corridor =
      corridorSampleAtS(corridor_samples, station_hint_m);
  if (corridor.has_value()) {
    sample.tangent = corridor->tangent;
    sample.left_bound_m = corridor->left_bound_m;
    sample.right_bound_m = corridor->right_bound_m;
    sample.racing_offset_m = dot(point - corridor->center, corridor->normal);
  }
  return sample;
}

[[nodiscard]] std::vector<TrajectoryPointSample>
buildBezierSamples(const std::span<const TrajectoryPointSample> samples,
                   const std::span<const CorridorSample> corridor_samples,
                   const std::size_t entry_index, const std::size_t corner_index,
                   const std::size_t exit_index, const CornerCandidate& corner,
                   const TurnSmoothingConfig& config, double& applied_shift_m) {
  const Point2 p0 = samples[entry_index].point;
  const Point2 p3 = samples[exit_index].point;
  const Point2 incoming =
      normalized(samples[corner_index].point - samples[corner_index - 1U].point);
  const Point2 outgoing =
      normalized(samples[corner_index + 1U].point - samples[corner_index].point);
  if (!(norm(incoming) > kTinyDistanceM) || !(norm(outgoing) > kTinyDistanceM)) {
    return {};
  }

  const Point2 entry_outward = leftNormal(incoming) * -corner.turn_sign;
  const Point2 exit_outward = leftNormal(outgoing) * -corner.turn_sign;
  const double entry_shift =
      outwardShiftFor(samples[entry_index], entry_outward, config);
  const double exit_shift = outwardShiftFor(samples[exit_index], exit_outward, config);
  applied_shift_m = std::max(entry_shift, exit_shift);

  const double local_length = pathLength(samples, entry_index, exit_index);
  const double control_distance =
      std::clamp(local_length * 0.35, 2.0, std::max(2.0, local_length * 0.6));
  const Point2 p1 = p0 + incoming * control_distance + entry_outward * entry_shift;
  const Point2 p2 = p3 - outgoing * control_distance + exit_outward * exit_shift;
  const double sample_step =
      sanitizedPositive(config.sample_step_m, 1.0, 0.1, std::max(0.1, local_length));
  const std::size_t steps = std::max<std::size_t>(
      3U, static_cast<std::size_t>(std::ceil(local_length / sample_step)));

  std::vector<TrajectoryPointSample> bezier_samples;
  bezier_samples.reserve(steps + 1U);
  const double s0 = samples[entry_index].s_m;
  const double s1 = samples[exit_index].s_m;
  for (std::size_t i = 0U; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const Point2 point = cubicBezier(p0, p1, p2, p3, t);
    if (!finite2D(point)) {
      return {};
    }
    bezier_samples.push_back(
        sampleForPoint(point, s0 + (s1 - s0) * t, corridor_samples));
  }
  populateSampleGeometry(bezier_samples);
  return bezier_samples;
}

[[nodiscard]] std::vector<TrajectoryPointSample>
replaceRange(const std::span<const TrajectoryPointSample> samples,
             const std::size_t entry_index, const std::size_t exit_index,
             const std::span<const TrajectoryPointSample> replacement) {
  std::vector<TrajectoryPointSample> result;
  result.reserve(samples.size() + replacement.size());
  result.insert(result.end(), samples.begin(),
                samples.begin() + static_cast<std::ptrdiff_t>(entry_index));
  result.insert(result.end(), replacement.begin(), replacement.end());
  result.insert(result.end(),
                samples.begin() + static_cast<std::ptrdiff_t>(exit_index + 1U),
                samples.end());
  populateSampleGeometry(result);
  return result;
}

[[nodiscard]] std::optional<CornerCandidate>
worstCorner(const std::span<const TrajectoryPointSample> samples,
            const TurnSmoothingConfig& config, TurnSmoothingStats& stats) {
  std::optional<CornerCandidate> best;
  for (std::size_t i = 1U; i + 1U < samples.size(); ++i) {
    const CornerCandidate candidate = cornerCandidateAt(samples, i, config);
    if (!candidate.valid) {
      continue;
    }
    ++stats.detected_corners;
    updateMinInnerMargin(stats, samples[i], candidate.turn_sign);
    if (!best.has_value() ||
        candidate.abs_heading_delta_rad > best->abs_heading_delta_rad) {
      best = candidate;
    }
  }
  return best;
}

[[nodiscard]] bool improvedShape(const TrajectoryShapeDiagnostics& before,
                                 const TrajectoryShapeDiagnostics& after,
                                 const TurnSmoothingConfig& config) {
  const double min_improvement = sanitizedPositive(config.min_heading_improvement_rad,
                                                   0.05, 0.0, std::numbers::pi);
  if (after.max_heading_delta_rad + min_improvement < before.max_heading_delta_rad) {
    return true;
  }
  return after.max_heading_delta_rad <= before.max_heading_delta_rad + 1.0e-9 &&
         after.max_curvature_jump_1pm + 1.0e-9 < before.max_curvature_jump_1pm;
}

} // namespace

TurnSmoothingResult
smoothTrajectoryTurns(const std::span<const TrajectoryPointSample> samples,
                      const std::span<const CorridorSample> corridor_samples,
                      const OccupancyGrid2D& prohibited_grid,
                      const TurnSmoothingConfig& config) {
  TurnSmoothingResult result{};
  result.stats.input_samples = samples.size();
  result.samples.assign(samples.begin(), samples.end());
  if (samples.size() < 3U || corridor_samples.empty()) {
    result.stats.output_samples = result.samples.size();
    result.valid = trajectorySamplesAreUsable(result.samples);
    return result;
  }
  populateSampleGeometry(result.samples);

  const TrajectoryShapeDiagnostics initial_shape =
      computeTrajectoryShapeDiagnostics(result.samples);
  result.stats.max_heading_delta_before_rad = initial_shape.max_heading_delta_rad;
  result.stats.max_curvature_jump_before_1pm = initial_shape.max_curvature_jump_1pm;

  const std::size_t max_passes =
      std::clamp<std::size_t>(config.max_passes, 0U, static_cast<std::size_t>(100U));
  for (std::size_t pass = 0U; pass < max_passes; ++pass) {
    TurnSmoothingStats detection_stats{};
    const std::optional<CornerCandidate> corner =
        worstCorner(result.samples, config, detection_stats);
    result.stats.detected_corners += detection_stats.detected_corners;
    if (std::isfinite(detection_stats.min_inner_margin_m)) {
      if (!std::isfinite(result.stats.min_inner_margin_m)) {
        result.stats.min_inner_margin_m = detection_stats.min_inner_margin_m;
      } else {
        result.stats.min_inner_margin_m = std::min(result.stats.min_inner_margin_m,
                                                   detection_stats.min_inner_margin_m);
      }
    }
    if (!corner.has_value()) {
      break;
    }

    ++result.stats.attempted_corners;
    const double entry_distance =
        sanitizedPositive(config.entry_distance_m, 45.0, 0.1, 5000.0);
    const double exit_distance =
        sanitizedPositive(config.exit_distance_m, 45.0, 0.1, 5000.0);
    const std::size_t entry_index =
        findEntryIndex(result.samples, corner->index, entry_distance);
    const std::size_t exit_index =
        findExitIndex(result.samples, corner->index, exit_distance);
    if (entry_index >= corner->index || exit_index <= corner->index) {
      ++result.stats.rejected_not_improved;
      break;
    }

    double applied_shift_m = 0.0;
    const std::vector<TrajectoryPointSample> replacement =
        buildBezierSamples(result.samples, corridor_samples, entry_index, corner->index,
                           exit_index, *corner, config, applied_shift_m);
    if (replacement.size() < 3U) {
      ++result.stats.rejected_not_improved;
      break;
    }

    std::vector<TrajectoryPointSample> candidate =
        replaceRange(result.samples, entry_index, exit_index, replacement);
    if (!pathIsTraversable(prohibited_grid, candidate)) {
      ++result.stats.rejected_prohibited;
      break;
    }
    const double corridor_margin =
        sanitizedPositive(config.min_corridor_margin_m, 0.5, 0.0, 1000.0);
    if (!pathInsideCorridor(candidate, corridor_samples, corridor_margin)) {
      ++result.stats.rejected_corridor;
      break;
    }

    const double previous_length = pathLength(result.samples);
    const double candidate_length = pathLength(candidate);
    const double max_length_ratio =
        sanitizedPositive(config.max_length_ratio, 1.25, 1.0, 100.0);
    if (previous_length > kTinyDistanceM &&
        candidate_length > previous_length * max_length_ratio) {
      ++result.stats.rejected_length;
      break;
    }

    const TrajectoryShapeDiagnostics before_shape =
        computeTrajectoryShapeDiagnostics(result.samples);
    const TrajectoryShapeDiagnostics after_shape =
        computeTrajectoryShapeDiagnostics(candidate);
    if (!improvedShape(before_shape, after_shape, config)) {
      ++result.stats.rejected_not_improved;
      break;
    }

    result.samples = std::move(candidate);
    result.changed = true;
    ++result.stats.smoothed_corners;
    result.stats.max_applied_outer_shift_m =
        std::max(result.stats.max_applied_outer_shift_m, applied_shift_m);
  }

  populateSampleGeometry(result.samples);
  const TrajectoryShapeDiagnostics final_shape =
      computeTrajectoryShapeDiagnostics(result.samples);
  result.stats.output_samples = result.samples.size();
  result.stats.max_heading_delta_after_rad = final_shape.max_heading_delta_rad;
  result.stats.max_curvature_jump_after_1pm = final_shape.max_curvature_jump_1pm;
  result.valid = trajectorySamplesAreUsable(result.samples) &&
                 pathIsTraversable(prohibited_grid, result.samples);
  return result;
}

} // namespace drone_city_nav
