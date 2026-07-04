#include "turn_smoothing_internal.hpp"

namespace drone_city_nav::turn_smoothing_detail {

[[nodiscard]] CornerCandidate
cornerCandidateAt(const std::span<const TrajectoryPointSample> samples,
                  const std::size_t index, const TurnSmoothingConfig& config,
                  const VelocityFollowerConfig& speed_config) {
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
      sanitizedPositive(config.trigger_min_radius_m, 16.0, 0.0, 10000.0);
  const double trigger_speed_limit =
      sanitizedPositive(config.trigger_speed_limit_mps, 12.0, 0.0, 1000.0);
  const double geometric_speed_limit = curvatureSpeedLimitMps(radius, speed_config);
  const bool triggered_by_heading = abs_delta >= trigger_heading_delta;
  const bool triggered_by_radius =
      radius < trigger_min_radius && abs_delta > trigger_heading_delta * 0.35;
  const bool triggered_by_speed = trigger_speed_limit > kTinyDistanceM &&
                                  geometric_speed_limit <= trigger_speed_limit &&
                                  abs_delta > trigger_heading_delta * 0.30;
  if (!triggered_by_heading && !triggered_by_radius && !triggered_by_speed) {
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
  if (!std::isfinite(bound) || !(bound > 0.0)) {
    return 0.0;
  }
  const double ratio = sanitizedPositive(config.outer_bias_ratio, 0.45, 0.0, 1.0);
  const double max_shift =
      sanitizedPositive(config.max_outer_shift_m, 12.0, 0.0, 1000.0);
  const double min_shift =
      sanitizedPositive(config.min_outer_shift_m, 2.0, 0.0, max_shift);
  const double available_shift = bound;
  return std::min(max_shift, std::min(available_shift,
                                      std::max(min_shift, available_shift * ratio)));
}

[[nodiscard]] Point2 cubicBezier(const Point2 p0, const Point2 p1, const Point2 p2,
                                 const Point2 p3, const double t) noexcept {
  const double u = 1.0 - t;
  return p0 * (u * u * u) + p1 * (3.0 * u * u * t) + p2 * (3.0 * u * t * t) +
         p3 * (t * t * t);
}

[[nodiscard]] Point2 tangentRelaxedOutward(const Point2 tangent, const Point2 outward,
                                           const double angle_rad) noexcept {
  const double angle = std::clamp(angle_rad, 0.0, std::numbers::pi / 2.0);
  const Point2 relaxed = tangent * std::cos(angle) + outward * std::sin(angle);
  const Point2 result = normalized(relaxed);
  if (!(norm(result) > kTinyDistanceM)) {
    return tangent;
  }
  return result;
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
    sample.lateral_offset_m = dot(point - corridor->center, corridor->normal);
  }
  return sample;
}

[[nodiscard]] std::vector<TrajectoryPointSample>
buildBezierSamples(const std::span<const TrajectoryPointSample> samples,
                   const std::span<const CorridorSample> corridor_samples,
                   const std::size_t entry_index, const std::size_t corner_index,
                   const std::size_t exit_index, const CornerCandidate& corner,
                   const TurnSmoothingConfig& config, const double outward_shift_scale,
                   const double relaxed_angle_rad, double& applied_shift_m) {
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
  const double shift_scale = std::clamp(outward_shift_scale, 0.0, 1.0);
  const double entry_shift =
      outwardShiftFor(samples[entry_index], entry_outward, config) * shift_scale;
  const double exit_shift =
      outwardShiftFor(samples[exit_index], exit_outward, config) * shift_scale;
  applied_shift_m = std::max(entry_shift, exit_shift);

  const double local_length = pathLength(samples, entry_index, exit_index);
  const double control_distance =
      std::clamp(local_length * 0.35, 2.0, std::max(2.0, local_length * 0.6));
  const Point2 entry_tangent =
      tangentRelaxedOutward(incoming, entry_outward, relaxed_angle_rad);
  const Point2 exit_tangent =
      tangentRelaxedOutward(outgoing, exit_outward, relaxed_angle_rad);
  const Point2 p1 = p0 + entry_tangent * control_distance + entry_outward * entry_shift;
  const Point2 p2 = p3 - exit_tangent * control_distance + exit_outward * exit_shift;
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

void cachedBezierSamples(const std::span<const TrajectoryPointSample> samples,
                         const std::span<const CorridorSample> corridor_samples,
                         const std::size_t entry_index, const std::size_t corner_index,
                         const std::size_t exit_index, const CornerCandidate& corner,
                         const TurnSmoothingConfig& config,
                         const double outward_shift_scale,
                         const double relaxed_angle_rad, double& applied_shift_m,
                         TurnSmoothingWorkBuffer& buffer, TurnSmoothingStats& stats) {
  const BezierCacheKey key{
      .corner_index = corner_index,
      .entry_index = entry_index,
      .exit_index = exit_index,
      .shift_scale = quantizedCacheValue(outward_shift_scale),
      .relaxed_angle_rad = quantizedCacheValue(relaxed_angle_rad),
      .sample_step_m = quantizedCacheValue(
          sanitizedPositive(config.sample_step_m, 1.0, 0.1, 10000.0)),
  };
  if (const auto iter = buffer.bezier_cache.find(key);
      iter != buffer.bezier_cache.end()) {
    ++stats.bezier_cache_hits;
    buffer.replacement = iter->second;
    applied_shift_m = 0.0;
    if (!buffer.replacement.empty()) {
      const Point2 entry_outward =
          leftNormal(normalized(samples[corner_index].point -
                                samples[corner_index - 1U].point)) *
          -corner.turn_sign;
      const Point2 exit_outward =
          leftNormal(normalized(samples[corner_index + 1U].point -
                                samples[corner_index].point)) *
          -corner.turn_sign;
      const double shift_scale = std::clamp(outward_shift_scale, 0.0, 1.0);
      applied_shift_m =
          std::max(outwardShiftFor(samples[entry_index], entry_outward, config),
                   outwardShiftFor(samples[exit_index], exit_outward, config)) *
          shift_scale;
    }
    return;
  }

  ++stats.bezier_cache_misses;
  const auto build_started_at = std::chrono::steady_clock::now();
  buffer.replacement = buildBezierSamples(
      samples, corridor_samples, entry_index, corner_index, exit_index, corner, config,
      outward_shift_scale, relaxed_angle_rad, applied_shift_m);
  stats.candidate_build_duration_ms += elapsedMilliseconds(build_started_at);
  buffer.bezier_cache.emplace(key, buffer.replacement);
}

[[nodiscard]] SmoothingAttempt
trySmoothCorner(const std::span<const TrajectoryPointSample> samples,
                const std::span<const CorridorSample> corridor_samples,
                const OccupancyGrid2D& prohibited_grid, const CornerCandidate& corner,
                const TurnSmoothingConfig& config, const double entry_distance_m,
                const double exit_distance_m, const double outward_shift_scale,
                const double relaxed_angle_rad,
                const VelocityFollowerConfig& speed_config,
                TurnSmoothingWorkBuffer& buffer, TurnSmoothingStats& stats) {
  SmoothingAttempt attempt{};
  attempt.entry_distance_m = entry_distance_m;
  attempt.exit_distance_m = exit_distance_m;
  attempt.shift_scale = outward_shift_scale;
  attempt.relaxed_angle_rad = relaxed_angle_rad;
  const std::size_t entry_index =
      findEntryIndex(samples, corner.index, entry_distance_m);
  const std::size_t exit_index = findExitIndex(samples, corner.index, exit_distance_m);
  attempt.entry_index = entry_index;
  attempt.exit_index = exit_index;
  if (entry_index >= corner.index || exit_index <= corner.index) {
    attempt.reject_reason = SmoothingRejectReason::kNotImproved;
    attempt.reject_detail = "invalid_window";
    return attempt;
  }
  attempt.before_metrics = cachedBeforeMetrics(samples, entry_index, exit_index,
                                               speed_config, buffer, stats);

  cachedBezierSamples(samples, corridor_samples, entry_index, corner.index, exit_index,
                      corner, config, outward_shift_scale, relaxed_angle_rad,
                      attempt.applied_shift_m, buffer, stats);
  attempt.replacement_sample_count = buffer.replacement.size();
  if (buffer.replacement.size() < 3U) {
    attempt.reject_reason = SmoothingRejectReason::kNotImproved;
    attempt.reject_detail = "replacement_too_short";
    return attempt;
  }

  const auto replace_started_at = std::chrono::steady_clock::now();
  replaceRangeInto(samples, entry_index, exit_index, buffer.replacement,
                   buffer.candidate);
  stats.candidate_replace_duration_ms += elapsedMilliseconds(replace_started_at);
  sampleRangeInto(buffer.candidate, entry_index,
                  entry_index + buffer.replacement.size() - 1U, buffer.after_local);
  attempt.after_metrics =
      localTrajectoryMetrics(buffer.after_local, speed_config, &stats);
  attempt.score = smoothingAttemptScore(attempt.after_metrics);
  const auto collision_started_at = std::chrono::steady_clock::now();
  const bool traversable = pathIsTraversableCached(prohibited_grid, buffer.candidate,
                                                   buffer.traversability_cache, stats);
  stats.collision_check_duration_ms += elapsedMilliseconds(collision_started_at);
  if (!traversable) {
    attempt.reject_reason = SmoothingRejectReason::kProhibited;
    attempt.reject_detail = "prohibited_intersection";
    return attempt;
  }

  const auto shape_started_at = std::chrono::steady_clock::now();
  const TrajectoryShapeDiagnostics before_shape =
      computeTrajectoryShapeDiagnostics(samples);
  const TrajectoryShapeDiagnostics after_shape =
      computeTrajectoryShapeDiagnostics(buffer.candidate);
  stats.shape_diagnostics_duration_ms += elapsedMilliseconds(shape_started_at);
  const char* const shape_reject_detail =
      shapeImprovementRejectDetail(before_shape, after_shape, config);
  if (std::string_view{shape_reject_detail} != "none") {
    attempt.reject_reason = SmoothingRejectReason::kNotImproved;
    attempt.reject_detail = shape_reject_detail;
    return attempt;
  }
  const SmoothingRejectReason regression_reason =
      candidateRegressionReason(attempt.before_metrics, attempt.after_metrics);
  if (regression_reason != SmoothingRejectReason::kNone) {
    attempt.reject_reason = regression_reason;
    attempt.reject_detail = regressionRejectDetail(regression_reason);
    return attempt;
  }

  attempt.samples = buffer.candidate;
  attempt.reject_reason = SmoothingRejectReason::kNone;
  attempt.reject_detail = "none";
  attempt.accepted = true;
  return attempt;
}

} // namespace drone_city_nav::turn_smoothing_detail
