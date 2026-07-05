#include "drone_city_nav/trajectory_speed_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] double sanitizedCruiseSpeed(const VelocityFollowerConfig& config) {
  return sanitizedPositive(config.cruise_speed_mps, 12.0, 0.0, 100.0);
}

[[nodiscard]] double sanitizedMinTurnSpeed(const VelocityFollowerConfig& config) {
  return std::min(sanitizedPositive(config.min_turn_speed_mps, 2.0, 0.0, 100.0),
                  sanitizedCruiseSpeed(config));
}

[[nodiscard]] double
effectiveVelocityDeltaAccelMps2(const VelocityFollowerConfig& config) {
  return sanitizedPositive(config.max_accel_mps2, 3.0, 1.0e-6, 100.0);
}

[[nodiscard]] double
effectiveVelocityDeltaDecelMps2(const VelocityFollowerConfig& config) {
  return sanitizedPositive(config.max_decel_mps2, 4.0, 1.0e-6, 100.0);
}

[[nodiscard]] double
effectiveSpeedProfileDecelMps2(const VelocityFollowerConfig& config) {
  const double fallback = effectiveVelocityDeltaDecelMps2(config);
  return sanitizedPositive(config.speed_profile_decel_mps2, fallback, 1.0e-6, 100.0);
}

[[nodiscard]] double segmentEndS(const TrajectorySegment& segment) noexcept {
  return segment.s_start_m + std::max(0.0, segment.length_m);
}

[[nodiscard]] std::size_t
segmentIndexForS(const std::span<const TrajectorySegment> trajectory,
                 const double s_m) {
  if (trajectory.empty()) {
    return 0U;
  }
  for (std::size_t i = 0U; i < trajectory.size(); ++i) {
    if (s_m <= segmentEndS(trajectory[i])) {
      return i;
    }
  }
  return trajectory.size() - 1U;
}

[[nodiscard]] double segmentCurvature(const TrajectorySegment& segment) noexcept {
  if (segment.kind != TrajectorySegmentKind::kArc ||
      !(segment.radius_m > kTinyDistanceM)) {
    return 0.0;
  }
  return (segment.sweep_rad >= 0.0 ? 1.0 : -1.0) / segment.radius_m;
}

[[nodiscard]] TrajectorySpeedSample
geometricSpeedSampleForSegment(const std::span<const TrajectorySegment> trajectory,
                               const std::size_t requested_segment_index,
                               const double requested_s_m,
                               const VelocityFollowerConfig& config) {
  TrajectorySpeedSample sample{};
  if (trajectory.empty()) {
    return sample;
  }
  const double cruise_speed = sanitizedCruiseSpeed(config);
  const double min_turn_speed = sanitizedMinTurnSpeed(config);
  const double max_lateral_accel =
      sanitizedPositive(config.max_lateral_accel_mps2, 3.0, 1.0e-6, 100.0);

  const double clamped_s =
      std::clamp(requested_s_m, 0.0, trajectoryLengthM(trajectory));
  const std::size_t segment_index =
      std::min(requested_segment_index, trajectory.size() - 1U);
  const TrajectorySegment& segment = trajectory[segment_index];
  sample.s_m = clamped_s;
  sample.segment_index = segment_index;
  sample.curvature_1pm = segmentCurvature(segment);
  sample.radius_m = std::numeric_limits<double>::quiet_NaN();
  sample.reason = SpeedConstraintType::kNone;
  sample.geometric_limit_mps = cruise_speed;
  if (std::abs(sample.curvature_1pm) > kTinyDistanceM) {
    sample.radius_m = 1.0 / std::abs(sample.curvature_1pm);
    sample.reason = SpeedConstraintType::kArc;
    sample.geometric_limit_mps = std::clamp(
        std::sqrt(max_lateral_accel * sample.radius_m), min_turn_speed, cruise_speed);
  }

  sample.profiled_limit_mps = sample.geometric_limit_mps;
  sample.constraint_s_m = sample.s_m;
  sample.constraint_limit_mps = sample.geometric_limit_mps;
  return sample;
}

[[nodiscard]] TrajectorySpeedSample
geometricSpeedSampleAtS(const std::span<const TrajectorySegment> trajectory,
                        const double s_m, const VelocityFollowerConfig& config) {
  return geometricSpeedSampleForSegment(trajectory, segmentIndexForS(trajectory, s_m),
                                        s_m, config);
}

[[nodiscard]] TrajectorySpeedSample
geometricSpeedSampleFromPointSample(const TrajectoryPointSample& point_sample,
                                    const std::size_t sample_index,
                                    const VelocityFollowerConfig& config) {
  TrajectorySpeedSample sample{};
  const double cruise_speed = sanitizedCruiseSpeed(config);
  const double min_turn_speed = sanitizedMinTurnSpeed(config);
  const double max_lateral_accel =
      sanitizedPositive(config.max_lateral_accel_mps2, 3.0, 1.0e-6, 100.0);
  sample.s_m = point_sample.s_m;
  sample.segment_index = sample_index;
  sample.curvature_1pm = point_sample.curvature_1pm;
  sample.radius_m = std::numeric_limits<double>::quiet_NaN();
  sample.reason = SpeedConstraintType::kNone;
  sample.geometric_limit_mps = cruise_speed;
  if (std::abs(sample.curvature_1pm) > kTinyDistanceM) {
    sample.radius_m = 1.0 / std::abs(sample.curvature_1pm);
    sample.reason = SpeedConstraintType::kArc;
    sample.geometric_limit_mps = std::clamp(
        std::sqrt(max_lateral_accel * sample.radius_m), min_turn_speed, cruise_speed);
  }
  sample.profiled_limit_mps = sample.geometric_limit_mps;
  sample.constraint_s_m = sample.s_m;
  sample.constraint_limit_mps = sample.geometric_limit_mps;
  return sample;
}

void mergeSpeedSample(std::vector<TrajectorySpeedSample>& samples,
                      const TrajectorySpeedSample& candidate) {
  constexpr double kMergeToleranceM = 1.0e-6;
  if (samples.empty() ||
      std::abs(samples.back().s_m - candidate.s_m) > kMergeToleranceM) {
    samples.push_back(candidate);
    return;
  }
  TrajectorySpeedSample& existing = samples.back();
  if (candidate.geometric_limit_mps < existing.geometric_limit_mps) {
    existing.geometric_limit_mps = candidate.geometric_limit_mps;
    existing.profiled_limit_mps = candidate.profiled_limit_mps;
    existing.reason = candidate.reason;
    existing.segment_index = candidate.segment_index;
    existing.curvature_1pm = candidate.curvature_1pm;
    existing.radius_m = candidate.radius_m;
    existing.constraint_s_m = candidate.constraint_s_m;
    existing.constraint_limit_mps = candidate.constraint_limit_mps;
  }
}

[[nodiscard]] double limitedSpeedForDistance(const double next_speed,
                                             const double distance_m,
                                             const double acceleration_mps2) {
  return std::sqrt(
      std::max(0.0, next_speed * next_speed +
                        2.0 * acceleration_mps2 * std::max(0.0, distance_m)));
}

[[nodiscard]] double interpolateSpeed(const double start_speed_mps,
                                      const double end_speed_mps,
                                      const double ratio) noexcept {
  if (!std::isfinite(start_speed_mps) || !std::isfinite(end_speed_mps)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double start_sq = start_speed_mps * start_speed_mps;
  const double end_sq = end_speed_mps * end_speed_mps;
  return std::sqrt(std::max(0.0, start_sq + (end_sq - start_sq) * ratio));
}

[[nodiscard]] std::size_t
firstProfileSampleNotBefore(const TrajectorySpeedProfile& profile, const double s_m) {
  if (profile.samples.empty()) {
    return 0U;
  }
  const auto it = std::ranges::lower_bound(
      profile.samples, s_m, {},
      [](const TrajectorySpeedSample& sample) { return sample.s_m; });
  if (it == profile.samples.end()) {
    return profile.samples.size() - 1U;
  }
  return static_cast<std::size_t>(std::distance(profile.samples.begin(), it));
}

[[nodiscard]] TrajectorySpeedSample
interpolateProfileSample(const TrajectorySpeedSample& start,
                         const TrajectorySpeedSample& end, const double s_m) {
  const double ds = end.s_m - start.s_m;
  if (!(ds > kTinyDistanceM)) {
    return end;
  }
  const double ratio = std::clamp((s_m - start.s_m) / ds, 0.0, 1.0);
  const TrajectorySpeedSample& limiting =
      start.profiled_limit_mps <= end.profiled_limit_mps ? start : end;

  TrajectorySpeedSample sample = limiting;
  sample.s_m = std::clamp(s_m, start.s_m, end.s_m);
  sample.geometric_limit_mps =
      interpolateSpeed(start.geometric_limit_mps, end.geometric_limit_mps, ratio);
  sample.profiled_limit_mps =
      interpolateSpeed(start.profiled_limit_mps, end.profiled_limit_mps, ratio);
  sample.curvature_1pm =
      start.curvature_1pm + (end.curvature_1pm - start.curvature_1pm) * ratio;
  sample.radius_m = std::abs(sample.curvature_1pm) > kTinyDistanceM
                        ? 1.0 / std::abs(sample.curvature_1pm)
                        : std::numeric_limits<double>::quiet_NaN();
  return sample;
}

void finalizeSpeedProfile(TrajectorySpeedProfile& profile,
                          const VelocityFollowerConfig& config) {
  if (profile.samples.empty()) {
    return;
  }
  profile.samples.back().profiled_limit_mps = 0.0;
  profile.samples.back().geometric_limit_mps = 0.0;
  profile.samples.back().reason = SpeedConstraintType::kGoal;
  profile.samples.back().constraint_s_m = profile.samples.back().s_m;
  profile.samples.back().constraint_limit_mps = 0.0;

  const double max_decel = effectiveSpeedProfileDecelMps2(config);
  for (std::size_t i = profile.samples.size() - 1U; i > 0U; --i) {
    const double ds = profile.samples[i].s_m - profile.samples[i - 1U].s_m;
    const double allowed =
        limitedSpeedForDistance(profile.samples[i].profiled_limit_mps, ds, max_decel);
    if (allowed < profile.samples[i - 1U].profiled_limit_mps) {
      profile.samples[i - 1U].profiled_limit_mps = allowed;
      profile.samples[i - 1U].reason = profile.samples[i].reason;
      profile.samples[i - 1U].segment_index = profile.samples[i].segment_index;
      profile.samples[i - 1U].curvature_1pm = profile.samples[i].curvature_1pm;
      profile.samples[i - 1U].radius_m = profile.samples[i].radius_m;
      profile.samples[i - 1U].constraint_s_m = profile.samples[i].constraint_s_m;
      profile.samples[i - 1U].constraint_limit_mps =
          profile.samples[i].constraint_limit_mps;
    }
  }

  const double max_accel = effectiveVelocityDeltaAccelMps2(config);
  for (std::size_t i = 1U; i < profile.samples.size(); ++i) {
    const double ds = profile.samples[i].s_m - profile.samples[i - 1U].s_m;
    const double allowed = limitedSpeedForDistance(
        profile.samples[i - 1U].profiled_limit_mps, ds, max_accel);
    if (allowed < profile.samples[i].profiled_limit_mps) {
      profile.samples[i].profiled_limit_mps = allowed;
    }
  }

  profile.valid = true;
}

[[nodiscard]] double sanitizedNonNegative(const double value,
                                          const double fallback) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::max(0.0, value);
}

[[nodiscard]] TrajectorySpeedSample
minProfileSampleInRange(const TrajectorySpeedProfile& profile, const double start_s_m,
                        const double end_s_m,
                        const TrajectorySpeedSample& fallback_sample) {
  const std::size_t start_index = firstProfileSampleNotBefore(profile, start_s_m);
  TrajectorySpeedSample limiting_sample = fallback_sample;
  const double bounded_end_s = std::max(start_s_m, end_s_m);
  for (std::size_t i = start_index; i < profile.samples.size(); ++i) {
    const TrajectorySpeedSample& sample = profile.samples[i];
    if (sample.s_m > bounded_end_s) {
      break;
    }
    if (sample.reason == SpeedConstraintType::kGoal) {
      continue;
    }
    if (sample.profiled_limit_mps < limiting_sample.profiled_limit_mps) {
      limiting_sample = sample;
    }
  }
  return limiting_sample;
}

[[nodiscard]] bool sameConstraint(const SpeedProfileConstraintDiagnostic& lhs,
                                  const TrajectorySpeedSample& rhs) noexcept {
  constexpr double kConstraintMergeToleranceM = 0.25;
  if (lhs.source != rhs.reason) {
    return false;
  }
  if (!std::isfinite(lhs.s_m) || !std::isfinite(rhs.constraint_s_m)) {
    return lhs.sample_index == rhs.segment_index;
  }
  return std::abs(lhs.s_m - rhs.constraint_s_m) <= kConstraintMergeToleranceM;
}

[[nodiscard]] bool
isIsolatedCurvatureSpike(const std::span<const TrajectorySpeedSample> samples,
                         const std::size_t index) noexcept {
  if (index == 0U || index + 1U >= samples.size()) {
    return false;
  }
  constexpr double kMinimumSpikeCurvature = 0.05;
  constexpr double kNeighborRatio = 0.45;
  const double current = std::abs(samples[index].curvature_1pm);
  if (!(current >= kMinimumSpikeCurvature)) {
    return false;
  }
  const double previous = std::abs(samples[index - 1U].curvature_1pm);
  const double next = std::abs(samples[index + 1U].curvature_1pm);
  return previous <= current * kNeighborRatio && next <= current * kNeighborRatio;
}

} // namespace

const char*
speedConstraintTypeName(const SpeedConstraintType constraint_type) noexcept {
  switch (constraint_type) {
    case SpeedConstraintType::kNone:
      return "none";
    case SpeedConstraintType::kArc:
      return "arc";
    case SpeedConstraintType::kGoal:
      return "goal";
  }
  return "unknown";
}

double distanceFromTrajectorySToEnd(const std::span<const TrajectorySegment> trajectory,
                                    const double s_m) {
  return std::max(0.0, trajectoryLengthM(trajectory) - std::max(0.0, s_m));
}

TrajectorySpeedProfile
buildTrajectorySpeedProfile(const std::span<const TrajectorySegment> trajectory,
                            const VelocityFollowerConfig& config) {
  TrajectorySpeedProfile profile{};
  if (!trajectoryIsUsable(trajectory)) {
    return profile;
  }

  const double length_m = trajectoryLengthM(trajectory);
  const double step_m =
      sanitizedPositive(config.speed_profile_sample_step_m, 1.0, 0.1, 100.0);
  const std::size_t regular_sample_count =
      static_cast<std::size_t>(std::ceil(length_m / step_m));
  std::vector<TrajectorySpeedSample> candidates;
  candidates.reserve(regular_sample_count + trajectory.size() * 3U + 2U);
  for (std::size_t i = 0U; i <= regular_sample_count; ++i) {
    const double s_m = std::min(length_m, static_cast<double>(i) * step_m);
    candidates.push_back(geometricSpeedSampleAtS(trajectory, s_m, config));
  }
  for (std::size_t i = 0U; i < trajectory.size(); ++i) {
    const TrajectorySegment& segment = trajectory[i];
    candidates.push_back(
        geometricSpeedSampleForSegment(trajectory, i, segment.s_start_m, config));
    candidates.push_back(
        geometricSpeedSampleForSegment(trajectory, i, segmentEndS(segment), config));
    if (segment.kind == TrajectorySegmentKind::kArc) {
      candidates.push_back(geometricSpeedSampleForSegment(
          trajectory, i, segment.s_start_m + segment.length_m * 0.5, config));
    }
  }

  std::ranges::sort(candidates, {}, &TrajectorySpeedSample::s_m);
  profile.samples.reserve(candidates.size());
  for (const TrajectorySpeedSample& candidate : candidates) {
    mergeSpeedSample(profile.samples, candidate);
  }

  if (profile.samples.empty()) {
    return profile;
  }
  finalizeSpeedProfile(profile, config);
  return profile;
}

TrajectorySpeedProfile buildTrajectorySpeedProfile(
    const std::span<const TrajectoryPointSample> trajectory_samples,
    const VelocityFollowerConfig& config) {
  TrajectorySpeedProfile profile{};
  if (!trajectorySamplesAreUsable(trajectory_samples)) {
    return profile;
  }

  profile.samples.reserve(trajectory_samples.size());
  for (std::size_t i = 0U; i < trajectory_samples.size(); ++i) {
    mergeSpeedSample(profile.samples, geometricSpeedSampleFromPointSample(
                                          trajectory_samples[i], i, config));
  }
  finalizeSpeedProfile(profile, config);
  return profile;
}

TrajectorySpeedSample speedProfileSampleAtS(const TrajectorySpeedProfile& profile,
                                            const double s_m) {
  if (!profile.valid || profile.samples.empty() || !std::isfinite(s_m)) {
    return TrajectorySpeedSample{};
  }
  const std::size_t end_index = firstProfileSampleNotBefore(profile, s_m);
  if (end_index == 0U) {
    return profile.samples.front();
  }
  if (end_index >= profile.samples.size()) {
    return profile.samples.back();
  }
  const TrajectorySpeedSample& end = profile.samples[end_index];
  if (s_m >= end.s_m) {
    return end;
  }
  const TrajectorySpeedSample& start = profile.samples[end_index - 1U];
  return interpolateProfileSample(start, end, s_m);
}

std::vector<SpeedProfileConstraintDiagnostic>
topSpeedProfileConstraints(const TrajectorySpeedProfile& profile,
                           const std::size_t max_constraints) {
  std::vector<SpeedProfileConstraintDiagnostic> constraints;
  if (!profile.valid || profile.samples.empty() || max_constraints == 0U) {
    return constraints;
  }

  for (const TrajectorySpeedSample& sample : profile.samples) {
    if (sample.reason != SpeedConstraintType::kArc ||
        !std::isfinite(sample.constraint_limit_mps)) {
      continue;
    }

    auto existing = std::ranges::find_if(
        constraints, [&sample](const SpeedProfileConstraintDiagnostic& diagnostic) {
          return sameConstraint(diagnostic, sample);
        });
    if (existing == constraints.end()) {
      SpeedProfileConstraintDiagnostic diagnostic{};
      diagnostic.sample_index = sample.segment_index;
      diagnostic.s_m = sample.constraint_s_m;
      diagnostic.radius_m = sample.radius_m;
      diagnostic.curvature_1pm = sample.curvature_1pm;
      diagnostic.speed_limit_mps = sample.constraint_limit_mps;
      diagnostic.profiled_limit_mps = sample.profiled_limit_mps;
      diagnostic.source = sample.reason;
      if (sample.segment_index < profile.samples.size()) {
        diagnostic.isolated_curvature_spike =
            isIsolatedCurvatureSpike(profile.samples, sample.segment_index);
      }
      constraints.push_back(diagnostic);
      continue;
    }

    const bool lower_limit = sample.constraint_limit_mps < existing->speed_limit_mps;
    const bool stronger_curvature =
        sample.constraint_limit_mps == existing->speed_limit_mps &&
        std::abs(sample.curvature_1pm) > std::abs(existing->curvature_1pm);
    if (lower_limit || stronger_curvature) {
      existing->sample_index = sample.segment_index;
      existing->s_m = sample.constraint_s_m;
      existing->radius_m = sample.radius_m;
      existing->curvature_1pm = sample.curvature_1pm;
      existing->speed_limit_mps = sample.constraint_limit_mps;
      existing->profiled_limit_mps = sample.profiled_limit_mps;
      existing->source = sample.reason;
      existing->isolated_curvature_spike =
          sample.segment_index < profile.samples.size()
              ? isIsolatedCurvatureSpike(profile.samples, sample.segment_index)
              : false;
    } else {
      existing->profiled_limit_mps =
          std::min(existing->profiled_limit_mps, sample.profiled_limit_mps);
    }
  }

  std::ranges::sort(constraints, [](const SpeedProfileConstraintDiagnostic& lhs,
                                    const SpeedProfileConstraintDiagnostic& rhs) {
    if (lhs.speed_limit_mps != rhs.speed_limit_mps) {
      return lhs.speed_limit_mps < rhs.speed_limit_mps;
    }
    return lhs.s_m < rhs.s_m;
  });
  if (constraints.size() > max_constraints) {
    constraints.resize(max_constraints);
  }
  return constraints;
}

ScalarSpeedPlan planScalarSpeed(const TrajectorySpeedProfile& profile,
                                const ScalarSpeedQuery& query,
                                const VelocityFollowerConfig& config) {
  ScalarSpeedPlan plan{};
  if (!profile.valid || profile.samples.empty() ||
      !std::isfinite(query.trajectory_s_m) || !std::isfinite(query.dt_s)) {
    return plan;
  }

  const double cruise_speed = sanitizedCruiseSpeed(config);
  const TrajectorySpeedSample current_sample =
      speedProfileSampleAtS(profile, query.trajectory_s_m);
  plan.valid = true;
  plan.constraint_type = current_sample.reason;
  plan.constraint_index = current_sample.segment_index;
  plan.profile_speed_limit_mps =
      std::clamp(current_sample.profiled_limit_mps, 0.0, cruise_speed);
  plan.limiting_constraint_distance_m =
      current_sample.reason == SpeedConstraintType::kNone
          ? std::numeric_limits<double>::quiet_NaN()
          : std::max(0.0, current_sample.constraint_s_m - query.trajectory_s_m);
  plan.limiting_constraint_speed_mps = current_sample.constraint_limit_mps;
  plan.limiting_allowed_speed_now_mps = current_sample.profiled_limit_mps;
  plan.limiting_curve_radius_m = current_sample.radius_m;
  plan.limiting_curvature_1pm = current_sample.curvature_1pm;

  const double current_speed =
      sanitizedNonNegative(query.current_speed_mps,
                           sanitizedNonNegative(query.previous_command_speed_mps, 0.0));
  const double lookahead_time =
      sanitizedPositive(config.speed_profile_lookahead_time_s, 1.0, 0.0, 30.0);
  const double lookahead_min =
      sanitizedPositive(config.speed_profile_lookahead_min_m, 5.0, 0.0, 500.0);
  const double requested_lookahead_max =
      sanitizedPositive(config.speed_profile_lookahead_max_m, 35.0, 0.0, 5000.0);
  const double lookahead_max = std::max(requested_lookahead_max, lookahead_min);
  plan.lookahead_distance_m =
      std::clamp(current_speed * lookahead_time, lookahead_min, lookahead_max);

  const TrajectorySpeedSample lookahead_sample = minProfileSampleInRange(
      profile, query.trajectory_s_m, query.trajectory_s_m + plan.lookahead_distance_m,
      current_sample);
  plan.lookahead_speed_limit_mps =
      std::clamp(lookahead_sample.profiled_limit_mps, 0.0, cruise_speed);
  plan.lookahead_constraint_type = lookahead_sample.reason;
  plan.lookahead_constraint_index = lookahead_sample.segment_index;
  plan.lookahead_constraint_distance_m =
      lookahead_sample.reason == SpeedConstraintType::kNone
          ? std::numeric_limits<double>::quiet_NaN()
          : std::max(0.0, lookahead_sample.constraint_s_m - query.trajectory_s_m);

  plan.speed_after_lookahead_mps =
      std::min(plan.profile_speed_limit_mps, plan.lookahead_speed_limit_mps);
  if (plan.lookahead_speed_limit_mps + 1.0e-9 < plan.profile_speed_limit_mps) {
    plan.constraint_type = lookahead_sample.reason;
    plan.constraint_index = lookahead_sample.segment_index;
    plan.limiting_constraint_distance_m =
        lookahead_sample.reason == SpeedConstraintType::kNone
            ? std::numeric_limits<double>::quiet_NaN()
            : std::max(0.0, lookahead_sample.constraint_s_m - query.trajectory_s_m);
    plan.limiting_constraint_speed_mps = lookahead_sample.constraint_limit_mps;
    plan.limiting_allowed_speed_now_mps = lookahead_sample.profiled_limit_mps;
    plan.limiting_curve_radius_m = lookahead_sample.radius_m;
    plan.limiting_curvature_1pm = lookahead_sample.curvature_1pm;
  }
  const double previous_speed =
      sanitizedNonNegative(query.previous_command_speed_mps, current_speed);
  const double dt = sanitizedPositive(query.dt_s, 0.1, 0.0, 10.0);
  const double max_accel = effectiveVelocityDeltaAccelMps2(config);
  const double max_decel = std::min(effectiveVelocityDeltaDecelMps2(config),
                                    effectiveSpeedProfileDecelMps2(config));
  const double max_speed_delta =
      (plan.speed_after_lookahead_mps >= previous_speed ? max_accel : max_decel) * dt;
  plan.accel_limited_speed_mps =
      previous_speed + std::clamp(plan.speed_after_lookahead_mps - previous_speed,
                                  -max_speed_delta, max_speed_delta);
  plan.final_scalar_speed_mps = plan.accel_limited_speed_mps;
  return plan;
}

TraversalTimeEstimate
estimateTraversalTime(const std::span<const TrajectoryPointSample> trajectory_samples,
                      const VelocityFollowerConfig& config,
                      const bool use_forward_backward_profile) {
  TraversalTimeEstimate estimate{};
  if (!trajectorySamplesAreUsable(trajectory_samples)) {
    return estimate;
  }

  TrajectorySpeedProfile profile{};
  if (use_forward_backward_profile) {
    profile = buildTrajectorySpeedProfile(trajectory_samples, config);
  } else {
    profile.samples.reserve(trajectory_samples.size());
    for (std::size_t i = 0U; i < trajectory_samples.size(); ++i) {
      mergeSpeedSample(profile.samples, geometricSpeedSampleFromPointSample(
                                            trajectory_samples[i], i, config));
    }
    profile.valid = !profile.samples.empty();
  }
  if (!profile.valid || profile.samples.empty()) {
    return estimate;
  }

  constexpr double kMinimumIntegrationSpeedMps = 0.1;
  estimate.valid = true;
  estimate.estimated_time_s = 0.0;
  for (const TrajectorySpeedSample& sample : profile.samples) {
    const double speed_limit = use_forward_backward_profile
                                   ? sample.profiled_limit_mps
                                   : sample.geometric_limit_mps;
    if (std::isfinite(speed_limit)) {
      if (!std::isfinite(estimate.min_speed_limit_mps)) {
        estimate.min_speed_limit_mps = speed_limit;
        estimate.max_speed_limit_mps = speed_limit;
      } else {
        estimate.min_speed_limit_mps =
            std::min(estimate.min_speed_limit_mps, speed_limit);
        estimate.max_speed_limit_mps =
            std::max(estimate.max_speed_limit_mps, speed_limit);
      }
    }
    if (sample.reason == SpeedConstraintType::kArc) {
      ++estimate.curvature_limited_samples;
    }
  }

  for (std::size_t i = 1U; i < trajectory_samples.size(); ++i) {
    double ds = trajectory_samples[i].s_m - trajectory_samples[i - 1U].s_m;
    if (!(ds > kTinyDistanceM) || !std::isfinite(ds)) {
      ds = distance(trajectory_samples[i - 1U].point, trajectory_samples[i].point);
    }
    if (!(ds > kTinyDistanceM) || !std::isfinite(ds)) {
      continue;
    }
    const TrajectorySpeedSample start =
        speedProfileSampleAtS(profile, trajectory_samples[i - 1U].s_m);
    const TrajectorySpeedSample end =
        speedProfileSampleAtS(profile, trajectory_samples[i].s_m);
    const double start_speed = use_forward_backward_profile ? start.profiled_limit_mps
                                                            : start.geometric_limit_mps;
    const double end_speed =
        use_forward_backward_profile ? end.profiled_limit_mps : end.geometric_limit_mps;
    const double average_speed =
        std::max(kMinimumIntegrationSpeedMps, 0.5 * (start_speed + end_speed));
    if (std::isfinite(average_speed)) {
      estimate.estimated_time_s += ds / average_speed;
    }
  }
  return estimate;
}

} // namespace drone_city_nav
