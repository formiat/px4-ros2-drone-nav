#include "drone_city_nav/safe_trajectory_truncation.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;
constexpr double kMinimumExecutablePrefixM = 0.5;
constexpr std::uint64_t kFnvOffsetBasis = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

[[nodiscard]] Point2 interpolate(const Point2 start, const Point2 end,
                                 const double ratio) noexcept {
  return Point2{start.x * (1.0 - ratio) + end.x * ratio,
                start.y * (1.0 - ratio) + end.y * ratio};
}

[[nodiscard]] Point2 normalized(const Point2 value) noexcept {
  const double norm = std::hypot(value.x, value.y);
  return norm > kTinyDistanceM ? Point2{value.x / norm, value.y / norm} : Point2{};
}

[[nodiscard]] TrajectoryPointSample
sampleAtS(const std::span<const TrajectoryPointSample> samples, const double s_m) {
  const double bounded_s_m = std::clamp(s_m, 0.0, samples.back().s_m);
  for (std::size_t index = 0U; index + 1U < samples.size(); ++index) {
    const TrajectoryPointSample& start = samples[index];
    const TrajectoryPointSample& end = samples[index + 1U];
    if (bounded_s_m > end.s_m && index + 2U < samples.size()) {
      continue;
    }
    const double station_delta_m = end.s_m - start.s_m;
    const double ratio =
        station_delta_m > kTinyDistanceM
            ? std::clamp((bounded_s_m - start.s_m) / station_delta_m, 0.0, 1.0)
            : 0.0;
    TrajectoryPointSample sample = ratio <= 0.5 ? start : end;
    sample.s_m = bounded_s_m;
    sample.point = interpolate(start.point, end.point, ratio);
    sample.tangent = normalized(interpolate(start.tangent, end.tangent, ratio));
    sample.curvature_1pm =
        start.curvature_1pm * (1.0 - ratio) + end.curvature_1pm * ratio;
    sample.z_m = start.z_m * (1.0 - ratio) + end.z_m * ratio;
    sample.vertical_slope_dz_ds =
        start.vertical_slope_dz_ds * (1.0 - ratio) + end.vertical_slope_dz_ds * ratio;
    return sample;
  }
  return samples.back();
}

void appendDistinct(std::vector<TrajectoryPointSample>& output,
                    TrajectoryPointSample sample) {
  if (!output.empty() &&
      distance(output.back().point, sample.point) <= kTinyDistanceM) {
    output.back() = std::move(sample);
    return;
  }
  output.push_back(std::move(sample));
}

void rebaseStations(std::vector<TrajectoryPointSample>& samples) {
  if (samples.empty()) {
    return;
  }
  samples.front().s_m = 0.0;
  for (std::size_t index = 1U; index < samples.size(); ++index) {
    samples[index].s_m = samples[index - 1U].s_m +
                         distance(samples[index - 1U].point, samples[index].point);
  }
}

void hashValue(std::uint64_t& hash, const double value) noexcept {
  hash ^= std::bit_cast<std::uint64_t>(value);
  hash *= kFnvPrime;
}

} // namespace

SafeTrajectoryTruncationResult
truncateTrajectoryBeforeBlocker(const std::span<const TrajectoryPointSample> samples,
                                const SafeTrajectoryTruncationRequest& request) {
  SafeTrajectoryTruncationResult result{};
  if (!trajectorySamplesAreUsable(samples)) {
    result.reason = "trajectory_invalid";
    return result;
  }
  if (!std::isfinite(request.blocker_path_distance_m) ||
      request.blocker_path_distance_m < 0.0 ||
      !std::isfinite(request.truncation_margin_m) ||
      request.truncation_margin_m < 0.0) {
    result.reason = "request_invalid";
    return result;
  }

  const std::optional<TrajectoryProjection> projection =
      projectOnTrajectorySamples(samples, request.current_position);
  if (!projection.has_value()) {
    result.reason = "projection_unavailable";
    return result;
  }

  result.current_s_m = projection->s_m;
  result.blocker_s_m =
      std::min(samples.back().s_m, projection->s_m + request.blocker_path_distance_m);
  result.stop_s_m = result.blocker_s_m - request.truncation_margin_m;
  if (!(result.stop_s_m > result.current_s_m + kMinimumExecutablePrefixM)) {
    result.applied = true;
    result.immediate_hold = true;
    result.reason = "stop_station_not_ahead";
    return result;
  }

  appendDistinct(result.samples, sampleAtS(samples, result.current_s_m));
  for (const TrajectoryPointSample& sample : samples) {
    if (sample.s_m > result.current_s_m + kTinyDistanceM &&
        sample.s_m < result.stop_s_m - kTinyDistanceM) {
      appendDistinct(result.samples, sample);
    }
  }
  appendDistinct(result.samples, sampleAtS(samples, result.stop_s_m));
  rebaseStations(result.samples);
  populateTrajectorySampleGeometry(result.samples);
  if (!trajectorySamplesAreUsable(result.samples)) {
    result.samples.clear();
    result.reason = "truncated_trajectory_invalid";
    return result;
  }

  result.applied = true;
  result.reason = "safe_prefix";
  return result;
}

std::uint64_t trajectoryPrefixFingerprint(
    const std::span<const TrajectoryPointSample> samples) noexcept {
  std::uint64_t hash = kFnvOffsetBasis;
  hash ^= static_cast<std::uint64_t>(samples.size());
  hash *= kFnvPrime;
  for (const TrajectoryPointSample& sample : samples) {
    hashValue(hash, sample.point.x);
    hashValue(hash, sample.point.y);
    hashValue(hash, sample.z_m);
    hashValue(hash, sample.s_m);
  }
  return hash;
}

TruncatedPrefixStitchResult
stitchTruncatedPrefixWithSuffix(const std::span<const TrajectoryPointSample> prefix,
                                const std::span<const TrajectoryPointSample> suffix,
                                const TruncatedPrefixStitchRequest& request) {
  TruncatedPrefixStitchResult result{};
  if (!trajectorySamplesAreUsable(prefix) || !trajectorySamplesAreUsable(suffix) ||
      !std::isfinite(request.max_join_distance_m) ||
      request.max_join_distance_m < 0.0) {
    result.reason = "invalid_input";
    return result;
  }
  const std::optional<TrajectoryProjection> current_projection =
      projectOnTrajectorySamples(prefix, request.current_position);
  const std::optional<TrajectoryProjection> join_projection =
      projectOnTrajectorySamples(prefix, request.truncation_point);
  if (!current_projection.has_value() || !join_projection.has_value()) {
    result.reason = "prefix_projection_unavailable";
    return result;
  }
  result.current_s_m = current_projection->s_m;
  result.prefix_join_s_m = join_projection->s_m;
  result.join_distance_m = distance(suffix.front().point, request.truncation_point);
  const double prefix_join_error_m = std::sqrt(join_projection->distance_sq);
  if (prefix_join_error_m > request.max_join_distance_m ||
      result.join_distance_m > request.max_join_distance_m) {
    result.reason = "join_point_mismatch";
    return result;
  }
  if (result.current_s_m > result.prefix_join_s_m + kMinimumExecutablePrefixM) {
    result.reason = "truncation_point_passed";
    return result;
  }

  appendDistinct(result.samples, sampleAtS(prefix, result.current_s_m));
  for (const TrajectoryPointSample& sample : prefix) {
    if (sample.s_m > result.current_s_m + kTinyDistanceM &&
        sample.s_m < result.prefix_join_s_m - kTinyDistanceM) {
      appendDistinct(result.samples, sample);
    }
  }
  appendDistinct(result.samples, sampleAtS(prefix, result.prefix_join_s_m));
  rebaseStations(result.samples);
  result.suffix_station_offset_m = result.samples.back().s_m;
  for (const TrajectoryPointSample& sample : suffix) {
    appendDistinct(result.samples, sample);
  }
  rebaseStations(result.samples);
  populateTrajectorySampleGeometry(result.samples);
  if (!trajectorySamplesAreUsable(result.samples)) {
    result.samples.clear();
    result.reason = "stitched_trajectory_invalid";
    return result;
  }
  result.applied = true;
  result.reason = "prefix_suffix_stitched";
  return result;
}

TruncationSuffixJoinValidation
validateTruncationSuffixJoin(const TrajectoryPointSample& prefix_terminal,
                             const TrajectoryPointSample& suffix_initial,
                             const TruncationSuffixJoinRequest& request) noexcept {
  TruncationSuffixJoinValidation result{};
  if (!std::isfinite(request.max_position_jump_m) ||
      request.max_position_jump_m < 0.0 ||
      (request.require_altitude_match && (!std::isfinite(request.max_altitude_jump_m) ||
                                          request.max_altitude_jump_m < 0.0)) ||
      !std::isfinite(prefix_terminal.z_m) || !std::isfinite(suffix_initial.z_m)) {
    result.reason = "invalid_input";
    return result;
  }

  result.position_jump_m = distance(prefix_terminal.point, suffix_initial.point);
  result.altitude_jump_m = std::abs(prefix_terminal.z_m - suffix_initial.z_m);
  if (result.position_jump_m > request.max_position_jump_m) {
    result.reason = "position_mismatch";
    return result;
  }
  if (request.require_tangent_match) {
    if (!std::isfinite(request.max_tangent_jump_rad) ||
        request.max_tangent_jump_rad < 0.0) {
      result.reason = "invalid_input";
      return result;
    }
    const Point2 prefix_tangent = normalized(prefix_terminal.tangent);
    const Point2 suffix_tangent = normalized(suffix_initial.tangent);
    if (std::hypot(prefix_tangent.x, prefix_tangent.y) <= kTinyDistanceM ||
        std::hypot(suffix_tangent.x, suffix_tangent.y) <= kTinyDistanceM) {
      result.reason = "invalid_tangent";
      return result;
    }
    const double tangent_dot = std::clamp(prefix_tangent.x * suffix_tangent.x +
                                              prefix_tangent.y * suffix_tangent.y,
                                          -1.0, 1.0);
    result.tangent_jump_rad = std::acos(tangent_dot);
    if (result.tangent_jump_rad > request.max_tangent_jump_rad) {
      result.reason = "tangent_mismatch";
      return result;
    }
  }
  if (request.require_altitude_match &&
      result.altitude_jump_m > request.max_altitude_jump_m) {
    result.reason = "altitude_mismatch";
    return result;
  }

  result.valid = true;
  result.reason = "join_valid";
  return result;
}

} // namespace drone_city_nav
