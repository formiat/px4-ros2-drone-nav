#include "drone_city_nav/trajectory_horizontal_handover.hpp"

#include "drone_city_nav/planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] Point2 add(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 scale(const Point2 point, const double factor) noexcept {
  return Point2{point.x * factor, point.y * factor};
}

[[nodiscard]] double vectorNorm(const Point2 point) noexcept {
  return std::hypot(point.x, point.y);
}

[[nodiscard]] Point2 normalizedVector(const Point2 point) noexcept {
  const double magnitude = vectorNorm(point);
  return magnitude > kTinyDistanceM ? scale(point, 1.0 / magnitude) : Point2{};
}

[[nodiscard]] bool finitePoint(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] double dot(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

[[nodiscard]] double tangentJumpRad(const Point2 lhs, const Point2 rhs) noexcept {
  const double lhs_norm = vectorNorm(lhs);
  const double rhs_norm = vectorNorm(rhs);
  if (!(lhs_norm > kTinyDistanceM) || !(rhs_norm > kTinyDistanceM)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::acos(std::clamp(dot(lhs, rhs) / (lhs_norm * rhs_norm), -1.0, 1.0));
}

[[nodiscard]] TrajectoryPointSample
sampleAtS(const std::span<const TrajectoryPointSample> samples, const double s_m) {
  const double bounded_s_m = std::clamp(s_m, 0.0, samples.back().s_m);
  for (std::size_t i = 0U; i + 1U < samples.size(); ++i) {
    const TrajectoryPointSample& start = samples[i];
    const TrajectoryPointSample& end = samples[i + 1U];
    if (bounded_s_m > end.s_m && i + 2U < samples.size()) {
      continue;
    }
    const double ds_m = end.s_m - start.s_m;
    const double ratio = ds_m > kTinyDistanceM
                             ? std::clamp((bounded_s_m - start.s_m) / ds_m, 0.0, 1.0)
                             : 0.0;
    TrajectoryPointSample sample = ratio <= 0.5 ? start : end;
    sample.s_m = bounded_s_m;
    sample.point = add(scale(start.point, 1.0 - ratio), scale(end.point, ratio));
    sample.tangent = normalizedVector(
        add(scale(start.tangent, 1.0 - ratio), scale(end.tangent, ratio)));
    sample.curvature_1pm =
        start.curvature_1pm * (1.0 - ratio) + end.curvature_1pm * ratio;
    sample.z_m = start.z_m * (1.0 - ratio) + end.z_m * ratio;
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

[[nodiscard]] Point2 cubicHermitePoint(const Point2 start, const Point2 start_tangent,
                                       const Point2 end, const Point2 end_tangent,
                                       const double tangent_scale_m,
                                       const double ratio) noexcept {
  const double t = std::clamp(ratio, 0.0, 1.0);
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
  const double h10 = t3 - 2.0 * t2 + t;
  const double h01 = -2.0 * t3 + 3.0 * t2;
  const double h11 = t3 - t2;
  return add(add(scale(start, h00), scale(start_tangent, h10 * tangent_scale_m)),
             add(scale(end, h01), scale(end_tangent, h11 * tangent_scale_m)));
}

[[nodiscard]] std::optional<double>
firstHardWindowStationAfter(const std::span<const TrajectoryPointSample> samples,
                            const double s_m) {
  for (const TrajectoryPointSample& sample : samples) {
    if (sample.s_m > s_m + kTinyDistanceM && sample.vertical_hard_window_active) {
      return sample.s_m;
    }
  }
  return std::nullopt;
}

[[nodiscard]] bool
geometryWithinLimits(const std::span<const TrajectoryPointSample> samples,
                     const HorizontalTrajectoryHandoverConfig& config,
                     double& max_sample_heading_delta_rad,
                     double& max_abs_curvature_1pm) noexcept {
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    max_abs_curvature_1pm =
        std::max(max_abs_curvature_1pm, std::abs(samples[i].curvature_1pm));
    if (i > 0U) {
      const double heading_delta =
          tangentJumpRad(samples[i - 1U].tangent, samples[i].tangent);
      if (std::isfinite(heading_delta)) {
        max_sample_heading_delta_rad =
            std::max(max_sample_heading_delta_rad, heading_delta);
      }
    }
  }
  return max_abs_curvature_1pm <= config.max_abs_curvature_1pm &&
         max_sample_heading_delta_rad <= config.max_sample_heading_delta_rad;
}

} // namespace

HorizontalTrajectoryHandoverResult buildHorizontalTrajectoryHandover(
    const std::span<const TrajectoryPointSample> current_samples,
    const std::span<const TrajectoryPointSample> candidate_samples,
    const HorizontalTrajectoryHandoverState& state,
    const HorizontalTrajectoryHandoverConfig& config,
    const OccupancyGrid2D* const validation_grid) {
  HorizontalTrajectoryHandoverResult result{};
  if (!config.enabled) {
    result.reason = "disabled";
    return result;
  }
  if (!state.current_position_valid || !finitePoint(state.current_position)) {
    result.reason = "position_invalid";
    return result;
  }
  if (!trajectorySamplesAreUsable(current_samples) ||
      !trajectorySamplesAreUsable(candidate_samples)) {
    result.reason = "trajectory_invalid";
    return result;
  }
  if (config.require_validation_grid && validation_grid == nullptr) {
    result.reason = "validation_grid_unavailable";
    return result;
  }

  const std::optional<TrajectoryProjection> old_projection =
      projectOnTrajectorySamples(current_samples, state.current_position);
  const std::optional<TrajectoryProjection> candidate_projection =
      projectOnTrajectorySamples(candidate_samples, state.current_position);
  if (!old_projection.has_value() || !candidate_projection.has_value()) {
    result.reason = "projection_unavailable";
    return result;
  }
  result.old_projection_s_m = old_projection->s_m;
  result.candidate_projection_s_m = candidate_projection->s_m;
  result.projection_jump_m =
      distance(old_projection->point, candidate_projection->point);
  result.tangent_jump_rad =
      tangentJumpRad(old_projection->tangent, candidate_projection->tangent);
  if (result.projection_jump_m <= config.trigger_projection_jump_m &&
      (!std::isfinite(result.tangent_jump_rad) ||
       result.tangent_jump_rad <= config.trigger_tangent_jump_rad)) {
    result.reason = "already_compatible";
    return result;
  }

  const TrajectoryVerticalTarget old_target =
      trajectoryVerticalTargetAtS(current_samples, old_projection->s_m);
  const TrajectoryVerticalTarget candidate_target =
      trajectoryVerticalTargetAtS(candidate_samples, candidate_projection->s_m);
  if (old_target.vertical_hard_window_active ||
      candidate_target.vertical_hard_window_active) {
    result.reason = "hard_window_active";
    return result;
  }

  const double speed_mps = state.current_horizontal_speed_valid &&
                                   std::isfinite(state.current_horizontal_speed_mps)
                               ? std::max(0.0, state.current_horizontal_speed_mps)
                               : 0.0;
  result.prefix_distance_m =
      std::clamp(speed_mps * std::max(0.0, config.prefix_time_s),
                 std::max(0.0, config.min_prefix_distance_m),
                 std::max(config.min_prefix_distance_m, config.max_prefix_distance_m));
  result.old_join_s_m = std::min(current_samples.back().s_m,
                                 old_projection->s_m + result.prefix_distance_m);
  double candidate_join_s_m =
      std::min(candidate_samples.back().s_m, candidate_projection->s_m +
                                                 result.prefix_distance_m +
                                                 config.candidate_lookahead_distance_m);
  if (const std::optional<double> hard_window_s =
          firstHardWindowStationAfter(candidate_samples, candidate_projection->s_m);
      hard_window_s.has_value()) {
    candidate_join_s_m =
        std::min(candidate_join_s_m,
                 *hard_window_s - std::max(config.sample_step_m, kTinyDistanceM));
  }
  if (!(result.old_join_s_m > old_projection->s_m + kTinyDistanceM) ||
      !(candidate_join_s_m > candidate_projection->s_m + kTinyDistanceM)) {
    result.reason = "insufficient_join_distance";
    return result;
  }
  result.candidate_join_s_m = candidate_join_s_m;

  const TrajectoryPointSample old_start =
      sampleAtS(current_samples, old_projection->s_m);
  const TrajectoryPointSample old_join =
      sampleAtS(current_samples, result.old_join_s_m);
  const TrajectoryPointSample candidate_join =
      sampleAtS(candidate_samples, result.candidate_join_s_m);
  result.join_distance_m = distance(old_join.point, candidate_join.point);
  if (result.join_distance_m > config.max_join_distance_m) {
    result.reason = "join_distance_exceeded";
    return result;
  }

  std::vector<TrajectoryPointSample> stitched;
  stitched.reserve(current_samples.size() + candidate_samples.size());
  appendDistinct(stitched, old_start);
  for (const TrajectoryPointSample& sample : current_samples) {
    if (sample.s_m > old_projection->s_m + kTinyDistanceM &&
        sample.s_m < result.old_join_s_m - kTinyDistanceM) {
      appendDistinct(stitched, sample);
    }
  }
  appendDistinct(stitched, old_join);

  const double bridge_scale_m =
      std::max(result.join_distance_m,
               0.5 * ((result.old_join_s_m - old_projection->s_m) +
                      (result.candidate_join_s_m - candidate_projection->s_m)));
  const std::size_t bridge_steps = static_cast<std::size_t>(
      std::max(4.0, std::ceil(std::max(result.join_distance_m, bridge_scale_m) /
                              std::max(config.sample_step_m, 0.1))));
  for (std::size_t step = 1U; step < bridge_steps; ++step) {
    const double ratio = static_cast<double>(step) / static_cast<double>(bridge_steps);
    TrajectoryPointSample sample = ratio <= 0.5 ? old_join : candidate_join;
    sample.point =
        cubicHermitePoint(old_join.point, old_join.tangent, candidate_join.point,
                          candidate_join.tangent, bridge_scale_m, ratio);
    sample.z_m = old_join.z_m * (1.0 - ratio) + candidate_join.z_m * ratio;
    appendDistinct(stitched, std::move(sample));
  }
  appendDistinct(stitched, candidate_join);
  const std::size_t stitched_join_index = stitched.size() - 1U;
  for (const TrajectoryPointSample& sample : candidate_samples) {
    if (sample.s_m > result.candidate_join_s_m + kTinyDistanceM) {
      appendDistinct(stitched, sample);
    }
  }
  populateTrajectorySampleGeometry(stitched);
  if (!trajectorySamplesAreUsable(stitched)) {
    result.reason = "stitched_trajectory_invalid";
    return result;
  }
  if (!geometryWithinLimits(stitched, config, result.max_sample_heading_delta_rad,
                            result.max_abs_curvature_1pm)) {
    result.reason = "geometry_limit_exceeded";
    return result;
  }
  if (validation_grid != nullptr) {
    std::vector<Point2> points;
    points.reserve(stitched.size());
    for (const TrajectoryPointSample& sample : stitched) {
      points.push_back(sample.point);
    }
    if (!pathIsTraversable(*validation_grid, points,
                           &result.non_traversable_segment_index)) {
      result.reason = "non_traversable";
      return result;
    }
  }

  result.stitched_join_s_m = stitched[stitched_join_index].s_m;
  result.candidate_station_offset_m =
      result.stitched_join_s_m - result.candidate_join_s_m;
  result.samples = std::move(stitched);
  result.applied = true;
  result.reason = "predicted_prefix_bridge";
  return result;
}

} // namespace drone_city_nav
