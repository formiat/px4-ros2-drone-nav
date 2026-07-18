#include "drone_city_nav/trajectory_vertical_handover.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;
constexpr double kMinimumHorizontalSpeedMps = 1.0;

[[nodiscard]] double monotoneEndpointSlope(const double slope,
                                           const double secant) noexcept {
  if (std::abs(secant) <= kTinyDistanceM || slope * secant <= 0.0) {
    return 0.0;
  }
  return std::copysign(std::min(std::abs(slope), 3.0 * std::abs(secant)), secant);
}

[[nodiscard]] double cubicHermite(const double start_z_m, const double end_z_m,
                                  const double start_slope, const double end_slope,
                                  const double length_m, const double t) noexcept {
  const double x = std::clamp(t, 0.0, 1.0);
  const double x2 = x * x;
  const double x3 = x2 * x;
  const double h00 = 2.0 * x3 - 3.0 * x2 + 1.0;
  const double h10 = x3 - 2.0 * x2 + x;
  const double h01 = -2.0 * x3 + 3.0 * x2;
  const double h11 = x3 - x2;
  return h00 * start_z_m + h10 * length_m * start_slope + h01 * end_z_m +
         h11 * length_m * end_slope;
}

[[nodiscard]] std::optional<std::size_t>
firstUpcomingHardWindowIndex(const std::span<const TrajectoryPointSample> samples,
                             const double current_s_m) noexcept {
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    if (samples[i].s_m + kTinyDistanceM >= current_s_m &&
        samples[i].vertical_hard_window_active) {
      return i;
    }
  }
  return std::nullopt;
}

} // namespace

VerticalTrajectoryHandoverResult reanchorTrajectoryVerticalPrefix(
    const std::span<const TrajectoryPointSample> current_samples,
    const std::span<TrajectoryPointSample> candidate_samples,
    const Point2 current_position, const VerticalTrajectoryHandoverState& state) {
  VerticalTrajectoryHandoverResult result{};
  if (!state.altitude_valid || !std::isfinite(state.current_altitude_m)) {
    result.reason = "altitude_invalid";
    return result;
  }
  if (!trajectorySamplesAreUsable(candidate_samples)) {
    result.reason = "candidate_invalid";
    return result;
  }

  const std::optional<TrajectoryProjection> candidate_projection =
      projectOnTrajectorySamples(candidate_samples, current_position);
  if (!candidate_projection.has_value()) {
    result.reason = "candidate_projection_unavailable";
    return result;
  }
  result.candidate_s_m = candidate_projection->s_m;
  const TrajectoryVerticalTarget target_before =
      trajectoryVerticalTargetAtS(candidate_samples, result.candidate_s_m);
  result.target_z_before_m = target_before.z_m;
  if (target_before.vertical_hard_window_active) {
    result.reason = "inside_hard_window";
    return result;
  }

  double anchor_z_m = state.current_altitude_m;
  double anchor_slope = 0.0;
  const std::optional<TrajectoryProjection> current_projection =
      projectOnTrajectorySamples(current_samples, current_position);
  if (current_projection.has_value()) {
    const TrajectoryVerticalTarget current_target =
        trajectoryVerticalTargetAtS(current_samples, current_projection->s_m);
    if (current_target.valid && std::isfinite(current_target.z_m)) {
      anchor_z_m = current_target.z_m;
      anchor_slope = current_target.vertical_slope_dz_ds;
    }
  } else if (state.vertical_velocity_valid &&
             std::isfinite(state.current_vertical_velocity_mps)) {
    const double horizontal_speed_mps = std::max(
        kMinimumHorizontalSpeedMps, std::isfinite(state.current_horizontal_speed_mps)
                                        ? state.current_horizontal_speed_mps
                                        : 0.0);
    anchor_slope = state.current_vertical_velocity_mps / horizontal_speed_mps;
  }
  result.anchor_z_m = anchor_z_m;

  const std::optional<std::size_t> hard_window_index =
      firstUpcomingHardWindowIndex(candidate_samples, result.candidate_s_m);
  if (!hard_window_index.has_value()) {
    for (TrajectoryPointSample& sample : candidate_samples) {
      sample.z_m = anchor_z_m;
    }
    result.applied = true;
    result.reason = "carry_current_target";
    result.join_s_m = candidate_samples.back().s_m;
    result.join_z_m = anchor_z_m;
    result.target_z_after_m = anchor_z_m;
    return result;
  }

  const TrajectoryPointSample& join_sample = candidate_samples[*hard_window_index];
  result.join_s_m = join_sample.s_m;
  result.join_z_m = join_sample.z_m;
  const double transition_length_m = result.join_s_m - result.candidate_s_m;
  if (!(transition_length_m > kTinyDistanceM)) {
    result.reason = "hard_window_immediate";
    return result;
  }

  const double secant = (result.join_z_m - anchor_z_m) / transition_length_m;
  const double start_slope = monotoneEndpointSlope(anchor_slope, secant);
  const double end_slope =
      monotoneEndpointSlope(join_sample.vertical_slope_dz_ds, secant);
  for (std::size_t i = 0U; i < *hard_window_index; ++i) {
    TrajectoryPointSample& sample = candidate_samples[i];
    if (sample.s_m <= result.candidate_s_m + kTinyDistanceM) {
      sample.z_m = anchor_z_m;
      continue;
    }
    const double t = (sample.s_m - result.candidate_s_m) / transition_length_m;
    sample.z_m = cubicHermite(anchor_z_m, result.join_z_m, start_slope, end_slope,
                              transition_length_m, t);
  }

  result.applied = true;
  result.reason = "joined_upcoming_hard_window";
  result.target_z_after_m =
      trajectoryVerticalTargetAtS(candidate_samples, result.candidate_s_m).z_m;
  return result;
}

} // namespace drone_city_nav
