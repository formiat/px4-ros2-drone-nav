#include "drone_city_nav/trajectory_update_continuity.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

[[nodiscard]] double norm(const Point2 point) noexcept {
  return std::hypot(point.x, point.y);
}

[[nodiscard]] double dot(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

[[nodiscard]] Point2 operator*(const Point2 point, const double scale) noexcept {
  return Point2{point.x * scale, point.y * scale};
}

[[nodiscard]] Point2 operator-(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] double tangentJumpRad(const Point2 lhs, const Point2 rhs) noexcept {
  const double lhs_norm = norm(lhs);
  const double rhs_norm = norm(rhs);
  if (!(lhs_norm > kTinyDistanceM) || !(rhs_norm > kTinyDistanceM)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::acos(std::clamp(dot(lhs, rhs) / (lhs_norm * rhs_norm), -1.0, 1.0));
}

[[nodiscard]] double speedLimitAtProjection(const TrajectorySpeedProfile& profile,
                                            const TrajectoryProjection& projection) {
  const TrajectorySpeedSample sample = speedProfileSampleAtS(profile, projection.s_m);
  return sample.profiled_limit_mps;
}

[[nodiscard]] Point2
tangentSpeedCommandAtProjection(const TrajectorySpeedProfile& profile,
                                const TrajectoryProjection& projection) {
  const double speed = speedLimitAtProjection(profile, projection);
  if (!std::isfinite(speed)) {
    return Point2{};
  }
  return projection.tangent * speed;
}

} // namespace

const char*
trajectoryContinuityDecisionName(const TrajectoryContinuityDecision decision) noexcept {
  switch (decision) {
    case TrajectoryContinuityDecision::kPreserveSmoother:
      return "preserve_smoother";
    case TrajectoryContinuityDecision::kResetSmoother:
      return "reset_smoother";
    case TrajectoryContinuityDecision::kRejectTrajectory:
      return "reject_trajectory";
  }
  return "unknown";
}

TrajectoryContinuityResult
evaluateTrajectoryContinuity(const std::span<const TrajectoryPointSample> old_samples,
                             const TrajectorySpeedProfile& old_speed_profile,
                             const std::span<const TrajectoryPointSample> new_samples,
                             const TrajectorySpeedProfile& new_speed_profile,
                             const Point2 current_position,
                             const Point2 previous_velocity_setpoint,
                             const bool previous_velocity_setpoint_valid,
                             const TrajectoryContinuityThresholds& thresholds) {
  TrajectoryContinuityResult result{};
  if (!trajectorySamplesAreUsable(old_samples) || !old_speed_profile.valid) {
    return result;
  }
  if (!trajectorySamplesAreUsable(new_samples) || !new_speed_profile.valid) {
    result.decision = TrajectoryContinuityDecision::kRejectTrajectory;
    result.reason = "new_trajectory_invalid";
    return result;
  }

  const std::optional<TrajectoryProjection> old_projection =
      projectOnTrajectorySamples(old_samples, current_position);
  const std::optional<TrajectoryProjection> new_projection =
      projectOnTrajectorySamples(new_samples, current_position);
  result.old_projection_valid = old_projection.has_value();
  result.new_projection_valid = new_projection.has_value();
  if (!old_projection.has_value() || !new_projection.has_value()) {
    result.decision = TrajectoryContinuityDecision::kRejectTrajectory;
    result.reason = "projection_unavailable";
    return result;
  }

  result.projection_jump_m = distance(old_projection->point, new_projection->point);
  result.tangent_jump_rad =
      tangentJumpRad(old_projection->tangent, new_projection->tangent);
  result.curvature_jump_1pm =
      std::abs(new_projection->curvature_1pm - old_projection->curvature_1pm);
  const double old_speed = speedLimitAtProjection(old_speed_profile, *old_projection);
  const double new_speed = speedLimitAtProjection(new_speed_profile, *new_projection);
  result.speed_limit_jump_mps = std::isfinite(old_speed) && std::isfinite(new_speed)
                                    ? std::abs(new_speed - old_speed)
                                    : std::numeric_limits<double>::quiet_NaN();
  const Point2 reference_command =
      previous_velocity_setpoint_valid
          ? previous_velocity_setpoint
          : tangentSpeedCommandAtProjection(old_speed_profile, *old_projection);
  result.tangent_speed_command_jump_mps =
      norm(tangentSpeedCommandAtProjection(new_speed_profile, *new_projection) -
           reference_command);

  const bool reject = result.projection_jump_m > thresholds.reject_projection_jump_m ||
                      (std::isfinite(result.tangent_jump_rad) &&
                       result.tangent_jump_rad > thresholds.reject_tangent_jump_rad) ||
                      result.tangent_speed_command_jump_mps >
                          thresholds.reject_tangent_speed_command_jump_mps;
  if (reject) {
    result.decision = TrajectoryContinuityDecision::kRejectTrajectory;
    result.reason = "control_discontinuity";
    return result;
  }

  const bool preserve =
      result.projection_jump_m <= thresholds.preserve_projection_jump_m &&
      (!std::isfinite(result.tangent_jump_rad) ||
       result.tangent_jump_rad <= thresholds.preserve_tangent_jump_rad) &&
      result.curvature_jump_1pm <= thresholds.preserve_curvature_jump_1pm &&
      (!std::isfinite(result.speed_limit_jump_mps) ||
       result.speed_limit_jump_mps <= thresholds.preserve_speed_limit_jump_mps) &&
      result.tangent_speed_command_jump_mps <=
          thresholds.preserve_tangent_speed_command_jump_mps;
  if (preserve) {
    result.decision = TrajectoryContinuityDecision::kPreserveSmoother;
    result.reason = "compatible";
    return result;
  }
  result.decision = TrajectoryContinuityDecision::kResetSmoother;
  result.reason = "moderate_discontinuity";
  return result;
}

} // namespace drone_city_nav
