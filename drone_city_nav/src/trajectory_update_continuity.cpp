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

[[nodiscard]] bool finiteSafeWindow(const TrajectoryVerticalTarget& target) noexcept {
  return target.vertical_hard_window_active &&
         std::isfinite(target.vertical_safe_min_z_m) &&
         std::isfinite(target.vertical_safe_max_z_m) &&
         target.vertical_safe_max_z_m >= target.vertical_safe_min_z_m;
}

[[nodiscard]] bool hardWindowChanged(const TrajectoryVerticalTarget& old_target,
                                     const TrajectoryVerticalTarget& new_target) {
  if (old_target.vertical_hard_window_active !=
      new_target.vertical_hard_window_active) {
    return true;
  }
  if (!old_target.vertical_hard_window_active) {
    return false;
  }
  return old_target.vertical_profile_passage_id !=
             new_target.vertical_profile_passage_id ||
         std::abs(old_target.vertical_safe_min_z_m - new_target.vertical_safe_min_z_m) >
             kTinyDistanceM ||
         std::abs(old_target.vertical_safe_max_z_m - new_target.vertical_safe_max_z_m) >
             kTinyDistanceM;
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
                             const TrajectoryContinuityThresholds& thresholds,
                             const TrajectoryVerticalContinuityState& vertical_state) {
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
  const TrajectoryVerticalTarget old_vertical_target =
      trajectoryVerticalTargetAtS(old_samples, old_projection->s_m);
  const TrajectoryVerticalTarget new_vertical_target =
      trajectoryVerticalTargetAtS(new_samples, new_projection->s_m);
  if (old_vertical_target.valid && new_vertical_target.valid) {
    result.vertical_target_z_jump_m =
        std::abs(new_vertical_target.z_m - old_vertical_target.z_m);
    const double horizontal_speed_mps = norm(reference_command);
    result.vertical_target_vz_jump_mps =
        std::abs((new_vertical_target.vertical_slope_dz_ds -
                  old_vertical_target.vertical_slope_dz_ds) *
                 horizontal_speed_mps);
    result.vertical_hard_window_changed =
        hardWindowChanged(old_vertical_target, new_vertical_target);
  }
  if (vertical_state.altitude_valid &&
      std::isfinite(vertical_state.current_altitude_m) &&
      finiteSafeWindow(new_vertical_target)) {
    const double tolerance_m =
        std::max(0.0, thresholds.vertical_hard_window_altitude_tolerance_m);
    result.vertical_hard_window_unsafe =
        vertical_state.current_altitude_m <
            new_vertical_target.vertical_safe_min_z_m - tolerance_m ||
        vertical_state.current_altitude_m >
            new_vertical_target.vertical_safe_max_z_m + tolerance_m;
  }
  result.preserve_vertical_smoother_state =
      old_vertical_target.valid && new_vertical_target.valid &&
      std::isfinite(result.vertical_target_z_jump_m) &&
      result.vertical_target_z_jump_m <= thresholds.preserve_vertical_target_z_jump_m &&
      std::isfinite(result.vertical_target_vz_jump_mps) &&
      result.vertical_target_vz_jump_mps <=
          thresholds.preserve_vertical_target_vz_jump_mps &&
      !result.vertical_hard_window_changed && !result.vertical_hard_window_unsafe;

  const bool reject = result.vertical_hard_window_unsafe ||
                      result.projection_jump_m > thresholds.reject_projection_jump_m ||
                      (std::isfinite(result.tangent_jump_rad) &&
                       result.tangent_jump_rad > thresholds.reject_tangent_jump_rad) ||
                      result.tangent_speed_command_jump_mps >
                          thresholds.reject_tangent_speed_command_jump_mps;
  if (reject) {
    result.decision = TrajectoryContinuityDecision::kRejectTrajectory;
    result.reason = result.vertical_hard_window_unsafe ? "vertical_hard_window_unsafe"
                                                       : "control_discontinuity";
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
          thresholds.preserve_tangent_speed_command_jump_mps &&
      result.preserve_vertical_smoother_state;
  if (preserve) {
    result.decision = TrajectoryContinuityDecision::kPreserveSmoother;
    result.reason = "compatible";
    return result;
  }
  result.decision = TrajectoryContinuityDecision::kResetSmoother;
  const bool vertical_discontinuity =
      result.vertical_hard_window_changed ||
      (std::isfinite(result.vertical_target_z_jump_m) &&
       result.vertical_target_z_jump_m >
           thresholds.preserve_vertical_target_z_jump_m) ||
      (std::isfinite(result.vertical_target_vz_jump_mps) &&
       result.vertical_target_vz_jump_mps >
           thresholds.preserve_vertical_target_vz_jump_mps);
  result.reason =
      vertical_discontinuity ? "vertical_discontinuity" : "moderate_discontinuity";
  return result;
}

} // namespace drone_city_nav
