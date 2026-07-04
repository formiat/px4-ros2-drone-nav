#include "drone_city_nav/velocity_command_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

constexpr double kTinyDistanceM = 1.0e-6;

struct VectorRateLimitResult {
  Point2 value{};
  double delta{std::numeric_limits<double>::quiet_NaN()};
};

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

[[nodiscard]] double norm(const Point2 point) noexcept {
  return std::hypot(point.x, point.y);
}

[[nodiscard]] double dot(const Point2 lhs, const Point2 rhs) noexcept {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

[[nodiscard]] double smoothstep(const double edge0, const double edge1,
                                const double value) noexcept {
  if (!(edge1 > edge0)) {
    return value >= edge1 ? 1.0 : 0.0;
  }
  const double t = std::clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

[[nodiscard]] Point2 normalized(const Point2 point) noexcept {
  const double length = norm(point);
  if (!(length > kTinyDistanceM)) {
    return Point2{};
  }
  return Point2{point.x / length, point.y / length};
}

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] VectorRateLimitResult
limitVectorRate(const Point2 desired, const Point2 previous, const bool previous_valid,
                const double dt_s, const double max_rate) {
  VectorRateLimitResult result{};
  result.value = desired;
  if (!previous_valid || !finite2D(previous) || !finite2D(desired)) {
    return result;
  }
  const double sanitized_rate = sanitizedPositive(max_rate, 0.0, 0.0, 1000.0);
  const Point2 delta = desired - previous;
  const double delta_norm = norm(delta);
  result.delta = delta_norm;
  if (!(sanitized_rate > 0.0)) {
    return result;
  }
  const double dt = sanitizedPositive(dt_s, 0.1, 0.0, 10.0);
  const double max_delta = sanitized_rate * dt;
  if (delta_norm <= max_delta || !(delta_norm > kTinyDistanceM)) {
    return result;
  }
  result.value = previous + delta * (max_delta / delta_norm);
  result.delta = max_delta;
  return result;
}

[[nodiscard]] Point2 boundedCorrectionByAngle(const Point2 correction_velocity,
                                              const double base_speed_mps,
                                              const double max_angle_rad) {
  const double max_angle = sanitizedPositive(max_angle_rad, 0.35, 0.0, 1.4);
  const double max_correction_mps = std::max(base_speed_mps, 1.0) * std::tan(max_angle);
  const double correction_speed = norm(correction_velocity);
  if (correction_speed <= max_correction_mps || !(correction_speed > kTinyDistanceM)) {
    return correction_velocity;
  }
  return correction_velocity * (max_correction_mps / correction_speed);
}

[[nodiscard]] Point2
curvatureFeedforwardVelocity(const TrajectoryProjection& projection,
                             const double scalar_speed_mps,
                             const VelocityFollowerConfig& config,
                             double& raw_angle_rad, double& angle_rad, double& scale) {
  raw_angle_rad = 0.0;
  angle_rad = 0.0;
  scale = 0.0;
  const double speed = std::max(0.0, scalar_speed_mps);
  const double anticipation_time_s =
      sanitizedPositive(config.curvature_feedforward_time_s, 0.5, 0.0, 10.0);
  const double max_angle =
      sanitizedPositive(config.max_curvature_feedforward_angle_rad, 0.7, 0.0, 1.4);
  const double deadband_angle = sanitizedPositive(
      config.curvature_feedforward_deadband_angle_rad, 0.0, 0.0, max_angle);
  const double full_angle = std::max(
      deadband_angle, sanitizedPositive(config.curvature_feedforward_full_angle_rad,
                                        deadband_angle, 0.0, max_angle));
  raw_angle_rad = projection.curvature_1pm * speed * anticipation_time_s;
  scale = smoothstep(deadband_angle, full_angle, std::abs(raw_angle_rad));
  angle_rad = std::clamp(raw_angle_rad * scale, -max_angle, max_angle);
  if (!(std::abs(angle_rad) > kTinyDistanceM)) {
    return Point2{};
  }
  const Point2 left_normal{-projection.tangent.y, projection.tangent.x};
  return left_normal * (std::max(speed, 1.0) * std::tan(angle_rad));
}

[[nodiscard]] double
speedAwareDerivativeDampingFactor(const double speed_mps,
                                  const double cross_track_lateral_velocity_mps,
                                  const VelocityFollowerConfig& config) noexcept {
  if (!(cross_track_lateral_velocity_mps > kTinyDistanceM)) {
    return 1.0;
  }
  const double min_speed = sanitizedPositive(
      config.speed_aware_derivative_damping_min_speed_mps, 8.0, 0.0, 1000.0);
  const double full_speed = std::max(
      min_speed, sanitizedPositive(config.speed_aware_derivative_damping_full_speed_mps,
                                   20.0, 0.0, 1000.0));
  const double max_factor = sanitizedPositive(
      config.speed_aware_derivative_damping_max_factor, 1.5, 1.0, 100.0);
  const double speed_factor =
      smoothstep(min_speed, full_speed, std::max(0.0, speed_mps));
  return 1.0 + (max_factor - 1.0) * speed_factor;
}

[[nodiscard]] double
adaptiveLateralResponseFactor(const VelocityCommandQuery& query,
                              const VelocityFollowerConfig& config) noexcept {
  if (!std::isfinite(query.current_cross_track_error_m) ||
      !std::isfinite(query.predicted_cross_track_error_m) ||
      query.predicted_cross_track_error_m <= query.current_cross_track_error_m) {
    return 1.0;
  }

  const double cross_track_growth_m =
      query.predicted_cross_track_error_m - query.current_cross_track_error_m;
  const double scale_m =
      sanitizedPositive(config.adaptive_lateral_response_scale_m, 3.0, 1.0e-6, 1000.0);
  const double max_factor =
      sanitizedPositive(config.adaptive_lateral_response_max_factor, 1.4, 1.0, 100.0);
  return std::clamp(1.0 + cross_track_growth_m / scale_m, 1.0, max_factor);
}

[[nodiscard]] double crossTrackFeedbackScale(
    const double cross_track_error_m, const double cross_track_lateral_velocity_mps,
    const VelocityFollowerConfig& config, double& closing_speed_target_mps) noexcept {
  closing_speed_target_mps = std::numeric_limits<double>::quiet_NaN();
  if (!(cross_track_error_m > kTinyDistanceM) ||
      !(cross_track_lateral_velocity_mps > kTinyDistanceM)) {
    return 1.0;
  }

  const double target_time_s =
      sanitizedPositive(config.cross_track_anti_overshoot_time_s, 1.0, 1.0e-6, 1000.0);
  closing_speed_target_mps = cross_track_error_m / target_time_s;
  if (!(cross_track_lateral_velocity_mps > closing_speed_target_mps)) {
    return 1.0;
  }

  const double min_scale = sanitizedPositive(
      config.cross_track_anti_overshoot_min_feedback_scale, 0.25, 0.0, 1.0);
  return std::clamp(closing_speed_target_mps / cross_track_lateral_velocity_mps,
                    min_scale, 1.0);
}

[[nodiscard]] Point2 crossTrackOvershootDampingVelocity(
    const VelocityCommandQuery& query, const VelocityFollowerConfig& config,
    const double damping_gain, double& actual_closing_speed_mps,
    double& closing_speed_limit_mps) noexcept {
  actual_closing_speed_mps = std::numeric_limits<double>::quiet_NaN();
  closing_speed_limit_mps = std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(query.actual_signed_cross_track_error_m) ||
      !std::isfinite(query.actual_cross_track_lateral_velocity_mps) ||
      !(norm(query.actual_path_tangent) > kTinyDistanceM)) {
    return Point2{};
  }

  const double signed_error_m = query.actual_signed_cross_track_error_m;
  if (!(std::abs(signed_error_m) > kTinyDistanceM)) {
    return Point2{};
  }

  const double error_sign = signed_error_m > 0.0 ? 1.0 : -1.0;
  actual_closing_speed_mps =
      -error_sign * query.actual_cross_track_lateral_velocity_mps;
  if (!(actual_closing_speed_mps > kTinyDistanceM)) {
    return Point2{};
  }

  const double target_time_s =
      sanitizedPositive(config.cross_track_anti_overshoot_time_s, 1.0, 1.0e-6, 1000.0);
  closing_speed_limit_mps = std::abs(signed_error_m) / target_time_s;
  if (!(actual_closing_speed_mps > closing_speed_limit_mps)) {
    return Point2{};
  }

  const double excess_closing_speed_mps =
      actual_closing_speed_mps - closing_speed_limit_mps;
  const Point2 actual_left_normal{-query.actual_path_tangent.y,
                                  query.actual_path_tangent.x};
  const Point2 away_from_path = normalized(actual_left_normal * error_sign);
  if (!(norm(away_from_path) > kTinyDistanceM)) {
    return Point2{};
  }

  const double gain = sanitizedPositive(damping_gain, 0.0, 0.0, 100.0);
  return away_from_path * (excess_closing_speed_mps * gain);
}

} // namespace

VelocityCommandPlan planVelocityCommand(const VelocityCommandQuery& query,
                                        const VelocityFollowerConfig& config) {
  VelocityCommandPlan plan{};
  if (!finite2D(query.current_position) || !finite2D(query.projection.point) ||
      !(norm(query.projection.tangent) > kTinyDistanceM) ||
      !std::isfinite(query.scalar_speed_mps)) {
    return plan;
  }

  const Point2 cross_track = query.projection.point - query.current_position;
  const double cross_track_error = std::sqrt(query.projection.distance_sq);
  const double cross_track_gain =
      sanitizedPositive(config.cross_track_gain, 0.25, 0.0, 10.0);
  const double cross_track_derivative_gain =
      sanitizedPositive(config.cross_track_derivative_gain, 0.8, 0.0, 10.0);
  const Point2 cross_track_direction = normalized(cross_track);
  Point2 cross_track_feedback{};
  Point2 cross_track_derivative_damping{};
  if (norm(cross_track_direction) > kTinyDistanceM) {
    plan.cross_track_lateral_velocity_mps =
        query.current_velocity_valid && finite2D(query.current_velocity)
            ? dot(query.current_velocity, cross_track_direction)
            : 0.0;
    const double derivative_speed =
        query.current_velocity_valid && finite2D(query.current_velocity)
            ? norm(query.current_velocity)
            : query.scalar_speed_mps;
    plan.cross_track_derivative_damping_factor = speedAwareDerivativeDampingFactor(
        derivative_speed, plan.cross_track_lateral_velocity_mps, config);
    plan.cross_track_derivative_gain_effective =
        cross_track_derivative_gain * plan.cross_track_derivative_damping_factor;
    plan.cross_track_feedback_scale = crossTrackFeedbackScale(
        cross_track_error, plan.cross_track_lateral_velocity_mps, config,
        plan.cross_track_closing_speed_target_mps);
    cross_track_feedback =
        cross_track_direction *
        (cross_track_gain * cross_track_error * plan.cross_track_feedback_scale);
    cross_track_derivative_damping =
        cross_track_direction * (-plan.cross_track_derivative_gain_effective *
                                 plan.cross_track_lateral_velocity_mps);
  }

  double curvature_feedforward_angle_rad = 0.0;
  double curvature_feedforward_raw_angle_rad = 0.0;
  double curvature_feedforward_scale = 0.0;
  const Point2 curvature_feedforward = curvatureFeedforwardVelocity(
      query.projection, query.scalar_speed_mps, config,
      curvature_feedforward_raw_angle_rad, curvature_feedforward_angle_rad,
      curvature_feedforward_scale);
  double actual_cross_track_closing_speed_mps =
      std::numeric_limits<double>::quiet_NaN();
  double actual_cross_track_closing_speed_limit_mps =
      std::numeric_limits<double>::quiet_NaN();
  const double overshoot_damping_gain =
      std::max(plan.cross_track_derivative_gain_effective, cross_track_derivative_gain);
  const Point2 cross_track_overshoot_damping = crossTrackOvershootDampingVelocity(
      query, config, overshoot_damping_gain, actual_cross_track_closing_speed_mps,
      actual_cross_track_closing_speed_limit_mps);
  const double adaptive_lateral_response_factor =
      adaptiveLateralResponseFactor(query, config);
  const Point2 raw_lateral_control =
      cross_track_feedback + cross_track_derivative_damping +
      cross_track_overshoot_damping + curvature_feedforward;

  const Point2 bounded_lateral_control =
      boundedCorrectionByAngle(raw_lateral_control, query.scalar_speed_mps,
                               config.max_lateral_control_angle_rad);
  const VectorRateLimitResult limited_lateral_control = limitVectorRate(
      bounded_lateral_control, query.previous_lateral_control_velocity,
      query.previous_lateral_control_velocity_valid, query.dt_s,
      config.max_lateral_control_rate_mps2 * adaptive_lateral_response_factor);
  const Point2 lateral_control = limited_lateral_control.value;

  const Point2 desired_direction =
      normalized(query.projection.tangent * std::max(query.scalar_speed_mps, 1.0) +
                 lateral_control);
  if (!(norm(desired_direction) > kTinyDistanceM)) {
    return plan;
  }

  const Point2 desired_velocity = desired_direction * query.scalar_speed_mps;
  const Point2 left_normal{-query.projection.tangent.y, query.projection.tangent.x};
  plan.valid = true;
  plan.desired_velocity_xy = desired_velocity;
  plan.cross_track_feedback_velocity = cross_track_feedback;
  plan.cross_track_derivative_damping_velocity = cross_track_derivative_damping;
  plan.cross_track_overshoot_damping_velocity = cross_track_overshoot_damping;
  plan.curvature_feedforward_velocity = curvature_feedforward;
  plan.raw_lateral_control_velocity = raw_lateral_control;
  plan.lateral_control_velocity = lateral_control;
  plan.cross_track_feedback_mps = norm(cross_track_feedback);
  plan.cross_track_derivative_damping_mps = norm(cross_track_derivative_damping);
  plan.actual_signed_cross_track_error_m = query.actual_signed_cross_track_error_m;
  plan.actual_cross_track_lateral_velocity_mps =
      query.actual_cross_track_lateral_velocity_mps;
  plan.actual_cross_track_closing_speed_mps = actual_cross_track_closing_speed_mps;
  plan.actual_cross_track_closing_speed_limit_mps =
      actual_cross_track_closing_speed_limit_mps;
  plan.cross_track_overshoot_damping_mps = norm(cross_track_overshoot_damping);
  plan.curvature_feedforward_mps = norm(curvature_feedforward);
  plan.curvature_feedforward_angle_rad = curvature_feedforward_angle_rad;
  plan.curvature_feedforward_raw_angle_rad = curvature_feedforward_raw_angle_rad;
  plan.curvature_feedforward_scale = curvature_feedforward_scale;
  plan.raw_lateral_control_mps = norm(raw_lateral_control);
  plan.lateral_control_mps = norm(lateral_control);
  plan.lateral_control_delta_mps = limited_lateral_control.delta;
  plan.adaptive_lateral_response_factor = adaptive_lateral_response_factor;
  plan.desired_velocity_tangent_mps = dot(desired_velocity, query.projection.tangent);
  plan.desired_velocity_normal_mps = dot(desired_velocity, left_normal);
  return plan;
}

} // namespace drone_city_nav
