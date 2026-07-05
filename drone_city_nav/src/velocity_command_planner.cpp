#include "drone_city_nav/velocity_command_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

[[nodiscard]] Point2 curvatureFeedforwardVelocity(
    const TrajectoryProjection& projection, const double scalar_speed_mps,
    const VelocityFollowerConfig& config, const double context_scale,
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
  const double sanitized_context_scale =
      sanitizedPositive(context_scale, 1.0, 0.0, 1.0);
  scale = smoothstep(deadband_angle, full_angle, std::abs(raw_angle_rad)) *
          sanitized_context_scale;
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
      config.speed_aware_derivative_damping_max_factor, 2.0, 1.0, 100.0);
  const double speed_factor =
      smoothstep(min_speed, full_speed, std::max(0.0, speed_mps));
  return 1.0 + (max_factor - 1.0) * speed_factor;
}

[[nodiscard]] double
crossTrackPGainFactor(const double cross_track_error_m,
                      const VelocityFollowerConfig& config) noexcept {
  const double start_m =
      sanitizedPositive(config.cross_track_p_gain_schedule_start_m, 0.0, 0.0, 1000.0);
  const double full_m =
      std::max(start_m, sanitizedPositive(config.cross_track_p_gain_schedule_full_m,
                                          2.5, 0.0, 1000.0));
  const double min_factor =
      sanitizedPositive(config.cross_track_p_gain_schedule_min_factor, 0.5, 0.0, 100.0);
  const double max_factor = std::max(
      min_factor, sanitizedPositive(config.cross_track_p_gain_schedule_max_factor, 1.3,
                                    0.0, 100.0));
  const double progress =
      smoothstep(start_m, full_m, std::max(0.0, cross_track_error_m));
  return min_factor + (max_factor - min_factor) * progress;
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
    plan.cross_track_p_gain_factor = crossTrackPGainFactor(cross_track_error, config);
    cross_track_feedback =
        cross_track_direction *
        (cross_track_gain * cross_track_error * plan.cross_track_p_gain_factor);
    cross_track_derivative_damping =
        cross_track_direction * (-plan.cross_track_derivative_gain_effective *
                                 plan.cross_track_lateral_velocity_mps);
  }

  double curvature_feedforward_angle_rad = 0.0;
  double curvature_feedforward_raw_angle_rad = 0.0;
  double curvature_feedforward_scale = 0.0;
  const Point2 curvature_feedforward = curvatureFeedforwardVelocity(
      query.projection, query.scalar_speed_mps, config,
      query.curvature_feedforward_context_scale, curvature_feedforward_raw_angle_rad,
      curvature_feedforward_angle_rad, curvature_feedforward_scale);
  const Point2 raw_lateral_control =
      cross_track_feedback + cross_track_derivative_damping + curvature_feedforward;

  const Point2 lateral_control =
      boundedCorrectionByAngle(raw_lateral_control, query.scalar_speed_mps,
                               config.max_lateral_control_angle_rad);

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
  plan.curvature_feedforward_velocity = curvature_feedforward;
  plan.raw_lateral_control_velocity = raw_lateral_control;
  plan.lateral_control_velocity = lateral_control;
  plan.cross_track_feedback_mps = norm(cross_track_feedback);
  plan.cross_track_derivative_damping_mps = norm(cross_track_derivative_damping);
  plan.curvature_feedforward_mps = norm(curvature_feedforward);
  plan.curvature_feedforward_angle_rad = curvature_feedforward_angle_rad;
  plan.curvature_feedforward_raw_angle_rad = curvature_feedforward_raw_angle_rad;
  plan.curvature_feedforward_scale = curvature_feedforward_scale;
  plan.curvature_feedforward_context_scale =
      sanitizedPositive(query.curvature_feedforward_context_scale, 1.0, 0.0, 1.0);
  plan.raw_lateral_control_mps = norm(raw_lateral_control);
  plan.lateral_control_mps = norm(lateral_control);
  plan.desired_velocity_tangent_mps = dot(desired_velocity, query.projection.tangent);
  plan.desired_velocity_normal_mps = dot(desired_velocity, left_normal);
  return plan;
}

} // namespace drone_city_nav
