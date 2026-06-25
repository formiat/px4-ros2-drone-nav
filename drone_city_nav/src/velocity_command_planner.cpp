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
curvatureAnticipationVelocity(const TrajectoryProjection& projection,
                              const double scalar_speed_mps,
                              const VelocityFollowerConfig& config, double& angle_rad) {
  angle_rad = 0.0;
  const double speed = std::max(0.0, scalar_speed_mps);
  const double anticipation_time_s =
      sanitizedPositive(config.curvature_velocity_anticipation_time_s, 0.5, 0.0, 10.0);
  const double max_angle = sanitizedPositive(
      config.max_curvature_velocity_anticipation_angle_rad, 0.7, 0.0, 1.4);
  angle_rad = std::clamp(projection.curvature_1pm * speed * anticipation_time_s,
                         -max_angle, max_angle);
  if (!(std::abs(angle_rad) > kTinyDistanceM)) {
    return Point2{};
  }
  const Point2 left_normal{-projection.tangent.y, projection.tangent.x};
  return left_normal * (std::max(speed, 1.0) * std::tan(angle_rad));
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
  Point2 requested_correction{};
  if (norm(cross_track_direction) > kTinyDistanceM) {
    plan.cross_track_lateral_velocity_mps =
        query.current_velocity_valid && finite2D(query.current_velocity)
            ? dot(query.current_velocity, cross_track_direction)
            : 0.0;
    const double correction_speed = std::max(
        0.0, cross_track_gain * cross_track_error -
                 cross_track_derivative_gain * plan.cross_track_lateral_velocity_mps);
    requested_correction = cross_track_direction * correction_speed;
  }

  const Point2 bounded_correction =
      boundedCorrectionByAngle(requested_correction, query.scalar_speed_mps,
                               config.max_cross_track_correction_angle_rad);
  const VectorRateLimitResult limited_correction = limitVectorRate(
      bounded_correction, query.previous_cross_track_correction_velocity,
      query.previous_cross_track_correction_velocity_valid, query.dt_s,
      config.max_cross_track_correction_rate_mps2);
  const Point2 correction = limited_correction.value;

  double curvature_anticipation_angle_rad = 0.0;
  const Point2 raw_curvature_anticipation =
      curvatureAnticipationVelocity(query.projection, query.scalar_speed_mps, config,
                                    curvature_anticipation_angle_rad);
  const VectorRateLimitResult limited_curvature_anticipation = limitVectorRate(
      raw_curvature_anticipation, query.previous_curvature_anticipation_velocity,
      query.previous_curvature_anticipation_velocity_valid, query.dt_s,
      config.max_curvature_velocity_anticipation_rate_mps2);
  const Point2 curvature_anticipation = limited_curvature_anticipation.value;

  const Point2 desired_direction =
      normalized(query.projection.tangent * std::max(query.scalar_speed_mps, 1.0) +
                 correction + curvature_anticipation);
  if (!(norm(desired_direction) > kTinyDistanceM)) {
    return plan;
  }

  const Point2 desired_velocity = desired_direction * query.scalar_speed_mps;
  const Point2 left_normal{-query.projection.tangent.y, query.projection.tangent.x};
  plan.valid = true;
  plan.desired_velocity_xy = desired_velocity;
  plan.raw_cross_track_correction_velocity = requested_correction;
  plan.cross_track_correction_velocity = correction;
  plan.raw_curvature_anticipation_velocity = raw_curvature_anticipation;
  plan.curvature_anticipation_velocity = curvature_anticipation;
  plan.raw_cross_track_correction_mps = norm(requested_correction);
  plan.cross_track_correction_mps = norm(correction);
  plan.cross_track_correction_delta_mps = limited_correction.delta;
  plan.desired_velocity_tangent_mps = dot(desired_velocity, query.projection.tangent);
  plan.desired_velocity_normal_mps = dot(desired_velocity, left_normal);
  plan.raw_curvature_anticipation_mps = norm(raw_curvature_anticipation);
  plan.curvature_anticipation_mps = norm(curvature_anticipation);
  plan.curvature_anticipation_delta_mps = limited_curvature_anticipation.delta;
  plan.curvature_anticipation_angle_rad = curvature_anticipation_angle_rad;
  return plan;
}

} // namespace drone_city_nav
