#include "drone_city_nav/velocity_smoother.hpp"

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

struct VelocityJerkLimitResult {
  Point2 velocity{};
  Point2 acceleration{};
  double acceleration_norm_mps2{std::numeric_limits<double>::quiet_NaN()};
  double jerk_mps3{std::numeric_limits<double>::quiet_NaN()};
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

[[nodiscard]] double sanitizedPositive(const double value, const double fallback,
                                       const double min_value,
                                       const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] double
effectiveVelocityDeltaAccelMps2(const VelocityFollowerConfig& config) {
  const double max_accel = sanitizedPositive(config.max_accel_mps2, 3.0, 1.0e-6, 100.0);
  const double max_lateral =
      sanitizedPositive(config.max_lateral_accel_mps2, 3.0, 1.0e-6, 100.0);
  return std::min(max_accel, max_lateral);
}

[[nodiscard]] double
effectiveVelocityDeltaDecelMps2(const VelocityFollowerConfig& config) {
  return sanitizedPositive(config.max_decel_mps2, 4.0, 1.0e-6, 100.0);
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

[[nodiscard]] VelocityJerkLimitResult
limitVelocitySetpointJerk(const Point2 desired_velocity, const Point2 previous_velocity,
                          const bool previous_velocity_valid,
                          const Point2 previous_acceleration,
                          const bool previous_acceleration_valid, const double dt_s,
                          const double max_jerk_mps3) {
  VelocityJerkLimitResult result{};
  result.velocity = desired_velocity;
  if (!previous_velocity_valid || !finite2D(previous_velocity) ||
      !finite2D(desired_velocity)) {
    return result;
  }

  const double dt = sanitizedPositive(dt_s, 0.1, 1.0e-6, 10.0);
  const Point2 requested_acceleration =
      (desired_velocity - previous_velocity) * (1.0 / dt);
  result.acceleration = requested_acceleration;
  result.acceleration_norm_mps2 = norm(requested_acceleration);
  if (!previous_acceleration_valid || !finite2D(previous_acceleration)) {
    return result;
  }

  const double max_jerk = sanitizedPositive(max_jerk_mps3, 0.0, 0.0, 1000.0);
  if (!(max_jerk > 0.0)) {
    return result;
  }

  const double previous_speed = norm(previous_velocity);
  if (!(previous_speed > kTinyDistanceM)) {
    const VectorRateLimitResult limited_acceleration = limitVectorRate(
        requested_acceleration, previous_acceleration, true, dt, max_jerk);
    result.acceleration = limited_acceleration.value;
    result.acceleration_norm_mps2 = norm(result.acceleration);
    result.jerk_mps3 = std::isfinite(limited_acceleration.delta)
                           ? limited_acceleration.delta / dt
                           : std::numeric_limits<double>::quiet_NaN();
    result.velocity = previous_velocity + result.acceleration * dt;
    return result;
  }

  const Point2 forward = previous_velocity * (1.0 / previous_speed);
  const Point2 left_normal{-forward.y, forward.x};
  const double requested_longitudinal_accel = dot(requested_acceleration, forward);
  const double requested_lateral_accel = dot(requested_acceleration, left_normal);
  const double previous_longitudinal_accel = dot(previous_acceleration, forward);
  const double previous_lateral_accel = dot(previous_acceleration, left_normal);
  const double max_accel_delta = max_jerk * dt;
  double limited_longitudinal_accel = requested_longitudinal_accel;
  if (requested_longitudinal_accel >= previous_longitudinal_accel ||
      requested_longitudinal_accel >= 0.0) {
    limited_longitudinal_accel = std::clamp(
        requested_longitudinal_accel, previous_longitudinal_accel - max_accel_delta,
        previous_longitudinal_accel + max_accel_delta);
  }
  const double limited_lateral_accel =
      std::clamp(requested_lateral_accel, previous_lateral_accel - max_accel_delta,
                 previous_lateral_accel + max_accel_delta);
  result.acceleration =
      forward * limited_longitudinal_accel + left_normal * limited_lateral_accel;
  result.acceleration_norm_mps2 = norm(result.acceleration);
  result.jerk_mps3 = norm(result.acceleration - previous_acceleration) / dt;
  result.velocity = previous_velocity + result.acceleration * dt;
  return result;
}

} // namespace

VelocityVectorLimitResult limitVelocityVectorDelta(const Point2 desired_velocity,
                                                   const Point2 previous_velocity,
                                                   const bool previous_velocity_valid,
                                                   const double dt_s,
                                                   const double max_delta_mps2) {
  return limitVelocityVectorDelta(desired_velocity, previous_velocity,
                                  previous_velocity_valid, dt_s, max_delta_mps2,
                                  max_delta_mps2);
}

VelocityVectorLimitResult
limitVelocityVectorDelta(const Point2 desired_velocity, const Point2 previous_velocity,
                         const bool previous_velocity_valid, const double dt_s,
                         const double max_accel_mps2, const double max_decel_mps2) {
  VelocityVectorLimitResult result{};
  result.velocity = desired_velocity;
  if (!previous_velocity_valid || !finite2D(previous_velocity) ||
      !finite2D(desired_velocity)) {
    result.delta_mps = std::numeric_limits<double>::quiet_NaN();
    return result;
  }

  const double dt = sanitizedPositive(dt_s, 0.1, 0.0, 10.0);
  const double max_accel_delta =
      sanitizedPositive(max_accel_mps2, 3.0, 0.0, 100.0) * dt;
  const double max_decel_delta =
      sanitizedPositive(max_decel_mps2, max_accel_mps2, 0.0, 100.0) * dt;
  const Point2 delta = desired_velocity - previous_velocity;
  const double previous_speed = norm(previous_velocity);
  if (!(previous_speed > kTinyDistanceM)) {
    const double delta_norm = norm(delta);
    result.delta_mps = delta_norm;
    if (delta_norm <= max_accel_delta || !(delta_norm > kTinyDistanceM)) {
      return result;
    }
    result.velocity = previous_velocity + delta * (max_accel_delta / delta_norm);
    result.delta_mps = max_accel_delta;
    return result;
  }

  const Point2 forward{previous_velocity.x / previous_speed,
                       previous_velocity.y / previous_speed};
  const double longitudinal_delta = delta.x * forward.x + delta.y * forward.y;
  const Point2 longitudinal{forward.x * longitudinal_delta,
                            forward.y * longitudinal_delta};
  const Point2 lateral{delta.x - longitudinal.x, delta.y - longitudinal.y};
  const double limited_longitudinal_delta =
      std::clamp(longitudinal_delta, -max_decel_delta, max_accel_delta);
  Point2 limited_lateral = lateral;
  const double lateral_norm = norm(lateral);
  if (lateral_norm > max_accel_delta && lateral_norm > kTinyDistanceM) {
    limited_lateral = lateral * (max_accel_delta / lateral_norm);
  }
  result.velocity =
      previous_velocity + forward * limited_longitudinal_delta + limited_lateral;
  result.delta_mps = norm(result.velocity - previous_velocity);
  return result;
}

VelocitySmootherPlan smoothVelocityCommand(const VelocitySmootherInput& input,
                                           const VelocityFollowerConfig& config) {
  VelocitySmootherPlan plan{};
  if (!finite2D(input.desired_velocity_xy) || !std::isfinite(input.dt_s)) {
    return plan;
  }

  const VelocityVectorLimitResult limited_velocity = limitVelocityVectorDelta(
      input.desired_velocity_xy, input.previous_velocity_setpoint,
      input.previous_velocity_setpoint_valid, input.dt_s,
      effectiveVelocityDeltaAccelMps2(config), effectiveVelocityDeltaDecelMps2(config));
  const VelocityJerkLimitResult jerk_limited_velocity = limitVelocitySetpointJerk(
      limited_velocity.velocity, input.previous_velocity_setpoint,
      input.previous_velocity_setpoint_valid,
      input.previous_velocity_acceleration_setpoint,
      input.previous_velocity_acceleration_setpoint_valid, input.dt_s,
      config.max_velocity_jerk_mps3);

  plan.valid = true;
  plan.velocity_xy = jerk_limited_velocity.velocity;
  plan.velocity_setpoint_acceleration_xy = jerk_limited_velocity.acceleration;
  plan.velocity_delta_mps =
      input.previous_velocity_setpoint_valid &&
              finite2D(input.previous_velocity_setpoint)
          ? norm(plan.velocity_xy - input.previous_velocity_setpoint)
          : limited_velocity.delta_mps;
  plan.desired_velocity_delta_mps = norm(input.desired_velocity_xy - plan.velocity_xy);
  plan.velocity_setpoint_acceleration_mps2 =
      jerk_limited_velocity.acceleration_norm_mps2;
  plan.velocity_setpoint_jerk_mps3 = jerk_limited_velocity.jerk_mps3;
  return plan;
}

} // namespace drone_city_nav
