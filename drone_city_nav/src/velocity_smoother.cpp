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

struct PathFrameVelocityLimitResult {
  Point2 velocity{};
  double delta_mps{std::numeric_limits<double>::quiet_NaN()};
  bool applied{false};
  double lateral_smoothing_factor{1.0};
  double lateral_response_accel_mps2{std::numeric_limits<double>::quiet_NaN()};
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

[[nodiscard]] double smoothStep(const double edge0, const double edge1,
                                const double value) noexcept {
  if (!(edge1 > edge0)) {
    return value >= edge1 ? 1.0 : 0.0;
  }
  const double t = std::clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
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
  return sanitizedPositive(config.max_accel_mps2, 3.0, 1.0e-6, 100.0);
}

[[nodiscard]] double
effectiveVelocityDeltaDecelMps2(const VelocityFollowerConfig& config) {
  return sanitizedPositive(config.max_decel_mps2, 4.0, 1.0e-6, 100.0);
}

[[nodiscard]] double
effectiveLateralResponseAccelMps2(const VelocityFollowerConfig& config,
                                  const double response_factor) {
  const double lateral_response_accel =
      sanitizedPositive(config.velocity_lateral_response_accel_mps2,
                        config.max_lateral_accel_mps2, 1.0e-6, 100.0);
  const double factor = sanitizedPositive(response_factor, 1.0, 1.0, 100.0);
  return lateral_response_accel * factor;
}

[[nodiscard]] double lateralSmoothingFactor(const VelocityFollowerConfig& config,
                                            const double speed_mps) noexcept {
  const double min_speed =
      sanitizedPositive(config.lateral_smoothing_min_speed_mps, 8.0, 0.0, 1000.0);
  const double full_speed =
      std::max(min_speed, sanitizedPositive(config.lateral_smoothing_full_speed_mps,
                                            20.0, 0.0, 1000.0));
  const double max_factor =
      sanitizedPositive(config.lateral_smoothing_max_factor, 1.0, 1.0, 100.0);
  return 1.0 + (max_factor - 1.0) * smoothStep(min_speed, full_speed, speed_mps);
}

[[nodiscard]] PathFrameVelocityLimitResult limitVelocityPathFrame(
    const Point2 desired_velocity, const Point2 previous_velocity,
    const bool previous_velocity_valid, const Point2 path_tangent, const double dt_s,
    const double max_accel_mps2, const double max_decel_mps2,
    const double lateral_response_accel_mps2, const VelocityFollowerConfig& config) {
  PathFrameVelocityLimitResult result{};
  result.velocity = desired_velocity;
  if (!previous_velocity_valid || !finite2D(previous_velocity) ||
      !finite2D(desired_velocity) || !finite2D(path_tangent)) {
    return result;
  }

  const Point2 tangent = normalized(path_tangent);
  if (!(norm(tangent) > kTinyDistanceM)) {
    return result;
  }

  result.applied = true;
  const double dt = sanitizedPositive(dt_s, 0.1, 0.0, 10.0);
  const double max_forward_accel_delta =
      sanitizedPositive(max_accel_mps2, 3.0, 0.0, 100.0) * dt;
  const double max_forward_decel_delta =
      sanitizedPositive(max_decel_mps2, max_accel_mps2, 0.0, 100.0) * dt;
  const double previous_speed = norm(previous_velocity);
  result.lateral_smoothing_factor = lateralSmoothingFactor(config, previous_speed);
  result.lateral_response_accel_mps2 =
      sanitizedPositive(lateral_response_accel_mps2, max_accel_mps2, 0.0, 100.0) /
      result.lateral_smoothing_factor;
  const double max_lateral_delta = result.lateral_response_accel_mps2 * dt;

  const Point2 normal{-tangent.y, tangent.x};
  const double previous_forward = dot(previous_velocity, tangent);
  const double desired_forward = dot(desired_velocity, tangent);
  const double previous_lateral = dot(previous_velocity, normal);
  const double desired_lateral = dot(desired_velocity, normal);
  const double limited_forward =
      previous_forward + std::clamp(desired_forward - previous_forward,
                                    -max_forward_decel_delta, max_forward_accel_delta);

  const double limited_lateral =
      previous_lateral + std::clamp(desired_lateral - previous_lateral,
                                    -max_lateral_delta, max_lateral_delta);

  result.velocity = tangent * limited_forward + normal * limited_lateral;
  result.delta_mps = norm(result.velocity - previous_velocity);
  return result;
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

[[nodiscard]] VelocityJerkLimitResult limitVelocitySetpointJerk(
    const Point2 desired_velocity, const Point2 previous_velocity,
    const bool previous_velocity_valid, const Point2 previous_acceleration,
    const bool previous_acceleration_valid, const double dt_s,
    const double max_longitudinal_jerk_mps3, const double max_lateral_jerk_mps3) {
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

  const double max_longitudinal_jerk =
      sanitizedPositive(max_longitudinal_jerk_mps3, 0.0, 0.0, 1000.0);
  const double max_lateral_jerk =
      sanitizedPositive(max_lateral_jerk_mps3, max_longitudinal_jerk, 0.0, 1000.0);
  if (!(max_longitudinal_jerk > 0.0) && !(max_lateral_jerk > 0.0)) {
    return result;
  }

  const double previous_speed = norm(previous_velocity);
  if (!(previous_speed > kTinyDistanceM)) {
    const double max_jerk = std::max(max_longitudinal_jerk, max_lateral_jerk);
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
  const double max_longitudinal_accel_delta = max_longitudinal_jerk * dt;
  const double max_lateral_accel_delta = max_lateral_jerk * dt;
  const double limited_longitudinal_accel =
      std::clamp(requested_longitudinal_accel,
                 previous_longitudinal_accel - max_longitudinal_accel_delta,
                 previous_longitudinal_accel + max_longitudinal_accel_delta);
  const double limited_lateral_accel = std::clamp(
      requested_lateral_accel, previous_lateral_accel - max_lateral_accel_delta,
      previous_lateral_accel + max_lateral_accel_delta);
  result.acceleration =
      forward * limited_longitudinal_accel + left_normal * limited_lateral_accel;
  result.acceleration_norm_mps2 = norm(result.acceleration);
  result.jerk_mps3 = norm(result.acceleration - previous_acceleration) / dt;
  result.velocity = previous_velocity + result.acceleration * dt;
  return result;
}

[[nodiscard]] VelocityVectorLimitResult limitVelocityVectorDeltaWithLateral(
    const Point2 desired_velocity, const Point2 previous_velocity,
    const bool previous_velocity_valid, const double dt_s, const double max_accel_mps2,
    const double max_decel_mps2, const double max_lateral_accel_mps2) {
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
  const double max_lateral_delta =
      sanitizedPositive(max_lateral_accel_mps2, max_accel_mps2, 0.0, 100.0) * dt;
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
  if (lateral_norm > max_lateral_delta && lateral_norm > kTinyDistanceM) {
    limited_lateral = lateral * (max_lateral_delta / lateral_norm);
  }
  result.velocity =
      previous_velocity + forward * limited_longitudinal_delta + limited_lateral;
  result.delta_mps = norm(result.velocity - previous_velocity);
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
  return limitVelocityVectorDeltaWithLateral(
      desired_velocity, previous_velocity, previous_velocity_valid, dt_s,
      max_accel_mps2, max_decel_mps2, max_accel_mps2);
}

VelocitySmootherPlan smoothVelocityCommand(const VelocitySmootherInput& input,
                                           const VelocityFollowerConfig& config) {
  VelocitySmootherPlan plan{};
  if (!finite2D(input.desired_velocity_xy) || !std::isfinite(input.dt_s)) {
    return plan;
  }

  const double max_accel_mps2 = effectiveVelocityDeltaAccelMps2(config);
  const double max_decel_mps2 = effectiveVelocityDeltaDecelMps2(config);
  const double lateral_response_accel_mps2 =
      effectiveLateralResponseAccelMps2(config, input.lateral_response_factor);
  const PathFrameVelocityLimitResult path_frame_limited_velocity =
      limitVelocityPathFrame(
          input.desired_velocity_xy, input.previous_velocity_setpoint,
          input.previous_velocity_setpoint_valid, input.path_tangent, input.dt_s,
          max_accel_mps2, max_decel_mps2, lateral_response_accel_mps2, config);
  const VelocityVectorLimitResult fallback_limited_velocity =
      path_frame_limited_velocity.applied
          ? VelocityVectorLimitResult{.velocity = path_frame_limited_velocity.velocity,
                                      .delta_mps =
                                          path_frame_limited_velocity.delta_mps}
          : limitVelocityVectorDeltaWithLateral(
                input.desired_velocity_xy, input.previous_velocity_setpoint,
                input.previous_velocity_setpoint_valid, input.dt_s, max_accel_mps2,
                max_decel_mps2, lateral_response_accel_mps2);
  const VelocityJerkLimitResult jerk_limited_velocity = limitVelocitySetpointJerk(
      fallback_limited_velocity.velocity, input.previous_velocity_setpoint,
      input.previous_velocity_setpoint_valid,
      input.previous_velocity_acceleration_setpoint,
      input.previous_velocity_acceleration_setpoint_valid, input.dt_s,
      config.max_velocity_jerk_mps3, config.max_lateral_velocity_jerk_mps3);

  plan.valid = true;
  plan.velocity_xy = jerk_limited_velocity.velocity;
  plan.velocity_setpoint_acceleration_xy = jerk_limited_velocity.acceleration;
  plan.velocity_delta_mps =
      input.previous_velocity_setpoint_valid &&
              finite2D(input.previous_velocity_setpoint)
          ? norm(plan.velocity_xy - input.previous_velocity_setpoint)
          : fallback_limited_velocity.delta_mps;
  plan.desired_velocity_delta_mps = norm(input.desired_velocity_xy - plan.velocity_xy);
  plan.velocity_setpoint_acceleration_mps2 =
      jerk_limited_velocity.acceleration_norm_mps2;
  plan.velocity_setpoint_jerk_mps3 = jerk_limited_velocity.jerk_mps3;
  plan.path_frame_lateral_smoothing_applied = path_frame_limited_velocity.applied;
  plan.lateral_smoothing_factor =
      path_frame_limited_velocity.applied
          ? path_frame_limited_velocity.lateral_smoothing_factor
          : 1.0;
  plan.smoother_lateral_response_accel_mps2 =
      path_frame_limited_velocity.applied
          ? path_frame_limited_velocity.lateral_response_accel_mps2
          : lateral_response_accel_mps2;
  return plan;
}

} // namespace drone_city_nav
