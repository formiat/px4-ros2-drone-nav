#include "drone_city_nav/offboard_velocity_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

constexpr double kTinySpeedMps = 1.0e-6;

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] double norm(const Point2 point) noexcept {
  return std::hypot(point.x, point.y);
}

[[nodiscard]] Point2 scale(const Point2 point, const double factor) noexcept {
  return Point2{point.x * factor, point.y * factor};
}

[[nodiscard]] Point2 add(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x + rhs.x, lhs.y + rhs.y};
}

[[nodiscard]] Point2 subtract(const Point2 lhs, const Point2 rhs) noexcept {
  return Point2{lhs.x - rhs.x, lhs.y - rhs.y};
}

[[nodiscard]] Point2 rotate(const Point2 point, const double angle_rad) noexcept {
  const double cosine = std::cos(angle_rad);
  const double sine = std::sin(angle_rad);
  return Point2{(point.x * cosine) - (point.y * sine),
                (point.x * sine) + (point.y * cosine)};
}

[[nodiscard]] Point2 limitVectorDelta(const Point2 previous, const Point2 desired,
                                      const double max_delta_mps,
                                      bool& limited) noexcept {
  limited = false;
  if (!std::isfinite(max_delta_mps) || max_delta_mps < 0.0) {
    return previous;
  }
  const Point2 delta = subtract(desired, previous);
  const double delta_norm = norm(delta);
  if (delta_norm <= max_delta_mps || delta_norm <= kTinySpeedMps) {
    return desired;
  }

  limited = true;
  return add(previous, scale(delta, max_delta_mps / delta_norm));
}

[[nodiscard]] Point2 limitHeadingRate(const Point2 previous, const Point2 desired,
                                      const double max_heading_delta_rad,
                                      bool& limited) noexcept {
  limited = false;
  const double previous_speed = norm(previous);
  const double desired_speed = norm(desired);
  if (!(previous_speed > kTinySpeedMps) || !(desired_speed > kTinySpeedMps) ||
      !(max_heading_delta_rad >= 0.0) || !std::isfinite(max_heading_delta_rad)) {
    return desired;
  }

  const double cross = (previous.x * desired.y) - (previous.y * desired.x);
  const double dot = (previous.x * desired.x) + (previous.y * desired.y);
  const double angle = std::atan2(cross, dot);
  if (std::abs(angle) <= max_heading_delta_rad) {
    return desired;
  }

  limited = true;
  const double clamped_angle = std::copysign(max_heading_delta_rad, angle);
  const Point2 previous_direction = scale(previous, 1.0 / previous_speed);
  return scale(rotate(previous_direction, clamped_angle), desired_speed);
}

} // namespace

OffboardVelocityLimiter::OffboardVelocityLimiter(const VelocityLimiterConfig& config)
    : config_{config} {
}

void OffboardVelocityLimiter::setConfig(const VelocityLimiterConfig& config) {
  config_ = config;
}

const VelocityLimiterConfig& OffboardVelocityLimiter::config() const noexcept {
  return config_;
}

void OffboardVelocityLimiter::reset() noexcept {
  previous_velocity_mps_ = Point2{};
  previous_velocity_valid_ = false;
}

VelocityLimiterOutput OffboardVelocityLimiter::update(const Point2 desired_velocity_mps,
                                                      const double dt_s) {
  VelocityLimiterOutput output{};
  if (!finite2D(desired_velocity_mps) || !(dt_s > 0.0) || !std::isfinite(dt_s)) {
    reset();
    return output;
  }

  if (!previous_velocity_valid_) {
    previous_velocity_mps_ = desired_velocity_mps;
    previous_velocity_valid_ = true;
    output.velocity_mps = desired_velocity_mps;
    return output;
  }

  output.raw_delta_mps = norm(subtract(desired_velocity_mps, previous_velocity_mps_));
  Point2 limited_velocity = desired_velocity_mps;
  if (config_.max_heading_rate_radps > 0.0 &&
      std::isfinite(config_.max_heading_rate_radps)) {
    limited_velocity = limitHeadingRate(previous_velocity_mps_, limited_velocity,
                                        config_.max_heading_rate_radps * dt_s,
                                        output.heading_rate_limited);
  }

  const double max_delta_mps = config_.max_vector_accel_mps2 > 0.0 &&
                                       std::isfinite(config_.max_vector_accel_mps2)
                                   ? config_.max_vector_accel_mps2 * dt_s
                                   : 0.0;
  limited_velocity = limitVectorDelta(previous_velocity_mps_, limited_velocity,
                                      max_delta_mps, output.vector_delta_limited);
  output.applied_delta_mps = norm(subtract(limited_velocity, previous_velocity_mps_));
  output.velocity_mps = limited_velocity;
  previous_velocity_mps_ = limited_velocity;
  return output;
}

} // namespace drone_city_nav
