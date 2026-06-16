#include "drone_city_nav/offboard_speed_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace drone_city_nav {
namespace {

constexpr double kEpsilon = 1.0e-6;

[[nodiscard]] bool finiteNonNegative(const double value) noexcept {
  return std::isfinite(value) && value >= 0.0;
}

[[nodiscard]] double boundedDesiredSpeed(const SpeedControllerConfig& config) noexcept {
  if (!finiteNonNegative(config.desired_speed_mps)) {
    return 0.0;
  }
  return config.desired_speed_mps;
}

[[nodiscard]] double boundedMinimum(const double minimum,
                                    const double desired) noexcept {
  if (!finiteNonNegative(minimum)) {
    return 0.0;
  }
  return std::clamp(minimum, 0.0, desired);
}

void applyLimit(double& allowed_speed, SpeedLimitReason& reason, const double limit,
                const SpeedLimitReason limit_reason) noexcept {
  if (std::isfinite(limit) && limit + kEpsilon < allowed_speed) {
    allowed_speed = std::max(0.0, limit);
    reason = limit_reason;
  }
}

[[nodiscard]] SpeedControllerOutput
holdOutput(const SpeedLimitReason reason = SpeedLimitReason::kHold) noexcept {
  SpeedControllerOutput output{};
  output.limit_reason = reason;
  return output;
}

} // namespace

const char* speedLimitReasonName(const SpeedLimitReason reason) noexcept {
  switch (reason) {
    case SpeedLimitReason::kHold:
      return "hold";
    case SpeedLimitReason::kInvalidInput:
      return "invalid_input";
    case SpeedLimitReason::kCruise:
      return "cruise";
    case SpeedLimitReason::kAcceleration:
      return "acceleration";
    case SpeedLimitReason::kGoal:
      return "goal";
    case SpeedLimitReason::kTurn:
      return "turn";
    case SpeedLimitReason::kClearance:
      return "clearance";
    case SpeedLimitReason::kHardStepCap:
      return "hard_step_cap";
    case SpeedLimitReason::kTrackingOverspeed:
      return "tracking_overspeed";
  }
  return "unknown";
}

double brakingLimitedSpeedMps(const SpeedControllerConfig& config,
                              const double distance_to_goal_m) noexcept {
  if (!std::isfinite(distance_to_goal_m) || distance_to_goal_m < 0.0 ||
      !(config.max_accel_mps2 > 0.0) || !std::isfinite(config.max_accel_mps2)) {
    return 0.0;
  }

  const double margin = finiteNonNegative(config.braking_safety_margin_m)
                            ? config.braking_safety_margin_m
                            : 0.0;
  const double remaining_distance_m = std::max(distance_to_goal_m - margin, 0.0);
  double limit = std::sqrt(2.0 * config.max_accel_mps2 * remaining_distance_m);

  if (finiteNonNegative(config.goal_slowdown_radius_m) &&
      config.goal_slowdown_radius_m > 0.0 &&
      distance_to_goal_m < config.goal_slowdown_radius_m) {
    const double desired = boundedDesiredSpeed(config);
    const double ratio =
        std::clamp(distance_to_goal_m / config.goal_slowdown_radius_m, 0.0, 1.0);
    limit = std::min(limit, desired * ratio);
  }

  return limit;
}

double turnLimitedSpeedMps(const SpeedControllerConfig& config,
                           const double turn_angle_rad) noexcept {
  const double desired = boundedDesiredSpeed(config);
  if (!std::isfinite(turn_angle_rad) || desired <= 0.0) {
    return 0.0;
  }

  const double threshold =
      std::clamp(config.turn_slowdown_angle_rad, 0.0, std::numbers::pi);
  const double angle = std::clamp(std::abs(turn_angle_rad), 0.0, std::numbers::pi);
  if (angle <= threshold) {
    return desired;
  }

  const double minimum = boundedMinimum(config.turn_slowdown_min_speed_mps, desired);
  constexpr double kFullSlowdownAngleRad = std::numbers::pi * 0.5;
  if (threshold >= kFullSlowdownAngleRad || angle >= kFullSlowdownAngleRad) {
    return minimum;
  }

  const double ratio =
      (angle - threshold) / std::max(kFullSlowdownAngleRad - threshold, kEpsilon);
  return desired + std::clamp(ratio, 0.0, 1.0) * (minimum - desired);
}

double clearanceLimitedSpeedMps(const SpeedControllerConfig& config,
                                const double local_clearance_m) noexcept {
  const double desired = boundedDesiredSpeed(config);
  if (!std::isfinite(local_clearance_m) || desired <= 0.0) {
    return desired;
  }

  const double slowdown_radius =
      finiteNonNegative(config.narrow_clearance_slowdown_radius_m)
          ? config.narrow_clearance_slowdown_radius_m
          : 0.0;
  if (slowdown_radius <= 0.0 || local_clearance_m >= slowdown_radius) {
    return desired;
  }

  const double minimum = boundedMinimum(config.narrow_clearance_min_speed_mps, desired);
  const double ratio = std::clamp(local_clearance_m / slowdown_radius, 0.0, 1.0);
  return minimum + ratio * (desired - minimum);
}

double advanceToward(const double current, const double target,
                     const double max_delta) noexcept {
  if (!std::isfinite(current) || !std::isfinite(target) ||
      !finiteNonNegative(max_delta)) {
    return 0.0;
  }
  if (std::abs(target - current) <= max_delta) {
    return target;
  }
  return current + std::copysign(max_delta, target - current);
}

OffboardSpeedController::OffboardSpeedController(const SpeedControllerConfig& config)
    : config_{config} {
}

void OffboardSpeedController::setConfig(const SpeedControllerConfig& config) {
  config_ = config;
}

const SpeedControllerConfig& OffboardSpeedController::config() const noexcept {
  return config_;
}

void OffboardSpeedController::reset() noexcept {
  previous_requested_speed_mps_ = 0.0;
}

SpeedControllerOutput
OffboardSpeedController::update(const SpeedControllerInput& input) {
  if (input.hold_position) {
    reset();
    return holdOutput();
  }
  if (!configIsUsable() || !std::isfinite(input.controller_dt_s) ||
      !(input.controller_dt_s > 0.0) || !finiteNonNegative(input.distance_to_goal_m) ||
      !std::isfinite(input.turn_angle_rad)) {
    reset();
    return holdOutput(SpeedLimitReason::kInvalidInput);
  }

  const double desired = boundedDesiredSpeed(config_);
  SpeedLimitReason reason = SpeedLimitReason::kCruise;
  double allowed_speed = desired;

  SpeedLimitBreakdown limits{};
  limits.goal_limit_mps = brakingLimitedSpeedMps(config_, input.distance_to_goal_m);
  limits.turn_limit_mps = turnLimitedSpeedMps(config_, input.turn_angle_rad);
  limits.clearance_limit_mps =
      clearanceLimitedSpeedMps(config_, input.local_clearance_m);
  limits.step_cap_limit_mps =
      config_.max_commanded_target_step_m / input.controller_dt_s;

  applyLimit(allowed_speed, reason, limits.goal_limit_mps, SpeedLimitReason::kGoal);
  applyLimit(allowed_speed, reason, limits.turn_limit_mps, SpeedLimitReason::kTurn);
  applyLimit(allowed_speed, reason, limits.clearance_limit_mps,
             SpeedLimitReason::kClearance);

  const double max_speed_delta = config_.max_accel_mps2 * input.controller_dt_s;
  double requested_speed =
      advanceToward(previous_requested_speed_mps_, allowed_speed, max_speed_delta);
  if (previous_requested_speed_mps_ + kEpsilon < allowed_speed &&
      requested_speed + kEpsilon < allowed_speed) {
    reason = SpeedLimitReason::kAcceleration;
  }

  const double step_cap_speed = std::max(0.0, limits.step_cap_limit_mps);
  if (config_.min_command_speed_mps > 0.0 && requested_speed > 0.0 &&
      allowed_speed > 0.0 && step_cap_speed > 0.0) {
    const double requested_minimum =
        boundedMinimum(config_.min_command_speed_mps, desired);
    const double safe_minimum =
        std::min(requested_minimum, std::min(allowed_speed, step_cap_speed));
    requested_speed = std::max(requested_speed, safe_minimum);
  }

  if (allowed_speed + kEpsilon < requested_speed) {
    requested_speed = allowed_speed;
  }
  if (finiteNonNegative(input.actual_speed_mps) &&
      input.actual_speed_mps > allowed_speed + max_speed_delta) {
    requested_speed =
        std::min(requested_speed, std::max(0.0, allowed_speed - max_speed_delta));
    reason = SpeedLimitReason::kTrackingOverspeed;
  }
  if (step_cap_speed + kEpsilon < requested_speed) {
    requested_speed = step_cap_speed;
    reason = SpeedLimitReason::kHardStepCap;
  }

  previous_requested_speed_mps_ = requested_speed;

  SpeedControllerOutput output{};
  output.requested_speed_mps = requested_speed;
  output.allowed_speed_mps = allowed_speed;
  output.target_step_m = requested_speed * input.controller_dt_s;
  output.braking_distance_m =
      (requested_speed * requested_speed) / (2.0 * config_.max_accel_mps2);
  output.limits = limits;
  output.limit_reason = reason;
  return output;
}

bool OffboardSpeedController::configIsUsable() const noexcept {
  return finiteNonNegative(config_.desired_speed_mps) &&
         std::isfinite(config_.max_accel_mps2) && config_.max_accel_mps2 > 0.0 &&
         finiteNonNegative(config_.max_commanded_target_step_m);
}

} // namespace drone_city_nav
