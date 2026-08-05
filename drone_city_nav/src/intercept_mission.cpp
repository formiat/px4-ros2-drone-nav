#include "drone_city_nav/intercept_mission.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] double norm(const Vec3& value) noexcept {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

[[nodiscard]] Vec3 subtract(const Point3& first, const Point3& second) noexcept {
  return Vec3{first.x - second.x, first.y - second.y, first.z - second.z};
}

[[nodiscard]] bool finite(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finite(const Vec3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

} // namespace

InterceptorHoldConfirmation::InterceptorHoldConfirmation(
    const Point3& hold_position, const InterceptorHoldConfig& config)
    : hold_position_{hold_position},
      config_{config} {
  if (!finite(hold_position_) || !(config_.position_tolerance_m > 0.0) ||
      !(config_.maximum_speed_mps >= 0.0) ||
      !(config_.confirmation_duration_s >= 0.0)) {
    throw std::invalid_argument{"invalid interceptor hold configuration"};
  }
}

InterceptorHoldUpdate
InterceptorHoldConfirmation::update(const TimedVehicleState& interceptor) {
  InterceptorHoldUpdate result;
  result.position_error_m = interceptor.position_valid && finite(interceptor.position)
                                ? norm(subtract(interceptor.position, hold_position_))
                                : std::numeric_limits<double>::infinity();
  result.speed_mps = interceptor.velocity_valid && finite(interceptor.velocity)
                         ? norm(interceptor.velocity)
                         : std::numeric_limits<double>::infinity();
  if (confirmed_) {
    result.confirmed = true;
    return result;
  }

  const bool stable = interceptor.stamp_ns > 0 && interceptor.armed &&
                      interceptor.airborne &&
                      result.position_error_m <= config_.position_tolerance_m &&
                      result.speed_mps <= config_.maximum_speed_mps;
  if (!stable) {
    stable_since_ns_.reset();
    return result;
  }
  if (!stable_since_ns_ || interceptor.stamp_ns < *stable_since_ns_) {
    stable_since_ns_ = interceptor.stamp_ns;
    return result;
  }

  const double stable_duration_s =
      static_cast<double>(interceptor.stamp_ns - *stable_since_ns_) * 1.0e-9;
  if (stable_duration_s >= config_.confirmation_duration_s) {
    confirmed_ = true;
    result.confirmed = true;
    result.newly_confirmed = true;
  }
  return result;
}

InterceptMissionEvaluator::InterceptMissionEvaluator(
    const Point3& evader_goal, const InterceptMissionConfig& config)
    : evader_goal_{evader_goal},
      config_{config} {
  if (!finite(evader_goal_) || !(config_.capture_radius_m > 0.0) ||
      !(config_.evader_goal_radius_m > 0.0)) {
    throw std::invalid_argument{"invalid intercept mission configuration"};
  }
}

double InterceptMissionEvaluator::sweptSeparation(
    const TimedVehicleState& interceptor,
    const TimedVehicleState& evader) const noexcept {
  const Vec3 current_relative = subtract(interceptor.position, evader.position);
  double minimum = norm(current_relative);
  if (!previous_interceptor_ || !previous_evader_) {
    return minimum;
  }
  const Vec3 previous_relative =
      subtract(previous_interceptor_->position, previous_evader_->position);
  const Vec3 delta{current_relative.x - previous_relative.x,
                   current_relative.y - previous_relative.y,
                   current_relative.z - previous_relative.z};
  const double delta_norm_squared =
      delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
  if (delta_norm_squared <= 1.0e-12) {
    return std::min(minimum, norm(previous_relative));
  }
  const double projection =
      -(previous_relative.x * delta.x + previous_relative.y * delta.y +
        previous_relative.z * delta.z) /
      delta_norm_squared;
  const double fraction = std::clamp(projection, 0.0, 1.0);
  const Vec3 closest{previous_relative.x + fraction * delta.x,
                     previous_relative.y + fraction * delta.y,
                     previous_relative.z + fraction * delta.z};
  return std::min(minimum, norm(closest));
}

InterceptMissionUpdate
InterceptMissionEvaluator::update(const TimedVehicleState& interceptor,
                                  const TimedVehicleState& evader) {
  InterceptMissionUpdate result{.outcome = outcome_,
                                .capture_detected = capture_detected_};
  if (!interceptor.position_valid || !evader.position_valid ||
      !finite(interceptor.position) || !finite(evader.position)) {
    return result;
  }

  result.separation_m = sweptSeparation(interceptor, evader);
  const bool mission_active =
      interceptor.armed && evader.armed && interceptor.airborne && evader.airborne;
  const bool captured_now =
      mission_active && result.separation_m <= config_.capture_radius_m;
  if (captured_now && !capture_detected_) {
    capture_detected_ = true;
    result.capture_detected = true;
    result.newly_captured = true;
  }

  if (outcome_ == InterceptMissionOutcome::kRunning && captured_now) {
    outcome_ = InterceptMissionOutcome::kIntercepted;
    result.outcome = outcome_;
    result.newly_terminal = true;
  } else if (outcome_ == InterceptMissionOutcome::kRunning) {
    const double goal_distance = norm(subtract(evader.position, evader_goal_));
    if (mission_active && goal_distance <= config_.evader_goal_radius_m) {
      outcome_ = InterceptMissionOutcome::kEvaderReachedGoal;
      result.outcome = outcome_;
      result.newly_terminal = true;
    }
  }

  previous_interceptor_ = interceptor;
  previous_evader_ = evader;
  return result;
}

const char*
interceptMissionOutcomeName(const InterceptMissionOutcome outcome) noexcept {
  switch (outcome) {
    case InterceptMissionOutcome::kRunning:
      return "running";
    case InterceptMissionOutcome::kIntercepted:
      return "intercepted";
    case InterceptMissionOutcome::kEvaderReachedGoal:
      return "evader_reached_goal";
  }
  return "unknown";
}

} // namespace drone_city_nav
