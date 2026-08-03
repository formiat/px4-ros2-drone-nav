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

} // namespace

InterceptMissionEvaluator::InterceptMissionEvaluator(
    const Point3& evader_goal, const InterceptMissionConfig& config)
    : evader_goal_{evader_goal},
      config_{config} {
  if (!finite(evader_goal_) || !(config_.capture_radius_m > 0.0) ||
      !(config_.evader_goal_radius_m > 0.0) ||
      config_.evader_goal_stop_speed_mps < 0.0 || config_.evader_goal_hold_s < 0.0) {
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
  InterceptMissionUpdate result{.outcome = outcome_};
  if (outcome_ != InterceptMissionOutcome::kRunning || !interceptor.position_valid ||
      !evader.position_valid || !finite(interceptor.position) ||
      !finite(evader.position)) {
    return result;
  }

  result.separation_m = sweptSeparation(interceptor, evader);
  const bool mission_active =
      interceptor.armed && evader.armed && interceptor.airborne && evader.airborne;
  if (mission_active && result.separation_m <= config_.capture_radius_m) {
    outcome_ = InterceptMissionOutcome::kIntercepted;
    result.outcome = outcome_;
    result.newly_terminal = true;
  } else {
    const double goal_distance = norm(subtract(evader.position, evader_goal_));
    const double speed = evader.velocity_valid
                             ? norm(evader.velocity)
                             : std::numeric_limits<double>::infinity();
    const bool captured = mission_active &&
                          goal_distance <= config_.evader_goal_radius_m &&
                          speed <= config_.evader_goal_stop_speed_mps;
    if (!captured) {
      evader_goal_hold_started_ns_.reset();
    } else if (!evader_goal_hold_started_ns_) {
      evader_goal_hold_started_ns_ = evader.stamp_ns;
    } else if (evader.stamp_ns >= *evader_goal_hold_started_ns_ &&
               static_cast<double>(evader.stamp_ns - *evader_goal_hold_started_ns_) *
                       1.0e-9 >=
                   config_.evader_goal_hold_s) {
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
