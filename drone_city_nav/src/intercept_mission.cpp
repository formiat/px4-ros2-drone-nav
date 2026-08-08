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

SweptVehicleSeparation sweptVehicleSeparation(
    const TimedVehicleState& first, const TimedVehicleState& second,
    const std::optional<TimedVehicleState>& previous_first,
    const std::optional<TimedVehicleState>& previous_second) noexcept {
  const Vec3 current_relative = subtract(first.position, second.position);
  SweptVehicleSeparation result;
  result.current_m = norm(current_relative);
  result.minimum_m = result.current_m;
  if (!previous_first || !previous_second) {
    return result;
  }
  const Vec3 previous_relative =
      subtract(previous_first->position, previous_second->position);
  const Vec3 delta{current_relative.x - previous_relative.x,
                   current_relative.y - previous_relative.y,
                   current_relative.z - previous_relative.z};
  const double delta_norm_squared =
      delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
  if (delta_norm_squared <= 1.0e-12) {
    const double previous_m = norm(previous_relative);
    if (previous_m < result.minimum_m) {
      result.minimum_m = previous_m;
      result.interpolation_fraction = 0.0;
    }
    return result;
  }
  const double projection =
      -(previous_relative.x * delta.x + previous_relative.y * delta.y +
        previous_relative.z * delta.z) /
      delta_norm_squared;
  const double fraction = std::clamp(projection, 0.0, 1.0);
  const Vec3 closest{previous_relative.x + fraction * delta.x,
                     previous_relative.y + fraction * delta.y,
                     previous_relative.z + fraction * delta.z};
  const double closest_m = norm(closest);
  if (closest_m < result.minimum_m) {
    result.minimum_m = closest_m;
    result.interpolation_fraction = fraction;
  }
  return result;
}

double minimumSweptVehicleSeparation(
    const TimedVehicleState& first, const TimedVehicleState& second,
    const std::optional<TimedVehicleState>& previous_first,
    const std::optional<TimedVehicleState>& previous_second) noexcept {
  return sweptVehicleSeparation(first, second, previous_first, previous_second)
      .minimum_m;
}

bool interceptMissionReady(const InterceptMissionReadiness& readiness) noexcept {
  return readiness.interceptor_navigation_ready && readiness.evader_navigation_ready &&
         readiness.interceptor_world_ready && readiness.evader_world_ready &&
         readiness.target_track_ready;
}

InterceptStateAdjudicationLifecycle::InterceptStateAdjudicationLifecycle(
    const InterceptStateAdjudicationConfig& config) {
  if (!(config.maximum_state_age_s > 0.0) ||
      !(config.maximum_degraded_duration_s > 0.0) ||
      !std::isfinite(config.maximum_state_age_s) ||
      !std::isfinite(config.maximum_degraded_duration_s)) {
    throw std::invalid_argument{"invalid intercept state adjudication configuration"};
  }
  maximum_state_age_ns_ = static_cast<std::int64_t>(config.maximum_state_age_s * 1.0e9);
  maximum_degraded_duration_ns_ =
      static_cast<std::int64_t>(config.maximum_degraded_duration_s * 1.0e9);
}

double InterceptStateAdjudicationLifecycle::stateAgeSeconds(
    const std::int64_t now_ns, const TimedVehicleState& state) noexcept {
  if (state.stamp_ns <= 0 || now_ns < state.stamp_ns) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(now_ns - state.stamp_ns) * 1.0e-9;
}

bool InterceptStateAdjudicationLifecycle::stateFresh(
    const std::int64_t now_ns, const TimedVehicleState& state) const noexcept {
  return state.stamp_ns > 0 && now_ns >= state.stamp_ns &&
         now_ns - state.stamp_ns <= maximum_state_age_ns_;
}

InterceptStateAdjudicationUpdate
InterceptStateAdjudicationLifecycle::update(const std::int64_t now_ns,
                                            const TimedVehicleState& interceptor,
                                            const TimedVehicleState& evader) noexcept {
  InterceptStateAdjudicationUpdate result{
      .interceptor_fresh = stateFresh(now_ns, interceptor),
      .evader_fresh = stateFresh(now_ns, evader),
      .interceptor_age_s = stateAgeSeconds(now_ns, interceptor),
      .evader_age_s = stateAgeSeconds(now_ns, evader),
  };
  if (result.interceptor_fresh && result.evader_fresh) {
    result.newly_recovered = degraded_since_ns_.has_value();
    degraded_since_ns_.reset();
    prolonged_failure_reported_ = false;
    return result;
  }

  result.status = InterceptStateAdjudicationStatus::kDegraded;
  if (!degraded_since_ns_.has_value() || now_ns < *degraded_since_ns_) {
    degraded_since_ns_ = now_ns;
    result.newly_degraded = true;
  }
  result.degraded_duration_s =
      static_cast<double>(now_ns - *degraded_since_ns_) * 1.0e-9;
  if (now_ns - *degraded_since_ns_ >= maximum_degraded_duration_ns_) {
    result.status = InterceptStateAdjudicationStatus::kProlongedFailure;
    result.newly_prolonged_failure = !prolonged_failure_reported_;
    prolonged_failure_reported_ = true;
  }
  return result;
}

InterceptorHoldConfirmation::InterceptorHoldConfirmation(
    const InterceptorHoldConfig& config)
    : config_{config} {
  if (!(config_.position_tolerance_m > 0.0) || !(config_.maximum_speed_mps >= 0.0) ||
      !(config_.confirmation_duration_s >= 0.0)) {
    throw std::invalid_argument{"invalid interceptor hold configuration"};
  }
}

InterceptorHoldUpdate
InterceptorHoldConfirmation::update(const TimedVehicleState& interceptor,
                                    const std::optional<Point3>& active_hold_position) {
  InterceptorHoldUpdate result;
  result.position_error_m =
      active_hold_position && finite(*active_hold_position) &&
              interceptor.position_valid && finite(interceptor.position)
          ? norm(subtract(interceptor.position, *active_hold_position))
          : std::numeric_limits<double>::infinity();
  result.speed_mps = interceptor.velocity_valid && finite(interceptor.velocity)
                         ? norm(interceptor.velocity)
                         : std::numeric_limits<double>::infinity();
  if (confirmed_) {
    result.confirmed = true;
    return result;
  }

  const bool stable = active_hold_position.has_value() && interceptor.stamp_ns > 0 &&
                      interceptor.armed && interceptor.airborne &&
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
  return minimumSweptVehicleSeparation(interceptor, evader, previous_interceptor_,
                                       previous_evader_);
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

  const SweptVehicleSeparation separation = sweptVehicleSeparation(
      interceptor, evader, previous_interceptor_, previous_evader_);
  result.separation_m = separation.minimum_m;
  result.current_separation_m = separation.current_m;
  result.interpolation_fraction = separation.interpolation_fraction;
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

void InterceptMissionEvaluator::resetTemporalContinuity() noexcept {
  previous_interceptor_.reset();
  previous_evader_.reset();
}

MultiInterceptMissionEvaluator::MultiInterceptMissionEvaluator(
    const Point3& evader_goal, const std::size_t interceptor_count,
    const InterceptMissionConfig& config)
    : evader_goal_{evader_goal},
      config_{config},
      previous_interceptors_(interceptor_count) {
  if (!finite(evader_goal_) || interceptor_count == 0U ||
      !(config_.capture_radius_m > 0.0) || !(config_.evader_goal_radius_m > 0.0)) {
    throw std::invalid_argument{"invalid multi-intercept mission configuration"};
  }
}

MultiInterceptMissionUpdate MultiInterceptMissionEvaluator::update(
    const std::span<const TimedVehicleState> interceptors,
    const TimedVehicleState& evader) {
  if (interceptors.size() != previous_interceptors_.size()) {
    throw std::invalid_argument{"interceptor count changed during mission"};
  }
  MultiInterceptMissionUpdate result{
      .outcome = outcome_,
      .capture_detected = capture_detected_,
      .capturing_interceptor_index = std::nullopt,
      .separation_m = std::numeric_limits<double>::infinity(),
      .current_separation_m = std::numeric_limits<double>::infinity()};
  if (!evader.position_valid || !finite(evader.position)) {
    return result;
  }

  const bool evader_active = evader.armed && evader.airborne;
  double best_capture_separation = std::numeric_limits<double>::infinity();
  double best_capture_current_separation = std::numeric_limits<double>::infinity();
  double best_capture_fraction = 1.0;
  for (std::size_t index = 0; index < interceptors.size(); ++index) {
    const TimedVehicleState& interceptor = interceptors[index];
    if (!interceptor.position_valid || !finite(interceptor.position)) {
      continue;
    }
    const SweptVehicleSeparation separation = sweptVehicleSeparation(
        interceptor, evader, previous_interceptors_[index], previous_evader_);
    if (separation.minimum_m < result.separation_m) {
      result.separation_m = separation.minimum_m;
      result.current_separation_m = separation.current_m;
      result.interpolation_fraction = separation.interpolation_fraction;
    }
    const bool captured = evader_active && interceptor.armed && interceptor.airborne &&
                          separation.minimum_m <= config_.capture_radius_m;
    if (captured && separation.minimum_m < best_capture_separation) {
      result.capturing_interceptor_index = index;
      best_capture_separation = separation.minimum_m;
      best_capture_current_separation = separation.current_m;
      best_capture_fraction = separation.interpolation_fraction;
    }
  }
  if (result.capturing_interceptor_index.has_value()) {
    result.separation_m = best_capture_separation;
    result.current_separation_m = best_capture_current_separation;
    result.interpolation_fraction = best_capture_fraction;
  }

  const bool captured_now = result.capturing_interceptor_index.has_value();
  if (captured_now && !capture_detected_) {
    capture_detected_ = true;
    result.capture_detected = true;
    result.newly_captured = true;
  }
  if (outcome_ == InterceptMissionOutcome::kRunning && captured_now) {
    outcome_ = InterceptMissionOutcome::kIntercepted;
    result.outcome = outcome_;
    result.newly_terminal = true;
  } else if (outcome_ == InterceptMissionOutcome::kRunning && evader_active) {
    const double goal_distance = norm(subtract(evader.position, evader_goal_));
    if (goal_distance <= config_.evader_goal_radius_m) {
      outcome_ = InterceptMissionOutcome::kEvaderReachedGoal;
      result.outcome = outcome_;
      result.newly_terminal = true;
    }
  }

  for (std::size_t index = 0; index < interceptors.size(); ++index) {
    previous_interceptors_[index] = interceptors[index];
  }
  previous_evader_ = evader;
  return result;
}

void MultiInterceptMissionEvaluator::resetTemporalContinuity() noexcept {
  std::fill(previous_interceptors_.begin(), previous_interceptors_.end(), std::nullopt);
  previous_evader_.reset();
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
    case InterceptMissionOutcome::kEvaderCrashed:
      return "evader_crashed";
    case InterceptMissionOutcome::kNoInterceptorsRemaining:
      return "no_interceptors_remaining";
  }
  return "unknown";
}

} // namespace drone_city_nav
