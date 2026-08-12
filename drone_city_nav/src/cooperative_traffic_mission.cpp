#include "drone_city_nav/cooperative_traffic_mission.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finite(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finite(const Vec3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] double distance(const Point3& first, const Point3& second) noexcept {
  return std::hypot(std::hypot(first.x - second.x, first.y - second.y),
                    first.z - second.z);
}

[[nodiscard]] double speed(const TimedVehicleState& state) noexcept {
  return std::hypot(std::hypot(state.velocity.x, state.velocity.y), state.velocity.z);
}

[[nodiscard]] bool continuityAvailable(const TimedVehicleState& current,
                                       const std::optional<TimedVehicleState>& previous,
                                       const std::int64_t maximum_gap_ns) noexcept {
  return previous.has_value() && current.stamp_ns > previous->stamp_ns &&
         current.stamp_ns - previous->stamp_ns <= maximum_gap_ns;
}

[[nodiscard]] bool separating(const TimedVehicleState& first,
                              const TimedVehicleState& second) noexcept {
  if (!first.position_valid || !second.position_valid || !first.velocity_valid ||
      !second.velocity_valid || !finite(first.position) || !finite(second.position) ||
      !finite(first.velocity) || !finite(second.velocity)) {
    return false;
  }
  const Vec3 relative_position{first.position.x - second.position.x,
                               first.position.y - second.position.y,
                               first.position.z - second.position.z};
  const Vec3 relative_velocity{first.velocity.x - second.velocity.x,
                               first.velocity.y - second.velocity.y,
                               first.velocity.z - second.velocity.z};
  return relative_position.x * relative_velocity.x +
             relative_position.y * relative_velocity.y +
             relative_position.z * relative_velocity.z >
         0.0;
}

} // namespace

CooperativeGoalHoldConfirmation::CooperativeGoalHoldConfirmation(
    const CooperativeGoalHoldConfig& config)
    : config_{config} {
  if (!(config_.goal_tolerance_m > 0.0) || !(config_.hold_position_tolerance_m > 0.0) ||
      !(config_.maximum_speed_mps >= 0.0) ||
      !(config_.confirmation_duration_s >= 0.0) ||
      !std::isfinite(config_.goal_tolerance_m) ||
      !std::isfinite(config_.hold_position_tolerance_m) ||
      !std::isfinite(config_.maximum_speed_mps) ||
      !std::isfinite(config_.confirmation_duration_s)) {
    throw std::invalid_argument{"invalid cooperative goal hold configuration"};
  }
}

CooperativeGoalHoldUpdate CooperativeGoalHoldConfirmation::update(
    const TimedVehicleState& state, const Point3& goal,
    const std::optional<Point3>& active_hold_position) {
  CooperativeGoalHoldUpdate result{
      .goal_distance_m = state.position_valid && finite(state.position) && finite(goal)
                             ? distance(state.position, goal)
                             : std::numeric_limits<double>::infinity(),
      .hold_position_error_m = state.position_valid && finite(state.position) &&
                                       active_hold_position &&
                                       finite(*active_hold_position)
                                   ? distance(state.position, *active_hold_position)
                                   : std::numeric_limits<double>::infinity(),
      .speed_mps = state.velocity_valid && finite(state.velocity)
                       ? speed(state)
                       : std::numeric_limits<double>::infinity(),
  };
  if (confirmed_) {
    result.confirmed = true;
    return result;
  }

  const bool stable =
      state.stamp_ns > 0 && state.armed && state.airborne &&
      active_hold_position.has_value() &&
      result.goal_distance_m <= config_.goal_tolerance_m &&
      result.hold_position_error_m <= config_.hold_position_tolerance_m &&
      result.speed_mps <= config_.maximum_speed_mps;
  if (!stable) {
    stable_since_ns_.reset();
    return result;
  }
  if (!stable_since_ns_.has_value() || state.stamp_ns < *stable_since_ns_) {
    stable_since_ns_ = state.stamp_ns;
    return result;
  }
  const double stable_duration_s =
      static_cast<double>(state.stamp_ns - *stable_since_ns_) * 1.0e-9;
  if (stable_duration_s >= config_.confirmation_duration_s) {
    confirmed_ = true;
    result.confirmed = true;
    result.newly_confirmed = true;
  }
  return result;
}

void CooperativeGoalHoldConfirmation::reset() noexcept {
  stable_since_ns_.reset();
  confirmed_ = false;
}

CooperativeSeparationMonitor::CooperativeSeparationMonitor(
    const std::size_t vehicle_count, const CooperativeSeparationConfig& config)
    : config_{config},
      previous_states_(vehicle_count),
      minimum_observed_separation_m_{std::numeric_limits<double>::infinity()} {
  if (vehicle_count < 2U || !(config_.desired_minimum_separation_m > 0.0) ||
      !(config_.release_separation_m > config_.desired_minimum_separation_m) ||
      !(config_.maximum_continuity_gap_s > 0.0) ||
      !std::isfinite(config_.desired_minimum_separation_m) ||
      !std::isfinite(config_.release_separation_m) ||
      !std::isfinite(config_.maximum_continuity_gap_s)) {
    throw std::invalid_argument{"invalid cooperative separation monitor configuration"};
  }
}

CooperativeSeparationUpdate
CooperativeSeparationMonitor::update(const std::span<const TimedVehicleState> states) {
  if (states.size() != previous_states_.size()) {
    throw std::invalid_argument{"cooperative vehicle count changed during mission"};
  }
  CooperativeSeparationUpdate result{
      .minimum_separation_m = std::numeric_limits<double>::infinity(),
      .first_minimum_index = std::nullopt,
      .second_minimum_index = std::nullopt,
      .active_desired_violation_count = 0U,
      .desired_violation_event_count = desired_violation_event_count_,
      .pairs = {},
  };
  result.pairs.reserve(states.size() * (states.size() - 1U) / 2U);
  const std::int64_t maximum_gap_ns =
      static_cast<std::int64_t>(std::llround(config_.maximum_continuity_gap_s * 1.0e9));
  for (std::size_t first = 0U; first < states.size(); ++first) {
    for (std::size_t second = first + 1U; second < states.size(); ++second) {
      const TimedVehicleState& first_state = states[first];
      const TimedVehicleState& second_state = states[second];
      if (!first_state.position_valid || !second_state.position_valid ||
          !finite(first_state.position) || !finite(second_state.position)) {
        continue;
      }
      const bool continuous =
          continuityAvailable(first_state, previous_states_[first], maximum_gap_ns) &&
          continuityAvailable(second_state, previous_states_[second], maximum_gap_ns);
      const SweptVehicleSeparation separation =
          sweptVehicleSeparation(first_state, second_state,
                                 continuous ? previous_states_[first] : std::nullopt,
                                 continuous ? previous_states_[second] : std::nullopt);
      if (separation.minimum_m < result.minimum_separation_m) {
        result.minimum_separation_m = separation.minimum_m;
        result.first_minimum_index = first;
        result.second_minimum_index = second;
      }
      minimum_observed_separation_m_ =
          std::min(minimum_observed_separation_m_, separation.minimum_m);

      const Pair pair{first, second};
      const bool was_active = active_desired_violations_.contains(pair);
      bool active = was_active;
      bool entered = false;
      bool released = false;
      if (!active && separation.minimum_m < config_.desired_minimum_separation_m) {
        active = true;
        entered = true;
        active_desired_violations_.insert(pair);
        ++desired_violation_event_count_;
      } else if (active && separation.current_m >= config_.release_separation_m &&
                 separating(first_state, second_state)) {
        active = false;
        released = true;
        active_desired_violations_.erase(pair);
      }
      result.pairs.push_back(CooperativePairSeparationUpdate{
          .first_index = first,
          .second_index = second,
          .separation = separation,
          .desired_violation_active = active,
          .newly_entered_desired_violation = entered,
          .newly_released_desired_violation = released,
      });
    }
  }
  for (std::size_t index = 0U; index < states.size(); ++index) {
    previous_states_[index] = states[index];
  }
  result.active_desired_violation_count = active_desired_violations_.size();
  result.desired_violation_event_count = desired_violation_event_count_;
  return result;
}

void CooperativeSeparationMonitor::resetTemporalContinuity() noexcept {
  std::fill(previous_states_.begin(), previous_states_.end(), std::nullopt);
}

} // namespace drone_city_nav
