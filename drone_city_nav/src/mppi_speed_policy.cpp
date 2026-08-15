#include "drone_city_nav/mppi_speed_policy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::size_t nearestGuideIndex(const mppi::State& state,
                                            const std::span<const Point2> guide) {
  std::size_t nearest_index = 0U;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < guide.size(); ++index) {
    const double candidate =
        std::hypot(guide[index].x - state.x, guide[index].y - state.y);
    if (candidate < nearest_distance) {
      nearest_distance = candidate;
      nearest_index = index;
    }
  }
  return nearest_index;
}

[[nodiscard]] double turnCurvature(const Point2 first, const Point2 center,
                                   const Point2 last) noexcept {
  const double first_length = distance(first, center);
  const double second_length = distance(center, last);
  if (first_length <= 1.0e-6 || second_length <= 1.0e-6) {
    return 0.0;
  }
  const double first_x = (center.x - first.x) / first_length;
  const double first_y = (center.y - first.y) / first_length;
  const double second_x = (last.x - center.x) / second_length;
  const double second_y = (last.y - center.y) / second_length;
  const double dot = std::clamp(first_x * second_x + first_y * second_y, -1.0, 1.0);
  return std::acos(dot) / (0.5 * (first_length + second_length));
}

[[nodiscard]] double windowedTurnCurvature(const std::span<const Point2> guide,
                                           const std::size_t center_index,
                                           const double measurement_window_m) noexcept {
  if (center_index == 0U || center_index + 1U >= guide.size()) {
    return 0.0;
  }
  const double half_window_m = 0.5 * measurement_window_m;
  std::size_t first_index = center_index;
  double first_distance_m = 0.0;
  while (first_index > 0U && first_distance_m < half_window_m) {
    first_distance_m += distance(guide[first_index], guide[first_index - 1U]);
    --first_index;
  }
  std::size_t last_index = center_index;
  double last_distance_m = 0.0;
  while (last_index + 1U < guide.size() && last_distance_m < half_window_m) {
    last_distance_m += distance(guide[last_index], guide[last_index + 1U]);
    ++last_index;
  }
  if (first_index == center_index || last_index == center_index) {
    return 0.0;
  }
  return turnCurvature(guide[first_index], guide[center_index], guide[last_index]);
}

void validateConfig(const MppiSpeedPolicyConfig& config) {
  if (!(config.cruise_speed_mps > 0.0) || !(config.absolute_speed_limit_mps > 0.0) ||
      !(config.maximum_lateral_acceleration_mps2 > 0.0) ||
      !(config.maximum_braking_acceleration_mps2 > 0.0) ||
      !(config.reaction_latency_s >= 0.0) ||
      !(config.observation_distance_m > config.observation_margin_m) ||
      !(config.observation_margin_m >= 0.0) || !(config.goal_margin_m >= 0.0) ||
      !(config.curvature_preview_distance_m > 0.0) ||
      !(config.curvature_measurement_window_m > 0.0) ||
      !(config.horizon_duration_s > 0.0) ||
      !(config.minimum_target_lookahead_m > 0.0) ||
      !(config.maximum_target_lookahead_m >= config.minimum_target_lookahead_m)) {
    throw std::invalid_argument{"invalid MPPI speed policy configuration"};
  }
}

} // namespace

double stoppingLimitedSpeed(const double available_distance_m,
                            const double terminal_speed_mps,
                            const double reaction_latency_s,
                            const double braking_acceleration_mps2) noexcept {
  if (!(available_distance_m > 0.0) || !(braking_acceleration_mps2 > 0.0) ||
      !(terminal_speed_mps >= 0.0) || !(reaction_latency_s >= 0.0)) {
    return 0.0;
  }
  const double latency_velocity = braking_acceleration_mps2 * reaction_latency_s;
  const double discriminant = latency_velocity * latency_velocity +
                              terminal_speed_mps * terminal_speed_mps +
                              2.0 * braking_acceleration_mps2 * available_distance_m;
  return std::max(0.0, std::sqrt(discriminant) - latency_velocity);
}

MppiSpeedPolicyResult evaluateMppiSpeedPolicy(const MppiSpeedPolicyConfig& config,
                                              const MppiSpeedPolicyInput& input) {
  validateConfig(config);
  MppiSpeedPolicyResult result;
  result.enabled = true;
  result.cruise_limit_mps = config.cruise_speed_mps;
  result.absolute_limit_mps = config.absolute_speed_limit_mps;
  result.terminal_goal_limit_enabled = input.terminal_goal_limit_enabled;
  result.observation_limit_mps = stoppingLimitedSpeed(
      config.observation_distance_m - config.observation_margin_m, 0.0,
      config.reaction_latency_s, config.maximum_braking_acceleration_mps2);
  if (input.terminal_goal_limit_enabled) {
    const double goal_distance =
        std::max(0.0, std::hypot(input.mission_goal.x - input.state.x,
                                 input.mission_goal.y - input.state.y) -
                          config.goal_margin_m);
    result.goal_limit_mps =
        stoppingLimitedSpeed(goal_distance, 0.0, config.reaction_latency_s,
                             config.maximum_braking_acceleration_mps2);
  }
  if (input.route_endpoint_remaining_m.has_value()) {
    const double route_endpoint_distance =
        std::max(0.0, *input.route_endpoint_remaining_m - config.goal_margin_m);
    result.route_endpoint_limit_mps =
        stoppingLimitedSpeed(route_endpoint_distance, 0.0, config.reaction_latency_s,
                             config.maximum_braking_acceleration_mps2);
  }
  if (input.route_constraint_speed_limit_mps.has_value()) {
    result.route_constraint_limit_mps =
        std::max(0.0, *input.route_constraint_speed_limit_mps);
  }

  if (input.guide.size() >= 3U) {
    const std::size_t nearest_index = nearestGuideIndex(input.state, input.guide);
    double distance_to_turn_m = 0.0;
    for (std::size_t index = nearest_index + 1U;
         index + 1U < input.guide.size() &&
         distance_to_turn_m <= config.curvature_preview_distance_m;
         ++index) {
      distance_to_turn_m += distance(input.guide[index - 1U], input.guide[index]);
      const double curvature = windowedTurnCurvature(
          input.guide, index, config.curvature_measurement_window_m);
      result.maximum_preview_curvature_1pm =
          std::max(result.maximum_preview_curvature_1pm, curvature);
      if (curvature <= 1.0e-6) {
        continue;
      }
      const double turn_speed =
          std::sqrt(config.maximum_lateral_acceleration_mps2 / curvature);
      const double approach_limit = stoppingLimitedSpeed(
          distance_to_turn_m, turn_speed, config.reaction_latency_s,
          config.maximum_braking_acceleration_mps2);
      result.curvature_limit_mps = std::min(result.curvature_limit_mps, approach_limit);
    }
  }

  result.reference_speed_mps = std::min(
      {result.cruise_limit_mps, result.absolute_limit_mps, result.curvature_limit_mps,
       result.observation_limit_mps, result.goal_limit_mps,
       result.route_endpoint_limit_mps, result.route_constraint_limit_mps});
  const std::array limits{
      std::pair{result.cruise_limit_mps, MppiSpeedLimiter::kCruise},
      std::pair{result.absolute_limit_mps, MppiSpeedLimiter::kAbsolute},
      std::pair{result.curvature_limit_mps, MppiSpeedLimiter::kCurvature},
      std::pair{result.observation_limit_mps, MppiSpeedLimiter::kObservation},
      std::pair{result.goal_limit_mps, MppiSpeedLimiter::kGoal},
      std::pair{result.route_endpoint_limit_mps, MppiSpeedLimiter::kRouteEndpoint},
      std::pair{result.route_constraint_limit_mps, MppiSpeedLimiter::kRouteConstraint},
  };
  result.active_limiter = std::min_element(limits.begin(), limits.end(),
                                           [](const auto& first, const auto& second) {
                                             return first.first < second.first;
                                           })
                              ->second;
  result.target_lookahead_m =
      std::clamp(result.reference_speed_mps * config.horizon_duration_s,
                 config.minimum_target_lookahead_m, config.maximum_target_lookahead_m);
  return result;
}

const char* mppiSpeedLimiterName(const MppiSpeedLimiter limiter) noexcept {
  switch (limiter) {
    case MppiSpeedLimiter::kCruise:
      return "cruise";
    case MppiSpeedLimiter::kAbsolute:
      return "absolute";
    case MppiSpeedLimiter::kCurvature:
      return "curvature";
    case MppiSpeedLimiter::kObservation:
      return "observation";
    case MppiSpeedLimiter::kGoal:
      return "goal";
    case MppiSpeedLimiter::kRouteEndpoint:
      return "route_endpoint";
    case MppiSpeedLimiter::kRouteConstraint:
      return "route_constraint";
  }
  return "unknown";
}

} // namespace drone_city_nav
