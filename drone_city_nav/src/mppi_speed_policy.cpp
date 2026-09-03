#include "drone_city_nav/mppi_speed_policy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::size_t
nearestGuideIndex(const mppi::State& state,
                  const std::span<const RouteSample3D> route) {
  std::size_t nearest_index = 0U;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < route.size(); ++index) {
    const double candidate =
        distance3D(route[index].position, Point3{state.x, state.y, state.z});
    if (candidate < nearest_distance) {
      nearest_distance = candidate;
      nearest_index = index;
    }
  }
  return nearest_index;
}

[[nodiscard]] double turnCurvature(const Point3& first, const Point3& center,
                                   const Point3& last) noexcept {
  const double first_length = distance3D(first, center);
  const double second_length = distance3D(center, last);
  if (first_length <= 1.0e-6 || second_length <= 1.0e-6) {
    return 0.0;
  }
  const Vec3 first_direction{(center.x - first.x) / first_length,
                             (center.y - first.y) / first_length,
                             (center.z - first.z) / first_length};
  const Vec3 second_direction{(last.x - center.x) / second_length,
                              (last.y - center.y) / second_length,
                              (last.z - center.z) / second_length};
  const double dot = std::clamp(first_direction.x * second_direction.x +
                                    first_direction.y * second_direction.y +
                                    first_direction.z * second_direction.z,
                                -1.0, 1.0);
  return std::acos(dot) / (0.5 * (first_length + second_length));
}

[[nodiscard]] double windowedTurnCurvature(const std::span<const RouteSample3D> route,
                                           const std::size_t center_index,
                                           const double measurement_window_m) noexcept {
  if (center_index == 0U || center_index + 1U >= route.size()) {
    return 0.0;
  }
  const double half_window_m = 0.5 * measurement_window_m;
  std::size_t first_index = center_index;
  double first_distance_m = 0.0;
  while (first_index > 0U && first_distance_m < half_window_m) {
    first_distance_m +=
        distance3D(route[first_index].position, route[first_index - 1U].position);
    --first_index;
  }
  std::size_t last_index = center_index;
  double last_distance_m = 0.0;
  while (last_index + 1U < route.size() && last_distance_m < half_window_m) {
    last_distance_m +=
        distance3D(route[last_index].position, route[last_index + 1U].position);
    ++last_index;
  }
  if (first_index == center_index || last_index == center_index) {
    return 0.0;
  }
  return turnCurvature(route[first_index].position, route[center_index].position,
                       route[last_index].position);
}

void validateConfig(const MppiSpeedPolicyConfig& config) {
  if (!(config.cruise_speed_mps > 0.0) || !(config.absolute_speed_limit_mps > 0.0) ||
      !(config.maximum_lateral_acceleration_mps2 > 0.0) ||
      !stoppingCapabilityIsValid(config.stopping_capability) ||
      !sensorBrakingContract3DIsValid(config.sensor_braking_contract,
                                      config.stopping_capability) ||
      !(config.goal_margin_m >= 0.0) || !(config.curvature_preview_distance_m > 0.0) ||
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
                            const StoppingCapability& capability) noexcept {
  const double braking_acceleration_mps2 =
      capability.maximum_commanded_horizontal_deceleration_mps2;
  const double reaction_latency_s = capability.reaction_latency_s;
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
  result.route_endpoint_semantics = input.route_endpoint_semantics;
  result.route_endpoint_stop_required =
      routeEndpointHasTerminalStop3D(input.route_endpoint_semantics);
  result.terminal_goal_limit_enabled = input.terminal_goal_limit_enabled;
  result.sensor_braking_limit_mps = sensorBrakingMaximumSpeedMps(
      config.sensor_braking_contract, config.stopping_capability,
      config.absolute_speed_limit_mps);
  if (input.terminal_goal_limit_enabled) {
    const double goal_distance =
        std::max(0.0, distance3D(input.mission_goal,
                                 Point3{input.state.x, input.state.y, input.state.z}) -
                          config.goal_margin_m);
    result.goal_limit_mps =
        stoppingLimitedSpeed(goal_distance, 0.0, config.stopping_capability);
  }
  if (result.route_endpoint_stop_required &&
      input.route_endpoint_remaining_m.has_value()) {
    // The stop point is the route's last sample. Its station distance is the
    // path still to fly along the route; beside the endpoint that distance
    // reads zero while the endpoint itself is still half a metre away, and a
    // zero allowance would leave the vehicle creeping toward a stop it can
    // reach at speed. The straight distance to the endpoint is the least the
    // vehicle must travel, so the allowance covers whichever is longer.
    double route_endpoint_distance = std::max(0.0, *input.route_endpoint_remaining_m);
    if (!input.route.empty()) {
      route_endpoint_distance =
          std::max(route_endpoint_distance,
                   distance3D(input.route.back().position,
                              Point3{input.state.x, input.state.y, input.state.z}));
    }
    result.route_endpoint_limit_mps =
        stoppingLimitedSpeed(route_endpoint_distance, 0.0, config.stopping_capability);
  }
  if (input.blocked_route_remaining_m.has_value()) {
    // The raw world blocks the route ahead and a replacement is not certified
    // yet: fly the validated prefix as if it ended at the block, so the
    // vehicle can stop before it whatever the replacement search decides.
    result.blocked_route_limit_mps =
        stoppingLimitedSpeed(std::max(0.0, *input.blocked_route_remaining_m), 0.0,
                             config.stopping_capability);
  }
  if (input.route_constraint_speed_limit_mps.has_value()) {
    result.route_constraint_limit_mps =
        std::max(0.0, *input.route_constraint_speed_limit_mps);
  }

  if (input.route.size() >= 3U) {
    const std::size_t nearest_index = nearestGuideIndex(input.state, input.route);
    double distance_to_turn_m = 0.0;
    for (std::size_t index = nearest_index + 1U;
         index + 1U < input.route.size() &&
         distance_to_turn_m <= config.curvature_preview_distance_m;
         ++index) {
      distance_to_turn_m +=
          distance3D(input.route[index - 1U].position, input.route[index].position);
      const double curvature = windowedTurnCurvature(
          input.route, index, config.curvature_measurement_window_m);
      result.maximum_preview_curvature_1pm =
          std::max(result.maximum_preview_curvature_1pm, curvature);
      if (curvature <= 1.0e-6) {
        continue;
      }
      const double turn_speed =
          std::sqrt(config.maximum_lateral_acceleration_mps2 / curvature);
      const double approach_limit = stoppingLimitedSpeed(distance_to_turn_m, turn_speed,
                                                         config.stopping_capability);
      result.curvature_limit_mps = std::min(result.curvature_limit_mps, approach_limit);
    }
  }

  result.reference_speed_mps =
      std::min({result.cruise_limit_mps, result.absolute_limit_mps,
                result.curvature_limit_mps, result.sensor_braking_limit_mps,
                result.goal_limit_mps, result.route_endpoint_limit_mps,
                result.route_constraint_limit_mps, result.blocked_route_limit_mps});
  const std::array limits{
      std::pair{result.cruise_limit_mps, MppiSpeedLimiter::kCruise},
      std::pair{result.absolute_limit_mps, MppiSpeedLimiter::kAbsolute},
      std::pair{result.curvature_limit_mps, MppiSpeedLimiter::kCurvature},
      std::pair{result.sensor_braking_limit_mps, MppiSpeedLimiter::kSensorBraking},
      std::pair{result.goal_limit_mps, MppiSpeedLimiter::kGoal},
      std::pair{result.route_endpoint_limit_mps, MppiSpeedLimiter::kRouteEndpoint},
      std::pair{result.route_constraint_limit_mps, MppiSpeedLimiter::kRouteConstraint},
      std::pair{result.blocked_route_limit_mps, MppiSpeedLimiter::kBlockedRoute},
  };
  result.active_limiter = std::min_element(limits.begin(), limits.end(),
                                           [](const auto& first, const auto& second) {
                                             return first.first < second.first;
                                           })
                              ->second;
  const double measured_speed_mps =
      std::hypot(std::hypot(static_cast<double>(input.state.vx),
                            static_cast<double>(input.state.vy)),
                 static_cast<double>(input.state.vz));
  result.sensor_braking_assessment = assessSensorBrakingContract3D(
      config.sensor_braking_contract, config.stopping_capability,
      std::max(result.reference_speed_mps, measured_speed_mps));
  if (!result.sensor_braking_assessment.accepted()) {
    result.reference_speed_mps = 0.0;
    result.active_limiter = MppiSpeedLimiter::kSensorBraking;
  }
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
    case MppiSpeedLimiter::kSensorBraking:
      return "sensor_braking";
    case MppiSpeedLimiter::kGoal:
      return "goal";
    case MppiSpeedLimiter::kRouteEndpoint:
      return "route_endpoint";
    case MppiSpeedLimiter::kRouteConstraint:
      return "route_constraint";
    case MppiSpeedLimiter::kBlockedRoute:
      return "blocked_route";
  }
  return "unknown";
}

} // namespace drone_city_nav
