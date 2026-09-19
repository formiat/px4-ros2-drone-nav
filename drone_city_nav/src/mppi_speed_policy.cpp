#include "drone_city_nav/mppi_speed_policy.hpp"

#include "drone_city_nav/mppi/mppi_clearance_cost.hpp"
#include "drone_city_nav/stopping_distance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
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

// The direction the sensor-braking contract is assessed along: the velocity
// while the vehicle moves, the route tangent where it stands, the worst
// direction when it has neither. Level flight is bounded by the horizontal
// limits alone; a climb answers to the weaker vertical deceleration.
constexpr double kMotionDirectionSpeedThresholdMps{0.25};

[[nodiscard]] Vec3 motionDirection(const MppiSpeedPolicyInput& input) noexcept {
  const Vec3 velocity{static_cast<double>(input.state.vx),
                      static_cast<double>(input.state.vy),
                      static_cast<double>(input.state.vz)};
  if (std::hypot(std::hypot(velocity.x, velocity.y), velocity.z) >=
      kMotionDirectionSpeedThresholdMps) {
    return velocity;
  }
  if (!input.route.empty()) {
    return input.route[nearestGuideIndex(input.state, input.route)].tangent;
  }
  return Vec3{};
}

// The speed a contact is left at: where the body's own clearance is nil the
// tube law admits nothing, and a vehicle resting against evidence could not
// leave it. 1 m/s is a 0.075 m tube at the measured response horizon, inside
// the 0.16 m the 0.55 m body model keeps beyond the rotor tips (0.39 m).
constexpr double kContactDepartureSpeedMps{1.0};

// The tightest speed a set of constrained samples admits: the tube law at each
// sample, and the stopping law for what the vehicle may carry on the way to
// it. The tightest answer wins.
[[nodiscard]] double
clearanceLimitedSpeed(const std::span<const ConstrainedHorizonSample3D> samples,
                      const MppiSpeedPolicyConfig& config, const double evidence_age_s,
                      const double forward_acceleration_mps2) noexcept {
  // The samples were measured on evidence this old, and the validators judge
  // on what arrived since: the free path to each sample is known only as of
  // then. The interval is latency in the stopping law, as the sensor-braking
  // contract owes its evidence age; without it the reference stayed at four
  // metres a second until a wall the scan had already seen stood two metres
  // ahead, and every horizon from there was rejected.
  StoppingCapability capability = config.stopping_capability;
  capability.reaction_latency_s +=
      config.reference_tracking_lag_s +
      (std::isfinite(evidence_age_s) ? std::max(0.0, evidence_age_s) : 0.0);
  double limit_mps = std::numeric_limits<double>::infinity();
  for (const ConstrainedHorizonSample3D& sample : samples) {
    // The progress floor stands on the margin the envelope keeps beyond the
    // body. Where the envelope already stands in evidence that margin is
    // spent, and the body's own clearance is all the tube has to fit in: in
    // the urban flight r286 the reference sat on the 3 m/s floor with the
    // body model 0.02 m from a wall the lidar had resolved beside the motion,
    // a revocation's braking overshot into it, and the rotor touched. The
    // body's clearance bounds the floor, down to the speed a contact is left
    // at.
    const double envelope_admissible_mps =
        std::max(config.clearance_minimum_progress_speed_mps,
                 static_cast<double>(mppi::tubeAdmissibleSpeedMps(
                     static_cast<float>(std::max(0.0, sample.clearance_m)),
                     static_cast<float>(config.clearance_response_time_s))));
    const double body_admissible_mps =
        std::max(kContactDepartureSpeedMps,
                 std::isfinite(sample.body_clearance_m)
                     ? static_cast<double>(mppi::tubeAdmissibleSpeedMps(
                           static_cast<float>(std::max(0.0, sample.body_clearance_m)),
                           static_cast<float>(config.clearance_response_time_s)))
                     : std::numeric_limits<double>::infinity());
    const double admissible_speed_mps =
        std::min(envelope_admissible_mps, body_admissible_mps);
    limit_mps = std::min(
        limit_mps,
        std::max(admissible_speed_mps,
                 stoppingLimitedSpeed(sample.distance_m, admissible_speed_mps,
                                      capability, config.sensor_braking_contract,
                                      forward_acceleration_mps2)));
  }
  return limit_mps;
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
      !(config.maximum_target_lookahead_m >= config.minimum_target_lookahead_m) ||
      !(config.clearance_response_time_s > 0.0) ||
      !(config.clearance_minimum_progress_speed_mps >= 0.0) ||
      !(config.reference_speed_rise_mps2 > 0.0) ||
      !(config.reference_tracking_lag_s >= 0.0)) {
    throw std::invalid_argument{"invalid MPPI speed policy configuration"};
  }
}

} // namespace

double stoppingLimitedSpeed(const double available_distance_m,
                            const double terminal_speed_mps,
                            const StoppingCapability& capability,
                            const SensorBrakingContract3D& contract,
                            const double forward_acceleration_mps2) noexcept {
  const double braking_acceleration_mps2 =
      capability.maximum_commanded_horizontal_deceleration_mps2;
  const double reaction_latency_s = capability.reaction_latency_s;
  if (!(available_distance_m > 0.0) || !(braking_acceleration_mps2 > 0.0) ||
      !(terminal_speed_mps >= 0.0) || !(reaction_latency_s >= 0.0)) {
    return 0.0;
  }
  // The law answers for the vehicle as it flies, not for one that already
  // decelerates: the jerk ramp from its forward acceleration to the full
  // deceleration is flown before the speed falls at all. A law of
  // instantaneous deceleration fell along its own curve at 4 m/s^2 once the
  // vehicle rode it, and the vehicle under it was accelerating: in the urban
  // flight r440 the clearance limit read 7.9, 7.0, 6.6, 6.1, 5.1, 4.6 and
  // 3.8 m/s over its last 1.3 s while the vehicle climbed from 3.8 to 5.6 m/s
  // at 3.7 m/s^2 beneath it, crossed it 1.05 s before the wall, and needed
  // 0.64 s and 3.5 m to turn that acceleration round. The flights r430 to
  // r452 carry fifteen to twenty-three such crossings each, the vehicle 2 to
  // 4.7 m/s above the reference, and three to twelve revocations at speed.
  // The sensor-braking contract reserves the ramp from the largest
  // acceleration, because it bounds a speed held for the whole flight; here
  // the largest acceleration would deny a vehicle creeping to its goal any
  // speed inside the last 0.6 m, so the measured one is charged.
  const JerkLimitedAxisStoppingConfig slowdown{
      .guaranteed_deceleration_mps2 = braking_acceleration_mps2,
      .maximum_acceleration_mps2 = contract.maximum_horizontal_acceleration_mps2,
      .maximum_jerk_mps3 = contract.maximum_control_jerk_mps3,
      .reaction_latency_s = reaction_latency_s,
  };
  // The instantaneous law bounds the answer from above.
  const double latency_velocity = braking_acceleration_mps2 * reaction_latency_s;
  double lower_mps = terminal_speed_mps;
  double upper_mps = std::sqrt(latency_velocity * latency_velocity +
                               terminal_speed_mps * terminal_speed_mps +
                               2.0 * braking_acceleration_mps2 * available_distance_m) -
                     latency_velocity;
  if (!(upper_mps > lower_mps)) {
    return std::max(0.0, upper_mps);
  }
  constexpr int kBisectionIterations{40};
  for (int iteration = 0; iteration < kBisectionIterations; ++iteration) {
    const double candidate_mps = std::midpoint(lower_mps, upper_mps);
    if (jerkLimitedAxisSlowdownDistanceM(candidate_mps, terminal_speed_mps,
                                         forward_acceleration_mps2,
                                         slowdown) <= available_distance_m) {
      lower_mps = candidate_mps;
    } else {
      upper_mps = candidate_mps;
    }
  }
  return lower_mps;
}

Vec3 mppiSpeedPolicyFacedDirection(const MppiSpeedPolicyInput& input) {
  return input.route.empty()
             ? motionDirection(input)
             : input.route[nearestGuideIndex(input.state, input.route)].tangent;
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
  const Vec3 braking_direction = motionDirection(input);
  // The laws that stop the vehicle short of evidence, or of the end of what
  // was certified, owe the lag of the loop behind a falling reference. The
  // goal and the turn ahead do not: arriving late at their speed meets
  // nothing, and charged to the curvature law the lag doubled the share of
  // the flight that law bound (6.8 to 15.1 percent of the ticks, r457 to
  // r459 against r461 to r463) and took 0.3 m/s off the mean speed.
  StoppingCapability law_capability = config.stopping_capability;
  law_capability.reaction_latency_s += config.reference_tracking_lag_s;
  result.sensor_braking_limit_mps = sensorBrakingMaximumSpeedMps(
      config.sensor_braking_contract, config.stopping_capability,
      config.absolute_speed_limit_mps, braking_direction);
  if (config.sensor_braking_contract.forward_horizontal_half_angle_rad <
      std::numbers::pi) {
    // A forward sensor sees the motion only while the vehicle faces it. The
    // motion the reference commands is the route's, so the route's tangent
    // names the direction where there is a route, and the velocity where
    // there is none: the velocity of a vehicle correcting its track at 1 m/s
    // swings through tens of degrees, and read off it the limit released and
    // bound again with every swing (r488: bound 56 percent of the ticks, the
    // speed cycling between 1 and 3 m/s).
    const Vec3 faced = mppiSpeedPolicyFacedDirection(input);
    // A climb inside the cone of the sensors that look up and down is theirs,
    // and its tangent's horizontal remnant points anywhere (r489: a shaft held
    // the vehicle at the unobserved speed for the whole climb). Every other
    // motion has a heading, and a wall stands across all of its elevations:
    // facing it is what lets the pair see it.
    if (sensorBrakingMotionUnfaced3D(config.sensor_braking_contract, faced,
                                     static_cast<double>(input.state.yaw))) {
      // No sensor sees a motion the vehicle does not face, so memory answers
      // for it: the contract is read with the range memory has observed along
      // it, and where it has observed nothing the vehicle waits for the gaze
      // to turn it. A fixed speed here was a blind one: r500 climbed a shaft
      // facing south-west, left it northward at the 1 m/s this rule then
      // admitted (1.5 m/s flown), and met a wall 1 m away that entered memory
      // 0.5 s before the contact, when the turning pair first saw it.
      result.unfaced_observed_range_m =
          std::min(config.sensor_braking_contract.guaranteed_detection_range_m,
                   input.unfaced_observed_range_m.value_or(0.0));
      result.sensor_braking_limit_mps =
          std::min(result.sensor_braking_limit_mps,
                   sensorBrakingMemorySpeedMps(config.sensor_braking_contract,
                                               config.stopping_capability,
                                               config.absolute_speed_limit_mps, faced,
                                               result.unfaced_observed_range_m));
    }
  }
  if (input.terminal_goal_limit_enabled) {
    const double goal_distance =
        std::max(0.0, distance3D(input.mission_goal,
                                 Point3{input.state.x, input.state.y, input.state.z}) -
                          config.goal_margin_m);
    result.goal_limit_mps = stoppingLimitedSpeed(
        goal_distance, 0.0, config.stopping_capability, config.sensor_braking_contract,
        input.forward_acceleration_mps2);
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
    result.route_endpoint_limit_mps = stoppingLimitedSpeed(
        route_endpoint_distance, 0.0, law_capability, config.sensor_braking_contract,
        input.forward_acceleration_mps2);
  }
  if (input.blocked_route_remaining_m.has_value()) {
    // The raw world blocks the route ahead and a replacement is not certified
    // yet: fly the validated prefix as if it ended a body margin before the
    // block, so the vehicle can stop clear of it whatever the replacement
    // search decides. The margin is the one the sensor-braking contract keeps
    // to evidence it stops for.
    result.blocked_route_limit_mps = stoppingLimitedSpeed(
        std::max(0.0, *input.blocked_route_remaining_m -
                          config.sensor_braking_contract.physical_margin_m),
        0.0, law_capability, config.sensor_braking_contract,
        input.forward_acceleration_mps2);
  }
  if (input.executed_horizon_clearance.has_value() &&
      input.executed_horizon_clearance->constrained()) {
    // The motion under execution comes this close to known occupied evidence,
    // this far ahead. Whatever the route promised when it was certified, the
    // tracking error the controller can accumulate within its response time
    // must fit inside the clearance the vehicle actually has there: the same
    // tube law the route certification applies, enforced on live evidence.
    // The floor keeps a tight spot leavable; the body validation stays the
    // only hard authority.
    //
    // The tube speed applies at each tight point, and the stopping law decides
    // what the vehicle may carry on the way to it; the tightest answer wins.
    // Reading the tube speed as an immediate cap is what made the reference
    // oscillate: a grazing sample far ahead dropped the reference, the horizon
    // shortened out of reach of the obstacle, the reference jumped back, and
    // the longer horizon found the obstacle again. Evidence beside the motion
    // owes no braking distance: the horizon is validated by the body against
    // the raw world and ends at rest, so "stop within the lateral clearance"
    // is not a physical requirement, and asking for it pinned every corridor
    // to the floor.
    result.clearance_limit_mps =
        std::min(result.clearance_limit_mps,
                 clearanceLimitedSpeed(
                     input.executed_horizon_clearance->constrained_samples, config,
                     input.esdf_evidence_age_s, input.forward_acceleration_mps2));
  }
  if (input.route_clearance.has_value() && input.route_clearance->constrained()) {
    // The same laws on the geometry the vehicle is committed to. The executed
    // horizon reaches only as far as the vehicle can stop, so its clearance
    // answer is a function of the speed the reference already asked for: at
    // rest it sees only the space beside the vehicle, admits cruise, and finds
    // the tight spot ahead only once the vehicle is fast enough to reach into
    // it. That loop is what let the reference climb out of every stop until
    // the raw validator rejected the horizon and stopped the vehicle again.
    // The route's clearance profile does not move with the speed, so it bounds
    // the reference before the horizon grows into the constraint.
    result.route_clearance_limit_mps =
        std::min(result.route_clearance_limit_mps,
                 clearanceLimitedSpeed(input.route_clearance->constrained_samples,
                                       config, input.esdf_evidence_age_s,
                                       input.forward_acceleration_mps2));
  }
  std::optional<double> observed_range_m;
  if (input.executed_horizon_clearance.has_value() &&
      input.executed_horizon_clearance->unobserved()) {
    observed_range_m = input.executed_horizon_clearance->distanceToUnobservedM();
  }
  if (input.route_observed_range_m.has_value() &&
      std::isfinite(*input.route_observed_range_m) &&
      *input.route_observed_range_m >= 0.0) {
    observed_range_m =
        std::min(observed_range_m.value_or(std::numeric_limits<double>::infinity()),
                 *input.route_observed_range_m);
  }
  if (observed_range_m.has_value()) {
    // The motion ahead enters space the evidence has not observed. The
    // sensor-braking contract bounds the speed by the range the sensor is
    // guaranteed to have seen; along this motion the evidence reaches only as
    // far as the first unobserved sample, so the contract is read with that
    // range in place of the guaranteed one. An opening whose inside the lidar
    // has not looked into yet is approached at the speed the vehicle can stop
    // from before it, and the range grows back as the view opens. The floor
    // keeps unobserved space enterable — it is traversable and carries no
    // penalty — at the speed the raw validators certify every horizon for.
    // The executed horizon ends where the vehicle can rest, so it reports a
    // frontier only once the vehicle can no longer stop before it; the route
    // ahead reports the frontier while there is still room to slow for it.
    double frontier_limit_mps = config.clearance_minimum_progress_speed_mps;
    if (*observed_range_m > config.sensor_braking_contract.physical_margin_m) {
      SensorBrakingContract3D observed_contract = config.sensor_braking_contract;
      observed_contract.guaranteed_detection_range_m = *observed_range_m;
      frontier_limit_mps = std::max(
          frontier_limit_mps, sensorBrakingMaximumSpeedMps(
                                  observed_contract, config.stopping_capability,
                                  config.absolute_speed_limit_mps, braking_direction));
    }
    result.unobserved_frontier_limit_mps = frontier_limit_mps;
    result.unobserved_frontier_range_m = *observed_range_m;
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
      const double approach_limit = stoppingLimitedSpeed(
          distance_to_turn_m, turn_speed, config.stopping_capability,
          config.sensor_braking_contract, input.forward_acceleration_mps2);
      result.curvature_limit_mps = std::min(result.curvature_limit_mps, approach_limit);
    }
  }

  result.reference_speed_mps = std::min(
      {result.cruise_limit_mps, result.absolute_limit_mps, result.curvature_limit_mps,
       result.sensor_braking_limit_mps, result.goal_limit_mps,
       result.route_endpoint_limit_mps, result.route_constraint_limit_mps,
       result.blocked_route_limit_mps, result.clearance_limit_mps,
       result.route_clearance_limit_mps, result.unobserved_frontier_limit_mps});
  const std::array limits{
      std::pair{result.cruise_limit_mps, MppiSpeedLimiter::kCruise},
      std::pair{result.absolute_limit_mps, MppiSpeedLimiter::kAbsolute},
      std::pair{result.curvature_limit_mps, MppiSpeedLimiter::kCurvature},
      std::pair{result.sensor_braking_limit_mps, MppiSpeedLimiter::kSensorBraking},
      std::pair{result.goal_limit_mps, MppiSpeedLimiter::kGoal},
      std::pair{result.route_endpoint_limit_mps, MppiSpeedLimiter::kRouteEndpoint},
      std::pair{result.route_constraint_limit_mps, MppiSpeedLimiter::kRouteConstraint},
      std::pair{result.blocked_route_limit_mps, MppiSpeedLimiter::kBlockedRoute},
      std::pair{result.clearance_limit_mps, MppiSpeedLimiter::kClearance},
      std::pair{result.route_clearance_limit_mps, MppiSpeedLimiter::kRouteClearance},
      std::pair{result.unobserved_frontier_limit_mps,
                MppiSpeedLimiter::kUnobservedFrontier},
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
      std::max(result.reference_speed_mps, measured_speed_mps), braking_direction);
  if (!result.sensor_braking_assessment.accepted()) {
    // The measured speed exceeds what the sensor range can stop within. The
    // reference already sits at or below the sensor-braking limit and the
    // excess above the reference is priced as overspeed, so the vehicle sheds
    // the excess instead of being asked to stop outright and then released
    // again on the next tick.
    result.active_limiter = MppiSpeedLimiter::kSensorBraking;
  }
  // A cap is always allowed to bite at once, so the reference may fall as far
  // as any limiter asks. It may only climb at the rate the airframe can
  // follow: a limit that lifts as the horizon shifts must not snap the
  // reference back up, because the controller answers each step with a fresh
  // burst of acceleration.
  result.unslewed_reference_speed_mps = result.reference_speed_mps;
  if (input.previous_reference_speed_mps.has_value() &&
      input.elapsed_since_previous_reference_s > 0.0 &&
      config.reference_speed_rise_mps2 > 0.0) {
    const double rise_ceiling_mps =
        std::max(0.0, *input.previous_reference_speed_mps) +
        config.reference_speed_rise_mps2 * input.elapsed_since_previous_reference_s;
    if (result.reference_speed_mps > rise_ceiling_mps) {
      result.reference_speed_mps = rise_ceiling_mps;
      result.reference_speed_rise_limited = true;
    }
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
    case MppiSpeedLimiter::kClearance:
      return "clearance";
    case MppiSpeedLimiter::kRouteClearance:
      return "route_clearance";
    case MppiSpeedLimiter::kUnobservedFrontier:
      return "unobserved_frontier";
  }
  return "unknown";
}

} // namespace drone_city_nav
