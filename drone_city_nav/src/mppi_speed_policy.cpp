#include "drone_city_nav/mppi_speed_policy.hpp"

#include "drone_city_nav/mppi/mppi_clearance_cost.hpp"
#include "drone_city_nav/stopping_distance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
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

// The acceleration the airframe can apply along a direction when each axis
// has its own bound: the horizontal figure for level flight, the vertical one
// for a climb or a descent, between them the axis that saturates first; with
// no direction at all (a vehicle at rest without a route) the horizontal one,
// as the laws read before.
[[nodiscard]] double alongDirection(const double horizontal_mps2,
                                    const double vertical_mps2,
                                    const Vec3& direction) noexcept {
  const double horizontal = std::hypot(direction.x, direction.y);
  const double length = std::hypot(horizontal, direction.z);
  if (!(length > 1.0e-9)) {
    return horizontal_mps2;
  }
  double limit = std::numeric_limits<double>::infinity();
  if (horizontal / length > 1.0e-9) {
    limit = std::min(limit, horizontal_mps2 / (horizontal / length));
  }
  if (std::abs(direction.z) / length > 1.0e-9) {
    limit = std::min(limit, vertical_mps2 / (std::abs(direction.z) / length));
  }
  return limit;
}

// The stopping laws that stop short of evidence, of the goal, of the route's
// end or of a turn brake along the motion: the deceleration and the
// acceleration they charge are the airframe's along that direction. They
// took the horizontal figures for every motion: a descent at 2.4 m/s toward
// the survey's vantage 1.9 m over a floor was braked by 4 m/s^2 on paper and
// by the airframe's 1.4 m/s^2 in the air, the clearance limit fell from 1.4 to
// 0 m/s over 0.5 s while the vehicle lost 0.4 m/s^2, and the floor was touched
// (r3164). The sensor-braking contract reads the direction itself.
[[nodiscard]] MppiSpeedPolicyConfig alongMotion(const MppiSpeedPolicyConfig& config,
                                                const Vec3& direction) {
  MppiSpeedPolicyConfig along = config;
  StoppingCapability& capability = along.stopping_capability;
  capability.maximum_commanded_horizontal_deceleration_mps2 =
      alongDirection(capability.maximum_commanded_horizontal_deceleration_mps2,
                     capability.guaranteed_vertical_deceleration_mps2, direction);
  capability.guaranteed_horizontal_deceleration_mps2 =
      std::min(capability.guaranteed_horizontal_deceleration_mps2,
               capability.maximum_commanded_horizontal_deceleration_mps2);
  along.sensor_braking_contract.maximum_horizontal_acceleration_mps2 = alongDirection(
      config.sensor_braking_contract.maximum_horizontal_acceleration_mps2,
      config.sensor_braking_contract.maximum_vertical_acceleration_mps2, direction);
  return along;
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
  // The contact floor is the speed a contact is left at, not the one it is
  // approached at: where the body will stand closer to evidence than it stands
  // now, the tube of its own clearance is all it may carry there. Approached at
  // the floor, a slab over a doorway took the body model's last centimetres
  // with a tenth of a metre of tracking error, the stop that followed rolled
  // the vehicle 17 degrees and a rotor touched the slab (r837).
  double current_body_clearance_m = std::numeric_limits<double>::infinity();
  for (const ConstrainedHorizonSample3D& sample : samples) {
    if (!(sample.distance_m > 0.0)) {
      current_body_clearance_m =
          std::min(current_body_clearance_m, sample.body_clearance_m);
    }
  }
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
    const double body_admissible_mps = std::max(
        sample.body_clearance_m >= current_body_clearance_m ? kContactDepartureSpeedMps
                                                            : 0.0,
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

// Roadmap item 21, specification K66: a body the memory follows moving owes
// the stopping law in every direction, unlike a wall beside the motion (which
// owes none: a wall does not step into the path). The speed the vehicle may
// carry toward a body is the one it can stop from before the body's envelope
// and the clearance, less what the body itself closes while the vehicle
// stops; the reference is bounded by that speed over the share of the
// commanded motion that closes on the body. A motion that does not close on
// the body owes it nothing, so the vehicle can always leave one that walks
// up to it; inside the envelope the body validation is the authority.
[[nodiscard]] double movingBodyLimitedSpeed(const MppiSpeedPolicyInput& input,
                                            const MppiSpeedPolicyConfig& config,
                                            double& nearest_m) noexcept {
  StoppingCapability capability = config.stopping_capability;
  capability.reaction_latency_s +=
      config.reference_tracking_lag_s + (std::isfinite(input.moving_body_age_s)
                                             ? std::max(0.0, input.moving_body_age_s)
                                             : 0.0);
  const Vec3 faced = mppiSpeedPolicyFacedDirection(input);
  const double faced_norm = std::hypot(std::hypot(faced.x, faced.y), faced.z);
  const Point3 at{input.state.x, input.state.y, input.state.z};
  const Vec3 velocity{static_cast<double>(input.state.vx),
                      static_cast<double>(input.state.vy),
                      static_cast<double>(input.state.vz)};
  constexpr double kLeastClosingShare{0.1};
  double limit_mps = std::numeric_limits<double>::infinity();
  for (const MovingBody3D& body : input.followed_bodies) {
    const double dx = at.x - body.centre_map.x;
    const double dy = at.y - body.centre_map.y;
    const double horizontal = std::hypot(dx, dy);
    const double gap_h = std::max(0.0, horizontal - body.radius_m);
    const double over = at.z - body.top_m;
    const double under = body.bottom_m - at.z;
    const double gap_v = std::max({0.0, over, under});
    const double gap = std::hypot(gap_h, gap_v);
    nearest_m = std::min(nearest_m, gap);
    if (!(gap > 1.0e-6) || !(horizontal > 1.0e-9)) {
      continue;
    }
    // The direction from the vehicle to the nearest point of the envelope.
    double vertical = 0.0;
    if (over > 0.0) {
      vertical = -gap_v;
    } else if (under > 0.0) {
      vertical = gap_v;
    }
    const Vec3 toward{-dx / horizontal * gap_h / gap, -dy / horizontal * gap_h / gap,
                      vertical / gap};
    const double share =
        faced_norm > 1.0e-9
            ? (faced.x * toward.x + faced.y * toward.y + faced.z * toward.z) /
                  faced_norm
            : 1.0;
    const double flown_closing =
        velocity.x * toward.x + velocity.y * toward.y + velocity.z * toward.z;
    if (share <= 0.0 && flown_closing <= 0.0) {
      continue;
    }
    // The body closes at its speed whichever way it walks: it could turn at
    // once (roadmap item 21 stage 2; the sweep is narrowed along the velocity,
    // the law is not).
    const double body_closing =
        std::hypot(body.velocity_map_mps.x, body.velocity_map_mps.y);
    double available_m = gap - config.moving_body_clearance_m;
    double allowed_mps = 0.0;
    if (available_m > 0.0) {
      allowed_mps = stoppingLimitedSpeed(available_m, 0.0, capability,
                                         config.sensor_braking_contract,
                                         input.forward_acceleration_mps2);
      if (body_closing > 0.0) {
        // The body keeps coming while the vehicle reacts and brakes.
        const double stop_s =
            capability.reaction_latency_s +
            allowed_mps / capability.guaranteed_horizontal_deceleration_mps2;
        available_m -= body_closing * stop_s;
        allowed_mps = available_m > 0.0
                          ? stoppingLimitedSpeed(available_m, 0.0, capability,
                                                 config.sensor_braking_contract,
                                                 input.forward_acceleration_mps2)
                          : 0.0;
      }
    }
    limit_mps = std::min(limit_mps, allowed_mps / std::max(share, kLeastClosingShare));
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
      !(config.reference_tracking_lag_s >= 0.0) ||
      !(config.moving_body_clearance_m >= 0.0)) {
    throw std::invalid_argument{"invalid MPPI speed policy configuration"};
  }
}

} // namespace

Vec3 mppiSpeedPolicyFacedDirection(const MppiSpeedPolicyInput& input) {
  return input.route.empty()
             ? motionDirection(input)
             : input.route[nearestGuideIndex(input.state, input.route)].tangent;
}

namespace {

// Roadmap item 17 stage 0: the contract's two central inputs are read off the
// sensor's latest frame instead of charged as constants. A pair that went
// blind was flown at the speed 6.4 m admits, and the 600 ms of evidence age
// were charged whatever the age was. A frame whose beams observe as much as a
// lit, textured scene gives stands behind the configured range; one whose
// beams observe nothing is blind; the range falls linearly between. Judged
// for the frame as a whole and not per ray: a bare room returns few surfaces
// and every ray a free one. Measured on 2026-09-27, once a match within the
// image's noise stopped counting as one: the stereo pair observes 0.30 to
// 0.48 of the pixels it samples in the lit location (p01 to p50, r767) and
// 0.11 to 0.21 in the dark one by its carried light (p10 to max, r768),
// which this reads as about 3.6 m of range, what a carried light was
// expected to give; the lidar observes 0.59 to 0.98 of its beams (r737 to
// r741).
// A blind forward sensor still leaves the sensors that look up and down: the
// range stops a millimetre above the margin, where the contract admits almost
// nothing forward and stays valid for the vertical cone.
constexpr double kBlindRangeAboveMarginM{1.0e-3};

MppiSpeedPolicyConfig measuredSensorContract(const MppiSpeedPolicyConfig& configured,
                                             const MppiSpeedPolicyInput& input,
                                             double& frame_range_m,
                                             double& frame_share) {
  MppiSpeedPolicyConfig config = configured;
  SensorBrakingContract3D& contract = config.sensor_braking_contract;
  frame_range_m = contract.guaranteed_detection_range_m;
  frame_share = 1.0;
  if (input.sensor_observed_fraction.has_value() &&
      std::isfinite(*input.sensor_observed_fraction)) {
    const double share =
        sensorFrameShare(*input.sensor_observed_fraction, input.sensor_light_headroom);
    frame_share = share;
    // The way the vehicle came is never closed (I3): back along it the
    // memory's observed range answers for a frame whose light has faded,
    // which leaves the vehicle at the edge of the dark blind in every
    // direction (r803 stood 90 s). Anywhere else the frame limits while the
    // light is not read sound: read for every motion, the memory flew r808 at
    // 2 m/s into the dark over B, which it had seen free while the light
    // still reached it. Under a light its driver reads sound the dark is the
    // place's (roadmap item 29): the memory answers for every motion, as far
    // as the pair and the time-of-flight ring have observed, and a large dark
    // space is crossed at the speed that range admits.
    frame_range_m = std::max(contract.physical_margin_m + kBlindRangeAboveMarginM,
                             share * contract.guaranteed_detection_range_m);
    // A void frame (K46): empty to the light's reach under a sound light, the
    // frame alone stands behind that depth, never past the configured range.
    if (input.sensor_void_range_m.has_value() &&
        std::isfinite(*input.sensor_void_range_m) && *input.sensor_void_range_m > 0.0) {
      frame_range_m =
          std::max(frame_range_m, std::min(*input.sensor_void_range_m,
                                           contract.guaranteed_detection_range_m));
    }
    // A patch the pair is silent in on the heading, with a matched rim
    // within reach, bounds the motion into it by the rim's depth, as a
    // surface seen there would (specification F13); a smoke face's bound
    // fallen to nought is the face at hand, a stop, not an absent bound
    // (K44: r2047 read the nought as none and flew on into the box).
    if (input.sensor_silent_range_m.has_value() &&
        std::isfinite(*input.sensor_silent_range_m) &&
        *input.sensor_silent_range_m >= 0.0) {
      frame_range_m = std::max(contract.physical_margin_m + kBlindRangeAboveMarginM,
                               std::min(frame_range_m, *input.sensor_silent_range_m));
    }
    contract.guaranteed_detection_range_m = std::max(
        {frame_range_m, input.motion_along_flown_path || input.light_reads_sound
                            ? std::min(contract.guaranteed_detection_range_m,
                                       input.unfaced_observed_range_m.value_or(0.0))
                            : 0.0});
  }
  // The sensor's own bound on the heading (a smoke face, a heat source's
  // standoff, K44, K49) caps what the memory answers for as well: the memory
  // saw that space free before the mark was laid (r2241: a heat source at
  // 1.2 m by the bound, 5.2 m by the memory under a sound light, flown into
  // to 2.6 m), and read whatever the frame observed, so that a frame emptied
  // by the mark (its returns through the ball dropped) still bounds.
  if (input.sensor_silent_range_m.has_value() &&
      std::isfinite(*input.sensor_silent_range_m) &&
      *input.sensor_silent_range_m >= 0.0) {
    const double floor_m = contract.physical_margin_m + kBlindRangeAboveMarginM;
    frame_range_m =
        std::max(floor_m, std::min(frame_range_m, *input.sensor_silent_range_m));
    contract.guaranteed_detection_range_m =
        std::max(floor_m, std::min(contract.guaranteed_detection_range_m,
                                   *input.sensor_silent_range_m));
  }
  if (input.sensor_evidence_age_s.has_value() &&
      std::isfinite(*input.sensor_evidence_age_s) &&
      *input.sensor_evidence_age_s >= 0.0) {
    contract.maximum_evidence_age_s = *input.sensor_evidence_age_s;
  }
  return config;
}

} // namespace

double sensorFrameShare(const double observed_fraction,
                        const std::optional<double> light_headroom) noexcept {
  return std::min(
      std::clamp((observed_fraction - kSensorBlindObservedFraction) /
                     (kSensorHealthyObservedFraction - kSensorBlindObservedFraction),
                 0.0, 1.0),
      light_headroom.has_value() && std::isfinite(*light_headroom)
          ? std::clamp(*light_headroom, 0.0, 1.0)
          : 1.0);
}

MppiSpeedPolicyResult evaluateMppiSpeedPolicy(const MppiSpeedPolicyConfig& configured,
                                              const MppiSpeedPolicyInput& input) {
  validateConfig(configured);
  MppiSpeedPolicyResult result;
  const MppiSpeedPolicyConfig config = measuredSensorContract(
      configured, input, result.sensor_frame_range_m, result.sensor_frame_share);
  result.sensor_measured_range_m =
      config.sensor_braking_contract.guaranteed_detection_range_m;
  result.sensor_evidence_age_s = config.sensor_braking_contract.maximum_evidence_age_s;
  result.enabled = true;
  result.cruise_limit_mps = config.cruise_speed_mps;
  result.absolute_limit_mps = config.absolute_speed_limit_mps;
  result.route_endpoint_semantics = input.route_endpoint_semantics;
  result.route_endpoint_stop_required =
      routeEndpointHasTerminalStop3D(input.route_endpoint_semantics);
  result.terminal_goal_limit_enabled = input.terminal_goal_limit_enabled;
  const Vec3 braking_direction = motionDirection(input);
  const MppiSpeedPolicyConfig along_motion = alongMotion(config, braking_direction);
  // The laws that stop the vehicle short of evidence, or of the end of what
  // was certified, owe the lag of the loop behind a falling reference. The
  // goal and the turn ahead do not: arriving late at their speed meets
  // nothing, and charged to the curvature law the lag doubled the share of
  // the flight that law bound (6.8 to 15.1 percent of the ticks, r457 to
  // r459 against r461 to r463) and took 0.3 m/s off the mean speed.
  StoppingCapability law_capability = along_motion.stopping_capability;
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
        goal_distance, 0.0, along_motion.stopping_capability,
        along_motion.sensor_braking_contract, input.forward_acceleration_mps2);
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
        route_endpoint_distance, 0.0, law_capability,
        along_motion.sensor_braking_contract, input.forward_acceleration_mps2);
    // A descent stops at the lowest height the route still has to fly, where
    // the route's vertical speed is nought, wherever the distance along it or
    // in the plane stands. Read off the distance alone, a vehicle that ran
    // past its route's corner and came down beside the goal kept 4 to 7 m/s of
    // the goal law while it passed the goal's height, and r3294 struck the
    // ground 6.7 m under it. The deceleration is the one a descent is flown at
    // while the plane brakes too, 0.55 of the vertical guarantee: 0.7 to
    // 0.8 m/s^2 against the guarantee's 1.4 (r3272, r3291, r3305), and read at
    // the whole of it the descents reached their feet at 2.4 to 2.8 m/s and
    // ran 1.2 to 3 m under them. A descent slower than 0.3 m/s is a level
    // flight's jitter and is not braked.
    constexpr double kCombinedBrakingShare{0.55};
    constexpr double kCombinedPlaneShare{0.6};
    // At that height and under it the law still admits the descent a quarter
    // of a metre stops: read at the height itself it gave nought to a vehicle
    // under it, the reference fell to nought and came back within tenths of a
    // second at the foot of every descent, and the speed swung from 4.2 to
    // 2.8 to 4.9 m/s (r3316). Aimed half a metre under the height instead,
    // the descents ended there and further: 0.1 to 0.5 m under their goals
    // off a three to one slant and 1.2 m off a one to one, and the climb back
    // at 0.3 to 1 m/s kept the vehicle from rest for 1 to 3 s at a goal
    // (r3363 to r3379).
    constexpr double kDescentFloorM{0.25};
    constexpr double kLevelDescentMps{0.3};
    const double descent_mps = -static_cast<double>(input.state.vz);
    const double speed_mps =
        std::hypot(std::hypot(input.state.vx, input.state.vy), input.state.vz);
    if (!input.route.empty() && descent_mps > kLevelDescentMps) {
      const double ahead_station_m =
          input.route.back().station_m - *input.route_endpoint_remaining_m;
      double lowest_z_m = input.route.back().position.z;
      for (const RouteSample3D& sample : input.route) {
        if (sample.station_m >= ahead_station_m) {
          lowest_z_m = std::min(lowest_z_m, sample.position.z);
        }
      }
      const MppiSpeedPolicyConfig downward = alongMotion(config, Vec3{0.0, 0.0, -1.0});
      const auto shared = [](StoppingCapability capability, const double share) {
        capability.maximum_commanded_horizontal_deceleration_mps2 *= share;
        capability.guaranteed_horizontal_deceleration_mps2 =
            std::min(capability.guaranteed_horizontal_deceleration_mps2,
                     capability.maximum_commanded_horizontal_deceleration_mps2);
        return capability;
      };
      StoppingCapability vertical_capability = downward.stopping_capability;
      vertical_capability.reaction_latency_s += config.reference_tracking_lag_s;
      const double descent_limit_mps = stoppingLimitedSpeed(
          std::max(static_cast<double>(input.state.z) - lowest_z_m, kDescentFloorM),
          0.0, shared(vertical_capability, kCombinedBrakingShare),
          downward.sensor_braking_contract, input.vertical_forward_acceleration_mps2);
      // The thrust is one: beside a descent being arrested the motion braked at
      // 2.6 to 3.9 m/s^2 of the 5 its law charged, and the goals were passed at
      // 1.2 to 2.2 m/s (r3311, r3312). The law charges 0.6 of it on a descent.
      const double plane_limit_mps = stoppingLimitedSpeed(
          route_endpoint_distance, 0.0, shared(law_capability, kCombinedPlaneShare),
          along_motion.sensor_braking_contract, input.forward_acceleration_mps2);
      result.route_endpoint_limit_mps =
          std::min({result.route_endpoint_limit_mps, plane_limit_mps,
                    descent_limit_mps * speed_mps / descent_mps});
    }
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
        0.0, law_capability, along_motion.sensor_braking_contract,
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
    result.clearance_limit_mps = std::min(
        result.clearance_limit_mps,
        clearanceLimitedSpeed(input.executed_horizon_clearance->constrained_samples,
                              along_motion, input.esdf_evidence_age_s,
                              input.forward_acceleration_mps2));
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
                                       along_motion, input.esdf_evidence_age_s,
                                       input.forward_acceleration_mps2));
  }
  if (input.vertical_motion_clearance.has_value() &&
      input.vertical_motion_clearance->constrained()) {
    // The same laws on the column the vertical motion sweeps. The horizon and
    // the route bound what the vehicle was asked to fly; the vertical channel
    // follows them loosely, and a descent that ran steeper than its route
    // passed the height it was to level at, 2.2 m/s down with a floor two
    // metres under it: the horizon met the floor 1.9 m ahead, and the laws
    // had nothing to brake over (r3177, the floor touched at 0.7 m/s). The
    // column bounds the vertical speed by what stops short of the surface
    // under a descent or over a climb, and the reference by the motion's
    // vertical share: a level flight over a floor owes it nothing.
    const double speed_mps =
        std::hypot(std::hypot(input.state.vx, input.state.vy), input.state.vz);
    const double vertical_share =
        speed_mps > 1.0e-6 ? std::abs(static_cast<double>(input.state.vz)) / speed_mps
                           : 0.0;
    if (vertical_share > 1.0e-6) {
      const double vertical_limit_mps = clearanceLimitedSpeed(
          input.vertical_motion_clearance->constrained_samples,
          alongMotion(config, Vec3{0.0, 0.0, static_cast<double>(input.state.vz)}),
          input.esdf_evidence_age_s, input.vertical_forward_acceleration_mps2);
      result.clearance_limit_mps =
          std::min(result.clearance_limit_mps, vertical_limit_mps / vertical_share);
    }
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
          distance_to_turn_m, turn_speed, along_motion.stopping_capability,
          along_motion.sensor_braking_contract, input.forward_acceleration_mps2);
      result.curvature_limit_mps = std::min(result.curvature_limit_mps, approach_limit);
    }
  }

  result.moving_body_limit_mps =
      movingBodyLimitedSpeed(input, config, result.nearest_moving_body_m);
  result.reference_speed_mps =
      std::min({result.cruise_limit_mps, result.absolute_limit_mps,
                result.curvature_limit_mps, result.sensor_braking_limit_mps,
                result.goal_limit_mps, result.route_endpoint_limit_mps,
                result.route_constraint_limit_mps, result.blocked_route_limit_mps,
                result.clearance_limit_mps, result.route_clearance_limit_mps,
                result.unobserved_frontier_limit_mps, result.moving_body_limit_mps});
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
      std::pair{result.moving_body_limit_mps, MppiSpeedLimiter::kMovingBody},
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
    // The climb starts from the previous reference or from the speed the
    // vehicle already has, whichever is higher. A reference at the actual
    // speed asks no acceleration at that instant, so the airframe follows it
    // as it follows the climb. Started from the previous reference alone,
    // every fall of a law for one tick (a fresh route's first projection, a
    // raw voxel that appears and clears, the gaze crossing the pair's field)
    // sent the reference to 0 and then climbed it from 0 while the vehicle
    // still flew at 2.4 m/s; the route-directed seed braked the vehicle toward
    // it at -4 m/s^2 until the two met (r632, ticks 3448 to 3451: reference
    // 0 -> 0.29 m/s, speed 2.55 -> 2.38). On the acceptance flights r632 to
    // r642 the reference sat rise-limited below the actual speed, with every
    // law admitting more, for 10.3 s per camera flight and 14.8 s per lidar
    // flight, and the flips of the laws cost 13.4 and 20.6 s of flight. The
    // fall stays instant: the reference never exceeds what the laws admit.
    const double rise_floor_mps =
        std::max(std::max(0.0, *input.previous_reference_speed_mps),
                 std::hypot(static_cast<double>(input.state.vx),
                            static_cast<double>(input.state.vy)));
    const double rise_ceiling_mps =
        rise_floor_mps +
        config.reference_speed_rise_mps2 * input.elapsed_since_previous_reference_s;
    if (result.reference_speed_mps > rise_ceiling_mps) {
      result.reference_speed_mps = rise_ceiling_mps;
      result.reference_speed_rise_limited = true;
    }
  }
  // A forward sensor blind along the motion, with nothing observed along it
  // in memory, holds the vehicle where it stands: a target the minimum
  // lookahead ahead drew it on at a tenth of a metre a second into the dark
  // it could not see, every law admitting nothing (r807, 2 m in 13 s).
  const SensorBrakingAssessment3D& along = result.sensor_braking_assessment;
  result.blind_along_motion =
      result.active_limiter == MppiSpeedLimiter::kSensorBraking &&
      along.guaranteed_detection_range_m > 0.0 &&
      along.guaranteed_detection_range_m <=
          along.physical_margin_m + kBlindRangeAboveMarginM;
  // A route with no free metre left ahead of the stop's margin holds the
  // vehicle the same way: on a target the minimum lookahead ahead, re-taken
  // each tick from where the vehicle is, a degree of residual tilt carried it
  // on at 0.2 m/s through the margin and into the crown it had stopped for
  // (r3258, 2.2 m in 12 s, a rotor on a branch at 0.1 m/s).
  result.route_remainder_spent = input.blocked_route_remaining_m.has_value() &&
                                 result.blocked_route_limit_mps <= 0.0;
  result.target_lookahead_m =
      result.blind_along_motion || result.route_remainder_spent
          ? 0.0
          : std::clamp(result.reference_speed_mps * config.horizon_duration_s,
                       config.minimum_target_lookahead_m,
                       config.maximum_target_lookahead_m);
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
    case MppiSpeedLimiter::kMovingBody:
      return "moving_body";
  }
  return "unknown";
}

} // namespace drone_city_nav
