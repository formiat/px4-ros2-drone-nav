#include "drone_city_nav/mppi/mppi_control_sequence.hpp"

#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/mppi/mppi_route_projection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <ranges>
#include <stdexcept>

namespace drone_city_nav::mppi {
namespace {

inline constexpr float kRouteCrossTrackVelocityGainPerSecond{1.0F};

[[nodiscard]] float clampMagnitude(const float value, const float limit) noexcept {
  return std::clamp(value, -limit, limit);
}

void clampHorizontal(float& x, float& y, const float limit) noexcept {
  const float magnitude = std::hypot(x, y);
  if (magnitude > limit && magnitude > 0.0F) {
    const float scale = limit / magnitude;
    x *= scale;
    y *= scale;
  }
}

void clampTranslational(float& x, float& y, float& z, const float limit) noexcept {
  const float magnitude = std::hypot(std::hypot(x, y), z);
  if (magnitude > limit && magnitude > 0.0F) {
    const float scale = limit / magnitude;
    x *= scale;
    y *= scale;
    z *= scale;
  }
}

struct RouteSample {
  float x_m{0.0F};
  float y_m{0.0F};
  float z_m{0.0F};
  float tangent_x{1.0F};
  float tangent_y{0.0F};
  float tangent_z{0.0F};
  float station_m{0.0F};
  bool valid{false};
};

[[nodiscard]] RouteSample sampleRoute(const std::span<const RouteSample3D> route,
                                      const float requested_station_m) noexcept {
  if (route.size() < 2U) {
    return {};
  }
  const float station_m =
      std::clamp(requested_station_m, route.front().station_m, route.back().station_m);
  for (std::size_t index = 0U; index + 1U < route.size(); ++index) {
    const RouteSample3D& first = route[index];
    const RouteSample3D& second = route[index + 1U];
    if (station_m > second.station_m && index + 2U < route.size()) {
      continue;
    }
    const float dx = second.x_m - first.x_m;
    const float dy = second.y_m - first.y_m;
    const float dz = second.z_m - first.z_m;
    const float length_m = std::hypot(std::hypot(dx, dy), dz);
    if (!(length_m > 1.0e-5F)) {
      continue;
    }
    const float station_length_m = second.station_m - first.station_m;
    const float ratio =
        station_length_m > 1.0e-5F
            ? std::clamp((station_m - first.station_m) / station_length_m, 0.0F, 1.0F)
            : 0.0F;
    return RouteSample{
        .x_m = std::lerp(first.x_m, second.x_m, ratio),
        .y_m = std::lerp(first.y_m, second.y_m, ratio),
        .z_m = std::lerp(first.z_m, second.z_m, ratio),
        .tangent_x = dx / length_m,
        .tangent_y = dy / length_m,
        .tangent_z = dz / length_m,
        .station_m = station_m,
        .valid = true,
    };
  }
  return {};
}

[[nodiscard]] float
routeStoppingAcceleration(const RouteSample& sample,
                          const StoppingCapability& stopping_capability) noexcept {
  const float horizontal_component = std::hypot(sample.tangent_x, sample.tangent_y);
  const float vertical_component = std::abs(sample.tangent_z);
  float limit = std::numeric_limits<float>::infinity();
  if (horizontal_component > 1.0e-5F) {
    limit = std::min(limit,
                     static_cast<float>(
                         stopping_capability.guaranteed_horizontal_deceleration_mps2) /
                         horizontal_component);
  }
  if (vertical_component > 1.0e-5F) {
    limit = std::min(
        limit,
        static_cast<float>(stopping_capability.guaranteed_vertical_deceleration_mps2) /
            vertical_component);
  }
  return std::isfinite(limit) ? limit : 0.0F;
}

[[nodiscard]] float
finiteRouteSpeedLimit(const float remaining_station_m, const RouteSample& sample,
                      const StoppingCapability& stopping_capability) noexcept {
  const float deceleration_mps2 =
      routeStoppingAcceleration(sample, stopping_capability);
  if (!(remaining_station_m > 0.0F) || !(deceleration_mps2 > 0.0F)) {
    return 0.0F;
  }
  const float latency_s =
      static_cast<float>(std::max(0.0, stopping_capability.reaction_latency_s));
  const float latency_velocity_mps = deceleration_mps2 * latency_s;
  return std::max(0.0F, std::sqrt(latency_velocity_mps * latency_velocity_mps +
                                  2.0F * deceleration_mps2 * remaining_station_m) -
                            latency_velocity_mps);
}

} // namespace

Control interpolateControl(const Control& first, const Control& second,
                           const float ratio) noexcept {
  const float clamped = std::clamp(ratio, 0.0F, 1.0F);
  return Control{
      .ax = std::lerp(first.ax, second.ax, clamped),
      .ay = std::lerp(first.ay, second.ay, clamped),
      .az = std::lerp(first.az, second.az, clamped),
      .yaw_accel = std::lerp(first.yaw_accel, second.yaw_accel, clamped),
  };
}

std::vector<Control> shiftControlSequence(const std::span<const Control> controls,
                                          const float dt_s, const double elapsed_s) {
  if (!(dt_s > 0.0F) || !std::isfinite(dt_s) || !std::isfinite(elapsed_s) ||
      elapsed_s < 0.0) {
    throw std::invalid_argument{"invalid MPPI control sequence timing"};
  }
  if (controls.empty()) {
    return {};
  }
  const double offset_steps = elapsed_s / static_cast<double>(dt_s);
  if (offset_steps >= static_cast<double>(controls.size())) {
    return std::vector<Control>(controls.size());
  }

  std::vector<Control> shifted;
  shifted.reserve(controls.size());
  const std::size_t last = controls.size() - 1U;
  for (std::size_t index = 0U; index < controls.size(); ++index) {
    const double source = static_cast<double>(index) + offset_steps;
    if (source >= static_cast<double>(last)) {
      shifted.push_back(controls[last]);
      continue;
    }
    const std::size_t lower = static_cast<std::size_t>(std::floor(source));
    const std::size_t upper = lower + 1U;
    shifted.push_back(
        interpolateControl(controls[lower], controls[upper],
                           static_cast<float>(source - static_cast<double>(lower))));
  }
  return shifted;
}

void limitControlSequence(const std::span<Control> controls,
                          const DynamicsConfig& dynamics,
                          const Control previous_applied_control,
                          const float first_control_interval_s) noexcept {
  Control previous = previous_applied_control;
  for (std::size_t step = 0U; step < controls.size(); ++step) {
    const float interval_s = step == 0U ? first_control_interval_s : dynamics.dt_s;
    const float maximum_delta = dynamics.maximum_control_jerk_mps3 * interval_s;
    Control& control = controls[step];
    clampHorizontal(control.ax, control.ay,
                    dynamics.maximum_horizontal_acceleration_mps2);
    control.az =
        clampMagnitude(control.az, dynamics.maximum_vertical_acceleration_mps2);
    control.yaw_accel =
        clampMagnitude(control.yaw_accel, dynamics.maximum_yaw_acceleration_radps2);
    control.ax = std::clamp(control.ax, previous.ax - maximum_delta,
                            previous.ax + maximum_delta);
    control.ay = std::clamp(control.ay, previous.ay - maximum_delta,
                            previous.ay + maximum_delta);
    control.az = std::clamp(control.az, previous.az - maximum_delta,
                            previous.az + maximum_delta);
    previous = control;
  }
}

namespace {

std::vector<Control>
buildGuideDirectedSeed(const State& initial, const State& target,
                       const std::span<const RouteSample3D> route,
                       const float initial_route_station_m,
                       const float reference_speed_mps, const DynamicsConfig& dynamics,
                       const std::size_t steps, const Control previous_applied_control,
                       const StoppingCapability& stopping_capability) {
  std::vector<Control> seed(steps);
  const float dx = target.x - initial.x;
  const float dy = target.y - initial.y;
  const float distance = std::hypot(dx, dy);
  const float direction_x = distance > 1.0e-3F ? dx / distance : 0.0F;
  const float direction_y = distance > 1.0e-3F ? dy / distance : 0.0F;
  const float requested_speed_mps =
      std::max(0.0F, std::isfinite(reference_speed_mps) ? reference_speed_mps : 0.0F);
  State predicted = initial;
  Control previous = previous_applied_control;
  float route_target_station_m = initial_route_station_m;
  float route_projection_station_m = initial_route_station_m;
  for (std::size_t index = 0U; index < steps; ++index) {
    if (route.size() >= 2U) {
      const MppiRouteProjection3D projection =
          projectOntoMppiRoute3D(predicted, route, route_projection_station_m);
      if (projection.valid) {
        route_projection_station_m = projection.station_m;
      }
    }
    const RouteSample current_route_sample =
        sampleRoute(route, route_projection_station_m);
    const float route_speed_mps =
        current_route_sample.valid
            ? std::min(
                  requested_speed_mps,
                  finiteRouteSpeedLimit(std::max(0.0F, route.back().station_m -
                                                           route_projection_station_m),
                                        current_route_sample, stopping_capability))
            : requested_speed_mps;
    route_target_station_m =
        current_route_sample.valid
            ? std::min(route.back().station_m,
                       std::max(route_target_station_m, route_projection_station_m) +
                           route_speed_mps * dynamics.dt_s)
            : route_target_station_m;
    const RouteSample route_sample = sampleRoute(route, route_target_station_m);
    const float tangent_x = route_sample.valid ? route_sample.tangent_x : direction_x;
    const float tangent_y = route_sample.valid ? route_sample.tangent_y : direction_y;
    const float position_error_x =
        (route_sample.valid ? route_sample.x_m : target.x) - predicted.x;
    const float position_error_y =
        (route_sample.valid ? route_sample.y_m : target.y) - predicted.y;
    const float desired_z = route_sample.valid ? route_sample.z_m : target.z;
    const float desired_vx = route_speed_mps * tangent_x;
    float desired_vy = route_speed_mps * tangent_y;
    float desired_vz =
        route_speed_mps * (route_sample.valid ? route_sample.tangent_z : 0.0F);
    float route_desired_vx = desired_vx;
    if (route_sample.valid) {
      const float tangent_error_m = position_error_x * route_sample.tangent_x +
                                    position_error_y * route_sample.tangent_y +
                                    (desired_z - predicted.z) * route_sample.tangent_z;
      route_desired_vx += kRouteCrossTrackVelocityGainPerSecond *
                          (position_error_x - tangent_error_m * route_sample.tangent_x);
      desired_vy += kRouteCrossTrackVelocityGainPerSecond *
                    (position_error_y - tangent_error_m * route_sample.tangent_y);
      desired_vz +=
          kRouteCrossTrackVelocityGainPerSecond *
          ((desired_z - predicted.z) - tangent_error_m * route_sample.tangent_z);
      clampHorizontal(
          route_desired_vx, desired_vy,
          std::min(requested_speed_mps, dynamics.maximum_horizontal_speed_mps));
      desired_vz =
          clampMagnitude(desired_vz, std::min(requested_speed_mps,
                                              dynamics.maximum_vertical_speed_mps));
      clampTranslational(
          route_desired_vx, desired_vy, desired_vz,
          std::min(requested_speed_mps, dynamics.maximum_translational_speed_mps));
    }
    const float route_velocity_gain =
        1.0F / std::max(dynamics.dt_s, std::numeric_limits<float>::epsilon());
    const float horizontal_velocity_gain =
        route_sample.valid ? route_velocity_gain : 0.8F;
    const float horizontal_position_gain = route_sample.valid ? 0.0F : 0.35F;
    const float vertical_velocity_gain =
        route_sample.valid ? route_velocity_gain : 0.5F;
    const float vertical_position_gain = route_sample.valid ? 0.0F : 0.8F;
    seed[index] = Control{
        .ax = horizontal_velocity_gain * (route_desired_vx - predicted.vx) +
              horizontal_position_gain * position_error_x,
        .ay = horizontal_velocity_gain * (desired_vy - predicted.vy) +
              horizontal_position_gain * position_error_y,
        .az = vertical_position_gain * (desired_z - predicted.z) +
              vertical_velocity_gain * (desired_vz - predicted.vz),
        .yaw_accel = 0.0F,
    };
    limitControlSequence(std::span<Control>{&seed[index], 1U}, dynamics, previous,
                         dynamics.dt_s);
    previous = seed[index];
    predicted = integrateReference(predicted, seed[index], dynamics);
  }
  return seed;
}

std::vector<Control> buildStraightRouteTerminalRestSeed(
    const State& initial, const State& target,
    const std::span<const RouteSample3D> route, const float initial_route_station_m,
    const float reference_speed_mps, const DynamicsConfig& dynamics,
    const std::size_t steps, const Control previous_applied_control,
    const StoppingCapability& stopping_capability) {
  if (route.size() < 2U || steps == 0U) {
    return buildGuideDirectedSeed(initial, target, route, initial_route_station_m,
                                  reference_speed_mps, dynamics, steps,
                                  previous_applied_control, stopping_capability);
  }
  const float horizon_duration_s = static_cast<float>(steps) * dynamics.dt_s;
  // A rest-to-rest finite maneuver has roughly half the cruise speed on average.
  const float terminal_station_m =
      std::min(route.back().station_m,
               initial_route_station_m +
                   0.5F * std::max(0.0F, reference_speed_mps) * horizon_duration_s);
  const RouteSample initial_route = sampleRoute(route, initial_route_station_m);
  const RouteSample terminal_route = sampleRoute(route, terminal_station_m);
  if (!initial_route.valid || !terminal_route.valid) {
    return buildGuideDirectedSeed(initial, target, route, initial_route_station_m,
                                  reference_speed_mps, dynamics, steps,
                                  previous_applied_control, stopping_capability);
  }
  const float route_interval_m = terminal_route.station_m - initial_route.station_m;
  const float chord_length_m =
      std::hypot(std::hypot(terminal_route.x_m - initial_route.x_m,
                            terminal_route.y_m - initial_route.y_m),
                 terminal_route.z_m - initial_route.z_m);
  constexpr float kStraightRouteToleranceM{1.0e-3F};
  if (route_interval_m > chord_length_m + kStraightRouteToleranceM) {
    return buildGuideDirectedSeed(initial, target, route, initial_route_station_m,
                                  reference_speed_mps, dynamics, steps,
                                  previous_applied_control, stopping_capability);
  }

  std::vector<Control> seed(steps);
  State predicted = initial;
  Control previous = previous_applied_control;
  for (std::size_t index = 0U; index < steps; ++index) {
    const float remaining_s = static_cast<float>(steps - index) * dynamics.dt_s;
    const float inverse_remaining_s = 1.0F / remaining_s;
    // Receding-horizon gains for the cubic boundary-value solution with zero
    // terminal velocity. This connector is valid only on a straight route
    // interval; curved intervals must follow their geometry above.
    const float position_gain = 6.0F * inverse_remaining_s * inverse_remaining_s;
    const float velocity_gain = 4.0F * inverse_remaining_s;
    seed[index] = Control{
        .ax = position_gain * (terminal_route.x_m - predicted.x) -
              velocity_gain * predicted.vx + dynamics.linear_drag_1ps * predicted.vx,
        .ay = position_gain * (terminal_route.y_m - predicted.y) -
              velocity_gain * predicted.vy + dynamics.linear_drag_1ps * predicted.vy,
        .az = position_gain * (terminal_route.z_m - predicted.z) -
              velocity_gain * predicted.vz + dynamics.linear_drag_1ps * predicted.vz,
        .yaw_accel = -velocity_gain * predicted.yaw_rate,
    };
    limitControlSequence(std::span<Control>{&seed[index], 1U}, dynamics, previous,
                         dynamics.dt_s);
    previous = seed[index];
    predicted = integrateReference(predicted, seed[index], dynamics);
  }
  return seed;
}

} // namespace

std::vector<Control> buildGuideDirectedNominalSeed(
    const State& initial, const State& target,
    const std::span<const RouteSample3D> route, const float initial_route_station_m,
    const float reference_speed_mps, const DynamicsConfig& dynamics,
    const std::size_t steps, const Control previous_applied_control,
    const StoppingCapability& stopping_capability) {
  return buildGuideDirectedSeed(initial, target, route, initial_route_station_m,
                                reference_speed_mps, dynamics, steps,
                                previous_applied_control, stopping_capability);
}

std::vector<Control> buildFiniteRouteDirectedSeed(
    const State& initial, const State& target,
    const std::span<const RouteSample3D> route, const float initial_route_station_m,
    const float reference_speed_mps, const DynamicsConfig& dynamics,
    const std::size_t steps, const Control previous_applied_control,
    const StoppingCapability& stopping_capability) {
  // A straight interval can use one rest-to-rest connector. Curved intervals
  // retain the actual route geometry; the finite-execution admission layer then
  // owns suffix backoff and the independently certified terminal braking tail.
  return buildStraightRouteTerminalRestSeed(
      initial, target, route, initial_route_station_m, reference_speed_mps, dynamics,
      steps, previous_applied_control, stopping_capability);
}

std::vector<Control> buildCooperativeSeparationAcquisitionCandidates(
    const State& initial, const State& target,
    const std::span<const RouteSample3D> route, const float initial_route_station_m,
    const float reference_speed_mps,
    const CooperativeSeparationAcquisition& acquisition, const DynamicsConfig& dynamics,
    const CooperativeConfig& cooperative, const std::size_t steps,
    const Control previous_applied_control, const float first_control_interval_s,
    const StoppingCapability& stopping_capability) {
  if (steps == 0U || !(reference_speed_mps >= 0.0F)) {
    throw std::invalid_argument{"invalid cooperative acquisition input"};
  }
  constexpr std::array speed_scales{1.0F, 0.8F, 0.6F, 0.4F, 0.0F, -0.35F};
  constexpr std::array separation_scales{1.0F, 1.0F, 0.9F, 0.8F, 1.0F, 1.0F};
  const RouteSample current_route = sampleRoute(route, initial_route_station_m);
  const float target_distance = std::hypot(target.x - initial.x, target.y - initial.y);
  float forward_x = 1.0F;
  float forward_y = 0.0F;
  if (current_route.valid) {
    forward_x = current_route.tangent_x;
    forward_y = current_route.tangent_y;
  } else if (target_distance > 1.0e-3F) {
    forward_x = (target.x - initial.x) / target_distance;
    forward_y = (target.y - initial.y) / target_distance;
  }
  const float reverse_distance_m = std::max(10.0F, 2.0F * reference_speed_mps);
  const State reverse_target{.x = initial.x - reverse_distance_m * forward_x,
                             .y = initial.y - reverse_distance_m * forward_y,
                             .z = initial.z};
  const Control separation = resolveCooperativePreferredAcceleration(
      acquisition.preference, dynamics, cooperative);
  const std::size_t active_steps =
      std::clamp<std::size_t>(static_cast<std::size_t>(std::ceil(
                                  cooperative.candidate_duration_s / dynamics.dt_s)),
                              1U, steps);
  std::vector<Control> candidates(kCooperativeAcquisitionCandidateCount * steps);
  for (std::size_t candidate_index = 0U;
       candidate_index < kCooperativeAcquisitionCandidateCount; ++candidate_index) {
    const float speed_scale = speed_scales.at(candidate_index);
    std::vector<Control> candidate;
    if (speed_scale > 0.0F) {
      candidate = buildFiniteRouteDirectedSeed(
          initial, target, route, initial_route_station_m,
          reference_speed_mps * speed_scale, dynamics, steps, previous_applied_control,
          stopping_capability);
    } else if (speed_scale < 0.0F) {
      candidate = buildGuideDirectedNominalSeed(
          initial, reverse_target, {}, 0.0F, -reference_speed_mps * speed_scale,
          dynamics, steps, previous_applied_control, stopping_capability);
    } else {
      candidate = buildGuideDirectedNominalSeed(
          initial, initial, {}, 0.0F, 0.0F, dynamics, steps, previous_applied_control,
          stopping_capability);
    }
    for (std::size_t step = 0U; step < active_steps; ++step) {
      candidate[step].ax += separation_scales.at(candidate_index) * separation.ax;
      candidate[step].ay += separation_scales.at(candidate_index) * separation.ay;
      candidate[step].az += separation_scales.at(candidate_index) * separation.az;
    }
    limitControlSequence(candidate, dynamics, previous_applied_control,
                         first_control_interval_s);
    std::ranges::copy(candidate, candidates.begin() + static_cast<std::ptrdiff_t>(
                                                          candidate_index * steps));
  }
  return candidates;
}

std::vector<Control> buildNonCooperativeSeparationAcquisitionCandidates(
    const State& initial, const State& target,
    const std::span<const RouteSample3D> route, const float initial_route_station_m,
    const float reference_speed_mps,
    const NonCooperativeSeparationAcquisition& acquisition,
    const DynamicsConfig& dynamics, const std::size_t steps,
    const Control previous_applied_control, const float first_control_interval_s,
    const StoppingCapability& stopping_capability) {
  if (steps == 0U || !(reference_speed_mps >= 0.0F) ||
      !(acquisition.candidate_acceleration_fraction > 0.0F) ||
      acquisition.candidate_acceleration_fraction > 1.0F ||
      !(acquisition.candidate_duration_s > 0.0F)) {
    throw std::invalid_argument{"invalid non-cooperative acquisition input"};
  }
  const RouteSample current_route = sampleRoute(route, initial_route_station_m);
  const float target_dx = target.x - initial.x;
  const float target_dy = target.y - initial.y;
  const float target_distance_m = std::hypot(target_dx, target_dy);
  float forward_x = 1.0F;
  float forward_y = 0.0F;
  if (current_route.valid) {
    forward_x = current_route.tangent_x;
    forward_y = current_route.tangent_y;
  } else if (target_distance_m > 1.0e-3F) {
    forward_x = target_dx / target_distance_m;
    forward_y = target_dy / target_distance_m;
  }
  const float horizontal_acceleration = acquisition.candidate_acceleration_fraction *
                                        dynamics.maximum_horizontal_acceleration_mps2;
  const float vertical_acceleration = acquisition.candidate_acceleration_fraction *
                                      dynamics.maximum_vertical_acceleration_mps2;
  const float away_horizontal_norm =
      std::hypot(acquisition.threat_direction_x, acquisition.threat_direction_y);
  Control away;
  if (away_horizontal_norm > 1.0e-5F) {
    away.ax = -horizontal_acceleration * acquisition.threat_direction_x /
              away_horizontal_norm;
    away.ay = -horizontal_acceleration * acquisition.threat_direction_y /
              away_horizontal_norm;
  }
  away.az = std::clamp(-vertical_acceleration * acquisition.threat_direction_z,
                       -vertical_acceleration, vertical_acceleration);
  const std::array biases{
      Control{},
      away,
      Control{.ax = -forward_y * horizontal_acceleration,
              .ay = forward_x * horizontal_acceleration},
      Control{.ax = forward_y * horizontal_acceleration,
              .ay = -forward_x * horizontal_acceleration},
      Control{.az = vertical_acceleration},
      Control{.az = -vertical_acceleration},
      Control{.ax = -forward_x * horizontal_acceleration,
              .ay = -forward_y * horizontal_acceleration},
      Control{},
  };
  const std::size_t active_steps =
      std::clamp<std::size_t>(static_cast<std::size_t>(std::ceil(
                                  acquisition.candidate_duration_s / dynamics.dt_s)),
                              1U, steps);
  const float reverse_distance_m = std::max(10.0F, 2.0F * reference_speed_mps);
  const State reverse_target{.x = initial.x - reverse_distance_m * forward_x,
                             .y = initial.y - reverse_distance_m * forward_y,
                             .z = initial.z};
  std::vector<Control> candidates(kNonCooperativeAcquisitionCandidateCount * steps);
  for (std::size_t candidate_index = 0U;
       candidate_index < kNonCooperativeAcquisitionCandidateCount; ++candidate_index) {
    const NonCooperativeManeuver maneuver =
        static_cast<NonCooperativeManeuver>(candidate_index);
    std::vector<Control> candidate;
    if (maneuver == NonCooperativeManeuver::kBackward) {
      candidate = buildGuideDirectedNominalSeed(
          initial, reverse_target, {}, 0.0F, reference_speed_mps, dynamics, steps,
          previous_applied_control, stopping_capability);
    } else if (maneuver == NonCooperativeManeuver::kBrake) {
      candidate = buildGuideDirectedNominalSeed(
          initial, initial, {}, 0.0F, 0.0F, dynamics, steps, previous_applied_control,
          stopping_capability);
    } else {
      candidate = buildFiniteRouteDirectedSeed(
          initial, target, route, initial_route_station_m, reference_speed_mps,
          dynamics, steps, previous_applied_control, stopping_capability);
    }
    const Control bias = biases.at(candidate_index);
    for (std::size_t step = 0U; step < active_steps; ++step) {
      candidate[step].ax += bias.ax;
      candidate[step].ay += bias.ay;
      candidate[step].az += bias.az;
    }
    limitControlSequence(candidate, dynamics, previous_applied_control,
                         first_control_interval_s);
    std::ranges::copy(candidate, candidates.begin() + static_cast<std::ptrdiff_t>(
                                                          candidate_index * steps));
  }
  return candidates;
}

std::vector<Control> buildCooperativeManeuverCandidates(
    const State& initial, const State& target, const std::span<const Control> nominal,
    const DynamicsConfig& dynamics, const CooperativeConfig& cooperative,
    const Control previous_applied_control, const float first_control_interval_s) {
  if (nominal.empty() || !(dynamics.dt_s > 0.0F) ||
      !(cooperative.candidate_acceleration_fraction > 0.0F) ||
      cooperative.candidate_acceleration_fraction > 1.0F ||
      !(cooperative.candidate_duration_s > 0.0F)) {
    throw std::invalid_argument{"invalid cooperative maneuver candidate input"};
  }
  std::vector<Control> candidates(kCooperativeManeuverCandidateCount * nominal.size());
  const float target_dx = target.x - initial.x;
  const float target_dy = target.y - initial.y;
  const float horizontal_speed = std::hypot(initial.vx, initial.vy);
  const float target_distance = std::hypot(target_dx, target_dy);
  float forward_x = 1.0F;
  float forward_y = 0.0F;
  if (horizontal_speed > 0.5F) {
    forward_x = initial.vx / horizontal_speed;
    forward_y = initial.vy / horizontal_speed;
  } else if (target_distance > 1.0e-3F) {
    forward_x = target_dx / target_distance;
    forward_y = target_dy / target_distance;
  }
  const float horizontal_bias = cooperative.candidate_acceleration_fraction *
                                dynamics.maximum_horizontal_acceleration_mps2;
  const float vertical_bias = cooperative.candidate_acceleration_fraction *
                              dynamics.maximum_vertical_acceleration_mps2;
  const std::size_t active_steps =
      std::clamp<std::size_t>(static_cast<std::size_t>(std::ceil(
                                  cooperative.candidate_duration_s / dynamics.dt_s)),
                              1U, nominal.size());
  const auto bias_for = [&](const CooperativeManeuver maneuver) {
    switch (maneuver) {
      case CooperativeManeuver::kKeep:
        return Control{};
      case CooperativeManeuver::kClimb:
        return Control{.az = vertical_bias};
      case CooperativeManeuver::kDescend:
        return Control{.az = -vertical_bias};
      case CooperativeManeuver::kLeft:
        return Control{.ax = -forward_y * horizontal_bias,
                       .ay = forward_x * horizontal_bias};
      case CooperativeManeuver::kRight:
        return Control{.ax = forward_y * horizontal_bias,
                       .ay = -forward_x * horizontal_bias};
      case CooperativeManeuver::kSlow:
        return Control{.ax = -forward_x * horizontal_bias,
                       .ay = -forward_y * horizontal_bias};
    }
    return Control{};
  };
  for (std::size_t candidate_index = 0U;
       candidate_index < kCooperativeManeuverCandidateCount; ++candidate_index) {
    const CooperativeManeuver maneuver =
        static_cast<CooperativeManeuver>(candidate_index);
    const Control bias = bias_for(maneuver);
    std::span<Control> candidate =
        std::span{candidates}.subspan(candidate_index * nominal.size(), nominal.size());
    for (std::size_t step = 0U; step < nominal.size(); ++step) {
      candidate[step] = nominal[step];
      if (step < active_steps) {
        candidate[step].ax += bias.ax;
        candidate[step].ay += bias.ay;
        candidate[step].az += bias.az;
      }
    }
    limitControlSequence(candidate, dynamics, previous_applied_control,
                         first_control_interval_s);
  }
  return candidates;
}

std::optional<float>
projectForwardRouteStation(const std::span<const RouteSample3D> route,
                           const State& state, const float minimum_station_m) noexcept {
  if (route.size() < 2U || !std::isfinite(minimum_station_m)) {
    return std::nullopt;
  }
  const MppiRouteProjection3D projection =
      projectOntoMppiRoute3D(state, route, minimum_station_m);
  return projection.valid
             ? std::optional<float>{std::max(projection.station_m, minimum_station_m)}
             : std::nullopt;
}

RiskTier maximumRequiredRiskTier(const std::span<const RouteSample3D> route,
                                 const float begin_station_m,
                                 const float end_station_m) noexcept {
  const float begin = std::min(begin_station_m, end_station_m);
  const float end = std::max(begin_station_m, end_station_m);
  RiskTier result = RiskTier::kPreferred;
  for (std::size_t index = 0U; index < route.size(); ++index) {
    const float previous_station =
        index == 0U ? route[index].station_m : route[index - 1U].station_m;
    const float next_station = index + 1U < route.size() ? route[index + 1U].station_m
                                                         : route[index].station_m;
    if (next_station < begin || previous_station > end) {
      continue;
    }
    result = static_cast<RiskTier>(
        std::max(static_cast<std::uint8_t>(result),
                 static_cast<std::uint8_t>(route[index].required_risk_tier)));
  }
  return result == RiskTier::kCollision ? RiskTier::kCritical : result;
}

} // namespace drone_city_nav::mppi
