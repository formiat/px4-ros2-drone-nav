#include "drone_city_nav/mppi/mppi_control_sequence.hpp"

#include "drone_city_nav/mppi/mppi_reference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace drone_city_nav::mppi {
namespace {

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

struct RouteSample {
  float x_m{0.0F};
  float y_m{0.0F};
  float z_m{0.0F};
  float tangent_x{1.0F};
  float tangent_y{0.0F};
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
    const float length_m = std::hypot(dx, dy);
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
        .station_m = station_m,
        .valid = true,
    };
  }
  return {};
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

std::vector<Control> buildGuideDirectedNominalSeed(
    const State& initial, const State& target,
    const std::span<const RouteSample3D> route, const float initial_route_station_m,
    const float reference_speed_mps, const DynamicsConfig& dynamics,
    const std::size_t steps, const Control previous_applied_control) {
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
  for (std::size_t index = 0U; index < steps; ++index) {
    const float elapsed_s = static_cast<float>(index + 1U) * dynamics.dt_s;
    const RouteSample route_sample =
        sampleRoute(route, initial_route_station_m + requested_speed_mps * elapsed_s);
    const float tangent_x = route_sample.valid ? route_sample.tangent_x : direction_x;
    const float tangent_y = route_sample.valid ? route_sample.tangent_y : direction_y;
    const float position_error_x =
        (route_sample.valid ? route_sample.x_m : target.x) - predicted.x;
    const float position_error_y =
        (route_sample.valid ? route_sample.y_m : target.y) - predicted.y;
    const float desired_vx = requested_speed_mps * tangent_x;
    const float desired_vy = requested_speed_mps * tangent_y;
    const float desired_z = route_sample.valid ? route_sample.z_m : target.z;
    seed[index] = Control{
        .ax = 0.8F * (desired_vx - predicted.vx) + 0.35F * position_error_x,
        .ay = 0.8F * (desired_vy - predicted.vy) + 0.35F * position_error_y,
        .az = 0.8F * (desired_z - predicted.z) - 0.5F * predicted.vz,
        .yaw_accel = 0.0F,
    };
    limitControlSequence(std::span<Control>{&seed[index], 1U}, dynamics, previous,
                         dynamics.dt_s);
    previous = seed[index];
    predicted = integrateReference(predicted, seed[index], dynamics);
  }
  return seed;
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
  float best_squared_distance = std::numeric_limits<float>::infinity();
  float best_station_m = minimum_station_m;
  for (std::size_t index = 0U; index + 1U < route.size(); ++index) {
    const RouteSample3D& first = route[index];
    const RouteSample3D& second = route[index + 1U];
    if (second.station_m + 1.0e-5F < minimum_station_m) {
      continue;
    }
    const float dx = second.x_m - first.x_m;
    const float dy = second.y_m - first.y_m;
    const float squared_length = dx * dx + dy * dy;
    const float station_length_m = second.station_m - first.station_m;
    if (!(squared_length > 1.0e-8F) || !(station_length_m > 1.0e-5F)) {
      continue;
    }
    const float minimum_ratio = std::clamp(
        (minimum_station_m - first.station_m) / station_length_m, 0.0F, 1.0F);
    const float ratio = std::clamp(
        ((state.x - first.x_m) * dx + (state.y - first.y_m) * dy) / squared_length,
        minimum_ratio, 1.0F);
    const float offset_x = state.x - (first.x_m + ratio * dx);
    const float offset_y = state.y - (first.y_m + ratio * dy);
    const float squared_distance = offset_x * offset_x + offset_y * offset_y;
    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      best_station_m = first.station_m + ratio * station_length_m;
    }
  }
  return std::isfinite(best_squared_distance)
             ? std::optional<float>{std::max(best_station_m, minimum_station_m)}
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
