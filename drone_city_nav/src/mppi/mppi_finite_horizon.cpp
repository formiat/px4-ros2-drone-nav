#include "drone_city_nav/mppi/mppi_finite_horizon.hpp"

#include "drone_city_nav/mppi/mppi_reference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace drone_city_nav::mppi {
namespace {

[[nodiscard]] float moveTowardZero(const float value,
                                   const float maximum_delta) noexcept {
  if (value > maximum_delta) {
    return value - maximum_delta;
  }
  if (value < -maximum_delta) {
    return value + maximum_delta;
  }
  return 0.0F;
}

[[nodiscard]] bool translationalControlIsZero(const Control& control) noexcept {
  constexpr float kControlTolerance{1.0e-6F};
  return std::abs(control.ax) <= kControlTolerance &&
         std::abs(control.ay) <= kControlTolerance &&
         std::abs(control.az) <= kControlTolerance;
}

[[nodiscard]] float arrivalShape(const std::size_t step, const std::size_t active_steps,
                                 const std::size_t ramp_steps) noexcept {
  const float rising = static_cast<float>(step + 1U) / static_cast<float>(ramp_steps);
  const float falling =
      static_cast<float>(active_steps - step) / static_cast<float>(ramp_steps);
  return std::min({1.0F, rising, falling});
}

[[nodiscard]] bool controlWithinLimits(const Control& control, const Control& previous,
                                       const DynamicsConfig& dynamics) noexcept {
  constexpr float kTolerance{1.0e-4F};
  const float maximum_delta = dynamics.maximum_control_jerk_mps3 * dynamics.dt_s;
  return std::hypot(control.ax, control.ay) <=
             dynamics.maximum_horizontal_acceleration_mps2 + kTolerance &&
         std::abs(control.az) <=
             dynamics.maximum_vertical_acceleration_mps2 + kTolerance &&
         std::abs(control.yaw_accel) <=
             dynamics.maximum_yaw_acceleration_radps2 + kTolerance &&
         std::abs(control.ax - previous.ax) <= maximum_delta + kTolerance &&
         std::abs(control.ay - previous.ay) <= maximum_delta + kTolerance &&
         std::abs(control.az - previous.az) <= maximum_delta + kTolerance;
}

void appendControl(FiniteHorizon& horizon, const Control& control,
                   const DynamicsConfig& dynamics) {
  horizon.controls.push_back(control);
  horizon.states.push_back(
      integrateReference(horizon.states.back(), control, dynamics));
}

[[nodiscard]] bool appendControlRelease(FiniteHorizon& horizon, Control& previous,
                                        const DynamicsConfig& dynamics,
                                        const std::size_t maximum_steps) {
  const float maximum_delta = dynamics.maximum_control_jerk_mps3 * dynamics.dt_s;
  while (!translationalControlIsZero(previous)) {
    if (horizon.arrival_control_count >= maximum_steps) {
      return false;
    }
    const Control control{
        .ax = moveTowardZero(previous.ax, maximum_delta),
        .ay = moveTowardZero(previous.ay, maximum_delta),
        .az = moveTowardZero(previous.az, maximum_delta),
    };
    if (!controlWithinLimits(control, previous, dynamics)) {
      return false;
    }
    appendControl(horizon, control, dynamics);
    previous = control;
    ++horizon.arrival_control_count;
  }
  previous.yaw_accel = 0.0F;
  return true;
}

[[nodiscard]] std::optional<std::vector<Control>>
buildArrivalControls(const State& initial, const DynamicsConfig& dynamics,
                     const std::size_t maximum_steps,
                     const float velocity_tolerance_mps) {
  const float dt_s = dynamics.dt_s;
  const float drag = std::max(0.0F, 1.0F - dynamics.linear_drag_1ps * dt_s);
  const float initial_speed =
      std::hypot(std::hypot(initial.vx, initial.vy), initial.vz);
  const bool translation_required = initial_speed > velocity_tolerance_mps;
  const bool yaw_required = std::abs(initial.yaw_rate) > velocity_tolerance_mps;
  if (!translation_required && !yaw_required) {
    return std::vector<Control>{Control{}};
  }

  for (std::size_t active_steps = 2U; active_steps + 1U <= maximum_steps;
       ++active_steps) {
    for (std::size_t ramp_steps = 1U; ramp_steps <= active_steps; ++ramp_steps) {
      double weighted_sum = 0.0;
      double yaw_sum = 0.0;
      for (std::size_t step = 0U; step < active_steps; ++step) {
        const double shape = arrivalShape(step, active_steps, ramp_steps);
        weighted_sum += std::pow(static_cast<double>(drag),
                                 static_cast<double>(active_steps - 1U - step)) *
                        shape;
        yaw_sum += shape;
      }
      if (!(weighted_sum > std::numeric_limits<double>::epsilon()) ||
          !(yaw_sum > std::numeric_limits<double>::epsilon())) {
        continue;
      }
      const double translation_scale =
          -std::pow(static_cast<double>(drag), static_cast<double>(active_steps)) /
          (static_cast<double>(dt_s) * weighted_sum);
      const double yaw_scale = -1.0 / (static_cast<double>(dt_s) * yaw_sum);
      const Control amplitude{
          .ax = static_cast<float>(translation_scale * initial.vx),
          .ay = static_cast<float>(translation_scale * initial.vy),
          .az = static_cast<float>(translation_scale * initial.vz),
          .yaw_accel = static_cast<float>(yaw_scale * initial.yaw_rate),
      };

      std::vector<Control> controls;
      controls.reserve(active_steps + 1U);
      Control previous{};
      bool valid = true;
      for (std::size_t step = 0U; step < active_steps; ++step) {
        const float shape = arrivalShape(step, active_steps, ramp_steps);
        const Control control{
            .ax = amplitude.ax * shape,
            .ay = amplitude.ay * shape,
            .az = amplitude.az * shape,
            .yaw_accel = amplitude.yaw_accel * shape,
        };
        if (!controlWithinLimits(control, previous, dynamics)) {
          valid = false;
          break;
        }
        controls.push_back(control);
        previous = control;
      }
      if (!valid || !controlWithinLimits(Control{}, previous, dynamics)) {
        continue;
      }
      controls.push_back(Control{});
      return controls;
    }
  }
  return std::nullopt;
}

} // namespace

std::optional<FiniteHorizon> buildFiniteHorizon(
    const std::span<const State> planned_states,
    const std::span<const Control> planned_controls,
    const std::size_t nominal_prefix_control_count, const DynamicsConfig& dynamics,
    const Control previous_applied_control, const FiniteHorizonConfig& config) {
  if (planned_states.size() != planned_controls.size() + 1U ||
      nominal_prefix_control_count > planned_controls.size() ||
      !(dynamics.dt_s > 0.0F) || !(dynamics.maximum_control_jerk_mps3 > 0.0F) ||
      !(config.terminal_velocity_tolerance_mps > 0.0F) ||
      !(config.maximum_horizontal_deceleration_mps2 > 0.0F)) {
    throw std::invalid_argument{"invalid finite MPPI horizon input"};
  }

  FiniteHorizon horizon;
  horizon.nominal_prefix_control_count = nominal_prefix_control_count;
  horizon.states.assign(
      planned_states.begin(),
      planned_states.begin() +
          static_cast<std::ptrdiff_t>(nominal_prefix_control_count + 1U));
  horizon.controls.assign(planned_controls.begin(),
                          planned_controls.begin() + static_cast<std::ptrdiff_t>(
                                                         nominal_prefix_control_count));
  const std::size_t available_steps =
      planned_controls.size() - nominal_prefix_control_count;
  if (available_steps == 0U) {
    return finiteHorizonHasTerminalRestState(horizon,
                                             config.terminal_velocity_tolerance_mps)
               ? std::optional<FiniteHorizon>{std::move(horizon)}
               : std::nullopt;
  }

  Control previous = nominal_prefix_control_count > 0U
                         ? planned_controls[nominal_prefix_control_count - 1U]
                         : previous_applied_control;
  if (!appendControlRelease(horizon, previous, dynamics, available_steps)) {
    return std::nullopt;
  }

  const std::size_t remaining_steps = available_steps - horizon.arrival_control_count;
  DynamicsConfig arrival_dynamics = dynamics;
  arrival_dynamics.maximum_horizontal_acceleration_mps2 =
      std::min(dynamics.maximum_horizontal_acceleration_mps2,
               config.maximum_horizontal_deceleration_mps2);
  const std::optional<std::vector<Control>> arrival_controls =
      buildArrivalControls(horizon.states.back(), arrival_dynamics, remaining_steps,
                           config.terminal_velocity_tolerance_mps);
  if (!arrival_controls.has_value()) {
    return std::nullopt;
  }
  for (const Control& control : *arrival_controls) {
    appendControl(horizon, control, arrival_dynamics);
    ++horizon.arrival_control_count;
  }
  while (horizon.controls.size() < planned_controls.size()) {
    appendControl(horizon, Control{}, arrival_dynamics);
    ++horizon.arrival_control_count;
  }
  if (horizon.controls.size() != planned_controls.size()) {
    return std::nullopt;
  }

  State& terminal = horizon.states.back();
  if (std::hypot(std::hypot(terminal.vx, terminal.vy), terminal.vz) >
          config.terminal_velocity_tolerance_mps ||
      std::abs(terminal.yaw_rate) > config.terminal_velocity_tolerance_mps) {
    return std::nullopt;
  }
  terminal.vx = 0.0F;
  terminal.vy = 0.0F;
  terminal.vz = 0.0F;
  terminal.yaw_rate = 0.0F;
  return horizon;
}

std::int64_t finitePathControlIntervalNanoseconds(const float dt_s) noexcept {
  constexpr double kMicrosecondsPerSecond{1.0e6};
  constexpr std::int64_t kNanosecondsPerMicrosecond{1'000LL};
  if (!std::isfinite(dt_s) || !(dt_s > 0.0F)) {
    return 0;
  }
  const std::int64_t interval_us = static_cast<std::int64_t>(
      std::llround(static_cast<double>(dt_s) * kMicrosecondsPerSecond));
  if (interval_us <= 0 || interval_us > std::numeric_limits<std::int64_t>::max() /
                                            kNanosecondsPerMicrosecond) {
    return 0;
  }
  return interval_us * kNanosecondsPerMicrosecond;
}

bool finiteHorizonHasTerminalRestState(const FiniteHorizon& horizon,
                                       const float velocity_tolerance_mps) noexcept {
  if (horizon.states.size() != horizon.controls.size() + 1U ||
      horizon.states.size() < 2U || horizon.controls.empty() ||
      !(velocity_tolerance_mps >= 0.0F)) {
    return false;
  }
  const State& terminal = horizon.states.back();
  const Control& terminal_control = horizon.controls.back();
  return std::hypot(std::hypot(terminal.vx, terminal.vy), terminal.vz) <=
             velocity_tolerance_mps &&
         std::abs(terminal.yaw_rate) <= velocity_tolerance_mps &&
         translationalControlIsZero(terminal_control) &&
         std::abs(terminal_control.yaw_accel) <= 1.0e-6F;
}

} // namespace drone_city_nav::mppi
