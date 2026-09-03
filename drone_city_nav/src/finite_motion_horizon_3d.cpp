#include "drone_city_nav/finite_motion_horizon_3d.hpp"

#include "drone_city_nav/control_route_projection_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace drone_city_nav {
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

[[nodiscard]] bool translationalControlIsZero(const MotionControl3D& control) noexcept {
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

[[nodiscard]] bool
controlWithinLimits(const MotionControl3D& control, const MotionControl3D& previous,
                    const MotionDynamicsConfig3D& dynamics) noexcept {
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

void appendControl(FiniteMotionHorizon3D& horizon, const MotionControl3D& control,
                   const MotionDynamicsConfig3D& dynamics) {
  horizon.controls.push_back(control);
  horizon.states.push_back(
      integrateMotionState3D(horizon.states.back(), control, dynamics));
}

[[nodiscard]] bool appendControlRelease(FiniteMotionHorizon3D& horizon,
                                        MotionControl3D& previous,
                                        const MotionDynamicsConfig3D& dynamics,
                                        const std::size_t maximum_steps) {
  const float maximum_delta = dynamics.maximum_control_jerk_mps3 * dynamics.dt_s;
  while (!translationalControlIsZero(previous)) {
    if (horizon.arrival_control_count >= maximum_steps) {
      return false;
    }
    const MotionControl3D control{
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

[[nodiscard]] std::optional<std::vector<MotionControl3D>> buildArrivalControls(
    const MotionState3D& initial, const MotionDynamicsConfig3D& dynamics,
    const std::size_t maximum_steps, const float velocity_tolerance_mps) {
  const float dt_s = dynamics.dt_s;
  const float drag = std::max(0.0F, 1.0F - dynamics.linear_drag_1ps * dt_s);
  const float initial_speed =
      std::hypot(std::hypot(initial.vx, initial.vy), initial.vz);
  const bool translation_required = initial_speed > velocity_tolerance_mps;
  const bool yaw_required = std::abs(initial.yaw_rate) > velocity_tolerance_mps;
  if (!translation_required && !yaw_required) {
    return std::vector<MotionControl3D>{MotionControl3D{}};
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
      const MotionControl3D amplitude{
          .ax = static_cast<float>(translation_scale * initial.vx),
          .ay = static_cast<float>(translation_scale * initial.vy),
          .az = static_cast<float>(translation_scale * initial.vz),
          .yaw_accel = static_cast<float>(yaw_scale * initial.yaw_rate),
      };

      std::vector<MotionControl3D> controls;
      controls.reserve(active_steps + 1U);
      MotionControl3D previous{};
      bool valid = true;
      for (std::size_t step = 0U; step < active_steps; ++step) {
        const float shape = arrivalShape(step, active_steps, ramp_steps);
        const MotionControl3D control{
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
      if (!valid || !controlWithinLimits(MotionControl3D{}, previous, dynamics)) {
        continue;
      }
      controls.push_back(MotionControl3D{});
      return controls;
    }
  }
  return std::nullopt;
}

} // namespace

FiniteMotionHorizonConfig3D
makeFiniteMotionHorizonConfig3D(const StoppingCapability& capability) noexcept {
  return FiniteMotionHorizonConfig3D{.stopping_capability = capability};
}

std::optional<FiniteMotionHorizon3D>
buildFiniteMotionHorizon3D(const std::span<const MotionState3D> planned_states,
                           const std::span<const MotionControl3D> planned_controls,
                           const std::size_t nominal_prefix_control_count,
                           const MotionDynamicsConfig3D& dynamics,
                           const MotionControl3D previous_applied_control,
                           const FiniteMotionHorizonConfig3D& config) {
  if (planned_states.size() != planned_controls.size() + 1U ||
      nominal_prefix_control_count > planned_controls.size() ||
      !(dynamics.dt_s > 0.0F) || !(dynamics.maximum_control_jerk_mps3 > 0.0F) ||
      !(config.terminal_velocity_tolerance_mps > 0.0F) ||
      !stoppingCapabilityIsValid(config.stopping_capability)) {
    throw std::invalid_argument{"invalid finite motion horizon input"};
  }

  FiniteMotionHorizon3D horizon;
  horizon.nominal_prefix_control_count = nominal_prefix_control_count;
  horizon.states.reserve(planned_states.size());
  horizon.controls.reserve(planned_controls.size());
  horizon.states.push_back(planned_states.front());
  MotionControl3D previous = previous_applied_control;
  for (std::size_t index = 0U; index < nominal_prefix_control_count; ++index) {
    const MotionControl3D& control = planned_controls[index];
    if (!controlWithinLimits(control, previous, dynamics)) {
      return std::nullopt;
    }
    appendControl(horizon, control, dynamics);
    previous = control;
  }
  const std::size_t available_steps =
      planned_controls.size() - nominal_prefix_control_count;
  if (available_steps == 0U) {
    return finiteMotionHorizonHasTerminalRestState3D(
               horizon, config.terminal_velocity_tolerance_mps)
               ? std::optional<FiniteMotionHorizon3D>{std::move(horizon)}
               : std::nullopt;
  }

  if (!appendControlRelease(horizon, previous, dynamics, available_steps)) {
    return std::nullopt;
  }

  const std::size_t remaining_steps = available_steps - horizon.arrival_control_count;
  MotionDynamicsConfig3D arrival_dynamics = dynamics;
  arrival_dynamics.maximum_horizontal_acceleration_mps2 =
      std::min(dynamics.maximum_horizontal_acceleration_mps2,
               static_cast<float>(
                   config.stopping_capability.guaranteed_horizontal_deceleration_mps2));
  arrival_dynamics.maximum_vertical_acceleration_mps2 =
      std::min(dynamics.maximum_vertical_acceleration_mps2,
               static_cast<float>(
                   config.stopping_capability.guaranteed_vertical_deceleration_mps2));
  const std::optional<std::vector<MotionControl3D>> arrival_controls =
      buildArrivalControls(horizon.states.back(), arrival_dynamics, remaining_steps,
                           config.terminal_velocity_tolerance_mps);
  if (!arrival_controls.has_value()) {
    return std::nullopt;
  }
  for (const MotionControl3D& control : *arrival_controls) {
    appendControl(horizon, control, arrival_dynamics);
    ++horizon.arrival_control_count;
  }
  while (horizon.controls.size() < planned_controls.size()) {
    appendControl(horizon, MotionControl3D{}, arrival_dynamics);
    ++horizon.arrival_control_count;
  }
  if (horizon.controls.size() != planned_controls.size()) {
    return std::nullopt;
  }

  MotionState3D& terminal = horizon.states.back();
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

std::optional<FiniteMotionHorizon3D>
buildFiniteBrakingHorizon3D(const MotionState3D& initial_state,
                            const std::size_t maximum_control_count,
                            const MotionDynamicsConfig3D& dynamics,
                            const MotionControl3D previous_applied_control,
                            const FiniteMotionHorizonConfig3D& config) {
  if (maximum_control_count == 0U) {
    return std::nullopt;
  }
  // buildFiniteMotionHorizon3D deliberately ignores the unpreserved source states and
  // controls. Supplying a zero-length nominal prefix therefore exercises the
  // same jerk-limited arrival generator used by normal finite horizons while
  // making the safety artifact independent of the planned command sequence.
  std::vector<MotionState3D> capacity_states(maximum_control_count + 1U, initial_state);
  std::vector<MotionControl3D> capacity_controls(maximum_control_count);
  return buildFiniteMotionHorizon3D(capacity_states, capacity_controls, 0U, dynamics,
                                    previous_applied_control, config);
}

RouteConvergentFiniteMotionHorizon3D buildRouteConvergentFiniteMotionHorizon3D(
    const std::span<const MotionState3D> planned_states,
    const std::span<const MotionControl3D> planned_controls,
    const MotionControl3D previous_applied_control,
    const MotionDynamicsConfig3D& dynamics,
    const std::span<const ControlRouteSample3D> route,
    const float initial_route_station_m, const float terminal_cross_track_tolerance_m,
    const std::size_t arrival_search_step_controls,
    const FiniteMotionHorizonConfig3D& config) {
  RouteConvergentFiniteMotionHorizon3D result;
  if (planned_states.size() != planned_controls.size() + 1U ||
      planned_controls.empty() || route.size() < 2U ||
      !std::isfinite(initial_route_station_m) ||
      !std::isfinite(terminal_cross_track_tolerance_m) ||
      !(terminal_cross_track_tolerance_m > 0.0F) ||
      arrival_search_step_controls == 0U) {
    return result;
  }

  std::size_t preserved_prefix_control_count = planned_controls.size();
  while (true) {
    ++result.arrival_shaping_attempts;
    std::optional<FiniteMotionHorizon3D> candidate = buildFiniteMotionHorizon3D(
        planned_states, planned_controls, preserved_prefix_control_count, dynamics,
        previous_applied_control, config);
    if (candidate.has_value()) {
      const ControlRouteProjection3D terminal_projection = projectOntoControlRoute3D(
          candidate->states.back(), route, initial_route_station_m);
      if (terminal_projection.valid &&
          (result.closest_terminal_cross_track_m < 0.0F ||
           terminal_projection.distance_m < result.closest_terminal_cross_track_m)) {
        result.closest_terminal_cross_track_m = terminal_projection.distance_m;
      }
      if (terminal_projection.valid &&
          terminal_projection.distance_m <= terminal_cross_track_tolerance_m) {
        result.nominal_prefix_control_count = candidate->nominal_prefix_control_count;
        result.horizon = std::move(candidate);
        return result;
      }
    }
    if (preserved_prefix_control_count == 0U) {
      return result;
    }
    preserved_prefix_control_count =
        preserved_prefix_control_count > arrival_search_step_controls
            ? preserved_prefix_control_count - arrival_search_step_controls
            : 0U;
  }
}

std::int64_t finitePathControlIntervalNanoseconds3D(const float dt_s) noexcept {
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

std::size_t finiteHorizonArrivalSearchStepControls3D(const float dt_s) noexcept {
  constexpr float kArrivalSearchIntervalS{0.5F};
  if (!std::isfinite(dt_s) || !(dt_s > 0.0F)) {
    return 0U;
  }
  return std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(kArrivalSearchIntervalS / dt_s)));
}

bool finiteMotionHorizonHasTerminalRestState3D(
    const FiniteMotionHorizon3D& horizon, const float velocity_tolerance_mps) noexcept {
  if (horizon.states.size() != horizon.controls.size() + 1U ||
      horizon.states.size() < 2U || horizon.controls.empty() ||
      !(velocity_tolerance_mps >= 0.0F)) {
    return false;
  }
  const MotionState3D& terminal = horizon.states.back();
  const MotionControl3D& terminal_control = horizon.controls.back();
  return std::hypot(std::hypot(terminal.vx, terminal.vy), terminal.vz) <=
             velocity_tolerance_mps &&
         std::abs(terminal.yaw_rate) <= velocity_tolerance_mps &&
         translationalControlIsZero(terminal_control) &&
         std::abs(terminal_control.yaw_accel) <= 1.0e-6F;
}

bool finiteMotionHorizonRestsFromState3D(const FiniteMotionHorizon3D& horizon,
                                         const std::size_t first_state_index,
                                         const double position_tolerance_m,
                                         const double velocity_tolerance_mps) noexcept {
  if (horizon.states.empty() || first_state_index >= horizon.states.size() ||
      !(position_tolerance_m >= 0.0) || !(velocity_tolerance_mps >= 0.0)) {
    return false;
  }
  const MotionState3D& terminal = horizon.states.back();
  for (std::size_t index = first_state_index; index < horizon.states.size(); ++index) {
    const MotionState3D& state = horizon.states[index];
    const double displacement_m = std::hypot(
        std::hypot(static_cast<double>(state.x) - static_cast<double>(terminal.x),
                   static_cast<double>(state.y) - static_cast<double>(terminal.y)),
        static_cast<double>(state.z) - static_cast<double>(terminal.z));
    const double speed_mps = std::hypot(std::hypot(state.vx, state.vy), state.vz);
    if (!(displacement_m <= position_tolerance_m) ||
        !(speed_mps <= velocity_tolerance_mps) ||
        !(std::abs(static_cast<double>(state.yaw_rate)) <= velocity_tolerance_mps)) {
      return false;
    }
  }
  return true;
}

} // namespace drone_city_nav
