#include "drone_city_nav/finite_motion_horizon_3d.hpp"

#include "drone_city_nav/control_route_projection_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool translationalControlIsZero(const MotionControl3D& control) noexcept {
  constexpr float kControlTolerance{1.0e-6F};
  return std::abs(control.ax) <= kControlTolerance &&
         std::abs(control.ay) <= kControlTolerance &&
         std::abs(control.az) <= kControlTolerance;
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

// One jerk-limited step from `from` toward `target`: the translational
// control moves along the straight segment between them by at most
// `translational_delta`, so every component changes by no more than the jerk
// limit allows and a control between two admissible ones stays admissible.
// The yaw channel has no jerk limit of its own and moves by at most
// `yaw_delta`.
[[nodiscard]] MotionControl3D stepToward(const MotionControl3D& from,
                                         const MotionControl3D& target,
                                         const float translational_delta,
                                         const float yaw_delta) noexcept {
  MotionControl3D next = target;
  const float dx = target.ax - from.ax;
  const float dy = target.ay - from.ay;
  const float dz = target.az - from.az;
  const float distance = std::hypot(std::hypot(dx, dy), dz);
  if (distance > translational_delta) {
    const float scale = translational_delta / distance;
    next.ax = from.ax + dx * scale;
    next.ay = from.ay + dy * scale;
    next.az = from.az + dz * scale;
  }
  const float dyaw = target.yaw_accel - from.yaw_accel;
  if (std::abs(dyaw) > yaw_delta) {
    next.yaw_accel = from.yaw_accel + std::copysign(yaw_delta, dyaw);
  }
  return next;
}

[[nodiscard]] bool controlsEqual(const MotionControl3D& left,
                                 const MotionControl3D& right) noexcept {
  return left.ax == right.ax && left.ay == right.ay && left.az == right.az &&
         left.yaw_accel == right.yaw_accel;
}

struct ArrivalProfileSimulation {
  std::vector<MotionControl3D> controls;
  MotionState3D terminal;
  // Full-amplitude step equivalents each channel commands over the profile:
  // the impulse the profile delivers is amplitude * dt * weight.
  double translational_weight{0.0};
  double yaw_weight{0.0};
};

// The arrival profile: a jerk-limited ramp from the control being applied now
// to the rest amplitude, a hold at that amplitude, and a jerk-limited ramp to
// zero. Starting from the applied control is what lets a profile rebuilt every
// tick carry the deceleration already reached forward; a profile that ramped
// in from zero every time would never get past the start of its own ramp
// under receding-horizon replanning.
[[nodiscard]] std::optional<ArrivalProfileSimulation>
simulateArrivalProfile(const MotionState3D& initial, const MotionControl3D& previous,
                       const MotionControl3D& amplitude, const std::size_t hold_steps,
                       const MotionDynamicsConfig3D& dynamics,
                       const std::size_t maximum_steps) {
  const float translational_delta = dynamics.maximum_control_jerk_mps3 * dynamics.dt_s;
  const float yaw_delta = std::max(dynamics.maximum_yaw_acceleration_radps2,
                                   std::numeric_limits<float>::min());
  ArrivalProfileSimulation simulation;
  MotionControl3D control = previous;
  const auto append = [&](const MotionControl3D& next) {
    if (simulation.controls.size() >= maximum_steps) {
      return false;
    }
    simulation.controls.push_back(next);
    control = next;
    return true;
  };
  while (!controlsEqual(control, amplitude)) {
    if (!append(stepToward(control, amplitude, translational_delta, yaw_delta))) {
      return std::nullopt;
    }
  }
  for (std::size_t step = 0U; step < hold_steps; ++step) {
    if (!append(amplitude)) {
      return std::nullopt;
    }
  }
  const MotionControl3D rest{};
  while (!controlsEqual(control, rest)) {
    if (!append(stepToward(control, rest, translational_delta, yaw_delta))) {
      return std::nullopt;
    }
  }
  if (simulation.controls.empty() && !append(rest)) {
    return std::nullopt;
  }

  const double amplitude_norm = std::hypot(
      std::hypot(static_cast<double>(amplitude.ax), static_cast<double>(amplitude.ay)),
      static_cast<double>(amplitude.az));
  const double yaw_amplitude = std::abs(static_cast<double>(amplitude.yaw_accel));
  simulation.terminal = initial;
  for (const MotionControl3D& step : simulation.controls) {
    simulation.terminal = integrateMotionState3D(simulation.terminal, step, dynamics);
    if (amplitude_norm > 0.0) {
      simulation.translational_weight +=
          (static_cast<double>(step.ax) * static_cast<double>(amplitude.ax) +
           static_cast<double>(step.ay) * static_cast<double>(amplitude.ay) +
           static_cast<double>(step.az) * static_cast<double>(amplitude.az)) /
          (amplitude_norm * amplitude_norm);
    }
    if (yaw_amplitude > 0.0) {
      simulation.yaw_weight += static_cast<double>(step.yaw_accel) /
                               static_cast<double>(amplitude.yaw_accel);
    }
  }
  // A profile without a hold still delivers about one step of its amplitude
  // over its ramps; the weight is what a correction divides by, so it is
  // never allowed to vanish.
  simulation.translational_weight = std::max(simulation.translational_weight, 1.0);
  simulation.yaw_weight = std::max(simulation.yaw_weight, 1.0);
  return simulation;
}

[[nodiscard]] MotionControl3D
clampArrivalAmplitude(MotionControl3D amplitude,
                      const MotionDynamicsConfig3D& dynamics) {
  const float horizontal = std::hypot(amplitude.ax, amplitude.ay);
  if (horizontal > dynamics.maximum_horizontal_acceleration_mps2) {
    const float scale = dynamics.maximum_horizontal_acceleration_mps2 / horizontal;
    amplitude.ax *= scale;
    amplitude.ay *= scale;
  }
  amplitude.az = std::clamp(amplitude.az, -dynamics.maximum_vertical_acceleration_mps2,
                            dynamics.maximum_vertical_acceleration_mps2);
  amplitude.yaw_accel =
      std::clamp(amplitude.yaw_accel, -dynamics.maximum_yaw_acceleration_radps2,
                 dynamics.maximum_yaw_acceleration_radps2);
  return amplitude;
}

// The amplitude a bang profile with `hold_steps` of hold and symmetric ramps
// at `ramp_delta` per step needs to deliver `impulse` over `dt_s` steps: the
// starting guess the simulated corrections refine.
[[nodiscard]] double bangAmplitudeGuess(const double impulse, const double hold_steps,
                                        const double ramp_delta,
                                        const double dt_s) noexcept {
  if (!(impulse > 0.0) || !(dt_s > 0.0)) {
    return 0.0;
  }
  if (!(ramp_delta > 0.0) || !std::isfinite(ramp_delta)) {
    return impulse / (dt_s * std::max(hold_steps, 1.0));
  }
  const double steps = impulse / dt_s;
  return 0.5 * ramp_delta *
         (std::sqrt(hold_steps * hold_steps + 4.0 * steps / ramp_delta) - hold_steps);
}

[[nodiscard]] std::optional<std::vector<MotionControl3D>>
buildArrivalControls(const MotionState3D& initial, const MotionControl3D& previous,
                     const MotionDynamicsConfig3D& dynamics,
                     const std::size_t maximum_steps,
                     const float velocity_tolerance_mps) {
  if (maximum_steps == 0U) {
    return std::nullopt;
  }
  const double dt_s = static_cast<double>(dynamics.dt_s);
  const double initial_speed = std::hypot(
      std::hypot(static_cast<double>(initial.vx), static_cast<double>(initial.vy)),
      static_cast<double>(initial.vz));
  const double translational_delta =
      static_cast<double>(dynamics.maximum_control_jerk_mps3) * dt_s;
  const double yaw_delta =
      static_cast<double>(dynamics.maximum_yaw_acceleration_radps2);
  // The profile is simulated under the integrator the horizon is checked
  // against, so the shedding above a speed cap and the drag are all in the
  // residual; each correction moves the amplitude by the residual the profile
  // still leaves, the shortest hold that rests within the tolerance wins.
  constexpr std::size_t kAmplitudeCorrections{24U};
  for (std::size_t hold_steps = 0U; hold_steps < maximum_steps; ++hold_steps) {
    const double translational_guess = bangAmplitudeGuess(
        initial_speed, static_cast<double>(hold_steps), translational_delta, dt_s);
    const double yaw_guess =
        bangAmplitudeGuess(std::abs(static_cast<double>(initial.yaw_rate)),
                           static_cast<double>(hold_steps), yaw_delta, dt_s);
    MotionControl3D amplitude = clampArrivalAmplitude(
        MotionControl3D{
            .ax = initial_speed > 0.0 ? static_cast<float>(-translational_guess *
                                                           initial.vx / initial_speed)
                                      : 0.0F,
            .ay = initial_speed > 0.0 ? static_cast<float>(-translational_guess *
                                                           initial.vy / initial_speed)
                                      : 0.0F,
            .az = initial_speed > 0.0 ? static_cast<float>(-translational_guess *
                                                           initial.vz / initial_speed)
                                      : 0.0F,
            .yaw_accel = static_cast<float>(
                -std::copysign(yaw_guess, static_cast<double>(initial.yaw_rate))),
        },
        dynamics);
    bool fits{false};
    for (std::size_t correction = 0U; correction < kAmplitudeCorrections;
         ++correction) {
      const std::optional<ArrivalProfileSimulation> simulation = simulateArrivalProfile(
          initial, previous, amplitude, hold_steps, dynamics, maximum_steps);
      if (!simulation.has_value()) {
        break;
      }
      fits = true;
      const MotionState3D& terminal = simulation->terminal;
      const double residual_speed =
          std::hypot(std::hypot(static_cast<double>(terminal.vx),
                                static_cast<double>(terminal.vy)),
                     static_cast<double>(terminal.vz));
      if (residual_speed <= static_cast<double>(velocity_tolerance_mps) &&
          std::abs(static_cast<double>(terminal.yaw_rate)) <=
              static_cast<double>(velocity_tolerance_mps)) {
        return simulation->controls;
      }
      const double translational_gain = 1.0 / (dt_s * simulation->translational_weight);
      const double yaw_gain = 1.0 / (dt_s * simulation->yaw_weight);
      const MotionControl3D corrected = clampArrivalAmplitude(
          MotionControl3D{
              .ax = static_cast<float>(static_cast<double>(amplitude.ax) -
                                       static_cast<double>(terminal.vx) *
                                           translational_gain),
              .ay = static_cast<float>(static_cast<double>(amplitude.ay) -
                                       static_cast<double>(terminal.vy) *
                                           translational_gain),
              .az = static_cast<float>(static_cast<double>(amplitude.az) -
                                       static_cast<double>(terminal.vz) *
                                           translational_gain),
              .yaw_accel =
                  static_cast<float>(static_cast<double>(amplitude.yaw_accel) -
                                     static_cast<double>(terminal.yaw_rate) * yaw_gain),
          },
          dynamics);
      if (controlsEqual(corrected, amplitude)) {
        // Saturated: this hold cannot deliver the impulse, a longer one must.
        break;
      }
      amplitude = corrected;
    }
    if (!fits) {
      // Even the ramps alone overrun the steps left; longer holds only add.
      return std::nullopt;
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

  MotionDynamicsConfig3D arrival_dynamics = dynamics;
  arrival_dynamics.maximum_horizontal_acceleration_mps2 =
      std::min(dynamics.maximum_horizontal_acceleration_mps2,
               static_cast<float>(
                   config.stopping_capability.guaranteed_horizontal_deceleration_mps2));
  arrival_dynamics.maximum_vertical_acceleration_mps2 =
      std::min(dynamics.maximum_vertical_acceleration_mps2,
               static_cast<float>(
                   config.stopping_capability.guaranteed_vertical_deceleration_mps2));
  // The arrival continues from the control the prefix ends on, or from the
  // applied control when there is no prefix: nothing is released to zero
  // first, the profile ramps straight from where the vehicle's command is.
  const std::optional<std::vector<MotionControl3D>> arrival_controls =
      buildArrivalControls(horizon.states.back(), previous, arrival_dynamics,
                           available_steps, config.terminal_velocity_tolerance_mps);
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
