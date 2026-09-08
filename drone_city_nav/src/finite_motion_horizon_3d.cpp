#include "drone_city_nav/finite_motion_horizon_3d.hpp"

#include "drone_city_nav/control_route_projection_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool translationalControlIsZero(const MotionControl3D& control) noexcept {
  return std::abs(control.ax) <= kTerminalRestControlToleranceMps2 &&
         std::abs(control.ay) <= kTerminalRestControlToleranceMps2 &&
         std::abs(control.az) <= kTerminalRestControlToleranceMps2;
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
  simulation.terminal = initial;
  for (const MotionControl3D& step : simulation.controls) {
    simulation.terminal = integrateMotionState3D(simulation.terminal, step, dynamics);
  }
  return simulation;
}

// The acceleration the arrival profile is allowed to command. It is the
// vehicle's guaranteed braking capability, which is weaker than the modelled
// maximum: a stop the profile promises must be one the airframe delivers even
// at the low end of its authority. It bounds the amplitude only. Integration,
// the jerk limit and the admissible-control envelope stay the canonical
// dynamics the horizon is later validated, certified and executed under, so
// the profile the builder simulates is the profile every downstream stage
// reconstructs from the same controls.
struct ArrivalAmplitudeLimits3D {
  float maximum_horizontal_acceleration_mps2{0.0F};
  float maximum_vertical_acceleration_mps2{0.0F};
  float maximum_yaw_acceleration_radps2{0.0F};
};

[[nodiscard]] ArrivalAmplitudeLimits3D
arrivalAmplitudeLimits(const MotionDynamicsConfig3D& dynamics,
                       const StoppingCapability& capability) noexcept {
  return ArrivalAmplitudeLimits3D{
      .maximum_horizontal_acceleration_mps2 = std::min(
          dynamics.maximum_horizontal_acceleration_mps2,
          static_cast<float>(capability.guaranteed_horizontal_deceleration_mps2)),
      .maximum_vertical_acceleration_mps2 = std::min(
          dynamics.maximum_vertical_acceleration_mps2,
          static_cast<float>(capability.guaranteed_vertical_deceleration_mps2)),
      .maximum_yaw_acceleration_radps2 = dynamics.maximum_yaw_acceleration_radps2,
  };
}

[[nodiscard]] MotionControl3D
clampArrivalAmplitude(MotionControl3D amplitude,
                      const ArrivalAmplitudeLimits3D& limits) {
  const float horizontal = std::hypot(amplitude.ax, amplitude.ay);
  if (horizontal > limits.maximum_horizontal_acceleration_mps2) {
    const float scale = limits.maximum_horizontal_acceleration_mps2 / horizontal;
    amplitude.ax *= scale;
    amplitude.ay *= scale;
  }
  amplitude.az = std::clamp(amplitude.az, -limits.maximum_vertical_acceleration_mps2,
                            limits.maximum_vertical_acceleration_mps2);
  amplitude.yaw_accel =
      std::clamp(amplitude.yaw_accel, -limits.maximum_yaw_acceleration_radps2,
                 limits.maximum_yaw_acceleration_radps2);
  return amplitude;
}

// The amplitude a bang profile with `hold_steps` of hold and symmetric ramps
// at `ramp_delta` per step needs to deliver `impulse` over `dt_s` steps: the
// starting guess the solver refines.
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

// The velocity a profile leaves, as the vector the solver drives to zero:
// the three translational components and the yaw rate.
using ArrivalResidual = std::array<double, 4U>;

[[nodiscard]] ArrivalResidual arrivalResidual(const MotionState3D& terminal) noexcept {
  return {static_cast<double>(terminal.vx), static_cast<double>(terminal.vy),
          static_cast<double>(terminal.vz), static_cast<double>(terminal.yaw_rate)};
}

[[nodiscard]] bool arrivalResidualWithinTolerance(const ArrivalResidual& residual,
                                                  const double tolerance) noexcept {
  return std::hypot(std::hypot(residual[0], residual[1]), residual[2]) <= tolerance &&
         std::abs(residual[3]) <= tolerance;
}

[[nodiscard]] double arrivalResidualNorm(const ArrivalResidual& residual) noexcept {
  return std::sqrt(residual[0] * residual[0] + residual[1] * residual[1] +
                   residual[2] * residual[2] + residual[3] * residual[3]);
}

[[nodiscard]] MotionControl3D amplitudeComponentOffset(MotionControl3D amplitude,
                                                       const std::size_t component,
                                                       const double offset) noexcept {
  switch (component) {
    case 0U:
      amplitude.ax = static_cast<float>(static_cast<double>(amplitude.ax) + offset);
      break;
    case 1U:
      amplitude.ay = static_cast<float>(static_cast<double>(amplitude.ay) + offset);
      break;
    case 2U:
      amplitude.az = static_cast<float>(static_cast<double>(amplitude.az) + offset);
      break;
    default:
      amplitude.yaw_accel =
          static_cast<float>(static_cast<double>(amplitude.yaw_accel) + offset);
      break;
  }
  return amplitude;
}

// Solves J * step = -residual by Gaussian elimination with partial pivoting.
[[nodiscard]] std::optional<ArrivalResidual>
solveNewtonStep(std::array<ArrivalResidual, 4U> jacobian_columns,
                ArrivalResidual residual) noexcept {
  // jacobian_columns.at(j).at(i) is d residual_i / d amplitude_j.
  constexpr std::size_t kDimension{4U};
  constexpr double kPivotTolerance{1.0e-9};
  std::array<std::array<double, kDimension>, kDimension> matrix{};
  for (std::size_t row = 0U; row < kDimension; ++row) {
    for (std::size_t column = 0U; column < kDimension; ++column) {
      matrix.at(row).at(column) = jacobian_columns.at(column).at(row);
    }
    residual.at(row) = -residual.at(row);
  }
  for (std::size_t pivot = 0U; pivot < kDimension; ++pivot) {
    std::size_t best = pivot;
    for (std::size_t row = pivot + 1U; row < kDimension; ++row) {
      if (std::abs(matrix.at(row).at(pivot)) > std::abs(matrix.at(best).at(pivot))) {
        best = row;
      }
    }
    if (std::abs(matrix.at(best).at(pivot)) <= kPivotTolerance) {
      return std::nullopt;
    }
    std::swap(matrix.at(pivot), matrix.at(best));
    std::swap(residual.at(pivot), residual.at(best));
    for (std::size_t row = pivot + 1U; row < kDimension; ++row) {
      const double factor = matrix.at(row).at(pivot) / matrix.at(pivot).at(pivot);
      for (std::size_t column = pivot; column < kDimension; ++column) {
        matrix.at(row).at(column) -= factor * matrix.at(pivot).at(column);
      }
      residual.at(row) -= factor * residual.at(pivot);
    }
  }
  ArrivalResidual step{};
  for (std::size_t index = kDimension; index-- > 0U;) {
    double sum = residual.at(index);
    for (std::size_t column = index + 1U; column < kDimension; ++column) {
      sum -= matrix.at(index).at(column) * step.at(column);
    }
    step.at(index) = sum / matrix.at(index).at(index);
  }
  return step;
}

struct ArrivalAmplitudeSolution {
  std::optional<std::vector<MotionControl3D>> controls;
  // The amplitude the solver ended on and the velocity it still leaves, for
  // the caller to size a longer hold from when the limits were reached.
  MotionControl3D amplitude;
  ArrivalResidual residual{};
  bool fits{false};
};

// Finds the amplitude that rests the vehicle for one hold length: Newton on
// the simulated terminal velocity, the Jacobian by forward differences, steps
// damped until the residual shrinks and clamped to the acceleration limits.
// The profile is simulated under the integrator the horizon is checked
// against, so the drag and the shedding above a speed cap are all in the
// residual.
[[nodiscard]] ArrivalAmplitudeSolution
solveArrivalAmplitude(const MotionState3D& initial, const MotionControl3D& previous,
                      MotionControl3D amplitude, const std::size_t hold_steps,
                      const MotionDynamicsConfig3D& dynamics,
                      const ArrivalAmplitudeLimits3D& limits,
                      const std::size_t maximum_steps, const double tolerance) {
  constexpr std::size_t kNewtonIterations{24U};
  constexpr std::size_t kDampingHalvings{6U};
  constexpr double kDifferenceStep{0.05};
  ArrivalAmplitudeSolution solution;
  solution.amplitude = amplitude;
  std::optional<ArrivalProfileSimulation> simulation = simulateArrivalProfile(
      initial, previous, amplitude, hold_steps, dynamics, maximum_steps);
  if (!simulation.has_value()) {
    return solution;
  }
  solution.fits = true;
  ArrivalResidual residual = arrivalResidual(simulation->terminal);
  for (std::size_t iteration = 0U; iteration < kNewtonIterations; ++iteration) {
    solution.amplitude = amplitude;
    solution.residual = residual;
    if (arrivalResidualWithinTolerance(residual, tolerance)) {
      solution.controls = std::move(simulation->controls);
      return solution;
    }
    std::array<ArrivalResidual, 4U> jacobian_columns{};
    bool jacobian_valid{true};
    for (std::size_t component = 0U; component < 4U && jacobian_valid; ++component) {
      double step = kDifferenceStep;
      std::optional<ArrivalProfileSimulation> offset = simulateArrivalProfile(
          initial, previous, amplitudeComponentOffset(amplitude, component, step),
          hold_steps, dynamics, maximum_steps);
      if (!offset.has_value()) {
        step = -kDifferenceStep;
        offset = simulateArrivalProfile(
            initial, previous, amplitudeComponentOffset(amplitude, component, step),
            hold_steps, dynamics, maximum_steps);
      }
      if (!offset.has_value()) {
        jacobian_valid = false;
        break;
      }
      const ArrivalResidual offset_residual = arrivalResidual(offset->terminal);
      for (std::size_t row = 0U; row < 4U; ++row) {
        jacobian_columns.at(component).at(row) =
            (offset_residual.at(row) - residual.at(row)) / step;
      }
    }
    if (!jacobian_valid) {
      return solution;
    }
    const std::optional<ArrivalResidual> newton_step =
        solveNewtonStep(jacobian_columns, residual);
    if (!newton_step.has_value()) {
      return solution;
    }
    const double residual_norm = arrivalResidualNorm(residual);
    bool accepted{false};
    double damping = 1.0;
    for (std::size_t halving = 0U; halving <= kDampingHalvings; ++halving) {
      const MotionControl3D candidate = clampArrivalAmplitude(
          MotionControl3D{
              .ax = static_cast<float>(static_cast<double>(amplitude.ax) +
                                       damping * (*newton_step)[0]),
              .ay = static_cast<float>(static_cast<double>(amplitude.ay) +
                                       damping * (*newton_step)[1]),
              .az = static_cast<float>(static_cast<double>(amplitude.az) +
                                       damping * (*newton_step)[2]),
              .yaw_accel = static_cast<float>(static_cast<double>(amplitude.yaw_accel) +
                                              damping * (*newton_step)[3]),
          },
          limits);
      if (controlsEqual(candidate, amplitude)) {
        // Clamped back onto the amplitude it already has: the limits are
        // reached, only a longer hold can deliver the rest of the impulse.
        return solution;
      }
      std::optional<ArrivalProfileSimulation> candidate_simulation =
          simulateArrivalProfile(initial, previous, candidate, hold_steps, dynamics,
                                 maximum_steps);
      if (candidate_simulation.has_value()) {
        const ArrivalResidual candidate_residual =
            arrivalResidual(candidate_simulation->terminal);
        if (arrivalResidualNorm(candidate_residual) < residual_norm) {
          amplitude = candidate;
          simulation = std::move(candidate_simulation);
          residual = candidate_residual;
          accepted = true;
          break;
        }
      }
      damping *= 0.5;
    }
    if (!accepted) {
      return solution;
    }
  }
  solution.amplitude = amplitude;
  solution.residual = residual;
  if (arrivalResidualWithinTolerance(residual, tolerance)) {
    solution.controls = std::move(simulation->controls);
  }
  return solution;
}

[[nodiscard]] std::optional<std::vector<MotionControl3D>> buildArrivalControls(
    const MotionState3D& initial, const MotionControl3D& previous,
    const MotionDynamicsConfig3D& dynamics, const ArrivalAmplitudeLimits3D& limits,
    const std::size_t maximum_steps, const float velocity_tolerance_mps) {
  if (maximum_steps == 0U) {
    return std::nullopt;
  }
  const double dt_s = static_cast<double>(dynamics.dt_s);
  const double tolerance = static_cast<double>(velocity_tolerance_mps);
  const double initial_speed = std::hypot(
      std::hypot(static_cast<double>(initial.vx), static_cast<double>(initial.vy)),
      static_cast<double>(initial.vz));
  const double translational_delta =
      static_cast<double>(dynamics.maximum_control_jerk_mps3) * dt_s;
  const double yaw_delta =
      static_cast<double>(dynamics.maximum_yaw_acceleration_radps2);
  const double amplitude_floor =
      std::max({static_cast<double>(limits.maximum_horizontal_acceleration_mps2),
                static_cast<double>(limits.maximum_vertical_acceleration_mps2),
                static_cast<double>(limits.maximum_yaw_acceleration_radps2), 1.0e-3});
  // The shortest hold that rests within the tolerance wins. A hold whose
  // amplitude hits the limits is lengthened by the steps the velocity it
  // still leaves needs at those limits, so the search does not crawl one step
  // at a time from a fast state.
  std::size_t hold_steps{0U};
  while (hold_steps < maximum_steps) {
    const double translational_guess = bangAmplitudeGuess(
        initial_speed, static_cast<double>(hold_steps), translational_delta, dt_s);
    const double yaw_guess =
        bangAmplitudeGuess(std::abs(static_cast<double>(initial.yaw_rate)),
                           static_cast<double>(hold_steps), yaw_delta, dt_s);
    const MotionControl3D guess = clampArrivalAmplitude(
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
        limits);
    ArrivalAmplitudeSolution solution =
        solveArrivalAmplitude(initial, previous, guess, hold_steps, dynamics, limits,
                              maximum_steps, tolerance);
    if (solution.controls.has_value()) {
      return std::move(solution.controls);
    }
    if (!solution.fits) {
      // Even the ramps alone overrun the steps left; longer holds only add.
      return std::nullopt;
    }
    const double remaining = arrivalResidualNorm(solution.residual);
    const std::size_t extra_steps = static_cast<std::size_t>(
        std::clamp(std::ceil(remaining / (dt_s * amplitude_floor)), 1.0,
                   static_cast<double>(maximum_steps)));
    hold_steps += extra_steps;
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

  const ArrivalAmplitudeLimits3D arrival_limits =
      arrivalAmplitudeLimits(dynamics, config.stopping_capability);
  // The arrival continues from the control the prefix ends on, or from the
  // applied control when there is no prefix: nothing is released to zero
  // first, the profile ramps straight from where the vehicle's command is.
  const std::optional<std::vector<MotionControl3D>> arrival_controls =
      buildArrivalControls(horizon.states.back(), previous, dynamics, arrival_limits,
                           available_steps, config.terminal_velocity_tolerance_mps);
  if (!arrival_controls.has_value()) {
    return std::nullopt;
  }
  for (const MotionControl3D& control : *arrival_controls) {
    appendControl(horizon, control, dynamics);
    ++horizon.arrival_control_count;
  }
  while (horizon.controls.size() < planned_controls.size()) {
    appendControl(horizon, MotionControl3D{}, dynamics);
    ++horizon.arrival_control_count;
  }
  if (horizon.controls.size() != planned_controls.size()) {
    return std::nullopt;
  }

  // The horizon rests on the states the integrator produced. A terminal
  // velocity forced to exactly zero would be a state no control in the
  // sequence explains: the publisher reconstructs each point's acceleration
  // from the velocity step, and would read the fabricated jump as a terminal
  // acceleration the wire contract rejects. The residual the profile leaves
  // is what `finiteMotionHorizonHasTerminalRestState3D` and the wire contract
  // both call rest.
  return finiteMotionHorizonHasTerminalRestState3D(
             horizon, config.terminal_velocity_tolerance_mps)
             ? std::optional<FiniteMotionHorizon3D>{std::move(horizon)}
             : std::nullopt;
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

std::vector<FiniteMotionHorizon3D>
buildFiniteBrakingHorizonsAlong3D(const FiniteMotionHorizon3D& command_horizon,
                                  const MotionDynamicsConfig3D& dynamics,
                                  const MotionControl3D previous_applied_control,
                                  const std::size_t arrival_search_step_controls,
                                  const FiniteMotionHorizonConfig3D& config) {
  std::vector<FiniteMotionHorizon3D> tails;
  if (command_horizon.states.size() != command_horizon.controls.size() + 1U ||
      command_horizon.controls.empty() ||
      command_horizon.nominal_prefix_control_count > command_horizon.controls.size()) {
    return tails;
  }
  const std::size_t step = std::max<std::size_t>(1U, arrival_search_step_controls);
  const std::size_t last_prefix = command_horizon.nominal_prefix_control_count;
  for (std::size_t prefix = 0U;; prefix = std::min(prefix + step, last_prefix)) {
    std::optional<FiniteMotionHorizon3D> tail =
        buildFiniteMotionHorizon3D(command_horizon.states, command_horizon.controls,
                                   prefix, dynamics, previous_applied_control, config);
    if (tail.has_value()) {
      tails.push_back(std::move(*tail));
    }
    if (prefix >= last_prefix) {
      break;
    }
  }
  return tails;
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
         std::abs(terminal_control.yaw_accel) <= kTerminalRestControlToleranceMps2;
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
