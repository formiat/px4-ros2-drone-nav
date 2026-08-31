#pragma once

#include "drone_city_nav/control_contracts_3d.hpp"
#include "drone_city_nav/finite_motion_horizon_3d.hpp"
#include "drone_city_nav/mppi/mppi_config.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace drone_city_nav::mppi {

using FiniteHorizonConfig = drone_city_nav::FiniteMotionHorizonConfig3D;
using FiniteHorizon = drone_city_nav::FiniteMotionHorizon3D;
using RouteConvergentFiniteHorizon =
    drone_city_nav::RouteConvergentFiniteMotionHorizon3D;

[[nodiscard]] inline FiniteHorizonConfig
makeFiniteHorizonConfig(const StoppingCapability& capability) noexcept {
  return makeFiniteMotionHorizonConfig3D(capability);
}

[[nodiscard]] inline std::optional<FiniteHorizon> buildFiniteHorizon(
    std::span<const State> planned_states, std::span<const Control> planned_controls,
    std::size_t nominal_prefix_control_count, const DynamicsConfig& dynamics,
    Control previous_applied_control, const FiniteHorizonConfig& config = {}) {
  return buildFiniteMotionHorizon3D(planned_states, planned_controls,
                                    nominal_prefix_control_count, dynamics,
                                    previous_applied_control, config);
}

// Builds an immediate jerk-limited stop from the exact current state. The
// control budget is explicit so the caller can certify the fallback inside the
// same finite execution window as its command horizon.
[[nodiscard]] inline std::optional<FiniteHorizon>
buildFiniteBrakingHorizon(const State& initial_state, std::size_t maximum_control_count,
                          const DynamicsConfig& dynamics,
                          Control previous_applied_control,
                          const FiniteHorizonConfig& config = {}) {
  return buildFiniteBrakingHorizon3D(initial_state, maximum_control_count, dynamics,
                                     previous_applied_control, config);
}

[[nodiscard]] inline RouteConvergentFiniteHorizon buildRouteConvergentFiniteHorizon(
    std::span<const State> planned_states, std::span<const Control> planned_controls,
    Control previous_applied_control, const DynamicsConfig& dynamics,
    std::span<const RouteSample3D> route, float initial_route_station_m,
    float terminal_cross_track_tolerance_m, std::size_t arrival_search_step_controls,
    const FiniteHorizonConfig& config = {}) {
  return buildRouteConvergentFiniteMotionHorizon3D(
      planned_states, planned_controls, previous_applied_control, dynamics, route,
      initial_route_station_m, terminal_cross_track_tolerance_m,
      arrival_search_step_controls, config);
}

[[nodiscard]] inline bool
finiteHorizonHasTerminalRestState(const FiniteHorizon& horizon,
                                  float velocity_tolerance_mps = 1.0e-3F) noexcept {
  return finiteMotionHorizonHasTerminalRestState3D(horizon, velocity_tolerance_mps);
}

[[nodiscard]] inline std::int64_t
finitePathControlIntervalNanoseconds(const float dt_s) noexcept {
  return finitePathControlIntervalNanoseconds3D(dt_s);
}

[[nodiscard]] inline std::size_t
finiteHorizonArrivalSearchStepControls(const float dt_s) noexcept {
  return finiteHorizonArrivalSearchStepControls3D(dt_s);
}

} // namespace drone_city_nav::mppi
