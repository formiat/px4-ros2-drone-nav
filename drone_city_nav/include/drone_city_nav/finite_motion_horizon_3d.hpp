#pragma once

#include "drone_city_nav/control_contracts_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

[[nodiscard]] FiniteMotionHorizonConfig3D
makeFiniteMotionHorizonConfig3D(const StoppingCapability& capability) noexcept;

[[nodiscard]] std::optional<FiniteMotionHorizon3D>
buildFiniteMotionHorizon3D(std::span<const MotionState3D> planned_states,
                           std::span<const MotionControl3D> planned_controls,
                           std::size_t nominal_prefix_control_count,
                           const MotionDynamicsConfig3D& dynamics,
                           MotionControl3D previous_applied_control,
                           const FiniteMotionHorizonConfig3D& config = {});

[[nodiscard]] std::optional<FiniteMotionHorizon3D> buildFiniteBrakingHorizon3D(
    const MotionState3D& initial_state, std::size_t maximum_control_count,
    const MotionDynamicsConfig3D& dynamics, MotionControl3D previous_applied_control,
    const FiniteMotionHorizonConfig3D& config = {});

// The stops available along a command horizon, earliest first: the stop that
// begins at once, then the stops that follow the horizon for one arrival
// search step more each, and last the horizon's own arrival. A plan carries
// the earliest of these the world admits as its braking tail. The stop that
// begins at once runs straight along the vehicle's velocity; where it runs
// into evidence the horizon itself was certified to turn away from, the next
// stop along the horizon is the earliest one the world allows, and the last
// is the command horizon, which ends at rest. Every entry follows the command
// horizon's own controls for its prefix and brakes with the same jerk-limited
// arrival the horizon was built with.
[[nodiscard]] std::vector<FiniteMotionHorizon3D>
buildFiniteBrakingHorizonsAlong3D(const FiniteMotionHorizon3D& command_horizon,
                                  const MotionDynamicsConfig3D& dynamics,
                                  MotionControl3D previous_applied_control,
                                  std::size_t arrival_search_step_controls,
                                  const FiniteMotionHorizonConfig3D& config = {});

[[nodiscard]] RouteConvergentFiniteMotionHorizon3D
buildRouteConvergentFiniteMotionHorizon3D(
    std::span<const MotionState3D> planned_states,
    std::span<const MotionControl3D> planned_controls,
    MotionControl3D previous_applied_control, const MotionDynamicsConfig3D& dynamics,
    std::span<const ControlRouteSample3D> route, float initial_route_station_m,
    float terminal_cross_track_tolerance_m, std::size_t arrival_search_step_controls,
    const FiniteMotionHorizonConfig3D& config = {});

[[nodiscard]] bool finiteMotionHorizonHasTerminalRestState3D(
    const FiniteMotionHorizon3D& horizon,
    float velocity_tolerance_mps = kTerminalRestVelocityToleranceMps) noexcept;

// True when every state from `first_state_index` onward already rests at the
// terminal state: within `position_tolerance_m` of it and slower than
// `velocity_tolerance_mps`. Such a tail commands no further motion, so an
// equivalent stationary owner may replace it before its lease ends.
[[nodiscard]] bool finiteMotionHorizonRestsFromState3D(
    const FiniteMotionHorizon3D& horizon, std::size_t first_state_index,
    double position_tolerance_m, double velocity_tolerance_mps) noexcept;

[[nodiscard]] std::int64_t finitePathControlIntervalNanoseconds3D(float dt_s) noexcept;

[[nodiscard]] std::size_t finiteHorizonArrivalSearchStepControls3D(float dt_s) noexcept;

} // namespace drone_city_nav
