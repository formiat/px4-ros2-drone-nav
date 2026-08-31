#pragma once

#include "drone_city_nav/control_contracts_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

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
    float velocity_tolerance_mps = 1.0e-3F) noexcept;

[[nodiscard]] std::int64_t finitePathControlIntervalNanoseconds3D(float dt_s) noexcept;

[[nodiscard]] std::size_t finiteHorizonArrivalSearchStepControls3D(float dt_s) noexcept;

} // namespace drone_city_nav
