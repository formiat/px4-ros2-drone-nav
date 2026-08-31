#pragma once

#include "drone_city_nav/dynamic_handoff_validator_3d.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"

#include <span>

namespace drone_city_nav::mppi {

using StaticRouteHandoffStatus = drone_city_nav::DynamicHandoffStatus3D;
using StaticRouteHandoffResult = drone_city_nav::DynamicHandoffResult3D;

[[nodiscard]] StaticRouteHandoffResult validateStaticRouteHandoff(
    const State& current_state, Control previous_applied_control,
    std::span<const RouteSample3D> candidate_route, float reference_speed_mps,
    float maximum_cross_track_m, float terminal_cross_track_tolerance_m,
    const BenchmarkConfig& config, const EsdfGrid& grid, std::span<const float> esdf_m);

[[nodiscard]] const char*
staticRouteHandoffStatusName(StaticRouteHandoffStatus status) noexcept;

// Adapts the MPPI backend to the controller-neutral activation port. The
// captured configuration is immutable for the lifetime of the validator.
[[nodiscard]] DynamicHandoffValidator3D
makeMppiDynamicHandoffValidator3D(BenchmarkConfig config);

} // namespace drone_city_nav::mppi
