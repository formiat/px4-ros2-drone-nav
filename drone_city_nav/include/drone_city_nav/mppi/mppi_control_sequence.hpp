#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav::mppi {

[[nodiscard]] Control interpolateControl(const Control& first, const Control& second,
                                         float ratio) noexcept;

[[nodiscard]] std::vector<Control>
shiftControlSequence(std::span<const Control> controls, float dt_s, double elapsed_s);

void limitControlSequence(std::span<Control> controls, const DynamicsConfig& dynamics,
                          Control previous_applied_control,
                          float first_control_interval_s) noexcept;

[[nodiscard]] std::vector<Control>
buildGuideDirectedNominalSeed(const State& initial, const State& target,
                              std::span<const RouteSample3D> route,
                              float initial_route_station_m, float reference_speed_mps,
                              const DynamicsConfig& dynamics, std::size_t steps,
                              Control previous_applied_control);

[[nodiscard]] std::optional<float>
projectForwardRouteStation(std::span<const RouteSample3D> route, const State& state,
                           float minimum_station_m) noexcept;

[[nodiscard]] RiskTier maximumRequiredRiskTier(std::span<const RouteSample3D> route,
                                               float begin_station_m,
                                               float end_station_m) noexcept;

} // namespace drone_city_nav::mppi
