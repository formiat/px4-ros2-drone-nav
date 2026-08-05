#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"

#include <span>

namespace drone_city_nav::mppi {

[[nodiscard]] State integrateReference(State state, Control control,
                                       const DynamicsConfig& config) noexcept;

[[nodiscard]] RolloutMetrics simulateReference(
    const State& initial_state, std::span<const Control> nominal_controls,
    std::span<const Control> noise_controls, const DynamicsConfig& dynamics,
    const RiskConfig& risk, const CostConfig& costs, const EsdfGrid& grid,
    std::span<const float> esdf, float target_x_m, float target_y_m,
    bool early_exit_on_collision, Control previous_applied_control = {},
    float reference_speed_mps = -1.0F, const FootprintConfig& footprint = {});

} // namespace drone_city_nav::mppi
