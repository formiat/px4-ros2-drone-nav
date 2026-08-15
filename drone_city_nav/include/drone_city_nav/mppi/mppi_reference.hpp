#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"

#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav::mppi {

[[nodiscard]] State integrateReference(State state, Control control,
                                       const DynamicsConfig& config) noexcept;

[[nodiscard]] Control equivalentControlFromMeasuredAcceleration(
    const State& state, float measured_ax_mps2, float measured_ay_mps2,
    float measured_az_mps2, const DynamicsConfig& config) noexcept;

struct ReferenceSimulationTrace {
  std::vector<State> horizon;
};

struct MppiProgressDiagnostics {
  float head_progress_m{0.0F};
  float terminal_progress_m{0.0F};
};

[[nodiscard]] Control
resolveCooperativePreferredAcceleration(const CooperativeManeuverPreference& preference,
                                        const DynamicsConfig& dynamics,
                                        const CooperativeConfig& cooperative) noexcept;

[[nodiscard]] MppiProgressDiagnostics
resolveUnroutedProgressDiagnostics(const RolloutMetrics& metrics,
                                   bool moving_target_enabled,
                                   float fixed_target_head_progress_m,
                                   float fixed_target_terminal_progress_m) noexcept;

[[nodiscard]] RolloutMetrics simulateReference(
    const State& initial_state, std::span<const Control> nominal_controls,
    std::span<const Control> noise_controls, const DynamicsConfig& dynamics,
    const RiskConfig& risk, const CostConfig& costs, const EsdfGrid& grid,
    std::span<const float> esdf, float target_x_m, float target_y_m,
    bool early_exit_on_collision, Control previous_applied_control = {},
    float reference_speed_mps = -1.0F, const FootprintConfig& footprint = {},
    std::optional<MovingTargetReference> moving_target = std::nullopt,
    ReferenceSimulationTrace* trace = nullptr,
    std::span<const DynamicAircraftTrajectory> dynamic_aircraft = {},
    std::optional<CooperativeManeuverPreference> cooperative_maneuver = std::nullopt,
    const CooperativeConfig& cooperative = {},
    std::optional<DynamicAircraftCostPolicy> dynamic_aircraft_cost_policy =
        std::nullopt,
    AltitudeEnvelopeConfig altitude_envelope = {});

} // namespace drone_city_nav::mppi
