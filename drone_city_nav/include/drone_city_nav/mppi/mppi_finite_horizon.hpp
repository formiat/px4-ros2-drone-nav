#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/stopping_capability.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav::mppi {

struct FiniteHorizonConfig {
  float terminal_velocity_tolerance_mps{1.0e-3F};
  StoppingCapability stopping_capability{
      .maximum_commanded_horizontal_deceleration_mps2 =
          std::numeric_limits<double>::max(),
      .guaranteed_horizontal_deceleration_mps2 = std::numeric_limits<double>::max(),
      .guaranteed_vertical_deceleration_mps2 = std::numeric_limits<double>::max(),
      .reaction_latency_s = 0.0,
  };
};

struct FiniteHorizon {
  std::vector<State> states;
  std::vector<Control> controls;
  std::size_t nominal_prefix_control_count{0U};
  std::size_t arrival_control_count{0U};
};

[[nodiscard]] FiniteHorizonConfig
makeFiniteHorizonConfig(const StoppingCapability& capability) noexcept;

[[nodiscard]] std::optional<FiniteHorizon> buildFiniteHorizon(
    std::span<const State> planned_states, std::span<const Control> planned_controls,
    std::size_t nominal_prefix_control_count, const DynamicsConfig& dynamics,
    Control previous_applied_control, const FiniteHorizonConfig& config = {});

[[nodiscard]] bool
finiteHorizonHasTerminalRestState(const FiniteHorizon& horizon,
                                  float velocity_tolerance_mps = 1.0e-3F) noexcept;

[[nodiscard]] std::int64_t finitePathControlIntervalNanoseconds(float dt_s) noexcept;

} // namespace drone_city_nav::mppi
