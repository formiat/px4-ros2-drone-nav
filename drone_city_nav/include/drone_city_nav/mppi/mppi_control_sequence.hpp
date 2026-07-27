#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace drone_city_nav::mppi {

[[nodiscard]] Control interpolateControl(const Control& first, const Control& second,
                                         float ratio) noexcept;

[[nodiscard]] std::vector<Control>
shiftControlSequence(std::span<const Control> controls, float dt_s, double elapsed_s);

[[nodiscard]] std::vector<Control>
buildGuideDirectedNominalSeed(const State& initial, const State& target,
                              const DynamicsConfig& dynamics, std::size_t steps,
                              std::uint64_t generation);

} // namespace drone_city_nav::mppi
