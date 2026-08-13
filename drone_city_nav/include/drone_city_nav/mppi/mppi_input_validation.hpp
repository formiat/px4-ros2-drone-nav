#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"

#include <cstddef>

namespace drone_city_nav::mppi {

void validateMppiTickInput(const MppiTickInput& input, std::size_t expected_steps,
                           std::size_t maximum_cooperative_peers);

} // namespace drone_city_nav::mppi
