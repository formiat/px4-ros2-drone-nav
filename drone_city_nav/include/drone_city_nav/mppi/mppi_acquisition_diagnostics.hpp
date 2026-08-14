#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi/mppi_separation_acquisition_coordinator.hpp"

#include <cstddef>

namespace drone_city_nav::mppi {

void populateSeparationAcquisitionResult(
    MppiTickResult& result, const SeparationAcquisitionCoordinatorResult& acquisition,
    bool cooperative_candidates_injected, std::size_t dynamic_aircraft_count) noexcept;

} // namespace drone_city_nav::mppi
