#pragma once

#include "drone_city_nav/timed_vehicle_state.hpp"
#include "drone_city_nav/types.hpp"

#include <optional>

namespace drone_city_nav {

// How close two vehicles came over the segment between their previous and
// their current state, so a separation is read from the swept motion and not
// from two samples that may straddle the closest approach.
struct SweptVehicleSeparation {
  double minimum_m{0.0};
  double current_m{0.0};
  double interpolation_fraction{1.0};
};

[[nodiscard]] SweptVehicleSeparation sweptVehicleSeparation(
    const TimedVehicleState& first, const TimedVehicleState& second,
    const std::optional<TimedVehicleState>& previous_first,
    const std::optional<TimedVehicleState>& previous_second) noexcept;

} // namespace drone_city_nav
