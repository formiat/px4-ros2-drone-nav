#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"

namespace drone_city_nav {

struct NavigationStatePredictionResult {
  mppi::State state{};
  double prediction_age_s{0.0};
  bool predicted{false};
  bool valid{false};
};

[[nodiscard]] NavigationStatePredictionResult
predictNavigationState(const mppi::State& state, double age_s,
                       double maximum_prediction_age_s) noexcept;

} // namespace drone_city_nav
