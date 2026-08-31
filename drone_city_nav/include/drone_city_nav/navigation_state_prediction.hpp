#pragma once

#include "drone_city_nav/motion_state_3d.hpp"

namespace drone_city_nav {

struct NavigationStatePredictionResult {
  MotionState3D state{};
  double prediction_age_s{0.0};
  bool predicted{false};
  bool valid{false};
};

[[nodiscard]] NavigationStatePredictionResult
predictNavigationState(const MotionState3D& state, double age_s,
                       double maximum_prediction_age_s) noexcept;

} // namespace drone_city_nav
