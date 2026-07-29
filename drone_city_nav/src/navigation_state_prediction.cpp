#include "drone_city_nav/navigation_state_prediction.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {

NavigationStatePredictionResult
predictNavigationState(const mppi::State& state, const double age_s,
                       const double maximum_prediction_age_s) noexcept {
  NavigationStatePredictionResult result{.state = state};
  if (!std::isfinite(age_s) || age_s < 0.0 ||
      !std::isfinite(maximum_prediction_age_s) || maximum_prediction_age_s < 0.0 ||
      age_s > maximum_prediction_age_s) {
    return result;
  }
  result.valid = true;
  result.prediction_age_s = age_s;
  if (age_s <= 0.0) {
    return result;
  }
  const float dt = static_cast<float>(age_s);
  result.state.x += result.state.vx * dt;
  result.state.y += result.state.vy * dt;
  result.state.z += result.state.vz * dt;
  result.state.yaw = std::remainder(result.state.yaw + result.state.yaw_rate * dt,
                                    static_cast<float>(2.0 * std::acos(-1.0)));
  result.predicted = true;
  return result;
}

} // namespace drone_city_nav
