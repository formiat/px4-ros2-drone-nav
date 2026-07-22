#include "drone_city_nav/vertical_capture_profile.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

[[nodiscard]] double finitePositive(const double value,
                                    const double fallback) noexcept {
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

} // namespace

VerticalCaptureProfileStep advanceVerticalCaptureProfile(
    const VerticalCaptureProfileState& previous, const double actual_z_m,
    const double actual_vz_mps, const bool actual_vz_valid, const double target_z_m,
    const double dt_s, const VerticalCaptureProfileConfig& config) noexcept {
  VerticalCaptureProfileStep result{};
  if (!std::isfinite(actual_z_m) || !std::isfinite(target_z_m) ||
      !std::isfinite(dt_s) || dt_s <= 0.0) {
    return result;
  }

  const double max_climb = finitePositive(config.max_climb_speed_mps, 4.0);
  const double max_descent = finitePositive(config.max_descent_speed_mps, 4.0);
  const double max_accel = finitePositive(config.max_accel_mps2, 3.5);
  const double max_jerk = finitePositive(config.max_jerk_mps3, 10.0);
  VerticalCaptureProfileState state = previous;
  if (!state.initialized || !std::isfinite(state.commanded_z_m) ||
      !std::isfinite(state.commanded_vz_mps) ||
      !std::isfinite(state.commanded_accel_mps2)) {
    state.initialized = true;
    state.commanded_z_m = actual_z_m;
    state.commanded_vz_mps = actual_vz_valid && std::isfinite(actual_vz_mps)
                                 ? std::clamp(actual_vz_mps, -max_descent, max_climb)
                                 : 0.0;
    state.commanded_accel_mps2 = 0.0;
  }

  const double error_m = target_z_m - state.commanded_z_m;
  if (std::abs(error_m) <= 1.0e-4 && std::abs(state.commanded_vz_mps) <= 1.0e-3 &&
      std::abs(state.commanded_accel_mps2) <= max_jerk * dt_s) {
    state.commanded_z_m = target_z_m;
    state.commanded_vz_mps = 0.0;
    state.commanded_accel_mps2 = 0.0;
    result.valid = true;
    result.target_reached = true;
    result.state = state;
    return result;
  }

  const double speed_limit = error_m >= 0.0 ? max_climb : max_descent;
  const double braking_speed = std::sqrt(2.0 * max_accel * std::abs(error_m));
  const double desired_vz =
      std::abs(error_m) <= 1.0e-4
          ? 0.0
          : std::copysign(std::min(speed_limit, braking_speed), error_m);
  const double desired_accel =
      std::clamp((desired_vz - state.commanded_vz_mps) / dt_s, -max_accel, max_accel);
  const double accel_delta_limit = max_jerk * dt_s;
  state.commanded_accel_mps2 =
      std::clamp(desired_accel, state.commanded_accel_mps2 - accel_delta_limit,
                 state.commanded_accel_mps2 + accel_delta_limit);
  state.commanded_vz_mps += state.commanded_accel_mps2 * dt_s;
  state.commanded_vz_mps = std::clamp(state.commanded_vz_mps, -max_descent, max_climb);
  const double next_z_m = state.commanded_z_m + state.commanded_vz_mps * dt_s;
  if ((target_z_m - state.commanded_z_m) * (target_z_m - next_z_m) <= 0.0) {
    state.commanded_z_m = target_z_m;
  } else {
    state.commanded_z_m = next_z_m;
  }

  result.valid = true;
  result.state = state;
  return result;
}

} // namespace drone_city_nav
