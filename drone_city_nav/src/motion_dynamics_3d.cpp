#include "drone_city_nav/motion_dynamics_3d.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace drone_city_nav {
namespace {

[[nodiscard]] float clampMagnitude(const float value, const float limit) noexcept {
  return std::clamp(value, -limit, limit);
}

void clampHorizontal(float& x, float& y, const float limit) noexcept {
  const float magnitude = std::hypot(x, y);
  if (magnitude > limit && magnitude > 0.0F) {
    const float scale = limit / magnitude;
    x *= scale;
    y *= scale;
  }
}

void clampTranslational(float& x, float& y, float& z, const float limit) noexcept {
  const float magnitude = std::hypot(std::hypot(x, y), z);
  if (magnitude > limit && magnitude > 0.0F) {
    const float scale = limit / magnitude;
    x *= scale;
    y *= scale;
    z *= scale;
  }
}

} // namespace

MotionState3D integrateMotionState3D(MotionState3D state, MotionControl3D control,
                                     const MotionDynamicsConfig3D& config) noexcept {
  clampHorizontal(control.ax, control.ay, config.maximum_horizontal_acceleration_mps2);
  control.az = clampMagnitude(control.az, config.maximum_vertical_acceleration_mps2);
  control.yaw_accel =
      clampMagnitude(control.yaw_accel, config.maximum_yaw_acceleration_radps2);

  const float drag = std::max(0.0F, 1.0F - config.linear_drag_1ps * config.dt_s);
  const float inherited_horizontal_speed_mps = std::hypot(state.vx, state.vy);
  const float inherited_vertical_speed_mps = std::abs(state.vz);
  const float inherited_translational_speed_mps =
      std::hypot(inherited_horizontal_speed_mps, inherited_vertical_speed_mps);
  const float inherited_yaw_rate_radps = std::abs(state.yaw_rate);
  state.vx = state.vx * drag + control.ax * config.dt_s;
  state.vy = state.vy * drag + control.ay * config.dt_s;
  state.vz = state.vz * drag + control.az * config.dt_s;
  clampHorizontal(
      state.vx, state.vy,
      std::max(config.maximum_horizontal_speed_mps, inherited_horizontal_speed_mps));
  state.vz = clampMagnitude(state.vz, std::max(config.maximum_vertical_speed_mps,
                                               inherited_vertical_speed_mps));
  clampTranslational(state.vx, state.vy, state.vz,
                     std::max(config.maximum_translational_speed_mps,
                              inherited_translational_speed_mps));

  state.yaw_rate =
      clampMagnitude(state.yaw_rate + control.yaw_accel * config.dt_s,
                     std::max(config.maximum_yaw_rate_radps, inherited_yaw_rate_radps));
  state.x += state.vx * config.dt_s;
  state.y += state.vy * config.dt_s;
  state.z += state.vz * config.dt_s;
  state.yaw = std::remainder(state.yaw + state.yaw_rate * config.dt_s,
                             2.0F * std::numbers::pi_v<float>);
  return state;
}

} // namespace drone_city_nav
