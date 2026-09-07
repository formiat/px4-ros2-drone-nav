#pragma once

#include "drone_city_nav/control_contracts_3d.hpp"

#include <cmath>

// One admissible-control law, compiled for both the host sampler and the
// device rollouts. The engine limits a control on the GPU and the host
// validates and certifies the sequence it gets back against the same bound, so
// the two must agree exactly: a device step that leaves the acceleration disk
// during a direction change is a candidate the host rejects, and the vehicle
// simply holds its previous horizon instead.
#if defined(__CUDACC__)
#define DRONE_CITY_NAV_CONTROL_LIMIT_FN __host__ __device__ inline
#else
#define DRONE_CITY_NAV_CONTROL_LIMIT_FN inline
#endif

namespace drone_city_nav {

DRONE_CITY_NAV_CONTROL_LIMIT_FN void
clampControlHorizontalMagnitude3D(float& x, float& y, const float limit) noexcept {
  const float magnitude = std::hypot(x, y);
  if (magnitude > limit && magnitude > 0.0F) {
    const float scale = limit / magnitude;
    x *= scale;
    y *= scale;
  }
}

// Both endpoints are inside the horizontal acceleration disk, so moving along
// their connecting segment preserves that limit while the common scale also
// satisfies each axis' jerk bound. Independent axis clamping can leave the
// disk during a direction change.
DRONE_CITY_NAV_CONTROL_LIMIT_FN void
limitControlHorizontalJerk3D(float& x, float& y, const float previous_x,
                             const float previous_y,
                             const float maximum_delta) noexcept {
  const float delta_x = x - previous_x;
  const float delta_y = y - previous_y;
  float scale = 1.0F;
  if (std::fabs(delta_x) > maximum_delta) {
    scale = std::fmin(scale, maximum_delta / std::fabs(delta_x));
  }
  if (std::fabs(delta_y) > maximum_delta) {
    scale = std::fmin(scale, maximum_delta / std::fabs(delta_y));
  }
  x = previous_x + delta_x * scale;
  y = previous_y + delta_y * scale;
}

// The admissible control nearest to `control` given the one applied before it:
// inside the acceleration limits and within one interval of jerk of
// `previous`.
[[nodiscard]] DRONE_CITY_NAV_CONTROL_LIMIT_FN MotionControl3D limitMotionControlStep3D(
    MotionControl3D control, const MotionControl3D& previous,
    const MotionDynamicsConfig3D& dynamics, const float interval_s) noexcept {
  clampControlHorizontalMagnitude3D(control.ax, control.ay,
                                    dynamics.maximum_horizontal_acceleration_mps2);
  control.az =
      std::fmin(std::fmax(control.az, -dynamics.maximum_vertical_acceleration_mps2),
                dynamics.maximum_vertical_acceleration_mps2);
  control.yaw_accel =
      std::fmin(std::fmax(control.yaw_accel, -dynamics.maximum_yaw_acceleration_radps2),
                dynamics.maximum_yaw_acceleration_radps2);
  const float maximum_delta = dynamics.maximum_control_jerk_mps3 * interval_s;
  limitControlHorizontalJerk3D(control.ax, control.ay, previous.ax, previous.ay,
                               maximum_delta);
  control.az = std::fmin(std::fmax(control.az, previous.az - maximum_delta),
                         previous.az + maximum_delta);
  return control;
}

} // namespace drone_city_nav
