#pragma once

namespace drone_city_nav {

// Controller-neutral translational and yaw state shared by prediction,
// trajectory validation, execution, and controller adapters.
struct MotionState3D {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float vx{0.0F};
  float vy{0.0F};
  float vz{0.0F};
  float yaw{0.0F};
  float yaw_rate{0.0F};
};

// Controller-neutral acceleration command paired with MotionState3D.
struct MotionControl3D {
  float ax{0.0F};
  float ay{0.0F};
  float az{0.0F};
  float yaw_accel{0.0F};
};

} // namespace drone_city_nav
