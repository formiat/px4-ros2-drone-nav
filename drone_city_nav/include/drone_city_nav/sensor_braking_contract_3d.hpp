#pragma once

#include "drone_city_nav/stopping_capability.hpp"
#include "drone_city_nav/types.hpp"

namespace drone_city_nav {

// What the lidar guarantees to see and what the airframe can do about it: the
// speed the vehicle may carry is the one it can stop from, inside the
// guaranteed detection range, after the evidence age and its own reaction.
struct SensorBrakingContract3D {
  double guaranteed_detection_range_m{30.0};
  double maximum_evidence_age_s{0.25};
  double physical_margin_m{3.0};
  // The acceleration the vehicle may still be applying along its motion when
  // the evidence arrives, per axis. Along a direction of motion the smaller
  // axis limit divided by that axis's share bounds the whole vector, so a
  // level flight is bounded by the horizontal limit alone and a pure climb by
  // the vertical one; a vector sum of both used to be assumed for every
  // direction, paired with the weakest deceleration of either axis, and
  // raising the horizontal acceleration then lowered the speed the contract
  // admitted in level flight.
  double maximum_horizontal_acceleration_mps2{4.0};
  double maximum_vertical_acceleration_mps2{4.0};
  double maximum_control_jerk_mps3{12.0};
};

[[nodiscard]] bool
sensorBrakingContract3DIsValid(const SensorBrakingContract3D& contract,
                               const StoppingCapability& stopping_capability) noexcept;

struct SensorBrakingAssessment3D {
  double speed_mps{0.0};
  double total_latency_s{0.0};
  double latency_distance_m{0.0};
  double stopping_distance_m{0.0};
  double physical_margin_m{0.0};
  double required_detection_range_m{0.0};
  double guaranteed_detection_range_m{0.0};
  double reserve_m{0.0};
  bool valid{false};

  [[nodiscard]] bool accepted() const noexcept;
};

// Assesses `speed_mps` along `direction`: the deceleration the vehicle is
// guaranteed along it and the acceleration it may still carry along it are
// the per-axis limits divided by the direction's axis shares. A zero direction
// assesses the worst direction, the one that needs the longest range: that is
// the cap the dynamics apply to every rollout, whichever way it points.
[[nodiscard]] SensorBrakingAssessment3D
assessSensorBrakingContract3D(const SensorBrakingContract3D& contract,
                              const StoppingCapability& stopping_capability,
                              double speed_mps,
                              const Vec3& direction = Vec3{}) noexcept;

// The fastest speed the contract admits along `direction`, no higher than
// `absolute_speed_limit_mps`; a zero direction gives the worst-direction cap.
[[nodiscard]] double
sensorBrakingMaximumSpeedMps(const SensorBrakingContract3D& contract,
                             const StoppingCapability& stopping_capability,
                             double absolute_speed_limit_mps,
                             const Vec3& direction = Vec3{}) noexcept;

} // namespace drone_city_nav
