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
  // Where the sensors look. A lidar that sees the whole sphere guarantees its
  // range along every direction, and the defaults say so. A forward camera
  // guarantees `guaranteed_detection_range_m` only up to this elevation above
  // and below the horizon; sensors looking straight up and down guarantee
  // `vertical_detection_range_m` inside a cone of this half-angle about the
  // vertical; a direction inside neither is observed by nothing, and the
  // vehicle moves along it no faster than `unobserved_speed_mps`, the speed a
  // contact is left at. The elevation is the motion's own, so the law holds
  // whichever way the vehicle is tilted while it flies level.
  double forward_vertical_half_angle_rad{1.5707963267948966};
  // The forward sensor's half-angle about the heading. The contract does not
  // know the heading; the speed policy, which does, holds a motion outside it
  // to `unobserved_speed_mps`.
  double forward_horizontal_half_angle_rad{3.141592653589793};
  double vertical_detection_range_m{0.0};
  double vertical_cone_half_angle_rad{0.0};
  double unobserved_speed_mps{1.0};
  // The physical margin of a vertical approach, which the body's height and
  // the vertical estimate set, where `physical_margin_m` is sized by the
  // body's horizontal envelope.
  double vertical_physical_margin_m{0.0};
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

// Whether `direction` is a motion no sensor sees at heading `yaw_rad`: outside
// the cone of the sensors that look up and down, and outside the forward
// sensor's horizontal field. A wall stands across every elevation of such a
// motion, so facing it is what lets the forward sensor see it. Never true for
// a sensor that sees all around, or for a zero direction.
[[nodiscard]] bool sensorBrakingMotionUnfaced3D(const SensorBrakingContract3D& contract,
                                                const Vec3& direction,
                                                double yaw_rad) noexcept;

// The fastest speed admitted along an unfaced motion: memory is its only
// witness, so the contract is read with the range memory has observed along
// it, capped by the sensor's own, and admits nothing where that is no more
// than the physical margin.
[[nodiscard]] double
sensorBrakingMemorySpeedMps(const SensorBrakingContract3D& contract,
                            const StoppingCapability& stopping_capability,
                            double absolute_speed_limit_mps, const Vec3& direction,
                            double observed_range_m) noexcept;

} // namespace drone_city_nav
