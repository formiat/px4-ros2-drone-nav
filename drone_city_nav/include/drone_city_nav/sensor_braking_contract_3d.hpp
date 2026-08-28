#pragma once

#include "drone_city_nav/stopping_capability.hpp"

namespace drone_city_nav {

struct SensorBrakingContract3D {
  double guaranteed_detection_range_m{30.0};
  double maximum_evidence_age_s{0.25};
  double physical_margin_m{3.0};
  double maximum_forward_acceleration_mps2{4.0};
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

[[nodiscard]] SensorBrakingAssessment3D
assessSensorBrakingContract3D(const SensorBrakingContract3D& contract,
                              const StoppingCapability& stopping_capability,
                              double speed_mps) noexcept;

[[nodiscard]] double
sensorBrakingMaximumSpeedMps(const SensorBrakingContract3D& contract,
                             const StoppingCapability& stopping_capability,
                             double absolute_speed_limit_mps) noexcept;

} // namespace drone_city_nav
