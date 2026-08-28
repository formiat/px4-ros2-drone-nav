#include "drone_city_nav/sensor_braking_contract_3d.hpp"

#include "drone_city_nav/stopping_distance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace drone_city_nav {
namespace {

[[nodiscard]] JerkLimitedAxisStoppingConfig
brakingConfig(const SensorBrakingContract3D& contract,
              const StoppingCapability& stopping_capability) noexcept {
  return JerkLimitedAxisStoppingConfig{
      .guaranteed_deceleration_mps2 =
          std::min(stopping_capability.guaranteed_horizontal_deceleration_mps2,
                   stopping_capability.guaranteed_vertical_deceleration_mps2),
      .maximum_acceleration_mps2 = contract.maximum_forward_acceleration_mps2,
      .maximum_jerk_mps3 = contract.maximum_control_jerk_mps3,
      .reaction_latency_s = 0.0,
  };
}

} // namespace

bool sensorBrakingContract3DIsValid(
    const SensorBrakingContract3D& contract,
    const StoppingCapability& stopping_capability) noexcept {
  return stoppingCapabilityIsValid(stopping_capability) &&
         std::isfinite(contract.guaranteed_detection_range_m) &&
         contract.guaranteed_detection_range_m > 0.0 &&
         std::isfinite(contract.maximum_evidence_age_s) &&
         contract.maximum_evidence_age_s >= 0.0 &&
         std::isfinite(contract.physical_margin_m) &&
         contract.physical_margin_m >= 0.0 &&
         contract.physical_margin_m < contract.guaranteed_detection_range_m &&
         std::isfinite(contract.maximum_forward_acceleration_mps2) &&
         contract.maximum_forward_acceleration_mps2 > 0.0 &&
         std::isfinite(contract.maximum_control_jerk_mps3) &&
         contract.maximum_control_jerk_mps3 > 0.0 &&
         std::isfinite(stopping_capability.reaction_latency_s) &&
         jerkLimitedAxisStoppingConfigIsValid(
             brakingConfig(contract, stopping_capability));
}

bool SensorBrakingAssessment3D::accepted() const noexcept {
  return valid && reserve_m >= 0.0;
}

SensorBrakingAssessment3D
assessSensorBrakingContract3D(const SensorBrakingContract3D& contract,
                              const StoppingCapability& stopping_capability,
                              const double speed_mps) noexcept {
  SensorBrakingAssessment3D assessment{
      .speed_mps = speed_mps,
      .physical_margin_m = contract.physical_margin_m,
      .guaranteed_detection_range_m = contract.guaranteed_detection_range_m,
  };
  if (!sensorBrakingContract3DIsValid(contract, stopping_capability) ||
      !std::isfinite(speed_mps) || speed_mps < 0.0) {
    assessment.required_detection_range_m = std::numeric_limits<double>::infinity();
    assessment.reserve_m = -std::numeric_limits<double>::infinity();
    return assessment;
  }
  assessment.total_latency_s =
      contract.maximum_evidence_age_s + stopping_capability.reaction_latency_s;
  assessment.latency_distance_m = speed_mps * assessment.total_latency_s;
  assessment.stopping_distance_m = jerkLimitedAxisStoppingDistanceM(
      speed_mps, contract.maximum_forward_acceleration_mps2,
      brakingConfig(contract, stopping_capability));
  assessment.required_detection_range_m = assessment.latency_distance_m +
                                          assessment.stopping_distance_m +
                                          contract.physical_margin_m;
  assessment.reserve_m =
      contract.guaranteed_detection_range_m - assessment.required_detection_range_m;
  assessment.valid = std::isfinite(assessment.total_latency_s) &&
                     std::isfinite(assessment.latency_distance_m) &&
                     std::isfinite(assessment.stopping_distance_m) &&
                     std::isfinite(assessment.required_detection_range_m) &&
                     std::isfinite(assessment.reserve_m);
  return assessment;
}

double sensorBrakingMaximumSpeedMps(const SensorBrakingContract3D& contract,
                                    const StoppingCapability& stopping_capability,
                                    const double absolute_speed_limit_mps) noexcept {
  if (!sensorBrakingContract3DIsValid(contract, stopping_capability) ||
      !std::isfinite(absolute_speed_limit_mps) || absolute_speed_limit_mps <= 0.0) {
    return 0.0;
  }
  if (assessSensorBrakingContract3D(contract, stopping_capability,
                                    absolute_speed_limit_mps)
          .accepted()) {
    return absolute_speed_limit_mps;
  }
  double lower_mps{0.0};
  double upper_mps{absolute_speed_limit_mps};
  constexpr int kBisectionIterations{64};
  for (int iteration = 0; iteration < kBisectionIterations; ++iteration) {
    const double candidate_mps = std::midpoint(lower_mps, upper_mps);
    if (assessSensorBrakingContract3D(contract, stopping_capability, candidate_mps)
            .accepted()) {
      lower_mps = candidate_mps;
    } else {
      upper_mps = candidate_mps;
    }
  }
  return lower_mps;
}

} // namespace drone_city_nav
