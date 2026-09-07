#include "drone_city_nav/sensor_braking_contract_3d.hpp"

#include "drone_city_nav/stopping_distance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

namespace drone_city_nav {
namespace {

constexpr double kShareEpsilon{1.0e-9};

// The horizontal and vertical shares of a unit direction; both zero when the
// direction is unspecified.
struct MotionShares3D {
  double horizontal{0.0};
  double vertical{0.0};

  [[nodiscard]] bool specified() const noexcept {
    return horizontal > kShareEpsilon || vertical > kShareEpsilon;
  }
};

[[nodiscard]] MotionShares3D motionShares(const Vec3& direction) noexcept {
  const double norm = std::hypot(std::hypot(direction.x, direction.y), direction.z);
  if (!std::isfinite(norm) || !(norm > kShareEpsilon)) {
    return {};
  }
  return MotionShares3D{.horizontal = std::hypot(direction.x, direction.y) / norm,
                        .vertical = std::abs(direction.z) / norm};
}

// The acceleration an axis pair admits along a direction: the axis that runs
// out first bounds the whole vector, so the along-direction limit is the
// smaller of each axis limit divided by its share.
[[nodiscard]] double alongDirection(const double horizontal_limit,
                                    const double vertical_limit,
                                    const MotionShares3D& shares) noexcept {
  double limit = std::numeric_limits<double>::infinity();
  if (shares.horizontal > kShareEpsilon) {
    limit = std::min(limit, horizontal_limit / shares.horizontal);
  }
  if (shares.vertical > kShareEpsilon) {
    limit = std::min(limit, vertical_limit / shares.vertical);
  }
  return limit;
}

[[nodiscard]] JerkLimitedAxisStoppingConfig
brakingConfig(const SensorBrakingContract3D& contract,
              const StoppingCapability& stopping_capability,
              const MotionShares3D& shares) noexcept {
  return JerkLimitedAxisStoppingConfig{
      .guaranteed_deceleration_mps2 = alongDirection(
          stopping_capability.guaranteed_horizontal_deceleration_mps2,
          stopping_capability.guaranteed_vertical_deceleration_mps2, shares),
      .maximum_acceleration_mps2 =
          alongDirection(contract.maximum_horizontal_acceleration_mps2,
                         contract.maximum_vertical_acceleration_mps2, shares),
      .maximum_jerk_mps3 = contract.maximum_control_jerk_mps3,
      .reaction_latency_s = 0.0,
  };
}

// Directions the worst case is searched over: the vertical share from level
// flight to a pure climb. Within each interval the along-direction limits are
// monotone, so the interval's worst deceleration and worst acceleration are
// read at its ends; the true worst direction is bounded by one of them.
constexpr std::size_t kWorstCaseSampleCount{33U};

[[nodiscard]] MotionShares3D sampledShares(const std::size_t sample) noexcept {
  const double vertical =
      static_cast<double>(sample) / static_cast<double>(kWorstCaseSampleCount - 1U);
  return MotionShares3D{.horizontal =
                            std::sqrt(std::max(0.0, 1.0 - vertical * vertical)),
                        .vertical = vertical};
}

template<typename Visitor>
void forEachWorstCaseBrakingConfig(const SensorBrakingContract3D& contract,
                                   const StoppingCapability& stopping_capability,
                                   Visitor&& visitor) {
  JerkLimitedAxisStoppingConfig previous =
      brakingConfig(contract, stopping_capability, sampledShares(0U));
  for (std::size_t sample = 1U; sample < kWorstCaseSampleCount; ++sample) {
    const JerkLimitedAxisStoppingConfig next =
        brakingConfig(contract, stopping_capability, sampledShares(sample));
    visitor(JerkLimitedAxisStoppingConfig{
        .guaranteed_deceleration_mps2 = std::min(previous.guaranteed_deceleration_mps2,
                                                 next.guaranteed_deceleration_mps2),
        .maximum_acceleration_mps2 = std::max(previous.maximum_acceleration_mps2,
                                              next.maximum_acceleration_mps2),
        .maximum_jerk_mps3 = contract.maximum_control_jerk_mps3,
        .reaction_latency_s = 0.0,
    });
    previous = next;
  }
}

[[nodiscard]] double stoppingDistanceM(const double speed_mps,
                                       const JerkLimitedAxisStoppingConfig& config) {
  return jerkLimitedAxisStoppingDistanceM(speed_mps, config.maximum_acceleration_mps2,
                                          config);
}

// The stopping distance at `speed_mps` for the direction, or the longest over
// all directions when none is specified.
[[nodiscard]] double
directionalStoppingDistanceM(const SensorBrakingContract3D& contract,
                             const StoppingCapability& stopping_capability,
                             const double speed_mps, const MotionShares3D& shares) {
  if (shares.specified()) {
    return stoppingDistanceM(speed_mps,
                             brakingConfig(contract, stopping_capability, shares));
  }
  double worst_m{0.0};
  forEachWorstCaseBrakingConfig(
      contract, stopping_capability, [&](const JerkLimitedAxisStoppingConfig& config) {
        worst_m = std::max(worst_m, stoppingDistanceM(speed_mps, config));
      });
  return worst_m;
}

} // namespace

bool sensorBrakingContract3DIsValid(
    const SensorBrakingContract3D& contract,
    const StoppingCapability& stopping_capability) noexcept {
  if (!stoppingCapabilityIsValid(stopping_capability) ||
      !std::isfinite(contract.guaranteed_detection_range_m) ||
      !(contract.guaranteed_detection_range_m > 0.0) ||
      !std::isfinite(contract.maximum_evidence_age_s) ||
      contract.maximum_evidence_age_s < 0.0 ||
      !std::isfinite(contract.physical_margin_m) || contract.physical_margin_m < 0.0 ||
      contract.physical_margin_m >= contract.guaranteed_detection_range_m ||
      !std::isfinite(contract.maximum_horizontal_acceleration_mps2) ||
      !(contract.maximum_horizontal_acceleration_mps2 > 0.0) ||
      !std::isfinite(contract.maximum_vertical_acceleration_mps2) ||
      !(contract.maximum_vertical_acceleration_mps2 > 0.0) ||
      !std::isfinite(contract.maximum_control_jerk_mps3) ||
      !(contract.maximum_control_jerk_mps3 > 0.0) ||
      !std::isfinite(stopping_capability.reaction_latency_s)) {
    return false;
  }
  bool valid{true};
  forEachWorstCaseBrakingConfig(
      contract, stopping_capability, [&](const JerkLimitedAxisStoppingConfig& config) {
        valid = valid && jerkLimitedAxisStoppingConfigIsValid(config);
      });
  return valid;
}

bool SensorBrakingAssessment3D::accepted() const noexcept {
  return valid && reserve_m >= 0.0;
}

SensorBrakingAssessment3D
assessSensorBrakingContract3D(const SensorBrakingContract3D& contract,
                              const StoppingCapability& stopping_capability,
                              const double speed_mps, const Vec3& direction) noexcept {
  SensorBrakingAssessment3D assessment{
      .speed_mps = speed_mps,
      .physical_margin_m = contract.physical_margin_m,
      .guaranteed_detection_range_m = contract.guaranteed_detection_range_m,
  };
  if (!sensorBrakingContract3DIsValid(contract, stopping_capability) ||
      !std::isfinite(speed_mps) || speed_mps < 0.0 || !std::isfinite(direction.x) ||
      !std::isfinite(direction.y) || !std::isfinite(direction.z)) {
    assessment.required_detection_range_m = std::numeric_limits<double>::infinity();
    assessment.reserve_m = -std::numeric_limits<double>::infinity();
    return assessment;
  }
  assessment.total_latency_s =
      contract.maximum_evidence_age_s + stopping_capability.reaction_latency_s;
  assessment.latency_distance_m = speed_mps * assessment.total_latency_s;
  assessment.stopping_distance_m = directionalStoppingDistanceM(
      contract, stopping_capability, speed_mps, motionShares(direction));
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
                                    const double absolute_speed_limit_mps,
                                    const Vec3& direction) noexcept {
  if (!sensorBrakingContract3DIsValid(contract, stopping_capability) ||
      !std::isfinite(absolute_speed_limit_mps) || absolute_speed_limit_mps <= 0.0) {
    return 0.0;
  }
  if (assessSensorBrakingContract3D(contract, stopping_capability,
                                    absolute_speed_limit_mps, direction)
          .accepted()) {
    return absolute_speed_limit_mps;
  }
  double lower_mps{0.0};
  double upper_mps{absolute_speed_limit_mps};
  constexpr int kBisectionIterations{64};
  for (int iteration = 0; iteration < kBisectionIterations; ++iteration) {
    const double candidate_mps = std::midpoint(lower_mps, upper_mps);
    if (assessSensorBrakingContract3D(contract, stopping_capability, candidate_mps,
                                      direction)
            .accepted()) {
      lower_mps = candidate_mps;
    } else {
      upper_mps = candidate_mps;
    }
  }
  return lower_mps;
}

} // namespace drone_city_nav
