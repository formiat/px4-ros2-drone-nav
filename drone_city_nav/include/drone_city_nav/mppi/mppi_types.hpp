#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace drone_city_nav::mppi {

enum class RiskTier : std::uint8_t {
  kPreferred = 0,
  kPlanning = 1,
  kCritical = 2,
  kCollision = 3,
};

struct State {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float vx{0.0F};
  float vy{0.0F};
  float vz{0.0F};
  float yaw{0.0F};
  float yaw_rate{0.0F};
};

struct MovingTargetReference {
  State state{};
  float capture_radius_m{5.0F};
  float vertical_deceleration_mps2{0.0F};
  float minimum_z_m{0.0F};
  float maximum_z_m{0.0F};
  bool bounded_vertical_motion{false};
};

enum class CooperativeManeuver : std::uint8_t {
  kKeep = 0,
  kClimb = 1,
  kDescend = 2,
  kLeft = 3,
  kRight = 4,
  kSlow = 5,
};

enum class NonCooperativeManeuver : std::uint8_t {
  kRouteCruise = 0,
  kAway = 1,
  kLeft = 2,
  kRight = 3,
  kClimb = 4,
  kDescend = 5,
  kBrake = 6,
  kBackward = 7,
};

[[nodiscard]] inline const char*
nonCooperativeManeuverName(const NonCooperativeManeuver maneuver) noexcept {
  switch (maneuver) {
    case NonCooperativeManeuver::kRouteCruise:
      return "route_cruise";
    case NonCooperativeManeuver::kAway:
      return "away";
    case NonCooperativeManeuver::kLeft:
      return "left";
    case NonCooperativeManeuver::kRight:
      return "right";
    case NonCooperativeManeuver::kClimb:
      return "climb";
    case NonCooperativeManeuver::kDescend:
      return "descend";
    case NonCooperativeManeuver::kBrake:
      return "brake";
    case NonCooperativeManeuver::kBackward:
      return "backward";
  }
  return "unknown";
}

struct DynamicAircraftSample {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

struct DynamicAircraftTrajectory {
  std::shared_ptr<const std::vector<DynamicAircraftSample>> samples;
  float footprint_radius_m{0.0F};
  std::size_t active_steps{0U};
};

struct DynamicAircraftCostPolicy {
  float strong_separation_m{5.0F};
  float anticipation_separation_m{5.0F};
  float strong_weight{80.0F};
  float anticipation_weight{0.0F};
  float time_to_collision_gain_s{0.0F};
  float maximum_time_to_collision_multiplier{1.0F};
};

struct CooperativeManeuverPreference {
  CooperativeManeuver maneuver{CooperativeManeuver::kKeep};
  float direction_x{0.0F};
  float direction_y{0.0F};
  float direction_z{0.0F};
  std::uint64_t generation{0U};
};

struct CooperativeSeparationAcquisition {
  CooperativeManeuverPreference preference{};
  float minimum_positive_progress_m{0.05F};
  float minimum_separation_gain_m{0.05F};
};

struct NonCooperativeSeparationAcquisition {
  float threat_direction_x{0.0F};
  float threat_direction_y{0.0F};
  float threat_direction_z{0.0F};
  float candidate_acceleration_fraction{0.95F};
  float candidate_duration_s{1.5F};
  std::uint64_t generation{0U};
};

#if defined(__CUDACC__)
#define DRONE_CITY_NAV_MPPI_HOST_DEVICE __host__ __device__
#else
#define DRONE_CITY_NAV_MPPI_HOST_DEVICE
#endif

struct DynamicAircraftCostContribution {
  float strong{0.0F};
  float anticipation{0.0F};
};

[[nodiscard]] DRONE_CITY_NAV_MPPI_HOST_DEVICE inline float
mppiMaximum(const float first, const float second) noexcept {
  return first > second ? first : second;
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_HOST_DEVICE inline float
mppiMinimum(const float first, const float second) noexcept {
  return first < second ? first : second;
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_HOST_DEVICE inline DynamicAircraftCostContribution
dynamicAircraftCostContribution(const float separation_m, const float elapsed_s,
                                const DynamicAircraftCostPolicy& policy) noexcept {
  const float bounded_elapsed_s = mppiMaximum(0.05F, elapsed_s);
  const float multiplier =
      mppiMinimum(policy.maximum_time_to_collision_multiplier,
                  1.0F + policy.time_to_collision_gain_s / bounded_elapsed_s);
  const float strong_shortfall_m =
      mppiMaximum(0.0F, policy.strong_separation_m - separation_m);
  const float anticipation_shortfall_m =
      mppiMaximum(0.0F, policy.anticipation_separation_m -
                            mppiMaximum(separation_m, policy.strong_separation_m));
  return DynamicAircraftCostContribution{
      .strong =
          multiplier * policy.strong_weight * strong_shortfall_m * strong_shortfall_m,
      .anticipation = multiplier * policy.anticipation_weight *
                      anticipation_shortfall_m * anticipation_shortfall_m,
  };
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_HOST_DEVICE inline float
clampMovingTargetAltitude(const float z_m, const float minimum_z_m,
                          const float maximum_z_m) noexcept {
  if (z_m < minimum_z_m) {
    return minimum_z_m;
  }
  return z_m > maximum_z_m ? maximum_z_m : z_m;
}

[[nodiscard]] DRONE_CITY_NAV_MPPI_HOST_DEVICE inline float
movingTargetAltitudeAt(const MovingTargetReference& target,
                       const float elapsed_s) noexcept {
  if (!target.bounded_vertical_motion || !(target.vertical_deceleration_mps2 > 0.0F)) {
    return target.state.z + target.state.vz * elapsed_s;
  }
  const float speed_mps = target.state.vz < 0.0F ? -target.state.vz : target.state.vz;
  const float stopping_time_s = speed_mps / target.vertical_deceleration_mps2;
  const float motion_time_s = elapsed_s < stopping_time_s ? elapsed_s : stopping_time_s;
  const float signed_deceleration_mps2 = target.state.vz < 0.0F
                                             ? -target.vertical_deceleration_mps2
                                             : target.vertical_deceleration_mps2;
  const float predicted_z_m =
      target.state.z + target.state.vz * motion_time_s -
      0.5F * signed_deceleration_mps2 * motion_time_s * motion_time_s;
  return clampMovingTargetAltitude(predicted_z_m, target.minimum_z_m,
                                   target.maximum_z_m);
}

#undef DRONE_CITY_NAV_MPPI_HOST_DEVICE

struct Control {
  float ax{0.0F};
  float ay{0.0F};
  float az{0.0F};
  float yaw_accel{0.0F};
};

struct CostBreakdown {
  float head_progress{0.0F};
  float progress{0.0F};
  float speed_tracking{0.0F};
  float guide_deviation{0.0F};
  float acceleration{0.0F};
  float jerk{0.0F};
  float yaw_change{0.0F};
  float control_effort{0.0F};
  float peer_separation{0.0F};
  float dynamic_aircraft_anticipation{0.0F};
  float dynamic_aircraft_survival{0.0F};
  float maneuver_preference{0.0F};
  float terminal{0.0F};
};

struct RolloutMetrics {
  State terminal_state{};
  CostBreakdown costs{};
  float soft_cost{0.0F};
  float critical_exposure_m{0.0F};
  float planning_exposure_m{0.0F};
  float minimum_clearance_m{0.0F};
  float minimum_target_separation_m{0.0F};
  float minimum_peer_separation_m{0.0F};
  float predicted_capture_time_s{-1.0F};
  RiskTier worst_tier{RiskTier::kPreferred};
  bool collision{false};
};

struct EsdfGrid {
  int width{0};
  int height{0};
  float resolution_m{0.0F};
  float origin_x_m{0.0F};
  float origin_y_m{0.0F};
  int depth{1};
  float origin_z_m{0.0F};
};

} // namespace drone_city_nav::mppi
