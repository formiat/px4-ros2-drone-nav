#include "drone_city_nav/mppi/mppi_input_validation.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <stdexcept>

namespace drone_city_nav::mppi {

void validateMppiTickInput(const MppiTickInput& input, const std::size_t expected_steps,
                           const std::size_t maximum_dynamic_aircraft) {
  if (input.moving_target.has_value()) {
    const MovingTargetReference& target = *input.moving_target;
    const bool invalid_vertical =
        target.bounded_vertical_motion &&
        (!std::isfinite(target.vertical_deceleration_mps2) ||
         !(target.vertical_deceleration_mps2 > 0.0F) ||
         !std::isfinite(target.minimum_z_m) || !std::isfinite(target.maximum_z_m) ||
         !(target.maximum_z_m > target.minimum_z_m) ||
         target.state.z < target.minimum_z_m || target.state.z > target.maximum_z_m);
    if (!std::isfinite(target.state.x) || !std::isfinite(target.state.y) ||
        !std::isfinite(target.state.z) || !std::isfinite(target.state.vx) ||
        !std::isfinite(target.state.vy) || !std::isfinite(target.state.vz) ||
        !(target.capture_radius_m > 0.0F) || invalid_vertical) {
      throw std::invalid_argument{"invalid moving target reference"};
    }
  }
  if (input.dynamic_aircraft.size() > maximum_dynamic_aircraft) {
    throw std::invalid_argument{"too many dynamic aircraft for MPPI engine"};
  }
  for (const DynamicAircraftTrajectory& aircraft : input.dynamic_aircraft) {
    if (!aircraft.samples || aircraft.samples->size() != expected_steps ||
        aircraft.active_steps == 0U || aircraft.active_steps > expected_steps ||
        !std::isfinite(aircraft.footprint_radius_m) ||
        !(aircraft.footprint_radius_m >= 0.0F) ||
        !std::ranges::all_of(
            *aircraft.samples, [](const DynamicAircraftSample& sample) {
              return std::isfinite(sample.x) && std::isfinite(sample.y) &&
                     std::isfinite(sample.z);
            })) {
      throw std::invalid_argument{"invalid dynamic aircraft trajectory"};
    }
  }
  if (input.cooperative_avoidance_active && input.noncooperative_avoidance_active) {
    throw std::invalid_argument{
        "cooperative and non-cooperative avoidance cannot be active together"};
  }
  if (input.dynamic_aircraft_cost_policy) {
    const DynamicAircraftCostPolicy& policy = *input.dynamic_aircraft_cost_policy;
    if (!std::isfinite(policy.strong_separation_m) ||
        !(policy.strong_separation_m > 0.0F) ||
        !std::isfinite(policy.anticipation_separation_m) ||
        policy.anticipation_separation_m < policy.strong_separation_m ||
        !std::isfinite(policy.strong_weight) || !(policy.strong_weight >= 0.0F) ||
        !std::isfinite(policy.anticipation_weight) ||
        !(policy.anticipation_weight >= 0.0F) ||
        !std::isfinite(policy.time_to_collision_gain_s) ||
        !(policy.time_to_collision_gain_s >= 0.0F) ||
        !std::isfinite(policy.maximum_time_to_collision_multiplier) ||
        policy.maximum_time_to_collision_multiplier < 1.0F) {
      throw std::invalid_argument{"invalid dynamic aircraft cost policy"};
    }
  }
  const auto valid_preference = [](const CooperativeManeuverPreference& preference) {
    return std::isfinite(preference.direction_x) &&
           std::isfinite(preference.direction_y) &&
           std::isfinite(preference.direction_z);
  };
  if (input.cooperative_maneuver && !valid_preference(*input.cooperative_maneuver)) {
    throw std::invalid_argument{"invalid cooperative maneuver preference"};
  }
  if (input.cooperative_acquisition &&
      (!valid_preference(input.cooperative_acquisition->preference) ||
       !(input.cooperative_acquisition->minimum_positive_progress_m >= 0.0F) ||
       !(input.cooperative_acquisition->minimum_separation_gain_m >= 0.0F))) {
    throw std::invalid_argument{"invalid cooperative separation acquisition"};
  }
  if (input.noncooperative_acquisition) {
    const NonCooperativeSeparationAcquisition& acquisition =
        *input.noncooperative_acquisition;
    if (!std::isfinite(acquisition.threat_direction_x) ||
        !std::isfinite(acquisition.threat_direction_y) ||
        !std::isfinite(acquisition.threat_direction_z) ||
        !std::isfinite(acquisition.candidate_acceleration_fraction) ||
        !(acquisition.candidate_acceleration_fraction > 0.0F) ||
        acquisition.candidate_acceleration_fraction > 1.0F ||
        !std::isfinite(acquisition.candidate_duration_s) ||
        !(acquisition.candidate_duration_s > 0.0F)) {
      throw std::invalid_argument{"invalid non-cooperative separation acquisition"};
    }
  }
}

} // namespace drone_city_nav::mppi
