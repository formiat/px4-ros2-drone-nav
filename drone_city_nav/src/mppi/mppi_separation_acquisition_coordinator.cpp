#include "drone_city_nav/mppi/mppi_separation_acquisition_coordinator.hpp"

#include <stdexcept>
#include <utility>

namespace drone_city_nav::mppi {

SeparationAcquisitionCoordinatorResult SeparationAcquisitionCoordinator::update(
    const SeparationAcquisitionCoordinatorInput& input) {
  if (input.cooperative_avoidance_active && input.noncooperative_avoidance_active) {
    throw std::invalid_argument{
        "separation acquisition modes cannot be active together"};
  }
  SeparationAcquisitionCoordinatorResult result;
  CooperativeSeparationAcquisitionLifecycleResult cooperative =
      cooperative_.update(CooperativeSeparationAcquisitionLifecycleInput{
          .avoidance_active = input.cooperative_avoidance_active,
          .acquisition = input.cooperative_acquisition,
          .evaluation =
              CooperativeSeparationAcquisitionEvaluationInput{
                  .initial_state = input.initial_state,
                  .target = input.target,
                  .route = input.route,
                  .initial_route_station_m = input.initial_route_station_m,
                  .reference_speed_mps = input.reference_speed_mps,
                  .previous_applied_control = input.previous_applied_control,
                  .first_control_interval_s = input.first_control_interval_s,
                  .grid = input.grid,
                  .esdf = input.esdf,
                  .known_solids = input.known_solids,
                  .aircraft = input.aircraft,
                  .config = input.config,
              },
      });
  NonCooperativeAcquisitionLifecycleResult noncooperative =
      noncooperative_.update(NonCooperativeAcquisitionLifecycleInput{
          .avoidance_active = input.noncooperative_avoidance_active,
          .acquisition = input.noncooperative_acquisition,
          .evaluation =
              NonCooperativeAcquisitionEvaluationInput{
                  .initial_state = input.initial_state,
                  .target = input.target,
                  .route = input.route,
                  .initial_route_station_m = input.initial_route_station_m,
                  .reference_speed_mps = input.reference_speed_mps,
                  .previous_applied_control = input.previous_applied_control,
                  .first_control_interval_s = input.first_control_interval_s,
                  .grid = input.grid,
                  .esdf = input.esdf,
                  .known_solids = input.known_solids,
                  .aircraft = input.aircraft,
                  .cost_policy = input.dynamic_aircraft_cost_policy,
                  .config = input.config,
              },
      });
  if (cooperative.nominal_reseed && noncooperative.nominal_reseed) {
    throw std::logic_error{"multiple separation acquisition reseeds requested"};
  }
  result.nominal_reseed = cooperative.nominal_reseed
                              ? std::move(cooperative.nominal_reseed)
                              : std::move(noncooperative.nominal_reseed);
  result.cooperative = std::move(cooperative.acquisition);
  result.noncooperative = std::move(noncooperative.acquisition);
  result.cooperative_acquisition_reseeded = cooperative.acquisition_reseeded;
  result.cooperative_release_reseeded = cooperative.release_reseeded;
  result.noncooperative_acquisition_reseeded = noncooperative.acquisition_reseeded;
  result.noncooperative_release_reseeded = noncooperative.release_reseeded;
  return result;
}

} // namespace drone_city_nav::mppi
