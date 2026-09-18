#include "drone_city_nav/mppi/mppi_separation_acquisition_coordinator.hpp"

#include <utility>

namespace drone_city_nav::mppi {

SeparationAcquisitionCoordinatorResult SeparationAcquisitionCoordinator::update(
    const SeparationAcquisitionCoordinatorInput& input) {
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
                  .aircraft = input.aircraft,
                  .config = input.config,
              },
      });
  result.nominal_reseed = std::move(cooperative.nominal_reseed);
  result.cooperative = std::move(cooperative.acquisition);
  result.cooperative_acquisition_reseeded = cooperative.acquisition_reseeded;
  result.cooperative_release_reseeded = cooperative.release_reseeded;
  return result;
}

} // namespace drone_city_nav::mppi
