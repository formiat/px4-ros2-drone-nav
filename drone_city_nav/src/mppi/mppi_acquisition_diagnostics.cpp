#include "drone_city_nav/mppi/mppi_acquisition_diagnostics.hpp"

namespace drone_city_nav::mppi {

void populateSeparationAcquisitionResult(
    MppiTickResult& result, const SeparationAcquisitionCoordinatorResult& acquisition,
    const bool cooperative_candidates_injected,
    const std::size_t dynamic_aircraft_count) noexcept {
  result.cooperative_acquisition_reseeded =
      acquisition.cooperative_acquisition_reseeded;
  result.cooperative_release_reseeded = acquisition.cooperative_release_reseeded;
  result.cooperative_acquisition_available = acquisition.cooperative.available;
  result.cooperative_acquisition_positive_progress =
      acquisition.cooperative.positive_progress;
  result.cooperative_acquisition_backward_fallback =
      acquisition.cooperative.backward_fallback;
  result.cooperative_acquisition_candidate_index =
      acquisition.cooperative.candidate_index;
  result.cooperative_acquisition_head_progress_m =
      acquisition.cooperative.head_progress_m;
  result.cooperative_acquisition_terminal_progress_m =
      acquisition.cooperative.terminal_progress_m;
  result.cooperative_acquisition_separation_gain_m =
      acquisition.cooperative.separation_gain_m;
  result.cooperative_candidates_injected = cooperative_candidates_injected;
  result.noncooperative_acquisition_reseeded =
      acquisition.noncooperative_acquisition_reseeded;
  result.noncooperative_release_reseeded = acquisition.noncooperative_release_reseeded;
  result.noncooperative_acquisition_available = acquisition.noncooperative.available;
  result.noncooperative_acquisition_candidate_index =
      acquisition.noncooperative.candidate_index;
  result.noncooperative_acquisition_maneuver = acquisition.noncooperative.maneuver;
  result.noncooperative_acquisition_minimum_separation_m =
      acquisition.noncooperative.minimum_separation_m;
  result.noncooperative_acquisition_separation_gain_m =
      acquisition.noncooperative.separation_gain_m;
  result.noncooperative_acquisition_head_progress_m =
      acquisition.noncooperative.head_progress_m;
  result.noncooperative_acquisition_terminal_progress_m =
      acquisition.noncooperative.terminal_progress_m;
  result.dynamic_aircraft_count = dynamic_aircraft_count;
}

} // namespace drone_city_nav::mppi
