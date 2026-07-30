#include "drone_city_nav/mppi_nominal_reseed.hpp"

namespace drone_city_nav {

MppiNominalReseedUpdate MppiNominalReseedTracker::update(
    const MppiNominalReseedObservation& observation) noexcept {
  const bool guide_changed = observation.guide_generation != 0U &&
                             observation.guide_generation != guide_generation_;
  const bool local_liveness_changed =
      observation.local_liveness_generation > local_liveness_generation_;
  const bool guide_liveness_changed =
      observation.guide_liveness_generation > guide_liveness_generation_;
  const bool safety_rejection_changed =
      observation.safety_rejection_generation > safety_rejection_generation_;
  const bool requested = guide_changed || local_liveness_changed ||
                         guide_liveness_changed || safety_rejection_changed ||
                         no_eligible_rollout_pending_;

  guide_generation_ = observation.guide_generation;
  local_liveness_generation_ = observation.local_liveness_generation;
  guide_liveness_generation_ = observation.guide_liveness_generation;
  safety_rejection_generation_ = observation.safety_rejection_generation;
  no_eligible_rollout_pending_ = false;
  if (requested) {
    ++generation_;
  }
  return {.generation = generation_, .requested = requested};
}

void MppiNominalReseedTracker::observeEligibleRolloutResult(
    const bool available) noexcept {
  no_eligible_rollout_pending_ = !available;
}

} // namespace drone_city_nav
