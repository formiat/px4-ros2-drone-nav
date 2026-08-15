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
  const bool direct_tracking_maneuver_changed =
      observation.direct_tracking_maneuver_generation >
      direct_tracking_maneuver_generation_;
  const bool no_eligible_reseed_pending =
      no_eligible_phase_ == MppiNoEligiblePhase::kReseedPending;
  const bool requested = guide_changed || local_liveness_changed ||
                         guide_liveness_changed || direct_tracking_maneuver_changed ||
                         no_eligible_reseed_pending;

  guide_generation_ = observation.guide_generation;
  local_liveness_generation_ = observation.local_liveness_generation;
  guide_liveness_generation_ = observation.guide_liveness_generation;
  direct_tracking_maneuver_generation_ =
      observation.direct_tracking_maneuver_generation;
  if (requested) {
    ++generation_;
    if (no_eligible_phase_ == MppiNoEligiblePhase::kReseedPending &&
        recovery_reseed_pending_ && !guide_changed) {
      no_eligible_phase_ = MppiNoEligiblePhase::kAwaitingGuideChange;
    } else if (no_eligible_phase_ != MppiNoEligiblePhase::kHealthy) {
      no_eligible_phase_ = MppiNoEligiblePhase::kAwaitingReseedResult;
    }
    recovery_reseed_pending_ = false;
  }
  return {
      .generation = generation_,
      .no_eligible_recovery_generation = no_eligible_recovery_generation_,
      .no_eligible_phase = no_eligible_phase_,
      .requested = requested,
  };
}

MppiEligibleRolloutUpdate MppiNominalReseedTracker::observeEligibleRolloutResult(
    const bool available, const bool nominal_reseeded) noexcept {
  if (available) {
    no_eligible_phase_ = MppiNoEligiblePhase::kHealthy;
    recovery_reseed_pending_ = false;
    return {
        .no_eligible_recovery_generation = no_eligible_recovery_generation_,
        .phase = no_eligible_phase_,
    };
  }
  if (no_eligible_phase_ == MppiNoEligiblePhase::kHealthy) {
    ++no_eligible_recovery_generation_;
    no_eligible_phase_ = MppiNoEligiblePhase::kReseedPending;
  } else if (no_eligible_phase_ == MppiNoEligiblePhase::kAwaitingReseedResult &&
             nominal_reseeded) {
    ++no_eligible_recovery_generation_;
    no_eligible_phase_ = MppiNoEligiblePhase::kReseedPending;
    recovery_reseed_pending_ = true;
    return {
        .no_eligible_recovery_generation = no_eligible_recovery_generation_,
        .phase = no_eligible_phase_,
        .guide_replan_requested = true,
    };
  }
  return {
      .no_eligible_recovery_generation = no_eligible_recovery_generation_,
      .phase = no_eligible_phase_,
  };
}

const char* mppiNoEligiblePhaseName(const MppiNoEligiblePhase phase) noexcept {
  switch (phase) {
    case MppiNoEligiblePhase::kHealthy:
      return "healthy";
    case MppiNoEligiblePhase::kReseedPending:
      return "reseed_pending";
    case MppiNoEligiblePhase::kAwaitingReseedResult:
      return "awaiting_reseed_result";
    case MppiNoEligiblePhase::kAwaitingGuideChange:
      return "awaiting_guide_change";
  }
  return "unknown";
}

} // namespace drone_city_nav
