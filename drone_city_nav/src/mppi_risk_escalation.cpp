#include "drone_city_nav/mppi_risk_escalation.hpp"

#include <stdexcept>

namespace drone_city_nav {

MppiRiskEscalation::MppiRiskEscalation(const MppiRiskEscalationConfig& config)
    : config_{config} {
  if (config_.recovery_stable_cycles == 0U) {
    throw std::invalid_argument{"risk escalation recovery cycles must be positive"};
  }
}

MppiRiskEscalationResult
MppiRiskEscalation::update(const MppiRiskEscalationObservation& observation) noexcept {
  const mppi::RiskTier previous = maximum_eligible_tier_;
  if (observation.reseed_generation > last_reseed_generation_) {
    last_reseed_generation_ = observation.reseed_generation;
    recovery_cycles_ = 0U;
    maximum_eligible_tier_ = maximum_eligible_tier_ == mppi::RiskTier::kPreferred
                                 ? mppi::RiskTier::kPlanning
                                 : mppi::RiskTier::kCritical;
  } else if (observation.stable_progress) {
    ++recovery_cycles_;
    if (recovery_cycles_ >= config_.recovery_stable_cycles) {
      maximum_eligible_tier_ = mppi::RiskTier::kPreferred;
      recovery_cycles_ = 0U;
    }
  } else {
    recovery_cycles_ = 0U;
  }
  return {
      .maximum_eligible_tier = maximum_eligible_tier_,
      .changed = previous != maximum_eligible_tier_,
  };
}

} // namespace drone_city_nav
