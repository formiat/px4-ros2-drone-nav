#include "drone_city_nav/mppi_rollout_budget.hpp"

#include <cmath>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool validConfig(const MppiRolloutBudgetConfig& config) noexcept {
  return config.full_rollouts > 0U && config.open_static_rollouts > 0U &&
         config.open_static_rollouts <= config.full_rollouts &&
         config.minimum_reduced_clearance_m > 0.0F && config.maximum_world_age_ms > 0.0;
}

} // namespace

MppiRolloutBudgetDecision
selectMppiRolloutBudget(const MppiRolloutBudgetConfig& config,
                        const MppiRolloutBudgetObservation& observation) noexcept {
  MppiRolloutBudgetDecision decision{
      .active_rollouts = config.full_rollouts,
      .reason = MppiRolloutBudgetReason::kFullInvalidConfiguration,
  };
  if (!validConfig(config)) {
    return decision;
  }
  if (!observation.route_available) {
    decision.reason = MppiRolloutBudgetReason::kFullUnavailableRoute;
    return decision;
  }
  if (!std::isfinite(observation.world_age_ms) || observation.world_age_ms < 0.0 ||
      observation.world_age_ms > config.maximum_world_age_ms ||
      !observation.clearance_valid || !std::isfinite(observation.clearance_m)) {
    decision.reason = MppiRolloutBudgetReason::kFullWorldUncertain;
    return decision;
  }
  if (observation.required_risk_tier != mppi::RiskTier::kPreferred) {
    decision.reason = MppiRolloutBudgetReason::kFullElevatedRisk;
    return decision;
  }
  if (observation.clearance_m < config.minimum_reduced_clearance_m) {
    decision.reason = MppiRolloutBudgetReason::kFullLowClearance;
    return decision;
  }
  if (observation.static_world) {
    decision.active_rollouts = config.open_static_rollouts;
    decision.reason = MppiRolloutBudgetReason::kReducedOpenStatic;
    decision.reduced = decision.active_rollouts < config.full_rollouts;
    return decision;
  }
  decision.reason = MppiRolloutBudgetReason::kFullNoStaticExploration;
  return decision;
}

std::string_view
mppiRolloutBudgetReasonName(const MppiRolloutBudgetReason reason) noexcept {
  switch (reason) {
    case MppiRolloutBudgetReason::kFullInvalidConfiguration:
      return "full_invalid_configuration";
    case MppiRolloutBudgetReason::kFullUnavailableRoute:
      return "full_unavailable_route";
    case MppiRolloutBudgetReason::kFullWorldUncertain:
      return "full_world_uncertain";
    case MppiRolloutBudgetReason::kFullElevatedRisk:
      return "full_elevated_risk";
    case MppiRolloutBudgetReason::kFullLowClearance:
      return "full_low_clearance";
    case MppiRolloutBudgetReason::kFullNoStaticExploration:
      return "full_no_static_exploration";
    case MppiRolloutBudgetReason::kReducedOpenStatic:
      return "reduced_open_static";
  }
  return "unknown";
}

} // namespace drone_city_nav
