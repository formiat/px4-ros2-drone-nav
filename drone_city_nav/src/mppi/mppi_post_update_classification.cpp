#include "drone_city_nav/mppi/mppi_post_update_classification.hpp"

#include <cmath>

namespace drone_city_nav::mppi {
namespace {

[[nodiscard]] bool finiteContract(const MppiEligibleRiskContract& contract) noexcept {
  return std::isfinite(contract.best_critical_exposure_m) &&
         std::isfinite(contract.best_planning_exposure_m) &&
         std::isfinite(contract.critical_exposure_tolerance_m) &&
         std::isfinite(contract.planning_exposure_tolerance_m) &&
         std::isfinite(contract.weight_sum) &&
         contract.best_critical_exposure_m >= 0.0F &&
         contract.best_planning_exposure_m >= 0.0F &&
         contract.critical_exposure_tolerance_m >= 0.0F &&
         contract.planning_exposure_tolerance_m >= 0.0F && contract.weight_sum > 0.0F;
}

[[nodiscard]] bool
finiteObservation(const MppiPostUpdateObservation& observation) noexcept {
  return std::isfinite(observation.critical_exposure_m) &&
         std::isfinite(observation.planning_exposure_m) &&
         observation.critical_exposure_m >= 0.0F &&
         observation.planning_exposure_m >= 0.0F;
}

} // namespace

MppiPostUpdateClassificationResult
classifyMppiPostUpdate(const MppiEligibleRiskContract& eligible,
                       const MppiPostUpdateObservation& observation) noexcept {
  MppiPostUpdateClassificationResult result;
  result.critical_exposure_limit_m =
      eligible.best_critical_exposure_m + eligible.critical_exposure_tolerance_m;
  result.planning_exposure_limit_m =
      eligible.best_planning_exposure_m + eligible.planning_exposure_tolerance_m;
  if (!eligible.available) {
    result.classification = MppiPostUpdateClassification::kNoEligibleRollout;
    return result;
  }
  if (!finiteContract(eligible) || !finiteObservation(observation)) {
    result.classification = MppiPostUpdateClassification::kInvalidMetrics;
    return result;
  }
  if (observation.raw_collision) {
    result.classification = MppiPostUpdateClassification::kRawCollision;
    return result;
  }
  if (observation.known_solid_collision) {
    result.classification = MppiPostUpdateClassification::kKnownSolidCollision;
    return result;
  }
  if (static_cast<std::uint8_t>(observation.tier) >
      static_cast<std::uint8_t>(eligible.tier)) {
    result.classification = MppiPostUpdateClassification::kRiskTierDegraded;
    return result;
  }
  if (observation.critical_exposure_m > result.critical_exposure_limit_m) {
    result.classification = MppiPostUpdateClassification::kCriticalExposureExceeded;
    return result;
  }
  if (observation.planning_exposure_m > result.planning_exposure_limit_m) {
    result.classification = MppiPostUpdateClassification::kPlanningExposureExceeded;
    return result;
  }
  result.classification = MppiPostUpdateClassification::kPreserved;
  result.contract_preserved = true;
  return result;
}

const char* mppiPostUpdateClassificationName(
    const MppiPostUpdateClassification classification) noexcept {
  switch (classification) {
    case MppiPostUpdateClassification::kPreserved:
      return "preserved";
    case MppiPostUpdateClassification::kNoEligibleRollout:
      return "no_eligible_rollout";
    case MppiPostUpdateClassification::kInvalidMetrics:
      return "invalid_metrics";
    case MppiPostUpdateClassification::kRawCollision:
      return "raw_collision";
    case MppiPostUpdateClassification::kKnownSolidCollision:
      return "known_solid_collision";
    case MppiPostUpdateClassification::kRiskTierDegraded:
      return "risk_tier_degraded";
    case MppiPostUpdateClassification::kCriticalExposureExceeded:
      return "critical_exposure_exceeded";
    case MppiPostUpdateClassification::kPlanningExposureExceeded:
      return "planning_exposure_exceeded";
  }
  return "unknown";
}

const char* mppiRiskTierName(const RiskTier tier) noexcept {
  switch (tier) {
    case RiskTier::kPreferred:
      return "preferred";
    case RiskTier::kPlanning:
      return "planning";
    case RiskTier::kCritical:
      return "critical";
    case RiskTier::kCollision:
      return "collision";
  }
  return "unknown";
}

} // namespace drone_city_nav::mppi
