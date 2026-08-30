#include "drone_city_nav/mppi/mppi_post_update_classification.hpp"

#include <cmath>

namespace drone_city_nav::mppi {
namespace {

[[nodiscard]] bool finiteContract(const MppiFeasibilityContract& contract) noexcept {
  return std::isfinite(contract.weight_sum) && contract.weight_sum > 0.0F;
}

} // namespace

MppiPostUpdateClassificationResult
classifyMppiPostUpdate(const MppiFeasibilityContract& feasibility,
                       const MppiPostUpdateObservation& observation) noexcept {
  MppiPostUpdateClassificationResult result;
  if (!feasibility.available) {
    result.classification = MppiPostUpdateClassification::kNoFeasibleRollout;
    return result;
  }
  if (!finiteContract(feasibility)) {
    result.classification = MppiPostUpdateClassification::kInvalidMetrics;
    return result;
  }
  if (observation.altitude_envelope_violation) {
    result.classification = MppiPostUpdateClassification::kAltitudeEnvelopeViolation;
    return result;
  }
  if (observation.route_terminal_cross_track_violation) {
    result.classification =
        MppiPostUpdateClassification::kRouteTerminalCrossTrackViolation;
    return result;
  }
  result.classification = MppiPostUpdateClassification::kPreserved;
  result.executable = true;
  return result;
}

const char* mppiPostUpdateClassificationName(
    const MppiPostUpdateClassification classification) noexcept {
  switch (classification) {
    case MppiPostUpdateClassification::kPreserved:
      return "preserved";
    case MppiPostUpdateClassification::kNoFeasibleRollout:
      return "no_feasible_rollout";
    case MppiPostUpdateClassification::kInvalidMetrics:
      return "invalid_metrics";
    case MppiPostUpdateClassification::kAltitudeEnvelopeViolation:
      return "altitude_envelope_violation";
    case MppiPostUpdateClassification::kRouteTerminalCrossTrackViolation:
      return "route_terminal_cross_track_violation";
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
