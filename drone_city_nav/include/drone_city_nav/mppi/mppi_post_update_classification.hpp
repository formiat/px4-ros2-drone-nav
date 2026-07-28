#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"

#include <cstdint>

namespace drone_city_nav::mppi {

enum class MppiPostUpdateClassification : std::uint8_t {
  kPreserved,
  kNoEligibleRollout,
  kInvalidMetrics,
  kRawCollision,
  kKnownSolidCollision,
  kRiskTierDegraded,
  kCriticalExposureExceeded,
  kPlanningExposureExceeded,
};

struct MppiEligibleRiskContract {
  bool available{false};
  RiskTier tier{RiskTier::kCollision};
  float best_critical_exposure_m{0.0F};
  float best_planning_exposure_m{0.0F};
  float critical_exposure_tolerance_m{0.0F};
  float planning_exposure_tolerance_m{0.0F};
  float weight_sum{0.0F};
};

struct MppiPostUpdateObservation {
  RiskTier tier{RiskTier::kCollision};
  bool raw_collision{true};
  bool known_solid_collision{false};
  float critical_exposure_m{0.0F};
  float planning_exposure_m{0.0F};
};

struct MppiPostUpdateClassificationResult {
  MppiPostUpdateClassification classification{
      MppiPostUpdateClassification::kNoEligibleRollout};
  bool contract_preserved{false};
  float critical_exposure_limit_m{0.0F};
  float planning_exposure_limit_m{0.0F};
};

[[nodiscard]] MppiPostUpdateClassificationResult
classifyMppiPostUpdate(const MppiEligibleRiskContract& eligible,
                       const MppiPostUpdateObservation& observation) noexcept;

[[nodiscard]] const char*
mppiPostUpdateClassificationName(MppiPostUpdateClassification classification) noexcept;

[[nodiscard]] const char* mppiRiskTierName(RiskTier tier) noexcept;

} // namespace drone_city_nav::mppi
