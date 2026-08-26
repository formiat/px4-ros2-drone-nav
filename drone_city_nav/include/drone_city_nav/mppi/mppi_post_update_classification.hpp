#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"

#include <cstdint>

namespace drone_city_nav::mppi {

enum class MppiPostUpdateClassification : std::uint8_t {
  kPreserved,
  kNoFeasibleRollout,
  kInvalidMetrics,
  kAltitudeEnvelopeViolation,
  kRawCollision,
  kUnknownSpaceViolation,
  kKnownSolidCollision,
  kRouteTerminalCrossTrackViolation,
};

struct MppiFeasibilityContract {
  bool available{false};
  float weight_sum{0.0F};
};

struct MppiPostUpdateObservation {
  bool altitude_envelope_violation{false};
  bool raw_collision{true};
  bool unknown_space_violation{false};
  bool known_solid_collision{false};
  bool route_terminal_cross_track_violation{false};
};

struct MppiPostUpdateClassificationResult {
  MppiPostUpdateClassification classification{
      MppiPostUpdateClassification::kNoFeasibleRollout};
  bool executable{false};
};

[[nodiscard]] MppiPostUpdateClassificationResult
classifyMppiPostUpdate(const MppiFeasibilityContract& feasibility,
                       const MppiPostUpdateObservation& observation) noexcept;

[[nodiscard]] const char*
mppiPostUpdateClassificationName(MppiPostUpdateClassification classification) noexcept;

[[nodiscard]] const char* mppiRiskTierName(RiskTier tier) noexcept;

} // namespace drone_city_nav::mppi
