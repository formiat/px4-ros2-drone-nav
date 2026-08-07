#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace drone_city_nav {

struct MppiRolloutBudgetConfig {
  std::size_t full_rollouts{8192U};
  std::size_t open_static_rollouts{6144U};
  std::size_t direct_tracking_rollouts{4096U};
  float minimum_reduced_clearance_m{8.0F};
  double maximum_world_age_ms{250.0};
  double maximum_tracking_age_ms{250.0};
};

struct MppiRolloutBudgetObservation {
  bool static_world{false};
  bool guide_available{false};
  bool direct_tracking{false};
  bool clearance_valid{false};
  float clearance_m{0.0F};
  double world_age_ms{0.0};
  double tracking_age_ms{0.0};
  mppi::RiskTier required_risk_tier{mppi::RiskTier::kPreferred};
};

enum class MppiRolloutBudgetReason : std::uint8_t {
  kFullInvalidConfiguration,
  kFullUnavailableGuide,
  kFullWorldUncertain,
  kFullTrackingUncertain,
  kFullElevatedRisk,
  kFullLowClearance,
  kFullNoStaticExploration,
  kReducedOpenStatic,
  kReducedDirectTracking,
};

struct MppiRolloutBudgetDecision {
  std::size_t active_rollouts{0U};
  MppiRolloutBudgetReason reason{MppiRolloutBudgetReason::kFullInvalidConfiguration};
  bool reduced{false};
};

[[nodiscard]] MppiRolloutBudgetDecision
selectMppiRolloutBudget(const MppiRolloutBudgetConfig& config,
                        const MppiRolloutBudgetObservation& observation) noexcept;

[[nodiscard]] std::string_view
mppiRolloutBudgetReasonName(MppiRolloutBudgetReason reason) noexcept;

} // namespace drone_city_nav
