#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"

#include <cstddef>
#include <cstdint>

namespace drone_city_nav {

struct MppiRiskEscalationConfig {
  std::size_t recovery_stable_cycles{20U};
};

struct MppiRiskEscalationObservation {
  std::uint64_t reseed_generation{0U};
  std::uint64_t no_eligible_recovery_generation{0U};
  bool stable_progress{false};
};

struct MppiRiskEscalationResult {
  mppi::RiskTier maximum_eligible_tier{mppi::RiskTier::kPreferred};
  bool changed{false};
};

class MppiRiskEscalation {
public:
  explicit MppiRiskEscalation(const MppiRiskEscalationConfig& config = {});

  [[nodiscard]] MppiRiskEscalationResult
  update(const MppiRiskEscalationObservation& observation) noexcept;

private:
  MppiRiskEscalationConfig config_{};
  std::uint64_t last_reseed_generation_{0U};
  std::uint64_t last_no_eligible_recovery_generation_{0U};
  std::size_t recovery_cycles_{0U};
  mppi::RiskTier maximum_eligible_tier_{mppi::RiskTier::kPreferred};
};

} // namespace drone_city_nav
