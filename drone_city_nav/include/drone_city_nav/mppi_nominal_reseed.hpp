#pragma once

#include <cstdint>

namespace drone_city_nav {

enum class MppiNoEligiblePhase : std::uint8_t {
  kHealthy,
  kReseedPending,
  kAwaitingReseedResult,
  kAwaitingGuideChange,
};

struct MppiNominalReseedObservation {
  std::uint64_t guide_generation{0U};
  std::uint64_t local_liveness_generation{0U};
  std::uint64_t guide_liveness_generation{0U};
  std::uint64_t safety_rejection_generation{0U};
};

struct MppiNominalReseedUpdate {
  std::uint64_t generation{0U};
  std::uint64_t no_eligible_recovery_generation{0U};
  MppiNoEligiblePhase no_eligible_phase{MppiNoEligiblePhase::kHealthy};
  bool requested{false};
};

struct MppiEligibleRolloutUpdate {
  std::uint64_t no_eligible_recovery_generation{0U};
  MppiNoEligiblePhase phase{MppiNoEligiblePhase::kHealthy};
  bool guide_replan_requested{false};
};

class MppiNominalReseedTracker {
public:
  [[nodiscard]] MppiNominalReseedUpdate
  update(const MppiNominalReseedObservation& observation) noexcept;

  [[nodiscard]] MppiEligibleRolloutUpdate
  observeEligibleRolloutResult(bool available, bool nominal_reseeded) noexcept;

private:
  std::uint64_t generation_{0U};
  std::uint64_t guide_generation_{0U};
  std::uint64_t local_liveness_generation_{0U};
  std::uint64_t guide_liveness_generation_{0U};
  std::uint64_t safety_rejection_generation_{0U};
  std::uint64_t no_eligible_recovery_generation_{0U};
  MppiNoEligiblePhase no_eligible_phase_{MppiNoEligiblePhase::kHealthy};
  bool recovery_reseed_pending_{false};
};

[[nodiscard]] const char* mppiNoEligiblePhaseName(MppiNoEligiblePhase phase) noexcept;

} // namespace drone_city_nav
