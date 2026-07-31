#pragma once

#include <cstdint>

namespace drone_city_nav {

struct MppiNominalReseedObservation {
  std::uint64_t guide_generation{0U};
  std::uint64_t local_liveness_generation{0U};
  std::uint64_t guide_liveness_generation{0U};
  std::uint64_t safety_rejection_generation{0U};
};

struct MppiNominalReseedUpdate {
  std::uint64_t generation{0U};
  bool requested{false};
};

class MppiNominalReseedTracker {
public:
  [[nodiscard]] MppiNominalReseedUpdate
  update(const MppiNominalReseedObservation& observation) noexcept;

  void observeEligibleRolloutResult(bool available) noexcept;

private:
  std::uint64_t generation_{0U};
  std::uint64_t guide_generation_{0U};
  std::uint64_t local_liveness_generation_{0U};
  std::uint64_t guide_liveness_generation_{0U};
  std::uint64_t safety_rejection_generation_{0U};
  bool no_eligible_episode_active_{false};
  bool no_eligible_episode_reseed_pending_{false};
};

} // namespace drone_city_nav
