#include "drone_city_nav/mppi/mppi_control_arbitration.hpp"

#include <algorithm>

namespace drone_city_nav::mppi {

bool stickyRouteCandidatePreference(MppiControlArbitrationState& state,
                                    const MppiControlArbitrationInput& input) noexcept {
  state.preferred_ticks = input.candidate_preferable ? state.preferred_ticks + 1U : 0U;
  if (!input.candidate_preferable) {
    return false;
  }
  if (state.previous_selection == MppiControlSelection::kRouteDirectedCandidate) {
    return true;
  }
  return state.preferred_ticks >= std::max<std::uint32_t>(input.switch_ticks, 1U);
}

} // namespace drone_city_nav::mppi
