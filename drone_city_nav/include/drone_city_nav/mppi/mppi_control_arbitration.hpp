#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"

#include <cstdint>

namespace drone_city_nav::mppi {

// Which source owns the control update.
//
// The weighted update and the deterministic route-directed candidate produce
// visibly different first controls, so a preference that flips tick to tick is
// felt as a jerk. Taking the update over from the weighted one is a switch and
// has to be earned: the candidate stays preferable for several consecutive
// ticks first. Handing it back is immediate, because the weighted update is
// the default owner and a candidate that stops being preferable has nothing to
// hold on to.
struct MppiControlArbitrationState {
  MppiControlSelection previous_selection{MppiControlSelection::kWeightedUpdate};
  std::uint32_t preferred_ticks{0U};
};

struct MppiControlArbitrationInput {
  // The policy prefers the candidate and its cost is within tolerance.
  bool candidate_preferable{false};
  std::uint32_t switch_ticks{1U};
};

// Advances `state` with this tick's preference and answers whether the
// candidate may take the update over on cost preference alone. A candidate
// that is itself the best feasible rollout, and one recovery forces, are
// decided elsewhere and are not subject to this.
[[nodiscard]] bool
stickyRouteCandidatePreference(MppiControlArbitrationState& state,
                               const MppiControlArbitrationInput& input) noexcept;

} // namespace drone_city_nav::mppi
