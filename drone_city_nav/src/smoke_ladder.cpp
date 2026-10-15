#include "drone_city_nav/smoke_ladder.hpp"

namespace drone_city_nav {

SmokeLadderAction SmokeLadder::observe(const SmokeLadderInput& input) {
  const bool in_smoke_now = input.blind && (input.smoke_share >= kZoneShare ||
                                            (input.lit && input.dead_reckoning));
  if (!in_smoke_now) {
    in_smoke_since_ns_.reset();
  } else if (!in_smoke_since_ns_.has_value()) {
    in_smoke_since_ns_ = input.now_ns;
  }
  const bool in_smoke =
      in_smoke_since_ns_.has_value() &&
      static_cast<double>(input.now_ns - *in_smoke_since_ns_) * 1.0e-9 >= kEntryS;
  const bool seeing = !input.blind && input.smoke_share < kClearShare;
  switch (state_) {
    case State::kNone:
      if (in_smoke) {
        state_ = State::kInside;
        ++entries_;
      }
      return SmokeLadderAction::kNone;
    case State::kInside:
      if (!seeing || !input.position_fresh) {
        return SmokeLadderAction::kNone;
      }
      if (entries_ >= 2U) {
        state_ = State::kGivenUp;
        return SmokeLadderAction::kGiveUp;
      }
      state_ = State::kHolding;
      hold_since_ns_ = input.now_ns;
      return SmokeLadderAction::kHold;
    case State::kHolding:
      if (in_smoke) {
        // The smoke came to the vehicle: a second entry.
        state_ = State::kInside;
        ++entries_;
        return SmokeLadderAction::kNone;
      }
      if (static_cast<double>(input.now_ns - hold_since_ns_) * 1.0e-9 >= kHoldS) {
        state_ = State::kNone;
        return SmokeLadderAction::kRestore;
      }
      return SmokeLadderAction::kNone;
    case State::kGivenUp:
      return SmokeLadderAction::kNone;
  }
  return SmokeLadderAction::kNone;
}

} // namespace drone_city_nav
