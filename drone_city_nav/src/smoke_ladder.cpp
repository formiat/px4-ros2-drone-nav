#include "drone_city_nav/smoke_ladder.hpp"

#include <cmath>

namespace drone_city_nav {

SmokeLadderAction SmokeLadder::observe(const SmokeLadderInput& input) {
  const bool in_smoke = input.blind && input.smoke_share >= kZoneShare;
  const bool seeing = !input.blind && input.smoke_share < kClearShare;
  if (seeing && input.position_fresh) {
    seeing_place_ = input.position;
    smoke_since_ns_ = 0;
    if (state_ != State::kNone) {
      state_ = State::kNone;
      return SmokeLadderAction::kRestore;
    }
    return SmokeLadderAction::kNone;
  }
  if (in_smoke && smoke_since_ns_ == 0) {
    smoke_since_ns_ = input.now_ns;
  } else if (!in_smoke && state_ == State::kNone) {
    smoke_since_ns_ = 0;
  }
  if (state_ == State::kNone && in_smoke && smoke_since_ns_ > 0 &&
      static_cast<double>(input.now_ns - smoke_since_ns_) * 1.0e-9 >= kBlindS &&
      seeing_place_.has_value() && input.position_fresh) {
    state_ = State::kRetreating;
    return SmokeLadderAction::kRetreat;
  }
  if (state_ == State::kRetreating && seeing_place_.has_value() &&
      std::hypot(input.position.x - seeing_place_->x,
                 input.position.y - seeing_place_->y,
                 input.position.z - seeing_place_->z) <= kSeeingReachM) {
    state_ = State::kHolding;
    hold_since_ns_ = input.now_ns;
    return SmokeLadderAction::kHold;
  }
  if (state_ == State::kHolding &&
      static_cast<double>(input.now_ns - hold_since_ns_) * 1.0e-9 >= kHoldS) {
    state_ = State::kGivenUp;
    return SmokeLadderAction::kGiveUp;
  }
  return SmokeLadderAction::kNone;
}

} // namespace drone_city_nav
