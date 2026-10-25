#include "drone_city_nav/stereo_void_frame.hpp"

#include <cmath>

namespace drone_city_nav {

bool isVoidFrame(const VoidFrameWitness& witness) noexcept {
  return witness.light_sound && std::isfinite(witness.raw_brightness_p95) &&
         witness.raw_brightness_p95 < kVoidRawGrey &&
         std::isfinite(witness.matched_within_reach_share) &&
         witness.matched_within_reach_share < kVoidMatchedShare &&
         witness.ring_surfaces_in_view == 0U;
}

} // namespace drone_city_nav
