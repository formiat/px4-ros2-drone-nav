#pragma once

#include <cstddef>

namespace drone_city_nav {

// The void frame (roadmap item 31, specification K46): a frame of the stereo
// pair under a carried light its driver reads sound that shows nothing
// within the light's reach. Measured on the rig over a textured floor
// (2026-10-25): a facing matte surface of the location's albedo (0.07) at the
// pair's confident depth, 5.2 m, reads 52 raw levels at the frame's centre
// and more nearer; the floor 7 m below reads 25 at the frame's 95th
// percentile, 9 m below 14, 11 m below 9, and the imager's noise alone 7. A
// frame whose 95th percentile of raw brightness (the gained frame's over the
// gain) is under 24, half the dark panel's at 5.2 m, shows no lit surface of
// that albedo covering 5 percent of the frame or more within the reach; and
// one the pair matches on less than 1 percent of (half the braking contract's
// blind 2 percent, K9) shows no smaller lit object the pair can see either;
// with no surface of the time-of-flight ring in the camera's field (the
// ring's own emitter sees 2.8 m whatever the light) the frame is empty to the
// confident depth. A black object in the dark is invisible to a camera, void
// or no void, as K7's guarantee is of the location's surfaces.
struct VoidFrameWitness {
  // The light's reading is at or over the outage line (K12, 0.25).
  bool light_sound{false};
  // The left frame's 95th percentile of grey over the camera's gain.
  double raw_brightness_p95{0.0};
  // The share of the frame's sampled pixels the pair matched.
  double observed_share{0.0};
  // The ring's surfaces within the camera's field.
  std::size_t ring_surfaces_in_view{0U};
};

inline constexpr double kVoidRawGrey{24.0};
inline constexpr double kVoidObservedShare{0.01};

[[nodiscard]] bool isVoidFrame(const VoidFrameWitness& witness) noexcept;

} // namespace drone_city_nav
