#pragma once

#include <cstddef>

namespace drone_city_nav {

// The void frame (roadmap item 31, specification K46): a frame of the stereo
// pair under a carried light its driver reads sound that shows nothing
// within the light's reach, the pair's confident depth (K7, 5.2 m). Measured
// on the rig over a textured floor and on the hall (2026-10-25): a facing
// matte surface of the location's darkest albedo (0.07) at 5.2 m reads 52 raw
// levels at the frame's centre, one of the mean albedo (0.10) at 7 m 46 and at
// 9 m 33; the hall's floor 14 m below and its ceiling 6 m above (at the top
// rows' slant of 10 m and more) lift the frame's 95th percentile to 18 to 39
// (r2174), the rig's floor 7 m below to 25, and the imager's noise alone to 7.
// A frame whose 95th percentile of raw brightness (the gained frame's over the
// gain) is under 46, what a surface of the mean albedo gives from beyond the
// reach, shows no plain lit surface of the location's albedo within it; one
// the pair matches within the confident depth on less than 1 percent of (half
// the braking contract's blind 2 percent, K9) shows no textured object within
// it either, whatever it matches farther (the hall's floor gives 5 percent of
// matches 12 to 20 m away); and with no surface of the time-of-flight ring in
// the camera's field (the ring's own emitter sees 2.8 m whatever the light)
// the frame is empty to the confident depth. A black object in the dark is
// invisible to a camera, void or no void, as K7's guarantee is of the
// location's surfaces; the ring's 2.8 m stand for it.
struct VoidFrameWitness {
  // The light's reading is at or over the outage line (K12, 0.25).
  bool light_sound{false};
  // The left frame's 95th percentile of grey over the camera's gain.
  double raw_brightness_p95{0.0};
  // The share of the frame's sampled pixels the pair matched within the
  // confident depth.
  double matched_within_reach_share{0.0};
  // The ring's surfaces within the camera's field.
  std::size_t ring_surfaces_in_view{0U};
};

inline constexpr double kVoidRawGrey{46.0};
inline constexpr double kVoidMatchedShare{0.01};

[[nodiscard]] bool isVoidFrame(const VoidFrameWitness& witness) noexcept;

} // namespace drone_city_nav
