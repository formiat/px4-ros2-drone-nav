#include "drone_city_nav/stereo_void_frame.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

// The rig's levels and the hall (2026-10-25): the hall's floor 14 m below and
// its ceiling 6 m above lift the frame to 18 to 39 levels with matches far
// beyond the reach (r2174), the rig's floor 13 m below to 7: void frames; a
// dark panel at the confident depth reads 52, a mean one at 7 m 46: not.
TEST(StereoVoidFrame, TheFloorBeyondTheReachIsAVoidFrameAndASurfaceWithinIsNot) {
  EXPECT_TRUE(isVoidFrame({.light_sound = true,
                           .raw_brightness_p95 = 7.0,
                           .matched_within_reach_share = 0.0,
                           .ring_surfaces_in_view = 0U}));
  EXPECT_TRUE(isVoidFrame({.light_sound = true,
                           .raw_brightness_p95 = 39.0,
                           .matched_within_reach_share = 0.0,
                           .ring_surfaces_in_view = 0U}));
  EXPECT_FALSE(isVoidFrame({.light_sound = true,
                            .raw_brightness_p95 = 46.0,
                            .matched_within_reach_share = 0.0,
                            .ring_surfaces_in_view = 0U}));
  EXPECT_FALSE(isVoidFrame({.light_sound = true,
                            .raw_brightness_p95 = 52.0,
                            .matched_within_reach_share = 0.0,
                            .ring_surfaces_in_view = 0U}));
}

// A lit object the pair matches within the reach on a hundredth of the frame,
// a surface the ring sees in the camera's field, or a light read under the
// outage line: none is a void frame, whatever the brightness.
TEST(StereoVoidFrame, AMatchedObjectWithinReachARingSurfaceOrAFailedLightIsNoVoid) {
  EXPECT_FALSE(isVoidFrame({.light_sound = true,
                            .raw_brightness_p95 = 7.0,
                            .matched_within_reach_share = 0.012,
                            .ring_surfaces_in_view = 0U}));
  EXPECT_FALSE(isVoidFrame({.light_sound = true,
                            .raw_brightness_p95 = 7.0,
                            .matched_within_reach_share = 0.0,
                            .ring_surfaces_in_view = 1U}));
  EXPECT_FALSE(isVoidFrame({.light_sound = false,
                            .raw_brightness_p95 = 7.0,
                            .matched_within_reach_share = 0.0,
                            .ring_surfaces_in_view = 0U}));
}

} // namespace
} // namespace drone_city_nav
