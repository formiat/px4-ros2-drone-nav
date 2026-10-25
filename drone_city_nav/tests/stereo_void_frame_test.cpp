#include "drone_city_nav/stereo_void_frame.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

// The rig's levels of 2026-10-25 (log/tools/void/reach): over the floor at 13
// and 11 m the frame is a void frame; at 7 m the floor still reads 25 at the
// 95th percentile; a dark panel at the confident depth reads 52.
TEST(StereoVoidFrame, TheFloorBeyondTheReachIsAVoidFrameAndTheFloorWithinIsNot) {
  EXPECT_TRUE(isVoidFrame({.light_sound = true,
                           .raw_brightness_p95 = 7.0,
                           .observed_share = 0.0,
                           .ring_surfaces_in_view = 0U}));
  EXPECT_TRUE(isVoidFrame({.light_sound = true,
                           .raw_brightness_p95 = 9.0,
                           .observed_share = 0.004,
                           .ring_surfaces_in_view = 0U}));
  EXPECT_FALSE(isVoidFrame({.light_sound = true,
                            .raw_brightness_p95 = 25.0,
                            .observed_share = 0.026,
                            .ring_surfaces_in_view = 0U}));
  EXPECT_FALSE(isVoidFrame({.light_sound = true,
                            .raw_brightness_p95 = 52.0,
                            .observed_share = 0.0,
                            .ring_surfaces_in_view = 0U}));
}

// A lit object the pair matches on a hundredth of the frame, a surface the
// ring sees in the camera's field, or a light read under the outage line:
// none is a void frame, whatever the brightness.
TEST(StereoVoidFrame, AMatchedObjectARingSurfaceOrAFailedLightIsNoVoid) {
  EXPECT_FALSE(isVoidFrame({.light_sound = true,
                            .raw_brightness_p95 = 7.0,
                            .observed_share = 0.012,
                            .ring_surfaces_in_view = 0U}));
  EXPECT_FALSE(isVoidFrame({.light_sound = true,
                            .raw_brightness_p95 = 7.0,
                            .observed_share = 0.0,
                            .ring_surfaces_in_view = 1U}));
  EXPECT_FALSE(isVoidFrame({.light_sound = false,
                            .raw_brightness_p95 = 7.0,
                            .observed_share = 0.0,
                            .ring_surfaces_in_view = 0U}));
}

} // namespace
} // namespace drone_city_nav
