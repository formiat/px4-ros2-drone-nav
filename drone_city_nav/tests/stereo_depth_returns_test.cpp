#include "drone_city_nav/stereo_depth_returns.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace drone_city_nav {
namespace {

// The working pair of roadmap item 14: 1280 px over 120 degrees, 0.20 m apart.
constexpr StereoPairGeometry kGeometry{.focal_px = 369.504,
                                       .principal_x_px = 640.0,
                                       .principal_y_px = 480.0,
                                       .baseline_m = 0.20};

TEST(StereoDepthReturns, ConfidentDepthFollowsTheDisparityErrorLaw) {
  StereoDepthReturnsConfig config;
  config.disparity_error_px = 0.45;
  config.allowed_depth_error_m = 0.25;
  const double depth_m = stereoConfidentDepthM(kGeometry, config);
  EXPECT_NEAR(depth_m, 6.41, 0.01);
  // At that depth the error law gives exactly the allowed error.
  EXPECT_NEAR(depth_m * depth_m * config.disparity_error_px /
                  (kGeometry.baseline_m * kGeometry.focal_px),
              config.allowed_depth_error_m, 1.0e-9);
}

TEST(StereoDepthReturns, APixelWithoutConfidentDepthYieldsNoReturn) {
  StereoDepthReturnsConfig config;
  config.pixel_stride = 1U;
  const std::size_t width = 4U;
  const std::size_t height = 1U;
  // 4 m, no match, 12 m (beyond the confident depth), 0.1 m (inside the
  // minimum range).
  const auto sixteenths = [](const double depth_m) {
    return static_cast<std::int16_t>(
        std::lround(16.0 * kGeometry.focal_px * kGeometry.baseline_m / depth_m));
  };
  const std::vector<std::int16_t> disparity{sixteenths(4.0), -16, sixteenths(12.0),
                                            sixteenths(0.1)};

  const std::vector<Point3> returns =
      stereoDepthReturns(disparity, width, height, kGeometry, config);

  ASSERT_EQ(returns.size(), 1U);
  // Pixel (0, 0) lies left of and above the principal point: forward, to the
  // left and up in the camera's forward-left-up frame.
  EXPECT_NEAR(returns[0].x, 4.0, 0.01);
  EXPECT_NEAR(returns[0].y, 640.0 * 4.0 / kGeometry.focal_px, 0.02);
  EXPECT_NEAR(returns[0].z, 480.0 * 4.0 / kGeometry.focal_px, 0.02);
}

TEST(StereoDepthReturns, TheStrideThinsTheReturns) {
  StereoDepthReturnsConfig config;
  config.pixel_stride = 4U;
  const std::vector<std::int16_t> disparity(
      16U * 8U, static_cast<std::int16_t>(16.0 * kGeometry.focal_px *
                                          kGeometry.baseline_m / 3.0));
  EXPECT_EQ(stereoDepthReturns(disparity, 16U, 8U, kGeometry, config).size(), 4U * 2U);
  EXPECT_TRUE(stereoDepthReturns(disparity, 16U, 7U, kGeometry, config).empty());
}

} // namespace
} // namespace drone_city_nav
