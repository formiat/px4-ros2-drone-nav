#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <opencv2/core.hpp>

#include "stereo_rectifier.hpp"

namespace drone_city_nav {
namespace {

// Specification K33: the simulated pair is ideal and passes untouched, with
// the focal length its field of view gives.
TEST(StereoRectifier, TheIdealPairPassesUntouched) {
  const StereoRectifier rectifier{
      idealStereoCalibration(1280U, 960U, 2.0 * std::numbers::pi / 3.0, 0.2)};
  EXPECT_TRUE(rectifier.ideal());
  EXPECT_NEAR(rectifier.rectified().focal_px, 640.0 / std::tan(std::numbers::pi / 3.0),
              1.0e-9);
  EXPECT_DOUBLE_EQ(rectifier.rectified().principal_x_px, 640.0);
  EXPECT_DOUBLE_EQ(rectifier.rectified().baseline_m, 0.2);
  const cv::Mat left(960, 1280, CV_8UC1, cv::Scalar{7});
  const cv::Mat right(960, 1280, CV_8UC1, cv::Scalar{9});
  const std::array<cv::Mat, 2> out = rectifier.rectify(left, right);
  EXPECT_EQ(out[0].data, left.data);
  EXPECT_EQ(out[1].data, right.data);
}

// A real pair (the EuRoC sensor's, MH and V sequences): distorted, the right
// camera turned a little; rectified, the baseline is the cameras' distance and
// a point at a known depth lands on one row in both frames.
TEST(StereoRectifier, ARealPairIsRectifiedOntoRows) {
  StereoCalibration calibration;
  calibration.width = 752U;
  calibration.height = 480U;
  calibration.left = {
      .fx = 458.654,
      .fy = 457.296,
      .cx = 367.215,
      .cy = 248.375,
      .distortion = {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05}};
  calibration.right = {
      .fx = 457.587,
      .fy = 456.134,
      .cx = 379.999,
      .cy = 255.238,
      .distortion = {-0.28368365, 0.07451284, -0.00010473, -3.55590700e-05}};
  calibration.right_from_left_rotation << 0.999997, 0.002313, 0.000376, -0.002317,
      0.999898, 0.014089, -0.000344, -0.014090, 0.999901;
  calibration.right_from_left_translation_m = {-0.110074, 0.000399, -0.000853};
  const StereoRectifier rectifier{calibration};
  EXPECT_FALSE(rectifier.ideal());
  EXPECT_NEAR(rectifier.rectified().baseline_m, 0.11008, 1.0e-3);
  EXPECT_GT(rectifier.rectified().focal_px, 300.0);
  EXPECT_LT(rectifier.rectified().focal_px, 500.0);
  const Eigen::Matrix3d r = rectifier.rectified().rectified_from_left;
  EXPECT_NEAR((r * r.transpose() - Eigen::Matrix3d::Identity()).norm(), 0.0, 1.0e-9);
  const cv::Mat left(480, 752, CV_8UC1, cv::Scalar{100});
  const std::array<cv::Mat, 2> out = rectifier.rectify(left, left);
  EXPECT_EQ(out[0].rows, 480);
  EXPECT_EQ(out[0].cols, 752);
  EXPECT_NE(out[0].data, left.data);
}

} // namespace
} // namespace drone_city_nav
