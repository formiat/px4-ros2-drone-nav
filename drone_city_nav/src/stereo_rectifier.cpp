#include "stereo_rectifier.hpp"

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] cv::Matx33d cameraMatrix(const PinholeCamera& camera) {
  return cv::Matx33d{camera.fx, 0.0, camera.cx, 0.0, camera.fy,
                     camera.cy, 0.0, 0.0,       1.0};
}

[[nodiscard]] bool sameCamera(const PinholeCamera& a, const PinholeCamera& b) noexcept {
  return a.fx == b.fx && a.fy == b.fy && a.cx == b.cx && a.cy == b.cy &&
         a.distortion == b.distortion;
}

[[nodiscard]] bool undistorted(const PinholeCamera& camera) noexcept {
  return camera.distortion == std::array<double, 4>{};
}

} // namespace

StereoCalibration idealStereoCalibration(const std::size_t width,
                                         const std::size_t height,
                                         const double horizontal_fov_rad,
                                         const double baseline_m) {
  const double focal_px =
      0.5 * static_cast<double>(width) / std::tan(0.5 * horizontal_fov_rad);
  // The principal point at the frame's centre, in coordinates where a
  // pixel's centre is its integer index: half a pixel in from width / 2.
  const PinholeCamera camera{.fx = focal_px,
                             .fy = focal_px,
                             .cx = 0.5 * static_cast<double>(width) - 0.5,
                             .cy = 0.5 * static_cast<double>(height) - 0.5};
  return StereoCalibration{.width = width,
                           .height = height,
                           .left = camera,
                           .right = camera,
                           .right_from_left_translation_m = {-baseline_m, 0.0, 0.0}};
}

StereoRectifier::StereoRectifier(const StereoCalibration& calibration) {
  const Eigen::Vector3d& t = calibration.right_from_left_translation_m;
  if (calibration.width == 0U || calibration.height == 0U ||
      !(calibration.left.fx > 0.0) || !(calibration.left.fy > 0.0) ||
      !(calibration.right.fx > 0.0) || !(calibration.right.fy > 0.0) ||
      !(t.norm() > 0.0)) {
    throw std::invalid_argument{"invalid stereo calibration"};
  }
  // The ideal pair: one undistorted pinhole twice, square pixels, the right
  // camera straight along the left one's x axis.
  ideal_ = sameCamera(calibration.left, calibration.right) &&
           undistorted(calibration.left) &&
           calibration.left.fx == calibration.left.fy &&
           calibration.right_from_left_rotation.isIdentity(1.0e-12) && t.y() == 0.0 &&
           t.z() == 0.0 && t.x() < 0.0;
  if (ideal_) {
    rectified_ = RectifiedStereo{.focal_px = calibration.left.fx,
                                 .principal_x_px = calibration.left.cx,
                                 .principal_y_px = calibration.left.cy,
                                 .baseline_m = -t.x()};
    return;
  }
  const cv::Size size{static_cast<int>(calibration.width),
                      static_cast<int>(calibration.height)};
  cv::Matx33d rotation;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rotation(row, column) = calibration.right_from_left_rotation(row, column);
    }
  }
  const cv::Vec3d translation{t.x(), t.y(), t.z()};
  const cv::Matx33d left_matrix = cameraMatrix(calibration.left);
  const cv::Matx33d right_matrix = cameraMatrix(calibration.right);
  const cv::Vec4d left_distortion{
      calibration.left.distortion[0], calibration.left.distortion[1],
      calibration.left.distortion[2], calibration.left.distortion[3]};
  const cv::Vec4d right_distortion{
      calibration.right.distortion[0], calibration.right.distortion[1],
      calibration.right.distortion[2], calibration.right.distortion[3]};
  cv::Mat left_rotation;
  cv::Mat right_rotation;
  cv::Mat left_projection;
  cv::Mat right_projection;
  cv::Mat disparity_to_depth;
  // Zero disparity at infinity, and only valid pixels kept (alpha 0): the
  // rectified frames carry no black border a corner detector would follow.
  cv::stereoRectify(left_matrix, left_distortion, right_matrix, right_distortion, size,
                    rotation, translation, left_rotation, right_rotation,
                    left_projection, right_projection, disparity_to_depth,
                    cv::CALIB_ZERO_DISPARITY, 0.0, size);
  cv::initUndistortRectifyMap(left_matrix, left_distortion, left_rotation,
                              left_projection, size, CV_32FC1, left_map_x_,
                              left_map_y_);
  cv::initUndistortRectifyMap(right_matrix, right_distortion, right_rotation,
                              right_projection, size, CV_32FC1, right_map_x_,
                              right_map_y_);
  rectified_.focal_px = left_projection.at<double>(0, 0);
  rectified_.principal_x_px = left_projection.at<double>(0, 2);
  rectified_.principal_y_px = left_projection.at<double>(1, 2);
  rectified_.baseline_m =
      -right_projection.at<double>(0, 3) / right_projection.at<double>(0, 0);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rectified_.rectified_from_left(row, column) =
          left_rotation.at<double>(row, column);
    }
  }
}

std::array<cv::Mat, 2> StereoRectifier::rectify(const cv::Mat& left,
                                                const cv::Mat& right) const {
  if (ideal_) {
    return {left, right};
  }
  std::array<cv::Mat, 2> rectified;
  cv::remap(left, rectified[0], left_map_x_, left_map_y_, cv::INTER_LINEAR);
  cv::remap(right, rectified[1], right_map_x_, right_map_y_, cv::INTER_LINEAR);
  return rectified;
}

} // namespace drone_city_nav
