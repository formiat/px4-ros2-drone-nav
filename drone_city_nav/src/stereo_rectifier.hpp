#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <opencv2/core/mat.hpp>

namespace drone_city_nav {

// Roadmap item 25 (specification K33): the stereo pair as its calibration
// states it. A camera is a pinhole with radial-tangential distortion
// (k1, k2, p1, p2); the right camera's frame is the left one's turned by
// `right_from_left_rotation` and moved by `right_from_left_translation_m`
// (x_right = R x_left + t, camera frames x right, y down, z forward). The
// simulated pair is the ideal one: no distortion, the same intrinsics, the
// right camera the baseline along the left one's x axis.
struct PinholeCamera {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  std::array<double, 4> distortion{};
};

struct StereoCalibration {
  std::size_t width{0U};
  std::size_t height{0U};
  PinholeCamera left{};
  PinholeCamera right{};
  Eigen::Matrix3d right_from_left_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d right_from_left_translation_m{Eigen::Vector3d::Zero()};
};

// The pair rectified: one focal length and principal point for both cameras,
// rows aligned, the right camera the baseline along the rectified left
// camera's x axis. `rectified_from_left` turns a direction of the left
// camera's frame into the rectified one's.
struct RectifiedStereo {
  double focal_px{0.0};
  double principal_x_px{0.0};
  double principal_y_px{0.0};
  double baseline_m{0.0};
  Eigen::Matrix3d rectified_from_left{Eigen::Matrix3d::Identity()};
};

// Frames rectified before any matching. An ideal pair is passed through
// untouched, so the simulated flights see exactly the frames they saw.
class StereoRectifier final {
public:
  explicit StereoRectifier(const StereoCalibration& calibration);

  [[nodiscard]] const RectifiedStereo& rectified() const noexcept {
    return rectified_;
  }

  [[nodiscard]] bool ideal() const noexcept {
    return ideal_;
  }

  // The rectified pair; the inputs themselves when the pair is ideal.
  [[nodiscard]] std::array<cv::Mat, 2> rectify(const cv::Mat& left,
                                               const cv::Mat& right) const;

private:
  RectifiedStereo rectified_{};
  bool ideal_{true};
  cv::Mat left_map_x_;
  cv::Mat left_map_y_;
  cv::Mat right_map_x_;
  cv::Mat right_map_y_;
};

// The ideal pair of the simulation: a horizontal field of view over the
// width, the principal point at the centre, the baseline along x.
[[nodiscard]] StereoCalibration idealStereoCalibration(std::size_t width,
                                                       std::size_t height,
                                                       double horizontal_fov_rad,
                                                       double baseline_m);

} // namespace drone_city_nav
