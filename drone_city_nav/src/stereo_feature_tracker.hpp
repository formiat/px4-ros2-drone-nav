#pragma once

#include "drone_city_nav/visual_inertial_odometry.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

// The image side of the visual-inertial estimator: corners of the left image
// followed from frame to frame and matched into the right image of the same
// frame, handed to the filter as normalized coordinates under stable ids.
// OpenCV lives here and nowhere in the estimator's library. The pair is
// rectified and ideal, as the simulated cameras are: one focal length, the
// principal point at the image centre, the right camera displaced along the
// image x axis.

namespace cv {
class Mat;
} // namespace cv

namespace drone_city_nav {

struct StereoFeatureTrackerConfig {
  std::size_t image_width{1280U};
  std::size_t image_height{960U};
  double horizontal_fov_rad{2.0943951023931953};
  // Corners kept alive, spread over a grid of cells so one textured wall does
  // not take them all.
  std::size_t maximum_features{200U};
  std::size_t grid_columns{8U};
  std::size_t grid_rows{6U};
  double minimum_distance_px{20.0};
  double corner_quality{0.01};
  int pyramid_levels{4};
  int window_px{21};
  // A track is kept when following it back lands within this distance of
  // where it started, in either direction of time or between the cameras.
  double maximum_round_trip_error_px{1.0};
  // A rectified pair keeps a point on its row.
  double maximum_row_error_px{1.5};
  double minimum_disparity_px{1.0};
  double maximum_disparity_px{256.0};
  // Frame-to-frame outliers are removed against the epipolar geometry.
  double epipolar_ransac_threshold_px{1.0};
};

struct StereoFeatureTrackerReport {
  std::size_t followed{0U};
  std::size_t lost_in_time{0U};
  std::size_t lost_epipolar{0U};
  std::size_t lost_in_stereo{0U};
  std::size_t added{0U};
};

class StereoFeatureTracker {
public:
  explicit StereoFeatureTracker(const StereoFeatureTrackerConfig& config);
  ~StereoFeatureTracker();
  StereoFeatureTracker(const StereoFeatureTracker&) = delete;
  StereoFeatureTracker& operator=(const StereoFeatureTracker&) = delete;

  // One frame of the pair, grey. The rotation takes a direction in the
  // previous left camera frame into the current one, from the gyroscope: a
  // turn of ten degrees between frames moves a corner sixty pixels, past what
  // the pyramid follows from a standing start.
  [[nodiscard]] std::vector<StereoFeatureObservation>
  track(const cv::Mat& left, const cv::Mat& right,
        const std::optional<Eigen::Matrix3d>& previous_to_current_rotation);

  [[nodiscard]] const StereoFeatureTrackerReport& report() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace drone_city_nav
