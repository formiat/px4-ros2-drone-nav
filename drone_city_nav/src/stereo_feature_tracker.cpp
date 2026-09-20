#include "stereo_feature_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace drone_city_nav {
namespace {

template<typename T> void keep(std::vector<T>& values, const std::vector<bool>& kept) {
  std::size_t out = 0U;
  for (std::size_t i = 0U; i < values.size(); ++i) {
    if (kept[i]) {
      values[out++] = values[i];
    }
  }
  values.resize(out);
}

} // namespace

struct StereoFeatureTracker::Impl {
  explicit Impl(const StereoFeatureTrackerConfig& configuration)
      : config_{configuration},
        focal_px_{0.5 * static_cast<double>(configuration.image_width) /
                  std::tan(0.5 * configuration.horizontal_fov_rad)},
        centre_px_{0.5F * static_cast<float>(configuration.image_width),
                   0.5F * static_cast<float>(configuration.image_height)} {
  }

  [[nodiscard]] std::vector<StereoFeatureObservation>
  track(const cv::Mat& left, const cv::Mat& right,
        const std::optional<Eigen::Matrix3d>& previous_to_current_rotation);

  StereoFeatureTrackerConfig config_;
  double focal_px_{1.0};
  cv::Point2f centre_px_;
  std::vector<cv::Mat> previous_left_pyramid_;
  std::vector<cv::Point2f> points_;
  std::vector<float> disparities_;
  std::vector<std::uint64_t> ids_;
  std::uint64_t next_id_{1U};
  StereoFeatureTrackerReport report_;
};

StereoFeatureTracker::StereoFeatureTracker(const StereoFeatureTrackerConfig& config)
    : impl_{std::make_unique<Impl>(config)} {
}

StereoFeatureTracker::~StereoFeatureTracker() = default;

const StereoFeatureTrackerReport& StereoFeatureTracker::report() const noexcept {
  return impl_->report_;
}

std::vector<StereoFeatureObservation> StereoFeatureTracker::track(
    const cv::Mat& left, const cv::Mat& right,
    const std::optional<Eigen::Matrix3d>& previous_to_current_rotation) {
  return impl_->track(left, right, previous_to_current_rotation);
}

std::vector<StereoFeatureObservation> StereoFeatureTracker::Impl::track(
    const cv::Mat& left, const cv::Mat& right,
    const std::optional<Eigen::Matrix3d>& previous_to_current_rotation) {
  report_ = {};
  const cv::Size window{config_.window_px, config_.window_px};
  const cv::TermCriteria criteria{cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                                  0.01};
  std::vector<cv::Mat> left_pyramid;
  std::vector<cv::Mat> right_pyramid;
  cv::buildOpticalFlowPyramid(left, left_pyramid, window, config_.pyramid_levels);
  cv::buildOpticalFlowPyramid(right, right_pyramid, window, config_.pyramid_levels);
  const auto inside = [this](const cv::Point2f& point) {
    return point.x >= 1.0F && point.y >= 1.0F &&
           point.x < static_cast<float>(config_.image_width) - 1.0F &&
           point.y < static_cast<float>(config_.image_height) - 1.0F;
  };
  // Follows `from` into the other pyramid starting at `guess`, and back; a
  // point survives when both passes converge and the way back ends where it
  // started.
  const auto follow = [&](const std::vector<cv::Mat>& from_pyramid,
                          const std::vector<cv::Mat>& to_pyramid,
                          const std::vector<cv::Point2f>& from,
                          std::vector<cv::Point2f>& to) {
    std::vector<bool> kept(from.size(), false);
    if (from.empty()) {
      return kept;
    }
    std::vector<unsigned char> forward_status;
    std::vector<unsigned char> backward_status;
    std::vector<float> error;
    std::vector<cv::Point2f> back = from;
    cv::calcOpticalFlowPyrLK(from_pyramid, to_pyramid, from, to, forward_status, error,
                             window, config_.pyramid_levels, criteria,
                             cv::OPTFLOW_USE_INITIAL_FLOW);
    cv::calcOpticalFlowPyrLK(to_pyramid, from_pyramid, to, back, backward_status, error,
                             window, config_.pyramid_levels, criteria,
                             cv::OPTFLOW_USE_INITIAL_FLOW);
    for (std::size_t i = 0U; i < from.size(); ++i) {
      const cv::Point2f round_trip = back[i] - from[i];
      kept[i] =
          forward_status[i] != 0U && backward_status[i] != 0U && inside(to[i]) &&
          std::hypot(round_trip.x, round_trip.y) <= config_.maximum_round_trip_error_px;
    }
    return kept;
  };
  const auto keepAll = [this](const std::vector<bool>& kept) {
    keep(points_, kept);
    keep(disparities_, kept);
    keep(ids_, kept);
  };

  if (!previous_left_pyramid_.empty() && !points_.empty()) {
    std::vector<cv::Point2f> guess = points_;
    if (previous_to_current_rotation.has_value()) {
      for (cv::Point2f& point : guess) {
        const Eigen::Vector3d turned =
            *previous_to_current_rotation *
            Eigen::Vector3d{(point.x - centre_px_.x) / focal_px_,
                            (point.y - centre_px_.y) / focal_px_, 1.0};
        if (turned.z() > 1.0e-3) {
          const cv::Point2f predicted{
              static_cast<float>(focal_px_ * turned.x() / turned.z()) + centre_px_.x,
              static_cast<float>(focal_px_ * turned.y() / turned.z()) + centre_px_.y};
          if (inside(predicted)) {
            point = predicted;
          }
        }
      }
    }
    const std::vector<cv::Point2f> previous = points_;
    std::vector<bool> kept =
        follow(previous_left_pyramid_, left_pyramid, previous, guess);
    report_.lost_in_time =
        static_cast<std::size_t>(std::count(kept.begin(), kept.end(), false));
    std::vector<cv::Point2f> from;
    std::vector<cv::Point2f> to;
    for (std::size_t i = 0U; i < kept.size(); ++i) {
      if (kept[i]) {
        from.push_back(previous[i]);
        to.push_back(guess[i]);
      }
    }
    points_ = guess;
    keepAll(kept);
    if (from.size() >= 12U) {
      std::vector<unsigned char> inliers;
      cv::findFundamentalMat(from, to, cv::FM_RANSAC,
                             config_.epipolar_ransac_threshold_px, 0.99, inliers);
      if (inliers.size() == from.size()) {
        std::vector<bool> epipolar(from.size());
        for (std::size_t i = 0U; i < from.size(); ++i) {
          epipolar[i] = inliers[i] != 0U;
        }
        report_.lost_epipolar = static_cast<std::size_t>(
            std::count(epipolar.begin(), epipolar.end(), false));
        keepAll(epipolar);
      }
    }
  } else {
    points_.clear();
    disparities_.clear();
    ids_.clear();
  }
  report_.followed = points_.size();

  // New corners where the grid has room, away from the ones already held.
  if (points_.size() < config_.maximum_features) {
    cv::Mat mask{left.size(), CV_8UC1, cv::Scalar{255}};
    const std::size_t cells = config_.grid_columns * config_.grid_rows;
    const std::size_t per_cell = (config_.maximum_features + cells - 1U) / cells;
    const float cell_width = static_cast<float>(config_.image_width) /
                             static_cast<float>(config_.grid_columns);
    const float cell_height = static_cast<float>(config_.image_height) /
                              static_cast<float>(config_.grid_rows);
    std::vector<std::size_t> occupancy(cells, 0U);
    const auto cellOf = [&](const cv::Point2f& point) {
      const auto column = std::min<std::size_t>(
          config_.grid_columns - 1U, static_cast<std::size_t>(point.x / cell_width));
      const auto row = std::min<std::size_t>(
          config_.grid_rows - 1U, static_cast<std::size_t>(point.y / cell_height));
      return row * config_.grid_columns + column;
    };
    for (const cv::Point2f& point : points_) {
      ++occupancy[cellOf(point)];
      cv::circle(mask, point, static_cast<int>(config_.minimum_distance_px),
                 cv::Scalar{0}, cv::FILLED);
    }
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(left, corners,
                            static_cast<int>(2U * config_.maximum_features),
                            config_.corner_quality, config_.minimum_distance_px, mask);
    for (const cv::Point2f& corner : corners) {
      if (points_.size() >= config_.maximum_features) {
        break;
      }
      std::size_t& held = occupancy[cellOf(corner)];
      if (held >= per_cell || !inside(corner)) {
        continue;
      }
      ++held;
      points_.push_back(corner);
      disparities_.push_back(0.0F);
      ids_.push_back(next_id_++);
      ++report_.added;
    }
  }

  // Into the right image along the row, from the disparity last seen.
  std::vector<cv::Point2f> right_points = points_;
  for (std::size_t i = 0U; i < right_points.size(); ++i) {
    right_points[i].x = std::max(1.0F, right_points[i].x - disparities_[i]);
  }
  std::vector<bool> kept = follow(left_pyramid, right_pyramid, points_, right_points);
  for (std::size_t i = 0U; i < kept.size(); ++i) {
    const float disparity = points_[i].x - right_points[i].x;
    kept[i] =
        kept[i] &&
        std::abs(points_[i].y - right_points[i].y) <= config_.maximum_row_error_px &&
        disparity >= config_.minimum_disparity_px &&
        disparity <= config_.maximum_disparity_px;
    disparities_[i] = disparity;
  }
  report_.lost_in_stereo =
      static_cast<std::size_t>(std::count(kept.begin(), kept.end(), false));
  keep(right_points, kept);
  keepAll(kept);

  std::vector<StereoFeatureObservation> observations;
  observations.reserve(points_.size());
  for (std::size_t i = 0U; i < points_.size(); ++i) {
    observations.push_back(StereoFeatureObservation{
        .id = ids_[i],
        .left = {(points_[i].x - centre_px_.x) / focal_px_,
                 (points_[i].y - centre_px_.y) / focal_px_},
        .right = {(right_points[i].x - centre_px_.x) / focal_px_,
                  (right_points[i].y - centre_px_.y) / focal_px_}});
  }
  previous_left_pyramid_ = std::move(left_pyramid);
  return observations;
}

} // namespace drone_city_nav
