#pragma once

#include "drone_city_nav/stereo_depth_returns.hpp"
#include "drone_city_nav/stereo_lit_range.hpp"
#include "drone_city_nav/types.hpp"

#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <unordered_map>
#include <vector>

// The stereo depth node's helpers (stereo_depth_node.cpp): the frame as grey,
// the noise mask over the disparity, the lone hits' rejection and the report's
// brightness text.
namespace drone_city_nav::stereo_depth_node_internal {

inline constexpr int kSignalWindowPx{9};
inline constexpr std::size_t kLaplacianBins{1024U};
inline constexpr int kNoiseSampleStride{4};
inline constexpr double kSignalOverNoise{2.0};
inline constexpr std::size_t kBrightnessBins{16U};
inline constexpr float kBrightnessBinLevels{16.0F};
inline constexpr std::size_t kMinimumBinSamples{50U};
// A noiseless render still quantizes: never trust less than a level.
inline constexpr double kMinimumNoiseLevels{1.0};

inline constexpr double kLoneHitVoxelM{0.25};
inline constexpr std::uint32_t kLoneHitVoxelLeastHits{2U};

[[nodiscard]] inline std::size_t brightnessBin(const float mean) noexcept {
  return std::min(kBrightnessBins - 1U, static_cast<std::size_t>(std::max(0.0F, mean) /
                                                                 kBrightnessBinLevels));
}

// A match stands where the local standard deviation clears its brightness's noise
// twofold (Immerkaer's operator by sixteenths of the range, A12; r764, r1476).
inline double rejectMatchesWithinNoise(const cv::Mat& grey, cv::Mat& disparity) {
  cv::Mat laplacian;
  const cv::Mat kernel = (cv::Mat_<float>(3, 3) << 1.0F, -2.0F, 1.0F, -2.0F, 4.0F,
                          -2.0F, 1.0F, -2.0F, 1.0F);
  cv::filter2D(grey, laplacian, CV_32F, kernel);
  cv::Mat grey_float;
  grey.convertTo(grey_float, CV_32F);
  cv::Mat mean;
  cv::Mat mean_square;
  const cv::Size window{kSignalWindowPx, kSignalWindowPx};
  cv::boxFilter(grey_float, mean, CV_32F, window);
  cv::boxFilter(grey_float.mul(grey_float), mean_square, CV_32F, window);
  // The operator's response is six noise deviations on a flat patch and far more on
  // an edge; its median is the flat patches' (r765: the mean read texture as noise).
  std::vector<std::size_t> histograms(kBrightnessBins * kLaplacianBins, 0U);
  std::array<std::size_t, kBrightnessBins> samples{};
  for (int row = 1; row + 1 < laplacian.rows; row += kNoiseSampleStride) {
    const float* const values = laplacian.ptr<float>(row);
    const float* const means = mean.ptr<float>(row);
    for (int column = 1; column + 1 < laplacian.cols; column += kNoiseSampleStride) {
      const std::size_t bin = brightnessBin(means[column]);
      ++histograms[bin * kLaplacianBins +
                   std::min<std::size_t>(
                       kLaplacianBins - 1U,
                       static_cast<std::size_t>(std::abs(values[column])))];
      ++samples[bin];
    }
  }
  const auto noiseOf = [&](const std::size_t first_bin, const std::size_t last_bin) {
    std::size_t count{0U};
    for (std::size_t bin = first_bin; bin <= last_bin; ++bin) {
      count += samples[bin];
    }
    std::size_t below{0U};
    for (std::size_t response = 0U; response < kLaplacianBins; ++response) {
      for (std::size_t bin = first_bin; bin <= last_bin; ++bin) {
        below += histograms[bin * kLaplacianBins + response];
      }
      if (2U * below >= count) {
        return 1.4826 * (static_cast<double>(response) + 0.5) / 6.0;
      }
    }
    return 0.0;
  };
  const double noise = noiseOf(0U, kBrightnessBins - 1U);
  // A brightness too rare in the frame to estimate takes the noise of the
  // next darker one (the frame's own for the darkest).
  std::array<float, kBrightnessBins> minimum_variance{};
  double carried = noise;
  for (std::size_t bin = 0U; bin < kBrightnessBins; ++bin) {
    if (samples[bin] >= kMinimumBinSamples) {
      carried = noiseOf(bin, bin);
    }
    minimum_variance[bin] = static_cast<float>(
        std::pow(kSignalOverNoise * std::max(carried, kMinimumNoiseLevels), 2.0));
  }
  for (int row = 0; row < disparity.rows; ++row) {
    const float* const means = mean.ptr<float>(row);
    const float* const squares = mean_square.ptr<float>(row);
    std::int16_t* const values = disparity.ptr<std::int16_t>(row);
    for (int column = 0; column < disparity.cols; ++column) {
      if (squares[column] - means[column] * means[column] <
          minimum_variance[brightnessBin(means[column])]) {
        values[column] = std::int16_t{-16};
      }
    }
  }
  return noise;
}

// A hit alone in its voxel is no surface: the latest scan carried every hit, and
// the matcher's strays (A12) were 98 percent of the hits blocking a route (r1481,
// 159 s of holds in 380); this rule brings them from 33 a frame to the noiseless 9,
// keeping 99 percent of the hits within a voxel of the truth (r1476, r782). A
// surface within the confident depth puts 19 rays in a voxel at 5.2 m.
inline void rejectLoneHits(std::vector<StereoDepthReturn>& returns) {
  std::unordered_map<std::uint64_t, std::uint32_t> hits_by_voxel;
  const auto voxel = [](const Point3& point) {
    const auto coordinate = [](const double value) {
      return static_cast<std::uint64_t>(
                 static_cast<std::int64_t>(std::floor(value / kLoneHitVoxelM)) +
                 (1LL << 20U)) &
             ((1ULL << 21U) - 1ULL);
    };
    return (coordinate(point.x) << 42U) | (coordinate(point.y) << 21U) |
           coordinate(point.z);
  };
  for (const StereoDepthReturn& stereo_return : returns) {
    if (stereo_return.hit) {
      ++hits_by_voxel[voxel(stereo_return.point)];
    }
  }
  std::erase_if(returns, [&](const StereoDepthReturn& stereo_return) {
    return stereo_return.hit &&
           hits_by_voxel[voxel(stereo_return.point)] < kLoneHitVoxelLeastHits;
  });
}

[[nodiscard]] inline cv::Mat grey(const sensor_msgs::msg::Image& image) {
  // The matcher reads the buffer only; OpenCV has no const view of one.
  auto* const data = const_cast<std::uint8_t*>(image.data.data());
  if (image.encoding == "mono8") {
    return cv::Mat{static_cast<int>(image.height), static_cast<int>(image.width),
                   CV_8UC1, data, image.step};
  }
  const cv::Mat rgb{static_cast<int>(image.height), static_cast<int>(image.width),
                    CV_8UC3, data, image.step};
  cv::Mat result;
  cv::cvtColor(rgb, result, cv::COLOR_RGB2GRAY);
  return result;
}

[[nodiscard]] inline std::string
brightnessByMetreText(const std::array<double, LitRangeTracker::kMetres>& brightness) {
  std::string text;
  for (std::size_t metre = 0U; metre < brightness.size(); ++metre) {
    text +=
        (metre == 0U ? "" : "/") +
        std::to_string(brightness[metre] < 0.0 ? -1L : std::lround(brightness[metre]));
  }
  return text;
}

} // namespace drone_city_nav::stereo_depth_node_internal
