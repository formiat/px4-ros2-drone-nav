// stereo_depth_accuracy DIR: the stereo pair's depth against the simulator's,
// on a record of pairs and the evaluation depth camera
// (tools/stereo_record_frames.py, the camera's gain applied by
// tools/stereo_gain_copy.py). The matching is the depth node's: semi-global
// matching far at full resolution and near at half, merged, and the matches
// within the image's noise rejected (stereo_depth_node.cpp). Printed by
// depth band: the share of true pixels matched, the share of matches within
// 0.25 m, and the depth and disparity errors at the 50th and 90th
// percentiles; then the confident depth the largest disparity error at the
// 90th percentile from 2 to 8 m gives (specification K7).
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <map>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr int kWidth{1280};
constexpr int kHeight{960};
constexpr double kBaselineM{0.20};
// The pair's 120 degrees across 1280 pixels.
const double kFocalPx = (kWidth / 2.0) / std::tan(std::numbers::pi / 3.0);
constexpr double kAllowedDepthErrorM{0.25};
constexpr int kFarDisparities{64};
constexpr int kNearDisparities{128};
constexpr int kBlockSize{5};

std::vector<char> npyPayload(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  std::vector<char> bytes((std::istreambuf_iterator<char>(stream)),
                          std::istreambuf_iterator<char>());
  const auto header =
      static_cast<unsigned>(static_cast<unsigned char>(bytes.at(8))) |
      (static_cast<unsigned>(static_cast<unsigned char>(bytes.at(9))) << 8U);
  return {bytes.begin() + 10 + header, bytes.end()};
}

float halfToFloat(const std::uint16_t half) {
  const int sign = (half >> 15U) & 1U;
  const int exponent = (half >> 10U) & 31U;
  const int mantissa = half & 1023U;
  const float value =
      exponent == 0    ? std::ldexp(static_cast<float>(mantissa), -24)
      : exponent == 31 ? INFINITY
                       : std::ldexp(static_cast<float>(mantissa | 1024), exponent - 25);
  return sign != 0 ? -value : value;
}

// A copy of StereoDepthNode::rejectMatchesWithinNoise.
void rejectMatchesWithinNoise(const cv::Mat& grey, cv::Mat& disparity) {
  constexpr int kSignalWindowPx{9};
  constexpr std::size_t kLaplacianBins{1024U};
  constexpr int kNoiseSampleStride{4};
  constexpr double kSignalOverNoise{3.0};
  constexpr double kMinimumNoiseLevels{1.0};
  cv::Mat laplacian;
  const cv::Mat kernel = (cv::Mat_<float>(3, 3) << 1.0F, -2.0F, 1.0F, -2.0F, 4.0F,
                          -2.0F, 1.0F, -2.0F, 1.0F);
  cv::filter2D(grey, laplacian, CV_32F, kernel);
  std::array<std::size_t, kLaplacianBins> histogram{};
  std::size_t samples{0U};
  for (int row = 1; row + 1 < laplacian.rows; row += kNoiseSampleStride) {
    const float* const values = laplacian.ptr<float>(row);
    for (int column = 1; column + 1 < laplacian.cols; column += kNoiseSampleStride) {
      ++histogram[std::min<std::size_t>(
          kLaplacianBins - 1U, static_cast<std::size_t>(std::abs(values[column])))];
      ++samples;
    }
  }
  std::size_t below{0U};
  double median_response{0.0};
  for (std::size_t bin = 0U; bin < kLaplacianBins; ++bin) {
    below += histogram[bin];
    if (2U * below >= samples) {
      median_response = static_cast<double>(bin) + 0.5;
      break;
    }
  }
  const double noise = 1.4826 * median_response / 6.0;
  cv::Mat grey_float;
  grey.convertTo(grey_float, CV_32F);
  cv::Mat mean;
  cv::Mat mean_square;
  const cv::Size window{kSignalWindowPx, kSignalWindowPx};
  cv::boxFilter(grey_float, mean, CV_32F, window);
  cv::boxFilter(grey_float.mul(grey_float), mean_square, CV_32F, window);
  const auto minimum_variance = static_cast<float>(
      std::pow(kSignalOverNoise * std::max(noise, kMinimumNoiseLevels), 2.0));
  for (int row = 0; row < disparity.rows; ++row) {
    const float* const means = mean.ptr<float>(row);
    const float* const squares = mean_square.ptr<float>(row);
    auto* const values = disparity.ptr<std::int16_t>(row);
    for (int column = 0; column < disparity.cols; ++column) {
      if (squares[column] - means[column] * means[column] < minimum_variance) {
        values[column] = std::int16_t{-16};
      }
    }
  }
}

struct Band {
  long truth{0};
  long matched{0};
  long within{0};
  std::vector<float> depth_error;
  std::vector<float> disparity_error;
};

float percentile(std::vector<float>& values, const double share) {
  if (values.empty()) {
    return 0.0F;
  }
  std::ranges::sort(values);
  return values[static_cast<std::size_t>(share *
                                         static_cast<double>(values.size() - 1U))];
}

} // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: stereo_depth_accuracy DIR\n");
    return 2;
  }
  const std::string directory = argv[1];
  std::map<std::string, std::map<std::string, std::string>> frames;
  std::ifstream index(directory + "/index.csv");
  std::string line;
  while (std::getline(index, line)) {
    std::stringstream fields(line);
    std::string name;
    std::string stamp;
    std::string width;
    std::string height;
    std::string file;
    std::getline(fields, name, ',');
    std::getline(fields, stamp, ',');
    std::getline(fields, width, ',');
    std::getline(fields, height, ',');
    std::getline(fields, file, ',');
    frames[stamp][name] = file;
  }
  const auto matcher = [](const int disparities) {
    return cv::StereoSGBM::create(0, disparities, kBlockSize,
                                  8 * kBlockSize * kBlockSize,
                                  32 * kBlockSize * kBlockSize, 1, 31, 10, 100, 2,
                                  cv::StereoSGBM::MODE_SGBM_3WAY);
  };
  const cv::Ptr<cv::StereoSGBM> far_matcher = matcher(kFarDisparities);
  const cv::Ptr<cv::StereoSGBM> near_matcher = matcher(kNearDisparities);
  const std::vector<double> edges{0.3, 1, 2, 4, 6, 8, 10, 14, 20, 40};
  std::vector<Band> bands(edges.size() - 1U);
  int used{0};
  int triplets{0};
  for (auto& [stamp, files] : frames) {
    if (files.size() < 3U) {
      continue;
    }
    // Every fourth triplet: the neighbours are nearly the same view.
    if (++triplets % 4 != 0) {
      continue;
    }
    ++used;
    std::vector<char> left_bytes = npyPayload(directory + "/" + files["stereo_left"]);
    std::vector<char> right_bytes = npyPayload(directory + "/" + files["stereo_right"]);
    const std::vector<char> depth_bytes =
        npyPayload(directory + "/" + files["evaluation_depth_left"]);
    const cv::Mat left(kHeight, kWidth, CV_8U, left_bytes.data());
    const cv::Mat right(kHeight, kWidth, CV_8U, right_bytes.data());
    cv::Mat disparity;
    far_matcher->compute(left, right, disparity);
    cv::Mat left_half;
    cv::Mat right_half;
    cv::resize(left, left_half, {}, 0.5, 0.5, cv::INTER_AREA);
    cv::resize(right, right_half, {}, 0.5, 0.5, cv::INTER_AREA);
    cv::Mat near_disparity;
    near_matcher->compute(left_half, right_half, near_disparity);
    const int near_threshold_16 = 16 * (kFarDisparities - 8) / 2;
    for (int row = 0; row < kHeight; ++row) {
      for (int column = 0; column < kWidth; ++column) {
        const std::int16_t near = near_disparity.at<std::int16_t>(row / 2, column / 2);
        auto& far = disparity.at<std::int16_t>(row, column);
        if (near > near_threshold_16 || far <= 0) {
          far = near > 0 ? static_cast<std::int16_t>(std::min(32767, 2 * near))
                         : std::int16_t{-16};
        }
      }
    }
    rejectMatchesWithinNoise(left, disparity);
    const auto* const depth =
        reinterpret_cast<const std::uint16_t*>(depth_bytes.data());
    // The left 256 columns have no match in the right image at the near
    // search's reach.
    for (int row = 0; row < kHeight; row += 4) {
      for (int column = 256; column < kWidth; column += 4) {
        const float truth = halfToFloat(depth[row * kWidth + column]);
        if (!(truth > 0.3F) || !std::isfinite(truth)) {
          continue;
        }
        const auto band =
            std::upper_bound(edges.begin(), edges.end(), truth) - edges.begin() - 1;
        if (band < 0 || band >= static_cast<long>(bands.size())) {
          continue;
        }
        Band& counts = bands[static_cast<std::size_t>(band)];
        ++counts.truth;
        const std::int16_t value = disparity.at<std::int16_t>(row, column);
        if (value <= 0) {
          continue;
        }
        const double disparity_px = value / 16.0;
        const double estimate_m = kFocalPx * kBaselineM / disparity_px;
        const double true_disparity_px = kFocalPx * kBaselineM / truth;
        ++counts.matched;
        const auto error = static_cast<float>(std::abs(estimate_m - truth));
        counts.within += error <= kAllowedDepthErrorM ? 1 : 0;
        counts.depth_error.push_back(error);
        counts.disparity_error.push_back(
            static_cast<float>(std::abs(disparity_px - true_disparity_px)));
      }
    }
  }
  std::printf("triplets %d focal %.1f px\n", used, kFocalPx);
  std::printf(
      "range_m      truth_px  coverage  within_0.25m  err_p50  err_p90  disp_err_p50  "
      "disp_err_p90\n");
  float worst_p90{0.0F};
  for (std::size_t band = 0U; band < bands.size(); ++band) {
    Band& counts = bands[band];
    if (counts.truth == 0) {
      continue;
    }
    const float disparity_p90 = percentile(counts.disparity_error, 0.9);
    if (edges[band] >= 2.0 && edges[band + 1U] <= 8.0) {
      worst_p90 = std::max(worst_p90, disparity_p90);
    }
    std::printf("%4.1f-%4.1f %10ld %8.2f %12.2f %9.2f %8.2f %12.2f %12.2f\n",
                edges[band], edges[band + 1U], counts.truth,
                static_cast<double>(counts.matched) / static_cast<double>(counts.truth),
                counts.matched > 0 ? static_cast<double>(counts.within) /
                                         static_cast<double>(counts.matched)
                                   : 0.0,
                static_cast<double>(percentile(counts.depth_error, 0.5)),
                static_cast<double>(percentile(counts.depth_error, 0.9)),
                static_cast<double>(percentile(counts.disparity_error, 0.5)),
                static_cast<double>(disparity_p90));
  }
  // The depth at which that disparity error is the allowed depth error.
  std::printf("CONFIDENT_DEPTH disparity_error_p90=%.2f px depth=%.2f m\n",
              static_cast<double>(worst_p90),
              worst_p90 > 0.0F
                  ? std::sqrt(kAllowedDepthErrorM * kFocalPx * kBaselineM / worst_p90)
                  : 0.0);
  return 0;
}
