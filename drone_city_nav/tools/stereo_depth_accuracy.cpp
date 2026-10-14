// stereo_depth_accuracy DIR: the stereo pair's depth against the simulator's,
// on a record of pairs and the evaluation depth camera
// (tools/stereo_record_frames.py, the camera's gain applied by
// tools/stereo_gain_copy.py). The matching is the depth node's: semi-global
// matching far at full resolution and near at half, merged, and the matches
// within the image's noise rejected (stereo_depth_node.cpp). Printed by
// depth band: the share of true pixels matched, the share of matches within
// 0.25 m, and the depth and disparity errors at the 50th and 90th
// percentiles; then the confident depth, where the depth error at the 90th
// percentile is a voxel (specification K7).
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
#include <tuple>
#include <vector>

namespace {

constexpr int kWidth{1280};
constexpr int kHeight{960};
constexpr double kBaselineM{0.20};
// The pair's 120 degrees across 1280 pixels.
const double kFocalPx = (kWidth / 2.0) / std::tan(std::numbers::pi / 3.0);
constexpr double kAllowedDepthErrorM{0.25};
// The stereo profile's confident depth (specification K7).
constexpr double kConfidentDepthM{5.2};
constexpr int kFarDisparities{64};
constexpr int kNearDisparities{128};
constexpr int kBlockSize{5};
constexpr int kSpeckleWindowPx{400};
constexpr double kLoneHitVoxelM{0.25};
constexpr int kLoneHitVoxelLeastHits{2};

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

// Copies of StereoDepthNode::rejectMatchesWithinNoise and its brightnessBin.
constexpr int kSignalWindowPx{9};
constexpr std::size_t kLaplacianBins{1024U};
constexpr int kNoiseSampleStride{4};
constexpr double kSignalOverNoise{2.0};
constexpr std::size_t kBrightnessBins{16U};
constexpr float kBrightnessBinLevels{16.0F};
constexpr std::size_t kMinimumBinSamples{50U};
constexpr double kMinimumNoiseLevels{1.0};

std::size_t brightnessBin(const float mean) noexcept {
  return std::min(kBrightnessBins - 1U, static_cast<std::size_t>(std::max(0.0F, mean) /
                                                                 kBrightnessBinLevels));
}

double rejectMatchesWithinNoise(const cv::Mat& grey, cv::Mat& disparity) {
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
  // The operator's response is six noise deviations on a flat patch and
  // far more on an edge; its median is the flat patches', and the edges of
  // a textured scene do not move it the way they moved the mean (r765: the
  // mean read texture as noise and discarded two thirds of a lit frame's
  // matches).
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
                                  32 * kBlockSize * kBlockSize, 1, 31, 10,
                                  kSpeckleWindowPx, 2, cv::StereoSGBM::MODE_SGBM_3WAY);
  };
  const cv::Ptr<cv::StereoSGBM> far_matcher = matcher(kFarDisparities);
  const cv::Ptr<cv::StereoSGBM> near_matcher = matcher(kNearDisparities);
  const std::vector<double> edges{0.3, 1, 2, 4, 6, 8, 10, 14, 20, 40};
  std::vector<Band> bands(edges.size() - 1U);
  int used{0};
  long near_matches{0};
  long phantoms{0};
  long strays{0};
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
    // The node's lone-hit rule: the hits of this frame within the confident
    // depth by 0.25 m voxel of the left camera's frame, at the node's stride.
    std::map<std::tuple<int, int, int>, int> hits_by_voxel;
    const auto voxel_of = [](const int row, const int column, const double z) {
      const double x = (column - kWidth / 2.0) * z / kFocalPx;
      const double y = (row - kHeight / 2.0) * z / kFocalPx;
      return std::make_tuple(static_cast<int>(std::floor(x / kLoneHitVoxelM)),
                             static_cast<int>(std::floor(y / kLoneHitVoxelM)),
                             static_cast<int>(std::floor(z / kLoneHitVoxelM)));
    };
    for (int row = 0; row < kHeight; row += 4) {
      for (int column = 0; column < kWidth; column += 4) {
        const std::int16_t value = disparity.at<std::int16_t>(row, column);
        if (value <= 0) {
          continue;
        }
        const double z = kFocalPx * kBaselineM / (value / 16.0);
        if (z < kConfidentDepthM) {
          ++hits_by_voxel[voxel_of(row, column, z)];
        }
      }
    }
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
        if (estimate_m < kConfidentDepthM &&
            hits_by_voxel[voxel_of(row, column, estimate_m)] < kLoneHitVoxelLeastHits) {
          continue;
        }
        // A stray: a hit within the confident depth with no truth within
        // 0.5 m of it, what blocks a route through open air.
        if (estimate_m < kConfidentDepthM) {
          const int radius = static_cast<int>(std::ceil(0.5 * kFocalPx / estimate_m));
          bool truth_near{false};
          for (int r2 = std::max(0, row - radius);
               r2 <= std::min(kHeight - 1, row + radius) && !truth_near; r2 += 2) {
            for (int c2 = std::max(0, column - radius);
                 c2 <= std::min(kWidth - 1, column + radius); c2 += 2) {
              const float near_truth = halfToFloat(depth[r2 * kWidth + c2]);
              if (near_truth > 0.3F && std::isfinite(near_truth)) {
                const double lateral =
                    std::hypot(r2 - row, c2 - column) * estimate_m / kFocalPx;
                if (std::hypot(lateral, near_truth - estimate_m) <= 0.5) {
                  truth_near = true;
                  break;
                }
              }
            }
          }
          strays += truth_near ? 0 : 1;
        }
        const double true_disparity_px = kFocalPx * kBaselineM / truth;
        // A phantom: a surface placed within 4.8 m a metre or more before
        // the truth, what blocks a route through open air.
        if (estimate_m < 4.8) {
          ++near_matches;
          phantoms += estimate_m < truth - 1.0 ? 1 : 0;
        }
        ++counts.matched;
        const auto error = static_cast<float>(std::abs(estimate_m - truth));
        counts.within += error <= kAllowedDepthErrorM ? 1 : 0;
        counts.depth_error.push_back(error);
        counts.disparity_error.push_back(
            static_cast<float>(std::abs(disparity_px - true_disparity_px)));
      }
    }
  }
  std::printf(
      "triplets %d focal %.1f px phantoms %.4f of %ld matches within 4.8 m; strays "
      "%.1f a frame\n",
      used, kFocalPx,
      static_cast<double>(phantoms) / static_cast<double>(std::max(1L, near_matches)),
      near_matches,
      static_cast<double>(strays) / static_cast<double>(std::max(1, used)));
  std::printf(
      "range_m      truth_px  coverage  within_0.25m  err_p50  err_p90  disp_err_p50  "
      "disp_err_p90\n");
  // The confident depth: the depth at which the depth error at the 90th
  // percentile is the allowed one, read in the band that holds it (its
  // disparity error e gives sqrt(0.25 * f * b / e) inside the band). With a
  // flat error, as on r782, that is the farthest band's; with an error that
  // grows with the depth, the band where it crosses.
  double confident_m{0.0};
  double confident_error_px{0.0};
  for (std::size_t band = 0U; band < bands.size(); ++band) {
    Band& counts = bands[band];
    if (counts.truth == 0) {
      continue;
    }
    const float disparity_p90 = percentile(counts.disparity_error, 0.9);
    const double depth_m =
        disparity_p90 > 0.0F
            ? std::sqrt(kAllowedDepthErrorM * kFocalPx * kBaselineM / disparity_p90)
            : 0.0;
    if (depth_m >= edges[band] && depth_m <= edges[band + 1U] && confident_m == 0.0) {
      confident_m = depth_m;
      confident_error_px = disparity_p90;
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
  std::printf("CONFIDENT_DEPTH disparity_error_p90=%.2f px depth=%.2f m\n",
              confident_error_px, confident_m);
  return 0;
}
