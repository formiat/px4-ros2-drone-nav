#include "stereo_frame_brightness.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace drone_city_nav {

double brightnessPercentile95(const cv::Mat& grey, const int stride) {
  std::array<std::size_t, 256U> histogram{};
  std::size_t samples{0U};
  for (int row = 0; row < grey.rows; row += stride) {
    const std::uint8_t* const values = grey.ptr<std::uint8_t>(row);
    for (int column = 0; column < grey.cols; column += stride) {
      ++histogram[values[column]];
      ++samples;
    }
  }
  std::size_t below{0U};
  for (std::size_t level = 0U; level < histogram.size(); ++level) {
    below += histogram[level];
    if (20U * below >= 19U * samples) {
      return static_cast<double>(level);
    }
  }
  return 255.0;
}

} // namespace drone_city_nav
