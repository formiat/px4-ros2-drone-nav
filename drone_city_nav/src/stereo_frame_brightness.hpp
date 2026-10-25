#pragma once

#include <opencv2/core.hpp>

namespace drone_city_nav {

// The 95th percentile of a grey frame, over the pixels at `stride` (the
// camera's gain aims it at 200 of 255; stereo_depth_node.cpp reads the light
// the frame has left off it, K15, and the void frame's raw brightness, K46).
[[nodiscard]] double brightnessPercentile95(const cv::Mat& grey, int stride);

} // namespace drone_city_nav
