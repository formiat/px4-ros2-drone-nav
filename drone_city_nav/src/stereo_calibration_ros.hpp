#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "stereo_rectifier.hpp"

namespace drone_city_nav {

// The stereo pair's calibration from a node's parameters (specification K33),
// the same names for the depth node and the camera estimator. The defaults are
// the simulated pair: 1280 x 960 over 120 degrees, no distortion, the right
// camera 0.2 m along the left one's x axis.
[[nodiscard]] inline StereoCalibration declareStereoCalibration(rclcpp::Node& node) {
  const auto values = [&node](const std::string& name,
                              const std::vector<double>& fallback,
                              const std::size_t size) {
    const std::vector<double> value =
        node.declare_parameter<std::vector<double>>(name, fallback);
    if (value.size() != size) {
      throw std::invalid_argument{name + " needs " + std::to_string(size) + " values"};
    }
    return value;
  };
  const double ideal_focal_px = 640.0 / std::tan(M_PI / 3.0);
  const std::vector<double> ideal{ideal_focal_px, ideal_focal_px, 640.0, 480.0};
  const auto camera = [&](const std::string& side) {
    const std::vector<double> intrinsics =
        values(side + "_camera_intrinsics", ideal, 4U);
    const std::vector<double> distortion =
        values(side + "_camera_distortion", {0.0, 0.0, 0.0, 0.0}, 4U);
    return PinholeCamera{
        .fx = intrinsics[0],
        .fy = intrinsics[1],
        .cx = intrinsics[2],
        .cy = intrinsics[3],
        .distortion = {distortion[0], distortion[1], distortion[2], distortion[3]}};
  };
  StereoCalibration calibration{.width = static_cast<std::size_t>(
                                    node.declare_parameter<int>("image_width", 1280)),
                                .height = static_cast<std::size_t>(
                                    node.declare_parameter<int>("image_height", 960)),
                                .left = camera("left"),
                                .right = camera("right")};
  const std::vector<double> rotation = values(
      "right_from_left_rotation", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, 9U);
  const std::vector<double> translation =
      values("right_from_left_translation_m", {-0.2, 0.0, 0.0}, 3U);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      calibration.right_from_left_rotation(row, column) =
          rotation[static_cast<std::size_t>(3 * row + column)];
    }
  }
  calibration.right_from_left_translation_m = {translation[0], translation[1],
                                               translation[2]};
  return calibration;
}

} // namespace drone_city_nav
