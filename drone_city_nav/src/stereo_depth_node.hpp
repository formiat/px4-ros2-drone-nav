#pragma once

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace drone_city_nav {

// The stereo depth node (src/stereo_depth_node.cpp). A factory, so that the
// process that hosts it chooses where its images come from: a camera driver
// over ROS, or, in simulation, an image source inside the same process.
[[nodiscard]] std::shared_ptr<rclcpp::Node>
makeStereoDepthNode(const rclcpp::NodeOptions& options);

} // namespace drone_city_nav
