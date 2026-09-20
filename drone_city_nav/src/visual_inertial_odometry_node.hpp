#pragma once

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace drone_city_nav {

// The visual-inertial odometry node (src/visual_inertial_odometry_node.cpp).
// A factory, so that the process that owns the pair's images hosts it: the
// estimator reads every frame, and two 1.2 MB images 7.5 times a second are
// handed over inside one process, not across the middleware.
[[nodiscard]] std::shared_ptr<rclcpp::Node>
makeVisualInertialOdometryNode(const rclcpp::NodeOptions& options);

// The command-line switch of the stereo processes that adds the node.
inline constexpr const char* kVisualInertialOdometrySwitch{
    "--visual-inertial-odometry"};

} // namespace drone_city_nav
