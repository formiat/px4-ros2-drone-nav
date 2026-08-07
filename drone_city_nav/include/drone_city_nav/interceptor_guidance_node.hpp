#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include <memory>

namespace drone_city_nav {

[[nodiscard]] std::shared_ptr<rclcpp::Node>
makeInterceptorGuidanceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

} // namespace drone_city_nav
