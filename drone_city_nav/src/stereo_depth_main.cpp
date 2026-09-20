#include <rclcpp/rclcpp.hpp>

#include "stereo_depth_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // One thread matches pairs, the other takes the time-of-flight scans.
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 2U};
  const auto node = drone_city_nav::makeStereoDepthNode(rclcpp::NodeOptions{});
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
