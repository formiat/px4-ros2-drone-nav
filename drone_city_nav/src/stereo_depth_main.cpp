#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "stereo_depth_node.hpp"
#include "visual_inertial_odometry_node.hpp"

int main(int argc, char** argv) {
  const std::vector<std::string> arguments =
      rclcpp::init_and_remove_ros_arguments(argc, argv);
  const bool estimator =
      std::find(arguments.begin(), arguments.end(),
                drone_city_nav::kVisualInertialOdometrySwitch) != arguments.end();
  // One thread matches pairs, one takes the time-of-flight scans, and one
  // more tracks features when the estimator is hosted.
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{},
                                                    estimator ? 3U : 2U};
  const auto node = drone_city_nav::makeStereoDepthNode(rclcpp::NodeOptions{});
  executor.add_node(node);
  std::shared_ptr<rclcpp::Node> odometry;
  if (estimator) {
    odometry = drone_city_nav::makeVisualInertialOdometryNode(rclcpp::NodeOptions{});
    executor.add_node(odometry);
  }
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
